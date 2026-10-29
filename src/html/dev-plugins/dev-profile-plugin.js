import {DEV_PROFILES} from './dev-profiles.js'

// Switching also restarts feature transforms: changing mock JSON alone cannot
// turn RX-only source into TX source. Selection lives for this server session.
export function devProfilePlugin(profile) {
    let switching = false
    return {
        name: 'dev-device-selector',
        apply: 'serve',
        transformIndexHtml() {
            return [{
                tag: 'script',
                attrs: {type: 'module', src: '/dev-plugins/dev-selector.js'},
                injectTo: 'body'
            }]
        },
        configureServer(server) {
            server.middlewares.use(async (req, res, next) => {
                if (req.url !== '/__dev/profile') return next()
                res.setHeader('Content-Type', 'application/json')
                res.setHeader('Cache-Control', 'no-store')
                const send = (code, data) => {
                    res.statusCode = code
                    res.end(JSON.stringify(data))
                }
                if (req.method === 'GET') {
                    return send(200, {
                        selected: profile.id,
                        switching,
                        profiles: DEV_PROFILES.map(({id, label}) => ({id, label}))
                    })
                }
                if (req.method !== 'POST') return send(405, {error: 'Method not allowed'})
                if (switching) return send(409, {error: 'A profile change is already in progress'})
                try {
                    let body = ''
                    for await (const chunk of req) {
                        body += chunk
                        if (body.length > 1024) return send(413, {error: 'Request too large'})
                    }
                    const {id} = JSON.parse(body)
                    if (!DEV_PROFILES.some(profile => profile.id === id)) {
                        return send(400, {error: 'Unknown profile'})
                    }
                    if (id === profile.id) return send(200, {selected: id})
                    server.ws.send({type: 'custom', event: 'dev:profile-changing', data: {id}})
                    switching = true
                    process.env.ELRS_DEV_PROFILE = id
                    // Finish the request before restarting its HTTP server.
                    res.once('finish', () => {
                        setTimeout(async () => {
                            try {
                                await server.restart()
                                server.ws.send({type: 'full-reload', path: '*'})
                            } catch (error) {
                                switching = false
                                server.config.logger.error('Profile restart failed: ' + error.message)
                            }
                        }, 100)
                    })
                    return send(200, {selected: id})
                } catch {
                    return send(400, {error: 'Invalid profile request'})
                }
            })
        }
    }
}
