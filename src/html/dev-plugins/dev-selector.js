// Injected only by the mock development server, never imported by the app.
const host = document.createElement('aside')
host.id = 'elrs-dev-selector'
const root = host.attachShadow({mode: 'open'})
root.innerHTML = `
    <style>
        :host { position: fixed; right: 12px; bottom: 12px; z-index: 2000;
            max-width: calc(100vw - 24px); font: 14px system-ui, sans-serif; }
        details { background: #17212e; color: #fff; border: 1px solid #63758a;
            border-radius: 8px; padding: 10px 12px; box-shadow: 0 3px 14px #0004; }
        summary { cursor: pointer; font-weight: 600; }
        label { display: block; margin: 12px 0 6px; }
        select { width: 100%; padding: 7px; background: #fff; color: #17212e; }
        p { max-width: 340px; margin: 8px 0 0; font-size: 12px; }
    </style>
    <details open>
        <summary>DEV · Appareil simulé</summary>
        <label for="device">Profil</label>
        <select id="device" disabled></select>
        <p>Changer de profil réinitialise les réglages simulés dans tous les onglets de ce serveur.</p>
        <p id="status" role="status">Chargement…</p>
    </details>
`
document.body.append(host)
const select = root.querySelector('select')
const status = root.querySelector('#status')
let current
// Reset the route in every tab before Vite reloads a different RX/TX interface.
import.meta.hot?.on('dev:profile-changing', () => {
    history.replaceState(null, '', '#info')
})
async function loadProfiles() {
    const response = await fetch('/__dev/profile', {cache: 'no-store'})
    if (!response.ok) throw new Error('Impossible de charger les profils.')
    const data = await response.json()
    current = data.selected
    select.replaceChildren(...data.profiles.map(({id, label}) => new Option(label, id)))
    select.value = current
    select.disabled = false
    status.textContent = 'Simulation locale · aucun appareil connecté'
}
select.addEventListener('change', async () => {
    const id = select.value
    select.disabled = true
    status.textContent = 'Changement de profil…'
    try {
        const response = await fetch('/__dev/profile', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({id})
        })
        if (!response.ok) throw new Error('Le changement de profil a échoué.')
        // Vite normally reloads on restart. Poll as a fallback, including when
        // the websocket is unavailable, without reloading the old profile.
        for (let attempt = 0; attempt < 60; attempt++) {
            await new Promise(resolve => setTimeout(resolve, 500))
            try {
                const check = await fetch('/__dev/profile', {
                    cache: 'no-store', signal: AbortSignal.timeout(1500)
                })
                const data = await check.json()
                if (data.selected === id && !data.switching) {
                    history.replaceState(null, '', '#info')
                    location.reload()
                    return
                }
            } catch { /* Server temporarily unavailable during restart. */ }
        }
        throw new Error('Le serveur met trop de temps à redémarrer. Recharge la page.')
    } catch (error) {
        status.textContent = error.message
        select.value = current
        select.disabled = false
    }
})
loadProfiles().catch(error => { status.textContent = error.message })
