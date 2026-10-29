import {readFileSync} from 'node:fs'

const hardwareRoot = new URL('../../hardware/', import.meta.url)
const targets = JSON.parse(readFileSync(new URL('targets.json', hardwareRoot), 'utf8'))

export const DEV_PROFILES = [
    {id: 'rp1', label: 'RadioMaster RP1 — RX 2.4 GHz', group: 'rx_2400', target: 'rp1'},
    {id: 'ranger-nano', label: 'RadioMaster Ranger Nano — TX 2.4 GHz', group: 'tx_2400', target: 'ranger-nano'},
    {id: 'generic', label: 'Simulation générique — RX ESP32 / LR1121'}
]

// Hardware comes from the same target definitions as the firmware.
// Runtime options remain examples for UI development, not factory defaults.
export function getDevProfile(id) {
    const definition = DEV_PROFILES.find(profile => profile.id === id)
    if (!definition) throw new Error('Unknown development profile: ' + id)
    const isGeneric = id === 'generic'
    const isTx = id === 'ranger-nano'
    const env = {
        VITE_FEATURE_IS_TX: String(isTx),
        VITE_FEATURE_IS_8285: String(id === 'rp1'),
        VITE_FEATURE_HAS_SX128X: String(!isGeneric),
        VITE_FEATURE_HAS_SX127X: 'false',
        VITE_FEATURE_HAS_LR1121: String(isGeneric),
        VITE_FEATURE_HAS_LR2021: 'false',
        VITE_FEATURE_HAS_SUBGHZ: String(isGeneric)
    }
    if (isGeneric) return {...definition, env}
    const target = targets.radiomaster[definition.group][definition.target]
    const type = isTx ? 'TX' : 'RX'
    const layout = JSON.parse(readFileSync(new URL(type + '/' + target.layout_file, hardwareRoot), 'utf8'))
    return {
        ...definition,
        env,
        hardware: {...layout, ...target.overlay},
        settings: {
            product_name: target.product_name,
            lua_name: target.lua_name,
            target: target.firmware,
            'module-type': type,
            'radio-type': 'SX128X',
            has_low_band: false,
            has_high_band: true,
            has_gps: false,
            custom_hardware: false,
            ssid: 'ExpressLRS ' + type
        },
        config: isTx
            ? {uid: [1, 2, 3, 4, 5, 6], 'button-actions': []}
            : {
                uid: [1, 2, 3, 4, 5, 6],
                modelid: 255,
                'force-tlm': false,
                'serial-protocol': 0,
                'sbus-failsafe': 0,
                vbind: 0
            }
    }
}
