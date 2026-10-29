# Dev Plugins

These plugins are only for `vite serve`. They exist to make local development
and device testing practical without changing the production bundle.

Use a dev plugin when the job depends on:
- local middleware
- mock endpoints
- proxying requests to real hardware during development

Do not put production build transforms here. Those belong in
[`../build-plugins`](../build-plugins/README.md).

## Plugins

### `dev-mock-plugin.js`

Purpose:
- serves mock ELRS config, hardware, and voltage-sampling endpoints during
  local development

Use when:
- you want to exercise the UI without real hardware
- you need stable mock responses for layout and flow work

Do not use when:
- validating device-specific behavior against a real target
- testing networking, timing, or firmware integration details that depend on
  the real hardware stack

### `dev-proxy-plugin.js`

Purpose:
- proxies non-Vite requests from the local dev server to a real device

Use when:
- you want the local UI code but real device responses
- testing against hardware is more important than deterministic mocks

Do not use when:
- you need offline or repeatable local-only testing
- you want to inspect or change mock API behavior inside the repo

## Device profiles

Run `npm run dev` and use **DEV · Appareil simulé** in the bottom-right corner.
The default is RP1 (or Ranger Nano if VITE_FEATURE_IS_TX=true).
Three profiles are available: RadioMaster RP1, RadioMaster Ranger Nano, and a
generic ESP32 / LR1121 RX simulation with PWM, GPS and voltage samples.

The real-device profiles load their layouts and overlays from
`../../hardware/targets.json`. Runtime options are simulated examples from
`dev-mock-plugin.js`, not factory defaults or settings read from a device.
Edit `dev-profiles.js` to adjust profile-specific config.

Switching profiles restarts Vite automatically, rebuilds RX/TX feature blocks,
and reloads the browser. Mock edits are reset; the profile is shared by all tabs
on this server and kept until the process stops. Source edits still reload as usual.
You can collapse the selector using its title.

Use `npm run dev:rx`, `npm run dev:tx`, or `npm run dev:rx-esp32` to choose a
starting profile (Windows supported via cross-env). Alternatively set
`ELRS_DEV_PROFILE=rp1|ranger-nano|generic`.
Mock profiles override VITE_FEATURE_* flags so UI and simulated device agree.
With VITE_ELRS_PROXY_TARGET, profiles and the selector are disabled; configure
VITE_FEATURE_* for your real hardware instead. Production builds also ignore
ELRS_DEV_PROFILE and never include the selector.
