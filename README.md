# Attune Firmware (XIAO Minimal)

This repository is intentionally trimmed to build one firmware target:

- **Target app:** `omi/firmware/xiao_minimal`
- **Board:** `xiao_ble/nrf52840/sense`
- **Output:** `zephyr.uf2`

## Build locally (nRF Connect SDK / west)

From repository root:

```bash
cd omi/firmware
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.7.0 v2.7.0
cd v2.7.0
west update
west zephyr-export
pip install -r zephyr/scripts/requirements.txt

west build --pristine always ../xiao_minimal -- \
  -DNCS_TOOLCHAIN_VERSION=NONE \
  -DCONF_FILE=../../xiao_minimal/prj.conf \
  -DDTC_OVERLAY_FILE=../../xiao_minimal/xiao_ble_sense.overlay
```

UF2 artifact:

```text
omi/firmware/v2.7.0/build/zephyr/zephyr.uf2
```

## CI

GitHub Actions builds `xiao_minimal` on pushes/PRs and uploads `zephyr.uf2` as an artifact.
