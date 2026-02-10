# Attune Firmware (XIAO Minimal)

This repository is intentionally trimmed to build one firmware target:

- **Target app:** `omi/firmware/xiao_minimal`
- **Board:** `xiao_ble/nrf52840/sense`
- **Output:** `zephyr.uf2`

## Local build environment

This repo includes a reproducible local setup for:

- nRF Connect SDK **v2.7.0**
- Zephyr SDK **0.16.5**
- `west`, `cmake`, `ninja`, and Python dependencies

---

## Option 1: Direct host install (fast iteration)

From repo root:

```bash
./scripts/setup_ncs_zephyr.sh
source ./scripts/ncs-env.sh
./scripts/build_xiao_minimal.sh
```

Equivalent manual build command (after `source ./scripts/ncs-env.sh`):

```bash
cd omi/firmware/xiao_minimal
west build -b xiao_ble/nrf52840/sense -p always .
```

### Install locations

- NCS workspace: `~/ncs/v2.7.0`
- Zephyr SDK: `~/zephyr-sdk/zephyr-sdk-0.16.5`

You can override defaults with env vars when running setup:

```bash
NCS_DIR=~/ncs ZEPHYR_SDK_DIR=~/zephyr-sdk ./scripts/setup_ncs_zephyr.sh
```

---

## Option 2: Docker build (clean/reproducible)

> Requires Docker on host.

Build image:

```bash
docker build -t attune-build .
```

Run build against your checkout:

```bash
docker run --rm -v "$PWD":/workspace attune-build
```

The Docker default command runs:

```bash
cd /workspace/omi/firmware/xiao_minimal
west build -b xiao_ble/nrf52840/sense -p always .
```

---

## CI

GitHub Actions builds `xiao_minimal` on pushes/PRs and uploads `zephyr.uf2` as an artifact.
