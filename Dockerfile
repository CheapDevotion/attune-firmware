FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG NCS_VERSION=v2.7.0
ARG ZEPHYR_SDK_VERSION=0.16.5

ENV NCS_VERSION=${NCS_VERSION}
ENV ZEPHYR_SDK_VERSION=${ZEPHYR_SDK_VERSION}
ENV NCS_DIR=/opt/ncs
ENV ZEPHYR_SDK_DIR=/opt/zephyr-sdk

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget xz-utils file git cmake ninja-build gperf ccache \
    dfu-util device-tree-compiler python3 python3-pip python3-venv \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip \
  && python3 -m pip install --no-cache-dir west ninja

WORKDIR /workspace
COPY scripts/setup_ncs_zephyr.sh /usr/local/bin/setup_ncs_zephyr.sh
COPY scripts/build_xiao_minimal.sh /usr/local/bin/build_xiao_minimal.sh
RUN chmod +x /usr/local/bin/setup_ncs_zephyr.sh /usr/local/bin/build_xiao_minimal.sh

RUN NCS_DIR=${NCS_DIR} ZEPHYR_SDK_DIR=${ZEPHYR_SDK_DIR} /usr/local/bin/setup_ncs_zephyr.sh --skip-deps

ENV WEST_TOPDIR=${NCS_DIR}/${NCS_VERSION}
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=${ZEPHYR_SDK_DIR}/zephyr-sdk-${ZEPHYR_SDK_VERSION}

CMD ["bash", "-lc", "cd /workspace/omi/firmware/xiao_minimal && west build -b xiao_ble/nrf52840/sense -p always ."]
