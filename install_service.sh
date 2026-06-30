#!/usr/bin/bash
# install_service.sh - 安装 / 卸载 smart_rc_car systemd 服务
# 用法: ./install_service.sh --install | --uninstall
#
# 上位机 Host       : NVIDIA Jetson Orin Nano
# 下位机 Controller : 鲸鱼 MC602 主控

set -e

SERVICE_NAME="rc-car"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

install_service() {
    echo "[INSTALL] 安装 ${SERVICE_NAME} 服务..."

    # 写入 systemd unit（ExecStartPre 等待推理服务就绪）
    sudo tee "${SERVICE_FILE}" > /dev/null <<EOF
[Unit]
Description=Smart RC Car Control
After=network.target jetson-infer-server.service
Requires=jetson-infer-server.service

[Service]
Type=simple
User=jetson
WorkingDirectory=${PROJECT_DIR}
ExecStartPre=/usr/bin/bash -c ' \
    echo "[rc-car] 等待推理服务 https://127.0.0.1:8808 ..."; \
    for i in \$(seq 1 30); do \
        curl -sk --connect-timeout 2 https://127.0.0.1:8808 > /dev/null 2>&1 && break; \
        sleep 2; \
    done'
ExecStart=/usr/bin/python3 ${PROJECT_DIR}/rc_car.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable "${SERVICE_NAME}.service"
    sudo systemctl start "${SERVICE_NAME}.service"

    echo "[INSTALL] 完成。状态:"
    systemctl status "${SERVICE_NAME}.service" --no-pager -l | head -8
}

uninstall_service() {
    echo "[UNINSTALL] 卸载 ${SERVICE_NAME} 服务..."

    sudo systemctl stop "${SERVICE_NAME}.service"  2>/dev/null || true
    sudo systemctl disable "${SERVICE_NAME}.service" 2>/dev/null || true
    sudo rm -f "${SERVICE_FILE}"
    sudo systemctl daemon-reload

    echo "[UNINSTALL] 完成"
}

case "${1}" in
    --install)   install_service ;;
    --uninstall) uninstall_service ;;
    *)
        echo "用法: $0 --install | --uninstall"
        exit 1
        ;;
esac
