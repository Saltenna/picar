#!/usr/bin/env bash
set -euo pipefail
echo "[picar-uninstall] Disabling services..."
sudo systemctl disable --now picar.service || true
sudo systemctl disable --now mavproxy.service || true
sudo systemctl disable --now mediamtx.service || true
echo "[picar-uninstall] Removing unit files..."
sudo rm -f /lib/systemd/system/picar.service /lib/systemd/system/mavproxy.service /lib/systemd/system/mediamtx.service
sudo systemctl daemon-reload
echo "[picar-uninstall] Done. Repo still at /opt/picar (remove manually if desired)."
