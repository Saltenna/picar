# PiCar helper targets
# Usage:
#   sudo make install
#   sudo make uninstall
#   make status

.PHONY: install uninstall status logs restart

install:
\tchmod +x ./install.sh
\tsudo ./install.sh

uninstall:
\tsudo systemctl disable --now picar.service || true
\tsudo systemctl disable --now mavproxy.service || true
\tsudo rm -f /lib/systemd/system/picar.service /lib/systemd/system/mavproxy.service
\tsudo systemctl daemon-reload
\t@echo "Systemd units removed. Repo left at /opt/picar (remove manually if desired)."

status:
\tsystemctl status picar.service --no-pager || true
\tsystemctl status mavproxy.service --no-pager || true

logs:
\tjournalctl -u picar.service -n 200 --no-pager || true

restart:
\tsudo systemctl restart picar.service
