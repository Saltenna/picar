#!/usr/bin/env bash
set -euo pipefail

# PiCar installer for Raspberry Pi OS (Trixie)
# - Installs OS packages
# - Clones / updates /opt/picar
# - Installs Node deps
# - (Optional) Installs MAVProxy in /opt/venvs/mavproxy
# - Installs and enables systemd services

LOG_PREFIX="[picar-install]"
say() { echo -e "${LOG_PREFIX} $*"; }
die() { echo -e "${LOG_PREFIX} ERROR: $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1; }

if [[ "${EUID}" -ne 0 ]]; then
  die "Please run as root (e.g. sudo ./install.sh)"
fi

SUDO_USER_NAME="${SUDO_USER:-}"
DEFAULT_RUN_USER="${SUDO_USER_NAME:-saltenna}"

PI_MODEL_RAW="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)"
PI_MODEL="${PI_MODEL_RAW:-unknown}"
PI_GEN="unknown"
if echo "$PI_MODEL" | grep -qi "Raspberry Pi 5"; then PI_GEN="5"; fi
if echo "$PI_MODEL" | grep -qiE "Raspberry Pi 4|Raspberry Pi 400|Compute Module 4"; then PI_GEN="4"; fi

say "Detected model: ${PI_MODEL} (Pi ${PI_GEN})"

prompt_yes_no() {
  local prompt="$1" default="$2" ans
  while true; do
    read -r -p "${prompt} [y/n] (default: ${default}): " ans || true
    ans="${ans:-$default}"
    case "$ans" in
      y|Y|yes|YES) return 0;;
      n|N|no|NO) return 1;;
      *) echo "Please enter y or n.";;
    esac
  done
}

prompt_choice() {
  local prompt="$1"; shift
  local default="$1"; shift
  local choices=("$@")
  local ans
  echo "$prompt"
  local i=1
  for c in "${choices[@]}"; do
    echo "  ${i}) ${c}"
    i=$((i+1))
  done
  while true; do
    read -r -p "Choose 1-${#choices[@]} (default: ${default}): " ans || true
    ans="${ans:-$default}"
    if [[ "$ans" =~ ^[0-9]+$ ]] && (( ans>=1 && ans<=${#choices[@]} )); then
      echo "${choices[$((ans-1))]}"
      return 0
    fi
    echo "Invalid choice."
  done
}

RUN_USER="$( (read -r -p "Which Linux user should run the services? (default: ${DEFAULT_RUN_USER}): " u && echo "${u:-$DEFAULT_RUN_USER}") || echo "$DEFAULT_RUN_USER" )"
say "Services will run as user: ${RUN_USER}"

if ! id -u "${RUN_USER}" >/dev/null 2>&1; then
  say "User '${RUN_USER}' does not exist."
  if prompt_yes_no "Create user '${RUN_USER}' now?" "y"; then
    useradd -m -s /bin/bash "${RUN_USER}"
    say "Created user '${RUN_USER}'."
  else
    die "User '${RUN_USER}' is required to run services."
  fi
fi

# Groups that are commonly needed on Pi for GPIO/camera/serial
for grp in video gpio dialout i2c spi; do
  if getent group "$grp" >/dev/null 2>&1; then
    usermod -aG "$grp" "${RUN_USER}" || true
  fi
done

say "Updating APT and installing base packages..."
apt-get update -y
apt-get install -y git nodejs npm ffmpeg python3 python3-venv python3-pip ca-certificates

# Camera choice (placeholder for future wiring into app/config)
CAMERA_TYPE="$(prompt_choice "Camera type?" "1" "pi-cam (libcamera/rpicam)" "usb webcam (/dev/video0)")"
say "Selected camera: ${CAMERA_TYPE}"
if [[ "${CAMERA_TYPE}" == "usb webcam (/dev/video0)" ]]; then
  # Helpful packages for USB/V4L2 workflows
  apt-get install -y v4l-utils || true
fi

USE_MAVPROXY="no"
if prompt_yes_no "Will you use MAVProxy (Pixhawk/ArduPilot PWM via MAVLink)?" "y"; then
  USE_MAVPROXY="yes"
fi
say "MAVProxy: ${USE_MAVPROXY}"
MAVPROXY_MASTER="/dev/ttyACM0"
MAVPROXY_BAUD="115200"
if [[ "${USE_MAVPROXY}" == "yes" ]]; then
  read -r -p "MAVProxy master device? (default: ${MAVPROXY_MASTER}): " _m || true
  MAVPROXY_MASTER="${_m:-$MAVPROXY_MASTER}"
  read -r -p "MAVProxy baudrate? (default: ${MAVPROXY_BAUD}): " _b || true
  MAVPROXY_BAUD="${_b:-$MAVPROXY_BAUD}"
fi


PWM_METHOD=""

if [[ "${USE_MAVPROXY}" == "yes" ]]; then
  PWM_METHOD="mavproxy"
else
  if [[ "${PI_GEN}" == "4" ]]; then
    # user requested pigpion specifically for Pi 4
    PWM_METHOD="pigpion"
  elif [[ "${PI_GEN}" == "5" ]]; then
    PWM_METHOD="libgpiod"
  else
    # safe-ish default
    PWM_METHOD="libgpiod"
  fi
fi
say "PWM method will be: ${PWM_METHOD}"

# Conditional packages for PWM method
if [[ "${PWM_METHOD}" == "libgpiod" ]]; then
  apt-get install -y gpiod || true
fi
if [[ "${PWM_METHOD}" == "pigpion" || "${PWM_METHOD}" == "pigpiod" ]]; then
  apt-get install -y pigpio || true
fi

# /opt layout
say "Ensuring /opt layout..."
mkdir -p /opt /opt/venvs
chmod 0755 /opt /opt/venvs

# Clone/update repo
REPO_DIR="/opt/picar"
REPO_SSH="git@github.com:Saltenna/picar.git"
REPO_HTTPS="https://github.com/Saltenna/picar.git"

if [[ -d "${REPO_DIR}/.git" ]]; then
  say "Updating existing repo at ${REPO_DIR}..."
  git -C "${REPO_DIR}" fetch --all --prune
  git -C "${REPO_DIR}" pull --ff-only || true
else
  say "Cloning repo into ${REPO_DIR}..."
  if [[ -d "${REPO_DIR}" && ! -e "${REPO_DIR}/.git" ]]; then
    die "${REPO_DIR} exists but is not a git repo. Move it aside and re-run."
  fi
  # Try SSH first; fall back to HTTPS if SSH fails (missing key, etc.)
  if git clone "${REPO_SSH}" "${REPO_DIR}" 2>/dev/null; then
    :
  else
    say "SSH clone failed; trying HTTPS clone..."
    git clone "${REPO_HTTPS}" "${REPO_DIR}"
  fi
fi

# Node deps
say "Installing Node dependencies..."
cd "${REPO_DIR}"
if [[ -f package-lock.json ]]; then
  npm ci --omit=dev || npm install --omit=dev
else
  npm install --omit=dev
fi

# Ensure ownership for runtime user
chown -R "${RUN_USER}:${RUN_USER}" "${REPO_DIR}" || true

# Configure picar-cfg.json
CFG="${REPO_DIR}/picar-cfg.json"
if [[ -f "${CFG}" ]]; then
  say "Updating ${CFG} (pwm_method + placeholders)..."
  python3 - <<PY
import json, pathlib
p = pathlib.Path("${CFG}")
cfg = json.loads(p.read_text())
cfg["pwm_method"] = "${PWM_METHOD}"
cfg["camera_type"] = "${CAMERA_TYPE}"
cfg["use_mavproxy"] = ("${USE_MAVPROXY}" == "yes")
p.write_text(json.dumps(cfg, indent=2) + "\n")
print("Wrote", p)
PY
else
  say "WARNING: ${CFG} not found; skipping config update."
fi

# MAVProxy install
if [[ "${USE_MAVPROXY}" == "yes" ]]; then
  say "Installing MAVProxy into /opt/venvs/mavproxy ..."
  apt-get install -y python3-dev build-essential || true
  mkdir -p /opt/venvs /var/log/mavproxy
  chown -R "${RUN_USER}:${RUN_USER}" /var/log/mavproxy || true

  python3 -m venv /opt/venvs/mavproxy
  /opt/venvs/mavproxy/bin/pip install --upgrade pip wheel
  /opt/venvs/mavproxy/bin/pip install --upgrade MAVProxy pyserial
fi

# systemd install with templating of User=
say "Installing systemd units..."
UNIT_SRC_DIR="${REPO_DIR}/systemd"
UNIT_DST_DIR="/lib/systemd/system"
mkdir -p "${UNIT_DST_DIR}"

install_unit() {
  local src="$1"
  local dst="${UNIT_DST_DIR}/$(basename "$src")"
  local unit_name="$(basename "$src")"
  # Replace the User= line; if none exists, add one under [Service]
  if grep -qE '^User=' "$src"; then
    sed -E "s/^User=.*/User=${RUN_USER}/" "$src" > "$dst"
  else
    awk -v u="${RUN_USER}" '
      /^\[Service\]$/ {print; print "User="u; next}
      {print}
    ' "$src" > "$dst"
  fi
  if [[ "$unit_name" == "mavproxy.service" ]]; then
    # Update master/baudrate flags if present
    sed -i -E "s/--master=([^ \\]+|\\S+)/--master=${MAVPROXY_MASTER//\//\\/}/" "$dst" || true
    sed -i -E "s/--baudrate[= ]+[0-9]+/--baudrate ${MAVPROXY_BAUD}/" "$dst" || true
    sed -i -E "s#/dev/ttyACM0#${MAVPROXY_MASTER}#g" "$dst" || true
    sed -i -E "s/--baudrate[ ]+115200/--baudrate ${MAVPROXY_BAUD}/g" "$dst" || true
  fi
  chmod 0644 "$dst"
  say "Installed $(basename "$dst")"
}

if [[ -d "${UNIT_SRC_DIR}" ]]; then
  for f in "${UNIT_SRC_DIR}"/*.service; do
    [[ -e "$f" ]] || continue
    install_unit "$f"
  done
else
  die "Missing ${UNIT_SRC_DIR} (expected systemd units)."
fi

systemctl daemon-reload

# Enable/disable services
say "Enabling picar.service..."
systemctl enable --now picar.service

if [[ "${USE_MAVPROXY}" == "yes" ]]; then
  say "Enabling mavproxy.service..."
  systemctl enable --now mavproxy.service
else
  if systemctl list-unit-files | grep -q '^mavproxy\.service'; then
    say "Disabling mavproxy.service..."
    systemctl disable --now mavproxy.service || true
  fi
fi

say "Done."
say "If you changed user group memberships, you may need to log out/in for them to take effect."
say "Check status: systemctl status picar.service"
