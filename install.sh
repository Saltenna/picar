#!/usr/bin/env bash
set -euo pipefail

# PiCar installer for Raspberry Pi OS (Trixie)
# Usage: sudo ./install.sh [--picar | --fleet]
#   --picar  Install rover software (picar + MAVProxy + optional MediaMTX)
#   --fleet  Install Fleet Manager server only
#   (no flag) prompts for choice

LOG_PREFIX="[picar-install]"
say() { echo -e "${LOG_PREFIX} $*"; }
die() { echo -e "${LOG_PREFIX} ERROR: $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1; }

if [[ "${EUID}" -ne 0 ]]; then
  die "Please run as root (e.g. sudo ./install.sh)"
fi

# ── Install mode ──────────────────────────────────────────────────────────────
INSTALL_MODE=""
for arg in "$@"; do
  case "$arg" in
    --picar) INSTALL_MODE="picar" ;;
    --fleet) INSTALL_MODE="fleet" ;;
    *) die "Unknown argument: $arg (use --picar or --fleet)" ;;
  esac
done

if [[ -z "$INSTALL_MODE" ]]; then
  echo ""
  echo "What would you like to install?"
  echo "  1) picar  — rover software (picar + MAVProxy + optional MediaMTX)"
  echo "  2) fleet  — Fleet Manager server (dashboard for multiple rovers)"
  while true; do
    read -r -p "Choose 1 or 2: " _mode || true
    case "$_mode" in
      1) INSTALL_MODE="picar"; break ;;
      2) INSTALL_MODE="fleet"; break ;;
      *) echo "Please enter 1 or 2." ;;
    esac
  done
fi
say "Install mode: ${INSTALL_MODE}"

# ── Paths ─────────────────────────────────────────────────────────────────────
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_BASE="/opt/venvs"
say "Picar directory: ${REPO_DIR}"
say "Venv base: ${VENV_BASE}"

# ── Shared helpers ───────────────────────────────────────────────────────────
UNIT_DST_DIR="/lib/systemd/system"

install_unit() {
  local src="$1"
  local dst="${UNIT_DST_DIR}/$(basename "$src")"
  local unit_name="$(basename "$src")"
  mkdir -p "${UNIT_DST_DIR}"
  if grep -qE '^User=' "$src"; then
    sed -E "s/^User=.*/User=${RUN_USER}/" "$src" > "$dst"
  else
    awk -v u="${RUN_USER}" '
      /^\[Service\]$/ {print; print "User="u; next}
      {print}
    ' "$src" > "$dst"
  fi
  if [[ "$unit_name" == "mavproxy.service" ]]; then
    sed -i -E "s/--master=([^ \\]+|\\S+)/--master=${MAVPROXY_MASTER//\//\\/}/" "$dst" || true
    sed -i -E "s/--baudrate[= ]+[0-9]+/--baudrate ${MAVPROXY_BAUD}/" "$dst" || true
    sed -i -E "s#/dev/ttyACM0#${MAVPROXY_MASTER}#g" "$dst" || true
    sed -i -E "s/--baudrate[ ]+115200/--baudrate ${MAVPROXY_BAUD}/g" "$dst" || true
  fi
  chmod 0644 "$dst"
  say "Installed $(basename "$dst")"
}

# ── Shared: run user ─────────────────────────────────────────────────────────
SUDO_USER_NAME="${SUDO_USER:-}"
DEFAULT_RUN_USER="${SUDO_USER_NAME:-saltenna}"
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

# ── Fleet Manager install ─────────────────────────────────────────────────────
if [[ "$INSTALL_MODE" == "fleet" ]]; then
  say "Installing Fleet Manager..."
  apt-get update -y
  apt-get install -y git nodejs npm

  read -r -p "Install picar repo to? (default: /opt/picar): " _fdir || true
  REPO_DIR="${_fdir:-/opt/picar}"
  REPO_SSH="git@github.com:Saltenna/picar.git"
  REPO_HTTPS="https://github.com/Saltenna/picar.git"

  if [[ -d "${REPO_DIR}/.git" ]]; then
    say "Updating existing repo at ${REPO_DIR}..."
    git -C "${REPO_DIR}" fetch --all --prune
    git -C "${REPO_DIR}" pull --ff-only || true
  else
    if git clone "${REPO_SSH}" "${REPO_DIR}" 2>/dev/null; then :
    else
      say "SSH clone failed; trying HTTPS..."
      git clone "${REPO_HTTPS}" "${REPO_DIR}"
    fi
  fi

  chown -R "${RUN_USER}:${RUN_USER}" "${REPO_DIR}" || true

  install_unit "${REPO_DIR}/systemd/fleet-manager.service"
  systemctl daemon-reload
  systemctl enable --now fleet-manager.service
  say "Fleet Manager installed and running on port 3000."
  say "Dashboard: http://$(hostname -I | awk '{print $1}'):3000"
  say "Done."
  exit 0
fi

# ── Picar rover install ───────────────────────────────────────────────────────
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
  echo "$prompt" >&2
  local i=1
  for c in "${choices[@]}"; do
    echo "  ${i}) ${c}" >&2
    i=$((i+1))
  done
  while true; do
    read -r -p "Choose 1-${#choices[@]} (default: ${default}): " ans || true
    ans="${ans:-$default}"
    if [[ "$ans" =~ ^[0-9]+$ ]] && (( ans>=1 && ans<=${#choices[@]} )); then
      echo "${choices[$((ans-1))]}"
      return 0
    fi
    echo "Invalid choice." >&2
  done
}

# Groups that are commonly needed on Pi for GPIO/camera/serial
for grp in video gpio dialout i2c spi; do
  if getent group "$grp" >/dev/null 2>&1; then
    usermod -aG "$grp" "${RUN_USER}" || true
  fi
done

say "Updating APT and installing base packages..."
apt-get update -y
apt-get install -y git nodejs npm ffmpeg python3 python3-venv python3-pip ca-certificates curl tar

# Camera choice (placeholder for future wiring into app/config)
CAMERA_TYPE="$(prompt_choice "Camera type?" "1" "pi-cam (libcamera/rpicam)" "usb webcam (/dev/video0)")"
say "Selected camera: ${CAMERA_TYPE}"
if [[ "${CAMERA_TYPE}" == "usb webcam (/dev/video0)" ]]; then
  # Helpful packages for USB/V4L2 workflows
  apt-get install -y v4l-utils || true
fi

USE_WEBRTC="no"
if [[ "${CAMERA_TYPE}" == "pi-cam (libcamera/rpicam)" ]] && prompt_yes_no "Install MediaMTX WebRTC video service?" "y"; then
  USE_WEBRTC="yes"
fi
say "WebRTC video: ${USE_WEBRTC}"

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
say "Ensuring ${VENV_BASE} exists..."
mkdir -p "${VENV_BASE}"
chmod 0755 "${VENV_BASE}"

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
  python3 -c "
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
cfg = json.loads(p.read_text())
cfg['pwm_method'] = sys.argv[2]
cfg['camera_type'] = sys.argv[3]
cfg['use_mavproxy'] = (sys.argv[4] == 'yes')
cfg['stream_codec'] = 'webrtc' if sys.argv[5] == 'yes' else 'h264'
cfg.setdefault('webrtc_protocol', 'https')
cfg.setdefault('webrtc_port', 8889)
cfg.setdefault('webrtc_path', 'cam')
p.write_text(json.dumps(cfg, indent=2) + '\n')
print('Wrote', p)
" "${CFG}" "${PWM_METHOD}" "${CAMERA_TYPE}" "${USE_MAVPROXY}" "${USE_WEBRTC}"
else
  say "WARNING: ${CFG} not found; skipping config update."
fi

# MAVProxy install
if [[ "${USE_MAVPROXY}" == "yes" ]]; then
  say "Installing MAVProxy into ${VENV_BASE}/mavproxy ..."
  apt-get install -y python3-dev build-essential || true
  mkdir -p "${VENV_BASE}" /var/log/mavproxy
  chown -R "${RUN_USER}:${RUN_USER}" /var/log/mavproxy || true

  python3 -m venv "${VENV_BASE}/mavproxy"
  "${VENV_BASE}/mavproxy/bin/pip" install --upgrade pip wheel
  "${VENV_BASE}/mavproxy/bin/pip" install --upgrade MAVProxy pyserial future
fi

# MediaMTX install for WebRTC video
if [[ "${USE_WEBRTC}" == "yes" ]]; then
  MEDIAMTX_VERSION="${MEDIAMTX_VERSION:-v1.17.1}"
  if [[ "${MEDIAMTX_VERSION}" != v* ]]; then
    MEDIAMTX_VERSION="v${MEDIAMTX_VERSION}"
  fi

  case "$(uname -m)" in
    aarch64|arm64) MEDIAMTX_ARCH="arm64" ;;
    armv7l|armhf)  MEDIAMTX_ARCH="armv7" ;;
    x86_64|amd64)  MEDIAMTX_ARCH="amd64" ;;
    *) die "Unsupported MediaMTX architecture: $(uname -m)" ;;
  esac

  say "Installing MediaMTX ${MEDIAMTX_VERSION} (${MEDIAMTX_ARCH}) into ${REPO_DIR}/bin ..."
  TMP_DIR="$(mktemp -d)"
  trap 'rm -rf "${TMP_DIR}"' EXIT
  MTX_URL="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_linux_${MEDIAMTX_ARCH}.tar.gz"
  curl -fL "${MTX_URL}" -o "${TMP_DIR}/mediamtx.tar.gz"
  tar -xzf "${TMP_DIR}/mediamtx.tar.gz" -C "${TMP_DIR}"
  mkdir -p "${REPO_DIR}/bin"
  install -m 0755 "${TMP_DIR}/mediamtx" "${REPO_DIR}/bin/mediamtx"
  chown -R "${RUN_USER}:${RUN_USER}" "${REPO_DIR}/bin" || true
fi

# systemd install with templating of User=
say "Installing systemd units..."
UNIT_SRC_DIR="${REPO_DIR}/systemd"

if [[ -d "${UNIT_SRC_DIR}" ]]; then
  for f in "${UNIT_SRC_DIR}"/*.service; do
    [[ -e "$f" ]] || continue
    [[ "$(basename "$f")" == "fleet-manager.service" ]] && continue
    install_unit "$f"
  done
else
  die "Missing ${UNIT_SRC_DIR} (expected systemd units)."
fi

systemctl daemon-reload

# Enable/disable services
if [[ "${USE_WEBRTC}" == "yes" ]]; then
  say "Enabling mediamtx.service..."
  systemctl enable --now mediamtx.service
else
  if systemctl list-unit-files | grep -q '^mediamtx\.service'; then
    say "Disabling mediamtx.service..."
    systemctl disable --now mediamtx.service || true
  fi
fi

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
