#!/usr/bin/env bash
# certs/setup-certs.sh
#
# Generates a local CA and a server certificate signed by it.
# Install ca.crt on your phone/desktop once; the browser will then trust the
# picar HTTPS servers without any warning or manual cert-acceptance step.
#
# Usage:
#   cd /path/to/picar/certs
#   bash setup-certs.sh
#
# Outputs:
#   ca.key      CA private key        — keep on Pi, never share
#   ca.crt      CA certificate        — install on every client device
#   key.pem     Server private key    — used by app.js
#   cert.pem    Server certificate    — used by app.js

set -e

# ── Prompt for Pi address ─────────────────────────────────────────────────────
echo ""
echo "Enter the IP address or hostname that clients will use to reach the Pi."
echo "Examples:  192.168.1.42   |   picar.local"
echo -n "Pi address: "
read -r PI_ADDR

if [ -z "$PI_ADDR" ]; then
  echo "Error: no address entered." >&2
  exit 1
fi

# Decide whether the SAN is an IP or a DNS name
if [[ "$PI_ADDR" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  SAN="IP:${PI_ADDR}"
else
  SAN="DNS:${PI_ADDR}"
fi

echo ""
echo "Using SAN: $SAN"
echo ""

# ── CA ────────────────────────────────────────────────────────────────────────
echo "1/4  Generating CA key…"
openssl genrsa -out ca.key 2048

echo "2/4  Generating CA certificate (10-year lifetime)…"
openssl req -new -x509 -days 3650 \
  -key ca.key \
  -out ca.crt \
  -subj "/CN=PiCar Local CA"

# ── Server cert ───────────────────────────────────────────────────────────────
echo "3/4  Generating server key…"
openssl genrsa -out key.pem 2048

echo "4/4  Generating server certificate signed by CA…"

# Write a temporary OpenSSL config with the SAN extension.
# iOS Safari requires a SAN; a CN-only cert is rejected even from a trusted CA.
EXTFILE=$(mktemp /tmp/picar-cert-ext.XXXXXX)
trap 'rm -f "$EXTFILE"' EXIT

cat > "$EXTFILE" <<EOF
[req]
req_extensions     = v3_req
distinguished_name = dn
prompt             = no
[dn]
CN = ${PI_ADDR}
[v3_req]
subjectAltName = ${SAN}
EOF

openssl req -new \
  -key key.pem \
  -out /tmp/picar-server.csr \
  -config "$EXTFILE"

openssl x509 -req -days 3650 \
  -in  /tmp/picar-server.csr \
  -CA  ca.crt -CAkey ca.key -CAcreateserial \
  -out cert.pem \
  -extfile "$EXTFILE" \
  -extensions v3_req

rm -f /tmp/picar-server.csr

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "Done. Files written to $(pwd):"
echo "  ca.crt   ← install this on every client device (phone, laptop, etc.)"
echo "  cert.pem ← server certificate (used by app.js)"
echo "  key.pem  ← server key         (used by app.js)"
echo ""
echo "Verify server cert SAN:"
openssl x509 -in cert.pem -noout -subject -issuer -dates -ext subjectAltName
echo ""
echo "Next step: copy ca.crt to your phone and install it as a trusted CA."
echo "See the 'HTTPS Certificates' section of README.md for instructions."
