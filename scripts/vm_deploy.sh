#!/usr/bin/env bash
set -euo pipefail

# vm_deploy.sh
# Usage on the VM:
#   sudo ./vm_deploy.sh GIT_REPO_URL [BRANCH]
# Example:
#   sudo ./vm_deploy.sh https://github.com/usuario/prueba6.git main

REPO_URL=${1:-}
BRANCH=${2:-main}
DEST_DIR=/opt/prueba6

if [ -z "$REPO_URL" ]; then
  echo "Usage: $0 GIT_REPO_URL [BRANCH]"
  exit 1
fi

echo "Updating OS and installing Docker & docker-compose..."
apt-get update
apt-get install -y apt-transport-https ca-certificates curl gnupg lsb-release git
curl -fsSL https://get.docker.com | sh
systemctl enable --now docker
if ! command -v docker-compose >/dev/null 2>&1; then
  curl -L "https://github.com/docker/compose/releases/download/v2.20.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
  chmod +x /usr/local/bin/docker-compose
fi

echo "Cloning repository into $DEST_DIR"
rm -rf "$DEST_DIR"
git clone --depth 1 --branch "$BRANCH" "$REPO_URL" "$DEST_DIR"

cd "$DEST_DIR/web"

echo "Creating .env file for docker-compose (edit values as needed)"
cat > .env <<EOF
# MQTT broker URL reachable from the device/VM. Use mqtts if available.
MQTT_BROKER=mqtt://test.mosquitto.org:1883
# FRONTEND_URL used for CORS in server
FRONTEND_URL=
EOF

echo "Building and starting containers"
docker-compose up -d --build

echo "Done. Backend should be running on port 3000."
echo "Remember to open firewall port 3000 (or better, put a reverse proxy with HTTPS in front)."
