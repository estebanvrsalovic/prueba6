VM deployment helper scripts

Files:
- `vm_deploy.sh`: run on a fresh Debian/Ubuntu VM as root to install Docker, clone your repo and start the backend container.

How to use:
1. Create a VM (Oracle Always Free, DigitalOcean droplet, etc.) and SSH into it.
2. Upload or `git clone` this repository on the VM, or run `sudo /path/to/vm_deploy.sh GIT_URL [branch]`.
3. Edit `web/.env` created by the script to set `MQTT_BROKER` and `FRONTEND_URL`.
4. Ensure the VM firewall allows port 3000 or configure a reverse proxy (nginx) with TLS.
