#!/bin/bash
# Setup Raspberry Pi as WiFi access point "TrashCanNet"

set -e

SSID="TrashCanNet"
PASSPHRASE="trash1234"
WLAN_IF="wlan0"
STATIC_IP="192.168.4.1"
DHCP_RANGE_START="192.168.4.2"
DHCP_RANGE_END="192.168.4.20"
DHCP_LEASE_TIME="24h"

echo "Updating and installing packages..."
sudo apt update
sudo apt install -y hostapd dnsmasq

echo "Stopping services while we configure them..."
sudo systemctl stop hostapd || true
sudo systemctl stop dnsmasq || true

echo "Backing up original config files (if not already backed up)..."
if [ -f /etc/dhcpcd.conf ] && [ ! -f /etc/dhcpcd.conf.ap.bak ]; then
  sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.ap.bak
fi

if [ -f /etc/dnsmasq.conf ] && [ ! -f /etc/dnsmasq.conf.orig ]; then
  sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
fi

if [ -f /etc/hostapd/hostapd.conf ] && [ ! -f /etc/hostapd/hostapd.conf.ap.bak ]; then
  sudo cp /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.ap.bak
fi

echo "Configuring static IP for ${WLAN_IF} in /etc/dhcpcd.conf..."
sudo bash -c "cat >> /etc/dhcpcd.conf" <<EOF

# --- TrashCan AP configuration ---
interface ${WLAN_IF}
    static ip_address=${STATIC_IP}/24
    nohook wpa_supplicant
# --- end TrashCan AP configuration ---
EOF

echo "Restarting dhcpcd..."
sudo service dhcpcd restart

echo "Writing /etc/dnsmasq.conf..."
sudo bash -c "cat > /etc/dnsmasq.conf" <<EOF
interface=${WLAN_IF}
dhcp-range=${DHCP_RANGE_START},${DHCP_RANGE_END},255.255.255.0,${DHCP_LEASE_TIME}
EOF

echo "Writing /etc/hostapd/hostapd.conf..."
sudo mkdir -p /etc/hostapd
sudo bash -c "cat > /etc/hostapd/hostapd.conf" <<EOF
interface=${WLAN_IF}
driver=nl80211
ssid=${SSID}
hw_mode=g
channel=7
wmm_enabled=1
auth_algs=1
wpa=2
wpa_passphrase=${PASSPHRASE}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

echo "Pointing hostapd to its config file..."
sudo sed -i 's|^#\?DAEMON_CONF=.*$|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

echo "Enabling and starting services..."
sudo systemctl unmask hostapd || true
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo systemctl restart hostapd
sudo systemctl restart dnsmasq

echo
echo "Done. Reboot recommended."
echo "After reboot, connect your Mac to WiFi SSID: ${SSID} (password: ${PASSPHRASE})."
echo "The Pi will be at ${STATIC_IP}, so open:  http://${STATIC_IP}:5000"
