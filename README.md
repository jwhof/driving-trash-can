create wifi hotspot
sudo nmcli dev wifi hotspot ifname wlan0 con-name trashcan-hotspot ssid TrashCanNet password 'trash1234'

ip addr show wlan0

http://10.42.0.1:5000

turn it off
sudo nmcli connection down trashcan-hotspot
sudo nmcli radio wifi off
