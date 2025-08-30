WIFI_INTERFACE="wlan0"

# Turn Wi-Fi off
/usr/sbin/ifconfig "$WIFI_INTERFACE" down

cd /
# Run script
sudo python3 home/Pranav/Desktop/fe25/initial-tests/startup-test/blink_led.py
cd /

sleep 15

# Turn Wi-Fi back on
/usr/sbin/ifconfig "$WIFI_INTERFACE" up