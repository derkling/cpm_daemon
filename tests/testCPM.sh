#!/bin/sh

if [ `id -u` -ne 0 ]; then
	echo "This script must be run as root"
	echo "Please sudo [username=user,password=pass] and then run it again"
	exit 1
fi

DIR=`pwd`

# Set window title for further window management
wmctrl -r :ACTIVE: -N "CPM Test Script"

xterm -geometry 190x36+0+24 -title "CPM Kernel Log" -e tail -f /var/log/syslog &
sleep 1 && wmctrl -a "CPM Test Script"

echo "Loading FSC-identification (exhaustive) GOVERNOR..."
modprobe cpm_governor_exhaustive > /dev/null 2>&1

echo "Loading FSC-ordering (performance) optimization POLICY..."
modprobe cpm_policy_performance > /dev/null 2>&1

echo "Loading (fake) system configuration..."
modprobe cpm_test_mp3gsm > /dev/null 2>&1

echo "Enabling CPM kernel framework..."
cd /sys/kernel/cpm
echo 1 > enable

echo "Press a key to load a new optimization policy"
read KEY

echo "Configuring optimization weights..."
cd /sys/kernel/cpm/swms
echo 1:1 > weight
echo 0:-40000 > weight

echo "Press a key start the CPM daemon"
read KEY

xterm  -geometry 100x21+0-36 -title "CPM Daemon Log" -e $DIR/cpm_daemon -c $DIR/xps_example.conf -d &
sleep 1 && wmctrl -a "CPM Test Script"

echo "CPM Up and Running"

echo "Press a key to stop the demo"
read KEY

echo "Disabling CPM kernel framework..."
cd /sys/kernel/cpm
echo 0 > enable

echo "Removing (fake) system configuration..."
rmmod cpm_test_mp3gsm

