#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="nanorobot_base"' >/etc/udev/rules.d/nanorobot_base_2303.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="nanorobot_base"' >/etc/udev/rules.d/nanorobot_base_340.rules

service udev reload
sleep 2
service udev restart