echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout",  SYMLINK+="xiaoyuan_base"' >/etc/udev/rules.d/xiaoyuan_base.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout",  SYMLINK+="xiaoyuan_imu"' >/etc/udev/rules.d/xiaoyuan_imu.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout",  SYMLINK+="xiaoyuan_rplidar"' >/etc/udev/rules.d/xiaoyuan_rplidar.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0006", MODE:="0777", GROUP:="dialout",  SYMLINK+="xiaoyuan_follow"' >/etc/udev/rules.d/xiaoyuan_follow.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="xiaoyuan_follow"' >/etc/udev/rules.d/xiaoyuan_follow_ch340.rules

service udev reload
sleep 2
service udev restart


