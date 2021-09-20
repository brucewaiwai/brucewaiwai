echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="TMwheeltec_controller", KERNELS=="4-2"' >/etc/udev/rules.d/TMwheeltec_controller.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="TMbattery", KERNELS=="1-3.4"' >/etc/udev/rules.d/TMbattery.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", GROUP:="dialout", SYMLINK+="TMdelta"' >/etc/udev/rules.d/TMdelta.rules
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE:="0777", GROUP:="dialout", SYMLINK+="TMarduino_mega"' >/etc/udev/rules.d/TMarduino_mega.rules

service udev reload
sleep 2
service udev restart


