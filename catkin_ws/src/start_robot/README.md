## How to connect joystick by bluetooth

1. 插入usb蓝牙适配器

2、查看适配器是否被系统识别
lsusb

3、安装蓝牙软件包
sudo apt-get install bluetooth bluez bluez-tools rfkill

 4、启用蓝牙服务
sudo systemctl start bluetooth.service


5、打开设备的蓝牙，终端输入以下命令启动蓝牙控制器
bluetoothctl

6、[bluetooth]命令行提示符下，启动扫描

scan on
7、出现设备名称和MAC地址后，连接设备

pair <设备MAC地址>
8、输入pin配对码（设备上获得），设备上也需要同意配对

9、设备断连后重新连接，需要[bluetooth]命令行提示符下，输入以下命令删除设备，重复7-8

remove <设备地址码>
10、输入以下命令建立串口连接

sudo rfcomm bind /dev/rfcomm0 <设备地址> <端口号>

11、输入minicom进入控制台，即可发送和接收蓝牙数据。

## How to remap usb device 
- remap dynamixel motors' port
add follow commands in sudo vim /etc/udev/rules.d/com_port.rules
KERNEL=="ttyUSB*" , ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="7523",MODE:="0   777" ,SYMLINK+="U2D2"
and then active it by following cmd in terminal
sudo  sudo udevadm trigger


