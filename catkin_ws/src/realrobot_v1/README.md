## name variable rules:

    1)  read() and wriet(), they are communication function,
 read() means acquire data from other files, device , class or modules; write() means transfer data to these place

    2) getOutput() and setInput(), they are interface of class, or modules, they are doors for communciating between them with envioroment
 
## Test joystick   
    sudo apt-get install jstest-gtk
    sudo jstest /dev/input/js0

## list more usb information
    enter:  udevadm info -a -n /dev/ttyACM0
    and I do add feature of "serial" to specify usb devices

## add device alias name of u2d2
    cd /etc/udev/rules.d/
    sudo touch remap_u2d2_name.rules
    sudo vim remap_u2d2_name.rules
        write --- ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="U2D2"
        ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
        ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="U2D2"
        ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="VN100"
        ACTION=="add", ATTRS{idVendor}=="1b4f", ATTRS{idProduct}=="f019", ATTRS{serial}=="B043674B503051364B2E3120FF06022E", SYMLINK+="BOTA_FL"
        ACTION=="add", ATTRS{idVendor}=="1b4f", ATTRS{idProduct}=="f019", ATTRS{serial}=="A3A28718503051364B2E3120FF0D0B1D", SYMLINK+="BOTA_FR"
        ACTION=="add", ATTRS{idVendor}=="1b4f", ATTRS{idProduct}=="f019", ATTRS{serial}=="D7D9BAA2503051364B2E3120FF132D11", SYMLINK+="BOTA_HL"
        ACTION=="add", ATTRS{idVendor}=="1b4f", ATTRS{idProduct}=="f019", ATTRS{serial}=="882ACF5B503051364B2E3120FF113013", SYMLINK+="BOTA_HR"

## then disconnect device and enter follow command:
    sudo service udev reload
    sudo service udev restart
    or 
    udevadm control --reload-rules && udevadm trigger
## and reconnect device,then you should find U2D2 device node in /dev folder

## about communication frequency
## real robot sensor data topic publish frequency depends on the read time of syncReadGroup,It has two main key:
## 1.USB interface latency_timer,## DynamixelSDK function 
    Method 1. Type following (you should do this everytime when the usb once was plugged out or the connection was dropped) 
        echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer  
    Method 2. If you want to set it as be done automatically, and don't want to do above everytime, make rules file in /etc/udev/rules.d/. For example, 
        echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules 
        sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/ 
        sudo udevadm control --reload-rules 
        sudo udevadm trigger --action=add 
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
## 2.the return delay time of dynamixel motor, you should set them in init function of real robot,otherwise the read time of syncRead will be very long.
    for example:one motor,four state feedback sync handle, 500us return delay time, it will spend (0.5x4 + 1x4 + read time)ms>6ms
    but if set the return delay time to 0,it will reduce to about (1x4 + read time)ms,around 4ms
## Note: the number of motors has little effect on the syncRead time if the return delay time set to 0,but it will be great effect on the syncRead time if the return delay time set to 500us.