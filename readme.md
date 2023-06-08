Aux Sensor Realsense T265 ke teensy 4.1
dengan Rapsbeery Pi 4, Ubuntu 20.04

1. Download and run code	
- $ git clone https://github.com/jihadrahmawan/RealsenseToSerial.git
- $ cd RealsenseToSerial/
- $ chmod +x T265ToSerial.py
- $ cd ..
- $ sudo nano t265.sh

isikan:
#!/bin/bash
/home/ubuntu/RealsenseToSerial/T265ToSerial.py


	- $ chmod +x t265.sh
	- $ ./t265.sh


	- pastikan sudah ter run dengan baik.
	

2. membuat auto run at boot
	- $ sudo nano /etc/systemd/system/t265.service
	- isikan:
[Unit]
Description=Realsense T265 Service
After=multi-user.target
StartLimitIntervalSec=0
Conflicts=

[Service]
User=ubuntu
EnvironmentFile=
ExecStartPre=
ExecStart=/home/ubuntu/t265.sh

Restart=on-failure
RestartSec=1

[Install]
WantedBy=multi-user.targe

	$ systemctl start t265
	$ systemctl enable t265
	
	
3. Reboot and test hasil nya,
  Jika saat selang 30 sec ~ 1min indikator teensy masih merah, 
  maka bisa melakukan hardplug dengan mencabut kabel usb Realsense, kemudian jeda 3 detik baru dicolokkan kembali.
