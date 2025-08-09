# Creating a setup on windows 10 with ubuntu 22 to work on the imoco project.

The "windows subsystem for linux" (WSL) version 02, although not version 01, can be used to create a setup that allows the user to work on the imoco project. This setup behaves more like a dual boot of windows with a linux distro and less like an instance of a linux distro in a virtual machine on windows, which allows better utilisation of all hardware resources. Here, we'll choose the ubuntu 22 distro.

Since the WSL 2 is originally intended to be used as a text terminal, it doesn't have a pre-installed desktop GUI package but we can install one of our choice, just like in any other instance of the ubuntu 22 distro. Here, we'll choose the lightweight XFCE desktop GUI package.

To use the desktop GUI features for the WSL 2 installation, we must to access the ubuntu 22 inside WSL 2 like a network computer, so we need to use the "remote desktop connection" (RDP) protocol. It comes pre-installed in windows 10 and we can also install it in ubuntu 22 through the XRDP package. To be clear, we've yet to test this setup with windows 11, so it may not work there.

We must make some configuration changes to avoid the network connection breaking suddenly. By default, this setup treats the WSL 2 instance like an already running network connection, which counts towards its maximum of one connection at a time, so we change the settings file to bypass that limitation.

After installing the XFCE and XRDP packages, we run an RDP server inside the WSL 2 text terminal. After that, we run the RDP client though its GUI features outside the WSL 2 text terminal in windows 10. Finally, we should be able to use either "localhost:3390" or "localhost:3389" as the IP/URL address with the RDP client's GUI features for creating a network connection between the two sides.

The only limitation of this setup is that the same software can't be run one both sides for ubuntu 22 here (the RDP client GUI side and the WSL 2 text terminal side). For example, since we'll be using the RDP server on the WSL 2 text terminal side, therefore we can't use it on the RDP client GUI side.

Inside ubuntu 22, we can install docker engine and visual studio code editor with sudo commands in the terminal. With them, we can start working on the imoco project. It should be possible to use a similar setup for linux distros inside docker container to use them with GUI features when running them on windows 10.

As the WSL 2 installation can access hardware resources with almost no restrictions, it should be possible to connect the docker container with the imoco project's hardware without any significant issues.

Going through the lists of refereces is recommended before using any of the listed commands to avoid any issues caused by accidental oversimplification of steps mentioned here. Currently, the ubuntu distro for WSL 2 is version 22 by default because it's the latest LTS version but that can change when the next LTS version is released.

# Steps for the windows 10 powershell administrator.

wsl --install

wsl --set-default-version 2

wsl --list --online

wsl --install -d Ubuntu

# Steps for the ubuntu 22 terminal.

sudo apt-get update -y && sudo apt-get upgrade -y

sudo apt install -y xrdp xfce4 xfce4-goodies

sudo cp /etc/xrdp/xrdp.ini /etc/xrdp/xrdp.ini.bak

sudo sed -i 's/3389/3390/g' /etc/xrdp/xrdp.ini

sudo sed -i 's/max_bpp=32/#max_bpp=32\nmax_bpp=128/g' /etc/xrdp/xrdp.ini

sudo sed -i 's/xserverbpp=24/#xserverbpp=24\nxserverbpp=128/g' /etc/xrdp/xrdp.ini

echo xfce4-session > ~/.xsession

sudo nano /etc/xrdp/startwm.sh

<make necessary changes based on reference notes>

sudo /etc/init.d/xrdp start

<access WSL with "localhost:3390" as URL in RDP>

sudo /etc/init.d/xrdp stop

# Reference links.

01. https://learn.microsoft.com/en-us/windows/wsl/install ~ "How to install Linux on Windows with WSL"

02. https://hub.tcno.co/windows/wsl/desktop-gui/  ~ "Install Desktop GUI for WSL | WSL Enable Desktop Guide"

03. https://c-nergy.be/blog/?p=16698 ~ "xRDP – Allow multiple sessions (local and remote) for the same user – HowTo"

04. https://learn.microsoft.com/en-us/windows/wsl/connect-usb ~ "WSL | Connect USB devices"

05. https://github.com/dorssel/usbipd-win/wiki/WSL-support ~ "usbipd-win | WSL support"

06. https://github.com/PINTO0309/wsl2_linux_kernel_usbcam_enable_conf ~ "Configuration file to build the kernel to access the USB camera connected to the host PC using USBIP from inside the WSL2."

07. https://github.com/IntelRealSense/realsense-ros ~ "ROS Wrapper for Intel(R) RealSense(TM) Cameras"

08. https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md ~ "librealsense | Linux Distribution"

09. https://askubuntu.com/questions/668829/lsusb-command-not-found

# Reference notes.

"/etc/xrdp/startwm.sh" file contents after edit:

  GNU nano 6.2                                      
  /etc/xrdp/startwm.sh                                                
  #!/bin/sh

if test -r /etc/profile; then
        . /etc/profile
fi

unset DBUS_SESSION_BUS_ADDRESS # workaround 03

unset XDG_RUNTIME_DIR # workaround 03

startxfce4

# Author details.

Swarn S. W. (it23139[at]lbtu.lv)
