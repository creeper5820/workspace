#!/bin/bash
sudo apt install -y /home/ubuntu/jlink.deb /home/ubuntu/ozone.deb
sudo ln -s /usr/bin/JLinkExe /usr/bin/JLink
sudo rm -rf /home/ubuntu/install.sh /home/ubuntu/zsh-install.sh /home/ubuntu/jlink.deb /home/ubuntu/ozone.deb