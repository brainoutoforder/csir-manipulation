#!/bin/bash

# Determine where the CF disk is mounted (sdb, for example)
MNT_PNT=$(sudo fdisk -l | grep "^Disk /dev/sd.: 8... MB" | grep -o "sd.")

# Prompt user to close pop-up windows
echo -n "Please close all windows showing CF card contents - Press Enter to Continue."
read -e POP_UPS

if [ ${MNT_PNT:-0} = "0" ]; then
  echo "Aborting! Could not locate CF Card - Please check if CF Card is inserted correctly"
else
  echo "Found 8GB CF Card mounted at: $MNT_PNT"

  sudo umount /media/*

  # Remove rules and ssh info
  sudo mkdir /media/tmp_mnt_root
  sudo mount /dev/${MNT_PNT}2 /media/tmp_mnt_root
  sudo rm /media/tmp_mnt_root/etc/udev/rules.d/70-persistent-net.rules
  sudo rm /media/tmp_mnt_root/etc/ssh/ssh_host_*

  # Unmount Media CF
  sudo umount /media/*

  echo -n "Please Enter a name for the CF Image. ie. Ubuntu910_backup: "
  read -e BACKUP_NAME

  # Creating backup archive
  time sudo fsarchiver -v -z 7 savefs ${BACKUP_NAME}Boot.fsa /dev/${MNT_PNT}1
  time sudo fsarchiver -v -z 7 savefs ${BACKUP_NAME}Root.fsa /dev/${MNT_PNT}2


  echo -e "\nBackup archives - ${BACKUP_NAME}Boot.fsa"
  echo "                  ${BACKUP_NAME}Root.fsa"
  echo "Created Successfully!"

fi
