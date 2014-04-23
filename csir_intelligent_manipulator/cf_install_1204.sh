#!/bin/bash

# Download the latest CF Boot and Root Images to the current directory.
[ ! -f wamBootGeode1204.rev_4.fsa ] && wget http://web.barrett.com/support/WAM_Installer/wamBootGeode1204.rev_4.fsa
[ ! -f wamRootGeode1204.rev_4.fsa ] && wget http://web.barrett.com/support/WAM_Installer/wamRootGeode1204.rev_4.fsa

mv wamBootGeode1204.rev_4.fsa wamBootGeode1204.fsa
mv wamRootGeode1204.rev_4.fsa wamRootGeode1204.fsa

# Determine where the CF disk is mounted (sdb, for example)
MNT_PNT=$(sudo fdisk -l | grep "^Disk /dev/sd.: 8... MB" | grep -o "sd.")

if [ ${MNT_PNT:-0} = "0" ]; then
  echo "Aborting! Could not locate CF Card - Please check if CF Card is inserted correctly"
else
  echo "Found 8GB CF Card mounted at: $MNT_PNT"
  # Unmount Media CF
  sudo umount /media/*

  # Use parted to partition the disk
  sudo parted -s /dev/$MNT_PNT mklabel msdos
  sudo parted -s /dev/$MNT_PNT mkpart primary ext2 1 64
  sudo parted -s /dev/$MNT_PNT mkpart primary ext4 64 8000
  sudo parted -s /dev/$MNT_PNT set 1 boot on
  
  # Restore filesystem images
  time sudo fsarchiver -v restfs wamBootGeode1204.fsa id=0,dest=/dev/${MNT_PNT}1

  time sudo fsarchiver -v restfs wamRootGeode1204.fsa id=0,dest=/dev/${MNT_PNT}2

  # Mount the new system
  sudo rm -rf /media/${MNT_PNT}2
  sudo mkdir /media/${MNT_PNT}2
  sudo mount /dev/${MNT_PNT}2 /media/${MNT_PNT}2
  sudo mount /dev/${MNT_PNT}1 /media/${MNT_PNT}2/boot
  sudo mount --bind /dev /media/${MNT_PNT}2/dev
  
  # Install the bootloader
  # Convert the 'b' of 'sdb' into the number '1', for example (sdb = hd1 for GRUB)
  let val=`echo ${MNT_PNT:2} |od -An -t dC |awk '{print $1}'`-97
  echo "val = $val"
  # Create a GRUB install script
cat <<-EOF | sudo tee /media/${MNT_PNT}2/grub.sh
     #!/bin/sh
     /usr/sbin/grub --batch <<END 
     find /boot/grub/stage1
     root (hd1,0)
     setup (hd1)
     quit
EOF
  # chroot into the new system and call our GRUB install script
  sudo chroot /media/${MNT_PNT}2 bash grub.sh

  # Delete the GRUB install script
  sudo rm /media/${MNT_PNT}2/grub.sh

  # Unmount
  sudo umount /media/${MNT_PNT}2/dev
  sudo umount /media/${MNT_PNT}2/boot
  sudo umount /media/${MNT_PNT}2

  #Clean up
  rm wamBootGeode1204.fsa
  rm wamRootGeode1204.fsa
  echo "CF Card Installation Complete! Please remove your new CF Card!"
fi


