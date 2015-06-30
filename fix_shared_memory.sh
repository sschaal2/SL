#!/bin/bash

# this script releases some memory if SL did not terminate properly and does not start anymore
# usage: sudo sh fix_shared_memory.sh

# if this occurs often, you may need to setup your shared memory correctly:
# check your current setting:
#      sysctl -A | grep shm
# which should show something like:
#     ...
#     kern.sysv.shmmax: 16777216
#     kern.sysv.shmmin: 1
#     kern.sysv.shmmni: 128
#     kern.sysv.shmseg: 32
#     kern.sysv.shmall: 4096
#     ...
# if you have values inferior to this, you can create the file /etc/sysctl.conf and filling it with:
#    kern.sysv.shmmax=16777216
#    kern.sysv.shmmin=1
#    kern.sysv.shmmni=128
#    kern.sysv.shmseg=32
#    kern.sysv.shmall=4096



for i in `ipcs | awk '{print $2}' | grep -e "[0-9].*"`
do
     ipcrm -m $i
     ipcrm -s $i
     ipcrm -q $i
     ipcrm -M $i
     ipcrm -S $i
     ipcrm -Q $i
done

echo ""
echo "if you need to use this file often, please consider setting up your shared memory correctly. Run:"
echo "\tcat fix_shared_memory.sh"
echo "for instructions"