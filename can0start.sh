#!/bin/bash
/usr/bin/busybox devmem 0x0c3030318 w 0x458
/usr/bin/busybox devmem 0x0c3030310 w 0x400
/usr/sbin/modprobe can
/usr/sbin/modprobe can_raw
/usr/sbin/modprobe mttcan
