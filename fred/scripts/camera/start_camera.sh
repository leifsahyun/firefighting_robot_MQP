#!/bin/bash
DEST_HOSTNAME=autoreg-071220.dyn.wpi.edu
PORT=3333
raspivid --vstab -w 640 -h 480 -t 0 -fps 30 -o - | nc -u $DEST_HOSTNAME $PORT
