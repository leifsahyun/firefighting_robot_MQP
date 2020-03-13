#!/bin/bash
PI_HOSTNAME="autoreg-293983.dyn.wpi.edu"
PORT=3333
OUT_FIFO="video_pipe.h264"
# if [ ! -f $OUT_FIFO ]; then
#	mkfifo $OUT_FIFO
# fi
nc -u -l -k -p $PORT | cat > $OUT_FIFO
