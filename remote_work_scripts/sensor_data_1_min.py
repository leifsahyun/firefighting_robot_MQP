#!/usr/bin/env python
import subprocess
import os

print("opening serial connection to esp32")
serial = None
try:
	serial = subprocess.Popen(["rosrun", "rosserial_python", "serial_node.py"])
	if serial.poll() is None:
		print("serial node opened")
	else:
		print("error opening serial node")
except:
	print("error opening serial node")

print("opening radar data node")
radar = None
try:
	radar = subprocess.Popen(["~/catkin_ws_mqp/devel/lib/radar_code/ros_radar_pub"])
	if radar.poll() is None:
		print("radar node opened")
	else:
		print("error opening radar node")
except:
	print("error opening radar node")

print("opening rosbag for recording")
bag_process = None
try:
	bag_process = subprocess.Popen(["rosbag", "record", "-a", "-o", "sensor_data_sample", "--duration=1m"])
	if bag_process.poll() is None:
		print("rosbag is recording")
	else:
		print("error opening rosbag process")
except:
	print("error opening rosbag process")

print("recording sensor data...")
if bag_process is not None:
	while bag_process.poll() is None:
		pass
	print("record complete")
else:
	print("error with rosbag process")

print("compressing record")
cwd = os.getcwd()
compression = subprocess.Popen("rosbag compress "+cwd+"/sensor_data_sample*", shell=True)
while compression.poll() is None:
	pass
print("record compressed")

if serial is not None:
	serial.kill()
if radar is not None:
	radar.kill()
