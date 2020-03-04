#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "radar_processing_helpers.h"
#include "acc_service.h"

/**
* Author: Leif Sahyun
* This node uses the distance-sensor-ros file in the acc_radar_code directory
* to get range measurements to nearest obstacle, then this node publishes that in ROS
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_node");
	ros::NodeHandle nh;
	ros::Publisher radar_pub = nh.advertise<sensor_msgs::Range>("radar_node",1000);
	ros::Rate loop_rate(10);

	acc_service_configuration_t config = service_envelope_setup();

	while(ros::ok())
	{
		double measurement = execute_envelope(config);
		sensor_msgs::Range distance_msg;
		distance_msg.range = measurement;
		printf("range measurement through ros node: %f\n", measurement);
		radar_pub.publish(distance_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	service_envelope_takedown(config);
}
