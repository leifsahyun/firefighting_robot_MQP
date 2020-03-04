#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "acc_service.h"

/**
* Author: Leif Sahyun
* This node uses the distance-sensor-ros file in the acc_radar_code directory
* to get range measurements to nearest obstacle, then this node publishes that in ROS
*/

extern "C" {
	acc_service_configuration_t service_envelope_setup();
	bool service_envelope_takedown(acc_service_configuration_t);
	double execute_envelope(acc_service_configuration_t);
	double sample_average_dist(acc_service_configuration_t, int);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar_node");
	ros::NodeHandle nh;
	ros::Publisher radar_pub = nh.advertise<sensor_msgs::Range>("radar",10);
	ros::Rate loop_rate(10);

	acc_service_configuration_t config = service_envelope_setup();

	while(ros::ok())
	{
		double measurement = sample_average_dist(config, 5);
		sensor_msgs::Range distance_msg;
		distance_msg.range = measurement;
		
		radar_pub.publish(distance_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	service_envelope_takedown(config);
}
