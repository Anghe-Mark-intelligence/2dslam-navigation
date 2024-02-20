/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

enum magic_STATE {
	
};



double distanceBetweenTwoPoints(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b){
	return sqrt(pow(a.pose.position.x - b.pose.position.x, 2) + pow(a.pose.position.y - b.pose.position.y, 2)
			+ pow(a.pose.position.z - b.pose.position.z, 2));
}

double distanceBetweenTwoPoints(float ax, float ay, float bx, float by){
	return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "aecom_commander_ugv");
	ros::NodeHandle nh;

	ros::Publisher robot_arm = nh.advertise<std_msgs::String>
	("/write_anc", 1);
	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(10);

	current_t_state = IDLE_DISARM;


	cmd_to_robot_arm[0].data = "{G0000#000P1500T1000!#001P1500T2000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}";
	cmd_to_robot_arm[1].data = "{G0001#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P1500T1000!}";
	cmd_to_robot_arm[2].data = "{G0002#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P0600T0300!}";
	cmd_to_robot_arm[3].data = "{G0003#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P1500T0300!}";
	cmd_to_robot_arm[4].data = "{G0004#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P1500T0300!}";
	cmd_to_robot_arm[5].data = "{G0005#000P1500T1000!#001P1500T2000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}";
	cmd_to_robot_arm[6].data = "{G0006#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P1500T1000!}";
	cmd_to_robot_arm[7].data = "{G0007#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P0600T0300!}";
	cmd_to_robot_arm[8].data = "{G0008#000P1500T1000!#001P1000T2000!#002P1600T1000!#003P1600T1000!#004P0830T1000!#005P1500T0300!}";
	cmd_to_robot_arm[9].data = "{G0009#000P2125T1000!#001P1350T2000!#002P2100T1000!#003P0950T2000!#004P1450T1000!#005P1200T1000!#006P1500T1000!#007P1500T1000!#008P1500T1000!#009P1500T1000!}";

	while(ros::ok()){
		if(count % 30 == 0)
		{
		robot_arm_pub.publish(cmd_to_robot_arm[robot_move_count]);
		robot_move_count++;
		}

		count++;
		if(robot_move_count > 9)
		{
			return 0;
		}
		ros::spinOnce();

	}
	return 0;
}
