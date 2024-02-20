/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h> 
#include <geometry_msgs/Twist.h>   //ugv cmd
//#include <ros_four1_msg/four1.h>  //ugv msg
#include <ros_agv4_msg/agv1.h>  //ugv msg
#include <math.h>
#include <tuple>
#include <vector>
#include <functional>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mavros_msgs/CommandLong.h>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

typedef Eigen::Vector3d Point;


enum MAGIC1_STATE {

};


geometry_msgs::PoseStamped sp_pose;
geometry_msgs::PoseStamped local_pose;
geometry_msgs::Twist ugv_sp_vel;
geometry_msgs::Transform ugv_pose;


tf::Quaternion q_local, q_orientation;
tf::Matrix3x3 m;


int current_t_state;
int last_t_state;
double roll_current, pitch_current, yaw_current;
double accept_distance;
bool line_yaw_updated = false;
int loop_counter = 0;


double distanceBetweenTwoPoints(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b){
	return sqrt(pow(a.pose.position.x - b.pose.position.x, 2) + pow(a.pose.position.y - b.pose.position.y, 2)
			+ pow(a.pose.position.z - b.pose.position.z, 2));
}

double distanceBetweenTwoPoints_seperate(float ax, float ay, float bx, float by){
	return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
}
double AngleFromOrigin(float z1, float z2){
	return sqrt(pow(z1 - z2, 2));
}
float wrap_pi(float a)
{
	if (!isfinite(a)) {
		return a;
	}
	return fmod(a + M_PI, 2.0f * M_PI) - M_PI;
}
float getDegAngle(Point p1, Point p2, Point p3)
{
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;

}
bool angle_controller(float angle_vz_sp, float sp_angle_z, float angle_z, float time)
{
	float sp_angle_z_wrap = wrp_pi(sp_angle_z);

	ugv_sp_vel.linear.x = 0.0f;
	ugv_sp_vel.angular.z = angle_vz_sp * fabs(sp_angle_z_wrap - angle_z);

	//normal case, angular speed is the same half circle of sp angle, means ugv will not move more than 180 degree
	if((sp_angle_z_wrap - angle_vz_sp <= 3.14f) && (sp_angle_z_wrap - angle_vz_sp >= -3.14f) &&
			/*(((angle_z > (sp_angle_z - 0.05f)) && (angle_vz_sp > 0.0f))
			|| ((angle_z < (sp_angle_z + 0.05f)) && (angle_vz_sp < 0.0f)))*/
			fabs(angle_z - sp_angle_z_wrap) < 0.01f)
	{
		ugv_sp_vel.angular.z = 0.0f;
		printf("angle_z, %f, wrap angle_sp %f\n", angle_z, sp_angle_z_wrap);
		return true;

	}
	// if ugv need to turn more than 180 degree
/*	else if (sp_angle_z_wrap * angle_vz_sp < 0)
	{
		ugv_sp_vel.angular.z = 0.0f;
		printf("not allowed, angle, velocity need to be same sign, or more than 180 degree, angle_z %f, wrap angle_sp %f\n", angle_z, sp_angle_z_wrap);
		return true;
	}*/
	else
	{
		return false;
	}
}
bool line_controller(float vx_sp, float sp_x, float sp_y, float pose, float pose_y, float yaw,float time)
{
	ugv_sp_vel.linear.x = vx_sp;
	float yaw_sp;
	//if(!line_yaw_updated){
	//	yaw_sp = wrap_pi((float)atan((sp_x - pose_x)/(sp_y - pose_y)));
	//	line_yaw_updated = true;
	//}
	Point p1(pose_x, pose_y, 0), p2(sp_x, sp_y, 0);

	if(vx_sp >= 0){
	Point p3(pose_x + cosf(yaw), pose_y + sinf(yaw), 0);
	yaw_sp = getDegAngle(p1,p2,p3);

	if((yaw_sp > 0.0f && yaw_ < 90.0f) || (yaw_sp >180 && yaw_sp < 270))
	{
		ugv_sp_vel.angular.z = -vx_sp;//by experiment, this configuration is ok with testing//-0.2f;//-0.005f * yaw_sp;
		if(yaw_sp > 0.0f && yaw_sp < 20.0f)
		{
			ugv_sp_vel.angular.z = -yaw_sp *  (vx_sp/ 20.0f);//-0.005f * yaw_sp;
		}
	}
	else if((yaw_sp > 270.0f && yaw_sp < 360.0f) || (yaw_sp >90.0f && yaw_sp < 180.0f))
	{
		ugv_sp_vel.angular.z = vx_sp;//0.2f;//-0.005f * yaw_sp);
		if(yaw_sp > 340.0f && yaw_sp < 360.0f)
		{
			ugv_sp_vel.angular = (360.0f - yaw_sp) *  (vx_sp/ 20.0f);//0.005f * (360.0f - yaw_sp);
		}
	}
	else
	{
		ugv_sp_vel.angular.z = 0.0f;
		ugv_sp_vel.linear.x = 0.0f;
		if(loop_counter % 20 == 0)
		{
		printf("wrong angle, refuse to execute\n");
		}
	}
	}
	else
	{
		Point p3(pose_x + cosf(yaw ), pose_y + sinf(yaw + M_PI), 0);
		yaw_sp = getDegAngle(p1,p2,p3);

		if((yaw_sp > 0.0f && yaw_sp) || (yaw_sp >180 && yaw_sp < 270))
		{
			ugv_sp_vel.angular.z = vx_sp;//-0.2f;//-0.005f * yaw_sp;
			if(yaw_sp > 0.0f && yaw_sp < 20.0f)
			{
				ugv_sp_vel.angular.z = yaw_sp *  (vx_sp/ 20.0f);//-0.005f * yaw_sp;
			}
		}
		else if((yaw_sp > 270.0f && yaw_sp < 360.0f) || (yaw_sp >90.0f && yaw_sp < 180.0f))
		{
			ugv_sp_vel.angular.z =;//0.2f;//-0.005f * yaw_sp);
			if(yaw_sp > 340.0f && yaw_sp < 360.0f)
			{
				ugv_sp_vel = (360.0f - yaw_sp) *  (-vx_sp/ 20.0f);//0.005f * (360.0f - yaw_sp);
			}
		}
		else
		{
			ugv_sp_vel.angular.z = 10.0f;
			ugv_sp_vel.linear.x = 10.0f;
			if(loop_counter % 20 == 0)
			{
			printf("wrong angle, refuse to execute\n");
			}
		}
	}


	
}
bool line_controller_noyaw(float vx_sp, float vy_sp, float sp_x, float sp_y, float pose_x, float pose_y, float yaw,float time)
{
//	ugv_sp_vel.linear.x = vx_sp;
	ugv_sp_vel.angular.z = 0.0f;
	float yaw_sp;
	//if(!line_yaw_updated){
	//	yaw_sp = wrap_pi((float)atan((sp_x - pose_x)/(sp_y - pose_y)));
	//	line_yaw_updated = true;
	//}
	Point p1(pose_x, pose_y, 0), p2(sp_x, sp_y, 0), p1_x(pose_x, 0, 0), p1_y(0, pose_y, 0), p2_x(sp_x, 0, 0), p2_y(0, sp_y, 0);
	Point p3(pose_x + cosf(yaw), pose_y + sinf(yaw), 0);

	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;

	Eigen::Vector3d v1_x = p2_x - p1_x;
	Eigen::Vector3d v1_y = p2_y - p1_y;

	if(loop_counter % 20 == 0)
	{
		printf("spx %f, spy %f, posex %f, posey %f\n", sp_x, sp_y, pose_x, pose_y);
	}
	//ugv_sp_vel.angular.z = 0.0f;//(yaw - yaw_sp);

	
	/*ugv_sp_vel.linear.x */float error_x = v1_x.norm() * vx_sp;
	/*ugv_sp_vel.linear.y*/ float error_y = v1_y.norm() * vy_sp;


	if(distanceBetweenTwoPoints_seperate(sp_x, sp_y, pose_x, pose_y) < 0.1f)
	{
		ugv_sp_vel.linear.x = 0.0f;
		ugv_sp_vel.linear.y = 0.0f;
		ugv_sp_vel.angular.z = 0.0f;
		line_yaw_updated = false;
		return true;
	}
	else
	{
	return false;
	}
}


