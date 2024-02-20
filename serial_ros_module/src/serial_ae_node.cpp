/***
 * aecom serial library
 * author: Eric Wei Weiye
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial ser;
std::string dev_name;
std::string read_to_topic;
std::string write_from_topic;
std::string param_read_to_topic = "/";
std::string param_read_enable = "/";
std::string param_write_from_topic = "/";
std::string param_write_enable = "/";
std::string param_dev_name = "/";
std::string param_baudrate = "/";
bool read_enable;
bool write_enable;
int baudrate;
void write_callback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO_STREAM("Writing to serial port" << msg->data);
	ser.write(msg->data);
}
void append_param_name()
{   /* append as /instance_name */
	param_read_to_topic.append(ros::this_node::getName());
	param_read_enable.append(ros::this_node::getName());
	param_write_enable.append(ros::this_node::getName());
	param_dev_name.append(ros::this_node::getName());


	/* apeend as /instance_name/param_name */
	param_read_to_topic.append("/read_to_topic");
	param_read_enable.append("/read_enable");
	param_write_from_topic.append("/write_from_topic");
	param_write_enable.append("/write_enable");
	param_dev_name.append("/dev_name");
	param_baudrate.append("/baudrate");
}

int main (int argc, char** argv){
	ros::init(argc, argv, "serial_ae_node");
	ros::NodeHandle nh;
	ros::this_node::getName();
	append_param_name();

	nh.getParam(param_dev_name, dev_name);
	if(baudrate<0)
	{
		ROS_ERROR_STREAM("invalid device name");
		return -1;
	}

	nh.getParam(param_baudrate, baudrate);
	if(baudrate<0)
	{
		ROS_ERROR_STREAM("baudrate must be positive");
		return -1;
	}
	nh.getParam(param_read_to_topic, read_to_topic);
	nh.getParam(param_read_enable, read_enable);
	nh.getParam(param_write_from_topic, write_from_topic);
	nh.getParam(param_write_enable, write_enable);

	ros::Subscriber write_sub = nh.subscribe(write_from_topic, 1, write_callback);
	ros::Publisher read_pub = nh.advertise<std_msgs::String>(read_to_topic, 1000);

	try
	{
		ser.setPort(dev_name);
		ser.setBaudrate((unsigned long)baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized");
	}else{
		return -1;
	}

	ros::Rate loop_rate(5);
	while(ros::ok()){

		ros::spinOnce();

		if(ser.available()){
			ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String result;
			result.data = ser.read(ser.available());
			ROS_INFO_STREAM("Read: " << result.data);
			read_pub.publish(result);
		}
		loop_rate.sleep();

	}
}



