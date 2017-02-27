#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "SerialPort.h"
#include <iostream>
#include <string>

int main(int argc, char **argv) 
{
	ros::init(argc,argv,"heartbeat_generator");
	ros::NodeHandle n;
	ros::Publisher hp;
	hp = n.advertise<std_msgs::Bool>("youbot_network_estopper/heartbeat",1);
	
	SerialPort serial_port ("/dev/ttyUSB0");
	bool portOpen = false;
	int counter = 0;

	ros::Rate rate(10); //Hz

	// Open serial port
	try
	{
		ROS_INFO("External button: Opening serial connection.");
		serial_port.Open(SerialPort::BAUD_9600,
						 SerialPort::CHAR_SIZE_8, 
						 SerialPort::PARITY_NONE, 
						 SerialPort::STOP_BITS_1, 
						 SerialPort::FLOW_CONTROL_NONE);
		portOpen = true;

		ROS_INFO("External button: STOP button up and running.");
	}

	catch (SerialPort::OpenFailed exception)
	{
		ROS_ERROR("External button: Failed to open the port");
	}

	catch (SerialPort::AlreadyOpen exception)
	{
		ROS_ERROR("External button: Port is already open");
	}
	
	ROS_INFO("Heartbeat node operational. Publishing heartbeat messages.");
	// Start main loop
	while (n.ok()) 
	{	
		
		std_msgs::Bool b;
		if ((portOpen == true) && serial_port.IsDataAvailable())
		{
			std::string message = serial_port.ReadLine();
			ROS_INFO("Extarnal button: %s", message.c_str());
			b.data=false;
		}
		else
		{
			// Publish the message
			b.data=true;
			hp.publish(b);
			
			if (counter >= 50)
			{
				ROS_INFO("node operational...");
				counter = 0;
			}
			else
				counter++;
		}

		ros::spinOnce();
		rate.sleep();
	}

	// Close connection
	if(serial_port.IsOpen())
	{
		ROS_INFO("Closing serial connection.");
		serial_port.Close();
	}

	ROS_INFO("External button: Serial connection closed.");	
}
