#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "Acrobot_PID.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>

float ActQ[2] = {0,0};
float ActQdot[2] = {0,0};

void listen_joint_state_callback(const sensor_msgs::JointState & state)
{
	ActQ[0] = state.position[0] * Rad2Deg;
	ActQ[1] = state.position[1] * Rad2Deg;
	ActQdot[0] = state.velocity[0] * Rad2Deg;
	ActQdot[1] = state.velocity[1] * Rad2Deg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "acrobot_control");

	ros::NodeHandle nhp;
	
	ros::Publisher TorqueCmd1_pub = nhp.advertise<std_msgs::Float64>("/acrobot/joint1_torque_controller/command", 100);
	std_msgs::Float64 TorqueCmd1;

	ros::Publisher TorqueCmd2_pub = nhp.advertise<std_msgs::Float64>("/acrobot/joint2_torque_controller/command", 100);
	std_msgs::Float64 TorqueCmd2;

	ros::Publisher PosCmd1_pub = nhp.advertise<std_msgs::Float64>("PosCmd1_msg", 100);
	std_msgs::Float64 PosCmd1;

	ros::Publisher PosCmd2_pub = nhp.advertise<std_msgs::Float64>("PosCmd2_msg", 100);
	std_msgs::Float64 PosCmd2;

	ros::Publisher PosAct1_pub = nhp.advertise<std_msgs::Float64>("PosAct1_msg", 100);
	std_msgs::Float64 PosAct1;

	ros::Publisher PosAct2_pub = nhp.advertise<std_msgs::Float64>("PosAct2_msg", 100);
	std_msgs::Float64 PosAct2;

	ros::Publisher VelCmd1_pub = nhp.advertise<std_msgs::Float64>("VelCmd1_msg", 100);
	std_msgs::Float64 VelCmd1;

	ros::Publisher VelCmd2_pub = nhp.advertise<std_msgs::Float64>("VelCmd2_msg", 100);
	std_msgs::Float64 VelCmd2;

	ros::Publisher VelAct1_pub = nhp.advertise<std_msgs::Float64>("VelAct1_msg", 100);
	std_msgs::Float64 VelAct1;

	ros::Publisher VelAct2_pub = nhp.advertise<std_msgs::Float64>("VelAct2_msg", 100);
	std_msgs::Float64 VelAct2;

	ros::Rate rate(1000);
	
	Acrobot acrobot;

	acrobot.InitParameter();

	ros::NodeHandle nhs;
	ros::Subscriber sub = nhs.subscribe("/acrobot/joint_states", 100, listen_joint_state_callback);

	while (ros::ok())
	{		

		acrobot.Trajectory(ActQ, ActQdot);		

		TorqueCmd1.data = acrobot.calcControlTorque1();	
		TorqueCmd1_pub.publish(TorqueCmd1);

		TorqueCmd2.data = acrobot.calcControlTorque2();	
		TorqueCmd2_pub.publish(TorqueCmd2);

		PosCmd1.data = acrobot.getPosCmd1();
		PosCmd1_pub.publish(PosCmd1);

		PosCmd2.data = acrobot.getPosCmd2();
		PosCmd2_pub.publish(PosCmd2);

		PosAct1.data = acrobot.getPosAct1();
		PosAct1_pub.publish(PosAct1);

		PosAct2.data = acrobot.getPosAct2();
		PosAct2_pub.publish(PosAct2);

		VelCmd1.data = acrobot.getVelCmd1();
		VelCmd1_pub.publish(VelCmd1);

		VelCmd2.data = acrobot.getVelCmd2();
		VelCmd2_pub.publish(VelCmd2);

		VelAct1.data = acrobot.getVelAct1();
		VelAct1_pub.publish(VelAct1);

		VelAct2.data = acrobot.getVelAct2();
		VelAct2_pub.publish(VelAct2);

		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}
