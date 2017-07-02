#include <vector>
#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#include "std_msgs/Float64MultiArray.h"

#include <cmath>


#define RATE 25.0//1.0/0.04

double
anglewrap(const double x)
{
	double y=x;

	if (x > M_PI)
		y=x-2.0*M_PI;

	else if (x<-M_PI)
		y=x+2.0*M_PI;

	return y;
}

double diffAngle(double x, double y)
{
	double diff;

	diff = fmod(y-x, 2*M_PI);
	if (diff < 0 )  diff  = diff + 2.0*M_PI;
	if (diff > M_PI) diff  = diff - 2.0*M_PI;

	return (-diff);
}


class pa10 : public hardware_interface::RobotHW
{
public:
	ros::Subscriber sub;
	ros::NodeHandle n_;
	ros::Publisher pub_smoothed_velocities, pub_cmd;


public:
	pa10()
{
		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle_1("S1", &pos[0], &vel[0], &eff[0]);
		jnt_state_interface.registerHandle(state_handle_1);

		hardware_interface::JointStateHandle state_handle_2("S2", &pos[1], &vel[1], &eff[1]);
		jnt_state_interface.registerHandle(state_handle_2);

		hardware_interface::JointStateHandle state_handle_3("E1", &pos[2], &vel[2], &eff[2]);
		jnt_state_interface.registerHandle(state_handle_3);

		hardware_interface::JointStateHandle state_handle_4("E2", &pos[3], &vel[3], &eff[3]);
		jnt_state_interface.registerHandle(state_handle_4);

		hardware_interface::JointStateHandle state_handle_5("W1", &pos[4], &vel[4], &eff[4]);
		jnt_state_interface.registerHandle(state_handle_5);

		hardware_interface::JointStateHandle state_handle_6("W2", &pos[5], &vel[5], &eff[5]);
		jnt_state_interface.registerHandle(state_handle_6);

		hardware_interface::JointStateHandle state_handle_7("W3", &pos[6], &vel[6], &eff[6]);
		jnt_state_interface.registerHandle(state_handle_7);

		hardware_interface::JointStateHandle state_handle_8("finger_joint_1", &pos[7], &vel[7], &eff[7]);
		jnt_state_interface.registerHandle(state_handle_8);

		hardware_interface::JointStateHandle state_handle_9("finger_joint_2", &pos[8], &vel[8], &eff[8]);
		jnt_state_interface.registerHandle(state_handle_9);

		registerInterface(&jnt_state_interface);

		// connect and register the joint position interface
		hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("S1"), &cmd[0]);
		jnt_vel_interface.registerHandle(vel_handle_1);

		hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("S2"), &cmd[1]);
		jnt_vel_interface.registerHandle(vel_handle_2);

		hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("E1"), &cmd[2]);
		jnt_vel_interface.registerHandle(vel_handle_3);

		hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("E2"), &cmd[3]);
		jnt_vel_interface.registerHandle(vel_handle_4);

		hardware_interface::JointHandle vel_handle_5(jnt_state_interface.getHandle("W1"), &cmd[4]);
		jnt_vel_interface.registerHandle(vel_handle_5);

		hardware_interface::JointHandle vel_handle_6(jnt_state_interface.getHandle("W2"), &cmd[5]);
		jnt_vel_interface.registerHandle(vel_handle_6);

		hardware_interface::JointHandle vel_handle_7(jnt_state_interface.getHandle("W3"), &cmd[6]);
		jnt_vel_interface.registerHandle(vel_handle_7);

		hardware_interface::JointHandle vel_handle_8(jnt_state_interface.getHandle("finger_joint_1"), &cmd[7]);
		jnt_vel_interface.registerHandle(vel_handle_8);

		hardware_interface::JointHandle vel_handle_9(jnt_state_interface.getHandle("finger_joint_2"), &cmd[8]);
		jnt_vel_interface.registerHandle(vel_handle_9);


		registerInterface(&jnt_vel_interface);

		n_ = ros::NodeHandle();
		//publisher for smoothed velocities
		//pub_smoothed_velocities =  n_.advertise<std_msgs::Float64MultiArray>("/smoothed_velocities", 10);
		//pub_cmd = n_.advertise<sensor_msgs::JointState >("/cmd", 10);

		mstart = ros::Time::now();


}


	/* Smoothed Velocities */
	void smooth_filter(float smooth=0.7)
	{
		//std_msgs::MultiArrayDimension dim;
		//dim.size = 9;
		//dim.stride = 9;

		//Publish Joint positions
		//std_msgs::Float64MultiArray pub_array;

		//pub_array.layout.dim.push_back(dim);
		//pub_array.layout.data_offset = 0;

		for (int i=0;i<9;i++) {
			sm_vel[i] = sm_vel[i]+smooth*(vel[i]-sm_vel[i]);
			//pub_array.data.push_back(sm_vel[i]);
		}
		//publish smoothed
		//pub_smoothed_velocities.publish(pub_array);
	}

	///////////////////////////////////////////////////////////////////
	/* Fuction which computes joint positions using joint velocities */
	///////////////////////////////////////////////////////////////////
	void veltopos(double cmd_[9])
	{
		//double cmd_p[9];
		for (int i=0;i<9;++i)
		{
			//compute velocity commands
			pos[i]=cmd_[i]/RATE+cmd_prev[i];
			//update previous commands
			cmd_prev[i] = cmd_[i];
		}
		//return the computed velocities
		//return cmd_p;
	}


	///////////////////////////////////////////////////////////////////
	/* Fuction which computes joint velocities using joint positions */
	///////////////////////////////////////////////////////////////////
	void postovel(double cmd_[9])
	{
		for (int i=0;i<9;++i)
		{
			//compute velocity commands
			//cmd[i]=anglewrap(cmd_[i]);

			vel[i]=diffAngle(cmd_[i],cmd_prev[i])*RATE;
			//update previous commands
			cmd_prev[i] = cmd_[i];
		}
	}	

	///////////////////
	/* Write Function*/
	///////////////////
	void write()
	{

		//std::cout<<"##joints Commands: "<<cmd_vel[0]<<" "<<cmd_vel[1]<<" "<<cmd_vel[2]<<" "<<cmd_vel[3]<<" "<<cmd_vel[4]<<" "<<cmd_vel[5]<<" "<<cmd_vel[6]<<" "<<cmd_vel[7]<<" "<<cmd_vel[8]<<"\n";
		//memcpy(cmd_prev,cmd,sizeof(double)*9);
	}



	///////////////////
	/* Read Function*/
	//////////////////
	void read()
	{	

		//define the type of the controller --> the command type
		controller_type = "VelocityJointInterface";
		VelControllWithPoseCommand = true;

		// if you want velocity commands
		if (controller_type=="VelocityJointInterface") 
		{
			// Do this cause its a fake interface!!
			//pa10::veltopos(cmd);
			//memcpy(vel,cmd,sizeof(double)*9);
			//efford is initialized to zero

			for (int i=0;i<9;++i)
			{
				vel[i]=cmd[i];
				pos[i]=(pos[i]+vel[i]/RATE);
				eff[i]=0.0;
			}

		}

		//if you want effort commands -- NOT IMPLEMENTED YET
		else if (controller_type =="EffortJointInterface")
		{
			for (int i=0;i<9;++i) 
			{
				pos[i]=0.0;
				vel[i]=0.0;
				eff[i]=0.0;
			}
		}

		else if (controller_type =="PositionJointInterface")
		{
			//if (!VelControllWithPoseCommand)
			//{
			// I want to send position commands to the robot
			// give positions direct to joints_states
			//memcpy(pos,cmd,sizeof(double)*9);
			//for (int i=0;i<9;++i)
			//{
			//	vel[i]=0.0;
			//	eff[i]=0.0;
			//}
			//}
			//else
			// I want to send velocities command to the robot
			//{
			pa10::postovel(cmd);
			pa10::smooth_filter(0.8);

			for (int i=0;i<9;++i)
			{
				pos[i]=anglewrap(pos[i]+sm_vel[i]/RATE);
				vel[i]=sm_vel[i];
				eff[i]=0.0;
			}

			//std_msgs::MultiArrayDimension dim;
			//dim.size = 9;
			//dim.stride = 9;

			//Publish Joint positions
			//sensor_msgs::JointState pub_array;

			//pub_array.header.stamp = ros::Time::now();

			//for (int i=0;i<9;i++)
			//	pub_array.position.push_back(cmd[i]);

			//pub_cmd.publish(pub_array);

			//memcpy(pos,cmd,sizeof(double)*9);
			//pa10::postovel(cmd);
			//pa10::smooth_filter(0.8);
			//}

		}


		//std::cout << "Time: " << (ros::Time::now()-mstart).toSec()*1e-9 << std::endl;
		//for (int i=0;i<9;i++)
		//	std::cout << vel[i] << "," ;

		//std::cout << "" << std::endl;
		//std::cout << "" << std::endl;

	}
	ros::Time get_time() { return ros::Time::now();} ;
	ros::Duration get_period() { ros::Duration(1.0/RATE);};

	void initialization_step()
	{
		for (int i=0;i<9;++i)
		{
			vel[i]=0.0;
			pos[i]=0.0;
			eff[i]=0.0;
			cmd[i]=0.0;
			sm_vel[i]=0.0;
		}
	}
private:
	hardware_interface::JointStateInterface jnt_state_interface;
	//hardware_interface::PositionJointInterface jnt_vel_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double cmd[9],cmd_prev[9],cmd_vel[9],sm_vel[9];
	double pos[9] ;
	double vel[9] ;
	double eff[9] ;
	std::string controller_type;
	bool VelControllWithPoseCommand;
	ros::Time mstart;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pa10_hardware_interface");
	ros::NodeHandle nh;
	ros::Rate loop_rate(RATE);
	pa10 robot;
	ros::Time now;

	controller_manager::ControllerManager cm(&robot,nh);

	robot.initialization_step();
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration dur = robot.get_period();

	while (ros::ok())
	{

		now = robot.get_time();
		robot.read();
		cm.update(now,dur);
		robot.write();
		loop_rate.sleep();
		dur = robot.get_time()-now;
	}
	spinner.stop();
}
