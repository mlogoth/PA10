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
#include <geometry_msgs/Twist.h>
#include <pa10_remote/pa10remote.h>

#include <cmath>


#define PI acos(-1)
using namespace std;
using namespace Eigen;

//global var
double qdot[7];

#define RATE 50.0

double anglewrap(double x)
{
	double y = x;
	if (x>PI)
		y = x-2.0*PI;
	else if (x<-M_PI)
		y = x +2.0*PI;
		
	return y;
}

#define SATUR(x,minx,maxx) max(min(x,maxx),minx)

class pa10 : public hardware_interface::RobotHW
{
public:
  ros::Publisher pub; //publisher to pub the f/t mesurements
  ros::NodeHandle n_; 
  
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
   

  }
  
  ////////////////////////////
  /* PA10 RELATED FUNCTIONS */
  ////////////////////////////
  
  
  /* INITIALIZATION */
  void pa10CommInit()
  {

     int status;
     struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
     struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.

 
     memset(&host_info, 0, sizeof host_info);

     std::cout << "Setting up the structs..."  << std::endl;

     host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
     host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

     //status = getaddrinfo("147.102.51.71", "4534", &host_info, &host_info_list);
     status = getaddrinfo("192.168.1.71", "4534", &host_info, &host_info_list);
     
     if (status != 0){  
         std::cout << "getaddrinfo error" << gai_strerror(status) ;
         exit(0);
     }

     std::cout << "Creating a socket..."  << std::endl;
     
     //the socket discriptor
     socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
 
     if (socketfd == -1){
         std::cout << "socket error\n" ;
         exit(0);
     }

     std::cout << "Connecting..."  << std::endl;
     status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);

     if (status == -1){
         std::cout << "connect error\n" ;
         exit(0);
      }

     std::cout << "Connection with PA 10 PC Established !!!";

     std::cout << "sending message..."  << std::endl;
     char *msg = "I020"; 
     int len;
     ssize_t bytes_sent;
     len = strlen(msg);

     bytes_sent = send(socketfd, msg, len, 0);

     //return socketfd;
	}
  
  
  /////////////////////////
  /* GET PA10 JOINT DATA */
  /////////////////////////
  void getPA10data()
  {
     float pa10data[7], pa10vel[7],pa10eff[7];

     std::cout << "Waiting to receive data..."  << std::endl;
     ssize_t bytes_received2;
     char incomming_data_buffer2[1000];
     //return the num of bytes reads from the socket
     bytes_received2 = recv(socketfd,incomming_data_buffer2,1000, 0);
     
     if (bytes_received2 == 0){
         std::cout << "host shut down." << std::endl ;
         exit(0);
     }

     if (bytes_received2 == -1){
         std::cout << "receive error!" << std::endl ;
         exit(0);
     }
         


     std::cout << bytes_received2 << " bytes received :" << std::endl ;
     std::cout << incomming_data_buffer2 << std::endl;
     incomming_data_buffer2[bytes_received2 -2] = '\0';

     std::cout << "Bypassed Get Angles !!!" << "\n";

     char *mstring;
     mstring = (char *)malloc(20 * sizeof(char));

     char *mstart,*mend;

     if(incomming_data_buffer2[0]=='I')
         mstart = &(incomming_data_buffer2[7]);
     else
         mstart = &(incomming_data_buffer2[1]);
         
     //Read the position values
     for (int count=0; count<7; count++){
         //returns a pointer to the first character 'a' in mstart
         mend = strchr(mstart,97);
         // copy the first mend-mstart chars of mstart to mstring
         strncpy(mstring,mstart,(int)(mend - mstart));
         //cout << "OK" << "\n";
         strcat(mstring,"\0");
         // ascii to float
         pa10data[count]=atof(mstring);
         mstart=mend+1;
         }
     
     // Read the velocity values
     mstart = mstart + 1; // Avoid 'V' character
     
     for (int count=0; count<7; count++){
     		 //returns a pointer to the first character 'a' in mstart
         mend = strchr(mstart,97);
         // copy the first mend-mstart chars of mstart to mstring
         strncpy(mstring,mstart,(int)(mend - mstart));
         //cout << "OK" << "\n";
         strcat(mstring,"\0");
         // ascii to float
         pa10vel[count]=atof(mstring);
         mstart=mend+1;
     }
     
     // Read the effort values
     mstart = mstart + 1; // Avoid 'T' character
     
     for (int count=0; count<7; count++){
     		 //returns a pointer to the first character 'a' in mstart
         mend = strchr(mstart,97);
         // copy the first mend-mstart chars of mstart to mstring
         strncpy(mstring,mstart,(int)(mend - mstart));
         //cout << "OK" << "\n";
         strcat(mstring,"\0");
         // ascii to float
         pa10eff[count]=atof(mstring);
         mstart=mend+1;
     }


     std::cout << "PA10: " << " q1: " << pa10data[0] << " q2: " << pa10data[1]
                           << " q3: " << pa10data[2] << " q4: " << pa10data[3]
                           << " q5: " << pa10data[4] << " q6: " << pa10data[5]
                           << " q7: " << pa10data[6] << "\n";

     for (int i = 0; i < 7; i++){
         // fill position pos[],vel[] and eff[] with joints state
         pos[i] = pa10data[i]*PI/180.0;
         
        
         
         vel[i] = pa10vel[i]*PI/180.0;
         
         cout << "Jvel[" << i <<  "]= " <<  vel[i] << endl;

         eff[i] = pa10eff[i];
         
         // if the hdw interface starts running
         if (init_cmd_prev) 
         	cmd_prev[i] =  pos[i];
     }
    
     init_cmd_prev = false;
	}

  
  ///////////////////////////////////
  /* SEND TO PA10 THE DESIRED DATA */
  ///////////////////////////////////
  void setPA10data(double* qdot)
  {
  
  	 //float limit[7] = {175.0, 92.0, 172.0, 135.0, 253.0, 163.0, 253.0};
  	 
  	 cout << "Desired Velocities" << endl;
  	 for (int i=0;i<9;i++)
  	 	cout << qdot[i] << ", ";
  	 cout << "" << endl;

     float DOF1 = qdot[0]*180.0/PI;
     float DOF2 = qdot[1]*180.0/PI;
     float DOF3 = qdot[2]*180.0/PI;
     float DOF4 = qdot[3]*180.0/PI;
     float DOF5 = qdot[4]*180.0/PI;
     float DOF6 = qdot[5]*180.0/PI;
     float DOF7 = qdot[6]*180.0/PI;


     std::string result;
     std::stringstream convert;
     convert<<"M"<<DOF1<<"a"<<DOF2<<"a"<<DOF3<<"a"<<DOF4<<"a"<<DOF5<<"a"<<DOF6<<"a"<<DOF7<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
     result=convert.str();

     //std::cout << "Result:" << result << std::endl;

     char *msg2 = new char[result.size()+1];
     msg2[result.size()]=0;
     memcpy(msg2,result.c_str(),result.size());
     int len2;
     ssize_t bytes_sent2;
     len2 = strlen(msg2);

     bytes_sent2 = send(socketfd, msg2, len2, 0);

     std::cout << "Send Message !!!" << "\n";
	}

  
  void updateQdot(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
     int i = 0;
     // print all the remaining numbers
     for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
		 {
			qdot[i] = *it;
			i++;
		 }
	}
	
 	
 	///////////////////
 	/* Smooth Filter */
  ///////////////////
  
  /* Smoothed Velocities */
  void smooth_filter(float smooth=0.7)
  {
  	for (int i=0;i<7;i++) {
  		sm_vel[i] = sm_vel[i]+smooth*(cmd_vel[i]-sm_vel[i]);	
	    cmd_vel[i] = sm_vel[i];
	    
	    //cout << cmd_vel[i] << endl;
	  }
  }
  
  
  
  ////////////////////////
  /* Set Controller Type*/
  ////////////////////////
  void setControllerType(std::string cntr_type)
  {
  	//define the type of the controller --> the command type
  	controller_type = cntr_type;
  }
  
  
  ///////////////////////////////////////////////////////////////////
  /* Fuction which computes joint velocities using joint positions */
	///////////////////////////////////////////////////////////////////
  void postovel(double cmd_[9])
  {
  	for (int i=0;i<7;++i) 
			{
				//compute velocity commands
			  cmd_vel[i]=anglewrap(cmd_[i]-cmd_prev[i])*RATE;
				//update previous commands
				cmd_prev[i] = cmd_[i];
			}
	}	


	
	///////////////////
	/* Read Function */
	///////////////////
	void read()
	{	
		// Read position velocity torque from the robot
		pa10::getPA10data();		
	}
	
	///////////////////
  /* Write Function*/
  ///////////////////
  void write()
  {
  	
  	// if you want velocity commands
		if (controller_type=="VelocityJointInterface") 
		{
			// send the commands to pa10
  		pa10::setPA10data(cmd);
		}
  	
  	// if you want velocity commands
		else if (controller_type=="PositionJointInterface") 
		{
			//convert position commands to velocities
  		pa10::postovel(cmd);
  		//smoothed velocities
  		pa10::smooth_filter(1.0);
  		// send the commands to pa10
  		
  		
  		for (int i=0;i<7;i++)
  			cout << "V[" << i <<  "]= " <<  cmd[i] << endl;
			
			double error[7];
			
			error[5]=anglewrap(pos[5]-cmd[5]);
			error[6]=anglewrap(pos[6]-cmd[6]);
			
			errorI[5] +=(1.0/RATE)*error[5];
			errorI[6] +=(1.0/RATE)*error[6];
			
			errorI[5] = SATUR(errorI[5],-0.2,0.2);
			errorI[6] = SATUR(errorI[6],-0.2,0.2);
			
			
  		cmd_vel[5]=(-0.06*error[5] - errorI[5]*0.0001)*180.0/PI;
  		cmd_vel[6]=(-0.06*error[6] - errorI[6]*0.0001)*180.0/PI;
  		
  		cout << cmd_vel[5] << endl;
  		cout << cmd_vel[6] << endl;
  		
  		//cmd_vel[6]=0.0;
  		
  		pa10::setPA10data(cmd_vel);
		}
  	
  	else 
  	{
  		std::cout << "No Supported Controller!" << std::endl ;
      exit(0);
  	}
  }
	
	
	
  ros::Time get_time() { return ros::Time::now(); } ;
  ros::Duration get_period() { ros::Duration(1.0/RATE);};

	void initvars() {
		for (int i=0;i<9;i++)
		{
			cmd[i]=0.0;
			cmd_prev[i] = 0.0;
			cmd_vel[i]=0.0;
			pos[i] = 0.0;
			vel[i] = 0.0;
			eff[i] = 0.0;
			sm_vel[i] =0.0;
			errorI[i]=0.0;
		}
		init_cmd_prev = true;
		
	}

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  ros::Publisher pub_smoothed_velocities;
  double cmd[9];
  double cmd_prev[9];
  double cmd_vel[9],sm_vel[9];
  double pos[9];
  double vel[9];
  double eff[9];
  std::string controller_type;
  bool VelControllWithPoseCommand, init_cmd_prev;
  int socketfd ; // The socket descripter
  
  double errorI[9];
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pa10_hardware_interface_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(RATE);

  pa10 robot;
  
  //initialize control manager
  controller_manager::ControllerManager cm(&robot,nh);
  
  //define the type of the controller --> the command type
  robot.setControllerType("VelocityJointInterface");
  
  //Initialize Communication with PA10
  robot.pa10CommInit();
	
	//Initialize Variables
	robot.initvars();
	
	ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Duration dur = robot.get_period();
  
  while (ros::ok())
  {
  	//time loop starts
  	ros::Time start =ros::Time::now(); 
    
    // read robot data
    robot.read();
    
    // update data
    cm.update(start, dur);
    
    //send data to the robot
    robot.write();
    
    //ros::spinOnce();
	  //loop_rate.sleep();
	  
	  loop_rate.sleep();
    
    // time loop ends  
    dur = robot.get_time()-start;
    
    //print dt
    printf("Loop dt:%lf\n", dur.toSec());
    

  }
  
 return 0;
}
