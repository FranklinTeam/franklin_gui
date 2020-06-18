/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "../include/franklin_gui/qnode.hpp"
#include <franklin/CmdRobot.h>
#include <franklin/PackageRobot.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace franklin_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void test(const std_msgs::Float32 msg){
	//int progressDataTest = (int) (msg.data*100);
	ROS_INFO("Test ");

}

bool QNode::init() {
	ros::init(init_argc,init_argv,"franklin_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	pub_cmd_1 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb1", 100);
	pub_cmd_2 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb2", 100);
	pub_cmd_3 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb3", 100);
	pub_cmd_4 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb4", 100);

	sub_state_1 = n.subscribe("/COM_tb1_to_ctrl", 100, &QNode::state_Callback1, this);
	sub_state_2 = n.subscribe("/COM_tb2_to_ctrl", 100, &QNode::state_Callback2, this);
	sub_state_3 = n.subscribe("/COM_tb3_to_ctrl", 100, &QNode::state_Callback3, this);
	sub_state_4 = n.subscribe("/COM_tb4_to_ctrl", 100, &QNode::state_Callback4, this);

	sub_odom_1 = n.subscribe("tb1/odom", 100, &QNode::odom_Callback1, this);
	sub_odom_2 = n.subscribe("tb2/odom", 100, &QNode::odom_Callback2, this);
	sub_odom_3 = n.subscribe("tb3/odom", 100, &QNode::odom_Callback3, this);
	sub_odom_4 = n.subscribe("tb4/odom", 100, &QNode::odom_Callback4, this);

	//tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("/" + robot_name + "/move_base", true);

  //wait for the action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
    //ROS_INFO("Waiting for the move_base action server to come up");

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"franklin_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	pub_cmd_1 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb1", 100);
	pub_cmd_2 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb2", 100);
	pub_cmd_3 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb3", 100);
	pub_cmd_4 = n.advertise<franklin::CmdRobot>("/COM_ctrl_to_tb4", 100);

	sub_state_1 = n.subscribe("/COM_tb1_to_ctrl", 100, &QNode::state_Callback1, this);
	sub_state_2 = n.subscribe("/COM_tb2_to_ctrl", 100, &QNode::state_Callback2, this);
	sub_state_3 = n.subscribe("/COM_tb3_to_ctrl", 100, &QNode::state_Callback3, this);
	sub_state_4 = n.subscribe("/COM_tb4_to_ctrl", 100, &QNode::state_Callback4, this);

	sub_odom_1 = n.subscribe("tb1/odom", 100, &QNode::odom_Callback1, this);
	sub_odom_2 = n.subscribe("tb2/odom", 100, &QNode::odom_Callback2, this);
	sub_odom_3 = n.subscribe("tb3/odom", 100, &QNode::odom_Callback3, this);
	sub_odom_4 = n.subscribe("tb4/odom", 100, &QNode::odom_Callback4, this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(100);
	int count = 0;
	this->all = true;
	this->row = 0;
	while ( ros::ok() ) {


		ros::spinOnce();
                loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendOne(double pX, double pY, double pT, std::string pNamespace){



}

void QNode::sendTargetPos(double pX, double pY, double pT, std::string pNamespace){
	ROS_INFO("INFO POS SENT !");

	if(!all){
		sendOne(pX, pY, pT, pNamespace);
	}else{
		sendOne(pX-0.5, pY, pT, "leonardo");
		sendOne(pX+0.5, pY, pT, "raphael");
		sendOne(pX, pY-0.5, pT, "donatello");
		sendOne(pX, pY+0.5, pT, "michelangelo");
	}
}

void QNode::onSelectionChanged(int currentRow){
	this->row = currentRow;
}

void QNode::odom_Callback1(const nav_msgs::Odometry odom){
		if(this->row == 0){
			this->odom_X = odom.pose.pose.position.x;
			this->odom_Y = odom.pose.pose.position.y;
		/*
			// quaternion to RPY conversion
			tf::Quaternion q(
					odom.pose.pose.orientation.x,
					odom.pose.pose.orientation.y,
					odom.pose.pose.orientation.z,
					odom.pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
		*/
			// angular position
			//this->odom_T = yaw;
			this->odom_T = odom.pose.pose.orientation.z;


			Q_EMIT odomS();
	}
}

void QNode::odom_Callback2(const nav_msgs::Odometry odom){
	if(this->row == 1){
		this->odom_X = odom.pose.pose.position.x;
		this->odom_Y = odom.pose.pose.position.y;
		this->odom_T = odom.pose.pose.orientation.z;

		Q_EMIT odomS();
 }
}

void QNode::odom_Callback3(const nav_msgs::Odometry odom){
	if(this->row == 2){
		this->odom_X = odom.pose.pose.position.x;
		this->odom_Y = odom.pose.pose.position.y;
		this->odom_T = odom.pose.pose.orientation.z;

		Q_EMIT odomS();
 }
}

void QNode::odom_Callback4(const nav_msgs::Odometry odom){
	if(this->row == 3){
		this->odom_X = odom.pose.pose.position.x;
		this->odom_Y = odom.pose.pose.position.y;
		this->odom_T = odom.pose.pose.orientation.z;

		Q_EMIT odomS();
 }
}

void QNode::state_Callback1(const franklin::PackageRobot msg){
	this->state1 = msg.state;
	this->pack1 = msg.package;
	Q_EMIT stateChanged();
}
void QNode::state_Callback2(const franklin::PackageRobot msg){
	this->state2 = msg.state;
	this->pack2 = msg.package;
	Q_EMIT stateChanged();
}
void QNode::state_Callback3(const franklin::PackageRobot msg){
	this->state3 = msg.state;
	this->pack3 = msg.package;
	Q_EMIT stateChanged();
}
void QNode::state_Callback4(const franklin::PackageRobot msg){
	this->state4 = msg.state;
	this->pack4 = msg.package;
	Q_EMIT stateChanged();
}

void QNode::sendCMD(std::string name, std::string target, double x, double y, bool toOther, bool receive){
	ROS_INFO("INFO CMD SENT !");
	franklin::CmdRobot msg;
	msg.reception = receive;
	msg.from_robot = toOther;
	msg.robot_name = target;
	msg.obj_x = x;
	msg.obj_y = y;

	if(name.compare("tb1") == 0){
		pub_cmd_1.publish(msg);
	}else if(name.compare("tb2") == 0){
		pub_cmd_2.publish(msg);
	}else if(name.compare("tb3") == 0){
		pub_cmd_3.publish(msg);
	}else if(name.compare("tb4") == 0){
		pub_cmd_4.publish(msg);
	}

}

void QNode::stopOne(bool b, std::string pNamespace){
	if(pNamespace.compare("leonardo") == 0){

	}else if(pNamespace.compare("raphael") == 0){

	}else if(pNamespace.compare("donatello") == 0){

	}else if(pNamespace.compare("michelangelo") == 0){

	}

}

void QNode::sendStop(bool b, std::string pNamespace){
	ROS_INFO("INFO STOP SENT !");

	if(!all){
		stopOne(b, pNamespace);
	}else{
		this->sendStopAll(b);
	}

}

void QNode::sendStopAll(bool b){
	stopOne(b, "leonardo");
	stopOne(b, "raphael");
	stopOne(b, "donatello");
	stopOne(b, "michelangelo");
}

void QNode::onAllChanged(int check){
	if(check == 0){
		this->all = false;
		ROS_INFO("New check state : FALSE");
	}else if(check == 2){
		this->all = true;
		ROS_INFO("New check state : TRUE");
	}
}

}  // namespace franklin_gui
