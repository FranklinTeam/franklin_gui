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
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	pub_cmd_1 = n.advertise<geometry_msgs::Pose2D>("leonardo/destination", 100);
	pub_cmd_2 = n.advertise<geometry_msgs::Pose2D>("raphael/destination", 100);
	pub_cmd_3 = n.advertise<geometry_msgs::Pose2D>("donatello/destination", 100);
	pub_cmd_4 = n.advertise<geometry_msgs::Pose2D>("michelangelo/destination", 100);
	pub_stop_1 = n.advertise<std_msgs::Bool>("leonardo/destination/stop", 100);
	pub_stop_2 = n.advertise<std_msgs::Bool>("raphael/destination/stop", 100);
	pub_stop_3 = n.advertise<std_msgs::Bool>("donatello/destination/stop", 100);
	pub_stop_4 = n.advertise<std_msgs::Bool>("michelangelo/destination/stop", 100);

	sub_info_dest = n.subscribe("f_info_dest", 100, &QNode::info_dest_Callback, this);
	sub_odom_1 = n.subscribe("leonardo/odom", 100, &QNode::odom_Callback1, this);
	sub_odom_2 = n.subscribe("raphael/odom", 100, &QNode::odom_Callback2, this);
	sub_odom_3 = n.subscribe("donatello/odom", 100, &QNode::odom_Callback3, this);
	sub_odom_4 = n.subscribe("michelangelo/odom", 100, &QNode::odom_Callback4, this);

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
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	pub_cmd_1 = n.advertise<geometry_msgs::Pose2D>("leonardo/destination", 100);
	pub_cmd_2 = n.advertise<geometry_msgs::Pose2D>("raphael/destination", 100);
	pub_cmd_3 = n.advertise<geometry_msgs::Pose2D>("donatello/destination", 100);
	pub_cmd_4 = n.advertise<geometry_msgs::Pose2D>("michelangelo/destination", 100);
	pub_stop_1 = n.advertise<std_msgs::Bool>("leonardo/destination/stop", 100);
	pub_stop_2 = n.advertise<std_msgs::Bool>("raphael/destination/stop", 100);
	pub_stop_3 = n.advertise<std_msgs::Bool>("donatello/destination/stop", 100);
	pub_stop_4 = n.advertise<std_msgs::Bool>("michelangelo/destination/stop", 100);

	sub_info_dest = n.subscribe("f_info_dest", 100, &QNode::info_dest_Callback, this); //&franklin_gui::QNode::info_dest_Callback, this
	sub_odom_1 = n.subscribe("leonardo/odom", 100, &QNode::odom_Callback1, this);
	sub_odom_2 = n.subscribe("raphael/odom", 100, &QNode::odom_Callback2, this);
	sub_odom_3 = n.subscribe("donatello/odom", 100, &QNode::odom_Callback3, this);
	sub_odom_4 = n.subscribe("michelangelo/odom", 100, &QNode::odom_Callback4, this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(100);
	int count = 0;
	this->all = true;
	this->row = 0;
	while ( ros::ok() ) {

                if(count == 0){
                    //stop at start
                    ROS_INFO("Stop at start");
										sendStop(true, "leonardo");
										sendStop(true, "raphael");
										sendStop(true, "donatello");
										sendStop(true, "michelangelo");
                }
								++count;
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
	if(pNamespace.compare("leonardo") == 0){
		pub_dest = pub_cmd_1;
	}else if(pNamespace.compare("raphael") == 0){
		pub_dest = pub_cmd_2;
	}else if(pNamespace.compare("donatello") == 0){
		pub_dest = pub_cmd_3;
	}else if(pNamespace.compare("michelangelo") == 0){
		pub_dest = pub_cmd_4;
	}

	if(pub_dest != NULL){

  	sendStop(false, pNamespace);

		geometry_msgs::Pose2D pose2D;
		pose2D.x = pX;
		pose2D.y = pY;
		pose2D.theta = pT;
		pub_dest.publish(pose2D);
  }

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

void QNode::info_dest_Callback(const std_msgs::Float32 msg){
	ROS_INFO("PROGRESS UPDATE RECEIVE");
	this->progressData = (int) (msg.data*100);
	Q_EMIT progressDataS();
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

void QNode::stopOne(bool b, std::string pNamespace){
	if(pNamespace.compare("leonardo") == 0){
		pub_stop = pub_stop_1;
	}else if(pNamespace.compare("raphael") == 0){
		pub_stop = pub_stop_2;
	}else if(pNamespace.compare("donatello") == 0){
		pub_stop = pub_stop_3;
	}else if(pNamespace.compare("michelangelo") == 0){
		pub_stop = pub_stop_4;
	}
	if(pub_stop != NULL){
		std_msgs::Bool msg;
		msg.data = b;
		pub_stop.publish(msg);
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
