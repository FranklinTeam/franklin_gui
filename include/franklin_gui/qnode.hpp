/**
 * @file /include/franklin_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef franklin_gui_QNODE_HPP_
#define franklin_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <franklin/CmdRobot.h>
#include <franklin/PackageRobot.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace franklin_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void sendCMD(std::string name, std::string target, double x, double y, bool toOther, bool receive);
  void sendTargetPos(double pX, double pY, double pT, std::string pNamespace);
  void sendOne(double pX, double pY, double pT, std::string pNamespace);
  void sendStop(bool b, std::string pNamespace);
  void stopOne(bool b, std::string pNamespace);
  void sendStopAll(bool b);
  void info_dest_Callback(const std_msgs::Float32 msg);
  void odom_Callback1(const nav_msgs::Odometry odom);
  void odom_Callback2(const nav_msgs::Odometry odom);
  void odom_Callback3(const nav_msgs::Odometry odom);
  void odom_Callback4(const nav_msgs::Odometry odom);
  void state_Callback1(const franklin::PackageRobot msg);
  void state_Callback2(const franklin::PackageRobot msg);
  void state_Callback3(const franklin::PackageRobot msg);
  void state_Callback4(const franklin::PackageRobot msg);
	int progressData;
  float odom_X, odom_Y,odom_T;
  std::string state1, state2, state3, state4;
  bool pack1, pack2, pack3, pack4;
  bool all;
  int row;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

public Q_SLOTS:
  void onAllChanged(int check);
  void onSelectionChanged(int currentRow);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void progressDataS();
  void odomS();
  void stateChanged();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  QStringListModel logging_model;

  ros::Subscriber sub_odom_1;
  ros::Subscriber sub_odom_2;
  ros::Subscriber sub_odom_3;
  ros::Subscriber sub_odom_4;
  ros::Subscriber sub_state_1;
  ros::Subscriber sub_state_2;
  ros::Subscriber sub_state_3;
  ros::Subscriber sub_state_4;
  ros::Publisher pub_cmd_1;
  ros::Publisher pub_cmd_2;
  ros::Publisher pub_cmd_3;
  ros::Publisher pub_cmd_4;

  //MoveBaseClient ac1;
  //MoveBaseClient ac2;
  //MoveBaseClient ac3;
  //MoveBaseClient ac4;


};

}  // namespace franklin_gui

#endif /* franklin_gui_QNODE_HPP_ */
