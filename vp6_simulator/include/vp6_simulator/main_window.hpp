/**
 * @file /include/vp6_simulator/main_window.hpp
 *
 * @brief Qt based gui for vp6_simulator.
 *
 * @date November 2010
 **/
#ifndef vp6_simulator_MAIN_WINDOW_H
#define vp6_simulator_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace vp6_simulator {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event); // Overloaded function
	
	ros::Publisher  jointStatePub;

	ros::Publisher display_publisher ;

	
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_btn_ros_clicked(bool check );
	void on_btn_add_clicked(bool check );
	void on_btn_clear_clicked(bool check);
	void on_btn_path_clicked(bool check);
	void on_btn_load_clicked(bool check);

	void on_btn_save_ik_clicked(bool check);

	void on_btn_grip_open_clicked(bool check);
	void on_btn_grip_close_clicked(bool check);

	void on_btn_jnt_cmd_clicked(bool check);


	void on_btn_sim_clicked(bool check);
	
	void computeJacobian();

    /******************************************
    ** Manual connections
    *******************************************/
    

private:
	Ui::MainWindowDesign ui;
	ros::AsyncSpinner * spinner;
	ros::NodeHandle * handle;

	//code for gazebo Matlab controller.

	ros::Publisher j1Command;
	ros::Publisher j2Command;
	ros::Publisher j3Command;
	ros::Publisher j4Command;
	ros::Publisher j5Command;
	ros::Publisher j6Command;

	ros::Subscriber velocitySubscriber; //subscribe to velocity command.
	ros::Subscriber matlabSubscriber; //subscribe to matlab joint positions

	float zvelocity;
	void velocityCallback(std_msgs::Float32::ConstPtr &msg);
	void matlabCallback(const geometry_msgs::TwistPtr &msg);

	//

	QNode qnode;

	std::vector<geometry_msgs::Pose> pointList;
	moveit::planning_interface::MoveGroup * group;
	moveit::planning_interface::MoveGroup * gripper_group;

	ros::ServiceClient * ik_service_client; // Service client for inverse kinematics solving.

 	void init_ros(int argc, char**argv);
 	void doPositionPlan(double x, double y, double z);
 	void printTrajectoryInformation(moveit_msgs::RobotTrajectory traj);
 	void doPathPlan();
 	//adds velocity interpolation and execute cartesian trajectory.
 	void executeCartesianPlan(moveit_msgs::RobotTrajectory traj);

 	void setGripperJointValues(double theta);

 	void simulateTrajectory();
 	void executeTrajectory(moveit_msgs::RobotTrajectory trajectory);

 	void initializeGazeboLink();
 	void updateJointValues(Eigen::MatrixXd joints, std::vector<double> current);
 	void updateJointValues(std::vector<double> current);

};

}  // namespace vp6_simulator

#endif // vp6_simulator_MAIN_WINDOW_H
