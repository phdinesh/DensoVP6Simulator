/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the Denso VP6242
 * @author : Dinesh Madusanke
 * @date February 2014
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/vp6_simulator/main_window.hpp"
#include <stdio.h>
#include <ros/ros.h>
#include <QTableWidget>
#include <string>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionIKRequest.h> 
#include <moveit_msgs/GetPositionIKResponse.h> 
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <math.h>
#include <fstream>

using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace vp6_simulator {

using namespace Qt;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
 


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
   init_ros(argc, argv);

   initializeGazeboLink();
	
}

MainWindow::~MainWindow() {}


/***ROS functions **/
void MainWindow::init_ros(int argc, char ** argv){

	ros::init(argc, argv,"Manipulator_control_center");
	ros::NodeHandle nh;
	group = new moveit::planning_interface::MoveGroup("manipulator");
	//gripper_group= new moveit::planning_interface::MoveGroup("end_effector");

	spinner = new ros::AsyncSpinner(1);
	spinner->start();

	ros::ServiceClient cl1 = nh.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
	ik_service_client = &cl1;
	handle = & nh;

	//For joint state publisher.
	display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	jointStatePub =  nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

 void MainWindow::on_btn_sim_clicked(bool check){
 	cout << "simulation called. \n";
 	simulateTrajectory();
 }

void MainWindow::on_btn_ros_clicked(bool check ) {


	double x = (ui.txtX)->text().toDouble();
	double y = (ui.txtY)->text().toDouble();
	double z = (ui.txtZ)->text().toDouble();
	if((ui.chk_mp)->isChecked()){
		printf("checked...");

	}
	printf("button clicked. %lf\n",x+y+z);
	doPositionPlan(x,y,z);

}
void MainWindow::on_btn_load_clicked(bool check){
	FILE * file = fopen("testcase.txt","r");
	printf("===== Load button clicked ===== \n");
	do{
		geometry_msgs:: Pose pStart;
		double x,y,z;
		fscanf(file,"%lf",&x);
		fscanf(file,"%lf",&y);
		fscanf(file,"%lf",&z);

		pStart.position.x = x;
		pStart.position.y = y;
		pStart.position.z = z;

		pStart.orientation.x = 0; pStart.orientation.y = 0.707; pStart.orientation.z = 0; pStart.orientation.w = 0.707;

		pointList.push_back(pStart);

		int row = (ui.tblPath)->rowCount();
		(ui.tblPath)->insertRow(row);
		(ui.tblPath)->setItem(row,0,new QTableWidgetItem(QString::number(pStart.position.x)));
		(ui.tblPath)->setItem(row,1,new QTableWidgetItem(QString::number(pStart.position.y)));
		(ui.tblPath)->setItem(row,2,new QTableWidgetItem(QString::number(pStart.position.z)));


	}while(!feof(file));
}

void MainWindow::on_btn_add_clicked(bool check){
	geometry_msgs:: Pose pStart;
	pStart.position.x = (ui.txtX)->text().toDouble();
	pStart.position.y = (ui.txtY)->text().toDouble();
	pStart.position.z = (ui.txtZ)->text().toDouble();
	pStart.orientation.x = 0; pStart.orientation.y = 0.707; pStart.orientation.z = 0; pStart.orientation.w = 0.707;

	
	pointList.push_back(pStart);



	int row = (ui.tblPath)->rowCount();
	(ui.tblPath)->insertRow(row);
	(ui.tblPath)->setItem(row,0,new QTableWidgetItem(QString::number(pStart.position.x)));
	(ui.tblPath)->setItem(row,1,new QTableWidgetItem(QString::number(pStart.position.y)));
	(ui.tblPath)->setItem(row,2,new QTableWidgetItem(QString::number(pStart.position.z)));


}

void MainWindow::on_btn_grip_open_clicked(bool check){

	setGripperJointValues(0.5);

}

void MainWindow::on_btn_grip_close_clicked(bool check){
	setGripperJointValues(0);
}

void MainWindow::on_btn_clear_clicked(bool check){
	pointList.clear();
	(ui.tblPath)->setRowCount(0);
}
void MainWindow::on_btn_path_clicked(bool check){
	doPathPlan();
}

void MainWindow::executeCartesianPlan(moveit_msgs::RobotTrajectory traj){


}
void printJointVecotors(std::vector<double> jnts){
	for(int i = 0; i < 6; i++ ){

		printf(", %lf", jnts[i]);
	}
	printf("\n");
}

void MainWindow::computeJacobian(){

	moveit::core::RobotModelConstPtr model =  group->getCurrentState()->getRobotModel();
	const moveit::core::JointModelGroup* joint_model_group = model->getJointModelGroup("manipulator");

	for(int i = 0; i < 50; i++){
		moveit::core::RobotStatePtr kinematic_state(group->getCurrentState()/*new robot_state::RobotState(model) */);


		//group->getCurrentState()->getJointV

		Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
		Eigen::MatrixXd jacobian;
		kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
			reference_point_position,
			jacobian);
		ROS_INFO_STREAM("Jacobian: \n" << jacobian);

		//try pseudo inverse
		Eigen::MatrixXd invJ = jacobian.inverse();
		ROS_INFO_STREAM("Inverse of Jacobian:\n " << invJ);

		Eigen::RowVectorXd velocity(6);
		velocity << 0,0,0,0,0,0;


		velocity(0,0) = ((ui.txt_j1)->text().toDouble()) ;
		velocity(0,1) = ((ui.txt_j2)->text().toDouble()) ;
		velocity(0,2) = ((ui.txt_j3)->text().toDouble()) ;
		velocity(0,3) = ((ui.txt_j4)->text().toDouble()) ;
		velocity(0,4) = ((ui.txt_j5)->text().toDouble()) ;
		velocity(0,5) = ((ui.txt_j6)->text().toDouble()) ;
		ROS_INFO_STREAM("velocity " << velocity);

		Eigen::MatrixXd qdot = invJ * velocity.transpose();

		ROS_INFO_STREAM("J-1 * v \n" << qdot);
		std::vector<double> values =  group->getCurrentJointValues();

		/**get joint angles from kinematic state **/
		values[0] = *(kinematic_state->getJointPositions("J1"));
		values[1] = *(kinematic_state->getJointPositions("J2"));
		values[2] = *(kinematic_state->getJointPositions("J3"));
		values[3] = *(kinematic_state->getJointPositions("J4"));
		values[4] = *(kinematic_state->getJointPositions("J5"));
		values[5] = *(kinematic_state->getJointPositions("J6"));
		printJointVecotors(values);

		qdot = qdot * 0.02;
		//publish kinematics.
		//ROS_INFO_STREAM("current values : " << values[]);

		values[0] += qdot(0,0);
		values[1] += qdot(1,0);
		values[2] += qdot(2,0);
		values[3] += qdot(3,0);
		values[4] += qdot(4,0);
		values[5] += qdot(5,0);

		printf("JOint values before \n");
		printJointVecotors(values);
		//ROS_INFO_STREAM("updated values : " << values);
		moveit::planning_interface::MoveGroup::Plan my_plan;
		printf("JOint values after \n");
		updateJointValues(values);

		//group->setJointValueTarget(values);

		printf("values of plan...");

		//group->plan(my_plan);

		//instead of planning create a virtual plan
		//ROS_INFO_STREAM("frame "<< my_plan.trajectory_.joint_trajectory.header.frame_id);
		//ROS_INFO_STREAM("Jnts" << my_plan.trajectory_.joint_trajectory.joint_names[5]);

	//	group->move();
		//moveit_msgs::RobotTrajectory v_trajectory;
		//v_trajectory.joint_trajectory.
		my_plan.trajectory_.joint_trajectory.header.frame_id = "/world";
//s
		my_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
		my_plan.trajectory_.joint_trajectory.joint_names.resize(6);

		my_plan.trajectory_.joint_trajectory.joint_names[0] = "J1";
		my_plan.trajectory_.joint_trajectory.joint_names[1] = "J2";
		my_plan.trajectory_.joint_trajectory.joint_names[2] = "J3";
		my_plan.trajectory_.joint_trajectory.joint_names[3] = "J4";
		my_plan.trajectory_.joint_trajectory.joint_names[4] = "J5";
		my_plan.trajectory_.joint_trajectory.joint_names[5] = "J6";

		my_plan.trajectory_.joint_trajectory.points.resize(1);

		my_plan.trajectory_.joint_trajectory.points[0].positions.resize(6);

		my_plan.trajectory_.joint_trajectory.points[0].positions[0] = values[0];
		my_plan.trajectory_.joint_trajectory.points[0].positions[1] = values[1];
		my_plan.trajectory_.joint_trajectory.points[0].positions[2] = values[2];
		my_plan.trajectory_.joint_trajectory.points[0].positions[3] = values[3];
		my_plan.trajectory_.joint_trajectory.points[0].positions[4] = values[4];
		my_plan.trajectory_.joint_trajectory.points[0].positions[5] = values[5];
		my_plan.trajectory_.joint_trajectory.points[0].time_from_start = ros::Duration(0);
		group->execute(my_plan);

		//ROS_INFO_STREAM("vitrual values printed out.");
	//	updateJointValues(values);

		ros::spinOnce();
		ros::Duration(0.002).sleep();

		//break;

	}


}

void MainWindow::updateJointValues(std::vector<double> current){

	std_msgs::Float64 value;

	value.data =  current[0];
	j1Command.publish(value);

	value.data = current[1];
	j2Command.publish(value);

	value.data = current[2];
	j3Command.publish(value);

	value.data = current[3];
	j4Command.publish(value);

	value.data = current[4];
	j5Command.publish(value);

	value.data =  current[5];
	j6Command.publish(value);

	ros::spinOnce();

}

void MainWindow::updateJointValues(Eigen::MatrixXd joints, std::vector<double > current){

	std_msgs::Float64 value;

	value.data = joints(0,0) + current[0];
	j1Command.publish(value);

	value.data = joints(1,0) + current[1];
	j2Command.publish(value);

	value.data = joints(2,0) + current[2];
	j3Command.publish(value);

	value.data = joints(3,0) + current[3];
	j4Command.publish(value);

	value.data = joints(4,0) + current[4];
	j5Command.publish(value);

	value.data = joints(5,0) + current[5];
	j6Command.publish(value);

	ros::spinOnce();
}

void MainWindow::initializeGazeboLink(){

	ros::NodeHandle nh;
	j1Command = nh.advertise<std_msgs::Float64>("/denso/joint1_position_controller/command", 10, true);
	j2Command = nh.advertise<std_msgs::Float64>("/denso/joint2_position_controller/command", 10, true);
	j3Command = nh.advertise<std_msgs::Float64>("/denso/joint3_position_controller/command", 10, true);
	j4Command = nh.advertise<std_msgs::Float64>("/denso/joint4_position_controller/command", 10, true);
	j5Command = nh.advertise<std_msgs::Float64>("/denso/joint5_position_controller/command", 10, true);
	j6Command = nh.advertise<std_msgs::Float64>("/denso/joint6_position_controller/command", 10, true);

	matlabSubscriber = nh.subscribe("/matlab/jnt_cmd",100, &MainWindow::matlabCallback, this);

}

void MainWindow::on_btn_save_ik_clicked(bool check){

	//computeJacobian(); return;

	FILE * ik_f = fopen("denso_out/ik_result.txt","w+");
	FILE * fk_f = fopen("denso_out/fk_in.txt","w+");

	if(ik_f == NULL){
		printf("Error opening output file. exiting operation\n"); return;
	}

	geometry_msgs:: Pose pStart;
	pStart.position.x = (ui.txtX)->text().toDouble();
	pStart.position.y = (ui.txtY)->text().toDouble();
	pStart.position.z = (ui.txtZ)->text().toDouble();
	pStart.orientation.x = 0; pStart.orientation.y = 0.707; pStart.orientation.z = 0; pStart.orientation.w = 0.707;


	moveit::core::RobotModelConstPtr model =  group->getCurrentState()->getRobotModel();
	const moveit::core::JointModelGroup* joint_model_group = model->getJointModelGroup("manipulator");

	moveit::core::RobotStatePtr kinematic_state(group->getCurrentState()/*new robot_state::RobotState(model) */);
	bool found_ik = kinematic_state->setFromIK(joint_model_group, pStart, 10, 0.1);

	std::vector<double> joint_values;
 	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	if(found_ik){
		for(std::size_t i=0; i < 6; ++i){
     		ROS_INFO("Joint %f", joint_values[i]);
     		fprintf(ik_f, "%lf ",joint_values[i]);
   		}
   		fprintf(ik_f,"\n");
   		fclose(ik_f);
	}

	moveit::planning_interface::MoveGroup::Plan my_plan;

	//group->setJointValueTarget(joint_values);
	//group->plan(my_plan);
	//group->move();

}

void saveMotionPlan(FILE * file, moveit_msgs::RobotTrajectory traj, bool velocity, double offset){

	int signs[] = {1,-1,-1,1,-1,1};

	std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = traj.joint_trajectory.points;
	int i = 0;
	double factor = (180.0) / M_PI;
	for(i = 0; i < pointVector.size(); i++){

		trajectory_msgs::JointTrajectoryPoint pi = pointVector.at(i);
		fprintf(file,"%lf %lf %lf %lf %lf %lf %lf\n",offset+pi.time_from_start.toSec(),
			factor * pi.positions[0],
			factor *pi.positions[1],
			factor *pi.positions[2],
			factor *pi.positions[3],
			factor *pi.positions[4],
			0/*factor *pi.positions[5]*/);

		if(velocity){
			fprintf(file,"%lf %lf %lf %lf %lf %lf\n",pi.velocities[0],pi.velocities[1],pi.velocities[2],pi.velocities[3],
			pi.velocities[4],pi.velocities[5]);			
		}
	}
}

void testMotionPlan(moveit_msgs::RobotTrajectory traj){

	int signs[] = {1,-1,-1,1,-1,1};

	std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = traj.joint_trajectory.points;
	int i = 0;
	double factor = (180.0) / M_PI;
	cout << "Size : " << pointVector.size();
	cout << " size taken" << endl;
	for(i = 0; i < pointVector.size(); i++){

		trajectory_msgs::JointTrajectoryPoint pi = pointVector.at(i);
		printf("%lf, %lf  %lf\n",pi.time_from_start.toSec(),pi.positions[0],pi.positions[1]);
		/*fprintf(file,"%lf %lf %lf %lf %lf %lf %lf\n",pi.time_from_start.toSec(),
			factor * pi.positions[0],
			factor *pi.positions[1],
			factor *pi.positions[2],
			factor *pi.positions[3],
			factor *pi.positions[4],
			0/*factor *pi.positions[5]);*/

	}
}


void MainWindow::printTrajectoryInformation(moveit_msgs::RobotTrajectory traj){

	std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = traj.joint_trajectory.points;
	int i = 0;
	(ui.tblData)->setRowCount(0);

	//QTableWidgetItem *it =  (ui.tblData)->takeItem(0,0);
	//it->setText(*(new QString("abc")));

	for(i=0; i < pointVector.size(); i++){

		trajectory_msgs::JointTrajectoryPoint pi = pointVector.at(i);

		printf( "Time from start : %lf\n", pi.time_from_start.toSec());
		printf("positions:  = %lf , %lf, %lf \n",pi.positions[0], pi.positions[1], pi.positions[2]);
		//if(i>0)
			//double x = pi.velocities[2];
		//printf("Velocity: = %lf, %lf, %lf \n", pi.velocities[1], pi.velocities[1], pi.velocities[2]);
		(ui.tblData)->insertRow(i);
		(ui.tblData)->setItem(i,0,new QTableWidgetItem(QString::number(pi.time_from_start.toSec())));

		(ui.tblData)->setItem(i,1,new QTableWidgetItem(QString::number(pi.positions[0])));
		(ui.tblData)->setItem(i,2,new QTableWidgetItem(QString::number(pi.positions[1])));
		(ui.tblData)->setItem(i,3,new QTableWidgetItem(QString::number(pi.positions[2])));
		(ui.tblData)->setItem(i,4,new QTableWidgetItem(QString::number(pi.positions[3])));
		(ui.tblData)->setItem(i,5,new QTableWidgetItem(QString::number(pi.positions[4])));
		(ui.tblData)->setItem(i,6,new QTableWidgetItem(QString::number(pi.positions[5])));

		//for velocities.
		(ui.tblData)->setItem(i,6+1,new QTableWidgetItem(QString::number(pi.accelerations[0])));
		(ui.tblData)->setItem(i,6+2,new QTableWidgetItem(QString::number(pi.accelerations[1])));
		(ui.tblData)->setItem(i,6+3,new QTableWidgetItem(QString::number(pi.accelerations[2])));
		(ui.tblData)->setItem(i,6+4,new QTableWidgetItem(QString::number(pi.accelerations[3])));
		(ui.tblData)->setItem(i,6+5,new QTableWidgetItem(QString::number(pi.accelerations[4])));
		(ui.tblData)->setItem(i,6+6,new QTableWidgetItem(QString::number(pi.accelerations[5])));


		//add to display table.
		

	}


}

// set direct joint target.

void MainWindow::on_btn_jnt_cmd_clicked(bool check){

	bool deg = (ui.chk_jng_deg)->isChecked();

	double factor = deg ? (M_PI / 180.0): 1;

	moveit::planning_interface::MoveGroup::Plan my_plan;
	std::vector<double> angles;
	
	angles.push_back( factor * ((ui.txt_j1)->text().toDouble()) );
	angles.push_back( factor * ((ui.txt_j2)->text().toDouble()) );
	angles.push_back( factor * ((ui.txt_j3)->text().toDouble()) );
	angles.push_back( factor * ((ui.txt_j4)->text().toDouble()) );
	angles.push_back( factor * ((ui.txt_j5)->text().toDouble()) );
	angles.push_back( factor * ((ui.txt_j6)->text().toDouble()) );

	group->setJointValueTarget(angles);
	group->plan(my_plan);
	group->move();

	printf("<joint value targt executed.>\n");

}

void MainWindow::doPathPlan(){

	bool should_out = (ui.chk_traj)->isChecked();
	FILE * out_file;

	if(should_out){
		printf("output enabled....\n");
		out_file = fopen("denso_out/trajectory.txt","w");
		if(out_file == NULL){
			printf("file opening failed...\n");
			should_out = 0;
		}else{
			printf("file opening sucess.\n");
		}
	}
	//Move to first point.
	moveit::planning_interface::MoveGroup::Plan my_plan;
	group->setPoseTarget(pointList[0]);
	group->plan(my_plan);
	group->move();

	printTrajectoryInformation(my_plan.trajectory_);

	if(should_out){

		saveMotionPlan(out_file, my_plan.trajectory_, (ui.chk_vel2)->isChecked(),0);
	}

	double time_f = my_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() + 1.0;
	//return;
	moveit_msgs::RobotTrajectory trajectory;

	group->clearPoseTargets();
	
	double f = group->computeCartesianPath(pointList,0.0002,0.0, trajectory);

	(ui.lblS)->setText(QString::number(f*100) +"% covered");
	//trajectory contains plan of joint angles, excluding joint velocities.
	// this will interpolate joint velocities.

	robot_trajectory::RobotTrajectory rt (group->getCurrentState()->getRobotModel(), "manipulator");

	//we can change currentstate to first of the trajectory if needed.
 	rt.setRobotTrajectoryMsg(*(group->getCurrentState()), trajectory); // trajectory is a moveit_msg.
 	trajectory_processing::IterativeParabolicTimeParameterization iptp;

	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  	rt.getRobotTrajectoryMsg(trajectory);
  	my_plan.trajectory_ = trajectory;

  	if(should_out){

		saveMotionPlan(out_file, my_plan.trajectory_, (ui.chk_vel2)->isChecked(),time_f);
		//fclose(out_file);
	}

	//updated for neluma.
	time_f  += my_plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec() + 10;

	//set joint target to home position.

	group->execute(my_plan);


	double factor = (M_PI / 180.0);

	//moveit::planning_interface::MoveGroup::Plan my_plan;
	std::vector<double> angles;
	
	angles.push_back( factor * 0 );
	angles.push_back( factor * 0 );
	angles.push_back( factor * -30 );
	angles.push_back( factor *  0);
	angles.push_back( factor *  0);
	angles.push_back( factor *  0);

	group->setJointValueTarget(angles);
	group->plan(my_plan);
	group->move();

	if(should_out){

		saveMotionPlan(out_file, my_plan.trajectory_, (ui.chk_vel2)->isChecked(),time_f+2);
		fclose(out_file);

		ROS_INFO("HOMING ok.\n");
	}


  	

	sleep(1);
	printTrajectoryInformation(my_plan.trajectory_);

}



void MainWindow::doPositionPlan(double x, double y, double z){

	moveit::planning_interface::MoveGroup::Plan my_plan;

	geometry_msgs:: Pose pStart;
	pStart.position.x = x;
	pStart.position.y = y;
	pStart.position.z = z;
	pStart.orientation.x = 0; pStart.orientation.y = 0.707;
	 pStart.orientation.z = 0; pStart.orientation.w = 0.707;

	group->setPoseTarget(pStart);
	group->plan(my_plan);
	group->move();
	sleep(1);
	//spinner->spinOnce();
	printf("position acquired...");

	if((ui.chk_mp)->isChecked()){
		FILE * mp = fopen("denso_out/motions.txt","w");
		saveMotionPlan(mp, my_plan.trajectory_, (ui.chk_vel1)->isChecked(),0);
		fclose(mp);
	}

	std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = my_plan.trajectory_.joint_trajectory.points;
	int last = pointVector.size();

	if(last > 0){
		ROS_INFO("Executing ... ");
		std::vector<double> lastPoint = pointVector[last - 1].positions;
		updateJointValues(lastPoint);
	}
}



void MainWindow::executeTrajectory(moveit_msgs::RobotTrajectory traj){

	 ros::Rate loop_rate(30);

	std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = traj.joint_trajectory.points;
	int i = 0;
	double factor = (180.0) / M_PI;
	cout << "Size : " << pointVector.size();
	cout << " size taken" << endl;

	sensor_msgs::JointState info;
	//ros::Time::now();
	double secs = pointVector.at(0).time_from_start.toSec();

	for(i = 0; i < pointVector.size(); i++){

		trajectory_msgs::JointTrajectoryPoint pi = pointVector.at(i);
		printf("%lf, %lf  %lf\n",pi.time_from_start.toSec(),pi.positions[0],pi.positions[1]);
		
		
		info.name.resize(6);
		info.position.resize(6);
		//First 


		info.name[0] = "J1"; info.name[1] = "J2";info.name[2] = "J3";
		info.name[3] = "J4";info.name[4] = "J5";info.name[5] = "J6";
		
		info.position[0] = pi.positions[0];
		info.position[1] = pi.positions[1];
		info.position[2] = pi.positions[2];
		info.position[3] = pi.positions[3];
		info.position[4] = pi.positions[4];
		info.position[5] = pi.positions[5];
		double diff = pi.time_from_start.toSec() - secs;

		sleep(diff < 0.5 ? 0.5:diff);
		ros::Duration(diff/2.0).sleep();
		secs = pi.time_from_start.toSec();

		//loop_rate.sleep();
		info.header.stamp = ros::Time::now()+ros::Duration(diff);
		jointStatePub.publish(info);


		


	}


}

void MainWindow::simulateTrajectory(){

	bool degrees = 0;

	if((ui.chk_sim_deg)->isChecked()){
		degrees = 1;
	}
	double signs[] = {+1,-1,-1,+1,-1,+1};

	if(!((ui.chk_sim_map)->isChecked()) ){
		signs[1] = signs[2] = signs[4] = 1;
	}

	double factor = degrees ? M_PI/180.0: 1 ;

	double t;
	//FILE * fp = fopen("sim.txt");
	ifstream fp("sim.txt");
	if(fp == NULL){

		printf("File opening failed.\n");
		return;
	}
	//std::vector<trajectory_msgs:: JointTrajectoryPoint> pointVector = traj.joint_trajectory.points;

	moveit_msgs::RobotTrajectory trajectory;
	//trajectory.position

	std::vector<trajectory_msgs::JointTrajectoryPoint> pointVector;

	trajectory_msgs::JointTrajectoryPoint point;

	std::vector<string> jnt_names;
	jnt_names.push_back("J1");jnt_names.push_back("J2");jnt_names.push_back("J3");
	jnt_names.push_back("J4");jnt_names.push_back("J5");jnt_names.push_back("J6");

	trajectory.joint_trajectory.joint_names = jnt_names;

	double angles[6];

	double min_limits[] = {-2.09, -1.57, 0.52, -2.09, -2.09,-2.62 };
	double max_limits[] = {2.09, 1.22, 2.62, 2.09, 2.09,2.62 };
	(ui.tblData)->setRowCount(0);
	int wrong_rows = 0;
	while(fp){ // scan 
		std::vector<double> pos;
		
		fp >>t; 
		fp >> angles[0] ;
		fp >> angles[1] ;
		fp >> angles[2] ;
		fp >> angles[3] ;
		fp >> angles[4] ;
		fp >> angles[5] ;
		//point.time_from_start = t;
		pos.push_back(angles[0]*factor/*signs[0]*/);
		pos.push_back(angles[1]*factor/*signs[1]*/);
		pos.push_back(angles[2]*factor/*signs[2]*/);
		pos.push_back(angles[3]*factor/*signs[3]*/);
		pos.push_back(angles[4]*factor/*signs[4]*/);
		pos.push_back(angles[5]*factor/*signs[5]*/);

		bool limit_ok = 1;

		for(int i = 0; i < 6; i++){

				if(angles[i]*factor*signs[i] < min_limits[i] || angles[i]*factor*signs[i] > max_limits[i]){
					limit_ok = 0;
					break;
				}

		}

		if(!limit_ok){
			(ui.tblData)->insertRow(wrong_rows);
			(ui.tblData)->setItem(wrong_rows,0,new QTableWidgetItem(QString::number(t)));
			(ui.tblData)->setItem(wrong_rows,1,new QTableWidgetItem(QString::number(angles[0])));
			(ui.tblData)->setItem(wrong_rows,2,new QTableWidgetItem(QString::number(angles[1])));
			(ui.tblData)->setItem(wrong_rows,3,new QTableWidgetItem(QString::number(angles[2])));
			(ui.tblData)->setItem(wrong_rows,4,new QTableWidgetItem(QString::number(angles[3])));
			(ui.tblData)->setItem(wrong_rows,5,new QTableWidgetItem(QString::number(angles[4])));
			(ui.tblData)->setItem(wrong_rows,6,new QTableWidgetItem(QString::number(angles[5])));

			wrong_rows = wrong_rows + 1;
		}


		point.positions = pos;
		ros::Duration dur(t);
		point.time_from_start = dur;
		
		pointVector.push_back(point);

	}

	trajectory.joint_trajectory.points = pointVector;

	cout << "upto now ok" << endl;
	//FILE * fw = fopen("sim_saved.txt","w");
	//testMotionPlan(trajectory);
	//fclose(fw);
	cout << "saved back \n";
	moveit::planning_interface::MoveGroup::Plan my_plan;

	my_plan.trajectory_ = trajectory;
	//
	//executeTrajectory(trajectory);

	//method ok.
	moveit_msgs::DisplayTrajectory display_trajectory;
	display_trajectory.trajectory.push_back(trajectory);
	display_publisher.publish(display_trajectory);
	sleep(5.0);
	//group->execute(my_plan);


}
 

void MainWindow::setGripperJointValues(double theta){
	/*sensor_msgs::JointState info;
	
	info.header.stamp = ros::Time::now();
	info.name.resize(2);
	info.position.resize(2);
	
	info.name[0] = "right_gripper_joint";
	info.name[1] = "left_gripper_joint";

	info.position[0] = theta;
	info.position[1] = theta;

	if(jointStatePub == NULL){

		printf("%s\n","Joint state publisher is null " );
		return ;
	}

	jointStatePub.publish(info);
	ros::spinOnce();*/

	std::vector< double> angles;
	angles.push_back(theta);
	angles.push_back(theta);
	gripper_group->setJointValueTarget(angles);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	gripper_group->plan(my_plan);
	gripper_group->move();
}

/***********
 *
 * Implementation for matlab gazebo link
 */

void MainWindow::matlabCallback(const geometry_msgs::TwistPtr &msg){

	moveit::planning_interface::MoveGroup::Plan my_plan;
	my_plan.trajectory_.joint_trajectory.header.frame_id = "/world";
	//s
	my_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
	my_plan.trajectory_.joint_trajectory.joint_names.resize(6);

	my_plan.trajectory_.joint_trajectory.joint_names[0] = "J1";
	my_plan.trajectory_.joint_trajectory.joint_names[1] = "J2";
	my_plan.trajectory_.joint_trajectory.joint_names[2] = "J3";
	my_plan.trajectory_.joint_trajectory.joint_names[3] = "J4";
	my_plan.trajectory_.joint_trajectory.joint_names[4] = "J5";
	my_plan.trajectory_.joint_trajectory.joint_names[5] = "J6";

	my_plan.trajectory_.joint_trajectory.points.resize(1);

	my_plan.trajectory_.joint_trajectory.points[0].positions.resize(6);

	my_plan.trajectory_.joint_trajectory.points[0].positions[0] = msg->linear.x;
	my_plan.trajectory_.joint_trajectory.points[0].positions[1] = msg->linear.y;
	my_plan.trajectory_.joint_trajectory.points[0].positions[2] = msg->linear.z;
	my_plan.trajectory_.joint_trajectory.points[0].positions[3] = msg->angular.x;
	my_plan.trajectory_.joint_trajectory.points[0].positions[4] = msg->angular.y;
	my_plan.trajectory_.joint_trajectory.points[0].positions[5] = msg->angular.z;

	my_plan.trajectory_.joint_trajectory.points[0].time_from_start = ros::Duration(0.0025);
	group->execute(my_plan);

	//updateJointValues(my_plan.trajectory_.joint_trajectory.points[0].positions);

	ros::spinOnce();
	ROS_INFO_STREAM("vitrual values printed out.");

}
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/


void MainWindow::closeEvent(QCloseEvent *event)
{

	QMainWindow::closeEvent(event);
}


/*************************************************
 * ** Implementation of Matlab Gazebo interface for vp6 1.0
 **/


void MainWindow::velocityCallback(std_msgs::Float32::ConstPtr &msg){

	zvelocity = msg->data ;

}

}  // namespace vp6_simulator




