/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

//void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);


#define RAD(deg) ((double)(deg) * M_PI / 180.0)
#define PI 3.14159265

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";
const string world_file = "./resources/world.urdf";

enum State 
{
	POSTURE = 0, 
	MOTION,
	CAUGHT,
	DRAG
};

// helper function 
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

int main() {

	// initial state 	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	
	// prepare controller2
	const string ee_link_name = "link7";
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.2);
	VectorXd g = VectorXd::Zero(dof);
	VectorXd b = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	//MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, ee_link_name, pos_in_ee_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
	
	//Load in sim data?
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
	const vector<string> object_names = {"cup"};
	Quaterniond _object_ori;
	//sim->getObjectPosition(object_names[0], _object_pos, _object_ori);
	sim->getObjectVelocity(object_names[0], _object_lin_vel, _object_ang_vel);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task->_use_interpolation_flag = true;
	posori_task->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 100.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	VectorXd q_init_desired(dof), q_desired(dof);
	q_init_desired << -30.0, 15.0, -15.0, -135.0, 0.0, 160.0, 45.0, 0.02*180/M_PI, -0.02*180/M_PI;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;
	
	//Drag state variables
	double theta_robot;
	double r;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;
	string caught_status = "0";

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	redis_client.set(CAUGHT_KEY, "0"); 

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, OBJECT_POS_KEY, _object_pos);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addStringToWriteCallback(0, CAUGHT_KEY, caught_status);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	double caught_time = 0;
	double drag_time;
	bool fTimerDidSleep = true;

	unsigned long long counter = 0;

	runloop = true;

	while (runloop) {
		sim->getObjectPosition(object_names[0], _object_pos, _object_ori);
		sim->getObjectVelocity(object_names[0], _object_lin_vel, _object_ang_vel);
		
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			if ( (robot->_q - q_init_desired).norm() < 0.15 ) {
				cout << "Posture To Motion" << endl;
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
				robot->position(ee_pos, control_link, control_point);
				posori_task->_desired_position = ee_pos - Vector3d(-0.1, -0.1, 0.1);
				posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				// posori_task->_desired_orientation = AngleAxisd(0.0000000000000001, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				state = MOTION;
			}
		} else if (state == MOTION) {
			/*
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;
			*/
			Vector3d x, x_d, dx_d, dx, F, objectPosOld, caught_threshold, caught_delta;
			VectorXd g(dof), joint_task_torque(dof), q_low(dof), q_high(dof), Gamma_mid(dof), Gamma_damp(dof);
			MatrixXd Gamma_Neutralizer = MatrixXd::Identity(dof,dof);
			Gamma_Neutralizer(0,0) = 0.0;

			double kp = 400;
			double kv = 20;
			double kpj = 100;
			double kvj = 30;

			double Vmax = 0.75; // Previously 0.75;
			
			robot->position(x, ee_link_name, pos_in_ee_link);
			robot->linearVelocity(dx, ee_link_name, pos_in_ee_link);
			robot->Jv(Jv, ee_link_name, pos_in_ee_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->nullspaceMatrix(N, Jv);



			q_low(0) = -2.8973;
			q_low(1) = -1.7628;
			q_low(2) = -2.8973;
			q_low(3) = -3.0718;
			q_low(4) = -2.8973;
			q_low(5) = -0.0175;
			q_low(6) = -2.8973;
			q_low(7) = 0.0;
			q_low(8) = -0.04;
			// set q_high
			q_high(0) = 2.8973;
			q_high(1) = 1.7628;
			q_high(2) = 2.8973;
			q_high(3) = -0.0698;
			q_high(4) = 2.8973;
			q_high(5) = 3.7525;
			q_high(6) = 2.8973;
			q_high(7) = 0.04;
			q_high(8) = 0.00;

			
			//x_d << _object_pos; // No anticipation or interception
			double interceptionScaling = 10;
			x_d << _object_pos + interceptionScaling*(_object_pos - objectPosOld);
			//x_d << -0.32, -0.49, 0.15;

			

			x_d(2) = 0.15; // Mouse target position for z-direction
			//cout << "\nMouse\n" << _object_pos << endl;
			//cout << "\nDesired Position\n" << x_d << endl;
			//cout << "\nX-pos\n" << x << endl;
			//cout << "\nDesired Position\n" << x_d << endl;

			dx_d = kp / kv * (x_d - x);
			double nu = sat(Vmax / dx_d.norm());

			F = Lambda * (-kv * (dx - nu * dx_d));
			//F = Lambda * (-kv * (dx - dx_d));
			Gamma_damp = -kvj * robot->_dq;

			Gamma_mid = -2 * kpj * (robot->_q - (q_high + q_low) / 2);
			Gamma_mid = Gamma_Neutralizer * Gamma_mid;

			q_desired = q_init_desired;
			//q_desired << 0, 0, 0, 0, 0, 0, 0, 0, 0;

			//joint_task_torque = -kpj*(robot->_q - q_desired) - kvj * robot->_dq;

			robot->gravityVector(g);
			
			command_torques = Jv.transpose()*F + N.transpose()*((Gamma_mid + Gamma_damp));

			if (counter % 100 == 0)
			{
				objectPosOld = _object_pos;
			}

			caught_threshold << 0.02, 0.02, 0.15;
			caught_delta = x - _object_pos;
			caught_delta = caught_delta.array().abs();

			if (caught_delta(0) <= caught_threshold(0) && caught_delta(1) <= caught_threshold(1) && caught_delta(2) <= caught_threshold(2))
			{
				state = CAUGHT;
				caught_time = time;
				q_desired << robot->_q;
				q_desired(7) = 0.01;
				q_desired(8) = -0.01;
				
				
				caught_status = "1"; 
			}
			
		} 
		else if (state == CAUGHT) {
			Vector3d x;
			double kp = 200;
			double kv = 20;
			command_torques = robot->_M*(-kp*(robot->_q - q_desired) - kv*robot->_dq);
			if (time > caught_time+2){
				robot->position(x, ee_link_name, pos_in_ee_link);
				state = DRAG;
				drag_time = time;
				theta_robot = atan2(x(1),x(0)) - PI/2;
				r = sqrt((x(0))*(x(0)) + (x(1))*(x(1)));
				caught_status = "2"; 
			}
		}
		else if (state == DRAG) {
			Vector3d x, x_d, dx_d, ddx_d, dx, F, objectPosOld, caught_threshold, caught_delta;
			VectorXd g(dof), joint_task_torque(dof), q_low(dof), q_high(dof), Gamma_mid(dof), Gamma_damp(dof);
			MatrixXd Gamma_Neutralizer = MatrixXd::Identity(dof,dof);
			Gamma_Neutralizer(0,0) = 0.0;

			double kp = 400;
			double kv = 20;
			double kpj = 100;
			double kvj = 30;

			double Vmax = 0.75; // Previously 0.75;
			
			robot->position(x, ee_link_name, pos_in_ee_link);
			robot->linearVelocity(dx, ee_link_name, pos_in_ee_link);
			robot->Jv(Jv, ee_link_name, pos_in_ee_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->nullspaceMatrix(N, Jv);

			double current_time = time - drag_time;

			q_low(0) = -2.8973;
			q_low(1) = -1.7628;
			q_low(2) = -2.8973;
			q_low(3) = -3.0718;
			q_low(4) = -2.8973;
			q_low(5) = -0.0175;
			q_low(6) = -2.8973;
			q_low(7) = 0.0;
			q_low(8) = -0.04;
			// set q_high
			q_high(0) = 2.8973;
			q_high(1) = 1.7628;
			q_high(2) = 2.8973;
			q_high(3) = -0.0698;
			q_high(4) = 2.8973;
			q_high(5) = 3.7525;
			q_high(6) = 2.8973;
			q_high(7) = 0.04;
			q_high(8) = 0.00;
			
			if (r > 0.61){
				r -= 0.0001;
			}
			else if (r < 0.59){
				r += 0.0001;
			}			
			
			
			x_d << r*sin(PI*current_time/2-theta_robot), r*cos(PI*current_time/2-theta_robot), 0.15;

			dx_d << r*PI*cos(PI*current_time/2-theta_robot)/2, -r*PI*sin(PI*current_time/2-theta_robot)/2, 0;

			ddx_d << -r*PI*PI*sin(PI*current_time/2-theta_robot)/4, -r*PI*PI*cos(PI*current_time/2-theta_robot)/4, 0;

			F = Lambda*(ddx_d -kp*(x - x_d) - kv*(dx - dx_d));

			Gamma_damp = -kvj * robot->_dq;

			Gamma_mid = -2 * kpj * (robot->_q - (q_high + q_low) / 2);
			Gamma_mid = Gamma_Neutralizer * Gamma_mid;

			q_desired = q_init_desired;
			
			command_torques = Jv.transpose()*F + N.transpose()*((Gamma_mid + Gamma_damp));
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
