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

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";
const string world_file = "./resources/world.urdf";

enum State 
{
	POSTURE = 0, 
	MOTION
};

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
	const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.1654);
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

	VectorXd q_init_desired(dof);
	q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0, 0.02*180/M_PI, -0.02*180/M_PI;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// containers
	Vector3d ee_pos;
	Matrix3d ee_rot;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToReadCallback(0, OBJECT_POS_KEY, _object_pos);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
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
			Vector3d x, x_d, dx, F;
			VectorXd g(dof), joint_task_torque(dof);

			double kp = 100;
			double kv = 20;
			double kpj = 50;
			double kvj = 14;
			
			robot->position(x, ee_link_name, pos_in_ee_link);
			robot->linearVelocity(dx, ee_link_name, pos_in_ee_link);
			robot->Jv(Jv, ee_link_name, pos_in_ee_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->nullspaceMatrix(N, Jv);
			
			
			x_d << _object_pos;
			//cout << "Position: " << _object_pos << endl;

			VectorXd q_desired = q_init_desired;
			//q_desired << 0, 0, 0, 0, 0, 0, 0;

			robot->gravityVector(g);

			command_torques = Jv.transpose()*(Lambda*(-kp*(x - x_d) - kv*dx)) + N.transpose()*(-kpj*(robot->_q - q_desired) - kvj*robot->_dq) + g;

			
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
