/**
 * @file simviz.cpp
 * @brief Simulation + visualization 
 * 
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>
#include "chai3d.h"

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"
#include "../include/object.h"

using namespace std;
using namespace Eigen;
using namespace chai3d;

// specify urdf and robots 
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "panda";
const string camera_name = "camera_fixed";
const string camera_2 = "camera_2";
const string base_link_name = "link0";
const string ee_link_name = "link7";
const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.2);
string caught_status = "0";

// dynamic objects information
const vector<string> object_names = {"cup"};
vector<Vector3d> object_pos;
vector<Vector3d> object_lin_vel;
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();

// redis client 
RedisClient redis_client; 

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

//Added Variables
Vector3d Delta = Vector3d(0.0, 0.0, 0.01);
Vector3d object_final;
Vector3d haptic_pos, haptic_force, haptic_vel;
Vector3d x;
Vector3d dx;
int score;
int lower = 0; //Scoring previous position tracker
double theta_robot;
double theta_mouse;

cFontPtr font;
cLabel* labelDescription;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	//graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 

	Eigen::Vector3d camera_2_pos, camera_2_lookat, camera_2_vertical;
	graphics->getCameraPose(camera_2, camera_2_pos, camera_2_vertical, camera_2_lookat);
	graphics->getCamera(camera_2)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();
	//robot->_q = VectorXd::Zero(9);
	//robot->_dq = VectorXd::Zero(9);
	//robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	//sim->setJointPositions(robot_name, robot->_q);
	//sim->setJointVelocities(robot_name, robot->_dq);
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
		Quaterniond _object_ori;
		sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
		sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
		object_pos.push_back(_object_pos);
		object_lin_vel.push_back(_object_lin_vel);
		object_ori.push_back(_object_ori);
		object_ang_vel.push_back(_object_ang_vel);
		//cout << "Position: " << _object_pos << endl;
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.9 * screenW;
	int windowH = 0.9 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;
	
	//Create Text for score
	font = NEW_CFONTCALIBRI20();
	labelDescription = new cLabel(font);
	graphics->getCamera(camera_name)->m_frontLayer->addChild(labelDescription);
	labelDescription->setText("Score: Broken :(");
	labelDescription->m_fontColor.setWhite();
	labelDescription->setLocalPos(50, 0);// windowH + 600);
	labelDescription->setFontScale(8);

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Camera 1: Top Down", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window_2 = glfwCreateWindow(windowW /3, windowH /3, "Camera 2", NULL, NULL);
	glfwSetWindowPos(window_2, windowPosX + 3*windowW / 4, windowPosY);
	glfwShowWindow(window_2);
	glfwMakeContextCurrent(window_2);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values 
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");  
	redis_client.set(CAUGHT_KEY, "0");  
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq); 

	// start simulation thread
	thread sim_thread(simulation, robot, sim);

	// initialize glew
	glewInitialize();

	// while window is open:
	int count = 0;
	Vector3d start_pos = Vector3d(1, -1, 1);

	fSimulationRunning = true;

	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);

		int width_2, height_2;
		glfwGetFramebufferSize(window_2, &width_2, &height_2);

		graphics->updateGraphics(robot_name, robot); 
		if (caught_status == "0") //Catching(MOTION) state
		{
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
			}
		} 
		else if (caught_status == "2") //DRAG state
		{
			robot->position(x, ee_link_name, pos_in_ee_link);
			//Object position = end effector position in x & y
			object_pos[0](0) = x(0);
			object_pos[0](1) = x(1);
			graphics->updateObjectGraphics(object_names[0], object_pos[0], object_ori[0]);
		}

		object_final = object_pos[0]; //Update key to send to controller
		
		glfwMakeContextCurrent(window);
		graphics->render(camera_name, width, height);
		glfwMakeContextCurrent(window_2);
		graphics->render(camera_2, width_2, height_2);

		// swap buffers
		glfwMakeContextCurrent(window);
		glfwSwapBuffers(window);
		glfwMakeContextCurrent(window_2);
		glfwSwapBuffers(window_2);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		Delta << 0,0,0;
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		Eigen::Vector3d cam_depth_axis_2;
		cam_depth_axis_2 = camera_2_lookat - camera_2_pos;
		cam_depth_axis_2.normalize();
		Eigen::Vector3d cam_up_axis_2;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis_2 << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis_2 = (camera_2_lookat - camera_2_pos).cross(cam_up_axis_2);
		cam_roll_axis_2.normalize();
		Eigen::Vector3d cam_lookat_axis_2 = camera_2_lookat;
		cam_lookat_axis_2.normalize();

		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		graphics->setCameraPose(camera_2, camera_2_pos, cam_up_axis_2, camera_2_lookat);
		
		count++;
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);
	glfwDestroyWindow(window_2);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim)
{
	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	VectorXd g = VectorXd::Zero(dof);
	string controller_status = "0";
	double kv = 10;  // can be set to 0 if no damping is needed

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.addEigenToReadCallback(0, HAPTIC_POS_KEY, haptic_pos);
	redis_client.addEigenToWriteCallback(0, HAPTIC_FORCE_KEY, haptic_force);
	redis_client.addEigenToReadCallback(0, HAPTIC_VEL_KEY, haptic_vel);
	redis_client.addStringToReadCallback(0, CAUGHT_KEY, caught_status);

	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.addEigenToWriteCallback(0, OBJECT_POS_KEY, object_final);


	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;

	// start simulation 
	fSimulationRunning = true;
	//Mouse z-position stays constant	
	object_pos[0](2) = 0.15;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// apply gravity compensation 
		robot->gravityVector(g);

		// set joint torques
		if (controller_status == "1") {
			sim->setJointTorques(robot_name, command_torques+ g);
		} else {
			sim->setJointTorques(robot_name, g - robot->_M * (kv * robot->_dq));
		}

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		
		
		
		// get dynamic object positions
		for (int i = 0; i < n_objects; ++i) {

			if (caught_status == "0"){
				haptic_force = -400*haptic_pos - haptic_vel;
			}
			else if (caught_status == "1") {
				haptic_force = -3000*haptic_pos - 30*haptic_vel;
			}
			else if (caught_status == "2") {
				robot->linearVelocity(dx, ee_link_name, pos_in_ee_link);

				haptic_force(0) = 0;
				haptic_force(1) = 10*dx(0);
				haptic_force(2) = 10*dx(1);
				
			}

			Matrix3d R;
			R << 0, 1, 0, 0, 0, 1, 1, 0, 0;
			haptic_pos = R*haptic_pos;
			haptic_pos(2) = 0;
			//haptic_pos = haptic_pos*10;

			double r = sqrt((object_pos[i](0))*(object_pos[i](0)) + (object_pos[i](1))*(object_pos[i](1)));
			//Accounts for drift of haptic device
			if (abs(haptic_pos(0)) < 0.002 && abs(haptic_pos(1)) < 0.0015){
				haptic_pos(0) = 0;
				haptic_pos(1) = 0;
			}
			
			object_pos[i] = object_pos[i] + haptic_pos*0.02;
			//inner
			if ( r < 0.2) {
				double theta = atan2(object_pos[i](1),object_pos[i](0));
				object_pos[i](0) = 0.2 * cos(theta);
				object_pos[i](1) = 0.2 * sin(theta);
			}
			//outer
			if (r > 0.875) {
				double theta = atan2(object_pos[i](1),object_pos[i](0));
				object_pos[i](0) = 0.875 * cos(theta);
				object_pos[i](1) = 0.875 * sin(theta);
			}
			if (r > 0.873) {
				double theta = atan2(object_pos[i](1),object_pos[i](0));
				double mag = r - 0.873;

				haptic_force(1) = haptic_force(1) - 5500*mag * cos(theta);
				haptic_force(2) = haptic_force(2) - 5500*mag * sin(theta);
			}

			if (r < 0.202) {
				double theta = atan2(object_pos[i](1),object_pos[i](0));
				double mag = r - 0.202;

				haptic_force(1) = haptic_force(1) - 5500*mag * cos(theta);
				haptic_force(2) = haptic_force(2) - 5500*mag * sin(theta);
			}
			
			//rold = r;

			sim->setObjectPosition(object_names[i], object_pos[i], object_ori[i]);
			//graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		}
		
		Vector3d mouse_pos = Vector3d(0.0, 0.0, 0.0);
		
		//Score tracking
		robot->position(x, ee_link_name, pos_in_ee_link);
		theta_robot = atan2(x(1),x(0));
		//Rotate mouse position into a moving frame aligned with the endeffector
		mouse_pos(0) = object_pos[0](0)*cos(theta_robot)+object_pos[0](1)*sin(theta_robot);
		mouse_pos(1) = -object_pos[0](0)*sin(theta_robot)+object_pos[0](1)*cos(theta_robot);
		theta_mouse = atan2(mouse_pos(1),mouse_pos(0));
		//Check if mouse crossed the robot path
		if (theta_mouse > 0 && lower == 1 && mouse_pos(0) > 0 && caught_status == "0" && controller_status == "1"){
			score += 1;
			lower = 0;
		}
		else if (theta_mouse < 0 && lower == 0 && mouse_pos(0) > 0 && caught_status == "0" && controller_status == "1"){
			score += 1;
			lower = 1;
		}
		//Set lower indicator to correct value based on mouse position
		if (theta_mouse > 0){
			lower = 0;
		}
		else if (theta_mouse < 0){
			lower = 1;
		}
		
		//cout << "\nScore:\n" << score << endl;
		//cout << "\nPos:\n" << theta_mouse << endl;
		labelDescription->setText("Score: " + to_string(score));

		// execute redis write callback
		redis_client.executeWriteCallback(0);		

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	haptic_force << 0, 0, 0;
	redis_client.executeWriteCallback(0);		
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			//fTransXp = set;
			//object_pos[0](0) = object_pos[0](0) + 0.1;
			Delta << 0.1, 0, 0;


			break;
		case GLFW_KEY_LEFT:
			//fTransXn = set;
			Delta << -0.1, 0, 0;
			//object_pos[0](0) = object_pos[0](0) - 0.1;

			break;
		case GLFW_KEY_UP:
			//fTransYp = set;
			Delta << 0, 0.1, 0;
			//object_pos[0](1) = object_pos[0](1) + 0.1;
			//Delta(1) = Delta(1) + 0.1;
			break;
		case GLFW_KEY_DOWN:
			//fTransYn = set;
			Delta << 0, -0.1, 0;
			//object_pos[0](1) = object_pos[0](1) - 0.1;
			//Delta(1) = Delta(1) - 0.1;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			Delta << 0,0,0;
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

