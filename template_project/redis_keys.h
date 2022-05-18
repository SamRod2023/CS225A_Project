/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller";

//const std::string OBJECT_POS_KEY = "sai2::sim::cup::actuator::pos";
const std::string OBJECT_POS_KEY = "sai2::sim::cup::actuator::pos";
const std::string HAPTIC_POS_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_position";
const std::string HAPTIC_VEL_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity";
const std::string HAPTIC_FORCE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_force";
const std::string CAUGHT_KEY = "sai2::sim::object::caught::status";

