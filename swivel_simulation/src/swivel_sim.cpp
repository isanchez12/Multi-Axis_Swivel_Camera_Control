
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "swivel_sim.h"

using namespace irk_simulation;

SWIVELSim::SWIVELSim(const std::string &prefix,
   hardware_interface::JointStateInterface &js_interface,
    hardware_interface::EffortJointInterface &ej_interface) :
  n_dof_(3),
  n_actuated_dof_(2),
  initialized_(false),
  prefix_(prefix),
  js_interface_(js_interface),
  ej_interface_(ej_interface),
  // Joint Information
  joint_name_(n_dof_),
  joint_position_(n_dof_),
  joint_velocity_(n_dof_),
  joint_effort_(n_dof_),
  joint_effort_command_(n_actuated_dof_),
  sim_joints_(n_dof_)
{   
}   


bool SWIVELSim::initSim(
    ros::NodeHandle nh,
    gazebo::physics::ModelPtr parent_model,
    std::vector<gazebo_ros_control::JointData> joints)
{
  // Hard code joint names

  // Actuated joints
  joint_name_[0] = prefix_ + "swivel_J0";
  joint_name_[1] = prefix_ + "swivel_J1";

  // Unactuated Joints  
  joint_name_[2] = prefix_ + "swivel_J2";
  

  for(unsigned int j=0; j < n_dof_; j++) {   
    // Initialize joint state
    joint_position_[j] = 0.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 0.0;

    // Register joint interfaces
    js_interface_.registerHandle(
        hardware_interface::JointStateHandle(
          joint_name_[j],
          &joint_position_[j], 
          &joint_velocity_[j], 
          &joint_effort_[j]));


    if(j < n_actuated_dof_) {
      joint_effort_command_[j] = 0.0;
      ej_interface_.registerHandle(
          hardware_interface::JointHandle(
            js_interface_.getHandle( joint_name_[j]),
            &joint_effort_command_[j]));
    }
  }   

  // Get the gazebo joints that correspond to the robot joints
  for(unsigned int j=0; j < n_dof_; j++)
  {
    ROS_DEBUG_STREAM("Getting pointer to gazebo joint: "<<joint_name_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name_[j]);

    if (joint) {
      sim_joints_[j] = joint;
    } else {
      ROS_ERROR_STREAM("This robot has a joint named \""<<joint_name_[j]<<"\" which is not in the gazebo parent_model.");
      return false;
    }
  }

  // Set initialized flag
  initialized_ = true;

  return true;
}

void SWIVELSim::readSim(ros::Time time, ros::Duration period)
{
  if(!initialized_) { return; }

  // Update state from gazebo for actuated joints
  for(unsigned int j=0; j < n_actuated_dof_; j++)
  {
    joint_position_[j] += angles::shortest_angular_distance(
        joint_position_[j], 
        sim_joints_[j]->GetAngle(0).Radian());
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned)0);
  }

}

void SWIVELSim::writeSim(ros::Time time, ros::Duration period)
{
  if(!initialized_) { return; }

  for(unsigned int j=0; j < n_actuated_dof_; j++) {
    sim_joints_[j]->SetForce(0,joint_effort_command_[j]);
  }
}
