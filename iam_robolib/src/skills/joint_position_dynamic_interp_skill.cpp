#include "iam_robolib/skills/joint_position_dynamic_interp_skill.h"

#include <cassert>
#include <iostream>

#include <franka/robot.h>

#include "iam_robolib/robot_state_data.h"
#include "iam_robolib/run_loop.h"
#include "iam_robolib/run_loop_shared_memory_handler.h"
#include "iam_robolib/trajectory_generator/joint_trajectory_generator.h"

#include <iam_robolib_common/run_loop_process_info.h>

void JointPositionDynamicInterpSkill::execute_skill() {
  assert(false);
}

void JointPositionDynamicInterpSkill::execute_skill_on_franka(
    run_loop* run_loop, FrankaRobot* robot, RobotStateData *robot_state_data) {

  double time = 0.0;
  int log_counter = 0;
  std::array<double, 16> pose_desired;

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
      *(shared_memory_handler->getRunLoopProcessInfoMutex()),
      boost::interprocess::defer_lock);
  SensorDataManager* sensor_data_manager = run_loop->get_sensor_data_manager();

  std::cout << "Will run the control loop\n";

  JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator_);

  JointSensorInfo joint_sensor_msg;

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::JointPositions {
    if (time == 0.0) {
      joint_trajectory_generator->initialize_trajectory(robot_state, SkillType::JointPositionSkill);
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        }
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
    }
    time += period.toSec();
    traj_generator_->time_ = time;
    traj_generator_->dt_ = period.toSec();
    if(time > 0.0) {
      traj_generator_->get_next_step();
    }

    bool done = termination_handler_->should_terminate_on_franka(robot_state,
                                                                 traj_generator_);
    franka::JointPositions joint_desired(joint_trajectory_generator->get_desired_joints());

    log_counter += 1;
    if (log_counter % 1 == 0) {
      pose_desired = robot_state.O_T_EE_d;
      robot_state_data->log_robot_state(pose_desired, robot_state, time);
    }

    JointSensorInfo new_joint_sensor_info;

    SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager->readJointSensorInfoMessage(
        new_joint_sensor_info);
    if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {

    }


    if (done && time > 0.0) {
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        }
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      return franka::MotionFinished(joint_desired);
    }

    return joint_desired;
  };

  robot->robot_.control(joint_pose_callback);
}

