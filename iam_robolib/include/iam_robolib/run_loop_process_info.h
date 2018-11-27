//
// Created by mohit on 11/20/18.
//

#pragma once

#include <cassert>
#include <string>

class RunLoopProcessInfo {
 public:
  RunLoopProcessInfo(int memory_region_idx): current_memory_region_(memory_region_idx) {};

  void set_new_skill_available(bool new_skill_available);

  bool get_new_skill_available();

  void set_is_running_skill(bool is_running_skill);

  bool get_is_running_skill();

  void set_skill_done(bool skill_done);

  bool get_skill_done();

  void set_skill_preempted(bool skill_preempted);

  bool get_skill_preempted();

/**
 * Memory index being used by the run loop.
 */
  int get_current_shared_memory_index();

/**
 * Memory index being used by the actionlib.
 */
  int get_current_free_shared_memory_index();

/**
 * Update shared memory region.
 */
  void update_shared_memory_region();

/**
 * Sensor index being used by the run loop.
 */
  int get_current_shared_sensor_index();

/**
 * Sensor index being used by the actionlib.
 */
  int get_current_free_shared_sensor_index();

/**
 * Update shared sensor region.
 */
  void update_shared_sensor_region();

/**
 * Feedback index being used by the run loop.
 */
  int get_current_shared_feedback_index();

/**
 * Feedback index being used by the actionlib.
 */
  int get_current_free_shared_feedback_index();

/**
 * Update shared feedback region.
 */
  void update_shared_feedback_region();

  bool can_run_new_skill();

/**
 * Return the id for the latest skill available.
 */
  int get_new_skill_id();

/**
 * Update current skill being executed.
 */
  void update_current_skill(int new_skill_id);

/**
 * Update new skill id. Written from actionlib.
 */
  void update_new_skill(int new_skill_id);

 private:
  bool new_skill_available_{false};
  bool is_running_skill_{false};
  bool skill_done_{false};
  bool skill_preempted_{false};

  int current_memory_region_{1};
  int current_sensor_region_{1};
  int current_feedback_region_{1};
  int current_skill_id_{-1};
  int new_skill_id_{-1};
};
