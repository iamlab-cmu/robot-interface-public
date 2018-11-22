#include <iam_robolib/run_loop.h>
#include <iam_robolib/duration.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "counter_trajectory_generator.h"


void setCurrentThreadToRealtime(bool throw_on_error) {
  // Change prints to exceptions.
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    std::cout << std::string("libfranka: unable to get maximum possible thread priority: ") +
        std::strerror(errno);
  }
  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      std::cout << std::string("libfranka: unable to set realtime scheduling: ") +
          std::strerror(errno);
    }
  }
}

bool RunLoop::init() {
  // TODO(Mohit): Initialize memory and stuff.
  bool throw_on_error;
  setCurrentThreadToRealtime(throw_on_error);
}

void RunLoop::start() {
  // Start processing, might want to do some pre-processing 
  std::cout << "start run loop.\n";

  // Create managed shared memory (segments) here.
  boost::interprocess::shared_memory_object::remove("run_loop_shared_memory_1");
  managed_shared_memory_ = boost::interprocess::managed_shared_memory(
          boost::interprocess::create_only,
          "run_loop_shared_memory",
          4 * 1024);
  // Add run loop process info to the main loop.
  run_loop_info_ = managed_shared_memory_.construct<RunLoopProcessInfo>
          ("run_loop_info")
          (1);
  // Add the inter-process mutex into memory. We will grab this each
  // time we want to update anything in the memory.
  run_loop_info_mutex_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
          ("run_loop_info_mutex")
          ();


  // Create shared memory objects.
  /* TODO(Mohit): Maybe we shold be using shared memory object instead of
   * managed managed shared memory.*/
  shared_memory_object_1_ = boost::interprocess::shared_memory_object(
          boost::interprocess::open_or_create,  // open or create
          "run_loop_shared_obj_1",              // name
          boost::interprocess::read_write       // read-only mode
  );

  // Allocate memory
  shared_memory_object_1_.truncate(8 * 1024);

  // TODO(Mohit): We can create multiple regions, each of which will hold
  // data for different types, e.g. trajectory generator params,
  // controller params, etc.
  // Map the region
  region_1_ =  boost::interprocess::mapped_region(
          shared_memory_object_1_,              // Memory-mappable object
          boost::interprocess::read_write,      // Access mode
          0,                                    // Offset from the beginning of shm
          8 * 1024                              // Length of the region
  );
  traj_gen_buffer_1_ = *reinterpret_cast<SharedBuffer*>(
      region_1_.get_address());

  shared_memory_object_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,  // open or create
      "run_loop_shared_obj_0",              // name
      boost::interprocess::read_write       // read-only mode
  );

  // Allocate memory
  shared_memory_object_0_.truncate(8 * 1024);

  // TODO(Mohit): We can create multiple regions, each of which will hold
  // data for different types, e.g. trajectory generator params,
  // controller params, etc.
  // Map the region
  region_0_ =  boost::interprocess::mapped_region(
      shared_memory_object_0_,              // Memory-mappable object
      boost::interprocess::read_write,      // Access mode
      0,                                    // Offset from the beginning of shm
      8 * 1024                              // Length of the region
  );
  traj_gen_buffer_0_ = *reinterpret_cast<SharedBuffer*>(
      region_0_.get_address());


}

TrajectoryGenerator* RunLoop::get_trajectory_generator_for_skill(
    int memory_region) {

  SharedBuffer buffer;
  if (memory_region == 0) {
    buffer = traj_gen_buffer_0_;
  } else {
    buffer = traj_gen_buffer_1_;
  }
  int traj_gen_id = buffer[0];

  if (traj_gen_id == 1) {
    // Create Counter based trajectory.
    CounterTrajectoryGenerator *traj_generator = \
      new CounterTrajectoryGenerator(buffer);
    traj_generator->parse_parameters();
    return traj_generator;
  } else {
    // Cannot create Trajectory generator for this skill. Throw error
    std::cout << "Cannot generate trajectory generator. Should throw exception."
        << std::endl;
    return 0;
  }
}

void RunLoop::stop() {
  // Maybe call this after exceptions or SIGINT or any Interrupt.
  // Stop the interface gracefully.
}

bool RunLoop::update() {

  std::cout << "In run loop update, will read from buffer\n";

  // Pointers might be a bit slow??
  // (Well we should be caching these so hopefully not)
  SkillInfo *skill = skill_manager_.get_current_skill();
  if (skill != 0) {
    skill->execute_skill();
  }

  auto& buffer = *reinterpret_cast<SharedBuffer*>(region_1_.get_address());
  for (int i = 0; i < 10; i++) {
     if (buffer[i] == -1) {
       // Task finished
       return false;
     }
    std::cout << buffer[i] << ", ";
  }
  std::cout << "Read from buffer\n";

  return true;
}

void RunLoop::start_new_task() {
}

void RunLoop::update_process_info() {
  bool is_executing_skill = skill_manager_.is_currently_executing_skill();
  {
    boost::interprocess::scoped_lock<
            boost::interprocess::interprocess_mutex> lock(*run_loop_info_mutex_);
    try {
      if (lock.try_lock()) {
        run_loop_info_->is_running_task_ = is_executing_skill;
        process_info_requires_update_ = false;

        // Check if new task is available
        if (run_loop_info_->new_task_available_) {
          // Get the parameters
          // Create new task Skill
          int new_skill_id = run_loop_info_->get_new_skill_id();
          run_loop_info_->update_current_skill(new_skill_id);

          // Add new skill
          SkillInfo *new_skill = new SkillInfo(new_skill_id);
          skill_manager_.add_skill(new_skill);

          // Update the shared memory region. This means that the actionlib service
          // will now write to the other memory region, i.e. not the current memory
          // region.
          run_loop_info_->update_shared_memory_region();
        }
      }
    } catch (boost::interprocess::lock_exception) {
      // TODO(Mohit): Do something better here.
      std::cout << "Cannot acquire lock for run loop info";
    }
  }

}

void RunLoop::run() {
  // Wait for sometime to let the client add data to the buffer
  std::this_thread::sleep_for(std::chrono::seconds(10));

  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  auto milli = std::chrono::milliseconds(1);

  while (1) {
    start = std::chrono::high_resolution_clock::now();

    // Execute the current skill (traj_generator, FBC are here)
    SkillInfo *skill = skill_manager_.get_current_skill();

    // NOTE: We keep on running the last skill even if it is finished!!
    if (skill != 0) {

      skill->execute_skill();
      SkillStatus status = skill->get_current_skill_status();

      if (status == SkillStatus::FINISHED) {
        process_info_requires_update_ = true;
      }
    }

    // Complete old skills and aquire new skills
    update_process_info();

    SkillInfo *new_skill = skill_manager_.get_current_skill();
    // Found new skill. Let's initialize it.
    if (new_skill != 0) {
      if ((skill != 0 && new_skill->get_skill_id() != skill->get_skill_id())
       ||  skill == 0) {
        // Generate things that are required here.
        TrajectoryGenerator *traj_generator = \
            get_trajectory_generator_for_skill(
                run_loop_info_->get_current_shared_memory_index());

        // Start skill, does any pre-processing if required.
        new_skill->start_skill(traj_generator);
      }
    }


    auto finish = std::chrono::high_resolution_clock::now();
    // Wait for start + milli - finish
    auto elapsed = start + milli - finish;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
