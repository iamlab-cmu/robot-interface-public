#include <iam_robolib/run_loop.h>
#include <iam_robolib/duration.h>
#include <iam_robolib/run_loop_process_info.h>

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

using SharedBuffer = std::array<float, 1024>;

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

  // Create shared memory here.
  boost::interprocess::shared_memory_object::remove("run_loop_shared_memory_1");
  managed_shared_memory_1_ = boost::interprocess::managed_shared_memory(
          boost::interprocess::create_only,
          "run_loop_shared_memory_1",
          4 * 1024);

  boost::interprocess::shared_memory_object::remove("run_loop_shared_memory_2");
  managed_shared_memory_2_ = boost::interprocess::managed_shared_memory(
          boost::interprocess::create_only,
          "run_loop_shared_memory_2",
          4 * 1024
  );

  // Add run loop process info to the main loop.
  run_loop_info_ = managed_shared_memory_1_.construct<RunLoopProcessInfo>
          ("run_loop_info")
          (1);
  run_loop_info_mutex_ = managed_shared_memory_1_.construct<boost::interprocess::interprocess_mutex>
          ("run_loop_info_mutex")
          ();

  // Add the inter-process mutex into memory. We will grab this each time we want
  // to update anything in the memory.


  // Create shared memory object.
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

void RunLoop::finish_current_task() {
  {
    boost::interprocess::scoped_lock<
            boost::interprocess::interprocess_mutex> lock(*run_loop_info_mutex_);
    try {
      if (lock.try_lock()) {
        run_loop_info_->is_running_task_ = false;
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
    if (skill != 0) {
      skill->execute_skill();

      SkillStatus status = skill->get_current_skill_status();

      if (status == SkillStatus::FINISHED) {
        finish_current_task();
      }
    }

    auto finish = std::chrono::high_resolution_clock::now();
    // Wait for start + milli - finish
    auto elapsed = start + milli - finish;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
