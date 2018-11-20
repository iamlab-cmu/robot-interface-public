#include <iam_robolib/run_loop.h>
#include <iam_robolib/duration.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>


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

}

void RunLoop::stop() {
  // Maybe call this after exceptions or SIGINT or any Interrupt.
  // Stop the interface gracefully.
}

void RunLoop::update() {
  std::cout << "update run loop\n";
}

void RunLoop::run() {
  auto milli = std::chrono::milliseconds(1);
  auto start = std::chrono::high_resolution_clock::now();
  int t = 0;
  while (t < 10) {
    start = std::chrono::high_resolution_clock::now();
    update();
    auto finish = std::chrono::high_resolution_clock::now();
    // Wait for start + milli - finish
    auto elapsed = start + milli - finish;
    std::this_thread::sleep_for(elapsed);
    std::cout << "Waited " << elapsed.count() << " ms\n";
    t = t + 1;
  }
}
