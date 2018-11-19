#include <iam_robolib/run_loop.h>
#include <iam_robolib/duration.h>

#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <exception>
#include <fstream>
#include <iostream>


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
  std::cout << "Will start run loop.\n";
}

void RunLoop::stop() {
  // Maybe call this after exceptions or SIGINT or any Interrupt.
  // Stop the interface gracefully.
}

void RunLoop::update() {
  std::cout << "update run loop\n";
}

void RunLoop::run() {
  while (1) {
    update();
  }
}
