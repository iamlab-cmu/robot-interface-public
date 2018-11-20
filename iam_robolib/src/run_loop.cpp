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

void RunLoop::update() {
  std::cout << "In run loop update, will read from buffer\n";

  auto& buffer = *reinterpret_cast<SharedBuffer*>(region_1_.get_address());
  for (int i = 0; i < 10; i++) {
    std::cout << buffer[i] << ", ";
  }
  std::cout << "Read from buffer\n";
}

void RunLoop::run() {
  // Wait for sometime to let the client add data to the buffer
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(10s);

  auto milli = std::chrono::milliseconds(1);
  int t = 0;
  while (t < 10) {
    auto start = std::chrono::high_resolution_clock::now();
    update();
    auto finish = std::chrono::high_resolution_clock::now();
    // Wait for start + milli - finish
    auto elapsed = start + milli - finish;
    std::this_thread::sleep_for(elapsed);
    std::cout << "Waited " << elapsed.count() << " ms\n";
    t = t + 1;
  }
}
