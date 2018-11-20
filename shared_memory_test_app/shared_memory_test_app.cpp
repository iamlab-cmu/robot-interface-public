#include <iostream>
#include <thread>
#include <array>
#include <chrono>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

using SharedBuffer = std::array<float, 1024>;

int main() {
  std::cout << "Hello world\n";
  boost::interprocess::managed_shared_memory segment(
      boost::interprocess::open_only,
      "run_loop_shared_memory_1"); //Shared memory object name

  boost::interprocess::shared_memory_object shm_1(
          boost::interprocess::open_only,       // open 
          "run_loop_shared_obj_1",              // name
          boost::interprocess::read_write       // read-only mode
  );

  //Map the whole shared memory in this process
  boost::interprocess::mapped_region region(shm_1, boost::interprocess::read_write);

  // char *mem = static_cast<char*>(region.get_address());
  auto& buffer = *reinterpret_cast<SharedBuffer*>(region.get_address());

  int t = 0;
  while (1) {
    for (int i = 0; i < 10; i++) {
      buffer[i] = i + t;
    }
    t = t + 10;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(2s);
  }

  return 0;
}
