#include<iostream>

#include <iam_robolib/run_loop.h>
#include <mutex>

int main() {
  std::cout << "Hello world\n";
  std::mutex m;
  RunLoop run_loop = RunLoop(std::ref(m));
  std::cout << "Will start run loop.\n";
  run_loop.start();
  std::cout << "Did start run loop.\n";
  std::cout << "Will run..\n";
  run_loop.run_on_franka();
  return 0;
}
