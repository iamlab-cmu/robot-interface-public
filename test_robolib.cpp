#include<iostream>

#include <iam_robolib/run_loop.h>

int main() {
  std::cout << "Hello world\n";
  RunLoop run_loop = RunLoop();
  std::cout << "Will start run loop.\n";
  run_loop.start();
  std::cout << "Did start run loop.\n";
  std::cout << "Will run..\n";
  run_loop.run();
  return 0;
}
