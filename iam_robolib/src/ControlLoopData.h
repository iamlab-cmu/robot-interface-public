#include <mutex>

#pragma once

class ControlLoopData {
 public:
  	ControlLoopData(std::mutex &m): mutex_(m) {};

  	std::mutex& mutex_;
  	bool has_data_=false;

  	double time_=0;
  	int counter_=0;

  	// Utils for printing
  	const int print_rate_=10;

  private:
 };