#include <mutex>

#pragma once

class ControlLoopData {
 public:
  	ControlLoopData(std::mutex &m): mutex_(m) {};

  	std::mutex& mutex_;
  	bool has_data_;

  	double time_;
  	int counter_; 

  	// Utils for printing
  	const int print_rate_=10;

  private:
 };