#include <chrono>
#include <iostream>

struct CostSecond {
  std::chrono::_V2::system_clock::time_point start_timestamp;
  std::chrono::_V2::system_clock::time_point end_timestamp;
  CostSecond() { start_timestamp = std::chrono::system_clock::now(); }

  ~CostSecond() {
    end_timestamp = std::chrono::system_clock::now();
    double cost_second =
        std::chrono::duration<double>(end_timestamp - start_timestamp).count();

    std::cout << "cost(s): " << cost_second << std::endl;
  }
};

struct CostMillisecond {
  std::chrono::_V2::system_clock::time_point start_timestamp;
  std::chrono::_V2::system_clock::time_point end_timestamp;
  CostMillisecond() { start_timestamp = std::chrono::system_clock::now(); }

  ~CostMillisecond() {
    end_timestamp = std::chrono::system_clock::now();
    double cost_milli = std::chrono::duration<double, std::milli>(
                            end_timestamp - start_timestamp)
                            .count();

    std::cout << "cost(ms): " << cost_milli << std::endl;
  }
};

struct CostMicrosecond {
  std::chrono::_V2::system_clock::time_point start_timestamp;
  std::chrono::_V2::system_clock::time_point end_timestamp;
  CostMicrosecond() { start_timestamp = std::chrono::system_clock::now(); }

  ~CostMicrosecond() {
    end_timestamp = std::chrono::system_clock::now();
    double cost_micro = std::chrono::duration<double, std::micro>(
                            end_timestamp - start_timestamp)
                            .count();

    std::cout << "cost(micro second): " << cost_micro << std::endl;
  }
};
