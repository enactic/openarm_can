#pragma once
#include <vector>
#include <string>

namespace sysid {

struct Sample {
    double input_torque;
    double position;
    double velocity;
    double torque;
};

class Logger {
public:
    Logger(const std::string& filename, size_t num_motors);
    ~Logger();

    void log(double time, const std::vector<Sample>& motors);
    void flush();

private:
    std::ofstream file_;
    size_t num_motors_;
};

} // namespace sysid
