#include "logger.hpp"
#include <fstream>
#include <stdexcept>

namespace sysid {

void Logger::reserve(size_t num_frames, size_t num_joints) {
    num_joints_ = num_joints;
    buffer_.reserve(num_frames);
}

void Logger::push_frame(const Frame& frame) {
    // Optional safety check (can remove for max speed)
    if (frame.joints.size() != num_joints_) {
        throw std::runtime_error("Logger: frame.joints size mismatch");
    }
    buffer_.push_back(frame);
}

void Logger::write_csv(const std::string& path) const {
    std::ofstream out(path);
    if (!out.is_open())
        throw std::runtime_error("Logger: could not open file for writing");

    // Header
    out << "time,joint,input_torque,pos,vel,tor\n";

    for (size_t i = 0; i < buffer_.size(); ++i) {
        const auto& frame = buffer_[i];
        for (size_t j = 0; j < num_joints_; ++j) {
            const auto& s = frame.joints[j];
            out << s.time << ","
                << j << ","
                << s.input_torque << ","
                << s.position << ","
                << s.velocity << ","
                << s.torque << "\n";
        }
    }
}

} // namespace sysid
