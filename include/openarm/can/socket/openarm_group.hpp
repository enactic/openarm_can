#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "openarm.hpp"

namespace openarm::can::socket {

struct OpenArmRefreshResult {
    std::string interface;
    int received = 0;
    int expected = 0;
    bool ok = false;
    std::string error;
};

class OpenArmGroup {
public:
    OpenArmGroup(const std::vector<std::string>& can_interfaces, bool enable_fd = false);
    ~OpenArmGroup();

    OpenArmGroup(const OpenArmGroup&) = delete;
    OpenArmGroup& operator=(const OpenArmGroup&) = delete;
    OpenArmGroup(OpenArmGroup&&) = delete;
    OpenArmGroup& operator=(OpenArmGroup&&) = delete;

    std::size_t size() const noexcept { return workers_.size(); }

    OpenArm& get_openarm(std::size_t index);
    const OpenArm& get_openarm(std::size_t index) const;

    OpenArm& get_openarm(const std::string& can_interface);
    const OpenArm& get_openarm(const std::string& can_interface) const;

    void enable_all();
    void disable_all();
    void set_zero_all();

    std::vector<OpenArmRefreshResult> refresh_all_and_recv(int timeout_us = 500);

private:
    struct Worker;

    std::vector<std::unique_ptr<Worker>> workers_;
    mutable std::mutex api_mutex_;
};

}  // namespace openarm::can::socket