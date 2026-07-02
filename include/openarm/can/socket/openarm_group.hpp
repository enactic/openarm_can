#pragma once

#include <memory>
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

    size_t size() const noexcept { return arms_.size(); }

    OpenArm& get_openarm(size_t index);
    const OpenArm& get_openarm(size_t index) const;

    OpenArm& get_openarm(const std::string& can_interface);
    const OpenArm& get_openarm(const std::string& can_interface) const;

    void enable_all();
    void disable_all();
    void set_zero_all();

    std::vector<OpenArmRefreshResult> refresh_all_and_recv(int timeout_us = 500);

private:
    std::vector<std::unique_ptr<OpenArm>> arms_;
};

}  // namespace openarm::can::socket