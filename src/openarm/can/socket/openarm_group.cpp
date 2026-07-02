#include <future>
#include <stdexcept>

#include <openarm/can/socket/openarm_group.hpp>

namespace openarm::can::socket {

OpenArmGroup::OpenArmGroup(const std::vector<std::string>& can_interfaces, bool enable_fd) {
    arms_.reserve(can_interfaces.size());

    for (const auto& can_interface : can_interfaces) {
        arms_.push_back(std::make_unique<OpenArm>(can_interface, enable_fd));
    }
}

OpenArm& OpenArmGroup::get_openarm(size_t index) {
    if (index >= arms_.size()) {
        throw std::out_of_range("OpenArmGroup index out of range");
    }
    return *arms_[index];
}

const OpenArm& OpenArmGroup::get_openarm(size_t index) const {
    if (index >= arms_.size()) {
        throw std::out_of_range("OpenArmGroup index out of range");
    }
    return *arms_[index];
}

OpenArm& OpenArmGroup::get_openarm(const std::string& can_interface) {
    for (auto& arm : arms_) {
        if (arm->can_interface() == can_interface) {
            return *arm;
        }
    }
    throw std::out_of_range("CAN interface not found in OpenArmGroup: " + can_interface);
}

const OpenArm& OpenArmGroup::get_openarm(const std::string& can_interface) const {
    for (const auto& arm : arms_) {
        if (arm->can_interface() == can_interface) {
            return *arm;
        }
    }
    throw std::out_of_range("CAN interface not found in OpenArmGroup: " + can_interface);
}

void OpenArmGroup::enable_all() {
    for (auto& arm : arms_) {
        arm->enable_all();
    }
}

void OpenArmGroup::disable_all() {
    for (auto& arm : arms_) {
        arm->disable_all();
    }
}

void OpenArmGroup::set_zero_all() {
    for (auto& arm : arms_) {
        arm->set_zero_all();
    }
}

std::vector<OpenArmRefreshResult> OpenArmGroup::refresh_all_and_recv(int timeout_us) {
    std::vector<std::future<OpenArmRefreshResult>> futures;
    futures.reserve(arms_.size());

    for (auto& arm_ptr : arms_) {
        futures.push_back(std::async(std::launch::async, [arm = arm_ptr.get(), timeout_us]() {
            OpenArmRefreshResult result;
            result.interface = arm->can_interface();

            try {
                result.expected = arm->expected_response_count();
                result.received = arm->refresh_all_and_recv(timeout_us);
                result.ok = (result.received == result.expected);
            } catch (const std::exception& e) {
                result.received = 0;
                result.expected = 0;
                result.ok = false;
                result.error = e.what();
            }

            return result;
        }));
    }

    std::vector<OpenArmRefreshResult> results;
    results.reserve(futures.size());

    for (auto& future : futures) {
        results.push_back(future.get());
    }

    return results;
}

}  // namespace openarm::can::socket