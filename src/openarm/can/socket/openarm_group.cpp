#include <condition_variable>
#include <exception>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include <openarm/can/socket/openarm_group.hpp>

namespace openarm::can::socket {

struct OpenArmGroup::Worker {
    explicit Worker(std::unique_ptr<OpenArm> arm_) : arm(std::move(arm_)) {}

    ~Worker() {
        stop_and_join();
    }

    Worker(const Worker&) = delete;
    Worker& operator=(const Worker&) = delete;
    Worker(Worker&&) = delete;
    Worker& operator=(Worker&&) = delete;

    void start() {
        thread = std::thread([this]() { run(); });
    }

    void request_stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
            request = false;
        }

        cv.notify_one();
    }

    void join() noexcept {
        if (thread.joinable()) {
            thread.join();
        }
    }

    void stop_and_join() noexcept {
        request_stop();
        join();
    }

    void request_refresh(int new_timeout_us) {
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (stop) {
                throw std::runtime_error("OpenArmGroup worker is stopped");
            }

            timeout_us = new_timeout_us;
            done = false;
            request = true;
        }

        cv.notify_one();
    }

    OpenArmRefreshResult wait_result() {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this]() { return done; });
        return result;
    }

    void run() noexcept {
        while (true) {
            int local_timeout_us = 0;

            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this]() { return stop || request; });

                if (stop) {
                    return;
                }

                local_timeout_us = timeout_us;
                request = false;
            }

            OpenArmRefreshResult local_result = execute_refresh(local_timeout_us);

            {
                std::lock_guard<std::mutex> lock(mutex);
                result = std::move(local_result);
                done = true;
            }

            cv.notify_all();
        }
    }

    OpenArmRefreshResult execute_refresh(int local_timeout_us) noexcept {
        OpenArmRefreshResult local_result;
        local_result.interface = arm->can_interface();

        try {
            local_result.expected = arm->expected_response_count();
            local_result.received = arm->refresh_all_and_recv(local_timeout_us);
            local_result.ok = (local_result.received == local_result.expected);
        } catch (const std::exception& e) {
            local_result.ok = false;
            local_result.error = e.what();
        } catch (...) {
            local_result.ok = false;
            local_result.error = "Unknown exception in OpenArmGroup worker";
        }

        return local_result;
    }

    std::unique_ptr<OpenArm> arm;
    std::thread thread;
    std::mutex mutex;
    std::condition_variable cv;

    bool stop = false;
    bool request = false;
    bool done = true;

    int timeout_us = 500;
    OpenArmRefreshResult result;
};

OpenArmGroup::OpenArmGroup(const std::vector<std::string>& can_interfaces, bool enable_fd) {
    std::set<std::string> seen_interfaces;

    for (const auto& can_interface : can_interfaces) {
        if (!seen_interfaces.insert(can_interface).second) {
            throw std::invalid_argument(
                "Duplicate CAN interface in OpenArmGroup: " + can_interface);
        }
    }

    workers_.reserve(can_interfaces.size());

    for (const auto& can_interface : can_interfaces) {
        workers_.push_back(
            std::make_unique<Worker>(std::make_unique<OpenArm>(can_interface, enable_fd)));
    }

    for (auto& worker : workers_) {
        worker->start();
    }
}

OpenArmGroup::~OpenArmGroup() {
    for (auto& worker : workers_) {
        worker->request_stop();
    }

    for (auto& worker : workers_) {
        worker->join();
    }
}

OpenArm& OpenArmGroup::get_openarm(std::size_t index) {
    std::lock_guard<std::mutex> lock(api_mutex_);

    if (index >= workers_.size()) {
        throw std::out_of_range("OpenArmGroup index out of range");
    }

    return *workers_[index]->arm;
}

const OpenArm& OpenArmGroup::get_openarm(std::size_t index) const {
    std::lock_guard<std::mutex> lock(api_mutex_);

    if (index >= workers_.size()) {
        throw std::out_of_range("OpenArmGroup index out of range");
    }

    return *workers_[index]->arm;
}

OpenArm& OpenArmGroup::get_openarm(const std::string& can_interface) {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        if (worker->arm->can_interface() == can_interface) {
            return *worker->arm;
        }
    }

    throw std::out_of_range("CAN interface not found in OpenArmGroup: " + can_interface);
}

const OpenArm& OpenArmGroup::get_openarm(const std::string& can_interface) const {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (const auto& worker : workers_) {
        if (worker->arm->can_interface() == can_interface) {
            return *worker->arm;
        }
    }

    throw std::out_of_range("CAN interface not found in OpenArmGroup: " + can_interface);
}

void OpenArmGroup::enable_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->arm->enable_all();
    }
}

void OpenArmGroup::disable_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->arm->disable_all();
    }
}

void OpenArmGroup::set_zero_all() {
    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->arm->set_zero_all();
    }
}

std::vector<OpenArmRefreshResult> OpenArmGroup::refresh_all_and_recv(int timeout_us) {
    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    std::lock_guard<std::mutex> lock(api_mutex_);

    for (auto& worker : workers_) {
        worker->request_refresh(timeout_us);
    }

    std::vector<OpenArmRefreshResult> results;
    results.reserve(workers_.size());

    for (auto& worker : workers_) {
        results.push_back(worker->wait_result());
    }

    return results;
}

}  // namespace openarm::can::socket