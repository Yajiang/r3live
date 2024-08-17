#include "slam_logger.h"

#include <iostream>

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
namespace slam {
namespace util {
Logger::Logger() {
    try {
        std::vector<spdlog::sink_ptr> sinks;
        if (is_console_log_enable_) {
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_level(spdlog::level::trace);
            sinks.push_back(console_sink);
        }
        if (is_save_log_file_) {
            auto rotate_sink =
                    std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file_path_, 1024 * 1024 * 5, 0);
            rotate_sink->set_level(spdlog::level::warn);
            sinks.push_back(rotate_sink);
        }
        logger_ = std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end());
    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "[Logger] Initialization failed: " << ex.what() << std::endl;
    }
}

Logger::~Logger() {
    spdlog::shutdown();
}

std::shared_ptr<spdlog::logger> Logger::GetLogger() {
    return logger_;
}

} // namespace util
} // namespace slam
