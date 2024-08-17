#ifndef SLAM_CORE_UTIL_SLAM_LOGGER_H_
#define SLAM_CORE_UTIL_SLAM_LOGGER_H_

#include "parameters.h"
#include "singleton.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

namespace slam {
namespace util {
class Logger : public SingletonT<Logger> {
  public:
    Logger();

    ~Logger();

    std::shared_ptr<spdlog::logger> GetLogger();

  private:
    std::shared_ptr<spdlog::logger> logger_;
#ifdef __aarch64__
    std::string log_file_path_ = "/opt/log/slam_log.txt";
#else
    std::string log_file_path_ = global_params::output_path + "/slam_log.txt";
#endif
    bool is_console_log_enable_ = true;
    bool is_save_log_file_ = true;
};

} // namespace util
} // namespace slam

// with line number
#define SLAM_TRACE(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::trace, __VA_ARGS__)
#define SLAM_DEBUG(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::debug, __VA_ARGS__)
#define SLAM_INFO(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::info, __VA_ARGS__)
#define SLAM_WARN(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::warn, __VA_ARGS__)
#define SLAM_ERROR(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::err, __VA_ARGS__)
#define SLAM_CRITICAL(...) \
    SPDLOG_LOGGER_CALL(slam::util::Logger::GetInstance()->GetLogger().get(), spdlog::level::critical, __VA_ARGS__)

// without line number
// #define SLAM_TRACE(...)       slam::util::Logger::GetInstance()->GetLogger()->trace(__VA_ARGS__)
// #define SLAM_DEBUG(...)       slam::util::Logger::GetInstance()->GetLogger()->debug(__VA_ARGS__)
// #define SLAM_INFO(...)        slam::util::Logger::GetInstance()->GetLogger()->info(__VA_ARGS__)
// #define SLAM_WARN(...)        slam::util::Logger::GetInstance()->GetLogger()->warn(__VA_ARGS__)
// #define SLAM_ERROR(...)       slam::util::Logger::GetInstance()->GetLogger()->error(__VA_ARGS__)
// #define SLAM_CRITICAL(...)    slam::util::Logger::GetInstance()->GetLogger()->critical(__VA_ARGS__)
#endif