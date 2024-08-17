#ifndef _UTIL_SLAM_LOGGER_H_
#define _UTIL_SLAM_LOGGER_H_

#include "singleton.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <iostream>

class Logger : public Singleton<Logger> {
public:
  Logger() {
    try {
      std::vector<spdlog::sink_ptr> sinks;
      if (m_IsConsoleLogEnable) {
        auto consoleSink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        consoleSink->set_level(spdlog::level::trace);
        sinks.push_back(consoleSink);
      }
      if (m_LogFilePath != "") {
        auto rotateSink =
            std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                m_LogFilePath, 1024 * 1024 * 5, 0);
        rotateSink->set_level(spdlog::level::warn);
        sinks.push_back(rotateSink);
      }

      m_Logger =
          std::make_unique<spdlog::logger>("", sinks.begin(), sinks.end());
    } catch (const spdlog::spdlog_ex &ex) {
      std::cout << "[Logger] Initialization failed: " << ex.what() << std::endl;
    }
  }

  ~Logger() { spdlog::shutdown(); }
  auto getLogger() -> spdlog::logger * { return m_Logger.get(); }

  std::unique_ptr<spdlog::logger> m_Logger;
private:
  std::string m_LogFilePath = "";
  bool m_IsConsoleLogEnable = true;
};

// with line number
#define RPA_TRACE(...)                                                         \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::trace, __VA_ARGS__)
#define RPA_DEBUG(...)                                                         \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::debug, __VA_ARGS__)
#define RPA_INFO(...)                                                          \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::info, __VA_ARGS__)
#define RPA_WARN(...)                                                          \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::warn, __VA_ARGS__)
#define RPA_ERROR(...)                                                         \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::err, __VA_ARGS__)
#define RPA_CRITICAL(...)                                                      \
  SPDLOG_LOGGER_CALL(Logger::getInstance()->getLogger(),                 \
                     spdlog::level::critical, __VA_ARGS__)

// without line number
// #define SLAM_TRACE(...)
// slam::util::Logger::GetInstance()->GetLogger()->trace(__VA_ARGS__) #define
// SLAM_DEBUG(...)
// slam::util::Logger::GetInstance()->GetLogger()->debug(__VA_ARGS__) #define
// SLAM_INFO(...)
// slam::util::Logger::GetInstance()->GetLogger()->info(__VA_ARGS__) #define
// SLAM_WARN(...)
// slam::util::Logger::GetInstance()->GetLogger()->warn(__VA_ARGS__) #define
// SLAM_ERROR(...)
// slam::util::Logger::GetInstance()->GetLogger()->error(__VA_ARGS__) #define
// SLAM_CRITICAL(...)
// slam::util::Logger::GetInstance()->GetLogger()->critical(__VA_ARGS__)
#endif