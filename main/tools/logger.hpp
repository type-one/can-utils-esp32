
/**
 * @file logger.hpp
 * @brief Logging utility for different platforms (ESP32, others).
 *
 * This header file provides macros for logging messages with different severity levels.
 * It supports both ESP32 platform using ESP-IDF logging and other platforms using standard C++ I/O functions.
 *
 * @author Laurent Lardinois
 * @date January 2025
 */

//-----------------------------------------------------------------------------//
// C++ Publish/Subscribe Pattern - Spare time development for fun              //
// (c) 2025 Laurent Lardinois https://be.linkedin.com/in/laurentlardinois      //
//                                                                             //
// https://github.com/type-one/PublishSubscribeESP32                           //
//                                                                             //
// MIT License                                                                 //
//                                                                             //
// This software is provided 'as-is', without any express or implied           //
// warranty.In no event will the authors be held liable for any damages        //
// arising from the use of this software.                                      //
//                                                                             //
// Permission is granted to anyone to use this software for any purpose,       //
// including commercial applications, and to alter itand redistribute it       //
// freely, subject to the following restrictions :                             //
//                                                                             //
// 1. The origin of this software must not be misrepresented; you must not     //
// claim that you wrote the original software.If you use this software         //
// in a product, an acknowledgment in the product documentation would be       //
// appreciated but is not required.                                            //
// 2. Altered source versions must be plainly marked as such, and must not be  //
// misrepresented as being the original software.                              //
// 3. This notice may not be removed or altered from any source distribution.  //
//-----------------------------------------------------------------------------//

#pragma once

#if !defined(LOGGER_HPP_)
#define LOGGER_HPP_

#include <filesystem>
#include <string>

#if (__cplusplus >= 202002L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 202002L))
#include <experimental/source_location>
#endif

#if defined(ESP_PLATFORM)
#if defined(DEBUG)
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#else
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif
#include <esp_log.h>
#else
#include <cstdio>
#include <cstdlib>
#include <map>
#endif

#if defined(__func__)
#define FUNCTION_ALIAS_ __func__
#else
#define FUNCTION_ALIAS_ __FUNCTION__
#endif

#if defined(__FILE_BASENAME__)
#define FILE_ALIAS_ __FILE_BASENAME__
#define FILE_ALIAS_STR                                                                                                 \
    std::string                                                                                                        \
    {                                                                                                                  \
        FILE_ALIAS_                                                                                                    \
    }
#elif defined(__FILE_NAME__)
#define FILE_ALIAS_ __FILE_NAME__
#define FILE_ALIAS_STR                                                                                                 \
    std::string                                                                                                        \
    {                                                                                                                  \
        FILE_ALIAS_                                                                                                    \
    }
#else
#define FILE_ALIAS_STR std::filesystem::path(__FILE__).filename()
#define FILE_ALIAS_ (FILE_ALIAS_STR).c_str()
#endif


#if defined(ESP_PLATFORM)
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html
// ESP_LOGE - Error (lowest)
// ESP_LOGW - Warning
// ESP_LOGI - Info
// ESP_LOGD - Debug
// ESP_LOGV - Verbose (highest)
#define LOG_ERROR(...)                                                                                                 \
    ESP_LOGE((FILE_ALIAS_STR + " [" + FUNCTION_ALIAS_ + ", line " + std::to_string(__LINE__) + "] ").c_str(),          \
        __VA_ARGS__) // NOLINT
#define LOG_WARNING(...)                                                                                               \
    ESP_LOGW((FILE_ALIAS_STR + " [" + FUNCTION_ALIAS_ + ", line " + std::to_string(__LINE__) + " ]").c_str(),          \
        __VA_ARGS__) // NOLINT
#define LOG_INFO(...)                                                                                                  \
    ESP_LOGI((FILE_ALIAS_STR + " [" + FUNCTION_ALIAS_ + ", line " + std::to_string(__LINE__) + "] ").c_str(),          \
        __VA_ARGS__) // NOLINT
#define LOG_DEBUG(...)                                                                                                 \
    ESP_LOGD((FILE_ALIAS_STR + " [" + FUNCTION_ALIAS_ + ", line " + std::to_string(__LINE__) + "] ").c_str(),          \
        __VA_ARGS__) // NOLINT
#define LOG_VERBOSE(...)                                                                                               \
    ESP_LOGV((FILE_ALIAS_STR + " [" + FUNCTION_ALIAS_ + ", line " + std::to_string(__LINE__) + "] ").c_str(),          \
        __VA_ARGS__) // NOLINT

//#elif defined(STM32_PLATFORM)
// TODO
// https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32CubeWBA_Trace_management

#else


#if (__cplusplus >= 202002L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 202002L))
namespace tools
{
    // https://stackoverflow.com/questions/57547273/how-to-use-source-location-in-a-variadic-template-function
    // https://github.com/gabime/spdlog/issues/1959

    using source_location = std::experimental::source_location;

    /**
     * @brief logging level
     *
     */
    enum class log_level
    {
        error,
        warning,
        info,
        debug,
        verbose
    };

    /**
     * @brief capture of source code location info
     *
     */
    struct format_with_location
    {
        const char* value = nullptr;
        source_location loc = {};

        format_with_location(const char* format_str,
            const source_location& location = source_location::current()) // NOLINT location at caller level
            : value(format_str)
            , loc(location)
        {
        }
    };

    /**
     * @brief variadic templated logging function
     *
     * @tparam Args
     * @param level log level
     * @param fmt C-like printf format string
     * @param args variadic printf arguments
     */
    template <typename... Args>
    void log(tools::log_level level, tools::format_with_location fmt, Args&&... args)
    {
        // https://dev.to/tenry/terminal-colors-in-c-c-3dgc
#if defined(__linux__)
        // https://man7.org/linux/man-pages/man5/terminal-colors.d.5.html
        static const std::map<log_level, std::string> lookup = { { log_level::error, "\033[31m[ERROR]" },
            { log_level::warning, "\033[33m[WARNING]" }, { log_level::info, "\033[32m[INFO]" },
            { log_level::debug, "\033[36m[DEBUG]" }, { log_level::verbose, "\033[34m[VERBOSE]" } };
        static const std::string end_restore { "\033[0m" };
#else
        static const std::map<log_level, std::string> lookup
            = { { log_level::error, "[ERROR]" }, { log_level::warning, "[WARNING]" }, { log_level::info, "[INFO]" },
                  { log_level::debug, "[DEBUG]" }, { log_level::verbose, "[VERBOSE]" } };
        static const std::string end_restore { "\0" };
#endif

        FILE* output = (level >= log_level::error) && (level <= log_level::warning) ? stderr : stdout;

        std::fprintf(output, "%s %s [%s, line %d] ", lookup.at(level).c_str(),
            std::filesystem::path(fmt.loc.file_name()).filename().c_str(), fmt.loc.function_name(), fmt.loc.line());
        std::fprintf(output, fmt.value, args...); // NOLINT source_location returns const char*
        std::fprintf(output, "%s\n", end_restore.c_str());
        std::fflush(output);
    }

}

#define LOG_ERROR(...) tools::log(tools::log_level::error, __VA_ARGS__)     // NOLINT
#define LOG_WARNING(...) tools::log(tools::log_level::warning, __VA_ARGS__) // NOLINT
#define LOG_INFO(...) tools::log(tools::log_level::info, __VA_ARGS__)       // NOLINT

#if defined(DEBUG)
#define LOG_DEBUG(...) tools::log(tools::log_level::debug, __VA_ARGS__)     // NOLINT
#define LOG_VERBOSE(...) tools::log(tools::log_level::verbose, __VA_ARGS__) // NOLINT
#else
#define LOG_DEBUG(...)
#define LOG_VERBOSE(...)
#endif

#else

// pre-C++20

// clang-format off
#define LOG_ERROR(...) do { std::fprintf(stderr, "[ERROR] %s [%s, line %d] ", FILE_ALIAS_, FUNCTION_ALIAS_, __LINE__); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); std::fflush(stderr); } while(false)     // NOLINT
#define LOG_WARNING(...) do { std::fprintf(stderr, "[WARNING] %s [%s, line %d] ", FILE_ALIAS_, FUNCTION_ALIAS_, __LINE__); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); std::fflush(stderr); } while(false) // NOLINT
#define LOG_INFO(...) do { std::printf("[INFO] %s [%s, line %d] ", FILE_ALIAS_, FUNCTION_ALIAS_, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); std::fflush(stdout); } while(false) // NOLINT

#if defined(DEBUG)
#define LOG_DEBUG(...) do { std::printf("[DEBUG] %s [%s, line %d] ", FILE_ALIAS_, FUNCTION_ALIAS_, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); std::fflush(stdout); } while(false)     // NOLINT
#define LOG_VERBOSE(...) do { std::printf("[VERBOSE] %s [%s, line %d] ", FILE_ALIAS_, FUNCTION_ALIAS_, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); std::fflush(stdout); } while(false) // NOLINT
#else
#define LOG_DEBUG(...)
#define LOG_VERBOSE(...)
#endif
// clang-format on

#endif // end pre-C++20

#endif // end std implem

#endif //  LOGGER_HPP_
