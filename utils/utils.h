/******************************************************************************
 * File:             utils.h
 *
 * Author:           Akash Sharma
 * Created:          03/07/21
 * Description:      Common utility functions
 *****************************************************************************/
#ifndef DISTRIBUTED_SLAM_UTILS_H
#define DISTRIBUTED_SLAM_UTILS_H
#include <logging.h>
#include <fstream>
#include <iostream>
#include <string>
#include <type_traits>

#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>

template<class S, class C, typename = void>
struct is_printable : ::std::false_type
{
};

template<class S, class C>
struct is_printable<S, C, decltype(void(std::declval<S &>() << std::declval<C const &>()))> : ::std::true_type
{
};

template<typename S, typename C>
inline constexpr bool is_printable_v = is_printable<S, C>::value;

namespace utils
{
    inline void setLogPattern()
    {
        SET_LOG_LEVEL();
        spdlog::set_pattern("[src %s] [func %!] [line %#] %^[%l] %v%$");
        spdlog::set_error_handler([](const std::string &msg) -> void {
            std::cerr << "Error in SPDLOG: " << msg << std::endl;
            std::abort();
        });
    }

    template<class VecElem, typename = std::enable_if_t<is_printable_v<std::fstream, VecElem>>>
    void writeVecToFile(const std::vector<VecElem> &vec, const std::string &filename)
    {
        std::fstream stream(filename.c_str(), std::fstream::out);
        for (size_t i = 0; i < vec.size(); ++i)
        {
            stream << vec.at(i) << std::endl;
        }
        stream.close();
    }

    void writeMultiRobotG2o(const gtsam::NonlinearFactorGraph &graph,
                            const gtsam::Values &estimate,
                            const std::string &filename);

    inline std::string printKey(const gtsam::Key& key)
    {
        return fmt::format("{}{}", static_cast<char>(gtsam::symbolChr(key)), gtsam::symbolIndex(key));
    }

    const std::string robot_names = {"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"};  // robot names

}  // namespace utils
#endif /* ifndef DISTRIBUTED_SLAM_UTILS_H */
