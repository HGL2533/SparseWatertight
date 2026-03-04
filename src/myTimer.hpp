#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>

class myTimer
{
public:
    myTimer()
    {
        start_time_ = clock::now();
        last_check_time_ = start_time_;
    }

    // 输出自上次checkTime或构造以来的时间
    void checkTime(const std::string& msg)
    {
        auto now = clock::now();
        printDuration(msg, last_check_time_, now);
        last_check_time_ = now;
    }

    // 输出自构造以来的总时间
    void TotalTime(const std::string& msg)
    {
        auto now = clock::now();
        printDuration(msg, start_time_, now);
    }

private:
    using clock = std::chrono::steady_clock;

    clock::time_point start_time_;
    clock::time_point last_check_time_;

    void printDuration(const std::string& msg,
        const clock::time_point& t1,
        const clock::time_point& t2)
    {
        auto duration = std::chrono::duration<double>(t2 - t1).count(); // 秒

        std::cout << msg << " : ";

        if (duration >= 1.0)
        {
            std::cout << std::fixed << std::setprecision(3)
                << duration << " s";
        }
        else
        {
            std::cout << std::fixed << std::setprecision(3)
                << duration * 1000.0 << " ms";
        }

        std::cout << std::endl;
    }
};