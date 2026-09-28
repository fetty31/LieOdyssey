#pragma once

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>
#include <fstream>
#include <unistd.h>

#ifndef LIO_PROFILE
#define LIO_PROFILE 0
#endif

namespace lio_ros
{

class Profiler
{
public:

    struct Stats
    {
        double last_ms = 0.0;
        double avg_ms  = 0.0;
        double max_ms  = 0.0;
        uint64_t calls = 0;
    };

public:

    explicit Profiler(std::string name = "PROFILER")
        : name_(std::move(name))
    {
        start_system_monitor();
    }

    ~Profiler()
    {
        running_ = false;
        if (monitor_thread_.joinable())
            monitor_thread_.join();
    }

public:

    void add_sample(const std::string& key, double ms)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        auto& s = stats_[key];

        // first sample init
        if (s.calls == 0)
        {
            s.last_ms = ms;
            s.avg_ms  = ms;
            s.max_ms  = ms;
            s.calls   = 1;
            return;
        }

        s.last_ms = ms;
        s.calls++;

        // incremental mean 
        s.avg_ms += (ms - s.avg_ms) / static_cast<double>(s.calls);

        // running max
        if (ms > s.max_ms)
            s.max_ms = ms;
    }

public:

    std::string str() const
    {
        std::lock_guard<std::mutex> lock(mtx_);

        std::ostringstream oss;

        oss << "\n+--------------------------------------------------+\n";
        oss << "| " << std::setw(20) << std::left << name_ << "\n";
        oss << "+--------------------------------------------------+\n";

        oss << std::fixed << std::setprecision(3);

        for (const auto& [name, s] : stats_)
        {
            oss << "| "
                << std::setw(30) << std::left << name.substr(0, 30)

                << " (ms) | last: "
                << std::setw(10) << std::right << s.last_ms

                << " | avg: "
                << std::setw(10) << std::right << s.avg_ms

                << " | max: "
                << std::setw(10) << std::right << s.max_ms

                << " |\n";
        }

        oss << "|--------------------------------------------------|\n";

        oss << "| CPU Load : " << std::setw(8) << cpu_percent_.load() << " %\n";
        oss << "| RAM Usage: " << std::setw(8) << ram_mb_.load()     << " MB\n";

        oss << "+--------------------------------------------------+\n";

        return oss.str();
    }

    void print() const
    {
        std::cout << str();
    }

public:

    double cpu() const { return cpu_percent_.load(); }
    double ram() const { return ram_mb_.load(); }

private:

    void start_system_monitor()
    {
        num_cores_ = std::thread::hardware_concurrency();
        if (num_cores_ == 0) num_cores_ = 1;

        running_ = true;

        monitor_thread_ = std::thread([this]()
        {
            while (running_)
            {
                update_ram();
                update_cpu();

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500));
            }
        });
    }

    void update_ram()
    {
        std::ifstream stat_stream("/proc/self/stat");

        std::string tmp;
        for (int i = 0; i < 22; i++)
            stat_stream >> tmp;

        unsigned long vsize;
        long rss;
        stat_stream >> vsize >> rss;

        long page_kb = sysconf(_SC_PAGE_SIZE) / 1024;

        double ram = rss * page_kb / 1024.0;

        ram_mb_.store(ram);
    }

    long get_total_cpu_time()
    {
        std::ifstream file("/proc/stat");

        std::string cpu;
        long user, nice, system, idle, iowait, irq, softirq, steal;

        file >> cpu >> user >> nice >> system >> idle
            >> iowait >> irq >> softirq >> steal;

        return user + nice + system + idle +
            iowait + irq + softirq + steal;
    }

    long get_process_cpu_time()
    {
        std::ifstream file("/proc/self/stat");

        std::string tmp;

        // skip first 13 fields safely
        for (int i = 0; i < 13; i++)
            file >> tmp;

        long utime, stime;
        file >> utime >> stime;

        return utime + stime;
    }

    void update_cpu()
    {
        long proc_time = get_process_cpu_time();
        long sys_time  = get_total_cpu_time();

        if (last_sys_time_ == 0)
        {
            last_proc_time_ = proc_time;
            last_sys_time_  = sys_time;
            return;
        }

        long proc_delta = proc_time - last_proc_time_;
        long sys_delta  = sys_time  - last_sys_time_;

        if (sys_delta > 0)
        {
            double cpu =
                100.0 *
                (double)proc_delta /
                (double)sys_delta /
                num_cores_;

            cpu_percent_.store(cpu);
        }

        last_proc_time_ = proc_time;
        last_sys_time_  = sys_time;
    }

private:

    mutable std::mutex mtx_;
    std::unordered_map<std::string, Stats> stats_;

    std::string name_;

    std::atomic<bool> running_{false};
    std::thread monitor_thread_;

    std::atomic<double> cpu_percent_{0.0};
    std::atomic<double> ram_mb_{0.0};

    long last_proc_time_ = 0;
    long last_sys_time_  = 0;
    int num_cores_ = 1;
};

//////////////////////////////////////////////////////////////////
// Scoped timer
//////////////////////////////////////////////////////////////////

class ScopedTimer
{
public:

    ScopedTimer(Profiler& p, const char* name)
        : profiler_(p), name_(name)
    {
        start_ = std::chrono::steady_clock::now();
    }

    ~ScopedTimer()
    {
        auto end = std::chrono::steady_clock::now();

        double ms =
            std::chrono::duration<double, std::milli>(end - start_).count();

        profiler_.add_sample(name_, ms);
    }

private:

    Profiler& profiler_;
    const char* name_;
    std::chrono::steady_clock::time_point start_;
};

} // namespace lio_ros

//////////////////////////////////////////////////////////////////
// Macros
//////////////////////////////////////////////////////////////////

#if LIO_PROFILE

#define LIO_CONCAT_IMPL(x, y) x##y
#define LIO_CONCAT(x, y) LIO_CONCAT_IMPL(x, y)

#define LIO_PROFILE_SCOPE(profiler, name) \
    lio_ros::ScopedTimer LIO_CONCAT(_prof_, __LINE__)(profiler, name)

#define LIO_PROFILE_FUNCTION(profiler) \
    LIO_PROFILE_SCOPE(profiler, __FUNCTION__)
#else

#define LIO_PROFILE_SCOPE(profiler, name)
#define LIO_PROFILE_FUNCTION(profiler)

#endif