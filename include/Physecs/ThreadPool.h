#pragma once

#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>

namespace physecs {
    class ThreadPool {
        int numThreads;
        bool isActive = true;
        std::vector<std::thread> threads;
        std::function<void(int)> task;
        std::mutex mutex;
        std::condition_variable cv;
        std::atomic_int currentTask = 0;
        int count = 0;
        std::atomic_int remainingTasks = 0;
        void run();
    public:
        ThreadPool(int numThreads);
        void parallelFor(int count, std::function<void(int)> func);
        ~ThreadPool();
    };
}
