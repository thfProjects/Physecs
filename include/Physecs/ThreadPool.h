#pragma once

#include <functional>
#include <thread>
#include <vector>
#include <condition_variable>

namespace physecs {
    class ThreadPool {
        int numThreads;
        bool isActive = true;
        std::vector<std::thread> threads;
        std::function<void(int)> task;
        std::vector<std::atomic_bool> start;
        std::atomic_int currentTask = 0;

        static constexpr int chunkFactor = 2;
        void doTasks();
        void run(int id);
    public:
        ThreadPool(int numThreads);
        void parallelFor(int count, std::function<void(int)> func);
        ~ThreadPool();
    };
}
