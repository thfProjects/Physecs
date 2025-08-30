#include "ThreadPool.h"

void physecs::ThreadPool::run() {
    while (true) {
        {
            std::unique_lock lock(mutex);
            cv.wait(lock, [this] { return currentTask || !isActive;});
            if (!isActive) break;
        }
        int i = currentTask.load();
        while (i) {
            if (!currentTask.compare_exchange_weak(i, i - 1)) continue;
            task(--i);
            --remainingTasks;
        }
    }
}

physecs::ThreadPool::ThreadPool(int numThreads): numThreads(numThreads) {
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadPool::run, this);
    }
}

void physecs::ThreadPool::parallelFor(int count, std::function<void(int)> func) {
    {
        std::unique_lock lock(mutex);
        task = std::move(func);
        this->count = count;
        currentTask.store(count);
        remainingTasks.store(count);
    }
    cv.notify_all();
    while (remainingTasks.load()) {
        _mm_pause();
    }
}

physecs::ThreadPool::~ThreadPool() {
    {
        std::unique_lock lock(mutex);
        isActive = false;
    }
    cv.notify_all();
    for (auto& th : threads) {
        th.join();
    }
}
