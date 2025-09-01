#include "ThreadPool.h"

void physecs::ThreadPool::run() {
    while (true) {
        start.wait(false);
        if (!isActive) break;

        int doneCount = 0;
        int i = currentTask.load();
        while (i) {
            const int numTasks = std::max(1, i / (numThreads * chunkFactor));
            if (!currentTask.compare_exchange_weak(i, i - numTasks)) continue;
            for (int j = 0; j < numTasks; ++j) task(--i);
            doneCount += numTasks;
        }
        if (doneCount) remainingTasks -= doneCount;
    }
}

physecs::ThreadPool::ThreadPool(int numThreads): numThreads(numThreads) {
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadPool::run, this);
    }
}

void physecs::ThreadPool::parallelFor(int count, std::function<void(int)> func) {
    task = std::move(func);
    currentTask.store(count);
    remainingTasks.store(count);

    start.store(true);
    start.notify_all();

    while (remainingTasks.load()) {
        _mm_pause();
    }

    start.store(false);
}

physecs::ThreadPool::~ThreadPool() {
    isActive = false;

    start.store(true);
    start.notify_all();

    for (auto& th : threads) {
        th.join();
    }
}
