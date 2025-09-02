#include "ThreadPool.h"

void physecs::ThreadPool::doTasks() {
    int i = currentTask.load();
    while (i) {
        const int numTasks = std::max(1, i / (numThreads * chunkFactor));
        if (!currentTask.compare_exchange_weak(i, i - numTasks)) continue;
        for (int j = 0; j < numTasks; ++j) task(--i);
    }
}

void physecs::ThreadPool::run(int id) {
    while (true) {
        start[id].wait(false);
        if (!isActive) break;

        doTasks();

        start[id].store(false);
    }
}

physecs::ThreadPool::ThreadPool(int numThreads): numThreads(numThreads), start(numThreads) {
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadPool::run, this, i);
        start[i].store(false);
    }
}

void physecs::ThreadPool::parallelFor(int count, std::function<void(int)> func) {
    task = std::move(func);
    currentTask.store(count);

    for (int i = 0; i < numThreads; ++i) {
        start[i].store(true);
        start[i].notify_one();
    }

    doTasks();

    for (int i = 0; i < numThreads; ++i) {
        while (start[i].load()) {
            _mm_pause();
        }
    }
}

physecs::ThreadPool::~ThreadPool() {
    isActive = false;

    for (int i = 0; i < numThreads; ++i) {
        start[i].store(true);
        start[i].notify_one();
    }

    for (auto& th : threads) {
        th.join();
    }
}
