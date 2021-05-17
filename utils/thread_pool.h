#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

class Task {
public:
    explicit Task(std::function<void()> function) : function_(std::move(function)) {
    }

    void operator()();

    void Cancel();

    bool IsCompleted() const;

    bool IsCompletedOrThrow() const;

    void Wait();

private:
    void ThrowIfError() const;

    bool completed_ = false;
    std::exception_ptr exceptionPtr_;
    std::function<void()> function_;

    std::condition_variable cv_;
    mutable std::mutex mutex_;
};

class ThreadPool {
public:
    explicit ThreadPool(size_t threadsNumber);

    ~ThreadPool();

    std::shared_ptr<Task> AddTask(std::function<void()> task);

    void Kill();

    void WaitAll();

private:
    void PollTasks();

    void Shutdown();

    std::condition_variable clientCv_;
    std::condition_variable cv_;
    std::mutex globalMutex_;
    bool shutdown_ = false;

    size_t inProcess_ = 0;
    std::deque<std::shared_ptr<Task>> tasks_;
    std::vector<std::thread> threads_;
};
