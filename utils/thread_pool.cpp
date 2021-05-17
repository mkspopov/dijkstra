#include "thread_pool.h"

void Task::operator()() {
    std::unique_lock lock(mutex_);
    if (!completed_) {  // TODO: canceled_
        try {
            function_();
            completed_ = true;
        } catch (...) {
            exceptionPtr_ = std::current_exception();
        }
    }
    cv_.notify_all();
}

void Task::Cancel() {
    std::unique_lock lock(mutex_);
    completed_ = true;  // TODO: canceled_
}

bool Task::IsCompleted() const {
    std::unique_lock lock(mutex_);
    return completed_;
}

bool Task::IsCompletedOrThrow() const {
    std::unique_lock lock(mutex_);
    if (completed_) {
        return true;
    }
    ThrowIfError();
    return false;
}

void Task::ThrowIfError() const {
    if (exceptionPtr_) {
        std::rethrow_exception(exceptionPtr_);
    }
}

void Task::Wait() {
    std::unique_lock lock(mutex_);
    while (!completed_ && exceptionPtr_ == nullptr) {
        cv_.wait(lock);
    }
}

ThreadPool::ThreadPool(size_t threadsNumber) {
    for (size_t i = 0; i < threadsNumber; ++i) {
        threads_.emplace_back([this]() {
            PollTasks();
        });
    }
}

ThreadPool::~ThreadPool() {
    Shutdown();
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

std::shared_ptr<Task> ThreadPool::AddTask(std::function<void()> task) {
    std::unique_lock lock(globalMutex_);
    if (shutdown_) {
        throw std::runtime_error("ThreadPool is shutting down.");
    }
    tasks_.emplace_back(std::make_shared<Task>(std::move(task)));
    cv_.notify_one();
    return tasks_.back();
}

void ThreadPool::Kill() {
    {
        std::unique_lock lock(globalMutex_);
        tasks_.clear();
    }
    Shutdown();
}

void ThreadPool::PollTasks() {
    std::unique_lock lock(globalMutex_);
    while (!shutdown_ || !tasks_.empty()) {
        if (tasks_.empty()) {
            clientCv_.notify_all();
            cv_.wait(lock);
        }

        while (!tasks_.empty()) {
            auto task = tasks_.front();
            tasks_.pop_front();
            ++inProcess_;

            lock.unlock();
            (*task)();
            lock.lock();

            --inProcess_;
        }

        if (inProcess_ == 0 && tasks_.empty()) {
            clientCv_.notify_all();
        }
    }
}

void ThreadPool::Shutdown() {
    std::unique_lock lock(globalMutex_);
    shutdown_ = true;
    cv_.notify_all();
}

void ThreadPool::WaitAll() {
    std::unique_lock lock(globalMutex_);

    while (inProcess_ > 0 || !tasks_.empty()) {
        clientCv_.wait(lock);
    }
}
