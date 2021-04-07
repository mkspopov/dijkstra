//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include <chrono>
#include <iostream>

#define ASSERT(expression) do { \
    if (!(expression)) {       \
        std::string exprStr(#expression); \
        throw std::runtime_error("Assertion failed: " + exprStr); \
    }                           \
    } while (false)             \

#define RUN_TEST(test_function) \
    {                            \
    std::cerr << "Running " << #test_function << " ...\n"; \
    Timer timer;                            \
    test_function(); \
    std::cerr << "Done " << #test_function << " in " << timer.Elapsed() / 1'000'000 << "ms\n"; \
    }

template <class Iterator>
class IteratorRange {
public:
    IteratorRange(Iterator begin, Iterator end)
        : begin_(begin)
        , end_(end) {
    }

    Iterator begin() const {
        return begin_;
    }

    Iterator end() const {
        return end_;
    }

private:
    Iterator begin_;
    Iterator end_;
};

class Logger {
public:
    class LineLogger {
    public:
        explicit LineLogger(const Logger& logger);

        LineLogger(const LineLogger&) = delete;
        LineLogger(LineLogger&&) = delete;

        LineLogger& operator=(const LineLogger&) = delete;
        LineLogger& operator=(LineLogger&&) = delete;

        ~LineLogger();

        template <class T>
        LineLogger&& operator<<(const T& something) {
            os_ << something << separator_;
            return std::move(*this);
        }

    private:
        std::ostream& os_;
        const char separator_ = ' ';
        const char lineEnd_ = '\n';
    };

    explicit Logger(std::ostream& os = std::cout);

private:
    std::ostream& os_;
};

Logger::LineLogger Log();

class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {
    }

    uint64_t Elapsed() const {
        return (std::chrono::high_resolution_clock::now() - start_).count();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};
