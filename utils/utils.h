//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include <chrono>
#include <iostream>
#include <random>
#include <tuple>

#define ASSERT(expression) do { \
    if (!(expression)) {       \
        std::string exprStr(#expression); \
        throw std::runtime_error("Assertion failed: " + exprStr); \
    }                           \
    } while (false)             \

#define RUN_TEST(test_function) do { \
    {                            \
    std::cerr << "Running " << #test_function << " ...\n"; \
    Timer timer;                            \
    test_function(); \
    std::cerr << "Done " << #test_function << " in " << timer.Elapsed() / 1'000'000 << "ms\n"; \
    }                                \
    } while (false)

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

std::mt19937& GetRng();

template <class ClassRef>
class Enumerate {
public:
    explicit Enumerate(ClassRef& c, size_t index = 0) : c_(c), index_(index) {
    }

    Enumerate& operator++() {
        ++index_;
        return *this;
    }

    auto operator*() {
        return std::tie(index_, c_[index_]);
    }

    bool operator!=(const Enumerate& rhs) const {
        return index_ != rhs.index_;
    }

    auto begin() { return Enumerate(c_, 0); }

    auto end() { return Enumerate(c_, c_.size()); }

private:
    ClassRef& c_;
    size_t index_ = 0;
};
