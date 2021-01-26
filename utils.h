//
// Created by mkspopov on 11.11.2020.
//

#ifndef DIJKSTRA_UTILS_H
#define DIJKSTRA_UTILS_H

#include <iostream>

#define ASSERT(expression) \
    if (!(expression)) {   \
        std::string exprStr(#expression); \
        throw std::runtime_error("Assertion failed: " + exprStr); \
    }                      \

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

#endif //DIJKSTRA_UTILS_H
