//
// Created by mkspopov on 11.11.2020.
//

#include "utils.h"

Logger::LineLogger Log() {
    static Logger logger;
    return Logger::LineLogger(logger);
}

Logger::Logger(std::ostream& os)
    : os_(os) {
}

Logger::LineLogger::LineLogger(const Logger& logger)
    : os_(logger.os_) {
}

Logger::LineLogger::~LineLogger() {
    os_ << lineEnd_;
}

Timer::Timer() : start_(std::chrono::high_resolution_clock::now()) {
}

uint64_t Timer::Elapsed() const {
    return (std::chrono::high_resolution_clock::now() - start_).count();
}

std::mt19937& GetRng() {
    static std::mt19937 gen;
    return gen;
}
