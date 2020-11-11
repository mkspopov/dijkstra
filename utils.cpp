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
