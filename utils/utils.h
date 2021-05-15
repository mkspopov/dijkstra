//
// Created by mkspopov on 11.11.2020.
//

#pragma once

#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <tuple>
#include <unordered_map>

#define RUN_TEST(test_function) do { \
    {                            \
    std::cerr << "Running " << #test_function << " ...\n"; \
    Timer timer;                            \
    test_function(); \
    std::cerr << "Done " << #test_function << " in " << timer.Elapsed() / 1'000'000 << "ms\n"; \
    }                                \
    } while (false)

template <class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vector) {
    for (const auto& elem : vector) {
        os << elem << ' ';
    }
    return os;
}

template <class K, class V>
std::ostream& operator<<(std::ostream& os, const std::pair<K, V>& pair) {
    os << pair.first << ' ' << pair.second;
    return os;
}

template <class K, class V, class _1, class _2, class _3, template<class...> class Map>
std::ostream& operator<<(std::ostream& os, const Map<K, V, _1, _2, _3>& map) {
    for (const auto& elem : map) {
        os << elem << ' ';
    }
    return os;
}

template <class Iterator>
class IteratorRange {
public:
    IteratorRange(Iterator begin, Iterator end)
        : begin_(begin), end_(end) {
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

template <class R>
auto IRange(R&& r) {
    return IteratorRange(r.begin(), r.end());
}

template <class Iterator, class Predicate>
class FilterIterator : std::iterator<std::forward_iterator_tag, typename Iterator::value_type> {
public:
    FilterIterator(Iterator current, Iterator end, Predicate predicate)
        : current_(std::move(current)), end_(std::move(end)), predicate_(std::move(predicate))
    {
        AdvanceIfNeeded();
    }

    FilterIterator& operator++() {
        ++current_;
        AdvanceIfNeeded();
        return *this;
    }

    auto operator*() const {
        return *current_;
    }

    bool operator==(const FilterIterator& rhs) const {
        return current_ == rhs.current_;
    }

private:
    void AdvanceIfNeeded() {
        while (current_ != end_ && !predicate_(*current_)) {
            ++current_;
        }
    }

    Iterator current_;
    Iterator end_;
    Predicate predicate_;
};

template <typename Integer>
class Range : public std::iterator<std::forward_iterator_tag, Integer> {
public:
    Range(Integer begin, Integer end)
        : begin_(begin), end_(end) {
    }

    Range begin() const {
        return Range(begin_, end_);
    }

    Range end() const {
        return Range(end_, end_);
    }

    Range& operator++() {
        ++begin_;
        return *this;
    }

    Integer operator*() const {
        return begin_;
    }

    bool operator==(const Range& rhs) const {
        return begin_ == rhs.begin_;
    }

private:
    Integer begin_;
    Integer end_;
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

    explicit Logger(std::ostream& os = std::cerr);

private:
    std::ostream& os_;
};

Logger::LineLogger Log();

class Timer {
public:
    Timer();

    uint64_t Elapsed() const;

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

template <typename From, typename To>
struct Mapping {
    bool TryAdd(From from, To to) {
        if (direct_.contains(from) || inverse_.contains(to)) {
            return false;
        }
        direct_.emplace(from, to);
        inverse_.emplace(to, from);
        return true;
    }

    bool Contains(From from) const {
        return direct_.contains(from);
    }

    To& Direct(From from) {
        return direct_.at(from);
    }

    From& Inverse(To to) {
        return inverse_.at(to);
    }

    std::unordered_map<From, To> direct_;
    std::unordered_map<To, From> inverse_;
};

#define ASSERT_EQUAL(lhs, rhs) do {                                            \
    auto left = (lhs);                                                         \
    auto right = (rhs);                                                        \
    if (!(left == right)) {                                                    \
        std::stringstream ss;                                                  \
        ss <<  __FILE__ << ':' << __LINE__ << std::endl;                       \
        ss << "\t\tASSERT_EQUAL failed:\t" << left << "\t!=\t" << right << std::endl;\
        ss << "\t\tArgs:\t" << #lhs << "\t!=\t" << #rhs << std::endl;                 \
        throw std::runtime_error(ss.str());                                    \
    }                                                                          \
} while (false)                                                                \

#define ASSERT(expression) ASSERT_EQUAL(expression, true)

template <typename Range>
auto ToVector(Range&& range) {
    std::vector<std::ranges::range_value_t<decltype(range)>> v;
    if constexpr(std::ranges::sized_range<decltype(range)>) {
        v.reserve(std::ranges::size(range));
    }

    std::copy(range.begin(), range.end(), std::back_inserter(v));
    return v;
}

template <class T>
void Dump(std::ostream& os, const T& value);

template <class T>
void Dump(std::ostream& os, const std::vector<T>& vector);

template <class K, class V, template <class...> class Map>
void Dump(std::ostream& os, const Map<K, V>& map);

template <class T>
void Dump(std::ostream& os, const T& value) {
    os.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

template <class T>
void Dump(std::ostream& os, const std::vector<T>& vector) {
    Dump(os, vector.size());
    for (const auto& value : vector) {
        Dump(os, value);
    }
}

template <class K, class V, template <class...> class Map>
void Dump(std::ostream& os, const Map<K, V>& map) {
    Dump(os, map.size());
    for (const auto& [key, value] : map) {
        Dump(os, key);
        Dump(os, value);
    }
}

template <class T>
void Load(std::istream& is, T& value);

template <class T>
void Load(std::istream& is, std::vector<T>& vector);

template <class K, class V, template<class...> class Map>
void Load(std::istream& is, Map<K, V>& map);

template <class T>
void Load(std::istream& is, T& value) {
    is.read(reinterpret_cast<char*>(&value), sizeof(value));
}

template <class T>
void Load(std::istream& is, std::vector<T>& vector) {
    size_t size;
    Load(is, size);
    vector.resize(size);
    for (auto& value : vector) {
        Load(is, value);
    }
}

template <class K, class V, template<class...> class Map>
void Load(std::istream& is, Map<K, V>& map) {
    size_t size;
    Load(is, size);
    for (size_t i = 0; i < size; ++i) {
        K key;
        V value;
        Load(is, key);
        Load(is, value);
        map.emplace(std::move(key), std::move(value));
    }
}
