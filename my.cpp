#include "utils/utils.h"

#include <iostream>
#include <ranges>
#include <tuple>
#include <vector>

struct Int {
    Int(int a) : a(a) {
    }
    Int(const Int& rhs) : a(rhs.a) {
        std::cout << "copied" << std::endl;
    }
    Int(Int&& rhs) noexcept : a(rhs.a) {
        std::cout << "moved" << std::endl;
    }

    int a;
};

int main() {
    const std::vector<Int>& a = {1, 2, 3};
    const std::vector<Int>& b = {4, 5, 6};
    std::cout << "First\n";
    auto r = {std::views::all(a), std::views::all(b)};
    std::cout << "Second\n";
    for (const auto& i : r | std::views::join) {
        std::cout << i.a << ' ';
    }
    std::cout << std::endl;
}
