#include "utils.h"

int main() {
    int x = -1;
    ASSERT_EQUAL(++x, 0);
    if (x != 0) {
        throw std::runtime_error("");
    }
}
