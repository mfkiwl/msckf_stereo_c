//
// Created by cg on 9/12/19.
//

#include <gtest/gtest.h>

#include "math_utils/math_utils.hpp"
#include "math_utils/random_numbers.h"

TEST(MathUtils, random_number) {
    random_numbers::RandomNumberGenerator random_gen;
    int rng01 = random_gen.uniformInteger(0, 100);

    int rng02 = mynt::uniform_integer(0, 100);

    std::cout << "rng_value: " << rng01 << ", " << rng02 << std::endl;
}
