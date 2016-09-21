#pragma once

#include <iostream>

#define ERR(x)  std::cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x)  std::cerr << "\033[22;34;1m" << x << "\033[0m";
