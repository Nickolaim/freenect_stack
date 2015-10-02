#pragma once  // Only can be included in VS

// Since the primary platform is Linux, CRT warnings and ..._s functions are not applicable.
#define _CRT_SECURE_NO_WARNINGS

#include <cassert>
#include <cstdint>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
