#pragma once

#include "SerialDefines.h"
#include "FastFix.h"
#ifdef _WIN32
#include "Impl/Windows.h"
#elif __linux__
#include "Impl/Linux.h"
#endif