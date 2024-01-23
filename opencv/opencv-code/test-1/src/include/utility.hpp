#pragma once

#include <algorithm>

namespace utility {

typedef struct {
    double t;
    double u;
    double x;
    double y;
} Brizer;

inline bool is_point_inline(double x1, double y1, double x2, double y2, double x, double y)
{
    bool b1 = ((y - y1) * (x2 - x1) - (x - x1) * (y2 - y1) == 0);
    bool b2 = (x >= std::min(x1, x2) && x <= std::max(x1, x2));
    bool b3 = (y >= std::min(y1, y2) && y <= std::max(y1, y2));

    return b1 && b2 && b3;
}

inline Brizer brzier(
    double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4)
{
    Brizer result;

    double divisor = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    result.t
        = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / divisor;

    result.u
        = ((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / divisor;

    if (result.t >= 0 && result.t <= 1) {
        result.x = x1 + result.t * (x2 - x1);
        result.y = y1 + result.t * (y2 - y1);
    }

    else if (result.u >= 0 && result.u <= 1) {
        result.x = x3 + result.u * (x4 - x3);
        result.y = y3 + result.u * (y4 - y3);
    }

    return result;
}

}