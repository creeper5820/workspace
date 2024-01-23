#include "utility.hpp"
#include <iostream>

class Solution {
public:
    std::vector<double> intersection(std::vector<int>& start1, std::vector<int>& end1, std::vector<int>& start2, std::vector<int>& end2)
    {
        std::vector<double> result;

        auto b = utility::brzier(
            start1[0], start1[1], end1[0], end1[1],
            start2[0], start2[1], end2[0], end2[1]);

        if (b.t >= 0 && b.t <= 1 || b.u >= 0 && b.u <= 1) {
            result.push_back(b.x);
            result.push_back(b.y);
        }

        return result;
    }
};

int main()
{
    auto b = utility::brzier(
        0, 0, 0, 6,
        0, 1, 0, 5);

    std::cout << "t=" << b.t << '\n'
              << "u=" << b.u << '\n';

    if (b.t <= 1 && b.t >= 0 || b.u <= 1 && b.u >= 0)
        std::cout << "( " << b.x << " , " << b.y << " )" << '\n';

    return 0;
}