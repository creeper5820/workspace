#include <iostream>
#include <string>

class Solution {
public:
    int find(std::string& string, int position_left, int position_right)
    {
        if (position_left > string.size() || position_right > string.size())
            return -1;

        int count = 0;

        while (position_left > 0 - 1 && position_right < string.size()) {

            if (string.at(position_left) == string.at(position_right))
                count++;
            else
                break;

            position_left--;
            position_right++;
        }

        return count;
    }

    bool load(int& count_max, int& position_max, const int count, const int position)
    {
        if (count_max < count) {
            count_max = count;
            position_max = position;

            return 1;
        }

        else
            return 0;
    }

    std::string longestPalindrome(std::string string)
    {
        int count_max_1 = 0;
        int count_max_2 = 0;
        int position_1 = 0;
        int position_2 = 0;

        for (int i = 0; i < string.size(); i++) {
            load(count_max_1, position_1, find(string, i, i), i);
        }

        for (int i = 0; i < string.size() - 1; i++) {
            load(count_max_2, position_2, find(string, i, i + 1), i);
        }

        if (count_max_1 > count_max_2)
            return string.substr(position_1 - count_max_1 + 1, count_max_1 * 2 - 1);
        else
            return string.substr(position_2 - count_max_2 + 1, count_max_2 * 2);
    }
};

int main()
{
    Solution solution;
    std::string test_1 = "asddsaasdfghjijhgfdsadasdasdsa";
    std::string test_2 = "ccc";

    std::cout << solution.longestPalindrome(test_1) << '\n';
    std::cout << solution.longestPalindrome(test_2) << '\n';

    return 0;
}