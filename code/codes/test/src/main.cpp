#include <initializer_list>
#include <iostream>

template <typename T>
T add(T f)
{
    return f;
}

template <typename T, typename... Args>
T add(T first, Args... args)
{
    return first + add<T>(args...);
}

template <class F, class... Args>
void expand(const F& f, Args&&... args)
{
    auto a = std::initializer_list<int> {
        ((f(std::forward<Args>(args)), 0))...
    };
}

int main()
{
    expand([](int i) -> void {
        std::cout << i << std::endl;
    },
        1, 2, 3);

    return 0;
}