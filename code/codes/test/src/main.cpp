#include <initializer_list>

#include <functional>
#include <iostream>
#include <memory>
#include <type_traits>

template <typename T>
class ITask {
public:
    using HandleType = std::function<void(T)>;

    ITask(HandleType callback)
    {
        callback_ = callback;
    }

    ITask() { }

    void set_handle(HandleType callback)
    {
        callback_ = callback;
    }

    void callback(T a)
    {
        callback_(a);
    }

private:
    HandleType callback_;
};

class Big { };

void print_error(float)
{
    std::cout << "error" << '\n';
}

template <typename T>
void print_number(T num)
{
    std::cout << num << '\n';
}

void handle(const Big&& big)
{
}

int main()
{
    // initialize
    auto task_1_ptr = std::make_shared<ITask<int>>();
    task_1_ptr->set_handle([](int) -> void {
        std::cout << "error" << '\n';
    });

    auto task_2_ptr = std::make_shared<ITask<float>>();
    task_2_ptr->set_handle(print_error);

    ////////
    // task_1_ptr->callback();
    // task_2_ptr->callback();

    print_number(2.3);
    print_number(std::string("2233"));
    ///////

    return 0;
}