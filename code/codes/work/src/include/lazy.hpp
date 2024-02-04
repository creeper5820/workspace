#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <tuple>
#include <utility>

namespace utility {

template <typename T, typename... Args>
class Lazy {

private:
    enum class InitStatus : uint8_t {
        UNINITIALIZE = 0,
        INITIALIZING = 1,
        INITIALIZED = 2
    };

    using ArgTupleT = std::tuple<Args...>;

private:
    InitStatus status_;

    // malloc the mamory of alignment
    static constexpr auto alignment_
        = std::max(alignof(T), alignof(ArgTupleT));
    static constexpr auto size_
        = std::max(sizeof(T), sizeof(ArgTupleT));

    alignas(alignment_) uint8_t data_[size_];

private:
    T& object()
    {
        auto& object = *reinterpret_cast<T*>(data_);
        auto& args = *reinterpret_cast<ArgTupleT*>(data_);

        if (status_ != InitStatus::INITIALIZED) {
            assert(status_ == InitStatus::UNINITIALIZE);
            status_ = InitStatus::INITIALIZING;

            auto args_move = std::move(args);
            args.~ArgTupleT();

            std::apply(
                [this]<typename... _Args>(_Args&&... args) {
                    new (data_) T(std::forward<_Args>(args)...);
                },
                std::move(args_move));

            status_ = InitStatus::INITIALIZED;
        }

        return object;
    }

public:
    Lazy(Args... args)
        : status_(InitStatus::UNINITIALIZE)
    {
        *reinterpret_cast<ArgTupleT*>(data_)
            = std::make_tuple(std::move(args)...);
    }

    T* get()
    {
        return &object();
    }

    T* operator->()
    {
        return &object();
    }

    T& operator*()
    {
        return object();
    }

    operator bool() const noexcept
    {
        return status_ == InitStatus::INITIALIZED;
    }

    template <typename CallableT>
    void weak_execute(const CallableT& callable)
    {
        if (status_ == InitStatus::INITIALIZED) {
            callable(reinterpret_cast<T*>(data_));
        }
    }
}; // class lazy
} // namespace utility