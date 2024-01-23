#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <tuple>

namespace utility
{

    template <typename T, typename... Args>
    class Lazy
    {
    public:
        Lazy(Args... args)
            : init_status_(InitStatus::UNINITIALIZE)
        {
            *reinterpret_cast<ArgTupleT *>(data_) = std::make_tuple(std::move(args)...);
        }

        T *get()
        {
            return &object();
        }

        T *operator->()
        {
            return &object();
        }

        T &operator*()
        {
            return object();
        }

        operator bool() const noexcept
        {
            return init_status_ == InitStatus::INITIALIZED;
        }

        template <typename CallableT>
        void weak_execute(const CallableT &callable)
        {
            if (init_status_ == InitStatus::INITIALIZED) {
                callable(reinterpret_cast<T *>(data_));
            }
        }

    private:
        T &object()
        {
            auto &object = *reinterpret_cast<T *>(data_);
            auto &args   = *reinterpret_cast<ArgTupleT *>(data_);

            if (init_status_ != InitStatus::INITIALIZED) {
                assert(init_status_ == InitStatus::UNINITIALIZE);
                init_status_ = InitStatus::INITIALIZING;

                auto moved_args = std::move(args);
                args.~ArgTupleT();

                construct_object(
                    std::move(moved_args), std::make_index_sequence<std::tuple_size_v<ArgTupleT>>{});

                init_status_ = InitStatus::INITIALIZED;
            }

            return object;
        }

        template <typename TupleT, std::size_t... I>
        void construct_object(TupleT &&t, std::index_sequence<I...>)
        {
            new (data_) T(std::get<I>(std::forward<TupleT>(t))...);
        }

        using ArgTupleT = std::tuple<Args...>;

        enum class InitStatus : uint8_t {
            UNINITIALIZE = 0,
            INITIALIZING = 1,
            INITIALIZED  = 2
        };
        InitStatus init_status_;

        static constexpr auto alignment = std::max(alignof(T), alignof(ArgTupleT));
        static constexpr auto data_size = std::max(sizeof(T), sizeof(ArgTupleT));

        alignas(alignment) uint8_t data_[data_size];
    };

} // namespace utility