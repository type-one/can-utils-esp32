/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file tag_invoke.hpp
 * @brief Provides utilities for Tag Invoke customization point.
 *
 * This is used to customize behavior for operations in a highly generic way. It’s part of the "customization point" 
 * mechanism, a concept often used in advanced C++ libraries to allow extension without modifying the original code.
 * The tag_invoke function essentially provides a hook to intercept operations and forward them to the appropriate 
 * implementation based on the types.
 * 
 * This header file contains the implementation of the tag_invoke customization point,
 * which allows for the customization of behavior for specific types through the use of
 * a customization point object (CPO). The implementation is based on the proposal
 * outlined in P1895R0 and follows the principles of Eric Niebler's Ranges library.
 *
 * The main components of this header file include:
 * - The `mireo::_tag_invoke` namespace, which contains internal implementation details
 *   for the tag_invoke mechanism.
 * - The `mireo::_tag_invoke_cpo` namespace, which defines the tag_invoke customization
 *   point object.
 * - Various type traits and concepts to check the invocability and noexcept properties
 *   of tag_invoke.
 * 
 * Principle of Type Erasure
 * 
 * In type erasure:
 *
 * - Encapsulation of Type-Specific Logic: You define a wrapper (like any.hpp in this case) that hides the 
 *   underlying type of the object it holds. This wrapper provides a uniform interface for interacting with the object.
 * - Behavioral Polymorphism: The behavior is preserved using a set of operations defined in the wrapper, like method 
 *   calls, without exposing the actual type.
 * - No Dependency on Base Classes: Unlike traditional polymorphism, type erasure doesn't rely on inheritance or 
 *   virtual function tables (vtables). Instead, it uses a combination of templates and dynamic memory allocation 
 *   to achieve its goal.
 *
 * Think of it like a "type-neutral box" where you can store objects of different types, but interact with them 
 * uniformly.
 *
 * @note This implementation is optimized for compile-time performance and does not
 * rely on the generality of the standard library's invoke_result and is_invocable traits.
 *
 * @copyright
 * Copyright (c) 2001-2023 Mireo, EU
 * 
 * @see
 * https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2019/p1895r0.pdf
 */

//-----------------------------------------------------------------------------//
// ESP32 C++ DBC/CAN parser - Spare time mod and FreeRTOS port for fun         //
// Laurent Lardinois https://be.linkedin.com/in/laurentlardinois               //
//                                                                             //
// https://github.com/type-one/can-utils-esp32                                 //
//                                                                             //
// A C++ DBC file parser, and a CAN telemetry tool, adapted for                //
// ESP32 micro-controller, forked and modified from MIREO version at           //
// https://github.com/mireo/can-utils                                          //
//-----------------------------------------------------------------------------//

#pragma once

/*

Read:

https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2019/p1895r0.pdf

Example:

// Define a new CPO named mylib::foo()
//
// “Customization point object.” This is a notion introduced by Eric Niebler’s Ranges library.
// [http://eel.is/c++draft/customization.point.object#1]

namespace mylib {

inline constexpr struct foo_cpo {
    template <typename T>
    auto operator()(const T& x) const -> mireo::tag_invoke_result_t<foo_cpo, const T&> {
        // CPO dispatches to tag_invoke() call; passes the CPO itself as first argument.
        return mireo::tag_invoke(*this, x);
    }
} foo;

}

// Use the mylib::foo() CPO
template <class T> requires std::invocable<mireo::tag_t<mylib::foo>, const T&>
void print_foo(const T& x) {
    // Just call the CPO like an ordinary function
    std::cout << mylib::foo(x) << std::endl;
}

// Customise the mylib::foo() CPO for othertype
namespace otherlib {

struct othertype {
    int x;

    friend int tag_invoke(mireo::tag_t<mylib::foo>, const othertype& x) {
        return x.x;
    }
};

}

// Can now call print_foo() function.
void example() {
    otherlib::othertype x;
    print_foo(x);
}

*/

#include <type_traits>
#include <utility>

/**
 * @namespace mireo
 * @brief Contains the implementation of tag_invoke customization point.
 */
namespace mireo
{
    /**
     * @namespace mireo::_tag_invoke
     * @brief Internal namespace for tag_invoke implementation details.
     */
    namespace _tag_invoke
    {
        /**
         * @struct mireo::_tag_invoke::_fn
         * @brief Function object for invoking tag_invoke.
         */
        struct _fn
        {
            /**
             * @brief Invokes the tag_invoke customization point.
             * @tparam CPO Customization point object type.
             * @tparam Args Argument types.
             * @param cpo Customization point object.
             * @param args Arguments to pass to the customization point.
             * @return Result of invoking tag_invoke with the given arguments.
             */
            template <typename CPO, typename... Args>
            constexpr auto operator()(CPO cpo, Args&&... args) const
                noexcept(noexcept(tag_invoke((CPO &&) cpo, (Args &&) args...)))
                    -> decltype(tag_invoke((CPO &&) cpo, (Args &&) args...))
            {
                return tag_invoke((CPO &&) cpo, (Args &&) args...);
            }
        };


        /**
         * @typedef mireo::_tag_invoke::tag_invoke_result_t
         * @brief Alias for the result type of invoking tag_invoke.
         * @tparam CPO Customization point object type.
         * @tparam Args Argument types.
         */
        template <typename CPO, typename... Args>
        using tag_invoke_result_t = decltype(tag_invoke(std::declval<CPO&&>(), std::declval<Args&&>()...));

        /**
         * @typedef mireo::_tag_invoke::yes_type
         * @brief Type indicating a successful tag_invoke.
         */
        using yes_type = char;

        /**
         * @typedef mireo::_tag_invoke::no_type
         * @brief Type indicating a failed tag_invoke.
         */
        using no_type = char (&)[2];

        /**
         * @brief Tries to invoke tag_invoke and returns yes_type on success.
         * @tparam CPO Customization point object type.
         * @tparam Args Argument types.
         * @return yes_type if tag_invoke is invocable, otherwise no_type.
         */
        template <typename CPO, typename... Args>
        auto try_tag_invoke(int) noexcept(noexcept(tag_invoke(std::declval<CPO&&>(), std::declval<Args&&>()...)))
            -> decltype(static_cast<void>(tag_invoke(std::declval<CPO&&>(), std::declval<Args&&>()...)), yes_type {});

        /**
         * @brief Fallback for try_tag_invoke that returns no_type.
         * @tparam CPO Customization point object type.
         * @tparam Args Argument types.
         * @return no_type.
         */
        template <typename CPO, typename... Args>
        no_type try_tag_invoke(...) noexcept(false);

        /**
         * @struct mireo::_tag_invoke::defer
         * @brief Helper struct to defer instantiation of a template.
         * @tparam T Template to defer.
         * @tparam Args Arguments to pass to the template.
         */
        template <template <typename...> class T, typename... Args>
        struct defer
        {
            using type = T<Args...>;
        };

        /**
         * @struct mireo::_tag_invoke::empty
         * @brief Empty struct used as a default type.
         */
        struct empty
        {
        };

    } // namespace _tag_invoke


    /**
     * @namespace mireo::_tag_invoke_cpo
     * @brief Contains the tag_invoke customization point object.
     */
    namespace _tag_invoke_cpo
    {
        /**
         * @var mireo::_tag_invoke_cpo::tag_invoke
         * @brief The tag_invoke customization point object.
         */
        inline constexpr _tag_invoke::_fn tag_invoke {};
    }

    using namespace _tag_invoke_cpo;

    /**
     * @typedef mireo::tag_t
     * @brief Alias for the type of a customization point object.
     * @tparam CPO Customization point object.
     */
    template <auto& CPO>
    using tag_t = std::remove_cvref_t<decltype(CPO)>;

    // Manually implement the traits here rather than defining them in terms of
    // the corresponding std::invoke_result/is_invocable/is_nothrow_invocable
    // traits to improve compile-times. We don't need all of the generality of the
    // std:: traits and the tag_invoke traits are used heavily through libunifex
    // so optimising them for compile time makes a big difference.

    /**
     * @typedef mireo::tag_invoke_result_t
     * @brief Alias for the result type of invoking tag_invoke.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     */
    using _tag_invoke::tag_invoke_result_t;


    /**
     * @brief Checks if tag_invoke is invocable with the given arguments.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     * @return true if tag_invoke is invocable, otherwise false.
     */
    template <typename CPO, typename... Args>
    inline constexpr bool is_tag_invocable_v
        = (sizeof(_tag_invoke::try_tag_invoke<CPO, Args...>(0)) == sizeof(_tag_invoke::yes_type));

    /**
     * @struct mireo::tag_invoke_result
     * @brief Trait to get the result type of invoking tag_invoke.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     */
    template <typename CPO, typename... Args>
    struct tag_invoke_result : std::conditional_t<is_tag_invocable_v<CPO, Args...>,
                                   _tag_invoke::defer<tag_invoke_result_t, CPO, Args...>, _tag_invoke::empty>
    {
    };

    /**
     * @typedef mireo::is_tag_invocable
     * @brief Trait to check if tag_invoke is invocable.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     */
    template <typename CPO, typename... Args>
    using is_tag_invocable = std::bool_constant<is_tag_invocable_v<CPO, Args...>>;

    /**
     * @brief Checks if tag_invoke is nothrow invocable with the given arguments.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     * @return true if tag_invoke is nothrow invocable, otherwise false.
     */
    template <typename CPO, typename... Args>
    inline constexpr bool is_nothrow_tag_invocable_v = noexcept(_tag_invoke::try_tag_invoke<CPO, Args...>(0));

    /**
     * @typedef mireo::is_nothrow_tag_invocable
     * @brief Trait to check if tag_invoke is nothrow invocable.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     */
    template <typename CPO, typename... Args>
    using is_nothrow_tag_invocable = std::bool_constant<is_nothrow_tag_invocable_v<CPO, Args...>>;

    /**
     * @concept mireo::tag_invocable
     * @brief Concept to check if tag_invoke is invocable.
     * @tparam CPO Customization point object type.
     * @tparam Args Argument types.
     */
    template <typename CPO, typename... Args>
    concept tag_invocable = is_tag_invocable_v<CPO, Args...>;

} // namespace mireo
