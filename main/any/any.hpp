/*==========================================================================================
    Copyright (c) 2001-2023 Mireo, EU

    Extremely efficient DBC file format parser, built on top of Boost Spirit.

    Strictly follows the grammar rules from the last updated DBC specification available at
    http://mcu.so/Microcontroller/Automotive/dbc-file-format-documentation_compress.pdf
===========================================================================================*/

/**
 * @file any.hpp
 * @brief This file contains the implementation of a type-erasing wrapper for various callable objects.
 *
 * This acts as the type-erased wrapper. It can hold any object, regardless of its type, as long as it 
 * satisfies the behavioral interface defined by the wrapper.
 * Inside, it often uses techniques like storing a pointer to an abstract concept object, which delegates 
 * calls to the concrete implementation. 
 * 
 * This file provides the `any`, `any_copyable`, `any_function`, and `any_copyable_function` templates
 * for type-erased storage and invocation of callable objects.
 * 
 * Type erasure is an advanced C++ design pattern that allows you to abstract away the concrete type of 
 * an object while maintaining its behavior. It essentially enables polymorphism without requiring 
 * inheritance or virtual functions.
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
 * @see https://github.com/facebookexperimental/libunifex/blob/main/doc/type_erasure.md
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

/**

Read:

https://github.com/facebookexperimental/libunifex/blob/main/doc/type_erasure.md

Synopsis:

template <typename... CPOs>
class any;

    Type-erasing wrapper is a move-only wrapper that implements a small-object optimisation
    that allows storing the wrapped object inline if it is smaller than the inline buffer,
    and heap-allocates storage for the object if it does not fit in the inline buffer.

template <typename... CPOs>
class any_copyable;

    Copyable variant of `mireo::any`. Requires wrapped objects to be copy-constructible and
    copy-assignable.

template <class Signature, typename... CPOs>
class any_function;

    Similar to std::function that can be further customized with CPOs; like mireo::any.

    mireo::any_function<int(int, char**)> the_main = main;
    the_main(argc, argv);

template <class Signature, typename... CPOs>
class any_copyable_function;

    Copyable variant of `mireo::any_function`.

    mireo::any_copyable_function<int(int, char**)> main1 = &main;
    auto main2 = main1;

*/

#include "tag_invoke.hpp"
#include <memory>
#include <type_traits>


namespace mireo
{

    //
    // this_
    //

    struct this_
    {
    };

    template <typename T>
    inline constexpr bool is_this_v = false;
    template <>
    inline constexpr bool is_this_v<this_> = true;
    template <>
    inline constexpr bool is_this_v<this_&> = true;
    template <>
    inline constexpr bool is_this_v<this_&&> = true;
    template <>
    inline constexpr bool is_this_v<const this_> = true;
    template <>
    inline constexpr bool is_this_v<const this_&> = true;
    template <>
    inline constexpr bool is_this_v<const this_&&> = true;

    namespace detail
    {

        struct _ignore
        {
            template <typename T>
            _ignore(T&&) noexcept
            {
            }
        };

        template <typename>
        struct _replace_this;

        template <>
        struct _replace_this<void>
        {
            template <typename Arg, typename>
            using apply = Arg;

            template <typename Arg>
            static Arg&& get(Arg&& arg, _ignore) noexcept
            {
                return (Arg &&) arg;
            }
        };

        template <>
        struct _replace_this<this_>
        {
            template <typename, typename T>
            using apply = T;

            template <typename T>
            static T&& get(_ignore, T& obj) noexcept
            {
                return (T &&) obj;
            }
        };

        template <>
        struct _replace_this<this_&>
        {
            template <typename, typename T>
            using apply = T&;

            template <typename T>
            static T& get(_ignore, T& obj) noexcept
            {
                return obj;
            }
        };

        template <>
        struct _replace_this<this_&&>
        {
            template <typename, typename T>
            using apply = T&&;

            template <typename T>
            static T&& get(_ignore, T& obj) noexcept
            {
                return (T &&) obj;
            }
        };

        template <>
        struct _replace_this<const this_&>
        {
            template <typename, typename T>
            using apply = const T&;

            template <typename T>
            static const T& get(_ignore, T& obj) noexcept
            {
                return obj;
            }
        };

        template <>
        struct _replace_this<const this_&&>
        {
            template <typename, typename T>
            using type = const T&&;

            template <typename T>
            static const T&& get(_ignore, T& obj) noexcept
            {
                return (const T&&)obj;
            }
        };

        template <typename Arg>
        using _normalize_t = std::conditional_t<is_this_v<Arg>, Arg, void>;

        template <typename T>
        using replace_this = _replace_this<_normalize_t<T>>;

        template <typename Arg, typename T>
        using replace_this_t = typename replace_this<Arg>::template apply<Arg, T>;

        template <typename Arg>
        using replace_this_with_void_ptr_t = std::conditional_t<is_this_v<Arg>, void*, Arg>;

        template <bool...>
        struct _extract_this
        {
            template <typename TFirst, typename... TRest>
            TFirst&& operator()(TFirst&& first, TRest&&...) const noexcept
            {
                return (TFirst &&) first;
            }
        };

        template <bool... IsThis>
        struct _extract_this<false, IsThis...>
        {
            template <typename... TRest>
            decltype(auto) operator()(_ignore, TRest&&... rest) const noexcept
            {
                static_assert(sizeof...(IsThis) > 0, "Arguments to extract_this");
                return _extract_this<IsThis...> {}((TRest &&) rest...);
            }
        };

        template <typename... Ts>
        using extract_this = _extract_this<is_this_v<Ts>...>;

        //
        // overload
        //

        template <typename CPO, typename Sig>
        struct _cpo_t
        {
            struct type;
        };

        // This type will have the associated namespaces
        // of CPO (by inheritance) but not those of Sig.
        template <typename CPO, typename Sig>
        struct _cpo_t<CPO, Sig>::type : CPO
        {
            constexpr type() = default;
            constexpr type(CPO) noexcept
            {
            }

            using base_cpo_t = CPO;
            using type_erased_signature_t = Sig;
        };

        template <typename CPO, typename Enable = void>
        struct base_cpo
        {
            using type = CPO;
        };

        template <typename CPO>
        struct base_cpo<CPO, std::void_t<typename CPO::base_cpo_t>>
        {
            using type = typename CPO::base_cpo_t;
        };

        template <typename CPO, typename Sig>
        inline constexpr typename _cpo_t<CPO, Sig>::type _cpo {};

        template <typename Sig>
        struct _sig
        {
        };

        template <typename Sig>
        inline constexpr _sig<Sig> const sig {};

        template <typename CPO>
        using base_cpo_t = typename base_cpo<CPO>::type;

        //
        // vtable
        //

        template <typename CPO, typename T, typename Ret, typename... Args>
        Ret _vtable_invoke(CPO cpo, replace_this_with_void_ptr_t<Args>... args) noexcept
        {
            void* this_ptr = extract_this<Args...> {}(args...);
            return ((CPO &&) cpo)(replace_this<Args>::get((decltype(args)&&)args, *static_cast<T*>(this_ptr))...);
        }

        template <typename CPO, typename Sig = typename CPO::type_erased_signature_t>
        class vtable_entry;

        template <typename CPO, typename Ret, typename... Args>
        class vtable_entry<CPO, Ret(Args...) noexcept>
        {
        public:
            using fn_t = Ret(base_cpo_t<CPO>, replace_this_with_void_ptr_t<Args>...) noexcept;

            constexpr fn_t* get() const noexcept
            {
                return _fn;
            }

            template <typename T>
            static constexpr vtable_entry create() noexcept
            {
                return vtable_entry { &_vtable_invoke<base_cpo_t<CPO>, T, Ret, Args...> };
            }

        private:
            explicit constexpr vtable_entry(fn_t* fn) noexcept
                : _fn(fn)
            {
            }

            fn_t* _fn;
        };

        template <typename CPO, typename Ret, typename... Args>
        class vtable_entry<CPO, Ret(Args...)>
        {
        public:
            using fn_t = Ret(base_cpo_t<CPO>, replace_this_with_void_ptr_t<Args>...);

            constexpr fn_t* get() const noexcept
            {
                return _fn;
            }

            template <typename T>
            static constexpr vtable_entry create() noexcept
            {
                return vtable_entry { &_vtable_invoke<base_cpo_t<CPO>, T, Ret, Args...> };
            }

        private:
            explicit constexpr vtable_entry(fn_t* fn) noexcept
                : _fn(fn)
            {
            }

            fn_t* _fn;
        };

        template <typename... CPOs>
        class vtable : private vtable_entry<CPOs>...
        {

            explicit constexpr vtable(vtable_entry<CPOs>... entries) noexcept
                : vtable_entry<CPOs> { entries }...
            {
            }

        public:
            template <typename T>
            static const vtable* inst()
            {
                static constexpr vtable v { vtable_entry<CPOs>::template create<T>()... };
                return &v;
            };

            template <typename CPO>
            constexpr auto get() const noexcept -> typename vtable_entry<CPO>::fn_t*
            {
                const vtable_entry<CPO>& entry = *this;
                return entry.get();
            }
        };

        //
        // get_wrapped_object / with_forwarding_tag_invoke
        //

        struct _get_wrapped_object_cpo
        {
            template <typename T>
            requires tag_invocable<_get_wrapped_object_cpo, T>
            auto operator()(T&& wrapper) const noexcept(is_nothrow_tag_invocable_v<_get_wrapped_object_cpo, T>)
                -> tag_invoke_result_t<_get_wrapped_object_cpo, T>
            {
                return mireo::tag_invoke(*this, static_cast<T&&>(wrapper));
            }
        };

        inline constexpr _get_wrapped_object_cpo get_wrapped_object {};

        template <typename Derived, typename CPO, typename Sig>
        struct _with_forwarding_tag_invoke;

        template <typename Derived, typename CPO>
        using with_forwarding_tag_invoke =
            typename _with_forwarding_tag_invoke<Derived, base_cpo_t<CPO>, typename CPO::type_erased_signature_t>::type;

        // noexcept(false) specialisation
        template <typename Derived, typename CPO, typename Ret, typename... Args>
        struct _with_forwarding_tag_invoke<Derived, CPO, Ret(Args...)>
        {
            struct type
            {
                friend Ret tag_invoke(CPO cpo, replace_this_t<Args, Derived>... args)
                {
                    auto& wrapper = extract_this<Args...> {}(args...);
                    auto& wrapped = get_wrapped_object(wrapper);
                    return static_cast<CPO&&>(cpo)(
                        replace_this<Args>::get(static_cast<decltype(args)&&>(args), wrapped)...);
                }
            };
        };

        // noexcept(true) specialisation
        template <typename Derived, typename CPO, typename Ret, typename... Args>
        struct _with_forwarding_tag_invoke<Derived, CPO, Ret(Args...) noexcept>
        {
            struct type
            {
                friend Ret tag_invoke(CPO cpo, replace_this_t<Args, Derived>... args) noexcept
                {
                    auto& wrapper = extract_this<Args...> {}(args...);
                    auto& wrapped = get_wrapped_object(wrapper);
                    return static_cast<CPO&&>(cpo)(
                        replace_this<Args>::get(static_cast<decltype(args)&&>(args), wrapped)...);
                }
            };
        };

        //
        // with_type_erased_tag_invoke
        //

        template <typename Derived, typename CPO, typename Sig>
        struct _with_type_erased_tag_invoke;

        template <typename Derived, typename CPO, typename Ret, typename... Args>
        struct _with_type_erased_tag_invoke<Derived, CPO, Ret(Args...)>
        {
            struct type
            {
                friend Ret tag_invoke(base_cpo_t<CPO> cpo, replace_this_t<Args, Derived>... args)
                {
                    using cpo_t = base_cpo_t<CPO>;
                    auto&& t = extract_this<Args...> {}((decltype(args)&&)args...);
                    void* ptr = get_object_address(t);
                    auto* fn = get_vtable(t)->template get<CPO>();
                    return fn((cpo_t &&) cpo, replace_this<Args>::get((decltype(args)&&)args, ptr)...);
                }
            };
        };

        //
        // any_heap_allocated_storage
        //

        template <typename T, typename Allocator>
        class _any_heap_allocated_storage
        {
            struct state;

            using allocator_type = typename std::allocator_traits<Allocator>::template rebind_alloc<state>;
            using allocator_traits = std::allocator_traits<allocator_type>;

            // This is the state that is actually heap-allocated.
            struct state
            {
                template <typename... Args>
                requires std::constructible_from<T, Args...>
                explicit state(std::allocator_arg_t, allocator_type allocator, std::in_place_type_t<T>, Args&&... args)
                    : object(static_cast<Args&&>(args)...)
                    , allocator(std::move(allocator))
                {
                }

                [[no_unique_address]] T object;
                [[no_unique_address]] allocator_type allocator;
            };

            // This is the base-class object that holds the pointer to the
            // heap-allocated state.
            struct base
            {
                template <typename... Args>
                requires std::constructible_from<T, Args...> base(
                    std::allocator_arg_t, allocator_type allocator, std::in_place_type_t<T>, Args&&... args)
                {
                    _state = allocator_traits::allocate(allocator, 1);
                    allocator_traits::construct(allocator, _state, std::allocator_arg, allocator, std::in_place_type<T>,
                        static_cast<Args&&>(args)...);
                }

                template <typename... Args>
                requires(!std::same_as<Allocator, allocator_type> && std::constructible_from<T, Args...>) explicit base(
                    std::allocator_arg_t, Allocator allocator, std::in_place_type_t<T>, Args&&... args)
                    : base(std::allocator_arg, allocator_type(std::move(allocator)), std::in_place_type<T>,
                        static_cast<Args&&>(args)...)
                {
                }

                base(const base& other) requires std::copy_constructible<T>
                    : base(std::allocator_arg, const_cast<const allocator_type&>(other._state->allocator),
                          std::in_place_type<T>, const_cast<const T&>(other._state->object))
                {
                }

                base(base&& other) noexcept
                    : _state(std::exchange(other._state, nullptr))
                {
                }

                ~base()
                {
                    if (_state != nullptr)
                    {
                        allocator_type alloc = std::move(_state->allocator);
                        _state->~state();
                        std::allocator_traits<allocator_type>::deallocate(alloc, _state, 1);
                    }
                }

            private:
                friend T& tag_invoke(tag_t<get_wrapped_object>, base& self) noexcept
                {
                    return self._state->object;
                }

                friend const T& tag_invoke(tag_t<get_wrapped_object>, const base& self) noexcept
                {
                    return self._state->object;
                }

                state* _state;
            };

            template <typename... CPOs>
            struct concrete final
            {
                class type : public base, private detail::with_forwarding_tag_invoke<type, CPOs>...
                {
                    using base::base;
                };
            };

        public:
            template <typename... CPOs>
            using type = typename concrete<CPOs...>::type;
        };

        template <typename T, typename Allocator, typename... CPOs>
        using any_heap_allocated_storage = typename _any_heap_allocated_storage<T, Allocator>::template type<CPOs...>;

        //
        // _destroy_cpo / _move_construct_cpo / _copy_construct_cpo / _invoke_cpo
        //

        struct _destroy_cpo
        {
            using type_erased_signature_t = void(this_&) noexcept;

            template <typename T>
            void operator()(T& object) const noexcept
            {
                object.~T();
            }
        };

        struct _move_construct_cpo
        {
            using type_erased_signature_t = void(void* p, this_&& src) noexcept;

            template <typename T>
            void operator()(void* p, T&& src) const noexcept
            {
                ::new (p) T(static_cast<T&&>(src));
            }
        };

        struct _copy_construct_cpo
        {
            using type_erased_signature_t = void(void* p, const this_& src) noexcept;

            template <typename T>
            void operator()(void* p, const T& src) const noexcept
            {
                ::new (p) T(src);
            }
        };

        template <typename Sig>
        struct _invoke_cpo;

        template <typename R, typename... Args>
        struct _invoke_cpo<R(Args...)>
        {
            using type_erased_signature_t = R(this_&, Args...);

            template <class F>
            static constexpr bool with_tag_invoke_v = mireo::is_tag_invocable_v<_invoke_cpo, F, Args...>;

            template <class F>
            static constexpr bool nothrow_invoke_v
                = with_tag_invoke_v<F> ? mireo::is_nothrow_tag_invocable_v<_invoke_cpo, F, Args...>
                                       : std::is_nothrow_invocable_v<F, Args...>;

            template <typename F>
            R operator()(F&& fn, Args... arg) const noexcept(nothrow_invoke_v<F>)
            {
                if constexpr (with_tag_invoke_v<F>)
                {
                    return mireo::tag_invoke(*this, (F &&) fn, (Args &&) arg...);
                }
                else
                {
                    return ((F &&) fn)((Args &&) arg...);
                }
            }
        };

    } // namespace detail

    //
    // with_type_erased_tag_invoke
    //
    // When defining a type-erasing wrapper type, Derived, you can privately inherit
    // from this class to have the type opt-in to customising the specified CPO.
    //

	/**
	 * @brief Type alias for type-erased tag invocation.
	 *
	 * This type alias is used to define a type-erased tag invocation mechanism.
	 * It utilizes the `detail::_with_type_erased_tag_invoke` template to achieve
	 * type erasure for the given `Derived` and `CPO` (Customization Point Object)
	 * types, based on the `type_erased_signature_t` defined within the `CPO`.
	 *
	 * @tparam Derived The derived type that will be used in the type-erased tag invocation.
	 * @tparam CPO The Customization Point Object type that defines the `type_erased_signature_t`.
	 */
    template <typename Derived, typename CPO>
    using with_type_erased_tag_invoke =
        typename detail::_with_type_erased_tag_invoke<Derived, CPO, typename CPO::type_erased_signature_t>::type;

    //
    // _any_object
    //

	/**
	 * @brief A template struct representing an object that can store any type.
	 *
	 * @tparam InlineSize The size of the inline storage.
	 * @tparam DefaultAllocator The default allocator type.
	 * @tparam IsCopyable A boolean indicating if the object is copyable.
	 * @tparam CPOs Customization point objects.
	 */
    template <std::size_t InlineSize, typename DefaultAllocator, bool IsCopyable, typename... CPOs>
    struct _any_object
    {
        // Pad size/alignment out to allow storage of at least a pointer.

		/**
		 * @brief The alignment of the inline storage, which is at least the alignment of a pointer.
		 */
        static constexpr std::size_t inline_alignment = alignof(void*);

		/**
		 * @brief The size of the inline storage, which is at least the size of a pointer.
		 */
        static constexpr std::size_t inline_size = InlineSize < sizeof(void*) ? sizeof(void*) : InlineSize;

        // move-constructor is (must be) noexcept

		/**
		 * @brief Determines if a type can be stored in place.
		 *
		 * @tparam T The type to check.
		 * @return true if the type can be stored in place, false otherwise.
		 */		
        template <typename T>
        static constexpr bool can_be_stored_inplace_v = (sizeof(T) <= inline_size && alignof(T) <= inline_alignment);

		/**
		 * @brief Indicates if the object is copyable.
		 */
        static constexpr bool copyable_v = IsCopyable;

		/**
		 * @brief A nested class representing the type of the object.
		 */
        class type;
    };

    /**
     * @brief A class that provides type-erased storage for any type.
     *
     * @tparam InlineSize The size of the inline storage.
     * @tparam DefaultAllocator The default allocator type.
     * @tparam IsCopyable A boolean indicating if the type is copyable.
     * @tparam CPOs Customization point objects.
     */
    template <std::size_t InlineSize, typename DefaultAllocator, bool IsCopyable, typename... CPOs>
    class _any_object<InlineSize, DefaultAllocator, IsCopyable, CPOs...>::type
        : private with_type_erased_tag_invoke<type, CPOs>...
    {
        using vtable_t = std::conditional_t<_any_object::copyable_v,
            detail::vtable<detail::_destroy_cpo, detail::_move_construct_cpo, detail::_copy_construct_cpo, CPOs...>,
            detail::vtable<detail::_destroy_cpo, detail::_move_construct_cpo, CPOs...>>;

        const vtable_t* _vtable = nullptr;
        alignas(inline_alignment) std::byte _storage[inline_size];

    public:
        /**
         * @brief Default constructor.
         */
        type() = default;

		/**
		 * @brief Constructs the type with an object.
		 * 
		 * @tparam T The type of the object.
		 * @param object The object to store.
		 */
        template <typename T>
        requires(!std::is_same_v<type, std::remove_cvref_t<T>>) type(T&& object)
            : type(std::in_place_type<std::remove_cvref_t<T>>, static_cast<T&&>(object))
        {
        }

		/**
		 * @brief Constructs the type with an allocator and a value.
		 * 
		 * @tparam T The type of the value.
		 * @tparam Allocator The type of the allocator.
		 * @param allocator The allocator to use.
		 * @param value The value to store.
		 */
        template <typename T, typename Allocator>
        explicit type(std::allocator_arg_t, Allocator allocator, T&& value) noexcept
            : type(std::allocator_arg, std::move(allocator), std::in_place_type<std::remove_cvref_t<T>>,
                static_cast<T&&>(value))
        {
        }

		/**
		 * @brief Constructs the type in-place with the given arguments.
		 * 
		 * @tparam T The type to construct.
		 * @tparam Args The types of the arguments.
		 * @param args The arguments to use for construction.
		 */
        template <typename T, typename... Args>
        requires _any_object::can_be_stored_inplace_v<T>
        explicit type(std::in_place_type_t<T>, Args&&... args)
            : _vtable(vtable_t::template inst<T>())
        {
            ::new (static_cast<void*>(&_storage)) T(static_cast<Args&&>(args)...);
        }

		/**
		 * @brief Constructs the type in-place with the given arguments and allocator.
		 * 
		 * @tparam T The type to construct.
		 * @tparam Allocator The type of the allocator.
		 * @tparam Args The types of the arguments.
		 * @param allocator The allocator to use.
		 * @param args The arguments to use for construction.
		 */
        template <typename T, typename... Args>
        requires(!_any_object::can_be_stored_inplace_v<T>) explicit type(std::in_place_type_t<T>, Args&&... args)
            : type(std::allocator_arg, DefaultAllocator(), std::in_place_type<T>, static_cast<Args&&>(args)...)
        {
        }

		/**
		 * @brief Constructs the type in-place with the given arguments and allocator.
		 * 
		 * @tparam T The type to construct.
		 * @tparam Alloc The type of the allocator.
		 * @tparam Args The types of the arguments.
		 * @param alloc The allocator to use.
		 * @param args The arguments to use for construction.
		 */
        template <typename T, typename Allocator, typename... Args>
        requires _any_object::can_be_stored_inplace_v<T>
        explicit type(std::allocator_arg_t, Allocator, std::in_place_type_t<T>, Args&&... args) noexcept
            : type(std::in_place_type<T>, static_cast<Args&&>(args)...)
        {
        }
	
		/**
		 * @brief Constructs an object of type `type` with heap allocation.
		 * 
		 * This constructor is used when the type `T` cannot be stored in place.
		 * It allocates memory on the heap for the object of type `T` and constructs it
		 * using the provided allocator and arguments.
		 * 
		 * @tparam T The type of the object to be stored.
		 * @tparam Alloc The type of the allocator to be used.
		 * @tparam Args The types of the arguments to be forwarded to the constructor of `T`.
		 * 
		 * @param alloc The allocator to be used for heap allocation.
		 * @param args The arguments to be forwarded to the constructor of `T`.
		 */
        template <typename T, typename Alloc, typename... Args>
        requires(!_any_object::can_be_stored_inplace_v<T>) explicit type(
            std::allocator_arg_t, Alloc alloc, std::in_place_type_t<T>, Args&&... args)
            : _vtable(vtable_t::template inst<detail::any_heap_allocated_storage<T, Alloc, CPOs...>>())
        {
            ::new (static_cast<void*>(&_storage)) detail::any_heap_allocated_storage<T, Alloc, CPOs...>(
                std::allocator_arg, std::move(alloc), std::in_place_type<T>, static_cast<Args&&>(args)...);
        }

		/**
		 * @brief Copy constructor.
		 * 
		 * @param other The other type to copy from.
		 */
        type(const type& other) noexcept requires _any_object::copyable_v : _vtable(other._vtable)
        {
            if (_vtable)
            {
                auto* copy_cons = _vtable->template get<detail::_copy_construct_cpo>();
                copy_cons(detail::_copy_construct_cpo {}, &_storage, get_object_address(other));
            }
        }

		/**
		 * @brief Move constructor.
		 * 
		 * @param other The other type to move from.
		 */
        type(type&& other) noexcept
            : _vtable(other._vtable)
        {
            if (_vtable)
            {
                auto* move_cons = _vtable->template get<detail::_move_construct_cpo>();
                move_cons(detail::_move_construct_cpo {}, &_storage, &other._storage);
            }
        }

		/**
		 * @brief Destructor.
		 */
        ~type() noexcept
        {
            if (_vtable)
            {
                auto* destroy = _vtable->template get<detail::_destroy_cpo>();
                destroy(detail::_destroy_cpo {}, &_storage);
            }
        }

		/**
		 * @brief Copy assignment operator.
		 * 
		 * @param other The other type to copy from.
		 * @return A reference to this type.
		 */
        type& operator=(const type& other) noexcept requires _any_object::copyable_v
        {
            if (std::addressof(other) == this)
			{
                return *this;
			}

            if (_vtable)
            {
                auto* destroy = _vtable->template get<detail::_destroy_cpo>();
                destroy(detail::_destroy_cpo {}, &_storage);
            }
            _vtable = other._vtable;
            if (_vtable)
            {
                auto* copy_cons = _vtable->template get<detail::_copy_construct_cpo>();
                copy_cons(detail::_copy_construct_cpo {}, &_storage, get_object_address(other));
            }
            return *this;
        }

		/**
		 * @brief Move assignment operator.
		 * 
		 * @param other The other type to move from.
		 * @return A reference to this type.
		 */
        type& operator=(type&& other) noexcept
        {
            if (std::addressof(other) == this)
			{
                return *this;
			}	

            if (_vtable)
            {
                auto* destroy = _vtable->template get<detail::_destroy_cpo>();
                destroy(detail::_destroy_cpo {}, &_storage);
            }
            _vtable = other._vtable;
            if (_vtable)
            {
                auto* move_cons = _vtable->template get<detail::_move_construct_cpo>();
                move_cons(detail::_move_construct_cpo {}, &_storage, &other._storage);
            }
            return *this;
        }

		/**
		 * @brief Assignment operator for a value.
		 * 
		 * @tparam T The type of the value.
		 * @param value The value to assign.
		 * @return A reference to this type.
		 */
        template <typename T>
        requires _any_object::can_be_stored_inplace_v<std::remove_cvref_t<T>> &&(
            !std::is_same_v<type, std::remove_cvref_t<T>>)type&
        operator=(T&& value) noexcept
        {
            if (_vtable)
            {
                auto* destroy = _vtable->template get<detail::_destroy_cpo>();
                destroy(detail::_destroy_cpo {}, &_storage);
            }
            using value_type = std::remove_cvref_t<T>;
            ::new (static_cast<void*>(&_storage)) value_type(static_cast<T&&>(value));
            _vtable = vtable_t::template inst<value_type>();
            return *this;
        }

		/**
		 * @brief Assignment operator for a value.
		 * 
		 * @tparam T The type of the value.
		 * @param value The value to assign.
		 * @return A reference to this type.
		 */
        template <typename T>
        requires(!_any_object::can_be_stored_inplace_v<std::remove_cvref_t<T>>)
            && (!std::is_same_v<type, std::remove_cvref_t<T>>)type& operator=(T&& value) noexcept
        {
            if (_vtable)
            {
                auto* destroy = _vtable->template get<detail::_destroy_cpo>();
                destroy(detail::_destroy_cpo {}, &_storage);
            }
            using value_type = detail::any_heap_allocated_storage<std::remove_cvref_t<T>, DefaultAllocator, CPOs...>;
            ::new (static_cast<void*>(&_storage)) value_type(std::allocator_arg, DefaultAllocator {},
                std::in_place_type<std::remove_cvref_t<T>>, static_cast<T&&>(value));
            _vtable = vtable_t::template inst<value_type>();
            return *this;
        }


		/**
		 * @brief Checks if the type contains a value.
		 * 
		 * @return true if the type contains a value, false otherwise.
		 */		
        explicit operator bool() const noexcept
        {
            return _vtable != nullptr;
        }

    private:
		/**
		 * @brief Gets the vtable of the type.
		 * 
		 * @param self The type to get the vtable from.
		 * @return The vtable of the type.
		 */
        friend const vtable_t* get_vtable(const type& self) noexcept
        {
            return self._vtable;
        }

		/**
		 * @brief Gets the address of the stored object.
		 * 
		 * @param self The type to get the object address from.
		 * @return The address of the stored object.
		 */		
        friend void* get_object_address(const type& self) noexcept
        {
            return const_cast<void*>(static_cast<const void*>(&self._storage));
        }
    };

    //
    // _any_function_t
    //

    /**
     * @brief Forward declaration of the _any_function_t template class.
     *
     * This class template is a forward declaration and is used to define a type
     * that can store and invoke any callable object. The template parameters
     * specify the signature of the callable, the inline storage size, the allocator
     * type, whether the callable is copyable, and any additional customization
     * point objects (CPOs).
     *
     * @tparam Sig The signature of the callable object.
     * @tparam InlineSize The size of the inline storage for the callable object.
     * @tparam DefaultAllocator The allocator type used for dynamic memory allocation.
     * @tparam IsCopyable A boolean indicating whether the callable object is copyable.
     * @tparam CPOs Additional customization point objects.
     */
    template <typename Sig, size_t InlineSize, typename DefaultAllocator, bool IsCopyable, typename... CPOs>
    class _any_function_t;

    /**
     * @class _any_function_t
     * @brief A template class that represents a type-erased callable object.
     *
     * This class is a specialization of the _any_function_t template for a given function signature.
     * It inherits from _any_object and provides an operator() to invoke the stored callable object.
     *
     * @tparam R The return type of the callable object.
     * @tparam Args The argument types of the callable object.
     * @tparam InlineSize The size of the inline storage for the callable object.
     * @tparam DefaultAllocator The allocator type used for dynamic memory allocation.
     * @tparam IsCopyable A boolean indicating whether the callable object is copyable.
     * @tparam CPOs Customization point objects for additional functionality.
     */
    template <typename R, typename... Args, size_t InlineSize, typename DefaultAllocator, bool IsCopyable,
        typename... CPOs>
    class _any_function_t<R(Args...), InlineSize, DefaultAllocator, IsCopyable, CPOs...>
        : public _any_object<InlineSize, DefaultAllocator, IsCopyable, detail::_invoke_cpo<R(Args...)>, CPOs...>::type
    {
        using invoke_cpo = detail::_invoke_cpo<R(Args...)>;
        using base_t = typename _any_object<InlineSize, DefaultAllocator, IsCopyable, detail::_invoke_cpo<R(Args...)>,
            CPOs...>::type;

    public:
        using result_type = R;

        using base_t::base_t; // use constructors from base

        R operator()(Args... arg) const noexcept
        {
            auto* invoke = get_vtable(*this)->template get<invoke_cpo>();
            return invoke(invoke_cpo {}, get_object_address(*this), (Args &&) arg...);
        }
    };

    //
    // basic_any / basic_any_copyable
    //

    /**
     * @brief Alias template for a copyable version of _any_object.
     *
     * This alias template defines a type that represents a copyable version of
     * the _any_object with the specified inline size, allocator, and customization
     * point objects (CPOs).
     *
     * @tparam InlineSize The size of the inline storage for the object.
     * @tparam DefaultAllocator The allocator type to use for dynamic memory allocation.
     * @tparam CPOs The customization point objects to be used with the _any_object.
     */
    template <std::size_t InlineSize, typename DefaultAllocator, typename... CPOs>
    using basic_any_copyable_t = typename _any_object<InlineSize, DefaultAllocator, true, CPOs...>::type;

    /**
     * @brief Alias template for a basic any type with customizable inline size, allocator, and customization point
     * objects (CPOs).
     *
     * @tparam InlineSize The size of the inline storage for the any object.
     * @tparam DefaultAllocator The allocator type to use for dynamic memory allocation.
     * @tparam CPOs Variadic template parameter pack for customization point objects.
     */
    template <std::size_t InlineSize, typename DefaultAllocator, typename... CPOs>
    using basic_any_t = typename _any_object<InlineSize, DefaultAllocator, false, CPOs...>::type;

    /**
     * @brief Alias template for a copyable version of basic_any with customizable inline size and allocator.
     *
     * @tparam InlineSize The size of the inline storage for the any object.
     * @tparam DefaultAllocator The allocator type to be used for dynamic memory allocation.
     * @tparam CPOs Customization point objects to be used with the any object.
     */
    template <std::size_t InlineSize, typename DefaultAllocator, auto&... CPOs>
    using basic_any_copyable = basic_any_copyable_t<InlineSize, DefaultAllocator, mireo::tag_t<CPOs>...>;

    /**
     * @brief Alias template for basic_any_t with specified inline size, allocator, and customization point objects
     * (CPOs).
     *
     * @tparam InlineSize The size of the inline storage.
     * @tparam DefaultAllocator The allocator type to use.
     * @tparam CPOs The customization point objects to be used.
     */
    template <std::size_t InlineSize, typename DefaultAllocator, auto&... CPOs>
    using basic_any = basic_any_t<InlineSize, DefaultAllocator, mireo::tag_t<CPOs>...>;

    //
    // any / any_copyable
    //

    template <typename... CPOs>
    using any_copyable_t = basic_any_copyable_t<4 * sizeof(void*), std::allocator<std::byte>, CPOs...>;

    template <auto&... CPOs>
    using any_copyable = any_copyable_t<mireo::tag_t<CPOs>...>;

    template <typename... CPOs>
    using any_t = basic_any_t<4 * sizeof(void*), std::allocator<std::byte>, CPOs...>;

    template <auto&... CPOs>
    using any = any_t<mireo::tag_t<CPOs>...>;

    //
    // basic_any_function / basic_any_copyable_function
    //

    /**
     * @brief Alias template for a basic any function type.
     *
     * This alias template defines a type for a basic any function with the specified
     * signature, inline size, allocator, and customization point objects (CPOs).
     *
     * @tparam Sig The function signature type.
     * @tparam InlineSize The size of the inline storage for the function object.
     * @tparam DefaultAllocator The allocator type to use for dynamic memory allocation.
     * @tparam CPOs The customization point objects (CPOs) to be used.
     */
    template <typename Sig, size_t InlineSize, typename DefaultAllocator, typename... CPOs>
    using basic_any_function_t = _any_function_t<Sig, InlineSize, DefaultAllocator, false, CPOs...>;

    /**
     * @brief Alias template for basic_any_function.
     *
     * This alias template defines a type `basic_any_function` which is a specialization of `basic_any_function_t`.
     *
     * @tparam Sig The function signature type.
     * @tparam InlineSize The size of the inline storage.
     * @tparam DefaultAllocator The allocator type to use by default.
     * @tparam CPOs Customization point objects (CPOs) to be used.
     */
    template <typename Sig, size_t InlineSize, typename DefaultAllocator, auto&... CPOs>
    using basic_any_function = basic_any_function_t<Sig, InlineSize, DefaultAllocator, mireo::tag_t<CPOs>...>;

    /**
     * @brief Alias template for a copyable function wrapper.
     *
     * This alias template defines a type for a copyable function wrapper with a specified signature,
     * inline storage size, allocator, and customization point objects (CPOs).
     *
     * @tparam Sig The function signature type.
     * @tparam InlineSize The size of the inline storage for the function object.
     * @tparam DefaultAllocator The allocator type to use for dynamic memory allocation.
     * @tparam CPOs Customization point objects (CPOs) to be used with the function wrapper.
     */
    template <typename Sig, size_t InlineSize, typename DefaultAllocator, typename... CPOs>
    using basic_any_copyable_function_t = _any_function_t<Sig, InlineSize, DefaultAllocator, true, CPOs...>;

    /**
     * @brief Alias template for a copyable function with customizable inline size and allocator.
     *
     * This alias template defines a type `basic_any_copyable_function` which is a specialization
     * of `basic_any_copyable_function_t` with the provided signature, inline size, allocator,
     * and customization point objects (CPOs).
     *
     * @tparam Sig The function signature type.
     * @tparam InlineSize The size of the inline storage for the function object.
     * @tparam DefaultAllocator The allocator type to use for dynamic memory allocation.
     * @tparam CPOs The customization point objects to be used.
     */
    template <typename Sig, size_t InlineSize, typename DefaultAllocator, auto&... CPOs>
    using basic_any_copyable_function
        = basic_any_copyable_function_t<Sig, InlineSize, DefaultAllocator, mireo::tag_t<CPOs>...>;

    //
    // any_function / any_copyable_function
    //

    /**
     * @brief Alias template for a function type that can hold any callable object.
     *
     * This alias template defines a type `any_function_t` which is a specialization of
     * `basic_any_function_t` with a fixed buffer size and allocator. It can hold any
     * callable object that matches the signature `Sig` and supports customization
     * point objects (CPOs).
     *
     * @tparam Sig The function signature that the callable object must match.
     * @tparam CPOs Customization point objects that can be used with the callable object.
     */
    template <typename Sig, typename... CPOs>
    using any_function_t = basic_any_function_t<Sig, 4 * sizeof(void*), std::allocator<std::byte>, CPOs...>;

    /**
     * @brief Type alias for a type-erased function wrapper.
     *
     * This type alias defines a type-erased function wrapper that can store any callable object
     * matching the signature `Sig`. The storage size is set to 4 times the size of a pointer.
     * The allocator used for dynamic memory allocation is `std::allocator<std::byte>`.
     * Additional customization points can be specified via `CPOs`.
     *
     * @tparam Sig The function signature that the callable object must match.
     * @tparam CPOs Additional customization points for the function wrapper.
     */
    template <typename Sig, auto&... CPOs>
    using any_function = any_function_t<Sig, mireo::tag_t<CPOs>...>;

    /**
     * @brief A type-erased, copyable function wrapper.
     *
     * This template alias defines a type-erased, copyable function wrapper that can store any callable object
     * (such as a function, lambda, or functor) with a specific signature. The callable object is stored in a
     * dynamically allocated buffer with a fixed size, and it can be copied using the specified allocator.
     *
     * @tparam Sig The function signature of the callable object (e.g., `void(int)`).
     * @tparam CPOs Customization point objects (CPOs) that can be used to customize the behavior of the function
     * wrapper.
     *
     * @note The buffer size is set to 4 times the size of a pointer (`4 * sizeof(void*)`), which should be sufficient
     *       to store most small callable objects without requiring additional heap allocations.
     */
    template <typename Sig, typename... CPOs>
    using any_copyable_function_t
        = basic_any_copyable_function_t<Sig, 4 * sizeof(void*), std::allocator<std::byte>, CPOs...>;

    /**
     * @brief A type-erased wrapper for copyable functions.
     *
     * This template alias defines a type-erased wrapper for functions that are copyable.
     * It uses the `any_copyable_function_t` template to create a type-erased function
     * object that can store any callable object matching the signature `Sig` and
     * supporting the specified customization point objects (CPOs).
     *
     * @tparam Sig The function signature (e.g., `void(int)`).
     * @tparam CPOs The customization point objects that the function must support.
     */
    template <typename Sig, auto&... CPOs>
    using any_copyable_function = any_copyable_function_t<Sig, mireo::tag_t<CPOs>...>;

    //
    // any_invoke
    //

    /**
     * @brief A constexpr instance of the _invoke_cpo template for type erasure.
     *
     * This template variable is used to create a constexpr instance of the
     * detail::_invoke_cpo template, which is a part of the type erasure mechanism.
     *
     * @tparam Sig The signature of the function to be invoked.
     */
    template <typename Sig>
    inline constexpr detail::_invoke_cpo<Sig> any_invoke {};

    //
    // overload
    //

    template <auto& CPO, typename Sig>
    using overload_t = typename detail::_cpo_t<tag_t<CPO>, Sig>::type;

    /**
     * @brief Overloads a customization point object (CPO) for a given signature.
     *
     * This function template provides a mechanism to overload a customization point object (CPO)
     * for a specific function signature. It returns a reference to the appropriate type-erased
     * function object that matches the given signature.
     *
     * @tparam Sig The function signature for which the CPO is being overloaded.
     * @tparam CPO The customization point object type.
     * @param cpo The customization point object instance.
     * @param sig An optional parameter used to deduce the function signature (defaulted to an empty instance).
     * @return A constant reference to the type-erased function object that matches the given signature.
     */
    template <typename Sig, typename CPO>
    constexpr typename detail::_cpo_t<CPO, Sig>::type const& overload(CPO const&, detail::_sig<Sig> = {}) noexcept
    {
        return detail::_cpo<CPO, Sig>;
    }

} // namespace mireo
