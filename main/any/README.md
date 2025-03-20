# Type Erasure in C++

Type erasure is an advanced C++ design pattern that allows you to abstract away the concrete type of an object while maintaining its behavior. It essentially enables polymorphism without requiring inheritance or virtual functions.

## Principle of Type Erasure

In type erasure:
- **Encapsulation of Type-Specific Logic**: You define a wrapper (like `any.hpp` in this case) that hides the underlying type of the object it holds. This wrapper provides a uniform interface for interacting with the object.
- **Behavioral Polymorphism**: The behavior is preserved using a set of operations defined in the wrapper, like method calls, without exposing the actual type.
- **No Dependency on Base Classes**: Unlike traditional polymorphism, type erasure doesn't rely on inheritance or virtual function tables (vtables). Instead, it uses a combination of templates and dynamic memory allocation to achieve its goal.

Think of it like a "type-neutral box" where you can store objects of different types, but interact with them uniformly.

## Key Components

### `any.hpp`
- Acts as the type-erased wrapper, capable of holding any object, regardless of its type, as long as it satisfies the behavioral interface defined by the wrapper.
- Internally, it often uses techniques like storing a pointer to an abstract concept object, which delegates calls to the concrete implementation.

### `tag_invoke.hpp`
- Used to customize behavior for operations in a highly generic way. It’s part of the "customization point" mechanism, a concept often used in advanced C++ libraries to allow extension without modifying the original code.
- The `tag_invoke` function essentially provides a hook to intercept operations and forward them to the appropriate implementation based on the types.

## Common Usages of Type Erasure

Type erasure is commonly used in scenarios where runtime polymorphism is needed but inheritance or vtables are not desirable. Examples include:
- **Generic Libraries**:
    - Libraries like `std::function`, `std::any`, and `std::optional` use type erasure under the hood. For example, `std::function` can hold any callable object (like lambdas, function pointers, or functors) using type erasure.
- **Runtime Flexibility**:
    - In scenarios where the types of objects are not known until runtime, type erasure can create a uniform interface for handling them.
- **Decoupling Modules**:
    - Type erasure allows defining APIs without exposing concrete types, which is useful for modular programming.
- **Extensibility**:
    - Using customization points (`tag_invoke`) gives you flexibility in extending functionality without tightly coupling components.

## The Core Idea Behind `tag_invoke`

The central concept of `tag_invoke` is to enable **customization and extensibility** for specific operations without modifying the original function or types involved. By leveraging template-based static polymorphism, it provides a mechanism for achieving behavior customization in a modular and decoupled way.

The mechanism involves the following key elements:
1. **A "tag" structure**: 
   - This lightweight struct represents the operation or behavior being customized. It serves as a marker or disambiguator for the customization logic.

2. **The `tag_invoke` free function**:
   - This is a standalone function that acts as the customization entry point. Users implement their desired behavior for specific "tag" structures and type combinations by overloading the `tag_invoke` function.

### Key Characteristics of `tag_invoke`

- **Argument-Dependent Lookup (ADL)**:
    - The C++ compiler employs ADL to resolve the appropriate `tag_invoke` implementation at compile time. This enables the behavior to be statically determined based on the types involved.

- **Customization Points**:
    - Instead of relying on inheritance or virtual dispatch, `tag_invoke` supports generic customization points, making it highly extensible. These customization points allow library developers to define hooks that users can implement for their own types.

- **Static Polymorphism**:
    - Unlike traditional runtime polymorphism, where behavior is determined dynamically via inheritance and virtual tables, `tag_invoke` achieves polymorphism at compile time, offering better performance and flexibility.

### How It Works

Here’s how the `tag_invoke` mechanism works step-by-step:
1. **Define a "tag" structure**:
    - Create a struct that represents the operation being customized. This tag is a simple, lightweight marker.
    ```cpp
    struct MyOperationTag {};
    ```

2. **Implement the `tag_invoke` function**:
    - Write an overload of the `tag_invoke` function for your tag and the relevant types. This function contains the logic for the customized operation.
    ```cpp
    int tag_invoke(MyOperationTag, int x, int y) {
        return x + y; // Customized behavior
    }
    ```

3. **Invoke the operation via the tag**:
    - Use the `tag_invoke` mechanism to perform the operation. The compiler resolves the appropriate overload using ADL.
    ```cpp
    MyOperationTag tag;
    int result = tag_invoke(tag, 2, 3); // Resolves to the custom implementation
    ```

### Benefits of Using `tag_invoke`

- **Separation of Concerns**: Behavior customization is defined externally without modifying the original types or functions.
- **Modularity**: Libraries and user code remain decoupled, enhancing maintainability.
- **Compile-Time Dispatch**: Ensures high performance by avoiding runtime overhead.
