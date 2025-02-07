#pragma once
#include <compare>
#include <concepts>

namespace units {
template <typename T, typename... U>
concept IsAnyOf = (std::same_as<T, U> || ...);

template <typename Unit>
concept SignedUnit = requires(Unit u) {
    { +u.value } -> std::signed_integral;
};

template <typename T>
constexpr T
Abs(T x) {
    return (x >= T{0}) ? x : -x;
}

struct Volt {
    int32_t value{};
};

template <typename T>
struct Micrometer {
    T value{};

    explicit constexpr operator Micrometer<int32_t>() const {
        static_assert(std::is_same_v<T, int16_t>);
        return {value};
    }
};

struct Step {
    int32_t value{};

    auto operator<=>(const Step& other) const = default;
};

struct Second {
    float value{};
};

struct Millisecond {
    int32_t value{};
};

struct Microsecond {
    int32_t value{};
};

template <class P, class Q>
struct Rational {
    float value{};
};

template <class P, class Q>
constexpr Rational<P, Q>
operator/(const P p, const Q q) {
    return {static_cast<float>(p.value) / q.value};
}

template <class P>
constexpr float
operator/(const P a, const P b) {
    return {static_cast<float>(a.value) / b.value};
}
template <class P, class Q, class R>
constexpr Rational<P, R>
operator*(const Rational<P, Q> a, const Rational<Q, R> b) {
    return {a.value * b.value};
}

template <class Q>
constexpr Step
operator*(const Rational<Step, Q> a, const Q b) {
    return {static_cast<int32_t>(a.value * b.value)};
}

template <SignedUnit Unit>
constexpr Unit
operator-(const Unit a, const Unit b) {
    return {a.value - b.value};
}

template <SignedUnit Unit>
constexpr Unit
operator-(const Unit a) {
    return {-a.value};
}

namespace literals {

constexpr ::units::Micrometer<int32_t>
operator""_um(uint64_t v) {
    return {static_cast<int32_t>(v)};
}

constexpr ::units::Second
operator""_s(long double v) {
    return {static_cast<float>(v)};
}

constexpr ::units::Millisecond
operator""_ms(uint64_t v) {
    return {static_cast<int32_t>(v)};
}

constexpr ::units::Microsecond
operator""_us(uint64_t v) {
    return {static_cast<int32_t>(v)};
}

constexpr ::units::Step
operator""_step(uint64_t v) {
    return {static_cast<int32_t>(v)};
}

static_assert((5_um).value == 5);
static_assert((5_ms).value == 5);

static_assert((4_um / 2_ms).value == 2.0f);
static_assert((50_um / 5_ms).value == 10.0f);

static_assert((5_step).value == 5);
static_assert((1_step).value == 1);
static_assert(5_step > 1_step);

}  // namespace literals
}  // namespace units