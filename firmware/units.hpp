#pragma once

namespace units {

template <typename T>
constexpr T
Abs(T x) {
    return (x >= 0) ? x : -x;
}

struct Volt {
    int32_t value{};
};

struct Micrometer {
    int32_t value{};
};

struct Count {
    int32_t value{};
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
constexpr Count
operator*(const Rational<Count, Q> a, const Q b) {
    return {static_cast<int32_t>(a.value * b.value)};
}

constexpr Count
operator-(const Count a, const Count b) {
    return {a.value - b.value};
}

constexpr Count
operator-(const Count a) {
    return {-a.value};
}

// todo: Use spaceship operator.
constexpr bool
operator<(const Count a, const Count b) {
    return a.value < b.value;
}

constexpr bool
operator>(const Count a, const Count b) {
    return a.value > b.value;
}

constexpr bool
operator>=(const Count a, const Count b) {
    return a.value >= b.value;
}

constexpr bool
operator<=(const Count a, const Count b) {
    return a.value <= b.value;
}

namespace literals {

constexpr ::units::Micrometer
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

constexpr ::units::Count
operator""_count(uint64_t v) {
    return {static_cast<int32_t>(v)};

    static_assert((5_um).value == 5);
    static_assert((5_ms).value == 5);
    // static_assert((5_count).value == 5);

    static_assert((4_um / 2_ms).value == 2.0f);
    static_assert((50_um / 5_ms).value == 10.0f);
}
}  // namespace literals
}  // namespace units