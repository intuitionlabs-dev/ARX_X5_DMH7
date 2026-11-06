#pragma once

#include <array>
#include <algorithm>
#include <cfloat>
#include <cmath>


namespace ruckig {

template<typename T>
inline T pow2(T v) {
    return v * v;
}


namespace roots {

// Absolute degeneracy threshold for dimensionful polynomial coefficients.
// FLT_EPSILON is relative to 1 and would erase small, valid time polynomials.
constexpr float coefficient_zero {2.220446049250313e-16f};

// Use own set class on stack for real-time capability
template<typename T, size_t N>
class Set {
protected:
    using Container = typename std::array<T, N>;
    using iterator = typename Container::iterator;

    Container data;
    size_t size {0};

public:
    // Sort when accessing the elements
    const iterator begin() {
        std::sort(data.begin(), data.begin() + size);
        return data.begin();
    }

    const iterator end() {
        return data.begin() + size;
    }

    void insert(T value) {
        data[size] = value;
        ++size;
    }
};


// Set that only inserts positive values
template<typename T, size_t N>
class PositiveSet: public Set<T, N> {
public:
    void insert(T value) {
        if (value >= 0) {
            Set<T, N>::insert(value);
        }
    }
};


//! Calculate all roots of a*x^3 + b*x^2 + c*x + d = 0
inline PositiveSet<float, 3> solve_cubic(float a, float b, float c, float d) {
    PositiveSet<float, 3> roots;

    if (std::abs(d) < coefficient_zero) {
        // First solution is x = 0
        roots.insert(0.0f);

        // Converting to a quadratic equation
        d = c;
        c = b;
        b = a;
        a = 0.0f;
    }

    if (std::abs(a) < coefficient_zero) {
        if (std::abs(b) < coefficient_zero) {
            // Linear equation
            if (std::abs(c) > coefficient_zero) {
                roots.insert(-d / c);
            }

        } else {
            // Quadratic equation
            const float discriminant = c * c - 4 * b * d;
            if (discriminant >= 0) {
                const float inv2b = 1.0f / (2 * b);
                const float y = std::sqrt(discriminant);
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }

    } else {
        // Cubic equation
        const float inva = 1.0f / a;
        const float invaa = inva * inva;
        const float bb = b * b;
        const float bover3a = b * inva / 3;
        const float p = (a * c - bb / 3) * invaa;
        const float halfq = (2 * bb * b - 9 * a * b * c + 27 * a * a * d) / 54 * invaa * inva;
        const float yy = p * p * p / 27 + halfq * halfq;

        constexpr float cos120 = -0.50f;
        constexpr float sin120 = 0.866025403784438646764f;

        if (yy > coefficient_zero) {
            // Sqrt is positive: one real solution
            const float y = std::sqrt(yy);
            const float uuu = -halfq + y;
            const float vvv = -halfq - y;
            const float www = std::abs(uuu) > std::abs(vvv) ? uuu : vvv;
            const float w = std::cbrt(www);
            roots.insert(w - p / (3 * w) - bover3a);
        } else if (yy < -coefficient_zero) {
            // Sqrt is negative: three real solutions
            const float x = -halfq;
            const float y = std::sqrt(-yy);
            float theta;
            float r;

            // Convert to polar form
            if (std::abs(x) > coefficient_zero) {
                theta = (x > 0.0f) ? std::atan(y / x) : (std::atan(y / x) + 3.14159265358979323846f);
                r = std::sqrt(x * x - yy);
            } else {
                // Vertical line
                theta = 3.14159265358979323846f / 2;
                r = y;
            }
            // Calculate cube root
            theta /= 3;
            r = 2 * std::cbrt(r);
            // Convert to complex coordinate
            const float ux = std::cos(theta) * r;
            const float uyi = std::sin(theta) * r;

            roots.insert(ux - bover3a);
            roots.insert(ux * cos120 - uyi * sin120 - bover3a);
            roots.insert(ux * cos120 + uyi * sin120 - bover3a);
        } else {
            // Sqrt is zero: two real solutions
            const float www = -halfq;
            const float w = 2 * std::cbrt(www);

            roots.insert(w - bover3a);
            roots.insert(w * cos120 - bover3a);
        }
    }
    return roots;
}

// Solve resolvent eqaution of corresponding Quartic equation
// The input x must be of length 3
// Number of zeros are returned
inline int solve_resolvent(std::array<float, 3>& x, float a, float b, float c) {
    constexpr float cos120 = -0.50f;
    constexpr float sin120 = 0.866025403784438646764f;

    a /= 3;
    const float a2 = a * a;
    float q = a2 - b / 3;
    const float r = (a * (2 * a2 - b) + c) / 2;
    const float r2 = r * r;
    const float q3 = q * q * q;

    if (r2 < q3) {
        const float qsqrt = std::sqrt(q);
        const float t = std::min(std::max(r / (q * qsqrt), -1.0f), 1.0f);
        q = -2 * qsqrt;

        const float theta = std::acos(t) / 3;
        const float ux = std::cos(theta) * q;
        const float uyi = std::sin(theta) * q;
        x[0] = ux - a;
        x[1] = ux * cos120 - uyi * sin120 - a;
        x[2] = ux * cos120 + uyi * sin120 - a;
        return 3;

    } else {
        float A = -std::cbrt(std::abs(r) + std::sqrt(r2 - q3));
        if (r < 0.0f) {
            A = -A;
        }
        const float B = (0.0f == A ? 0.0f : q / A);

        x[0] = (A + B) - a;
        x[1] = -(A + B) / 2 - a;
        x[2] = std::sqrt(3.0f) * (A - B) / 2;
        if (std::abs(x[2]) < coefficient_zero) {
            x[2] = x[1];
            return 2;
        }

        return 1;
    }
}

//! Calculate all roots of the monic quartic equation: x^4 + a*x^3 + b*x^2 + c*x + d = 0
inline PositiveSet<float, 4> solve_quart_monic(float a, float b, float c, float d) {
    PositiveSet<float, 4> roots;

    if (std::abs(d) < coefficient_zero) {
        if (std::abs(c) < coefficient_zero) {
            roots.insert(0.0f);

            const float D = a * a - 4 * b;
            if (std::abs(D) < coefficient_zero) {
                roots.insert(-a / 2);
            } else if (D > 0.0f) {
                const float sqrtD = std::sqrt(D);
                roots.insert((-a - sqrtD) / 2);
                roots.insert((-a + sqrtD) / 2);
            }
            return roots;
        }

        if (std::abs(a) < coefficient_zero && std::abs(b) < coefficient_zero) {
            roots.insert(0.0f);
            roots.insert(-std::cbrt(c));
            return roots;
        }
    }

    const float a3 = -b;
    const float b3 = a * c - 4 * d;
    const float c3 = -a * a * d - c * c + 4 * b * d;

    std::array<float, 3> x3;
    const int number_zeroes = solve_resolvent(x3, a3, b3, c3);

    float y = x3[0];
    // Choosing Y with maximal absolute value.
    if (number_zeroes != 1) {
        if (std::abs(x3[1]) > std::abs(y)) {
            y = x3[1];
        }
        if (std::abs(x3[2]) > std::abs(y)) {
            y = x3[2];
        }
    }

    float q1, q2, p1, p2;

    float D = y * y - 4 * d;
    if (std::abs(D) < coefficient_zero) {
        q1 = q2 = y / 2;
        D = a * a - 4 * (b - y);
        if (std::abs(D) < coefficient_zero) {
            p1 = p2 = a / 2;
        } else {
            const float sqrtD = std::sqrt(D);
            p1 = (a + sqrtD) / 2;
            p2 = (a - sqrtD) / 2;
        }
    } else {
        const float sqrtD = std::sqrt(D);
        q1 = (y + sqrtD) / 2;
        q2 = (y - sqrtD) / 2;
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    constexpr float eps {16 * coefficient_zero};

    D = p1 * p1 - 4 * q1;
    if (std::abs(D) < eps) {
        roots.insert(-p1 / 2);
    } else if (D > 0.0f) {
        const float sqrtD = std::sqrt(D);
        roots.insert((-p1 - sqrtD) / 2);
        roots.insert((-p1 + sqrtD) / 2);
    }

    D = p2 * p2 - 4 * q2;
    if (std::abs(D) < eps) {
        roots.insert(-p2 / 2);
    } else if (D > 0.0f) {
        const float sqrtD = std::sqrt(D);
        roots.insert((-p2 - sqrtD) / 2);
        roots.insert((-p2 + sqrtD) / 2);
    }

    return roots;
}

//! Calculate the quartic equation: x^4 + b*x^3 + c*x^2 + d*x + e = 0
inline PositiveSet<float, 4> solve_quart_monic(const std::array<float, 4>& polynom) {
    return solve_quart_monic(polynom[0], polynom[1], polynom[2], polynom[3]);
}


//! Evaluate a polynomial of order N at x
template<size_t N>
inline float poly_eval(const std::array<float, N>& p, float x) {
    float retVal = 0.0f;
    if constexpr (N == 0) {
        return retVal;
    }

    if (std::abs(x) < coefficient_zero) {
        retVal = p[N - 1];
    } else if (x == 1.0f) {
        for (int i = N - 1; i >= 0; i--) {
            retVal += p[i];
        }
    } else {
        float xn = 1.0f;

        for (int i = N - 1; i >= 0; i--) {
            retVal += p[i] * xn;
            xn *= x;
        }
    }

    return retVal;
}

// Calculate the derivative poly coefficients of a given poly
template<size_t N>
inline std::array<float, N-1> poly_derivative(const std::array<float, N>& coeffs) {
    std::array<float, N-1> deriv;
    for (size_t i = 0; i < N - 1; ++i) {
        deriv[i] = (N - 1 - i) * coeffs[i];
    }
    return deriv;
}

template<size_t N>
inline std::array<float, N-1> poly_monic_derivative(const std::array<float, N>& monic_coeffs) {
    std::array<float, N-1> deriv;
    deriv[0] = 1.0f;
    for (size_t i = 1; i < N - 1; ++i) {
        deriv[i] = (N - 1 - i) * monic_coeffs[i] / (N - 1);
    }
    return deriv;
}

// Safe Newton Method
constexpr float tolerance {FLT_EPSILON};

// Calculate a single zero of polynom p(x) inside [lbound, ubound]
// Requirements: p(lbound)*p(ubound) < 0, lbound < ubound
template<size_t N, size_t maxIts = 128>
inline float shrink_interval(const std::array<float, N>& p, float l, float h) {
    const float fl = poly_eval(p, l);
    const float fh = poly_eval(p, h);
    if (fl == 0.0f) {
        return l;
    }
    if (fh == 0.0f) {
        return h;
    }
    if (fl > 0.0f) {
        std::swap(l, h);
    }

    float rts = (l + h) / 2;
    float dxold = std::abs(h - l);
    float dx = dxold;
    const auto deriv = poly_derivative(p);
    float f = poly_eval(p, rts);
    float df = poly_eval(deriv, rts);
    float temp;
    for (size_t j = 0; j < maxIts; j++) {
        if ((((rts - h) * df - f) * ((rts - l) * df - f) > 0.0f) || (std::abs(2 * f) > std::abs(dxold * df))) {
            dxold = dx;
            dx = (h - l) / 2;
            rts = l + dx;
            if (l == rts) {
                break;
            }
        } else {
            dxold = dx;
            dx = f / df;
            temp = rts;
            rts -= dx;
            if (temp == rts) {
                break;
            }
        }

        if (std::abs(dx) < tolerance) {
            break;
        }

        f = poly_eval(p, rts);
        df = poly_eval(deriv, rts);
        if (f < 0.0f) {
            l = rts;
        } else {
            h = rts;
        }
    }

    return rts;
}

} // namespace roots

} // namespace ruckig
