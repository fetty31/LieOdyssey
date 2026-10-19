#pragma once

#include <cmath>
#include <limits>

namespace ins_ros::utils::chi_square {

namespace detail {

// Regularized lower incomplete gamma series: P(a, x) for x < a + 1.
inline double gamma_series(double a, double x)
{
    const double gln = std::lgamma(a);

    double ap = a;
    double sum = 1.0 / a;
    double term = sum;

    for (int n = 0; n < 1000; ++n)
    {
        ap += 1.0;
        term *= x / ap;
        sum += term;

        if (std::fabs(term) < std::fabs(sum) * 1e-16)
            break;
    }

    return sum * std::exp(-x + a * std::log(x) - gln);
}

// Regularized upper incomplete gamma continued fraction: Q(a, x) for x >= a + 1.
inline double gamma_continued_fraction(double a, double x)
{
    const double gln = std::lgamma(a);
    const double fpmin = 1e-300;

    double b = x + 1.0 - a;
    double c = 1.0 / fpmin;
    double d = 1.0 / b;
    double h = d;

    for (int i = 1; i <= 1000; ++i)
    {
        const double an = -static_cast<double>(i) * (static_cast<double>(i) - a);

        b += 2.0;

        d = an * d + b;
        if (std::fabs(d) < fpmin)
            d = fpmin;

        c = b + an / c;
        if (std::fabs(c) < fpmin)
            c = fpmin;

        d = 1.0 / d;

        const double delta = d * c;
        h *= delta;

        if (std::fabs(delta - 1.0) < 1e-16)
            break;
    }

    return std::exp(-x + a * std::log(x) - gln) * h;
}

// Regularized lower incomplete gamma function P(a, x).
inline double regularized_gamma_p(double a, double x)
{
    if (x <= 0.0)
        return 0.0;

    if (x < a + 1.0)
        return gamma_series(a, x);

    return 1.0 - gamma_continued_fraction(a, x);
}

} // namespace detail

/**
 * @brief Cumulative distribution function of the chi-square distribution.
 *
 * @param x   Value at which to evaluate the CDF (>= 0).
 * @param dof Degrees of freedom (>= 1).
 * @return P(X <= x), 0.0 for invalid input.
 */
inline double cdf(double x, int dof)
{
    if (dof < 1 || x <= 0.0)
        return 0.0;

    return detail::regularized_gamma_p(
        0.5 * static_cast<double>(dof),
        0.5 * x);
}

/**
 * @brief Quantile (inverse CDF) of the chi-square distribution.
 *
 * The threshold is derived from the requested degrees of freedom (normally the
 * measurement dimension) and confidence level, so nothing has to be
 * hard-coded. The inversion is a bisection on the CDF above and is accurate to
 * far below the precision needed for gating.
 *
 * Example: quantile(3, 0.95) = 7.8147, the classic 95% threshold of a 3D
 * measurement.
 *
 * @param dof        Degrees of freedom (>= 1), normally the measurement dimension.
 * @param confidence Probability content of the interval, in (0, 1).
 * @return Chi-square threshold, or NaN if the input is invalid.
 */
inline double quantile(int dof, double confidence)
{
    if (dof < 1 ||
        !std::isfinite(confidence) ||
        confidence <= 0.0 ||
        confidence >= 1.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Bracket the root: start from the mean of the distribution and double
    // until the CDF reaches the requested confidence.
    double lo = 0.0;
    double hi = static_cast<double>(dof);

    for (int i = 0; i < 200 && cdf(hi, dof) < confidence; ++i)
        hi *= 2.0;

    if (cdf(hi, dof) < confidence)
        return std::numeric_limits<double>::quiet_NaN();

    // Bisection.
    for (int i = 0; i < 200 && (hi - lo) > 1e-13 * (1.0 + hi); ++i)
    {
        const double mid = 0.5 * (lo + hi);

        if (cdf(mid, dof) < confidence)
            lo = mid;
        else
            hi = mid;
    }

    return 0.5 * (lo + hi);
}

} // namespace ins_ros::utils::chi_square
