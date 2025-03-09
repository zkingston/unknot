#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <limits>
#include <optional>
#include <algorithm>
#include <random>

#include "constants.hh"

/**
 * Generate a random unit vector on a sphere with given radius.
 *
 * @param radius The radius of the sphere.
 * @return A random unit vector scaled by radius.
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
[[nodiscard]] inline static auto uniform_on_sphere(RealT radius = 1.0F) -> VectorT
{
    static thread_local std::mt19937 gen{std::random_device{}()};
    static thread_local std::normal_distribution<RealT> d{0, 1};

    VectorT s = VectorT::Zero();
    std::generate(s.data(), s.data() + dimension, [&] { return d(gen); });
    s.normalize();

    return s * radius;
}

/**
 * Linear interpolation between two vectors.
 *
 * @param start Starting point.
 * @param target Target point.
 * @param t Interpolation parameter [0,1].
 * @return Interpolated vector between start and target based on parameter t
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
[[nodiscard]] auto interpolate(const VectorT &start, const VectorT &target, const RealT t) -> VectorT
{
    return start + t * (target - start);
}

/**
 * Find the closest points between two line segments and their distance.
 * Adapted from this Stack Overflow answer:
 * https://stackoverflow.com/questions/2824478/shortest-distance-between-two-line-segments
 *
 * @param a0, a1 Endpoints of first segment.
 * @param b0, b1 Endpoints of second segment
 * @return Tuple containing closest points on segments and distance between them.
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
[[nodiscard]] inline static auto
segment_intersect(const VectorT &a0, const VectorT &a1, const VectorT &b0, const VectorT &b1)
    -> std::tuple<std::optional<VectorT>, std::optional<VectorT>, RealT>
{
    // Calculate segment directions and normalize
    VectorT A = a1 - a0;
    VectorT B = b1 - b0;
    const auto mA = A.norm();
    const auto mB = B.norm();
    A /= mA;
    B /= mB;

    const auto &cross = A.cross(B);
    const auto &denom = std::pow(cross.norm(), 2);

    // Handle case where segments are parallel
    if (std::abs(denom) < epsilon)
    {
        const auto &d0 = A.dot(b0 - a0);
        const auto &d1 = A.dot(b1 - a0);

        if (d0 <= 0.F and d1 <= 0)
        {
            if (std::abs(d0) < std::abs(d1))
            {
                return {a0, b0, (a0 - b0).norm()};
            }

            return {a0, b1, (a0 - b1).norm()};
        }
        else if (mA <= d0 and mA <= d1)
        {
            if (std::abs(d0) < std::abs(d1))
            {
                return {a1, b0, (a1 - b0).norm()};
            }

            return {a1, b1, (a1 - b1).norm()};
        }

        return {std::nullopt, std::nullopt, (((d0 * A) + a0) - b0).norm()};
    }

    // Calculate closest points between non-parallel segments
    const auto t = b0 - a0;
    const auto dA = t.dot(B.cross(cross));
    const auto dB = t.dot(A.cross(cross));
    const auto t0 = dA / denom;
    const auto t1 = dB / denom;

    // Calculate projected points
    VectorT pA = a0 + (A * t0);  // Projected closest point on segment A
    VectorT pB = b0 + (B * t1);  // Projected closest point on segment B

    // Clamp projections to segments
    pA = (t0 < 0) ? a0 : ((t0 > mA) ? a1 : pA);
    pB = (t1 < 0) ? b0 : ((t1 > mB) ? b1 : pB);

    // Recalculate if points are outside segments
    if (t0 < 0 or t0 > mA)
    {
        auto dot = B.dot(pA - b0);
        dot = std::clamp(dot, RealT{0}, mB);
        pB = b0 + (B * dot);
    }

    if (t1 < 0 or t1 > mB)
    {
        auto dot = A.dot(pB - a0);
        dot = std::clamp(dot, RealT{0}, mA);
        pA = a0 + (A * dot);
    }

    return {pA, pB, (pA - pB).norm()};
}

/**
 * Calculate distance between two line segments.
 *
 * @param a0, a1 Endpoints of first segment.
 * @param b0, b1 Endpoints of second segment.
 * @return Distance between segments.
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
[[nodiscard]] inline static auto
segment_distance(const VectorT &a0, const VectorT &a1, const VectorT &b0, const VectorT &b1) -> RealT
{
    const auto &[_a, _b, distance] = segment_intersect<RealT>(a0, a1, b0, b1);
    return distance;
}

/**
 * Calculate how far an endpoint of the first segment can move toward a target before the segment collides
 * with the other.
 *
 * @param p0, p1 Endpoints of first segment.
 * @param q0, q1 Endpoints of second segment.
 * @param target Target position for p0.
 * @return Interpolation factor between [0,1] indicating safe movement distance.
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
[[nodiscard]] inline static auto move_segment_till_collision(
    const VectorT &p0,
    const VectorT &p1,
    const VectorT &q0,
    const VectorT &q1,
    const VectorT &target) -> RealT
{
    const auto &v_p1 = p0 - p1;
    const auto &v_r = target - p0;
    const auto &v_q = q1 - q0;
    const auto &v_o1 = p1 - q0;

    const auto denom1 = v_r.cross(v_q).dot(v_o1);
    return (std::abs(denom1) > epsilon) ?
               std::clamp(-v_p1.cross(v_q).dot(v_o1) / denom1, RealT{0.}, RealT{1.}) :
               RealT{1.};
}

template <typename RealT>
struct PolyLine
{
    using VectorT = Eigen::Vector<RealT, dimension>;
    using size_type = typename std::vector<VectorT>::size_type;

    std::vector<VectorT> points;

    explicit PolyLine()
    {
    }

    explicit PolyLine(const std::vector<VectorT> &points) : points(points)
    {
    }

    static auto random(size_type n) -> PolyLine
    {
        std::vector<VectorT> pts(n);
        std::generate(pts.begin(), pts.end(), [] { return VectorT::Random(); });
        return PolyLine(std::move(pts));
    }

    [[nodiscard]] auto size() const -> size_type
    {
        return points.size();
    }

    auto operator[](std::size_t idx) -> VectorT &
    {
        return points[idx];
    }

    auto operator[](std::size_t idx) const -> const VectorT &
    {
        return points[idx];
    }

    auto remove(size_type index)
    {
        points.erase(points.begin() + index);
    }

    /**
     * Calculate axis-aligned bounding box for the polyline
     * @return Pair of vectors representing min and max corners of the AABB
     */
    [[nodiscard]] auto aabb() const -> std::pair<VectorT, VectorT>
    {
        if (points.empty())
        {
            return {VectorT::Zero(), VectorT::Zero()};
        }

        VectorT max = VectorT::Constant(std::numeric_limits<float>::min());
        VectorT min = VectorT::Constant(std::numeric_limits<float>::max());

        for (const auto &p : points)
        {
            min = min.cwiseMin(p);
            max = max.cwiseMax(p);
        }

        return {min, max};
    }

    /**
     * Scale the polyline to fit within a normalized [-1, 1] box
     */
    auto scale()
    {
        const auto &[min, max] = aabb();

        const auto &range = 1. / (max - min).minCoeff();
        const auto &middle = (min + max) * 0.5F;

        std::transform(
            points.begin(),
            points.end(),
            points.begin(),
            [&middle, &range](const auto &p) { return 2.0F * (p - middle) * range; });
    }

    /**
     * Get segment starting at vertex index (wraps around at end).
     *
     * @return Pair of vectors representing segment endpoints.
     */
    [[nodiscard]] auto segment(std::size_t index) const -> std::pair<const VectorT &, const VectorT &>
    {
        return {points[index % points.size()], points[(index + 1) % points.size()]};
    }

    /**
     * Calculate minimum distance between any two non-adjacent segments.
     */
    [[nodiscard]] auto clearance() const -> RealT
    {
        RealT clearance = std::numeric_limits<RealT>::max();
        for (auto i = 0U; i < points.size(); ++i)
        {
            const auto &s1 = segment(i);
            for (auto j = 2U; j < points.size() - 1; ++j)
            {
                const auto &s2 = segment(i + j);
                clearance =
                    std::min(clearance, segment_distance<RealT>(s1.first, s1.second, s2.first, s2.second));
            }
        }

        return clearance;
    }

    /**
     * Calculate energy based on inverse square of segment distances. Higher energy indicates closer segments.
     */
    [[nodiscard]] auto energy() const -> RealT
    {
        RealT energy = 0;
        for (auto i = 0U; i < points.size(); ++i)
        {
            const auto &s1 = segment(i);
            for (auto j = i + 2; j < points.size() - 1; ++j)
            {
                const auto &s2 = segment(j);
                energy += 1. / std::pow(segment_distance<RealT>(s1.first, s1.second, s2.first, s2.second), 2);
            }
        }

        return energy;
    }

    /**
     * Find potential safe interpolation points when moving a vertex toward target.
     *
     * @param index Index of vertex to move.
     * @param target Target to move vertex towards.
     * @return Vector of interpolation factors between 0 and 1.
     */
    [[nodiscard]] auto find_interpolation_intersections(size_type index, const VectorT &target) const
        -> std::vector<RealT>
    {
        std::vector<RealT> zeros;
        zeros.reserve(points.size());

        const auto ni = (index + 1) % points.size();
        const auto pi = (index + points.size() - 1) % points.size();

        const auto &p0 = points[index];
        const auto &p1 = points[ni];
        const auto &p2 = points[pi];

        // Find all possible collision points with other segments
        for (auto i = 0U; i < points.size(); ++i)
        {
            const auto i_next = (i + 1) % points.size();

            const auto &q0 = points[i];
            const auto &q1 = points[i_next];

            // Check moving p0-p1 segment
            if (i != index and i_next != index and i != ni and i_next != ni)
            {
                zeros.emplace_back(move_segment_till_collision<RealT>(p0, p1, q0, q1, target));
            }

            // Check moving p0-p2 segment
            if (i != index and i_next != index and i != pi and i_next != pi)
            {
                zeros.emplace_back(move_segment_till_collision<RealT>(p0, p2, q0, q1, target));
            }
        }

        std::sort(zeros.begin(), zeros.end());

        if (zeros.front() == 0.F)
        {
            zeros.erase(zeros.begin());
        }

        // Remove duplicate values
        zeros.erase(
            std::unique(
                zeros.begin(),
                zeros.end(),
                [](const auto &a, const auto &b) { return std::abs(a - b) < epsilon; }),
            zeros.end());

        return zeros;
    }

    /**
     * Elementary move algorithm for polyline simplification. Tries to remove vertices by attemping to
     * interpolate them to the midpoint of the adjacent vertices. That is, a straight line is possible between
     * adjacent vertices, thus this vertex is redundant.
     *
     * @tparam only_one Only remove one vertex
     * @tparam debug Provide a debug output of a vector intermediate states.
     */
    template <bool only_one = false, bool debug = false>
    [[nodiscard]] auto elm() const -> std::
        conditional_t<debug, std::pair<std::vector<PolyLine<RealT>>, bool>, std::pair<PolyLine<RealT>, bool>>
    {
        auto k_prime = *this;

        bool removed = false;
        std::vector<PolyLine<RealT>> result;
        for (auto i = 0U; i < k_prime.size() and k_prime.size() > 3;)
        {
            const auto p0 = k_prime[i];
            const auto &p_p = k_prime[(i + k_prime.size() - 1) % k_prime.size()];
            const auto &p_n = k_prime[(i + 1) % k_prime.size()];
            const auto &midpoint = interpolate(p_p, p_n, RealT{0.5});
            const auto &zeros = k_prime.find_interpolation_intersections(i, midpoint);

            // Ignore if full extent not possible
            if (zeros.empty() or std::abs(zeros.back() - 1.) > epsilon)
            {
                i++;
                continue;
            }

            // Check if vertex can be safely moved to all intermediate positions
            bool valid = true;
            for (const auto &z : zeros)
            {
                k_prime[i] = interpolate(p0, midpoint, z);
                auto clr = k_prime.clearance();

                if (clr < tolerance)
                {
                    valid = false;
                    break;
                }
            }

            k_prime[i] = p0;

            if (valid)
            {
                // If in debug mode, record intermediate states
                if constexpr (debug)
                {
                    for (const auto &z : zeros)
                    {
                        k_prime[i] = interpolate(p0, midpoint, z);
                        result.emplace_back(k_prime);
                    }
                }

                // Remove vertex and restart from beginning
                k_prime.remove(i);
                removed = true;
                i = 0;

                if constexpr (debug)
                {
                    result.emplace_back(k_prime);
                }

                if constexpr (only_one)
                {
                    break;
                }
            }
            else
            {
                i++;
            }
        }

        if constexpr (debug)
        {
            return {result, removed};
        }
        else
        {
            return {k_prime, removed};
        }
    }

    /**
     * Linear Interpolation Planner. Moves a vertex towards a target as far as possible without causing
     * self-intersection.
     *
     * @param index Index of vertex to move.
     * @param target Target to move vertex towards.
     */
    [[nodiscard]] auto lip(size_type index, const VectorT &target) const -> std::pair<PolyLine<RealT>, RealT>
    {
        auto k_prime = *this;
        const auto &p0 = points[index];
        auto zeros = find_interpolation_intersections(index, target);

        RealT best = RealT{0.};
        for (auto &z : zeros)
        {
            k_prime[index] = interpolate(p0, target, z);
            auto clr = k_prime.clearance();

            // // If collision detected, back off until safe
            if (clr < tolerance)
            {
                // Binary search to find largest safe z
                while (z > tolerance and clr < tolerance)
                {
                    z /= RealT{2.};
                    k_prime[index] = interpolate(p0, target, z);
                    clr = k_prime.clearance();
                }

                // Update best if valid position found
                if (clr > tolerance)
                {
                    best = std::max(z, best);
                }

                break;
            }

            best = z;
        }

        k_prime[index] = p0 + best * (target - p0);
        return {k_prime, best};
    }

    /**
     * Random Linear Interpolation Planner. Moves a random vertex towards a random target as far as possible
     * without causing self-intersection.
     */
    [[nodiscard]] auto random_lip() const -> std::pair<PolyLine<RealT>, RealT>
    {
        return lip(rand() % size(), uniform_on_sphere<RealT>(std::sqrt(RealT{3.})));
    }

    /**
     * Optimize PolyLine by moving vertices to reduce energy.
     *
     * @tparam debug If true, records intermediate states.
     * @param max_iter Maximum number of optimization iterations.
     */
    template <bool debug = false>
    [[nodiscard]] auto opt(size_type max_iter = 1) const
        -> std::conditional_t<debug, std::vector<PolyLine<RealT>>, PolyLine<RealT>>
    {
        auto k_prime = *this;
        std::vector<PolyLine<RealT>> r;
        std::vector<VectorT> deltas(k_prime.size());

        for (auto iter = 0U; iter < max_iter; ++iter)
        {
            std::fill(deltas.begin(), deltas.end(), VectorT::Zero());

            // Calculate repulsive forces between segments
            for (auto i = 0U; i < k_prime.size(); ++i)
            {
                const auto &s1 = k_prime.segment(i);
                for (auto j = 2U; j < k_prime.size() - 1; ++j)
                {
                    const auto &s2 = k_prime.segment(i + j);

                    const auto &[a, b, dist] =
                        segment_intersect<RealT>(s1.first, s1.second, s2.first, s2.second);

                    if (not a or not b)
                    {
                        continue;
                    }

                    // Force is inversely proportional to square of distance
                    const auto &vec = (*a - *b) / std::pow(dist, 2);

                    // Apply forces to segment endpoints
                    deltas[i] += vec;
                    deltas[(i + 1) % k_prime.size()] += vec;
                    deltas[(i + j) % k_prime.size()] -= vec;
                    deltas[(i + j + 1) % k_prime.size()] -= vec;
                }
            }

            // Add repulsive forces between vertices
            for (auto i = 0U; i < k_prime.size(); ++i)
            {
                for (auto j = i + 1; j < k_prime.size(); ++j)
                {
                    const auto &diff = k_prime[j] - k_prime[i];
                    const auto &scaled = diff / diff.squaredNorm();
                    deltas[i] -= scaled;
                    deltas[j] += scaled;
                }
            }

            // Apply forces to each vertex, moving as far as possible without collisions
            for (auto i = 0U; i < k_prime.size(); ++i)
            {
                const auto radius = std::sqrt(RealT{3});
                VectorT target = k_prime[i] + deltas[i].normalized();
                const auto norm = target.norm();

                if (norm > radius)
                {
                    target *= radius / norm;
                }

                k_prime = k_prime.lip(i, target).first;
            }

            if constexpr (debug)
            {
                r.emplace_back(k_prime);
            }
        }

        if constexpr (debug)
        {
            return r;
        }
        else
        {
            return k_prime;
        }
    }
};
