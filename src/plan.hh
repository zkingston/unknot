#pragma once

#include <fmt/core.h>

#include <limits>
#include <chrono>

#include "knot.hh"

template <typename RealT, bool debug = false>
struct Heuristic
{
    std::size_t max_microseconds = 1e6 * 5;  // 5 seconds
    std::size_t opt_steps = 10;
    std::size_t lip_steps = 100;

    [[nodiscard]] auto unknot(const PolyLine<RealT> &k_start) -> std::vector<PolyLine<RealT>>
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::size_t elapsed_time = 0;

        auto k = k_start;
        std::vector<PolyLine<RealT>> r{k};
        std::size_t i = 0;

        while (k.size() > 4 and elapsed_time < max_microseconds)
        {
            auto istart = std::chrono::high_resolution_clock::now();

            if constexpr (debug)
            {
                fmt::print("Iter {:4d}: {:3d} vrtx\n  {:10.2f} en\n", i++, k.size(), k.energy());
            }

            const auto &[elm_im, removed] = k.template elm<false, debug>();

            if (removed)
            {
                if constexpr (debug)
                {
                    r.insert(r.end(), elm_im.begin(), elm_im.end());
                    k = r.back();
                }
                else
                {
                    k = elm_im;
                    r.emplace_back(k);
                }
            }

            if constexpr (debug)
            {
                fmt::print("  {:10.2f} en {:d} vrtx after elm\n", k.energy(), k.size());
            }

            auto opt_im = k.template opt<debug>(opt_steps);

            if constexpr (debug)
            {
                r.insert(r.end(), opt_im.begin(), opt_im.end());
                k = r.back();
            }
            else
            {
                k = opt_im;
                r.emplace_back(k);
            }

            if constexpr (debug)
            {
                fmt::print("  {:10.2f} en after opt\n", k.energy());
            }

            auto energy = k.energy();
            for (auto j = 0U; j < lip_steps; ++j)
            {
                const auto &[k_prime, _z] = k.random_lip();
                const auto new_energy = k_prime.energy();

                if (new_energy < energy)
                {
                    k = k_prime;
                    energy = new_energy;
                    r.emplace_back(k_prime);
                }
            }

            if constexpr (debug)
            {
                fmt::print("  {:10.2f} en after lip\n", energy);
            }

            auto iend = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(iend - start).count();

            if constexpr (debug)
            {
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(iend - istart);
                fmt::print("  Took {:4d}ms\n", duration.count() / 1000);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if constexpr (debug)
        {
            fmt::print("Plan took {:4d}ms\n", duration.count() / 1000);
        }

        return r;
    }
};

template <typename RealT, bool debug = false>
struct RandomTree
{
    std::size_t max_microseconds = 1e6 * 5;  // 5 seconds
    std::size_t random_iterations = 100;
    std::mt19937 gen{std::random_device{}()};

    [[nodiscard]] auto random_tree(const PolyLine<RealT> &k_start) -> std::vector<PolyLine<RealT>>
    {
        const auto &[elm, removed] = k_start.template elm<true, debug>();
        if (removed)
        {
            if constexpr (debug)
            {
                return elm;
            }
            else
            {
                return std::vector<PolyLine<RealT>>{elm};
            }
        }

        std::vector<PolyLine<RealT>> tree{k_start};
        std::vector<std::size_t> parents{std::numeric_limits<std::size_t>::max()};
        std::vector<RealT> weights{RealT{1.}};

        tree.reserve(random_iterations + 1);
        parents.reserve(random_iterations + 1);
        weights.reserve(random_iterations + 1);

        const auto add_node =
            [&tree, &parents, &weights](
                const PolyLine<RealT> &knot, std::size_t parent, float weight) -> std::size_t
        {
            tree.emplace_back(knot);
            parents.emplace_back(parent);
            weights.emplace_back(weight);
            return tree.size() - 1;
        };

        const auto path = [&tree, &parents](std::size_t idx) -> std::vector<PolyLine<RealT>>
        {
            std::vector<PolyLine<RealT>> r{tree[idx]};
            while (parents[idx] != std::numeric_limits<std::size_t>::max())
            {
                idx = parents[idx];
                r.emplace_back(tree[idx]);
            }
            std::reverse(r.begin(), r.end());
            return r;
        };

        for (auto i = 0U; i < random_iterations; ++i)
        {
            std::discrete_distribution<std::size_t> dist(weights.begin(), weights.end());
            std::size_t idx = dist(gen);
            const auto &k_prime = tree[idx];
            weights[idx] /= 2.F;

            const auto &[lk, t] = k_prime.random_lip();
            const auto new_idx = add_node(lk, idx, t);

            const auto &[elm, removed] = k_prime.template elm<true, debug>();
            if (removed)
            {
                if constexpr (debug)
                {
                    return path(add_node(elm.back(), new_idx, 1.));
                }
                else
                {
                    return path(add_node(elm, new_idx, 1.));
                }
            }
        }

        std::vector<RealT> energies(tree.size());
        std::transform(
            tree.cbegin(), tree.cend(), energies.begin(), [](const auto &k) { return k.energy(); });

        auto min_it = std::min_element(energies.begin(), energies.end());
        const auto min_index = std::distance(energies.begin(), min_it);

        if (min_index != 0)
        {
            return path(min_index);
        }

        std::discrete_distribution<std::size_t> dist(weights.begin(), weights.end());
        return path(dist(gen));
    }

    [[nodiscard]] auto unknot(const PolyLine<RealT> &k_start) -> std::vector<PolyLine<RealT>>
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::size_t elapsed_time = 0;

        auto k = k_start;
        std::vector<PolyLine<RealT>> r{k};
        std::size_t i = 0;

        while (k.size() > 3 and elapsed_time < max_microseconds)
        {
            auto istart = std::chrono::high_resolution_clock::now();

            if constexpr (debug)
            {
                fmt::print("Iter {:4d}: {:3d} vrtx {:10.2f} en\n", i++, k.size(), k.energy());
            }

            auto rt = random_tree(k);
            r.insert(r.end(), rt.begin(), rt.end());

            k = r.back();

            auto iend = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(iend - start).count();

            if constexpr (debug)
            {
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(iend - istart);
                fmt::print("  Took {:4d}ms\n", duration.count() / 1000);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if constexpr (debug)
        {
            fmt::print("Plan took {:4d}ms\n", duration.count() / 1000);
        }

        return r;
    }
};
