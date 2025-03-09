#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <fmt/core.h>
#include <fmt/ostream.h>

#include "knot.hh"
#include "constants.hh"

template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
  : ostream_formatter
{
};

/**
 * Loads 3D points from a JSON file
 *
 * @param filepath Path to the JSON file
 * @return Vector of Eigen::Vector3f points
 * @throws std::runtime_error if file cannot be opened or parsed
 */
template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
auto load_knot_json(const std::filesystem::path &filepath) -> std::vector<VectorT>
{
    // Check if file exists
    if (!std::filesystem::exists(filepath))
    {
        throw std::runtime_error("File does not exist: " + filepath.string());
    }

    // Open and read the file
    std::ifstream file(filepath);
    if (not file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filepath.string());
    }

    // Parse JSON
    auto data = nlohmann::json::parse(file);

    std::vector<VectorT> points;
    points.reserve(data.size());

    for (const auto &point : data)
    {
        VectorT v;
        for (auto i = 0U; i < dimension; ++i)
        {
            v[i] = point[i];
        }

        points.emplace_back(v);
    }

    return points;
}

template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
auto get_knot_json(const PolyLine<RealT> &knot) -> nlohmann::json
{
    nlohmann::json j;

    for (const auto &point : knot.points)
    {
        nlohmann::json jp;

        for (auto i = 0U; i < dimension; ++i)
        {
            jp.emplace_back(point[i]);
        }

        j.emplace_back(jp);
    }

    return j;
}

template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
auto save_knot_json(const PolyLine<RealT> &knot, const std::filesystem::path &filepath)
{
    auto j = get_knot_json(knot);
    std::ofstream file(filepath);
    file << j.dump();
}

template <typename RealT, typename VectorT = Eigen::Vector<RealT, dimension>>
auto save_knot_trajectory_json(
    const std::vector<PolyLine<RealT>> &knots,
    const std::filesystem::path &filepath)
{
    nlohmann::json j;

    for (const auto &knot : knots)
    {
        auto jk = get_knot_json(knot);
        j.emplace_back(jk);
    }

    std::ofstream file(filepath);
    file << j.dump();
}
