#include "knot.hh"
#include "plan.hh"
#include "util.hh"

#ifdef UNKNOT_VIZ
#include "visualize.hh"
#endif

#include <cxxopts.hpp>

auto main(int argc, char **argv) -> int
{
    cxxopts::Options options("Knot", "");

    options.add_options()("d,debug", "Enable debugging")                               //
        ("p,planner", "Use random planner", cxxopts::value<bool>())                    //
        ("s,seed", "Seed", cxxopts::value<std::size_t>())                              //
        ("f,file", "File name of knot to untangle", cxxopts::value<std::string>())     //
        ("o,output", "Output filename for trajectory", cxxopts::value<std::string>())  //
#ifdef UNKNOT_VIZ
        ("v,visualize", "Visualize", cxxopts::value<bool>())  //
#endif
        ;

    auto result = options.parse(argc, argv);

    auto seed = time(0);
    if (result.count("seed"))
    {
        seed = result["seed"].as<std::size_t>();
    }

    srand(seed);
    fmt::print("Seed: {}\n", seed);

    PolyLine<float> k;
    if (result.count("file"))
    {
        auto points = load_knot_json<float>(result["file"].as<std::string>());
        k.points = points;
    }
    else
    {
        k = PolyLine<float>::random(10);
    }

    std::vector<PolyLine<float>> r;

    if (result.count("planner") and result["planner"].as<bool>())
    {
        RandomTree<float, true> planner;
        r = planner.unknot(k);
    }
    else
    {
        Heuristic<float, true> planner;
        r = planner.unknot(k);
    }

    if (result.count("output"))
    {
        save_knot_trajectory_json(r, result["output"].as<std::string>());
    }

#ifdef UNKNOT_VIZ
    if (result.count("visualize") and result["visualize"].as<bool>())
    {
        return display_knots<float>(r);
    }
#endif
}
