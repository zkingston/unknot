# Unknot

An implementation of the approach described in "Using Motion Planning for Knot Untangling" by Ladd and Kavraki [1] in C++.
The main script is implemented in `unknot.cc`.
Once compiled, it provides a few command line options:
```bash
./unknot \
  -f <input knot JSON> \
  -o <output trajectory JSON> \
  -s <specific random seed> \
  -v \ # Enable visualization
  -p   # Use the random planner rather than the heuristic search
```

Some basic unknots are provided in the `resources/` folder as a JSON list of points:
- Lebrecht Goeritz's 11 crossing unknot (`goeritz.json`),
- Morwen Thistlethwaite's 15 crossing unknot (`thistlewaite.json`),
- Mitsuyuki Ochiai's 16 crossing unknot (`ochiai.json`)
  
All of these typically solve in under a second on my M2 Macbook Air.
For example:
```bash
./build/unknot -f resources/goeritz.json -v
```
Will find a sequence of moves that untangles Goertitz's unknot using a simple application of heuristics and display them with a simple 3D visualizer.
Visualization of the unknotting process is done using a simple [implot3d](https://github.com/brenocq/implot3d) display.
The planner of [1] can be run by passing the `-p` argument to the script.

# Compiling

Compile the script with CMake using Ninja (not necessary but recommended):
```bash
cmake -Bbuild -GNinja .
cmake --build build
```
CMake should pull down all dependencies besides [`Eigen 3`](https://eigen.tuxfamily.org/index.php?title=Main_Page).
If for some reason visualization is throwing a fit (e.g., something with GLAD, GLFW, etc.), visualization can be disabled in the compilation with the flag `UNKNOT_VIZ`, e.g.,
```
cmake -Bbuild -GNinja -DUNKNOT_VIZ=OFF .
cmake --build build
```

# Other Knot Stuff

I highly recommend looking at [this post by Peter Prevos](https://horizonofreason.com/science/unknot-diagrams-trivial-knot-collection/) for some more information on unknotting.
There is also the [Wikipedia page on unknots](https://en.wikipedia.org/wiki/Unknot) and the [unknotting problem](https://en.wikipedia.org/wiki/Unknotting_problem).
There is also these very helpful articles, [2] and [3] (see also the associated [KnotPlot](https://knotplot.com/) software).

# Why?

This paper has been very compelling to me since I first discovered it!
I love motion planning for wacky, high-dimensional things - thus, fun hobby project.

# Maybe at some point in the future...

- [ ] Add more unknots (e.g., [Wolfgang Haken's](https://horizonofreason.com/images/knots/haken-gordian-knot.png))
- [ ] Convert knot diagrams to ball-and-stick JSON
- [ ] Compare against other unknotting software
- [ ] Parallelism?

# References

- [1] [A. M. Ladd and L. E. Kavraki, "Using Motion Planning for Knot Untangling," International Journal of Robotics Research, vol. 23, no. 7-8, pp. 797–808, 2004.](https://journals.sagepub.com/doi/10.1177/0278364904045469) 
A preprint is available [at this link.](https://kavrakilab.org/publications/ladd-kavraki2004using-motion-planning.pdf)
- [2] [Ligocki, Terry J., and James A. Sethian. "Recognizing knots using simulated annealing." Journal of Knot Theory and Its Ramifications 3.04 (1994): 477-495.](https://www.worldscientific.com/doi/abs/10.1142/S0218216594000356) 
A preprint is available [at this link.](https://cds.cern.ch/record/265879/files/P00024440.pdf?version=1)
- [3] [Scharein, Robert Glenn. Interactive topological drawing. Diss. University of British Columbia, 1998.](https://knotplot.com/thesis/).
