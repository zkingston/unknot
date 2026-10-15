# Credits

Citations follow APA 7; the reference list is at the end.

## Method and code

The planner (vertex removal, straight-line moves, energy relaxation, random trees) is Ladd and Kavraki's (2004).

## Diagrams

Gauss codes of the named diagrams are from Burton et al. (2024, Appendix A); the original sources below are the ones Burton et al. cite in their Table 1.

| Preset | Source |
| --- | --- |
| H, J | Henrich & Kauffman (2014), Figure 4 |
| Culprit | Kauffman & Lambropoulou (2011) |
| Monster | Petronio & Zanellati (2016); Curtis (2001) |
| Goeritz (3D) | Goeritz (1934) |
| Thistlethwaite (3D) | Petronio & Zanellati (2016), Figure 9; Curtis (2001) |
| Ochiai I (3D), Ochiai II, Ochiai III | Ochiai (1990), Figures 1–3 |
| Ochiai IV (Suzuki) | Ochiai (1990), Figure 4, a modified diagram originally due to Suzuki |
| Tuzun–Sikora | Tuzun & Sikora (2018), Figure 8 |
| Freedman–He–Wang | Freedman et al. (1994), Figure 6.1 |
| Petronio–Zanellati 31, 120, 138; Haken's Gordian unknot | Petronio & Zanellati (2016) |
| D28, D43, Petronio–Zanellati 78, Fake Freedman–He–Wang, Ochiai II reduced | Burton et al. (2024): §3 (D28, D43, PZ78, reduced from PZ138), §4.1 (Fake FHW), §2 (Ochiai II reduced) |
| DeepMind 42, DeepMind very hard #0–384 | Applebaum et al. (2025), Figure 17 for the 42-crossing diagram; data from Google DeepMind (2025) |
| Trefoil, figure-eight, cinquefoil, three-twist | PD codes of 3₁, 4₁, 5₁ and 5₂ from The Knot Atlas (n.d.) |

The DeepMind data are licensed CC BY 4.0.

`tools/hard-unknots.py` certifies every diagram an unknot and draws it offline with SnapPy and its spherogram module (Culler et al., n.d.), whose orthogonal layout follows Tamassia (1987).

## Algorithms

- Strand pickup (a jammed curve lifts a strand that passes over everything and lays it on a route crossing fewer strands): after the `pickup` simplification of SnapPy's spherogram module (Culler et al., n.d.), carried out in 3D.
- Segment distance: Ericson (2004, §5.1.9).
- Segment–triangle test: Möller & Trumbore (1997).
- Tube frames: Wang et al. (2008).
- Smoothing: corner cutting after Chaikin (1974), with every cut checked as a Delta move.
- PD layout for typed codes: Tutte (1963).
- Image import: thresholding by Otsu (1979) or Sauvola & Pietikäinen (2000); thinning by Zhang & Suen (1984); maximum-weight matching (Edmonds, 1965) ported from van Rantwijk (n.d.).
- Invariants: the Jones polynomial from the Kauffman (1987) bracket, summed crossing by crossing over the pairings of the open strands; the Alexander polynomial by fraction-free elimination (Bareiss, 1968). PD and braid conventions follow The Knot Atlas (n.d.) and KnotInfo (Livingston & Moore, n.d.).
- Drawing: polyline simplification (Douglas & Peucker, 1973; Ramer, 1972) and Catmull–Rom splines (Catmull & Rom, 1974).

## Type

Iosevka Term Slab 34.9.0 (Li, n.d.), SIL Open Font License 1.1; full text in `src/fonts/OFL.txt`.

## References

Applebaum, T., Blackwell, S., Davies, A., Edlich, T., Juhász, A., Lackenby, M., Tomašev, N., & Zheng, D. (2025). The unknotting number, hard unknot diagrams, and reinforcement learning. *Experimental Mathematics*. Advance online publication. https://doi.org/10.1080/10586458.2025.2542174

Bareiss, E. H. (1968). Sylvester's identity and multistep integer-preserving Gaussian elimination. *Mathematics of Computation, 22*(103), 565–578. https://doi.org/10.1090/S0025-5718-1968-0226829-0

Burton, B. A., Chang, H.-C., Löffler, M., Maria, C., de Mesmay, A., Schleimer, S., Sedgwick, E., & Spreer, J. (2024). Hard diagrams of the unknot. *Experimental Mathematics, 33*(3), 482–500. https://doi.org/10.1080/10586458.2022.2161676

Catmull, E., & Rom, R. (1974). A class of local interpolating splines. In R. E. Barnhill & R. F. Riesenfeld (Eds.), *Computer aided geometric design* (pp. 317–326). Academic Press. https://doi.org/10.1016/B978-0-12-079050-0.50020-5

Chaikin, G. M. (1974). An algorithm for high-speed curve generation. *Computer Graphics and Image Processing, 3*(4), 346–349. https://doi.org/10.1016/0146-664X(74)90028-8

Culler, M., Dunfield, N. M., Goerner, M., & Weeks, J. R. (n.d.). *SnapPy* (Version 3.3.2) [Computer software]. https://snappy.computop.org

Curtis, F. (2001). *Unknot equivalence*. http://f2.org/maths/kt/unknoteq.html

Douglas, D. H., & Peucker, T. K. (1973). Algorithms for the reduction of the number of points required to represent a digitized line or its caricature. *Cartographica, 10*(2), 112–122. https://doi.org/10.3138/FM57-6770-U75U-7727

Edmonds, J. (1965). Maximum matching and a polyhedron with 0,1-vertices. *Journal of Research of the National Bureau of Standards, Section B: Mathematics and Mathematical Physics, 69B*(1–2), 125–130. https://doi.org/10.6028/jres.069B.013

Ericson, C. (2004). *Real-time collision detection*. CRC Press. https://doi.org/10.1201/b14581

Freedman, M. H., He, Z.-X., & Wang, Z. (1994). Möbius energy of knots and unknots. *Annals of Mathematics, 139*(1), 1–50. https://doi.org/10.2307/2946626

Goeritz, L. (1934). Bemerkungen zur Knotentheorie. *Abhandlungen aus dem Mathematischen Seminar der Universität Hamburg, 10*(1), 201–210. https://doi.org/10.1007/BF02940674

Google DeepMind. (2025). *Very hard unknots* [Data set]. CC BY 4.0. https://storage.googleapis.com/gdm-unknotting/very_hard_unknots.csv

Henrich, A., & Kauffman, L. H. (2014). Unknotting unknots. *The American Mathematical Monthly, 121*(5), 379–390. https://doi.org/10.4169/amer.math.monthly.121.05.379

Kauffman, L. H. (1987). State models and the Jones polynomial. *Topology, 26*(3), 395–407. https://doi.org/10.1016/0040-9383(87)90009-7

Kauffman, L. H., & Lambropoulou, S. (2011). Hard unknots and collapsing tangles. In L. H. Kauffman, S. Lambropoulou, S. Jablan, & J. H. Przytycki (Eds.), *Introductory lectures on knot theory* (pp. 187–247). World Scientific. https://doi.org/10.1142/9789814313001_0009

Ladd, A. M., & Kavraki, L. E. (2004). Using motion planning for knot untangling. *The International Journal of Robotics Research, 23*(7–8), 797–808. https://doi.org/10.1177/0278364904045469

Livingston, C., & Moore, A. H. (n.d.). *KnotInfo: Table of knot invariants*. https://knotinfo.math.indiana.edu

Möller, T., & Trumbore, B. (1997). Fast, minimum storage ray-triangle intersection. *Journal of Graphics Tools, 2*(1), 21–28. https://doi.org/10.1080/10867651.1997.10487468

Ochiai, M. (1990). Non-trivial projections of the trivial knot. *Astérisque, 192*, 7–10. https://www.numdam.org/item/AST_1990__192__7_0/

Otsu, N. (1979). A threshold selection method from gray-level histograms. *IEEE Transactions on Systems, Man, and Cybernetics, 9*(1), 62–66. https://doi.org/10.1109/TSMC.1979.4310076

Petronio, C., & Zanellati, A. (2016). Algorithmic simplification of knot diagrams: New moves and experiments. *Journal of Knot Theory and Its Ramifications, 25*(10), Article 1650059. https://doi.org/10.1142/S0218216516500590

Ramer, U. (1972). An iterative procedure for the polygonal approximation of plane curves. *Computer Graphics and Image Processing, 1*(3), 244–256. https://doi.org/10.1016/S0146-664X(72)80017-0

Sauvola, J., & Pietikäinen, M. (2000). Adaptive document image binarization. *Pattern Recognition, 33*(2), 225–236. https://doi.org/10.1016/S0031-3203(99)00055-2

Steele, G. L., Jr., Lea, D., & Flood, C. H. (2014). Fast splittable pseudorandom number generators. In *Proceedings of the 2014 ACM International Conference on Object Oriented Programming Systems Languages & Applications* (pp. 453–472). Association for Computing Machinery. https://doi.org/10.1145/2660193.2660195

Tamassia, R. (1987). On embedding a graph in the grid with the minimum number of bends. *SIAM Journal on Computing, 16*(3), 421–444. https://doi.org/10.1137/0216030

The Knot Atlas. (n.d.). http://katlas.org

Tutte, W. T. (1963). How to draw a graph. *Proceedings of the London Mathematical Society, s3-13*(1), 743–767. https://doi.org/10.1112/plms/s3-13.1.743

Tuzun, R. E., & Sikora, A. S. (2018). Verification of the Jones unknot conjecture up to 22 crossings. *Journal of Knot Theory and Its Ramifications, 27*(3), Article 1840009. https://doi.org/10.1142/S0218216518400096

van Rantwijk, J. (n.d.). *Maximum weighted matching* [Computer software]. http://jorisvr.nl/article/maximum-matching

Wang, W., Jüttler, B., Zheng, D., & Liu, Y. (2008). Computation of rotation minimizing frames. *ACM Transactions on Graphics, 27*(1), Article 2. https://doi.org/10.1145/1330511.1330513

Zhang, T. Y., & Suen, C. Y. (1984). A fast parallel algorithm for thinning digital patterns. *Communications of the ACM, 27*(3), 236–239. https://doi.org/10.1145/357994.358023
