# References

What the packer takes from the literature, and what it does not.

## Primary sources

**Crainic, T. G., Perboli, G., & Tadei, R. (2008). "Extreme Point-Based Heuristics
for Three-Dimensional Bin Packing." *INFORMS Journal on Computing* 20(3), 368-384.**
Working paper: CIRRELT-2007-41, <https://www.cirrelt.ca/documentstravail/cirrelt-2007-41.pdf>

Used for:
- **The Extreme Point placement rule** (Section 3). When an item with sizes
  `(w,d,h)` is placed at `(x,y,z)`, it generates new candidate points by
  projecting `(x+w, y, z)` along Y and Z, `(x, y+d, z)` along X and Z, and
  `(x, y, z+h)` along X and Y. Each point slides until it meets the nearest
  item or the container wall. Implemented in `PalletPacker._spawn_eps` and
  `_project`.
- **The Residual Space merit function** (Section 4.3): place the item on the EP
  minimising `(RSx - w) + (RSy - d) + (RSz - h)`, where RS is the free distance
  along each axis from the EP to the nearest item or wall. Implemented in
  `_residual_space` and used as the base score in `try_place`.
- **Item ordering rules** (Section 4.1): Volume-Height, Height-Volume,
  Area-Height, Height-Area, and the Clustered variants that bucket base area or
  height into delta-percent bands before sorting. Implemented in
  `ordering_rules`; these seed the search.

Not used: the multi-bin aspect (a pallet is a single open-topped bin, not a bin
packing problem), and the Corner Point comparison baseline.

**Ramos, A. G., Oliveira, J. F., & Lopes, M. P. (2016). "A physical packing
sequence algorithm for the container loading problem with static mechanical
equilibrium conditions." *International Transactions in Operational Research*
23(1-2), 215-238.**
<https://onlinelibrary.wiley.com/doi/abs/10.1111/itor.12124>

This is the primary source for **the stability criterion**, and it is the paper
the robotics literature cites for it: an item is statically stable when its
centre of gravity lies within its support polygon, the minimal convex hull of
all contact points. That replaced the old `SUPPORT_THRESHOLD = 0.75` area test.

Paywalled; the criterion was verified against two open-access sources that
state and use it (below), not read in the original.

Sibling paper, same criterion embedded in a biased random-key genetic algorithm
over maximal-spaces: **Ramos, Oliveira, Goncalves & Lopes (2016), "A container
loading algorithm with static mechanical equilibrium stability constraints,"
*Transportation Research Part B* 91, 565-581**,
<https://www.sciencedirect.com/science/article/abs/pii/S0191261515302022>.
Also paywalled, also not read.

What the real method does that this implementation does not: it solves a system
of linear inequalities for the contact forces across the whole cargo assembly
and finds admissible force values by quadratic programming, requiring every
normal force to be non-negative; if no such solution exists the packing is
unstable. For a *single* rigid body that test is exactly equivalent to the
closed-form condition implemented here (with non-negative forces summing to the
weight, the achievable centres of pressure are precisely the convex hull of the
contact points), so `_stable` is correct per box. It is weaker than the paper
for the assembly as a whole. The method also covers horizontal stability,
sliding when friction is exceeded and tipping about a bottom edge, which is not
modelled here at all.

**Nascimento, Queiroz & Junqueira. "Comparing a static equilibrium based method
with the support factor for horizontal cargo stability in the container loading
problem." *Pesquisa Operacional*.**
<https://www.scielo.br/j/pope/a/3Fcz83HRMVSZP7f6J6SNZLp/?lang=en> (open access)

Read for the equations, since the Ramos papers are paywalled. Source for the
force and torque balance formulation above, and for the finding that the
equilibrium method tolerates average support well under 70%, which is the
quantitative version of the argument for dropping the 75% area threshold.

**Gao, Z., Wang, L., Kong, Y., & Chong, N. Y. (2025). "Online 3D Bin Packing
with Fast Stability Validation and Stable Rearrangement Planning."**
<https://arxiv.org/pdf/2507.09123> (open access)

Read in full. Source for the **Load Bearable Convex Polygon (LBCP)**
refinement in `_support_polygon`. Their observation: taking the support polygon
from raw geometric contact assumes every hull vertex can produce arbitrary
support force, which breaks once a stack is three or more layers deep, because
a box can rest on a region of the box below that is itself unsupported. The fix
is to intersect against each supporting box's load-bearable polygon rather than
its whole top face, recursively: a box on the deck can bear load over its
entire top face, a partially supported box only over its own support polygon.

This caught a genuine bug. Before the change, a box placed on the unsupported
overhang of a box below it was accepted as stable; it is now rejected. The
constraint costs a little on the (arbitrary) `stability_score` metric,
0.9099 vs 0.9119 mean over 10 seeds, but produces 1.4" lower stacks and is
physically correct.

Also useful for its ordering of the criteria: support area is necessary but not
sufficient; centre-of-mass-in-support-polygon is stricter but still incomplete;
full force and torque equilibrium is necessary and sufficient. This packer sits
at the middle level, plus LBCP.

**Bischoff, E. E. (2006). "Three-dimensional packing of items with limited load
bearing strength." *European Journal of Operational Research* 168(3), 952-966.**
<https://www.sciencedirect.com/science/article/abs/pii/S0377221704003170>

Used for **load-bearing limits**. Each carton has a rated top load
(`max_load`, from a lbs-per-square-inch rating times its footprint). A box's
weight is split among its supporters in proportion to contact area and passed
recursively down the support chain; a placement that would exceed any carton's
rating is rejected. Implemented in `_load_delta`, `_propagate` and `_load_ok`.

Not used: the paper's opportunity-cost formulation for choosing *which* item to
place given bearing strength. Here bearing strength is a hard feasibility
constraint only, which the paper explicitly notes is the weaker of the two
approaches.

**Parreno, F., Alvarez-Valdes, R., Oliveira, J. F., & Tamarit, J. M. (2008).
"A Maximal-Space Algorithm for the Container Loading Problem." *INFORMS Journal
on Computing* 20(3), 412-422.** Tech report: <https://www.uv.es/sestio/TechRep/tr03-07.pdf>
And the follow-up: **"A hybrid GRASP/VND algorithm for two- and
three-dimensional bin packing" (2010), *Annals of OR* 179, 203-220.**
<https://link.springer.com/article/10.1007/s10479-008-0449-4>

Used for **the search strategy**: GRASP construction with a restricted candidate
list instead of a purely greedy pick, followed by destroy-and-rebuild
improvement. Their computational study found removing 10% or 30% of blocks
significantly worse than larger fractions and settled on 50% as the cheapest of
the statistically indistinguishable good values; `rebuild_ratio` defaults to
0.5 for that reason. Implemented in `ep_packer.search`.

Not used: **maximal-spaces** themselves. The paper's free-space representation
(a non-disjoint list of largest empty cuboids, split and pruned on each
placement) is an alternative to Extreme Points, not a complement. Extreme
Points were chosen because the update rule is O(n) per placement and much
simpler to get right. Their block-building step (packing columns and layers of
identical boxes) was also skipped, and would likely help now that boxes come
from a small carton catalog with many repeats.

## Background consulted but not implemented

- Martello, S., Pisinger, D., & Vigo, D. (2000). "The Three-Dimensional Bin
  Packing Problem." *Operations Research* 48(2), 256-267. Exact approach.
- Bischoff, E. E., & Ratcliff, M. S. W. (1995). "Issues in the development of
  approaches to container loading." *Omega* 23(4), 377-390.
  <https://www.sciencedirect.com/science/article/abs/pii/030504839500015G>
  Argues that most algorithms ignore weight and load-bearing entirely.
- Davies, A. P., & Bischoff, E. E. (1999). "Weight distribution considerations
  in container loading." *EJOR* 114(3), 509-527.
  <https://www.sciencedirect.com/science/article/abs/pii/S0377221798001398>
  Relevant to the balance term in `stability_score`.
- Ramos, A. G., Oliveira, J. F., & Lopes, M. P. "Cargo Stability in the
  Container Loading Problem - State-of-the-Art and Future Research Directions."
  <https://link.springer.com/chapter/10.1007/978-3-319-71583-4_23> Survey.

## Wanted but not obtained

No open copy found for Bischoff (2006) or either Ramos et al. (2016) paper.
Searched publisher sites, institutional repositories (IPP, U. Porto, INESC TEC)
and aggregators; only abstracts are public. The load-bearing implementation and
the attribution of the support-polygon criterion therefore rest on secondary
sources, cited above, rather than the originals.

## Not grounded in any source

The `W_*` weights in the height-map packer, the `EP_WEIGHT_BIAS` term that
pushes heavy cargo toward the deck, and the `stability_score` function itself
are all local inventions, tuned empirically against the benchmark in this repo.
`stability_score` is a convenience objective for comparing runs, not a
validated measure of real-world pallet stability.
