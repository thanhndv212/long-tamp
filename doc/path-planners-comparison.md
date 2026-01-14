# HPP-core planners: DiffusingPlanner vs BiRRT* vs kPRM*

This note explains and compares three planners implemented in `hpp-core`:

- DiffusingPlanner (RRT-style, “diffuse” all connected components each step)
- BiRRT* (bidirectional RRT* with a post-connection improvement mode)
- kPRM* (PRM* variant connecting each node to k-nearest neighbors)

The descriptions below are based on the current `hpp-core` implementations.

## Common concepts in hpp-core

All three planners operate on a `Roadmap`:

- **Node**: a valid robot configuration.
- **Edge**: a directed edge with an associated `Path`.
- **Steering method**: builds a candidate path between two configurations.
- **Constraints**: a `ConstraintSet` that can project / validate configurations.
- **Path projector**: optional post-processing that projects a path onto constraints.
- **Path validation**: checks collision/feasibility and can return the longest valid prefix.

In these implementations, planners typically:

1. sample a configuration `q_rand`,
2. create a candidate path via the steering method,
3. (optionally) project the path,
4. validate it, keeping either the full path or a valid prefix,
5. add nodes/edges into the roadmap.

## 1) DiffusingPlanner (generic RRT, multi-component extension)

Implementation:

- Source: `hpp-core/src/diffusing-planner.cc`
- Header: `hpp-core/include/hpp/core/diffusing-planner.hh`

### What it is

`DiffusingPlanner` is an RRT-like planner that, at each iteration, extends **each connected component** of the roadmap toward a single random sample `q_rand`.

This differs from “classic single-tree RRT”, which extends only one tree (usually the start tree) per iteration.

### Preconditions / target type

In `startSolve()`, it checks that the problem target is `GoalConfigurations` (goals are explicit goal configurations). If the target is not of this type, it throws.

### Parameters

Declared in the file:

- `DiffusingPlanner/extensionStepLength` (float)
  - If positive, limits the extension path length by extracting only the first `stepLength` portion.
- `DiffusingPlanner/extensionStepRatio` (float)
  - When the attempted path is invalid and a valid prefix exists, keep only a fraction of the valid prefix.
  - Expected in `(0, 1)`; ignored if negative.

### One iteration (`oneStep`) — code-level flow

Each `oneStep()` does **two stages**.

#### Stage A — extend each connected component toward the same `q_rand`

1. Sample a random configuration `q_rand` with `configurationShooter_`.
2. For each connected component `cc` in the roadmap:
   - Find the nearest node `near = nearestNode(q_rand, cc)`.
   - Compute a candidate extension path with `extend(near, q_rand)`.
     - `extend()` may:
       - project `q_rand` onto the constraint manifold (via `ConfigProjector::projectOnKernel` followed by `constraints->apply`),
       - call the steering method from `near` to the projected target,
       - truncate by `extensionStepLength`,
       - apply the global `PathProjector` if present.
   - Validate the returned path with `PathValidation::validate`, retrieving `validPath` (valid prefix).
   - If a non-empty valid segment exists:
     - If the path was invalid and `extensionStepRatio` is in `(0, 1)`, shrink `validPath` to `ratio * length(validPath)`.
     - Let `q_new = validPath->end()`.
     - Add `q_new` as a new node with edges from `near` using `addNodeAndEdges(near, q_new, validPath)`.
     - If another component already created the exact same `q_new` earlier in this iteration, edge insertion is delayed and done after the loop to avoid mutating connected components while iterating.

Outcome: potentially 0–N new nodes are created (often ~one per connected component), all “pointing” toward the same random sample.

#### Stage B — try to connect new nodes (and bridge components)

After stage A, it tries to create additional connections:

1. For each pair of newly added nodes `(n1, n2)`:
   - Try steering `q1 -> q2`, optional path projection, validate.
   - If fully valid: add bidirectional edges.
   - Else if a non-empty valid prefix exists: add an intermediate node reached by that prefix from `n1`.
2. For each new node `n1`, try connecting it to the list of “nearest nodes per component” collected in stage A (`nearestNeighbors`):
   - Only attempt if they are in different connected components.
   - Same pattern: steer, optional project, validate; add edges if valid; or add an intermediate node from the valid prefix.

This “connect new nodes across components” step is one reason this planner can sometimes merge connected components relatively aggressively.

### Pros

- **Fast feasibility in many settings**: classic RRT behavior can find a first path quickly.
- **Component-aware exploration**: extending each connected component can help when you have multiple goal configurations in separate components, or when the roadmap already contains multiple components.
- **Simple tuning**: only two planner-specific parameters (`extensionStepLength`, `extensionStepRatio`).

### Cons

- **Not asymptotically optimal**: no rewiring / best-parent optimization; path quality can plateau.
- **Potentially heavy per-iteration cost**: each iteration loops over all connected components and then tries connections among new nodes.
- **Quality depends strongly on steering + validation**: if validation often truncates, the planner may create many short edges and many nodes.

## 2) BiRRT* (bidirectional RRT* + improvement)

Implementation:

- Source: `hpp-core/src/path-planner/bi-rrt-star.cc`

### What it is

`BiRrtStar` grows two RRT* trees (start-side and goal-side) until they connect, then continues by sampling and rewiring to reduce path cost.

It maintains a *roadmap graph* plus two “best-parent” maps (`toRoot_[0]`, `toRoot_[1]`) that define a tree structure for cost-to-come computations from either root.

### Key parameters

- `BiRRT*/maxStepLength`
- `BiRRT*/gamma`
- `BiRRT*/minimalPathLength`

It uses the standard RRT* ball radius shape:

- $r(n) = \min\left(\gamma (\log n / n)^{1/d},\; \text{maxStepLength}\right)$

### Behavior summary

- **Before connection** (2 connected components): extend one tree toward `q`, then try to connect the other tree to `q`, swapping roles each iteration.
- **After connection** (1 connected component): repeatedly add samples and attempt rewiring to reduce costs (done from both roots).

### Pros

- **Anytime behavior with improving solution quality**: keeps refining after finding an initial solution.
- **Asymptotically optimal (in the RRT* sense)** under typical assumptions.
- **Good for “need something now, improve later” workflows**.

### Cons

- **More sensitive to parameters and implementation details** than plain RRT.
- **Rewiring adds overhead** (more local paths to validate).
- Requires reliable cost computations; the parent-map reconstruction logic is subtle.

## 3) kPRM* (k-nearest PRM*)

Implementation:

- Source: `hpp-core/src/path-planner/k-prm-star.cc`

### What it is

`kPrmStar` is a batch roadmap builder:

1. sample `N` valid configurations (nodes),
2. connect each node to its `k` nearest neighbors using steering + validation,
3. connect init and goal(s) similarly.

It sets

- $k = \lfloor (2e) \log N + 0.5 \rfloor$

which is a common PRM* scaling.

### Parameter

- `kPRM*/numberOfNodes`

### Pros

- **Roadmap reuse**: excellent if you solve many queries in the same space.
- **Predictable build phase**: you choose how many nodes you want.
- **Asymptotically optimal (PRM* flavor)** with the logarithmic neighbor rule (under typical assumptions).

### Cons

- **Can do a lot of work before finding any path** (batch nature).
- **Edge validation dominates runtime** if local planning and collision checking are expensive.
- **Memory/graph size** can grow quickly.

## Comparison: when to use what

### If you want the first feasible path fast

- Prefer **DiffusingPlanner** (RRT) or **BiRRT***.
- Choose **BiRRT*** if you also want the planner to keep improving path quality.

### If you want a reusable roadmap for many queries

- Prefer **kPRM***.

### If constraints/projection are tricky

- **DiffusingPlanner** explicitly uses `ConfigProjector::projectOnKernel` when available, then applies constraints.
- **BiRRT*** also has a heuristic for difficult projections (midpoint fallback during constraint application in `extend`).
- **kPRM*** relies mainly on direct constraint application during sampling and path projection/validation during connection; it may spend many attempts in the sampling loop if valid constrained samples are hard to find.

## Practical best practices (tuning + debugging)

- **Start with feasibility, then optimize**
  - Use DiffusingPlanner or BiRRT* to confirm the problem is solvable.
  - Once feasible, consider BiRRT* (anytime improvement) or kPRM* (batch roadmap) depending on your workload.

- **Tune step sizes to the scale of free motion**
  - DiffusingPlanner: set `DiffusingPlanner/extensionStepLength` to a step that is not so long that validation fails immediately, and not so short that progress is slow.
  - BiRRT*: set `BiRRT*/maxStepLength` similarly; too large increases validation failures, too small bloats the roadmap.

- **Use `extensionStepRatio` only when needed**
  - If you see many extensions truncated by collision, `extensionStepRatio` in `(0.1, 0.5)` can reduce “grazing” behavior by taking smaller, safer steps along the valid prefix.

- **Measure what matters**
  - For all planners, log:
    - number of steering calls,
    - path projection success rate,
    - path validation success rate and typical valid-prefix length,
    - roadmap node/edge counts over time.

- **Avoid overbuilding PRM graphs**
  - With kPRM*, increase `numberOfNodes` gradually; if validation is expensive, doubling `N` can more than double runtime due to extra neighbor edges.

---

If you want, I can also add links from the Doxygen main page to this Markdown file (hpp-core/doc/main-page/package.hh) so it shows up in generated docs.
