#!/usr/bin/env python3
"""
Base class for manipulation tasks.

Provides ManipulationTask base class with common structure.
"""

from typing import List, Dict, Any, Optional, Sequence, Tuple

import re

from agimus_spacelab.planning import (
    SceneBuilder,
    ConfigGenerator,
    GraphBuilder,
    ConstraintBuilder,
    FactoryConstraintRegistry
)



class ManipulationTask:
    """
    Base class for manipulation tasks.
    
    Provides common structure for task definition, constraint creation,
    graph building, and configuration management.
    """

    def __init__(
        self,
        joint_bounds=None,
        FILE_PATHS: Optional[Dict[str, Any]] = None,
        task_name: str = "Manipulation Task",
        backend: str = "pyhpp",
    ):
        """
        Initialize manipulation task.
        
        Args:
            task_name: Descriptive name for the task
            backend: "corba" or "pyhpp" - which backend to use
        """
        self.task_name = task_name
        self.backend = backend.lower()
        self.scene_builder = SceneBuilder(
            joint_bounds=joint_bounds,
            FILE_PATHS=FILE_PATHS,
            backend=backend,
        )
        self.planner = None
        self.robot = None
        self.ps = None
        self.graph = None
        self.config_gen = None
        self.task_config = None
        self.use_factory = False
        self.pyhpp_constraints = {}

    def get_robot_names(self) -> List[str]:
        return self.task_config.ROBOT_NAMES

    def get_composite_names(self) -> List[str]:
        return self.task_config.ROBOT_NAMES

    def get_object_names(self) -> List[str]:
        return self.task_config.OBJECTS

    def get_environment_names(self) -> List[str]:
        return self.task_config.ENVIRONMENT_NAMES

    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return self.task_config.ROBOTS

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return self.task_config.OBJECTS

    def create_constraints(self) -> None:
        """Create all transformation constraints for both backends.

        In factory mode, uses FactoryConstraintRegistry to create constraints
        with factory naming conventions:
        - Grasp: "{gripper} grasps {handle}"
        - Placement: "place_{object}"
        - Complement: "{base}/complement"

        For CORBA, they're stored in the problem solver.
        For PyHPP, they're stored in self.pyhpp_constraints and passed to
        the factory constructor.
        """
        robot = self.robot
        if self.use_factory:
            print("    Registering constraints for factory mode...")
            self._create_factory_constraints(robot)
        else:
            print("    Creating constraints manually...")
            self._create_manual_constraints(robot)

        print("   ✓ Created transformation constraints")

    def _create_factory_constraints(self, robot) -> None:
        """Create constraints with factory naming.

        Uses FactoryConstraintRegistry.

        Uses constraint definitions from config but registers them with
        factory naming conventions.
        """
        cfg = self.task_config

        # Use FactoryConstraintRegistry for proper factory naming
        registry = FactoryConstraintRegistry(
            self.ps, robot=robot, backend=self.backend
        )

        # Get constraint definitions from config
        constraint_defs = cfg.get_constraint_defs()

        # Object name for placement constraints (e.g., "frame_gripper")
        obj_name = cfg.TOOL_NAME

        # For PyHPP: when the object has no contact surfaces, the
        # ConstraintGraphFactory.buildPlacement no-contacts path creates
        # LockedJoint constraints internally to parameterise the free-state
        # foliation.  Pre-registering a RelativeTransformation placement
        # constraint causes buildPlacement to bypass that path (because
        # placeAlreadyCreated becomes True), which puts the wrong constraint
        # type on the f_01 sub-edge and makes projection fail with a constant
        # residual (~0.177).  Skip placement/complement registration in that
        # case so the factory uses its own LockedJoint foliation.
        contacts_per_obj = getattr(cfg, "CONTACT_SURFACES_PER_OBJECT", None)
        env_contacts = getattr(cfg, "ENVIRONMENT_CONTACTS", None)
        has_contacts = bool(
            contacts_per_obj and any(contacts_per_obj)
        ) and bool(env_contacts)
        skip_placement = (self.backend == "pyhpp") and (not has_contacts)

        # Register all constraints with factory naming
        # Maps user names -> factory names
        self._constraint_name_map = registry.register_from_defs(
            constraint_defs, obj_name, skip_placement=skip_placement
        )

        # Store for PyHPP graph building
        self.pyhpp_constraints = registry.get_factory_constraints_arg()

    def _create_manual_constraints(self, robot) -> None:
        """Create constraints with custom naming (manual mode)."""
        cfg = self.task_config
        cb = ConstraintBuilder

        # Get constraint definitions from config
        constraint_defs = cfg.get_constraint_defs()

        # Create all constraints from definitions
        constraints = cb.create_constraints_from_defs(
            self.ps, constraint_defs, robot=robot, backend=self.backend
        )

        # Store for PyHPP graph building
        self.pyhpp_constraints = constraints

    def create_graph(self, graph_constraints: Optional[List[str]] = None):
        """Create and configure constraint graph."""
        robot = self.robot
        problem = self.ps

        # Initialize GraphBuilder
        self.graph_builder = GraphBuilder(
            self.planner, robot, problem, backend=self.backend
        )

        if self.use_factory:
            # Pass pre-registered constraints to factory (PyHPP uses them
            # directly; CORBA already has them in the problem solver)
            return self.graph_builder.create_factory_graph(
                self.task_config,
                pyhpp_constraints=self.pyhpp_constraints,
                graph_constraints=graph_constraints,
                q_init=self.q_init,
            )
        else:
            return self.graph_builder.create_manual_graph(
                self.task_config,
                pyhpp_constraints=self.pyhpp_constraints,
                graph_constraints=graph_constraints,
            )

    def setup_collision_management(self) -> None:
        """
        Configure collision checking (disable expected contacts, etc.).
        Override in subclass if needed.
        """
        pass

    def build_initial_config(self) -> List[float]:
        """Build the initial configuration. Must be implemented by subclass."""
        raise NotImplementedError(
            "Subclass must implement build_initial_config(). "
            "Return the full initial configuration vector for all robot joints "
            "and objects."
        )

    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """
        Generate all intermediate configurations.
        Override in subclass.
        """
        raise NotImplementedError(
            "Subclass must implement generate_configurations()"
        )

    def setup(
        self,
        validation_step: float = 0.01,
        projector_step: float = 0.1,
        freeze_joint_substrings: Optional[List[str]] = None,
        skip_graph: bool = False,
    ):
        """
        Complete task setup: scene, constraints, graph.

        Args:
            validation_step: Path validation discretization
            projector_step: Path projector step
            freeze_joint_substrings: Joint name patterns to lock globally.
                Creates locked joint constraints added before graph init.
            skip_graph: If True, skip graph creation and ConfigGenerator init.
                Use this when GraspSequencePlanner will build phase graphs.
                Saves time by avoiding wasteful full graph creation.
        """
        # Store for later use
        self._freeze_joint_substrings = freeze_joint_substrings

        print("=" * 70)
        print(f"{self.task_name}")
        print("=" * 70)

        # 1. Scene setup
        print("\n1. Setting up scene...")
        self.planner, self.robot, self.ps = self.scene_builder.build(
            robot_names=self.get_robot_names(),
            environment_names=self.get_environment_names(),
            composite_names=self.get_composite_names(),
            object_names=self.get_object_names(),
            validation_step=validation_step,
            projector_step=projector_step
        )
        # Get initial configuration
        self.q_init = self.build_initial_config()

        # 2. Custom collision management
        self.setup_collision_management()

        # 3. Create constraints
        print("\n2. Creating constraints...")
        self.create_constraints()

        # 4. Create locked joint constraints if requested (factory mode only)
        # In manual mode, tasks typically handle joint freezing at config
        # generation time via _freeze_unused_joints(), so we skip adding
        # locked constraints to the graph which would conflict.
        graph_constraints = None
        patterns = freeze_joint_substrings
        if patterns is None and self.use_factory:
            # Only auto-detect FREEZE_JOINT_SUBSTRINGS for factory mode
            patterns = getattr(self, "FREEZE_JOINT_SUBSTRINGS", None)

        if patterns:
            # Build a reference config to extract joint values
            q_ref = self.q_init
            if q_ref:
                constraint_names, frozen_names = (
                    ConstraintBuilder.create_locked_joint_constraints(
                        self.ps, self.robot, q_ref, patterns,
                        backend=self.backend,
                    )
                )
                if frozen_names:
                    print(
                        f"    ✓ Created locked joint constraints: "
                        f"{', '.join(sorted(frozen_names))}"
                    )
                    graph_constraints = constraint_names

        # Store for use by GraspSequencePlanner (phase graph rebuilding)
        self._graph_constraints = graph_constraints

        if skip_graph:
            # Skip graph creation for grasp sequence mode
            # GraspSequencePlanner will build minimal phase graphs
            print("\n3. Skipping graph creation (will be built by planner)")
            print("   ✓ Scene and constraints ready for phase graph building")
            # Initialize GraphBuilder without creating graph yet
            self.graph_builder = GraphBuilder(
                self.planner, self.robot, self.ps, backend=self.backend
            )
            # ConfigGenerator will be initialized after first phase graph
            self.graph = None
            self.config_gen = None
        else:
            # 5. Create graph (with global constraints added before init)
            print("\n3. Creating constraint graph...")
            self.graph = self.create_graph(graph_constraints=graph_constraints)

            # Make graph available to backend for validation/introspection
            if hasattr(self.planner, 'graph'):
                self.planner.graph = self.graph

            # 6. Initialize configuration generator
            self.config_gen = ConfigGenerator(
                self.robot,
                self.graph,
                self.planner,
                self.ps,
                backend=self.backend,
            )

        print("\n   ✓ Task setup complete")

    def _build_reference_config_for_locking(self) -> Optional[List[float]]:
        """Build a reference config for locked joint constraints.

        Uses robot's current/neutral config before config_gen is available.
        """
        robot = self.robot
        if robot is None:
            return None

        # Try to get current config
        get_current = getattr(robot, "getCurrentConfig", None)
        if callable(get_current):
            try:
                return list(get_current())
            except Exception:
                pass

        # Try neutral config
        get_neutral = getattr(robot, "neutralConfiguration", None)
        if callable(get_neutral):
            try:
                return list(get_neutral())
            except Exception:
                pass

        return None

    def run(
        self,
        visualize: bool = True,
        solve: bool = False,
        preferred_configs: List[str] = [],
        max_iterations: int = 1000,
        solve_mode: str = "manipulation-planner",
        transition_edges: Optional[Sequence[str]] = None,
        transition_waypoints: Optional[Sequence[Sequence[float]]] = None,
        generate_waypoints_via_edges: bool = False,
        record: bool = True,
        output_dir: str = "/home/dvtnguyen/devel/demos",
        video_name: Optional[str] = None,
        framerate: int = 25,
    ) -> Dict[str, Any]:
        """
        Run the complete task workflow.

        Args:
            visualize: Whether to visualize configurations
            solve: Whether to solve the planning problem
            preferred_configs: List of intermediate config keys in order
            record: If True, record video of path playback (default: True)
            output_dir: Directory for video output (default: /home/dvtnguyen/devel/demos)
            video_name: Optional custom name for video file
            framerate: Video framerate in fps (default: 25)

        Returns:
            Dictionary with configs, paths, and other results
        """
        if not self.graph:
            raise RuntimeError("Must call setup() before run()")

        solve_mode = str(solve_mode).strip().lower()
        allowed_modes = {"manipulation-planner", "transition-planner"}
        if solve_mode not in allowed_modes:
            raise ValueError(
                f"Unknown solve_mode: {solve_mode}. "
                f"Expected one of {sorted(allowed_modes)}"
            )

        # 4. Generate configurations
        print("\n4. Generating configurations...")
        q_init = self.q_init
        configs = self.generate_configurations(q_init)
        # check if PATH_OPTIMIZER is set in task_config
        if hasattr(self.task_config, "PATH_OPTIMIZER"):
            self.optimizer = self.task_config.PATH_OPTIMIZER
        else:
            self.optimizer = "RandomShortcut"
        # 5. Visualize
        if visualize:
            print("\n5. Starting visualization...")
            try:
                self.planner.visualize(configs.get("q_init", q_init))
                print("   ✓ Initial configuration displayed")
            except Exception as e:
                print(f"   ⚠ Visualization failed: {e}")

        # 6. Solve
        if solve and "q_goal" in configs:
            print("\n6. Solving planning problem...")

            def _reset_goals_if_possible() -> None:
                if self.ps is None:
                    return
                reset = getattr(self.ps, "resetGoalConfigs", None)
                if callable(reset):
                    reset()

            def _ordered_config_keys(cfgs: Dict[str, Any]) -> List[str]:
                # Always plan from q_init to q_goal.
                if "q_init" not in cfgs or "q_goal" not in cfgs:
                    return []

                # Factory mode: q_wp_<i>_<edge>
                wp = []
                for k in cfgs.keys():
                    m = re.match(r"^q_wp_(\d+)_", k)
                    if m:
                        wp.append((int(m.group(1)), k))
                if wp:
                    wp_sorted = [k for _, k in sorted(wp, key=lambda t: t[0])]
                    return ["q_init", *wp_sorted, "q_goal"]

                # Manual mode (common naming convention)
                preferred = preferred_configs
                mids = [k for k in preferred if k in cfgs]
                if mids:
                    return ["q_init", *mids, "q_goal"]

                # Fallback: any q_* keys in insertion order.
                mids = [
                    # k
                    # for k in cfgs.keys()
                    # if k.startswith("q_") and k not in ("q_init", "q_goal")
                ]
                return ["q_init", *mids, "q_goal"]

            def _parse_factory_waypoints(
                cfgs: Dict[str, Any],
            ) -> Tuple[List[str], List[List[float]]]:
                """Parse factory waypoint keys q_wp_<i>_<edgeName>.

                Returns:
                    (edges, waypoints) where len(waypoints)=len(edges)+1.
                """
                if "q_init" not in cfgs or "q_goal" not in cfgs:
                    return [], []

                items: List[Tuple[int, str, str]] = []
                for k in cfgs.keys():
                    m = re.match(r"^q_wp_(\d+)_(.+)$", k)
                    if m:
                        items.append((int(m.group(1)), k, m.group(2)))

                if not items:
                    return [], []

                items_sorted = sorted(items, key=lambda t: t[0])
                edges = [edge_name for _, _, edge_name in items_sorted]
                waypoints = [
                    list(cfgs["q_init"]),
                    *[list(cfgs[k]) for _, k, _ in items_sorted],
                    list(cfgs["q_goal"]),
                ]
                return edges, waypoints

            def _compute_transition_inputs(
                cfgs: Dict[str, Any],
            ) -> Tuple[List[str], List[List[float]]]:
                """Compute (edges, waypoints) for transition-planner mode."""
                # 1) Explicit waypoints take precedence.
                if transition_waypoints is not None:
                    if not transition_edges:
                        raise ValueError(
                            "transition_waypoints requires transition_edges"
                        )
                    edges = [str(e) for e in transition_edges]
                    waypoints = [list(w) for w in transition_waypoints]
                    if len(waypoints) != len(edges) + 1:
                        raise ValueError(
                            "Expected len(transition_waypoints) == "
                            "len(transition_edges) + 1"
                        )
                    return edges, waypoints

                # 2) Parse factory waypoint naming convention.
                edges, waypoints = _parse_factory_waypoints(cfgs)
                if edges and waypoints:
                    return edges, waypoints

                # 3) Optional generation via edges.
                if transition_edges:
                    edges = [str(e) for e in transition_edges]
                    if not generate_waypoints_via_edges:
                        raise ValueError(
                            (
                                "transition_edges provided but no waypoints "
                                "found. Either pass transition_waypoints, "
                                "provide configs named q_wp_<i>_<edge>, or "
                                "set generate_waypoints_via_edges=True."
                            )
                        )
                    if self.config_gen is None:
                        raise RuntimeError(
                            (
                                "ConfigGenerator not initialized; "
                                "call setup() first"
                            )
                        )
                    if "q_init" not in cfgs or "q_goal" not in cfgs:
                        raise ValueError("Missing q_init/q_goal in configs")

                    q_current = list(cfgs["q_init"])
                    waypoints = [q_current]
                    # Generate intermediate waypoints for all but last edge.
                    for i, edge_name in enumerate(edges[:-1]):
                        label = f"q_wp_{i}_{edge_name}"
                        ok, q_next = self.config_gen.generate_via_edge(
                            edge_name,
                            q_current,
                            config_label=label,
                        )
                        if not ok or q_next is None:
                            raise RuntimeError(
                                (
                                    "Failed to generate waypoint via edge "
                                    f"'{edge_name}'"
                                )
                            )
                        q_current = list(q_next)
                        waypoints.append(q_current)

                    # Use task-provided goal as final waypoint.
                    waypoints.append(list(cfgs["q_goal"]))
                    return edges, waypoints

                raise ValueError(
                    "transition-planner mode requires explicit inputs. "
                    "Provide (transition_edges, transition_waypoints), or add "
                    "q_wp_<i>_<edge> configs, or pass transition_edges with "
                    "generate_waypoints_via_edges=True."
                )

            seq = _ordered_config_keys(configs)

            if not seq or len(seq) < 2:
                print("   ⚠ Planning skipped: missing q_init/q_goal")
            elif len(seq) == 2:
                _reset_goals_if_possible()
                self.planner.set_initial_config(configs["q_init"])
                self.planner.add_goal_config(configs["q_goal"])
                success = self.planner.solve(
                    max_iterations=max_iterations,
                    optimizer=self.optimizer,
                )
                if success:
                    print("   ✓ Planning successful")
                else:
                    print("   ⚠ Planning failed")

                if success and visualize:
                    print("\n7. Playing solution path...")
                    try:
                        if record and hasattr(self.planner, "play_and_record_path"):
                            video_file = self.planner.play_and_record_path(
                                path_index=0,
                                video_name=video_name,
                                output_dir=output_dir,
                                framerate=framerate,
                            )
                            print(f"   ✓ Path playback complete")
                            print(f"   📹 Video recorded: {video_file}")
                        else:
                            self.planner.play_path(0)
                            print("   ✓ Path playback complete")
                    except Exception as e:
                        print(f"   ⚠ Path playback failed: {e}")
            else:
                if solve_mode == "transition-planner":
                    edges, waypoints = _compute_transition_inputs(configs)

                    # Apply transition-planner optimizer config (optional)
                    global_opts = getattr(
                        self.task_config, "TRANSITION_OPTIMIZERS", None
                    )
                    per_edge = getattr(
                        self.task_config, "TRANSITION_OPTIMIZERS_BY_EDGE", None
                    )
                    if isinstance(per_edge, dict):
                        per_edge_opts = per_edge
                    else:
                        per_edge_opts = {}

                    for e in edges:
                        opts = per_edge_opts.get(e, global_opts)
                        if opts:
                            try:
                                self.planner.set_transition_optimizers(
                                    e, list(opts), clear_existing=True
                                )
                            except Exception as exc:
                                raise RuntimeError(
                                    (
                                        "Failed to set transition optimizers "
                                        f"for '{e}': {exc}"
                                    )
                                )

                    try:
                        path_id = self.planner.plan_transition_sequence(
                            edges,
                            waypoints,
                            validate=True,
                            reset_roadmap=True,
                            time_parameterize=True,
                            store=True,
                        )
                    except Exception as exc:
                        raise RuntimeError(
                            f"transition-planner planning failed: {exc}"
                        )

                    if visualize:
                        print("\n7. Playing solution path...")
                        try:
                            if record and hasattr(self.planner, "play_and_record_path"):
                                video_file = self.planner.play_and_record_path(
                                    path_index=int(path_id),
                                    video_name=video_name,
                                    output_dir=output_dir,
                                    framerate=framerate,
                                )
                                print(f"   ✓ Path playback complete")
                                print(f"   📹 Video recorded: {video_file}")
                            else:
                                self.planner.play_path(int(path_id))
                                print("   ✓ Path playback complete")
                        except Exception as e:
                            print(f"   ⚠ Path playback failed: {e}")

                    return {
                        "configs": configs,
                        "planner": self.planner,
                        "robot": self.robot,
                        "ps": self.ps,
                        "graph": self.graph,
                        "viewer": (
                            self.planner.viewer if self.planner else None
                        ),
                        "path_id": int(path_id),
                        "solve_mode": solve_mode,
                    }

                path_ids: List[int] = []
                for i in range(len(seq) - 1):
                    a, b = seq[i], seq[i + 1]
                    seg = f"{i + 1}/{len(seq) - 1}"
                    print(f"\n   Segment {seg}: {a} -> {b}")
                    _reset_goals_if_possible()
                    self.planner.set_initial_config(configs[a])
                    self.planner.add_goal_config(configs[b])
                    success = self.planner.solve(
                        max_iterations=max_iterations,
                        optimizer=self.optimizer,
                    )
                    if success:
                        print("   ✓ Planning successful")
                    else:
                        print("   ⚠ Planning failed")
                        break
                    # Record the latest path id when available (CORBA).
                    if self.ps is not None:
                        num_paths = getattr(self.ps, "numberPaths", None)
                        if callable(num_paths):
                            try:
                                path_ids.append(int(num_paths()) - 1)
                            except Exception:
                                pass

                # Concatenate path segments when available (CORBA).
                if len(path_ids) > 1 and self.ps is not None:
                    concat = getattr(self.ps, "concatenatePath", None)
                    if callable(concat):
                        try:
                            for j in range(1, len(path_ids)):
                                concat(path_ids[0], path_ids[j])
                        except Exception:
                            pass

                if visualize:
                    print("\n7. Playing solution path...")
                    try:
                        # Use concatenated path when known, otherwise 0.
                        pid = path_ids[0] if path_ids else 0
                        if record and hasattr(self.planner, "play_and_record_path"):
                            video_file = self.planner.play_and_record_path(
                                path_index=pid,
                                video_name=video_name,
                                output_dir=output_dir,
                                framerate=framerate,
                            )
                            print(f"   ✓ Path playback complete")
                            print(f"   📹 Video recorded: {video_file}")
                        else:
                            self.planner.play_path(pid)
                            print("   ✓ Path playback complete")
                    except Exception as e:
                        print(f"   ⚠ Path playback failed: {e}")

        return {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
            "viewer": self.planner.viewer if self.planner else None
        }


__all__ = ["ManipulationTask"]
