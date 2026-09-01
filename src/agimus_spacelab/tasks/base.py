#!/usr/bin/env python3
"""
Base class for manipulation tasks.

Provides ManipulationTask base class with common structure.
"""

import logging
import re
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Sequence, Tuple

from agimus_spacelab.logging import get_logger
from agimus_spacelab.planning import (
    ConfigGenerator,
    ConstraintBuilder,
    FactoryConstraintRegistry,
    GraphBuilder,
    SceneBuilder,
)

logger = get_logger("tasks.base")


class ManipulationTask(ABC):
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
        viewer_type: str = "auto",
        log_dir: Optional[str] = "auto",
        log_level: str = "INFO",
    ):
        """
        Initialize manipulation task.

        Args:
            task_name: Descriptive name for the task
            backend: "pyhpp" - which backend to use
            viewer_type: Viewer to use — "viser", "gepetto", or "auto" (default).
                ``"auto"`` prefers viser, falling back gracefully when
                unavailable.
            log_dir: Directory for run logs. "auto" (default) creates
                /tmp/agimus_spacelab/<task_slug>_<YYYYMMDD_HHMMSS>/;
                None disables logging entirely.
            log_level: Console log verbosity ("DEBUG"/"INFO"/"WARNING"/
                "ERROR", default "INFO"). The log file (when log_dir is
                set) always captures full DEBUG detail regardless of this.
        """
        self.task_name = task_name
        self.backend = backend.lower()
        self.viewer_type = viewer_type
        self.scene_builder = SceneBuilder(
            joint_bounds=joint_bounds,
            FILE_PATHS=FILE_PATHS,
            backend=backend,
            viewer_type=viewer_type,
        )
        self.planner = None
        self.robot = None
        self.ps = None
        self.graph = None
        self.graph_builder = None
        self.config_gen = None
        self.task_config = None
        self.use_factory = False
        self.pyhpp_constraints = {}

        # Structured run logger (optional — only created when log_dir is set)
        if log_dir == "auto":
            import datetime
            import os

            slug = re.sub(r"[^a-zA-Z0-9]+", "_", task_name).strip("_").lower()
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.join("/tmp", "agimus_spacelab", f"{slug}_{stamp}")

        # Console (and, when log_dir is set, file-mirrored) output for the
        # `logging`-based progress messages modules emit via
        # agimus_spacelab.logging.get_logger(__name__) -- e.g.
        # planning/config.py's ConfigGenerator. Idempotent (configure_logging
        # skips re-attaching handlers), and independent of RunLogger below:
        # this is human-readable terminal/text output, RunLogger is the
        # separate structured JSONL event stream.
        from agimus_spacelab.logging import configure_logging

        _console_level = getattr(logging, str(log_level).upper(), logging.INFO)
        configure_logging(log_dir=log_dir, console_level=_console_level)

        self.run_logger = None
        if log_dir is not None:
            try:
                import socket
                import sys

                from agimus_spacelab.logging import RunLogger

                self.run_logger = RunLogger(log_dir)
                self.run_logger.log(
                    "run_start",
                    task_name=task_name,
                    backend=backend,
                    hostname=socket.gethostname(),
                    python_version=sys.version.split()[0],
                )
            except Exception:
                self.run_logger = None  # Never crash on logger init

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

        They're stored in self.pyhpp_constraints and pushed to
        graph_builder via set_pyhpp_constraints().
        """
        robot = self.robot
        if self.use_factory:
            logger.info("Registering constraints for factory mode...")
            self._create_factory_constraints(robot)
        else:
            logger.info("Creating constraints manually...")
            self._create_manual_constraints(robot)

        logger.info("✓ Created transformation constraints")

    def _create_factory_constraints(self, robot) -> None:
        """Create constraints with factory naming.

        Uses FactoryConstraintRegistry.

        Uses constraint definitions from config but registers them with
        factory naming conventions.
        """
        cfg = self.task_config

        # Use FactoryConstraintRegistry for proper factory naming
        registry = FactoryConstraintRegistry(
            self.ps,
            robot=robot,
            backend=self.backend,
            backend_obj=self.planner,
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
        has_contacts = bool(contacts_per_obj and any(contacts_per_obj)) and bool(
            env_contacts
        )
        skip_placement = self.planner.skip_placement_for_no_contacts and (
            not has_contacts
        )

        # Register all constraints with factory naming
        # Maps user names -> factory names
        self._constraint_name_map = registry.register_from_defs(
            constraint_defs, obj_name, skip_placement=skip_placement
        )

        # Store for PyHPP graph building
        self.pyhpp_constraints = registry.get_factory_constraints_arg()
        # Push into graph_builder if it already exists (skip_graph path)
        if self.graph_builder is not None:
            self.graph_builder.set_pyhpp_constraints(self.pyhpp_constraints)

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
        # Push into graph_builder if it already exists (skip_graph path)
        if self.graph_builder is not None:
            self.graph_builder.set_pyhpp_constraints(self.pyhpp_constraints)

    def create_graph(self, graph_constraints: Optional[List[str]] = None):
        """Create and configure constraint graph."""
        robot = self.robot
        problem = self.ps

        # Initialize GraphBuilder
        self.graph_builder = GraphBuilder(
            self.planner, robot, problem, backend=self.backend
        )
        # Seed graph builder with any previously registered PyHPP constraints
        self.graph_builder.set_pyhpp_constraints(self.pyhpp_constraints)

        if self.use_factory:
            # Pass pre-registered constraints to factory
            return self.graph_builder.create_factory_graph(
                self.task_config,
                graph_constraints=graph_constraints,
                q_init=self.q_init,
            )
        else:
            return self.graph_builder.create_manual_graph(
                self.task_config,
                graph_constraints=graph_constraints,
            )

    def setup_collision_management(self) -> None:
        """
        Configure collision checking (disable expected contacts, etc.).
        Override in subclass if needed.
        """
        pass

    @abstractmethod
    def build_initial_config(self) -> List[float]:
        """Build the initial configuration. Must be implemented by subclass."""
        raise NotImplementedError(
            "Subclass must implement build_initial_config(). "
            "Return the full initial configuration vector for all robot joints "
            "and objects."
        )

    def generate_configurations(self, q_init: List[float]) -> Dict[str, List[float]]:
        """
        Generate all intermediate configurations.
        Only required when using the base run() pipeline.
        Override in subclass; raises NotImplementedError at call time if not.
        """
        raise NotImplementedError("Subclass must implement generate_configurations()")

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

        # Emit config snapshot so the full task configuration is captured
        # before any scene loading (crash-safe: even if setup fails later,
        # the snapshot has been flushed to the JSONL file).
        self._log_setup_snapshot(
            {
                "validation_step": validation_step,
                "projector_step": projector_step,
                "freeze_joint_substrings": freeze_joint_substrings,
                "skip_graph": skip_graph,
            }
        )

        # 1. Scene setup
        logger.info("1. Setting up scene...")
        self.planner, self.robot, self.ps = self.scene_builder.build(
            robot_names=self.get_robot_names(),
            environment_names=self.get_environment_names(),
            composite_names=self.get_composite_names(),
            object_names=self.get_object_names(),
            validation_step=validation_step,
            projector_step=projector_step,
        )
        # Get initial configuration
        self.q_init = self.build_initial_config()

        # Apply optimizer config from task_config (overrides backend defaults).
        # This wires YAML optimization: fields set via yaml_loader into the backend.
        self._apply_optimizer_config()

        # Apply time parameterization method config (stp / trapezoidal / toppra)
        self._apply_time_parameterization_config()

        # 2. Custom collision management
        self.setup_collision_management()

        # 3. Create constraints
        logger.info("2. Creating constraints...")
        self.create_constraints()

        # 4. Create locked joint constraints if requested (factory mode only)
        # In manual mode, tasks typically handle joint freezing at config
        # generation time via _freeze_unused_joints(), so we skip adding
        # locked constraints to the graph which would conflict.
        graph_constraints = self._setup_locked_joint_constraints(
            freeze_joint_substrings
        )

        # Store for use by GraspSequencePlanner (phase graph rebuilding)
        self._graph_constraints = graph_constraints

        self._finalize_graph_setup(graph_constraints, skip_graph)

        logger.info("✓ Task setup complete")

    def _log_setup_snapshot(self, setup_params: dict) -> None:
        """Emit the task-config snapshot to the run logger.

        Crash-safe: even if setup fails later, the snapshot has been flushed
        to the JSONL file. No-op when no run logger is configured; any
        logging error is swallowed so a logger failure can't break setup.

        Args:
            setup_params: dict of setup parameters to record alongside the
                task config (validation_step, projector_step,
                freeze_joint_substrings, skip_graph).
        """
        if self.run_logger is not None:
            try:
                self.run_logger.log_task_config(
                    task_config=self.task_config,
                    setup_params=setup_params,
                    backend=self.backend,
                    task_name=self.task_name,
                )
            except Exception:
                pass

    def _apply_optimizer_config(self) -> None:
        """Apply optimizer config from task_config (overrides backend defaults).

        Wires YAML optimization fields (set via yaml_loader) into the backend's
        transition planner. Only fields present on task_config are forwarded;
        no call is made if none are set or the planner lacks the method.
        """
        if self.task_config is not None and hasattr(
            self.planner, "configure_transition_planner"
        ):
            opt_kwargs = {}
            for field, kwarg in (
                ("RANDOM_SHORTCUT_LOOPS", "random_shortcut_loops"),
                (
                    "SPLINE_ZERO_DERIVATIVES_AT_STATE",
                    "spline_zero_derivatives_at_state",
                ),
            ):
                if hasattr(self.task_config, field):
                    opt_kwargs[kwarg] = getattr(self.task_config, field)
            if opt_kwargs:
                self.planner.configure_transition_planner(**opt_kwargs)

    def _apply_time_parameterization_config(self) -> None:
        """Apply time parameterization method config (stp / trapezoidal / toppra).

        Forwards the TOPPRA/time-parameterization fields present on task_config
        to the backend. No call is made if none are set or the planner lacks
        the method. Kept as a separate named method from _apply_optimizer_config
        (rather than generalized into a shared helper) per the refactor plan:
        two call sites is thin justification for an abstraction layer, and the
        configure method names differ per call.
        """
        if self.task_config is not None and hasattr(
            self.planner, "configure_time_parameterization_method"
        ):
            tp_kwargs = {}
            for field, kwarg in (
                ("TIME_PARAM_METHOD", "method"),
                ("TOPPRA_VELOCITY_SCALE", "toppra_velocity_scale"),
                ("TOPPRA_EFFORT_SCALE", "toppra_effort_scale"),
                ("TOPPRA_SOLVER", "toppra_solver"),
                ("TOPPRA_N", "toppra_N"),
                ("TOPPRA_INTERPOLATION", "toppra_interpolation"),
                ("TOPPRA_GRIDPOINT_METHOD", "toppra_gridpoint_method"),
                ("TOPPRA_ACTIVE_JOINTS", "toppra_active_joints"),
            ):
                if hasattr(self.task_config, field):
                    tp_kwargs[kwarg] = getattr(self.task_config, field)
            if tp_kwargs:
                self.planner.configure_time_parameterization_method(**tp_kwargs)

    def _setup_locked_joint_constraints(
        self, freeze_joint_substrings: Optional[List[str]] = None
    ) -> Optional[List[str]]:
        """Create locked joint constraints if requested (factory mode only).

        In manual mode, tasks typically handle joint freezing at config
        generation time via _freeze_unused_joints(), so we skip adding locked
        constraints to the graph which would conflict.

        When ``freeze_joint_substrings`` is None and the task is in factory
        mode, freeze patterns are read from task_config.FREEZE_JOINT_SUBSTRINGS
        (set by the YAML loader).

        Args:
            freeze_joint_substrings: Joint name patterns to lock globally.

        Returns:
            List of constraint names to add before graph init, or None when no
            locked joint constraints are created.
        """
        graph_constraints = None
        patterns = freeze_joint_substrings
        if patterns is None and self.use_factory:
            # Read freeze patterns from task_config (set by YAML loader)
            patterns = getattr(self.task_config, "FREEZE_JOINT_SUBSTRINGS", None)

        if patterns:
            # Build a reference config to extract joint values
            q_ref = self.q_init
            if q_ref:
                constraint_names, frozen_names = (
                    ConstraintBuilder.create_locked_joint_constraints(
                        self.ps,
                        self.robot,
                        q_ref,
                        patterns,
                        backend=self.backend,
                    )
                )
                if frozen_names:
                    logger.info(
                        "✓ Created locked joint constraints: %s",
                        ", ".join(sorted(frozen_names)),
                    )
                    graph_constraints = constraint_names

        return graph_constraints

    def _finalize_graph_setup(
        self, graph_constraints: Optional[List[str]], skip_graph: bool
    ) -> None:
        """Finalize graph setup: either skip (grasp-sequence mode) or build.

        When ``skip_graph`` is True, initialize a graph-less GraphBuilder so
        GraspSequencePlanner can build minimal phase graphs later. Otherwise
        create the full constraint graph and initialize the ConfigGenerator.
        """
        if skip_graph:
            # Skip graph creation for grasp sequence mode
            # GraspSequencePlanner will build minimal phase graphs
            logger.info("3. Skipping graph creation (will be built by planner)")
            logger.info("✓ Scene and constraints ready for phase graph building")
            # Initialize GraphBuilder without creating graph yet
            self.graph_builder = GraphBuilder(
                self.planner, self.robot, self.ps, backend=self.backend
            )
            # Seed with registered PyHPP constraints so phase graph builds work
            self.graph_builder.set_pyhpp_constraints(self.pyhpp_constraints)
            # ConfigGenerator will be initialized after first phase graph
            self.graph = None
            self.config_gen = None
        else:
            # 5. Create graph (with global constraints added before init)
            logger.info("3. Creating constraint graph...")
            self.graph = self.create_graph(graph_constraints=graph_constraints)

            # Make graph available to backend for validation/introspection
            if hasattr(self.planner, "graph"):
                self.planner.graph = self.graph

            # 6. Initialize configuration generator
            self.config_gen = ConfigGenerator(
                self.robot,
                self.graph,
                self.planner,
                self.ps,
                backend=self.backend,
            )

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

    @staticmethod
    def _ordered_config_keys(
        cfgs: Dict[str, Any], preferred_configs: List[str]
    ) -> List[str]:
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

    @staticmethod
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

    def _reset_goals_if_possible(self) -> None:
        if self.ps is None:
            return
        reset = getattr(self.ps, "resetGoalConfigs", None)
        if callable(reset):
            reset()

    def _compute_transition_inputs(
        self,
        cfgs: Dict[str, Any],
        transition_edges: Optional[Sequence[str]],
        transition_waypoints: Optional[Sequence[Sequence[float]]],
        generate_waypoints_via_edges: bool,
    ) -> Tuple[List[str], List[List[float]]]:
        """Compute (edges, waypoints) for transition-planner mode."""
        # 1) Explicit waypoints take precedence.
        if transition_waypoints is not None:
            if not transition_edges:
                raise ValueError("transition_waypoints requires transition_edges")
            edges = [str(e) for e in transition_edges]
            waypoints = [list(w) for w in transition_waypoints]
            if len(waypoints) != len(edges) + 1:
                raise ValueError(
                    "Expected len(transition_waypoints) == " "len(transition_edges) + 1"
                )
            return edges, waypoints

        # 2) Parse factory waypoint naming convention.
        edges, waypoints = self._parse_factory_waypoints(cfgs)
        if edges and waypoints:
            return edges, waypoints

        # 3) Optional generation via edges.
        if transition_edges:
            edges = [str(e) for e in transition_edges]
            if not generate_waypoints_via_edges:
                raise ValueError(
                    "transition_edges provided but no waypoints "
                    "found. Either pass transition_waypoints, "
                    "provide configs named q_wp_<i>_<edge>, or "
                    "set generate_waypoints_via_edges=True."
                )
            if self.config_gen is None:
                raise RuntimeError(
                    "ConfigGenerator not initialized; " "call setup() first"
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
                        "Failed to generate waypoint via edge " f"'{edge_name}'"
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

    def _play_and_record(
        self,
        path_index: int,
        record: bool,
        video_name: Optional[str],
        output_dir: Optional[str],
        framerate: int,
    ) -> None:
        logger.info("7. Playing solution path...")
        try:
            if record and hasattr(self.planner, "play_and_record_path"):
                video_file = self.planner.play_and_record_path(
                    path_index=path_index,
                    video_name=video_name,
                    output_dir=output_dir,
                    framerate=framerate,
                )
                logger.info("✓ Path playback complete")
                logger.info("📹 Video recorded: %s", video_file)
            else:
                self.planner.play_path(path_index)
                logger.info("✓ Path playback complete")
        except Exception as e:
            logger.warning("Path playback failed: %s", e)

    def _build_result(self, configs: Dict[str, Any], **extra: Any) -> Dict[str, Any]:
        result = {
            "configs": configs,
            "planner": self.planner,
            "robot": self.robot,
            "ps": self.ps,
            "graph": self.graph,
            "viewer": self.planner.viewer if self.planner else None,
        }
        result.update(extra)
        return result

    def _solve_transition_planner(
        self,
        configs: Dict[str, Any],
        transition_edges: Optional[Sequence[str]],
        transition_waypoints: Optional[Sequence[Sequence[float]]],
        generate_waypoints_via_edges: bool,
    ) -> int:
        """Solve via transition-planner: resolve edges/waypoints, apply
        per-edge/global optimizer config, plan the full transition
        sequence. Returns the resulting path id.
        """
        edges, waypoints = self._compute_transition_inputs(
            configs,
            transition_edges,
            transition_waypoints,
            generate_waypoints_via_edges,
        )

        # Apply transition-planner optimizer config (optional)
        global_opts = getattr(self.task_config, "TRANSITION_OPTIMIZERS", None)
        per_edge = getattr(self.task_config, "TRANSITION_OPTIMIZERS_BY_EDGE", None)
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
                        "Failed to set transition optimizers " f"for '{e}': {exc}"
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
            raise RuntimeError(f"transition-planner planning failed: {exc}")

        return int(path_id)

    def _solve_manipulation_planner(
        self,
        configs: Dict[str, Any],
        seq: List[str],
        max_iterations: int,
    ) -> Tuple[List[int], bool]:
        """Solve by iterating seq pairwise via the manipulation-planner.

        Returns (path_ids, success): path_ids is the per-segment path id
        list (empty when the backend doesn't expose numberPaths), success
        is the last-attempted segment's solve() result.
        Callers need both to decide whether/what to play back -- see
        docs/plans/refactor-manipulation-task-run.md Step 6a for why a
        single `success` bool isn't sufficient on its own (the len(seq)==2
        vs len(seq)>2 playback-gating difference this preserves).
        """
        path_ids: List[int] = []
        for i in range(len(seq) - 1):
            a, b = seq[i], seq[i + 1]
            seg = f"{i + 1}/{len(seq) - 1}"
            logger.info("Segment %s: %s -> %s", seg, a, b)
            self._reset_goals_if_possible()
            self.planner.set_initial_config(configs[a])
            self.planner.add_goal_config(configs[b])
            success = self.planner.solve(
                max_iterations=max_iterations,
                optimizer=self.optimizer,
            )
            if success:
                logger.info("✓ Planning successful")
            else:
                logger.warning("Planning failed")
                break
            # Record the latest path id when available.
            if self.ps is not None:
                num_paths = getattr(self.ps, "numberPaths", None)
                if callable(num_paths):
                    try:
                        path_ids.append(int(num_paths()) - 1)
                    except Exception:
                        pass

        # Concatenate path segments when available.
        if len(path_ids) > 1 and self.ps is not None:
            concat = getattr(self.ps, "concatenatePath", None)
            if callable(concat):
                try:
                    for j in range(1, len(path_ids)):
                        concat(path_ids[0], path_ids[j])
                except Exception:
                    pass

        return path_ids, success

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
        output_dir: Optional[str] = None,
        video_name: Optional[str] = None,
        framerate: int = 25,
    ) -> Dict[str, Any]:
        """
        Run the complete task workflow.

        Args:
            visualize: Whether to visualize configurations
            solve: Whether to solve the planning problem
            preferred_configs: List of intermediate config keys in order.
                Used by _ordered_config_keys() when configs aren't named
                via the q_wp_<i>_<edge> factory convention.
            max_iterations: Max RRT iterations for manipulation-planner
                mode's solve() calls.
            solve_mode: "manipulation-planner" (default) or
                "transition-planner". Note: transition-planner is only
                honored when the resolved config sequence has more than 2
                entries -- see docs/plans/refactor-manipulation-task-run.md
                "Decision needed: the dead solve_mode gate".
            transition_edges: Edge names for transition-planner mode.
                Required (with transition_waypoints or
                generate_waypoints_via_edges) to reach that mode at all.
            transition_waypoints: Explicit waypoints for transition-planner
                mode; length must be len(transition_edges) + 1.
            generate_waypoints_via_edges: If True and transition_edges is
                set but transition_waypoints isn't, generate intermediate
                waypoints via self.config_gen.generate_via_edge() for each
                edge but the last.
            record: If True, record video of path playback (default: True)
            output_dir: Directory for video output. Defaults to
                ``agimus_spacelab.visualization.default_video_output_dir()``
                when not given.
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
        logger.info("4. Generating configurations...")
        q_init = self.q_init
        configs = self.generate_configurations(q_init)
        # check if PATH_OPTIMIZER is set in task_config
        if hasattr(self.task_config, "PATH_OPTIMIZER"):
            self.optimizer = self.task_config.PATH_OPTIMIZER
        else:
            self.optimizer = "RandomShortcut"
        # 5. Visualize
        if visualize:
            logger.info("5. Starting visualization...")
            try:
                self.planner.visualize(configs.get("q_init", q_init))
                logger.info("✓ Initial configuration displayed")
            except Exception as e:
                logger.warning("Visualization failed: %s", e)

        # 6. Solve
        if solve and "q_goal" in configs:
            logger.info("6. Solving planning problem...")

            seq = self._ordered_config_keys(configs, preferred_configs)

            if not seq or len(seq) < 2:
                logger.warning("Planning skipped: missing q_init/q_goal")
            else:
                # solve_mode is only honored for len(seq) > 2 -- a
                # pre-existing (likely accidental) gate, preserved exactly
                # here rather than fixed as part of this structural
                # refactor. See docs/plans/refactor-manipulation-task-run.md
                # "Decision needed: the dead solve_mode gate".
                if solve_mode == "transition-planner" and len(seq) > 2:
                    path_id = self._solve_transition_planner(
                        configs,
                        transition_edges,
                        transition_waypoints,
                        generate_waypoints_via_edges,
                    )

                    if visualize:
                        self._play_and_record(
                            path_id, record, video_name, output_dir, framerate
                        )

                    return self._build_result(
                        configs, path_id=int(path_id), solve_mode=solve_mode
                    )

                path_ids, success = self._solve_manipulation_planner(
                    configs, seq, max_iterations
                )

                # len(seq) == 2 (the former dedicated branch) only played
                # back on success; the N-segment loop always attempted
                # playback of whatever partial progress exists. Both
                # preserved exactly via this single condition: for
                # len(seq) == 2 it reduces to `success`, for len(seq) > 2
                # it's always True.
                if visualize and (len(seq) > 2 or success):
                    # Use concatenated path when known, otherwise 0.
                    pid = path_ids[0] if path_ids else 0
                    self._play_and_record(
                        pid, record, video_name, output_dir, framerate
                    )

        return self._build_result(configs)


__all__ = ["ManipulationTask"]
