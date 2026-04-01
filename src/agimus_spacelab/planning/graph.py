#!/usr/bin/env python3
"""
Constraint graph builder for manipulation tasks.

Provides GraphBuilder for creating constraint graphs with dual backend support.
"""

from __future__ import annotations

import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Type, Union

from agimus_spacelab.config.base_config import BaseTaskConfig, ConstraintDef
from agimus_spacelab.planning.constraints import (
    ConstraintBuilder,
)

# Import for CORBA backend
try:
    from hpp.corbaserver.manipulation import (
        ConstraintGraph,
        ConstraintGraphFactory,
        Rule,
        Constraints
    )
    HAS_CORBA_GRAPH = True
except ImportError:
    HAS_CORBA_GRAPH = False
    ConstraintGraph = None
    ConstraintGraphFactory = None
    Rule = None
    Constraints = None

# Import for PyHPP backend
try:
    from pyhpp.manipulation import Graph as PyHPPGraph
    from pyhpp.manipulation.constraint_graph_factory import (
        ConstraintGraphFactory as PyHPPConstraintGraphFactory,
        Rule as PyHPPRule
    )
    HAS_PYHPP_GRAPH = True
except ImportError:
    HAS_PYHPP_GRAPH = False
    PyHPPGraph = None
    PyHPPConstraintGraphFactory = None
    PyHPPRule = None


class GraphBuilder:
    """
    Builder class for creating constraint graphs with dual backend support.
    
    Supports both manual graph construction (node by node, edge by edge) and
    factory-based automatic graph generation.
    """

    def __init__(self, planner, robot, ps, backend: str = "corba"):
        """
        Initialize graph builder.
        
        Args:
            planner: Planner instance
            robot: Robot/Device instance
            ps: ProblemSolver or Problem instance
            backend: "corba" or "pyhpp"
        """
        self.planner = planner
        self.robot = robot
        self.ps = ps
        self.backend = backend.lower()
        self.graph = None
        self.factory = None

        # Track manually created states and edges
        self.states = {}  # name -> id
        self.edges = {}   # name -> id
        self.edge_topology = {}  # name -> (from_state, to_state)

    def _attach_graph_to_problem_if_supported(self) -> None:
        """Attach the current graph to the backend problem, when required.

        In PyHPP, the `Problem` must have a constraint graph set via
        `problem.constraintGraph(graph)`. If this is not done, planning fails
        with: "No graph in the problem.".
        """

        if self.backend != "pyhpp":
            return
        if self.graph is None or self.ps is None:
            return

        attach = getattr(self.ps, "constraintGraph", None)
        if callable(attach):
            attach(self.graph)

    def initiate_graph(self, name: str = "graph") -> Any:
        """
        Initialize an empty constraint graph for manual construction.
        
        Args:
            name: Graph name
            
        Returns:
            ConstraintGraph or Graph instance
        """
        print(f"   Creating manual constraint graph: {name}")

        if self.backend == "corba":
            if not HAS_CORBA_GRAPH:
                raise ImportError("CORBA backend not available")

            self.graph = ConstraintGraph(self.robot, "graph")
            self.ps.setMaxIterProjection(100)
            self.ps.setErrorThreshold(1e-4)
            print("   ✓ CORBA graph initialized (manual mode)")

        else:  # pyhpp
            if not HAS_PYHPP_GRAPH:
                raise ImportError("PyHPP backend not available")

            self.graph = PyHPPGraph(name, self.robot, self.ps)
            self.graph.maxIterations(100)
            self.graph.errorThreshold(1e-4)
            print("   ✓ PyHPP graph initialized (manual mode)")

        return self.graph

    def add_state(self, name: str, is_waypoint: bool = False,
                  priority: int = 0) -> int:
        """
        Add a state to the graph.

        Args:
            name: State name
            is_waypoint: Whether this is a waypoint state
            priority: State priority
            
        Returns:
            State ID
        """
        if self.graph is None:
            raise RuntimeError(
                "Graph not initialized. Call initiate_graph() first"
            )

        if self.backend == "corba":
            # CORBA uses createNode
            state_id = self.graph.createNode([name], is_waypoint)
        else:  # pyhpp
            # PyHPP uses createState
            state_id = self.graph.createState(name, is_waypoint, priority)

        self.states[name] = state_id
        print(f"    ✓ State '{name}' created (ID: {state_id})")
        return state_id

    def add_states(self, names: List[str], is_waypoint: bool = False) -> None:
        """
        Add multiple states to the graph at once.

        For CORBA backend, this is more efficient than calling add_state
        multiple times as it creates all nodes in a single call.

        Args:
            names: List of state names
            is_waypoint: Whether these are waypoint states

        Note:
            For PyHPP backend, this calls add_state for each name.
        """
        if self.graph is None:
            raise RuntimeError(
                "Graph not initialized. Call initiate_graph() first"
            )

        if self.backend == "corba":
            # CORBA can create multiple nodes at once
            self.graph.createNode(names, is_waypoint)
            for name in names:
                self.states[name] = name  # CORBA uses name as ID
            print(f"    ✓ Created {len(names)} states: {names}")
        else:  # pyhpp
            # PyHPP creates states one at a time
            for name in names:
                self.add_state(name, is_waypoint)

    def add_edge(self, from_state: str, to_state: str, name: str,
                 weight: int = 1,
                 containing_state: Optional[str] = None) -> int:
        """
        Add an edge between two states.
        
        Args:
            from_state: Source state name
            to_state: Target state name
            name: Edge name
            weight: Edge weight for planning
            containing_state: State whose constraints apply during edge
            
        Returns:
            Edge ID
        """
        if self.graph is None:
            raise RuntimeError(
                "Graph not initialized. Call initiate_graph() first"
            )

        # Verify states exist
        if from_state not in self.states:
            raise ValueError(f"Source state '{from_state}' not found")
        if to_state not in self.states:
            raise ValueError(f"Target state '{to_state}' not found")

        if self.backend == "corba":
            # CORBA uses createEdge with state names
            edge_id = self.graph.createEdge(
                from_state, to_state, name, weight,
                containing_state or from_state
            )
        else:  # pyhpp
            # PyHPP uses createTransition with state objects
            from_id = self.states[from_state]
            to_id = self.states[to_state]
            containing_id = self.states[containing_state or from_state]
            edge_id = self.graph.createTransition(
                from_id, to_id, name, weight, containing_id
            )

        self.edges[name] = edge_id
        self.edge_topology[name] = (from_state, to_state)
        print(f"    ✓ Edge '{name}': {from_state} → {to_state} "
              f"(ID: {edge_id})")
        return edge_id

    def add_state_constraints(
        self,
        state_name: str,
        constraints: List[Any],
        constraint_names: Optional[List[str]] = None
    ) -> None:
        """
        Add constraints to a state.

        Args:
            state_name: Name of the state to add constraints to
            constraints: List of constraint objects (PyHPP) or ignored (CORBA)
            constraint_names: List of constraint names (CORBA) or ignored
                (PyHPP)

        Note:
            - CORBA: Uses constraint_names to reference constraints by name
            - PyHPP: Uses constraint objects directly
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")

        if state_name not in self.states:
            raise ValueError(f"State '{state_name}' not found")

        if self.backend == "corba":
            if constraint_names is None:
                raise ValueError(
                    "constraint_names required for CORBA backend"
                )
            # CORBA uses addConstraints with Constraints object
            self.graph.addConstraints(
                node=state_name,
                constraints=Constraints(numConstraints=constraint_names)
            )
            print(f"    ✓ Added constraints {constraint_names} "
                  f"to state '{state_name}'")

        else:  # pyhpp
            if constraints is None or len(constraints) == 0:
                raise ValueError(
                    "constraints list required for PyHPP backend"
                )
            # PyHPP uses addNumericalConstraint for each constraint
            state_id = self.states[state_name]
            for constraint in constraints:
                self.graph.addNumericalConstraint(state_id, constraint)
            print(f"    ✓ Added {len(constraints)} constraint(s) "
                  f"to state '{state_name}'")

    def add_edge_constraints(
        self,
        edge_name: str,
        constraints: List[Any],
        constraint_names: Optional[List[str]] = None
    ) -> None:
        """
        Add constraints to an edge (path constraints).

        Args:
            edge_name: Name of the edge to add constraints to
            constraints: List of constraint objects (PyHPP) or ignored (CORBA)
            constraint_names: List of constraint names (CORBA) or ignored
                (PyHPP)

        Note:
            - CORBA: Uses constraint_names to reference constraints by name
            - PyHPP: Uses constraint objects directly
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")

        if edge_name not in self.edges:
            raise ValueError(f"Edge '{edge_name}' not found")

        if self.backend == "corba":
            # CORBA uses addConstraints with Constraints object
            # Empty constraint_names is valid (no path constraints)
            names = constraint_names or []
            self.graph.addConstraints(
                edge=edge_name,
                constraints=Constraints(numConstraints=names)
            )
            if names:
                print(f"    ✓ Added constraints {names} to edge '{edge_name}'")
            else:
                print(f"    ✓ Added empty constraints to edge '{edge_name}'")

        else:  # pyhpp
            # PyHPP uses addNumericalConstraintsToTransition
            edge_id = self.edges[edge_name]
            if constraints and len(constraints) > 0:
                self.graph.addNumericalConstraintsToTransition(
                    edge_id, constraints
                )
                print(f"    ✓ Added {len(constraints)} constraint(s) "
                      f"to edge '{edge_name}'")
            else:
                # No constraints to add (free motion edge)
                print(f"    ✓ No constraints added to edge '{edge_name}' "
                      "(free motion)")

    def add_global_constraints(
        self,
        constraint_names: List,
    ) -> bool:
        """Add numerical constraints globally to the graph.

        Adds constraints to the entire graph (all nodes and edges).

        This is useful for:
        - Locked joint constraints (freeze joints during planning)
        - Any other constraints that should apply everywhere

        Note: Must be called BEFORE the graph is initialized.

        Args:
            constraint_names: CORBA: list of constraint name strings.
                PyHPP: list of Implicit/LockedJoint constraint objects.

        Returns:
            True if constraints were added successfully
        """
        if not constraint_names or self.graph is None:
            return False

        try:
            if self.backend == "corba":
                self.graph.addConstraints(
                    graph=True,
                    constraints=Constraints(numConstraints=constraint_names),
                )
            else:  # pyhpp
                self.graph.addNumericalConstraintsToGraph(constraint_names)
            print(f"    ✓ Added {len(constraint_names)} global constraints: ",
                  f" {constraint_names}")
            return True
        except Exception as e:
            print(f"   ⚠ Failed to add global constraints: {e}")
            return False

    def finalize_manual_graph(self) -> Any:
        """
        Finalize manual graph construction and initialize it.
        
        Returns:
            Initialized graph
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")

        if self.backend == "corba":
            # CORBA graph initialization
            self.graph.initialize()
            print("   ✓ CORBA graph initialized")
        else:  # pyhpp
            # PyHPP graph initialization.
            # IMPORTANT: attach to problem BEFORE initialize().
            # Problem::constraintGraph(graph) calls graph.problem(problem)
            # which calls Graph::invalidate(), resetting isInit_=false for
            # all components. So we must attach first, then initialize.
            # Similarly, maxIterations/errorThreshold setters call
            # invalidate() — set them now before initialize().
            self._attach_graph_to_problem_if_supported()
            self.graph.maxIterations(10000)
            self.graph.errorThreshold(1e-4)
            self.graph.initialize()
            print("   ✓ PyHPP graph initialized")

        return self.graph

    def _prepapre_factory_inputs(self, config):
        """Prepare and validate factory inputs from config.

        Args:
            grippers: List of gripper names
            objects: List of object names
            handles_per_object: List of handle lists, one per object
            contact_surfaces_per_object: List of contact surface lists per
                object (defaults to empty lists)
            environment_contacts: List of environment contact surface names
            rules: Optional list of Rule objects for graph generation
            valid_pairs: Optional dict mapping gripper names to list of
                valid handle names. Uses setPossibleGrasps to restrict
                which gripper-handle pairs are allowed.
        Returns:
            Prepared config with defaults filled in."""

        # grippers, objects, handles_per_object are required
        if not config.GRIPPERS:
            raise ValueError("At least one gripper required")
        if not config.OBJECTS:
            raise ValueError("At least one object required")
        if len(config.HANDLES_PER_OBJECT) != len(config.OBJECTS):
            raise ValueError(
                "handles_per_object length must match objects length"
            )
        if config.CONTACT_SURFACES_PER_OBJECT is not None:
            if len(config.CONTACT_SURFACES_PER_OBJECT) != len(
                config.OBJECTS
            ):
                raise ValueError(
                    "contact_surfaces_per_object length must match "
                    "objects length"
                )
        else:
            config.CONTACT_SURFACES_PER_OBJECT = [
                [] for _ in config.OBJECTS
            ]

        if config.ENVIRONMENT_CONTACTS is None:
            config.ENVIRONMENT_CONTACTS = []
        return config

    def create_factory_graph(
        self,
        config: BaseTaskConfig,
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
        graph_constraints: Optional[List[str]] = None,
    ) -> Any:
        """
        Create constraint graph using factory for both backends.

        Args:
            grippers: List of gripper names
            objects: List of object names
            handles_per_object: List of handle lists, one per object
            contact_surfaces_per_object: List of contact surface lists per
                object (defaults to empty lists)
            environment_contacts: List of environment contact surface names
            rules: Optional list of Rule objects for graph generation
            valid_pairs: Optional dict mapping gripper names to list of
                valid handle names. Uses setPossibleGrasps to restrict
                which gripper-handle pairs are allowed.
            graph_constraints: Optional list of constraint names to add
                globally to the graph before initialization (e.g., locked
                joint constraints).

        Returns:
            ConstraintGraph or Graph instance
        """
        print("    Using ConstraintGraphFactory for automatic graph "
              "generation")

        # Backend-specific setup
        if self.backend == "corba":
            if not HAS_CORBA_GRAPH:
                raise ImportError("CORBA backend not available")

            # Create CORBA constraint graph
            self.graph = ConstraintGraph(self.robot, "graph")
            self.factory = ConstraintGraphFactory(self.graph)

        else:  # pyhpp
            if not HAS_PYHPP_GRAPH:
                raise ImportError("PyHPP backend not available")

            # Create PyHPP constraint graph
            self.graph = PyHPPGraph("graph", self.robot, self.ps)
            self.factory = PyHPPConstraintGraphFactory(
                self.graph, constraints=pyhpp_constraints or {}
            )

        # Prepare valid factory inputs
        config = self._prepapre_factory_inputs(config)

        # Set grippers
        self.factory.setGrippers(config.GRIPPERS)
        print(f"    \u2713 Set grippers: {config.GRIPPERS}")

        # Set objects with handles and contact surfaces
        self.factory.setObjects(
            config.OBJECTS, config.HANDLES_PER_OBJECT,
            config.CONTACT_SURFACES_PER_OBJECT
        )
        print(f"    \u2713 Set objects: {config.OBJECTS}")

        # Set environment contacts if provided
        if config.ENVIRONMENT_CONTACTS:
            self.factory.environmentContacts(config.ENVIRONMENT_CONTACTS)
            print(f"    \u2713 Set environment contacts: "
                  f"{config.ENVIRONMENT_CONTACTS}")

        # Set grasp restrictions
        if config.RULES is not None:
            self.factory.setRules(config.RULES)
            print("    ✓ Set custom rules")
        elif config.VALID_PAIRS is not None:
            self.factory.setPossibleGrasps(config.VALID_PAIRS)
            print("    ✓ Set possible grasps from valid_pairs")

        # Apply sequential filter if provided (strict 2-state limit)
        if hasattr(config, '_SEQUENTIAL_FILTER'):
            seq_filter = config._SEQUENTIAL_FILTER
            self.factory.graspIsAllowed.append(seq_filter)
            print("    ✓ Applied SequentialGraspFilter")
            print("      Will limit graph to current→next state only")

        # Generate graph
        self.factory.generate()
        print("    ✓ Generated graph structure")

        # Add global constraints before initialization (e.g., locked joints)
        if graph_constraints:
            self.add_global_constraints(graph_constraints)

        # Initialize graph.
        # IMPORTANT for pyhpp: Problem::constraintGraph(graph) calls
        # graph.problem(problem) -> Graph::invalidate() -> isInit_=false.
        # And maxIterations/errorThreshold setters also call invalidate().
        # All three must happen BEFORE graph.initialize() for pyhpp.
        if self.backend == "pyhpp":
            self._attach_graph_to_problem_if_supported()
            self.graph.maxIterations(10000)
            self.graph.errorThreshold(1e-4)
        self.graph.initialize()
        if self.backend == "corba":
            self._attach_graph_to_problem_if_supported()
        print("    \u2713 Graph initialized")

        # Store states and edges for tracking
        self._extract_factory_graph_structure()

        # Print factory-generated nodes for reference
        print(f"    \u2139 Factory created {len(self.states)} nodes:")
        for node_name in list(self.states.keys()):
            print(f"      - {node_name}")
        print(f"    \u2139 Factory created {len(self.edges)} edges:")
        for edge_name in list(self.edges.keys()):
            from_state, to_state = self.edge_topology.get(
                edge_name, ("unknown", "unknown")
            )
            print(f"      - {edge_name}: {from_state} → {to_state}")
        return self.graph

    def create_manual_graph(
        self,
        config: BaseTaskConfig,
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
        graph_constraints: Optional[List[str]] = None,
    ) -> Any:
        """Create the manual graph using GraphBuilder (both backends).

        Args:
            config: Task configuration
            pyhpp_constraints: PyHPP constraint objects (for pyhpp backend)
            graph_constraints: Optional list of constraint names to add
                globally (e.g., locked joint constraints)
        """
        print("    Building graph manually")

        cfg = config
        graph_def = getattr(cfg, "GRASP_FG_GRAPH", None)
        if not isinstance(graph_def, dict):
            raise RuntimeError("Missing 'GRASP_FG_GRAPH' graph config")

        # PyHPP constraint objects (CORBA uses names)
        constraints = (
            pyhpp_constraints if self.backend == "pyhpp" else None
        )

        # Create empty graph
        graph_name = (
            "manipulation_graph" if self.backend == "pyhpp" else "graph"
        )
        self.initiate_graph(name=graph_name)

        # Create states (order matters for solver performance)
        state_names = getattr(cfg, "GRAPH_NODES", None) or list(
            graph_def.get("states", {}).keys()
        )
        if self.backend == "corba":
            self.add_states(state_names)
        else:
            for state_name in state_names:
                self.add_state(state_name)
        print(f"    ✓ Created {len(state_names)} states")

        # Create edges from declarative definition
        for edge_name, edge_info in graph_def.get("edges", {}).items():
            self.add_edge(
                edge_info["from"],
                edge_info["to"],
                edge_name,
                edge_info.get("weight", 1),
                edge_info["in"],
            )
        print("    ✓ Created edges (transitions)")

        # Add constraints to states from declarative definition
        states_def = graph_def.get("states", {})
        for state_name, state_info in states_def.items():
            constraint_names = state_info.get("constraints", [])
            if constraint_names:
                if self.backend == "corba":
                    self.add_state_constraints(
                        state_name, [], constraint_names=constraint_names
                    )
                else:
                    constraint_objs = [
                        constraints[n] for n in constraint_names
                    ]
                    self.add_state_constraints(state_name, constraint_objs)
        print("    ✓ Added constraints to nodes")

        # Add constraints to edges from declarative definition
        edge_constraints_def = graph_def.get("edge_constraints", {})
        for constraint_name, edge_list in edge_constraints_def.items():
            for edge_name in edge_list:
                if self.backend == "corba":
                    self.add_edge_constraints(
                        edge_name, [], constraint_names=[constraint_name]
                    )
                else:
                    self.add_edge_constraints(
                        edge_name, [constraints[constraint_name]]
                    )

        # Free motion edges (no path constraints)
        for edge_name in graph_def.get("free_motion_edges", []):
            if self.backend == "corba":
                self.add_edge_constraints(edge_name, [], constraint_names=[])
            else:
                self.add_edge_constraints(edge_name, [])
        print("    ✓ Added constraints to edges")

        # Set constant RHS (CORBA only)
        if self.backend == "corba":
            for constraint_name, is_constant in graph_def.get(
                "constant_rhs", {}
            ).items():
                self.ps.setConstantRightHandSide(constraint_name, is_constant)
            print("    ✓ Set constant right-hand side")

            # # Set security margins BEFORE initialize (CORBA only)
            # for edge_name in cfg.PLACEMENT_EDGES:
            #     self.graph.setSecurityMarginForEdge(
            #         edge_name,
            #         cfg.TOOL_CONTACT_JOINT,
            #         cfg.DISPENSER_CONTACT_JOINT,
            #         cfg.CONTACT_MARGIN,
            #     )
            # print(
            #     f"    ✓ Set security margin ({cfg.CONTACT_MARGIN}m) "
            #     "for placement edges"
            # )

        # Add global constraints before initialization (e.g., locked joints)
        if graph_constraints:
            self.add_global_constraints(graph_constraints)

        # Initialize graph
        self.finalize_manual_graph()
        print("    ✓ Graph initialized")
        return self.get_graph()

    # ---------------------------------------------------------------------
    # Task-config-driven entrypoints
    # ---------------------------------------------------------------------

    TaskConfigT = Union[Type[BaseTaskConfig], BaseTaskConfig]

    @staticmethod
    def _as_task_config(task: TaskConfigT) -> Type[BaseTaskConfig]:
        # Task configs in this repo are typically class-based (class
        # attributes). If the caller passes an instance, recover the class.
        return task if isinstance(task, type) else task.__class__

    def build_graph_for_task(
        self,
        task: TaskConfigT,
        *,
        mode: Optional[str] = None,
        name: str = "graph",
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
    ) -> Any:
        """Build a constraint graph from a task specification.

        Args:
            task: A `BaseTaskConfig` subclass (or instance).
            mode: "factory" or "manual". If None, defaults to:
                - "manual" if the task defines STATES/EDGES,
                - otherwise "factory".
            name: Graph name (manual mode only; factory mode uses "graph").
            pyhpp_constraints: Optional dict (name -> constraint object) to
                seed the PyHPP factory's available constraints.
        """

        task_cls = self._as_task_config(task)
        chosen_mode = mode
        if chosen_mode is None:
            if getattr(task_cls, "STATES", None) and getattr(
                task_cls, "EDGES", None
            ):
                chosen_mode = "manual"
            else:
                chosen_mode = "factory"

        if chosen_mode == "factory":
            contact_surfaces = list(
                getattr(task_cls, "CONTACT_SURFACES_PER_OBJECT", [])
            )
            env_contacts = list(getattr(task_cls, "ENVIRONMENT_CONTACTS", []))
            return self.create_factory_graph(
                grippers=list(getattr(task_cls, "GRIPPERS", [])),
                objects=list(getattr(task_cls, "OBJECTS", [])),
                handles_per_object=task_cls.get_handles_per_object(),
                contact_surfaces_per_object=(contact_surfaces or None),
                environment_contacts=(env_contacts or None),
                rules=None,
                valid_pairs=dict(getattr(task_cls, "VALID_PAIRS", {})) or None,
                pyhpp_constraints=pyhpp_constraints,
            )

        if chosen_mode == "manual":
            return self.build_manual_graph_from_task(task_cls, name=name)

        raise ValueError("mode must be 'factory' or 'manual'")

    def build_manual_graph_from_task(
        self,
        task: TaskConfigT,
        *,
        name: str = "graph",
    ) -> Any:
        """Build a manual graph (states/edges/constraints) from BaseTaskConfig.

        This is useful for tasks that define a custom topology (not purely
        combinatorial grasp/placement graphs).
        """

        task_cls = self._as_task_config(task)
        self.initiate_graph(name=name)

        # 1) Create constraints declared by the task.
        pyhpp_constraint_objects: Dict[str, Any] = {}
        for cdef in task_cls.get_constraint_defs():
            if not isinstance(cdef, ConstraintDef):
                raise TypeError(
                    "Task get_constraint_defs() must return ConstraintDef "
                    "items"
                )

            if cdef.type == "grasp":
                if not cdef.gripper or not cdef.obj or not cdef.transform:
                    raise ValueError(f"Invalid grasp ConstraintDef: {cdef}")
                created = ConstraintBuilder.create_grasp_constraint(
                    self.ps,
                    cdef.name,
                    cdef.gripper,
                    cdef.obj,
                    cdef.transform,
                    mask=cdef.mask,
                    robot=self.robot,
                    backend=self.backend,
                )
                if created is not None:
                    pyhpp_constraint_objects[cdef.name] = created

            elif cdef.type == "placement":
                if not cdef.obj or not cdef.transform:
                    raise ValueError(
                        f"Invalid placement ConstraintDef: {cdef}"
                    )
                created = ConstraintBuilder.create_placement_constraint(
                    self.ps,
                    cdef.name,
                    cdef.obj,
                    cdef.transform,
                    cdef.mask,
                    robot=self.robot,
                    backend=self.backend,
                )
                if created is not None:
                    pyhpp_constraint_objects[cdef.name] = created

            elif cdef.type == "complement":
                if not cdef.obj or not cdef.transform:
                    raise ValueError(
                        f"Invalid complement ConstraintDef: {cdef}"
                    )
                created = ConstraintBuilder.create_complement_constraint(
                    self.ps,
                    cdef.name,
                    cdef.obj,
                    cdef.transform,
                    cdef.mask,
                    robot=self.robot,
                    backend=self.backend,
                )
                cname = f"{cdef.name}/complement"
                if created is not None:
                    pyhpp_constraint_objects[cname] = created

            else:
                raise ValueError(f"Unknown constraint type '{cdef.type}'")

        # 2) Create states.
        for state in task_cls.STATES.values():
            self.add_state(
                state.name,
                is_waypoint=state.is_waypoint,
                priority=state.priority,
            )

        # 3) Create edges.
        for edge in task_cls.EDGES.values():
            self.add_edge(
                edge.from_state,
                edge.to_state,
                edge.name,
                weight=int(edge.weight),
                containing_state=edge.containing_state,
            )

        # 4) Attach constraints to states and edges.
        for state in task_cls.STATES.values():
            if state.constraints:
                if self.backend == "corba":
                    self.add_state_constraints(
                        state.name,
                        constraints=[],
                        constraint_names=state.constraints,
                    )
                else:
                    missing = [
                        n
                        for n in state.constraints
                        if n not in pyhpp_constraint_objects
                    ]
                    if missing:
                        raise KeyError(
                            "State '%s' references unknown constraints: %s"
                            % (state.name, missing)
                        )
                    objs = [
                        pyhpp_constraint_objects[n] for n in state.constraints
                    ]
                    self.add_state_constraints(state.name, constraints=objs)

        for edge in task_cls.EDGES.values():
            if self.backend == "corba":
                self.add_edge_constraints(
                    edge.name,
                    constraints=[],
                    constraint_names=edge.path_constraints,
                )
            else:
                missing = [
                    n
                    for n in edge.path_constraints
                    if n not in pyhpp_constraint_objects
                ]
                if missing:
                    raise KeyError(
                        "Edge '%s' references unknown constraints: %s"
                        % (edge.name, missing)
                    )
                objs = [
                    pyhpp_constraint_objects[n] for n in edge.path_constraints
                ]
                self.add_edge_constraints(edge.name, constraints=objs)

        return self.finalize_manual_graph()

    def _extract_factory_graph_structure(self) -> None:
        """
        Extract states and edges from factory-generated graph.
        
        Populates self.states, self.edges, and self.edge_topology.
        """
        try:
            if self.backend == "corba":
                # CORBA graph has nodes and edges dictionaries
                for node_name in self.graph.nodes.keys():
                    self.states[node_name] = node_name

                for edge_name in self.graph.edges.keys():
                    self.edges[edge_name] = edge_name
                    try:
                        result = self.graph.getNodesConnectedByEdge(edge_name)
                        from_node, to_node = result[:2]
                        self.edge_topology[edge_name] = (from_node, to_node)
                    except Exception:
                        pass
            else:  # pyhpp
                # PyHPP graph - extract all states from the graph
                if hasattr(self.graph, 'getStateNames'):
                    for state_name in self.graph.getStateNames():
                        self.states[state_name] = state_name

                if hasattr(self.graph, 'getTransitionNames'):
                    for edge_name in self.graph.getTransitionNames():
                        self.edges[edge_name] = edge_name
                        try:
                            edge_obj = None
                            get_transition = getattr(
                                self.graph, 'getTransition', None
                            )
                            if callable(get_transition):
                                edge_obj = get_transition(edge_name)

                            get_nodes = getattr(
                                self.graph,
                                'getNodesConnectedByTransition',
                                None,
                            )
                            if not callable(get_nodes):
                                continue

                            # Depending on bindings, this may accept either a
                            # Transition object or a transition name.
                            try:
                                query = (
                                    edge_obj
                                    if edge_obj is not None
                                    else edge_name
                                )
                                result = get_nodes(query)
                            except Exception:
                                result = get_nodes(edge_name)

                            is_pair = isinstance(result, tuple)
                            is_pair = is_pair and len(result) >= 2
                            if not is_pair:
                                continue

                            def _state_name(s: Any) -> Optional[str]:
                                if isinstance(s, str):
                                    return s
                                name_attr = getattr(s, 'name', None)
                                if callable(name_attr):
                                    try:
                                        return name_attr()
                                    except Exception:
                                        return None
                                if isinstance(name_attr, str):
                                    return name_attr
                                return None

                            from_name = _state_name(result[0])
                            to_name = _state_name(result[1])
                            if from_name and to_name:
                                self.edge_topology[edge_name] = (
                                    from_name,
                                    to_name,
                                )
                        except Exception:
                            # Skip if bindings don't expose topology helpers.
                            continue
        except Exception as e:
            print(f"    \u26a0 Could not extract graph structure: {e}")

    def apply_state_constraints(
        self,
        state_name: str,
        q: List[float],
        max_iterations: int = 10000,
        error_threshold: float = 1e-4
    ) -> Tuple[bool, List[float], float]:
        """
        Apply state constraints to project configuration.
        
        Args:
            state_name: Name of the state
            q: Configuration to project
            max_iterations: Maximum projection iterations
            error_threshold: Convergence threshold
            
        Returns:
            Tuple of (success, projected_config, error)
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")

        if self.backend == "corba":
            # CORBA uses applyNodeConstraints
            success, q_proj, error = self.graph.applyNodeConstraints(
                state_name, list(q)
            )
        else:  # pyhpp
            # PyHPP uses applyStateConstraints with a State object.
            # NOTE: Do NOT call self.graph.maxIterations() or
            # self.graph.errorThreshold() setters here — they call
            # Graph::invalidate() which resets isInit_=false for all
            # components, breaking the initialized state.
            # Projection parameters are set during graph creation (before
            # graph.initialize() is called).
            # Look up state object and apply constraints
            state_obj = self.graph.getState(state_name)
            success, q_proj, error = self.graph.applyStateConstraints(
                state_obj, np.array(q)
            )

        return success, q_proj, error

    def get_graph(self) -> Any:
        """Get the constraint graph."""
        return self.graph

    def get_states(self) -> Dict[str, int]:
        """Get dictionary of state names to IDs."""
        return self.states.copy()

    def get_edges(self) -> Dict[str, int]:
        """Get dictionary of edge names to IDs."""
        return self.edges.copy()

    def get_edge_topology(self) -> Dict[str, Tuple[str, str]]:
        """Get dictionary of edge names to (from, to) tuples."""
        return self.edge_topology.copy()

    def build_phase_graph(
        self,
        config: BaseTaskConfig,
        held_grasps: Dict[str, str],
        next_grasp: Tuple[str, str],
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
        graph_constraints: Optional[List[str]] = None,
        use_sequential_filter: bool = True,
    ) -> Any:
        """Build a minimal phase graph for incremental multi-grasp planning.
        
        This method creates a drastically reduced constraint graph containing
        only the states and edges needed for a single grasp transition.
        
        Two filtering strategies are available:
        1. setPossibleGrasps (use_sequential_filter=False):
           - Restricts which gripper-handle pairs are valid
           - Still generates intermediate states (e.g., free, A, B, A+B)
           - Results in ~4-10 states per phase
        
        2. SequentialGraspFilter (use_sequential_filter=True, default):
           - Only allows current state and immediate next state
           - Prevents all intermediate/parallel grasp combinations
           - Results in exactly 2 states + waypoints per phase
           - Dramatically reduces graph from O(N!) to O(1) states per
             transition
        
        This reduces graph size from combinatorial explosion to linear growth
        for sequential multi-grasp tasks.
        
        Args:
            config: Task configuration (GRIPPERS, OBJECTS, etc.)
            held_grasps: Currently held grasps {gripper: handle}
            next_grasp: Next grasp to achieve (gripper, handle)
            pyhpp_constraints: PyHPP constraint objects (for pyhpp backend)
            graph_constraints: Optional list of constraint names to add
                globally (e.g., locked joint constraints)
            use_sequential_filter: If True, uses SequentialGraspFilter to
                strictly limit to current→next state transition only.
                If False, uses setPossibleGrasps (allows intermediate states).
        
        Returns:
            Newly created ConstraintGraph or Graph instance
        
        Example:
            >>> # Start free, grasp handle1 with gripper1
            >>> graph = builder.build_phase_graph(
            ...     config, held_grasps={}, next_grasp=("gripper1", "handle1")
            ... )
            >>> # Now hold handle1, grasp handle2 with gripper2
            >>> graph = builder.build_phase_graph(
            ...     config,
            ...     held_grasps={"gripper1": "handle1"},
            ...     next_grasp=("gripper2", "handle2"),
            ... )
        """
        print(
            f"\n    Building phase graph: held={held_grasps}, "
            f"next={next_grasp}"
        )

        if use_sequential_filter:
            print("    Using SequentialGraspFilter (strict 2-state limit)")
        else:
            print("    Using setPossibleGrasps (allows intermediate states)")

        # Delete existing graph to allow recreation (CORBA stores by name)
        if self.graph is not None:
            try:
                if self.backend == "corba":
                    # CORBA: delete graph by name on server
                    graph_name = "graph"
                    self.ps.client.manipulation.graph.deleteGraph(graph_name)
                    print(f"    ✓ Deleted existing graph '{graph_name}'")

                    # CORBA: also reset cached TransitionPlanner
                    # It holds a reference to the old graph
                    if hasattr(self.planner, 'reset_transition_planner'):
                        self.planner.reset_transition_planner()
                        print("    ✓ Reset TransitionPlanner")
                else:
                    # PyHPP: graph is local object, just clear reference
                    print("    ✓ Clearing existing graph reference")

                # Clear internal state
                self.graph = None
                self.factory = None
                self.states = {}
                self.edges = {}
                self.edge_topology = {}
            except Exception as e:
                # Graph might not exist, that's ok
                print(f"    ⓘ Note: {e}")

        # Build minimal VALID_PAIRS for this phase
        phase_valid_pairs = {}

        # Include already-held grasps (must remain valid in graph)
        for gripper, handle in held_grasps.items():
            if gripper not in phase_valid_pairs:
                phase_valid_pairs[gripper] = []
            if handle not in phase_valid_pairs[gripper]:
                phase_valid_pairs[gripper].append(handle)

        # Include the next grasp transition
        next_gripper, next_handle = next_grasp
        if next_gripper not in phase_valid_pairs:
            phase_valid_pairs[next_gripper] = []
        if next_handle not in phase_valid_pairs[next_gripper]:
            phase_valid_pairs[next_gripper].append(next_handle)

        print(f"    Phase VALID_PAIRS: {phase_valid_pairs}")

        # Create a derived config with filtered VALID_PAIRS
        # Use a simple namespace to avoid full class copying
        from types import SimpleNamespace
        phase_config = SimpleNamespace()

        # Copy required attributes from original config
        for attr in dir(config):
            is_private = attr.startswith("_")
            is_callable = callable(getattr(config, attr))
            if not is_private and not is_callable:
                setattr(phase_config, attr, getattr(config, attr))

        # Override VALID_PAIRS for this phase
        phase_config.VALID_PAIRS = phase_valid_pairs

        # ------------------------------------------------------------------
        # Restrict GRIPPERS and OBJECTS to only phase-relevant items.
        # The full DisplayAllStates config has 5+ grippers and 50+ handles,
        # giving (n_handles+1)^n_grippers candidate states for the
        # graspIsAllowed CORBA callback — that makes factory.generate() take
        # hours.  Keep only the grippers and objects that participate in this
        # specific phase transition.
        # ------------------------------------------------------------------
        phase_grippers = list(phase_valid_pairs.keys())
        phase_config.GRIPPERS = phase_grippers
        phase_handle_set = {
            h for handles in phase_valid_pairs.values() for h in handles
        }
        _orig_objects = list(getattr(config, "OBJECTS", []))
        _orig_handles_per_obj = list(getattr(config, "HANDLES_PER_OBJECT", []))
        _orig_contacts_per_obj = list(
            getattr(config, "CONTACT_SURFACES_PER_OBJECT", None)
            or [[] for _ in _orig_objects]
        )
        phase_objects, phase_handles_per_obj, phase_contacts_per_obj = [], [], []
        for obj, handles, contacts in zip(
            _orig_objects, _orig_handles_per_obj, _orig_contacts_per_obj
        ):
            if any(h in phase_handle_set for h in handles):
                phase_objects.append(obj)
                phase_handles_per_obj.append(handles)
                phase_contacts_per_obj.append(contacts)
        phase_config.OBJECTS = phase_objects
        phase_config.HANDLES_PER_OBJECT = phase_handles_per_obj
        phase_config.CONTACT_SURFACES_PER_OBJECT = phase_contacts_per_obj
        print(
            f"    Phase graph restricted to "
            f"grippers={phase_grippers} objects={phase_objects}"
        )

        # Also override RULES to None (setPossibleGrasps takes precedence)
        phase_config.RULES = None

        # Apply sequential filter to enforce strict 2-state limit
        # This prevents the factory from generating intermediate states
        if use_sequential_filter:
            # Import the sequential filter
            try:
                from agimus_spacelab.planning.sequential_grasp_filter import (
                    SequentialGraspFilter,
                    grasps_dict_to_tuple,
                )

                # Use RESTRICTED gripper/handle lists to keep state-space tiny.
                # The full config (DisplayAllStates) has 5+ grippers × 50+
                # handles; using the restricted lists keeps the state-space at
                # most (n_handles+1)^n_phase_grippers which is usually 2.
                grippers = list(phase_config.GRIPPERS)
                handles = []
                for obj_handles in phase_config.HANDLES_PER_OBJECT:
                    handles.extend(obj_handles)

                # Build current grasps dict for phase grippers only
                current_grasps_full = {g: None for g in grippers}
                current_grasps_full.update(
                    {g: h for g, h in held_grasps.items() if g in grippers}
                )

                # Create the sequential filter
                seq_filter = SequentialGraspFilter(
                    grippers=grippers,
                    handles=handles,
                    current_grasps=current_grasps_full,
                    next_grasp=next_grasp,
                )

                print("    ✓ Created SequentialGraspFilter:")
                current_tuple = grasps_dict_to_tuple(
                    current_grasps_full, grippers, handles
                )
                print(f"      Current: {current_tuple}")
                print(f"      Next: {seq_filter.next_grasps}")

                # Store filter for factory injection
                phase_config._SEQUENTIAL_FILTER = seq_filter

            except ImportError as e:
                print(f"    ⚠ SequentialGraspFilter not available: {e}")
                print("    ⚠ Falling back to setPossibleGrasps")
                use_sequential_filter = False

        # Use the factory graph creation with restricted pairs
        # This will call factory.setPossibleGrasps(phase_valid_pairs)
        # and optionally factory.graspIsAllowed.append(seq_filter)
        return self.create_factory_graph(
            phase_config,
            pyhpp_constraints=pyhpp_constraints,
            graph_constraints=graph_constraints,
        )


__all__ = ["GraphBuilder"]
