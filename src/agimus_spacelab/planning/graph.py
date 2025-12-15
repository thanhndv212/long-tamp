#!/usr/bin/env python3
"""
Constraint graph builder for manipulation tasks.

Provides GraphBuilder for creating constraint graphs with dual backend support.
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Optional, Any, Type, Union

from agimus_spacelab.config.base_config import BaseTaskConfig, ConstraintDef
from agimus_spacelab.planning.constraints import (
    ConstraintBuilder,
    FactoryConstraintLibrary,
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
        
    def create_manual_graph(self, name: str = "graph") -> Any:
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
            print("   ✓ CORBA graph initialized (manual mode)")
            
        else:  # pyhpp
            if not HAS_PYHPP_GRAPH:
                raise ImportError("PyHPP backend not available")
            
            self.graph = PyHPPGraph(name, self.robot, self.ps)
            self.graph.maxIterations(10000)
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
                "Graph not initialized. Call create_manual_graph() first"
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
                "Graph not initialized. Call create_manual_graph() first"
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
                "Graph not initialized. Call create_manual_graph() first"
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

    def finalize_manual_graph(self) -> Any:
        """
        Finalize manual graph construction and initialize it.
        
        Returns:
            Initialized graph
        """
        if self.graph is None:
            raise RuntimeError("Graph not initialized")
        
        print(f"   Finalizing graph with {len(self.states)} states "
              f"and {len(self.edges)} edges")
        
        if self.backend == "corba":
            # CORBA graph initialization
            self.graph.initialize()
            print("   ✓ CORBA graph initialized")
        else:  # pyhpp
            # PyHPP graph initialization
            self.graph.initialize()
            self._attach_graph_to_problem_if_supported()
            print("   ✓ PyHPP graph initialized")
        
        return self.graph

    def create_factory_graph(
        self,
        grippers: List[str],
        objects: List[str],
        handles_per_object: List[List[str]],
        contact_surfaces_per_object: Optional[List[List[str]]] = None,
        environment_contacts: Optional[List[str]] = None,
        rules: Optional[List] = None,
        valid_pairs: Optional[Dict[str, List[str]]] = None,
        pyhpp_constraints: Optional[Dict[str, Any]] = None,
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
            self.graph.maxIterations(10000)
            self.graph.errorThreshold(1e-4)
            # PyHPP factory can be provided a dict of already-available
            # numerical constraints (name -> constraint object).
            self.factory = PyHPPConstraintGraphFactory(
                self.graph, constraints=pyhpp_constraints or {}
            )
        
        # Set grippers
        self.factory.setGrippers(grippers)
        print(f"    \u2713 Set grippers: {grippers}")
        
        # Set objects with handles and contact surfaces
        if contact_surfaces_per_object is None:
            contact_surfaces_per_object = [[] for _ in objects]
        self.factory.setObjects(
            objects, handles_per_object, contact_surfaces_per_object
        )
        print(f"    \u2713 Set objects: {objects}")
        
        # Set environment contacts if provided
        if environment_contacts:
            self.factory.environmentContacts(environment_contacts)
            print(f"    \u2713 Set environment contacts: "
                  f"{environment_contacts}")
        
        # Set grasp restrictions
        if rules is not None:
            # Use explicitly provided rules
            self.factory.setRules(rules)
            print("    \u2713 Set custom rules")
        elif valid_pairs is not None:
            # Use setPossibleGrasps - more convenient than rules
            # valid_pairs format: {gripper_name: [handle1, handle2, ...]}
            self.factory.setPossibleGrasps(valid_pairs)
            print("    \u2713 Set possible grasps from valid_pairs")
        # If neither rules nor valid_pairs, allow all (default behavior)
        
        # Generate graph
        self.factory.generate()
        print("    \u2713 Generated graph structure")
        
        # Initialize graph
        self.graph.initialize()
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

    def get_factory_constraint_library(
        self, task: TaskConfigT
    ) -> FactoryConstraintLibrary:
        """Return a ConstraintGraphFactory naming helper for this task."""

        task_cls = self._as_task_config(task)
        return FactoryConstraintLibrary(
            grippers=list(getattr(task_cls, "GRIPPERS", [])),
            objects=list(getattr(task_cls, "OBJECTS", [])),
            handles_per_object=task_cls.get_handles_per_object(),
        )

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
        self.create_manual_graph(name=name)

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
                            edge_obj = self.graph.getTransition(edge_name)
                            # getNodesConnectedByTransition returns strings
                            # (state names) via output parameters converted
                            # to tuple
                            result = self.graph.getNodesConnectedByTransition(
                                edge_obj
                            )
                            print("Edge topology result:", result)
                            # Result should be (from_name, to_name) as strings
                            if isinstance(result, tuple) and len(result) >= 2:
                                from_name, to_name = result[0], result[1]
                                self.edge_topology[edge_name] = (
                                    from_name, to_name
                                )
                        except Exception:
                            # Silently skip if method not available
                            print("Could not get edge topology for", edge_name)
                            pass
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
            # PyHPP uses applyConstraints
            # Store current parameters
            old_max_iter = self.graph.maxIterations()
            old_error = self.graph.errorThreshold()
            
            # Set temporary parameters
            self.graph.maxIterations(max_iterations)
            self.graph.errorThreshold(error_threshold)
            
            # Apply constraints
            success, q_proj, error = self.graph.applyConstraints(
                state_name, list(q)
            )
            
            # Restore parameters
            self.graph.maxIterations(old_max_iter)
            self.graph.errorThreshold(old_error)
        
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


__all__ = ["GraphBuilder"]
