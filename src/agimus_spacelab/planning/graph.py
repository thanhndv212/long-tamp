#!/usr/bin/env python3
"""
Constraint graph builder for manipulation tasks.

Provides GraphBuilder for creating constraint graphs with dual backend support.
"""

from typing import Dict, List, Tuple, Optional, Any

# Import for CORBA backend
try:
    from hpp.corbaserver.manipulation import (
        ConstraintGraph,
        ConstraintGraphFactory,
        Rule
    )
    HAS_CORBA_GRAPH = True
except ImportError:
    HAS_CORBA_GRAPH = False
    ConstraintGraph = None
    ConstraintGraphFactory = None
    Rule = None

# Import for PyHPP backend
try:
    from pyhpp.manipulation import Graph
    HAS_PYHPP_GRAPH = True
except ImportError:
    HAS_PYHPP_GRAPH = False
    Graph = None


class GraphBuilder:
    """
    Builder class for creating constraint graphs with dual backend support.
    
    Supports both manual graph construction (node by node, edge by edge) and
    factory-based automatic graph generation (CORBA only).
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
            
            self.graph = Graph(name, self.robot, self.ps)
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
            # CORBA uses createEdge
            edge_id = self.graph.createEdge(
                from_state, to_state, name, weight,
                containing_state or from_state
            )
        else:  # pyhpp
            # PyHPP uses createEdge/createTransition
            edge_id = self.graph.createEdge(
                from_state, to_state, name, weight,
                containing_state or from_state
            )
        
        self.edges[name] = edge_id
        self.edge_topology[name] = (from_state, to_state)
        print(f"    ✓ Edge '{name}': {from_state} → {to_state} "
              f"(ID: {edge_id})")
        return edge_id
    
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
            print("   ✓ PyHPP graph initialized")
        
        return self.graph
    
    def create_factory_graph(
        self,
        grippers: List[str],
        objects: List[str],
        handles: Dict[str, List[str]],
        rules: Optional[List] = None
    ) -> Any:
        """
        Create constraint graph using factory (CORBA only for now).
        
        Args:
            grippers: List of gripper names
            objects: List of object names
            handles: Dictionary mapping object names to handle lists
            rules: Optional list of Rule objects for graph generation
            
        Returns:
            ConstraintGraph instance
        """
        if self.backend != "corba":
            raise NotImplementedError(
                "Factory mode currently only supported for CORBA backend"
            )
        
        if not HAS_CORBA_GRAPH:
            raise ImportError("CORBA backend not available")
        
        print("   Creating factory-based constraint graph")
        print(f"     Grippers: {grippers}")
        print(f"     Objects: {objects}")
        
        # Create constraint graph
        self.graph = ConstraintGraph(self.robot, "graph")
        
        # Create factory
        self.factory = ConstraintGraphFactory(self.graph)
        
        # Set grippers
        for gripper in grippers:
            self.factory.setGrippers([gripper])
        
        # Set objects with handles
        for obj_name in objects:
            if obj_name in handles:
                obj_handles = handles[obj_name]
                self.factory.setObjects(
                    [obj_name],
                    [obj_handles],
                    [[]]  # No contact surfaces
                )
        
        # Generate graph with rules
        if rules is None:
            # Default: generate all possible grasps
            print("     Using default rules (all grasps)")
            rules = [Rule(grippers, objects, True)]
        
        self.factory.generate(rules)
        
        # Initialize graph
        self.graph.initialize()
        
        # Store states and edges for tracking
        try:
            for node_name in self.graph.nodes.keys():
                self.states[node_name] = node_name
            
            for edge_name in self.graph.edges.keys():
                self.edges[edge_name] = edge_name
                # Try to get topology
                try:
                    result = self.graph.getNodesConnectedByEdge(edge_name)
                    from_node, to_node = result[:2]
                    self.edge_topology[edge_name] = (from_node, to_node)
                except Exception:
                    pass
        except Exception as e:
            print(f"     ⚠ Could not extract graph structure: {e}")
        
        print(f"   ✓ Factory graph created with {len(self.states)} states "
              f"and {len(self.edges)} edges")
        return self.graph
    
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
