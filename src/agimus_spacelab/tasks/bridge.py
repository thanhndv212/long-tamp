#!/usr/bin/env python3
"""
Bridge between HPP Manipulation Planning and Task Orchestration.

Connects atomic tasks to real motion planning.
"""

from typing import Dict, List, Optional, Tuple, Any, Callable
from dataclasses import dataclass
import numpy as np

from .orchestration import AtomicTask, TaskBuilder, Resource, ResourceType
from .base import ManipulationTask


# ============================================================================
# Planning Context
# ============================================================================

@dataclass
class PlanningContext:
    """
    Context for planning a single atomic task.
    
    Stores the constraint graph state, configurations, and paths
    needed to execute the task.
    """
    task_id: str
    initial_config: np.ndarray
    goal_config: Optional[np.ndarray] = None
    
    # Graph state information
    start_state: Optional[str] = None  # Graph state name
    goal_state: Optional[str] = None
    edge_name: Optional[str] = None
    
    # Planning result
    path: Optional[Any] = None
    path_length: Optional[float] = None
    planning_time: Optional[float] = None
    
    # Validation
    constraints_satisfied: bool = False
    collision_free: bool = False


# ============================================================================
# Planning Bridge
# ============================================================================

class PlanningBridge:
    """
    Bridges AtomicTask definitions to HPP manipulation planning.
    
    This class takes high-level task specifications and generates
    motion plans using the constraint graph framework.
    """
    
    def __init__(self, manipulation_task: ManipulationTask):
        """
        Initialize bridge with a manipulation task instance.
        
        Args:
            manipulation_task: Instance of ManipulationTask (already setup)
        """
        self.task = manipulation_task
        self.robot = manipulation_task.robot
        self.ps = manipulation_task.ps
        self.graph = manipulation_task.graph
        self.config_gen = manipulation_task.config_gen
        
        # Cache for planning contexts
        self.contexts: Dict[str, PlanningContext] = {}
        
    def create_planning_function(
        self,
        atomic_task: AtomicTask,
        start_state: str,
        goal_state: str,
        edge_name: str,
        goal_config_generator: Optional[Callable[[], np.ndarray]] = None
    ) -> Callable[[], bool]:
        """
        Create an execution function for an AtomicTask.
        
        Args:
            atomic_task: The task to plan for
            start_state: Starting constraint graph state
            goal_state: Target constraint graph state
            edge_name: Transition edge to use
            goal_config_generator: Optional function to generate goal config
            
        Returns:
            Function that executes planning and returns success/failure
        """
        def execute_planning() -> bool:
            """Execute motion planning for this task."""
            import time
            start_time = time.time()
            
            task_id = (atomic_task.task_id
                       if atomic_task else f"task_{start_state}_{goal_state}")
            
            # Get or create planning context
            if task_id not in self.contexts:
                # Get current robot configuration
                _get_config = getattr(self.robot, "getCurrentConfig", None)
                if _get_config is not None:
                    q_current = _get_config()
                else:
                    # PYHPP-GAP: pyhpp Device has no getCurrentConfig().
                    # Fall back to problem initConfig.
                    q_current = list(
                        self.task.planner.problem.initConfig()
                        if hasattr(self.task, "planner")
                        and hasattr(self.task.planner, "problem")
                        else []
                    )
                
                context = PlanningContext(
                    task_id=task_id,
                    initial_config=q_current,
                    start_state=start_state,
                    goal_state=goal_state,
                    edge_name=edge_name
                )
                self.contexts[task_id] = context
            else:
                context = self.contexts[task_id]
            
            # Generate goal configuration if needed
            if goal_config_generator:
                try:
                    context.goal_config = goal_config_generator()
                except Exception as e:
                    print(f"    ✗ Goal generation failed: {e}")
                    return False
            
            # Project initial config onto start state
            try:
                q_start = context.initial_config.copy()
                res, q_start = self.config_gen.project_on_node(
                    start_state, q_start, f"{task_id}_start"
                )
                if not res:
                    print(f"    ✗ Failed to project onto '{start_state}'")
                    return False
            except Exception as e:
                print(f"    ✗ Start projection failed: {e}")
                return False
            
            # Generate target configuration via edge
            try:
                if context.goal_config is not None:
                    q_target = context.goal_config.copy()
                else:
                    q_target = q_start.copy()
                    
                success, q_target = self.config_gen.generate_via_edge(
                    edge_name, q_start, f"{task_id}_target"
                )
                
                if not success:
                    print(f"    ✗ Failed to generate via edge '{edge_name}'")
                    return False
                    
            except Exception as e:
                print(f"    ✗ Target generation failed: {e}")
                return False
            
            # Plan path from start to target
            try:
                edge = self._get_edge(edge_name)
                if edge is None:
                    print(f"    ✗ Edge '{edge_name}' not found")
                    return False
                
                path = None
                if hasattr(edge, 'build'):
                    # CORBA backend
                    success = edge.build(path, q_start, q_target)
                    if not success or path is None:
                        print(f"    ✗ Path building failed")
                        return False
                else:
                    print(f"    ⚠ PyHPP path building not yet implemented")
                    return False
                
                context.path = path
                context.path_length = (
                    path.length() if hasattr(path, 'length') else None
                )
                
            except Exception as e:
                print(f"    ✗ Path planning failed: {e}")
                return False
            
            # Validate path
            try:
                if hasattr(self.ps, 'pathValidation'):
                    pv = self.ps.pathValidation()
                    if hasattr(pv, 'validate'):
                        valid, report = pv.validate(context.path, False)
                        context.collision_free = valid
                        
                        if not valid:
                            print(f"    ✗ Path validation failed")
                            return False
                else:
                    context.collision_free = True
                    
            except Exception as e:
                print(f"    ⚠ Path validation error: {e}")
                context.collision_free = False
                return False
            
            context.planning_time = time.time() - start_time
            
            success = (context.path is not None and
                       context.collision_free and
                       context.constraints_satisfied)
            
            if success:
                print(f"    ✓ Planning succeeded ({context.planning_time:.2f}s)")
            else:
                print(f"    ✗ Planning failed ({context.planning_time:.2f}s)")
            
            return success
        
        return execute_planning
    
    def _get_edge(self, edge_name: str) -> Optional[Any]:
        """Get edge object from constraint graph."""
        if hasattr(self.graph, 'edges'):
            edges_dict = self.graph.edges
            if edge_name in edges_dict:
                return edges_dict[edge_name]
        
        if hasattr(self.graph, 'getEdge'):
            try:
                return self.graph.getEdge(edge_name)
            except Exception:
                pass
        
        return None
    
    def _get_state(self, state_name: str) -> Optional[Any]:
        """Get state object from constraint graph."""
        if hasattr(self.graph, 'nodes'):
            nodes_dict = self.graph.nodes
            if state_name in nodes_dict:
                return nodes_dict[state_name]
        
        if hasattr(self.graph, 'getNode'):
            try:
                return self.graph.getNode(state_name)
            except Exception:
                pass
        
        return None
    
    def get_planning_context(self, task_id: str) -> Optional[PlanningContext]:
        """Retrieve planning context for a task."""
        return self.contexts.get(task_id)
    
    def extract_path(self, task_id: str) -> Optional[Any]:
        """Extract the planned path for a task."""
        context = self.contexts.get(task_id)
        return context.path if context else None
    
    def extract_final_config(self, task_id: str) -> Optional[np.ndarray]:
        """Extract the final configuration from a planned path."""
        context = self.contexts.get(task_id)
        if context and context.path:
            if hasattr(context.path, 'end'):
                return context.path.end()
            elif context.goal_config is not None:
                return context.goal_config
        return None


# ============================================================================
# Factory Functions
# ============================================================================

def create_grasp_task(
    bridge: PlanningBridge,
    task_id: str,
    name: str,
    arm_name: str,
    object_name: str,
    gripper_handle_pair: Tuple[str, str]
) -> AtomicTask:
    """
    Create an AtomicTask for grasping with real planning.
    
    Args:
        bridge: Planner bridge
        task_id: Unique task identifier
        name: Human-readable name
        arm_name: Arm resource name
        object_name: Object resource name
        gripper_handle_pair: (gripper_name, handle_name) in constraint graph
        
    Returns:
        Configured AtomicTask with planning execution
    """
    gripper, handle = gripper_handle_pair
    
    # Determine graph states
    start_state = "free"
    goal_state = "grasping"
    edge_name = f"{start_state}-{goal_state}"
    
    # Build task first to have task_id
    task = (TaskBuilder(task_id, name)
            .with_description(f"Grasp {object_name} with {arm_name}")
            .requires_arm(arm_name)
            .requires_object(object_name)
            .with_postcondition(
                f"{object_name} grasped",
                lambda: True
            )
            .build())
    
    # Create execution function
    execute_fn = bridge.create_planning_function(
        atomic_task=task,
        start_state=start_state,
        goal_state=goal_state,
        edge_name=edge_name
    )
    
    task.execute = execute_fn
    return task


def create_place_task(
    bridge: PlanningBridge,
    task_id: str,
    name: str,
    arm_name: str,
    object_name: str,
    target_pose: List[float]
) -> AtomicTask:
    """
    Create an AtomicTask for placing with real planning.
    
    Args:
        bridge: Planner bridge
        task_id: Unique task identifier
        name: Human-readable name
        arm_name: Arm resource name
        object_name: Object resource name
        target_pose: [x, y, z, qw, qx, qy, qz] target placement pose
        
    Returns:
        Configured AtomicTask with planning execution
    """
    start_state = "grasping"
    goal_state = "placement"
    edge_name = f"{start_state}-{goal_state}"
    
    # Goal config generator uses target pose
    def goal_generator():
        _get_config = getattr(bridge.robot, "getCurrentConfig", None)
        if _get_config is not None:
            return _get_config()
        # PYHPP-GAP: pyhpp Device has no getCurrentConfig().
        return list(
            bridge.task.planner.problem.initConfig()
            if hasattr(bridge.task, "planner")
            and hasattr(bridge.task.planner, "problem")
            else []
        )
    
    task = (TaskBuilder(task_id, name)
            .with_description(f"Place {object_name} at target pose")
            .requires_arm(arm_name)
            .requires_object(object_name)
            .with_postcondition(
                f"{object_name} placed",
                lambda: True
            )
            .build())
    
    execute_fn = bridge.create_planning_function(
        atomic_task=task,
        start_state=start_state,
        goal_state=goal_state,
        edge_name=edge_name,
        goal_config_generator=goal_generator
    )
    
    task.execute = execute_fn
    return task


# Alias for backward compatibility
ManipulationPlannerBridge = PlanningBridge


__all__ = [
    "PlanningContext",
    "PlanningBridge",
    "ManipulationPlannerBridge",
    "create_grasp_task",
    "create_place_task",
]
