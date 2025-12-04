# Agimus SpaceLab: Comprehensive Improvement Plan

**Date:** December 3, 2025  
**Status:** Analysis Complete - Implementation Roadmap  
**Priority:** HIGH - Critical for multi-arm collaborative manipulation

---

## Executive Summary

The agimus_spacelab package provides a solid foundation for multi-arm manipulation planning but lacks critical integration with hpp-manipulation's core capabilities. This document outlines a structured improvement plan to transform it from a task orchestration framework into a complete manipulation planning system with behavior tree integration.

**Key Achievement Targets:**
- ✅ Real motion planning integration (not mocks)
- ✅ Behavior tree framework for dynamic task composition
- ✅ Leverage hpp-manipulation's foliation and constraint graph patterns
- ✅ Multi-arm synchronization primitives
- ✅ Failure recovery and replanning

---

## Architecture Analysis

### Current System (3-Layer)

```
┌─────────────────────────────────────────┐
│     Task Orchestration Layer            │  ← Implemented
│  - Dependency management                │
│  - Resource allocation                  │
│  - Mock execution                       │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│        Atomic Task Layer                │  ← Implemented
│  - Task definitions                     │
│  - Pre/postconditions                   │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│     Motion Planning Layer               │  ← NOT CONNECTED
│  - HPP Manipulation (exists)            │
│  - No bridge to orchestrator            │
└─────────────────────────────────────────┘
```

### Target System (5-Layer with BT)

```
┌─────────────────────────────────────────┐
│       Assembly Mission Layer            │  ← TO IMPLEMENT
│  - High-level goals                     │
│  - Success criteria                     │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│      Behavior Tree Layer                │  ← TO IMPLEMENT
│  - Dynamic task composition             │
│  - Reactive execution                   │
│  - Failure handling                     │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│    Task Orchestration Layer             │  ← ENHANCE
│  - Multi-arm scheduling                 │
│  - Resource management                  │
│  - Execution monitoring                 │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│       Atomic Task Layer                 │  ← ENHANCE
│  - Real planning integration            │
│  - Constraint validation                │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│     HPP Manipulation Layer              │  ← INTEGRATE
│  - Constraint graphs                    │
│  - Foliation-based planning             │
│  - Path optimization                    │
└─────────────────────────────────────────┘
```

---

## Phase 1: Core Integration (Weeks 1-3) 🔥 **CRITICAL**

### 1.1 Manipulation Planner Bridge ✅ **DONE**

**File Created:** `manipulation_bridge.py`

**Purpose:** Connect AtomicTask execution to real HPP planning

**Components:**
- `PlanningContext`: Stores planning state per task
- `ManipulationPlannerBridge`: Executes motion planning
- `create_grasp_task()`, `create_place_task()`: Factory functions

**Integration Pattern:**
```python
from manipulation_bridge import ManipulationPlannerBridge, create_grasp_task

# Setup manipulation task (existing)
task = GraspFrameGripperTask(backend="corba")
task.setup()

# Create bridge
bridge = ManipulationPlannerBridge(task)

# Create atomic task with real planning
atomic_task = create_grasp_task(
    bridge=bridge,
    task_id="t1_grasp_fg",
    name="UR10 grasp frame_gripper",
    arm_name="UR10",
    object_name="frame_gripper",
    gripper_handle_pair=("spacelab/g_ur10_tool", "frame_gripper/h_FG_tool")
)

# Add to orchestrator
orchestrator = TaskOrchestrator()
orchestrator.add_task(atomic_task)
orchestrator.run()  # Now executes REAL planning!
```

**Next Steps:**
1. Test bridge with existing `task_grasp_frame_gripper.py`
2. Add error handling for planning failures
3. Extract path durations for execution monitoring
4. Cache successful paths for replanning

---

### 1.2 Constraint Graph Pattern Library (Week 2)

**File to Create:** `graph_patterns.py`

**Problem:** Manual graph construction is error-prone and doesn't leverage hpp-manipulation patterns.

**Solution:** Library of reusable constraint graph patterns.

```python
"""
Pre-built constraint graph patterns for common manipulation scenarios.
"""

class GraphPatternLibrary:
    """Factory for common manipulation graph patterns."""
    
    @staticmethod
    def create_grasp_pattern(
        robot, ps, gripper_name, handle_name,
        use_pregrasp=True, use_complement=True
    ):
        """
        Create standard grasp pattern:
        
        free ──approach──> pregrasp ──grasp──> grasping
         │                    │                   │
         └────────────────────┴───────────────────┘
                     (complement path)
        
        Returns:
            graph, (states, edges) tuple
        """
        from hpp.corbaserver.manipulation import ConstraintGraph
        
        graph = ConstraintGraph(robot, "grasp_pattern")
        
        # Create states
        graph.createNode(["free", "pregrasp", "grasping"])
        
        # Create edges
        graph.createEdge("free", "pregrasp", "approach", 10)
        graph.createEdge("pregrasp", "grasping", "grasp", 1)
        graph.createEdge("grasping", "pregrasp", "release", 1)
        graph.createEdge("free", "free", "free_motion", 1)
        
        # Add constraints
        # ... (use ConstraintBuilder)
        
        graph.initialize()
        return graph
    
    @staticmethod
    def create_pick_and_place_pattern(
        robot, ps, gripper_name, handle_name,
        placement_surfaces, use_preplace=True
    ):
        """
        Create pick-and-place pattern:
        
        placement ──approach──> pregrasp ──grasp──> grasping
            │                                          │
            │                                          │
            └──────────────place───────────────────────┘
                       (via preplace)
        
        Returns:
            graph, (states, edges) tuple
        """
        # Implementation...
        pass
    
    @staticmethod
    def create_handover_pattern(
        robot, ps, 
        gripper1_name, gripper2_name, handle_name
    ):
        """
        Create handover pattern for multi-arm coordination:
        
        arm1_holding ──approach──> both_holding ──release1──> arm2_holding
        
        Returns:
            graph, (states, edges) tuple
        """
        # Implementation...
        pass
    
    @staticmethod
    def create_dual_arm_assembly_pattern(
        robot, ps,
        gripper1_name, handle1_name,
        gripper2_name, handle2_name,
        assembly_constraint
    ):
        """
        Create dual-arm assembly pattern:
        
        both_grasping ──align──> aligned ──assemble──> assembled
        
        Where 'assembled' enforces relative pose constraint between objects.
        
        Returns:
            graph, (states, edges) tuple
        """
        # Implementation...
        pass
```

**Usage in Tasks:**
```python
class GraspTask(ManipulationTask):
    def create_graph(self):
        from graph_patterns import GraphPatternLibrary
        
        graph = GraphPatternLibrary.create_grasp_pattern(
            self.robot, self.ps,
            gripper_name="spacelab/g_ur10_tool",
            handle_name="frame_gripper/h_FG_tool",
            use_pregrasp=True,
            use_complement=True
        )
        return graph
```

**Benefit:** Reduces task code from 200+ lines to ~50 lines.

---

### 1.3 Factory-Based Graph Generation (Week 3)

**File to Enhance:** `spacelab_tools.py` → `GraphBuilder` class

**Problem:** Only manual graph construction supported; factory mode unused.

**Solution:** Add factory-based automatic graph generation.

```python
class GraphBuilder:
    """Enhanced builder with factory support."""
    
    def create_graph_with_factory(
        self,
        grippers: List[str],
        objects: Dict[str, Dict],  # object_name -> {handles: [...], surfaces: [...]}
        rules: str = "auto"  # 'auto', 'all', 'sequential', custom
    ):
        """
        Create constraint graph using ConstraintGraphFactory.
        
        Args:
            grippers: List of gripper names
            objects: Dict mapping object names to their handles/surfaces
            rules: Rule generation strategy
            
        Returns:
            Initialized constraint graph
        """
        if self.backend == "corba":
            from hpp.corbaserver.manipulation import (
                ConstraintGraph, ConstraintGraphFactory, Rule
            )
            
            graph = ConstraintGraph(self.robot, "auto_graph")
            factory = ConstraintGraphFactory(graph)
            
            # Set grippers
            factory.setGrippers(grippers)
            
            # Set objects
            obj_names = list(objects.keys())
            handles_per_obj = [objects[obj]["handles"] for obj in obj_names]
            surfaces_per_obj = [objects[obj].get("surfaces", []) for obj in obj_names]
            
            factory.setObjects(obj_names, handles_per_obj, surfaces_per_obj)
            
            # Generate rules
            if rules == "auto":
                # Allow all valid pairs
                factory_rules = [Rule([".*"], [".*"], True)]
            elif rules == "sequential":
                # Only allow specific sequence
                factory_rules = self._generate_sequential_rules(objects)
            else:
                factory_rules = rules  # Custom rules provided
            
            factory.setRules(factory_rules)
            
            # Generate and initialize
            factory.generate()
            graph.initialize()
            
            return graph
        elif self.backend == "pyhpp":
            # PyHPP factory implementation
            raise NotImplementedError("PyHPP factory not yet implemented")
```

**Usage:**
```python
builder = GraphBuilder(planner, robot, ps, backend="corba")

objects = {
    "frame_gripper": {
        "handles": ["frame_gripper/h_FG_tool"],
        "surfaces": []
    },
    "RS1": {
        "handles": ["RS1/h_RS1_FG"],
        "surfaces": ["RS1/surface_top"]
    }
}

graph = builder.create_graph_with_factory(
    grippers=["spacelab/g_ur10_tool"],
    objects=objects,
    rules="auto"
)
```

---

## Phase 2: Behavior Tree Integration (Weeks 4-6) 🔥 **HIGH PRIORITY**

### 2.1 Behavior Tree Core (Week 4)

**File to Create:** `behavior_tree.py`

**Purpose:** Reactive task composition and failure recovery.

**Key Concepts:**
- **Selector**: Try children in order until one succeeds (OR)
- **Sequence**: Execute children in order until one fails (AND)
- **Parallel**: Execute children concurrently
- **Decorator**: Modify child behavior (retry, timeout, etc.)

```python
"""
Behavior Tree framework for manipulation planning.

Enables dynamic, reactive task composition with:
- Hierarchical task decomposition
- Failure recovery
- Conditional execution
- Parallel actions
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Optional, Callable
import time


class NodeStatus(Enum):
    """Execution status of a behavior tree node."""
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class BehaviorNode(ABC):
    """Base class for all behavior tree nodes."""
    
    def __init__(self, name: str):
        self.name = name
        self.status = NodeStatus.RUNNING
        
    @abstractmethod
    def tick(self) -> NodeStatus:
        """
        Execute one step of the behavior.
        
        Returns:
            NodeStatus indicating result
        """
        pass
    
    def reset(self):
        """Reset node state."""
        self.status = NodeStatus.RUNNING


class ActionNode(BehaviorNode):
    """Leaf node that executes an action."""
    
    def __init__(self, name: str, action: Callable[[], bool]):
        super().__init__(name)
        self.action = action
        
    def tick(self) -> NodeStatus:
        try:
            success = self.action()
            self.status = NodeStatus.SUCCESS if success else NodeStatus.FAILURE
        except Exception as e:
            print(f"Action '{self.name}' exception: {e}")
            self.status = NodeStatus.FAILURE
        
        return self.status


class ConditionNode(BehaviorNode):
    """Leaf node that checks a condition."""
    
    def __init__(self, name: str, condition: Callable[[], bool]):
        super().__init__(name)
        self.condition = condition
        
    def tick(self) -> NodeStatus:
        self.status = (NodeStatus.SUCCESS if self.condition() 
                      else NodeStatus.FAILURE)
        return self.status


class SequenceNode(BehaviorNode):
    """Executes children in order until one fails."""
    
    def __init__(self, name: str, children: List[BehaviorNode] = None):
        super().__init__(name)
        self.children = children or []
        self.current_child = 0
        
    def add_child(self, child: BehaviorNode):
        self.children.append(child)
        
    def tick(self) -> NodeStatus:
        while self.current_child < len(self.children):
            child = self.children[self.current_child]
            status = child.tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.current_child = 0  # Reset for next execution
                return NodeStatus.FAILURE
            else:  # SUCCESS
                self.current_child += 1
        
        # All children succeeded
        self.current_child = 0
        return NodeStatus.SUCCESS


class SelectorNode(BehaviorNode):
    """Tries children in order until one succeeds."""
    
    def __init__(self, name: str, children: List[BehaviorNode] = None):
        super().__init__(name)
        self.children = children or []
        self.current_child = 0
        
    def add_child(self, child: BehaviorNode):
        self.children.append(child)
        
    def tick(self) -> NodeStatus:
        while self.current_child < len(self.children):
            child = self.children[self.current_child]
            status = child.tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.SUCCESS:
                self.current_child = 0
                return NodeStatus.SUCCESS
            else:  # FAILURE
                self.current_child += 1
        
        # All children failed
        self.current_child = 0
        return NodeStatus.FAILURE


class ParallelNode(BehaviorNode):
    """Executes children concurrently."""
    
    def __init__(self, name: str, 
                 children: List[BehaviorNode] = None,
                 success_threshold: int = None,  # None = all must succeed
                 failure_threshold: int = 1):    # 1 = fail on first failure
        super().__init__(name)
        self.children = children or []
        self.success_threshold = success_threshold or len(self.children)
        self.failure_threshold = failure_threshold
        
    def add_child(self, child: BehaviorNode):
        self.children.append(child)
        
    def tick(self) -> NodeStatus:
        success_count = 0
        failure_count = 0
        running_count = 0
        
        for child in self.children:
            status = child.tick()
            
            if status == NodeStatus.SUCCESS:
                success_count += 1
            elif status == NodeStatus.FAILURE:
                failure_count += 1
            else:  # RUNNING
                running_count += 1
        
        # Check thresholds
        if failure_count >= self.failure_threshold:
            return NodeStatus.FAILURE
        elif success_count >= self.success_threshold:
            return NodeStatus.SUCCESS
        else:
            return NodeStatus.RUNNING


class DecoratorNode(BehaviorNode):
    """Modifies behavior of a single child."""
    
    def __init__(self, name: str, child: BehaviorNode):
        super().__init__(name)
        self.child = child


class RetryDecorator(DecoratorNode):
    """Retries child on failure."""
    
    def __init__(self, name: str, child: BehaviorNode, max_retries: int = 3):
        super().__init__(name, child)
        self.max_retries = max_retries
        self.retry_count = 0
        
    def tick(self) -> NodeStatus:
        status = self.child.tick()
        
        if status == NodeStatus.FAILURE:
            self.retry_count += 1
            if self.retry_count < self.max_retries:
                print(f"Retrying '{self.child.name}' ({self.retry_count}/{self.max_retries})")
                self.child.reset()
                return NodeStatus.RUNNING
            else:
                print(f"Max retries reached for '{self.child.name}'")
                self.retry_count = 0
                return NodeStatus.FAILURE
        elif status == NodeStatus.SUCCESS:
            self.retry_count = 0
        
        return status


class TimeoutDecorator(DecoratorNode):
    """Fails if child exceeds timeout."""
    
    def __init__(self, name: str, child: BehaviorNode, timeout: float):
        super().__init__(name, child)
        self.timeout = timeout
        self.start_time = None
        
    def tick(self) -> NodeStatus:
        if self.start_time is None:
            self.start_time = time.time()
        
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            print(f"Timeout for '{self.child.name}' ({elapsed:.1f}s > {self.timeout:.1f}s)")
            self.start_time = None
            return NodeStatus.FAILURE
        
        status = self.child.tick()
        
        if status != NodeStatus.RUNNING:
            self.start_time = None
        
        return status


class BehaviorTree:
    """
    Behavior tree for reactive task execution.
    
    Example:
        tree = BehaviorTree("Assembly")
        
        sequence = SequenceNode("Pick and Place")
        sequence.add_child(ActionNode("Grasp", grasp_fn))
        sequence.add_child(ActionNode("Transport", transport_fn))
        sequence.add_child(ActionNode("Place", place_fn))
        
        tree.set_root(sequence)
        
        while tree.tick() == NodeStatus.RUNNING:
            time.sleep(0.1)
    """
    
    def __init__(self, name: str, root: Optional[BehaviorNode] = None):
        self.name = name
        self.root = root
        
    def set_root(self, root: BehaviorNode):
        self.root = root
        
    def tick(self) -> NodeStatus:
        if self.root is None:
            return NodeStatus.FAILURE
        
        return self.root.tick()
    
    def run(self, max_iterations: int = 1000) -> bool:
        """
        Run tree until completion or max iterations.
        
        Returns:
            True if tree succeeded
        """
        for i in range(max_iterations):
            status = self.tick()
            
            if status == NodeStatus.SUCCESS:
                print(f"Behavior tree '{self.name}' succeeded")
                return True
            elif status == NodeStatus.FAILURE:
                print(f"Behavior tree '{self.name}' failed")
                return False
            
            time.sleep(0.01)  # Small delay
        
        print(f"Behavior tree '{self.name}' exceeded max iterations")
        return False


# ============================================================================
# Example Usage
# ============================================================================

def example_assembly_tree():
    """Example: Pick-and-place with fallback."""
    
    # Define actions (would be real planning functions)
    def grasp_primary():
        print("  Attempting primary grasp...")
        return False  # Simulate failure
    
    def grasp_alternative():
        print("  Attempting alternative grasp...")
        return True
    
    def transport():
        print("  Transporting...")
        return True
    
    def place():
        print("  Placing...")
        return True
    
    # Build tree
    tree = BehaviorTree("Assembly")
    
    # Root sequence
    root = SequenceNode("Pick and Place")
    
    # Grasp with fallback
    grasp_selector = SelectorNode("Grasp with Fallback")
    grasp_selector.add_child(ActionNode("Primary Grasp", grasp_primary))
    grasp_selector.add_child(ActionNode("Alternative Grasp", grasp_alternative))
    
    root.add_child(grasp_selector)
    root.add_child(ActionNode("Transport", transport))
    root.add_child(ActionNode("Place", place))
    
    tree.set_root(root)
    
    # Run
    print("Running assembly behavior tree...")
    success = tree.run()
    print(f"Result: {'SUCCESS' if success else 'FAILURE'}")


if __name__ == "__main__":
    example_assembly_tree()
```

---

### 2.2 Manipulation-Specific BT Nodes (Week 5)

**File to Create:** `manipulation_behaviors.py`

**Purpose:** Pre-built BT nodes for common manipulation patterns.

```python
"""
Manipulation-specific behavior tree nodes.
"""

from behavior_tree import ActionNode, SequenceNode, SelectorNode, ParallelNode
from manipulation_bridge import ManipulationPlannerBridge


class GraspAction(ActionNode):
    """Action node for grasping."""
    
    def __init__(self, bridge: ManipulationPlannerBridge,
                 gripper_name: str, handle_name: str):
        def grasp_fn():
            # Use bridge to execute grasp planning
            return bridge.execute_grasp(gripper_name, handle_name)
        
        super().__init__(f"Grasp {handle_name}", grasp_fn)


class PlaceAction(ActionNode):
    """Action node for placing."""
    
    def __init__(self, bridge: ManipulationPlannerBridge,
                 object_name: str, target_pose: list):
        def place_fn():
            return bridge.execute_place(object_name, target_pose)
        
        super().__init__(f"Place {object_name}", place_fn)


class HandoverSequence(SequenceNode):
    """Sequence for arm-to-arm handover."""
    
    def __init__(self, bridge: ManipulationPlannerBridge,
                 arm1: str, arm2: str, object_name: str):
        super().__init__(f"Handover {object_name} from {arm1} to {arm2}")
        
        # arm1 moves to handover position
        self.add_child(ActionNode(
            f"{arm1} to handover pose",
            lambda: bridge.execute_move(arm1, "handover_pose")
        ))
        
        # arm2 approaches
        self.add_child(ActionNode(
            f"{arm2} approaches",
            lambda: bridge.execute_approach(arm2, object_name)
        ))
        
        # arm2 grasps
        self.add_child(ActionNode(
            f"{arm2} grasps",
            lambda: bridge.execute_grasp(arm2, object_name)
        ))
        
        # arm1 releases
        self.add_child(ActionNode(
            f"{arm1} releases",
            lambda: bridge.execute_release(arm1, object_name)
        ))


class AdaptiveGraspSelector(SelectorNode):
    """Tries multiple grasp strategies."""
    
    def __init__(self, bridge: ManipulationPlannerBridge,
                 object_name: str, grasp_strategies: list):
        super().__init__(f"Adaptive grasp {object_name}")
        
        for strategy in grasp_strategies:
            self.add_child(GraspAction(
                bridge, strategy["gripper"], strategy["handle"]
            ))
```

---

### 2.3 Integration with Task Orchestrator (Week 6)

**File to Enhance:** `task_orchestration.py`

**Add:**
```python
class BehaviorTreeOrchestrator(TaskOrchestrator):
    """
    Orchestrator that uses behavior trees for task execution.
    
    Combines resource management with reactive BT execution.
    """
    
    def __init__(self, behavior_tree: BehaviorTree, 
                 max_concurrent_tasks: int = 2):
        super().__init__(max_concurrent_tasks)
        self.behavior_tree = behavior_tree
        
    def run_with_behavior_tree(self) -> bool:
        """Execute using behavior tree control flow."""
        # Setup resources
        # ...
        
        # Run behavior tree
        return self.behavior_tree.run()
```

---

## Phase 3: Advanced Features (Weeks 7-10) 🔄 **MEDIUM PRIORITY**

### 3.1 Foliation-Aware Planning (Week 7)

**Problem:** Current implementation doesn't explicitly handle constraint foliations.

**Solution:** Extend `ConfigurationGenerator` to track leaf membership.

```python
class FoliationAwareConfigGenerator(ConfigurationGenerator):
    """
    Configuration generator that tracks foliation leaves.
    
    Ensures generated configs belong to correct leaves for reachability.
    """
    
    def __init__(self, robot, graph, ps, backend="corba"):
        super().__init__(robot, graph, ps, backend)
        self.leaf_cache: Dict[str, Any] = {}  # edge -> leaf identifier
        
    def generate_via_edge_with_leaf(
        self,
        edge_name: str,
        q_from: np.ndarray,
        config_label: str,
        leaf_identifier: Optional[Any] = None
    ) -> Tuple[bool, np.ndarray]:
        """
        Generate config ensuring it's in the same foliation leaf as q_from.
        
        Uses right-hand side of edge constraints to define leaf.
        """
        # Set RHS from q_from (defines leaf)
        if self.backend == "corba":
            edge = self.graph.edges[edge_name]
            # Set constraint RHS from q_from
            # This ensures generated config is in same leaf
        
        # Generate as usual
        return super().generate_via_edge(edge_name, q_from, config_label)
```

---

### 3.2 Multi-Arm Synchronization (Week 8)

**File to Create:** `synchronization.py`

```python
"""
Synchronization primitives for multi-arm coordination.
"""

class SynchronizationBarrier:
    """
    Barrier for synchronizing multiple arms.
    
    All arms must reach barrier before any proceed.
    """
    
    def __init__(self, num_arms: int):
        self.num_arms = num_arms
        self.waiting_arms: Set[str] = set()
        
    def arrive(self, arm_name: str) -> bool:
        """
        Arm arrives at barrier.
        
        Returns:
            True if all arms have arrived
        """
        self.waiting_arms.add(arm_name)
        return len(self.waiting_arms) >= self.num_arms
    
    def reset(self):
        """Reset barrier for reuse."""
        self.waiting_arms.clear()


class TemporalAlignment:
    """
    Ensures temporal alignment of collaborative actions.
    
    Example: Both arms must be at assembly position simultaneously.
    """
    
    def __init__(self, tolerance: float = 0.1):
        self.tolerance = tolerance  # seconds
        self.arm_arrival_times: Dict[str, float] = {}
        
    def register_arrival(self, arm_name: str):
        """Register arm arrival time."""
        import time
        self.arm_arrival_times[arm_name] = time.time()
        
    def is_synchronized(self) -> bool:
        """Check if all arms arrived within tolerance."""
        if len(self.arm_arrival_times) < 2:
            return False
        
        times = list(self.arm_arrival_times.values())
        return (max(times) - min(times)) <= self.tolerance
```

**Usage:**
```python
# In behavior tree
barrier = SynchronizationBarrier(num_arms=2)

parallel = ParallelNode("Prepare Assembly")
parallel.add_child(ActionNode("UR10 prepare", lambda: ur10_prep() and barrier.arrive("UR10")))
parallel.add_child(ActionNode("VISPA prepare", lambda: vispa_prep() and barrier.arrive("VISPA")))

sequence = SequenceNode("Assembly")
sequence.add_child(parallel)
sequence.add_child(ActionNode("Assemble", assemble_fn))
```

---

### 3.3 Failure Recovery Strategies (Week 9)

**File to Create:** `recovery_strategies.py`

```python
"""
Failure recovery strategies for manipulation tasks.
"""

class RecoveryStrategy(ABC):
    """Base class for recovery strategies."""
    
    @abstractmethod
    def recover(self, failed_task: AtomicTask, 
                context: dict) -> Optional[BehaviorNode]:
        """
        Generate recovery behavior tree.
        
        Returns:
            Recovery tree or None if unrecoverable
        """
        pass


class RetryWithAlternativeGrasp(RecoveryStrategy):
    """Try alternative grasp if primary fails."""
    
    def recover(self, failed_task, context):
        alternatives = context.get("alternative_grasps", [])
        
        if not alternatives:
            return None
        
        selector = SelectorNode("Try Alternative Grasps")
        for alt in alternatives:
            selector.add_child(GraspAction(
                context["bridge"], alt["gripper"], alt["handle"]
            ))
        
        return selector


class ReplanWithRelaxedConstraints(RecoveryStrategy):
    """Replan with relaxed constraints."""
    
    def recover(self, failed_task, context):
        # Modify constraint tolerances
        # Re-attempt planning
        pass


class SwitchArm(RecoveryStrategy):
    """Switch to different arm if current arm fails."""
    
    def recover(self, failed_task, context):
        alternative_arm = context.get("alternative_arm")
        if not alternative_arm:
            return None
        
        # Create new task with different arm
        # Return action node
        pass
```

---

### 3.4 Learning from Execution (Week 10)

**File to Create:** `execution_learning.py`

```python
"""
Learn from execution history to improve future planning.
"""

class ExecutionStatistics:
    """Track execution metrics per task type."""
    
    def __init__(self):
        self.task_durations: Dict[str, List[float]] = {}
        self.success_rates: Dict[str, float] = {}
        self.common_failures: Dict[str, List[str]] = {}
        
    def record_execution(self, task_id: str, duration: float, 
                        success: bool, error: Optional[str] = None):
        """Record task execution."""
        if task_id not in self.task_durations:
            self.task_durations[task_id] = []
        
        self.task_durations[task_id].append(duration)
        
        # Update success rate
        # ...
        
        if not success and error:
            if task_id not in self.common_failures:
                self.common_failures[task_id] = []
            self.common_failures[task_id].append(error)
    
    def predict_duration(self, task_id: str) -> float:
        """Predict task duration based on history."""
        if task_id not in self.task_durations:
            return 60.0  # Default
        
        return np.median(self.task_durations[task_id])
    
    def get_most_common_failure(self, task_id: str) -> Optional[str]:
        """Get most common failure mode."""
        if task_id not in self.common_failures:
            return None
        
        from collections import Counter
        counter = Counter(self.common_failures[task_id])
        return counter.most_common(1)[0][0]
```

---

## Phase 4: Optimization & Polish (Weeks 11-12) 🔧 **LOW PRIORITY**

### 4.1 Path Optimization Integration

**Enhance:** `manipulation_bridge.py`

```python
def optimize_path(self, task_id: str, 
                 method: str = "partial_shortcut") -> bool:
    """
    Optimize planned path using hpp-manipulation optimizers.
    
    Args:
        task_id: Task to optimize
        method: 'partial_shortcut', 'progressive', 'graph_optimizer'
    """
    context = self.contexts.get(task_id)
    if not context or not context.path:
        return False
    
    if self.task.backend == "corba":
        ps = self.task.ps
        
        # Add path to problem
        ps.addPath(context.path)
        
        # Optimize
        if method == "partial_shortcut":
            ps.optimizePath(0)  # Path index 0
        elif method == "progressive":
            ps.selectPathOptimizer("Progressive")
            ps.optimizePath(0)
        elif method == "graph_optimizer":
            from hpp.manipulation import GraphOptimizer
            optimizer = GraphOptimizer(ps)
            optimizer.optimize(context.path)
        
        # Get optimized path
        context.path = ps.getPath(1)  # Optimized path at index 1
        context.path_length = context.path.length()
        
        return True
    
    return False
```

---

### 4.2 Visualization Enhancements

**File to Create:** `execution_visualizer.py`

```python
"""
Real-time visualization of task execution.
"""

def visualize_execution_timeline(orchestrator: TaskOrchestrator,
                                output_file: str = "timeline.html"):
    """
    Generate Gantt chart of task execution.
    
    Shows:
    - Task start/end times
    - Parallel execution
    - Dependencies
    - Resource usage
    """
    import plotly.figure_factory as ff
    
    # Extract task execution data
    # Create Gantt chart
    # Save to HTML
    pass


def visualize_behavior_tree(tree: BehaviorTree,
                            output_file: str = "bt_graph.png"):
    """
    Visualize behavior tree structure.
    
    Uses graphviz to show:
    - Node hierarchy
    - Node types
    - Execution flow
    """
    import graphviz
    
    # Traverse tree
    # Create graph
    # Render
    pass
```

---

### 4.3 Testing Infrastructure

**Files to Create:**
- `tests/test_manipulation_bridge.py`
- `tests/test_behavior_tree.py`
- `tests/test_orchestration.py`

```python
"""
Unit tests for manipulation bridge.
"""

import pytest
from manipulation_bridge import ManipulationPlannerBridge

def test_grasp_task_creation():
    """Test creating grasp task with real planning."""
    # Setup mock manipulation task
    # Create bridge
    # Create atomic task
    # Verify execution function exists
    pass

def test_planning_context_caching():
    """Test planning contexts are cached correctly."""
    # Execute task
    # Verify context created
    # Execute again
    # Verify context reused
    pass

def test_path_extraction():
    """Test extracting paths from planning contexts."""
    # Execute task
    # Extract path
    # Verify path properties
    pass
```

---

## Summary: Critical Next Steps

### **Immediate Actions (This Week)**

1. ✅ **Test `manipulation_bridge.py`** 
   - Run with existing `task_grasp_frame_gripper.py`
   - Verify real planning works
   - Debug any integration issues

2. **Create Simple Example**
   ```bash
   cd script/spacelab
   python example_bridge_integration.py
   ```
   
   Should demonstrate:
   - Loading task
   - Creating bridge
   - Generating atomic tasks with real planning
   - Running orchestrator with real execution

3. **Document API Usage**
   - Add examples to README
   - Create tutorial notebook
   - Record demo video

### **Week 2-3 Priorities**

1. Implement `graph_patterns.py`
2. Add factory support to `GraphBuilder`
3. Test with multi-arm scenarios

### **Week 4-6 Priorities**

1. Implement behavior tree core
2. Create manipulation-specific nodes
3. Integrate with orchestrator

### **Metrics for Success**

- ✅ Real motion planning replaces mocks
- ✅ Multi-arm tasks execute in parallel
- ✅ Behavior trees enable dynamic composition
- ✅ Failure recovery works automatically
- ✅ Task code reduced by 60%
- ✅ Planning success rate > 90%

---

## Conclusion

The agimus_spacelab package has a **solid foundation** but needs critical integration with hpp-manipulation's core capabilities. The most important step is **bridging atomic tasks to real motion planning** (Phase 1.1, already implemented in `manipulation_bridge.py`).

Once this bridge is tested and working, the behavior tree integration (Phase 2) will enable truly **reactive, adaptive multi-arm manipulation** that goes beyond what current manipulation planning frameworks offer.

The combination of:
- hpp-manipulation's constraint graph planning
- Task orchestration's resource management  
- Behavior trees' reactive control

Will create a **world-class multi-arm manipulation planning system** suitable for complex assembly tasks.

**Next Command:**
```bash
cd /home/dvtnguyen/devel/hpp/src/agimus_spacelab/script/spacelab
python -c "from manipulation_bridge import ManipulationPlannerBridge; print('Bridge ready!')"
```

Let's proceed! 🚀
