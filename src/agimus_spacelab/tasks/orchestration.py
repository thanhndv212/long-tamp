#!/usr/bin/env python3
"""
Task Orchestration Framework for Multi-Arm Collaborative Manipulation.

This module provides infrastructure for:
- Task decomposition and dependency management
- Multi-arm resource allocation
- Synchronization points for collaborative actions
"""

from typing import List, Dict, Set, Optional, Tuple, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import time
from abc import ABC, abstractmethod


# ============================================================================
# Core Definitions
# ============================================================================

class TaskStatus(Enum):
    """Execution status of a task."""
    PENDING = "pending"
    READY = "ready"
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"


class ResourceType(Enum):
    """Types of resources that can be allocated."""
    ARM = "arm"
    OBJECT = "object"
    WORKSPACE = "workspace"
    GRIPPER = "gripper"


@dataclass
class Resource:
    """Represents a physical or logical resource."""
    resource_type: ResourceType
    name: str
    available: bool = True
    allocated_to: Optional[str] = None  # Task ID
    
    def __hash__(self):
        return hash((self.resource_type, self.name))


@dataclass
class TaskPrecondition:
    """Precondition that must be satisfied before task execution."""
    description: str
    check: Callable[[], bool]  # Function that returns True if satisfied
    
    def is_satisfied(self) -> bool:
        """Check if precondition is met."""
        try:
            return self.check()
        except Exception as e:
            print(f"Precondition check failed: {e}")
            return False


@dataclass
class TaskPostcondition:
    """Postcondition that should be true after successful execution."""
    description: str
    verify: Callable[[], bool]  # Function that returns True if verified
    
    def is_verified(self) -> bool:
        """Verify postcondition."""
        try:
            return self.verify()
        except Exception as e:
            print(f"Postcondition verification failed: {e}")
            return False


# ============================================================================
# Atomic Task Definition
# ============================================================================

@dataclass
class AtomicTask:
    """
    Indivisible unit of work in manipulation planning.
    
    Represents a single manipulation action that cannot be further decomposed.
    """
    task_id: str
    name: str
    description: str
    
    # Resource requirements
    required_resources: Set[Resource] = field(default_factory=set)
    
    # Dependencies
    depends_on: List[str] = field(default_factory=list)  # Task IDs
    enables: List[str] = field(default_factory=list)
    
    # Execution conditions
    preconditions: List[TaskPrecondition] = field(default_factory=list)
    postconditions: List[TaskPostcondition] = field(default_factory=list)
    
    # Execution function
    execute: Optional[Callable[[], bool]] = None  # Returns True on success
    
    # State
    status: TaskStatus = TaskStatus.PENDING
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    error_message: Optional[str] = None
    
    # Execution parameters
    max_retries: int = 3
    retry_count: int = 0
    timeout: float = 300.0  # 5 minutes default
    
    def can_execute(
        self, resource_manager: 'ResourceManager'
    ) -> Tuple[bool, str]:
        """Check if task can be executed now."""
        # Check preconditions
        for precond in self.preconditions:
            if not precond.is_satisfied():
                return False, f"Precondition not met: {precond.description}"
                
        # Check resource availability
        for resource in self.required_resources:
            if not resource_manager.is_available(resource):
                return False, f"Resource unavailable: {resource.name}"
                
        return True, "Ready to execute"
        
    def verify_completion(self) -> Tuple[bool, str]:
        """Verify task completed successfully."""
        for postcond in self.postconditions:
            if not postcond.is_verified():
                return False, f"Postcondition failed: {postcond.description}"
        return True, "Task verified"
        
    def get_duration(self) -> Optional[float]:
        """Get task execution duration."""
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return None


# ============================================================================
# Resource Manager
# ============================================================================

class ResourceManager:
    """Manages allocation and deallocation of resources."""
    
    def __init__(self):
        self.resources: Dict[str, Resource] = {}
        self.allocations: Dict[str, Set[Resource]] = {}  # task_id -> resources
        
    def register_resource(self, resource: Resource) -> None:
        """Register a new resource."""
        key = f"{resource.resource_type.value}:{resource.name}"
        self.resources[key] = resource
        
    def register_arm(self, name: str) -> None:
        """Register an arm as a resource."""
        self.register_resource(Resource(ResourceType.ARM, name))
        
    def register_object(self, name: str) -> None:
        """Register an object as a resource."""
        self.register_resource(Resource(ResourceType.OBJECT, name))
        
    def register_workspace(self, name: str) -> None:
        """Register a workspace zone as a resource."""
        self.register_resource(Resource(ResourceType.WORKSPACE, name))
        
    def is_available(self, resource: Resource) -> bool:
        """Check if resource is available."""
        key = f"{resource.resource_type.value}:{resource.name}"
        if key not in self.resources:
            return False
        return self.resources[key].available
        
    def allocate(self, task_id: str, resources: Set[Resource]) -> bool:
        """Allocate resources to a task."""
        # Check all available first (atomic check)
        for resource in resources:
            if not self.is_available(resource):
                return False
                
        # Allocate all
        for resource in resources:
            key = f"{resource.resource_type.value}:{resource.name}"
            self.resources[key].available = False
            self.resources[key].allocated_to = task_id
            
        self.allocations[task_id] = resources
        return True
        
    def deallocate(self, task_id: str) -> None:
        """Release all resources allocated to a task."""
        if task_id not in self.allocations:
            return
            
        for resource in self.allocations[task_id]:
            key = f"{resource.resource_type.value}:{resource.name}"
            if key in self.resources:
                self.resources[key].available = True
                self.resources[key].allocated_to = None
                
        del self.allocations[task_id]
        
    def get_allocated_resources(self, task_id: str) -> Set[Resource]:
        """Get resources allocated to a task."""
        return self.allocations.get(task_id, set())
        
    def get_allocation_summary(self) -> Dict[str, Any]:
        """Get summary of current allocations."""
        return {
            "total_resources": len(self.resources),
            "available": sum(
                1 for r in self.resources.values() if r.available
            ),
            "allocated": sum(
                1 for r in self.resources.values() if not r.available
            ),
            "allocations_by_task": {
                task_id: [
                    f"{r.resource_type.value}:{r.name}" for r in resources
                ]
                for task_id, resources in self.allocations.items()
            }
        }


# ============================================================================
# Task Dependency Graph
# ============================================================================

class TaskDependencyGraph:
    """Manages dependencies between tasks and determines execution order."""
    
    def __init__(self):
        self.tasks: Dict[str, AtomicTask] = {}
        self.dependencies: Dict[str, Set[str]] = {}  # task_id -> prerequisites
        
    def add_task(self, task: AtomicTask) -> None:
        """Add a task to the graph."""
        self.tasks[task.task_id] = task
        
        # Build dependency edges
        if task.task_id not in self.dependencies:
            self.dependencies[task.task_id] = set()
            
        for dep_id in task.depends_on:
            self.dependencies[task.task_id].add(dep_id)
            
    def get_ready_tasks(self) -> List[AtomicTask]:
        """Get tasks that are ready to execute."""
        ready = []
        
        for task in self.tasks.values():
            if task.status != TaskStatus.PENDING:
                continue
                
            # Check all dependencies completed
            deps_satisfied = True
            for dep_id in task.depends_on:
                if dep_id not in self.tasks:
                    deps_satisfied = False
                    break
                    
                dep_task = self.tasks[dep_id]
                if dep_task.status != TaskStatus.SUCCESS:
                    deps_satisfied = False
                    break
                    
            if deps_satisfied:
                ready.append(task)
                
        return ready
        
    def get_concurrent_tasks(
        self,
        ready_tasks: List[AtomicTask],
        resource_manager: ResourceManager
    ) -> List[AtomicTask]:
        """From ready tasks, determine which can run concurrently."""
        if not ready_tasks:
            return []
            
        concurrent = []
        reserved_resources = set()
        
        for task in ready_tasks:
            # Check resource conflicts
            conflict = False
            for resource in task.required_resources:
                if resource in reserved_resources:
                    conflict = True
                    break
                    
            if not conflict:
                concurrent.append(task)
                reserved_resources.update(task.required_resources)
                
        return concurrent
        
    def has_circular_dependency(self) -> bool:
        """Check for circular dependencies."""
        def visit(task_id: str, visited: Set[str], path: Set[str]) -> bool:
            if task_id in path:
                return True  # Cycle detected
                
            if task_id in visited:
                return False
                
            visited.add(task_id)
            path.add(task_id)
            
            for dep_id in self.dependencies.get(task_id, []):
                if visit(dep_id, visited, path):
                    return True
                    
            path.remove(task_id)
            return False
            
        visited = set()
        for task_id in self.tasks.keys():
            if visit(task_id, visited, set()):
                return True
                
        return False
        
    def get_execution_summary(self) -> Dict[str, Any]:
        """Get summary of task states."""
        summary = {
            "total": len(self.tasks),
            "by_status": {},
            "completed": 0,
            "failed": 0,
            "pending": 0,
        }
        
        for task in self.tasks.values():
            status_key = task.status.value
            summary["by_status"][status_key] = (
                summary["by_status"].get(status_key, 0) + 1
            )
            
            if task.status == TaskStatus.SUCCESS:
                summary["completed"] += 1
            elif task.status == TaskStatus.FAILURE:
                summary["failed"] += 1
            elif task.status == TaskStatus.PENDING:
                summary["pending"] += 1
                
        return summary


# ============================================================================
# Task Orchestrator
# ============================================================================

class TaskOrchestrator:
    """Orchestrates execution of multiple tasks with dependencies."""
    
    def __init__(self, max_concurrent_tasks: int = 2):
        self.dependency_graph = TaskDependencyGraph()
        self.resource_manager = ResourceManager()
        self.max_concurrent = max_concurrent_tasks
        self.running_tasks: Dict[str, AtomicTask] = {}
        
    def add_task(self, task: AtomicTask) -> None:
        """Add a task to orchestrator."""
        self.dependency_graph.add_task(task)
        
    def setup_resources(
        self,
        arms: List[str],
        objects: List[str],
        workspaces: List[str] = None
    ) -> None:
        """Register all available resources."""
        for arm in arms:
            self.resource_manager.register_arm(arm)
            
        for obj in objects:
            self.resource_manager.register_object(obj)
            
        if workspaces:
            for ws in workspaces:
                self.resource_manager.register_workspace(ws)
                
    def can_execute_task(self, task: AtomicTask) -> Tuple[bool, str]:
        """Check if task can be executed now."""
        can_exec, reason = task.can_execute(self.resource_manager)
        if not can_exec:
            return False, reason
            
        if len(self.running_tasks) >= self.max_concurrent:
            return False, "Max concurrent tasks reached"
            
        return True, "OK"
        
    def execute_task(self, task: AtomicTask) -> bool:
        """Execute a single task."""
        task.status = TaskStatus.RUNNING
        task.start_time = time.time()
        self.running_tasks[task.task_id] = task
        
        # Allocate resources
        if not self.resource_manager.allocate(
            task.task_id, task.required_resources
        ):
            task.status = TaskStatus.BLOCKED
            task.error_message = "Failed to allocate resources"
            del self.running_tasks[task.task_id]
            return False
            
        print(f"[ORCHESTRATOR] Executing: {task.name} (ID: {task.task_id})")
        
        try:
            # Execute task
            if task.execute:
                success = task.execute()
            else:
                print(f"  ⚠ No execution function for {task.task_id}")
                success = False
                
            task.end_time = time.time()
            
            if success:
                # Verify postconditions
                verified, msg = task.verify_completion()
                if verified:
                    task.status = TaskStatus.SUCCESS
                    print(f"  ✓ {task.name} completed "
                          f"({task.get_duration():.2f}s)")
                else:
                    task.status = TaskStatus.FAILURE
                    task.error_message = f"Verification failed: {msg}"
                    print(f"  ✗ {task.name} verification failed: {msg}")
            else:
                task.status = TaskStatus.FAILURE
                task.error_message = "Execution returned False"
                print(f"  ✗ {task.name} execution failed")
                
        except Exception as e:
            task.status = TaskStatus.FAILURE
            task.error_message = str(e)
            task.end_time = time.time()
            print(f"  ✗ {task.name} exception: {e}")
            
        finally:
            # Release resources
            self.resource_manager.deallocate(task.task_id)
            if task.task_id in self.running_tasks:
                del self.running_tasks[task.task_id]
                
        return task.status == TaskStatus.SUCCESS
        
    def run(self, dry_run: bool = False) -> bool:
        """Execute all tasks respecting dependencies."""
        print("\n" + "=" * 70)
        print("TASK ORCHESTRATION")
        print("=" * 70)
        
        # Validate graph
        if self.dependency_graph.has_circular_dependency():
            print("✗ ERROR: Circular dependency detected!")
            return False
            
        print(f"\nTotal tasks: {len(self.dependency_graph.tasks)}")
        print(f"Max concurrent: {self.max_concurrent}")
        
        if dry_run:
            print("\n[DRY RUN MODE - No execution]")
            self._show_execution_plan()
            return True
            
        # Execute tasks
        print("\n" + "-" * 70)
        print("EXECUTION")
        print("-" * 70)
        
        iteration = 0
        max_iterations = len(self.dependency_graph.tasks) * 10
        
        while iteration < max_iterations:
            iteration += 1
            
            ready_tasks = self.dependency_graph.get_ready_tasks()
            
            if not ready_tasks and not self.running_tasks:
                break
                
            concurrent = self.dependency_graph.get_concurrent_tasks(
                ready_tasks, self.resource_manager
            )
            
            for task in concurrent:
                can_exec, reason = self.can_execute_task(task)
                if can_exec:
                    self.execute_task(task)
                else:
                    task.status = TaskStatus.BLOCKED
                    print(f"  ⚠ {task.name} blocked: {reason}")
                    
            if not concurrent and self.running_tasks:
                time.sleep(0.1)
                
        # Summary
        print("\n" + "-" * 70)
        print("SUMMARY")
        print("-" * 70)
        
        summary = self.dependency_graph.get_execution_summary()
        print(f"Total tasks: {summary['total']}")
        print(f"  ✓ Completed: {summary['completed']}")
        print(f"  ✗ Failed: {summary['failed']}")
        print(f"  ⧖ Pending: {summary['pending']}")
        
        success = (summary['failed'] == 0 and summary['pending'] == 0)
        
        if success:
            print("\n✓ All tasks completed successfully!")
        else:
            print("\n✗ Some tasks failed or incomplete")
            
        return success
        
    def _show_execution_plan(self) -> None:
        """Display planned execution order."""
        print("\nExecution Plan:")
        
        level = 0
        executed = set()
        
        while len(executed) < len(self.dependency_graph.tasks):
            level_tasks = []
            
            for task_id, task in self.dependency_graph.tasks.items():
                if task_id in executed:
                    continue
                    
                deps_done = all(dep in executed for dep in task.depends_on)
                if deps_done:
                    level_tasks.append(task)
                    
            if not level_tasks:
                break
                
            print(f"\n  Level {level}:")
            for task in level_tasks:
                resources_str = ", ".join(
                    f"{r.resource_type.value}:{r.name}"
                    for r in task.required_resources
                )
                print(f"    - {task.name}")
                print(f"      Resources: {resources_str}")
                executed.add(task.task_id)
                
            level += 1


# ============================================================================
# Task Builder
# ============================================================================

class TaskBuilder:
    """Fluent builder for creating tasks."""
    
    def __init__(self, task_id: str, name: str):
        self.task = AtomicTask(task_id=task_id, name=name, description="")
        
    def with_description(self, desc: str) -> 'TaskBuilder':
        self.task.description = desc
        return self
        
    def requires_arm(self, arm_name: str) -> 'TaskBuilder':
        self.task.required_resources.add(Resource(ResourceType.ARM, arm_name))
        return self
        
    def requires_object(self, obj_name: str) -> 'TaskBuilder':
        self.task.required_resources.add(
            Resource(ResourceType.OBJECT, obj_name)
        )
        return self
        
    def requires_workspace(self, ws_name: str) -> 'TaskBuilder':
        self.task.required_resources.add(
            Resource(ResourceType.WORKSPACE, ws_name)
        )
        return self
        
    def depends_on(self, *task_ids: str) -> 'TaskBuilder':
        self.task.depends_on.extend(task_ids)
        return self
        
    def with_precondition(
        self, desc: str, check: Callable[[], bool]
    ) -> 'TaskBuilder':
        self.task.preconditions.append(TaskPrecondition(desc, check))
        return self
        
    def with_postcondition(
        self, desc: str, verify: Callable[[], bool]
    ) -> 'TaskBuilder':
        self.task.postconditions.append(TaskPostcondition(desc, verify))
        return self
        
    def with_execution(self, func: Callable[[], bool]) -> 'TaskBuilder':
        self.task.execute = func
        return self
        
    def with_timeout(self, seconds: float) -> 'TaskBuilder':
        self.task.timeout = seconds
        return self
        
    def build(self) -> AtomicTask:
        return self.task


__all__ = [
    'TaskStatus',
    'ResourceType',
    'Resource',
    'TaskPrecondition',
    'TaskPostcondition',
    'AtomicTask',
    'ResourceManager',
    'TaskDependencyGraph',
    'TaskOrchestrator',
    'TaskBuilder',
]
