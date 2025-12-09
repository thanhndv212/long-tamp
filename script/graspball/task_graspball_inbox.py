#!/usr/bin/env python3
"""
Task: UR5 grasps pokeball from box.

This task demonstrates a complete pick-and-place sequence:
1. Ball rests on ground (placement)
2. Gripper approaches ball (gripper-above-ball)
3. Gripper grasps ball on ground (grasp-placement)
4. Ball is lifted from ground (ball-above-ground)
5. Free motion with grasped ball (grasp)

Task sequence:
1. placement -> approach-ball -> gripper-above-ball
2. gripper-above-ball -> grasp-ball -> grasp-placement
3. grasp-placement -> take-ball-up -> ball-above-ground
4. ball-above-ground -> take-ball-away -> grasp
5. grasp -> approach-ground -> ball-above-ground
6. ball-above-ground -> put-ball-down -> grasp-placement
7. grasp-placement -> move-gripper-up -> gripper-above-ball
8. gripper-above-ball -> move-gripper-away -> placement

Supports both CORBA and PyHPP backends.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Dict
import numpy as np

from agimus_spacelab.tasks import ManipulationTask
from agimus_spacelab.planning import ConstraintBuilder
from agimus_spacelab.visualization import (
    print_joint_info,
    visualize_constraint_graph,
    visualize_all_handles,
    visualize_all_grippers,
)
# Add config directory
config_dir = Path(__file__).parent.parent / "config"
sys.path.insert(0, str(config_dir))

from graspball_config import (
    PATHS,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
)

# Import backend availability flags for validation
try:
    from agimus_spacelab.backends import get_available_backends
    _available = get_available_backends()
    HAS_CORBA = 'corba' in _available
    HAS_PYHPP = 'pyhpp' in _available
except ImportError:
    HAS_CORBA = False
    HAS_PYHPP = False

# CORBA-specific imports for manual graph building
try:
    from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
    from hpp.corbaserver import Client
except ImportError:
    ConstraintGraph = None
    Constraints = None
    Client = None

# PyHPP-specific imports for manual graph building
try:
    from pyhpp.manipulation import Graph
    from pyhpp.constraints import (
        RelativeTransformation,
        Transformation,
        ComparisonTypes,
        ComparisonType,
        Implicit,
    )
    from pinocchio import SE3, Quaternion, StdVec_Bool as Mask
except ImportError:
    Graph = None
    RelativeTransformation = None
    Transformation = None
    ComparisonTypes = None
    ComparisonType = None
    Implicit = None
    SE3 = None
    Quaternion = None
    Mask = None


# ============================================================================
# Task Implementation
# ============================================================================

class GraspBallTask(ManipulationTask):
    """UR5 grasps pokeball from box."""
    
    def __init__(self, backend: str = "corba"):
        """
        Initialize task.
        
        Args:
            backend: "corba" or "pyhpp" - which backend to use
        """
        super().__init__(
            joint_bounds=JointBounds,
            FILE_PATHS=PATHS,
            task_name="Grasp Ball Manipulation",
            backend=backend
        )
        self.config = ManipulationConfig
        self.pyhpp_constraints = {}
        self.pyhpp_states = None
        self.pyhpp_edges = None
    
        self.robot_names = ["ur5"]
        self.composite_names = ["ur5-pokeball"]
        self.object_names = ["pokeball"]
        self.environment_names = ["ground", "box"]
    def get_joint_groups(self) -> List[str]:
        """Return joint groups from configuration."""
        return ["UR5"]  # Only UR5 robot joints

    def get_objects(self) -> List[str]:
        """Return list of objects from configuration."""
        return ["pokeball"]
    
    def build_initial_config(self) -> List[float]:
        """Build the initial configuration for UR5 + pokeball."""
        return InitialConfigurations.FULL_INIT
    
    def setup_scene(self, validation_step: float, projector_step: float):
        """
        Custom scene setup for graspball (UR5 robot, not SpaceLab).
        
        This overrides the default scene builder since we use different
        robot/environment URDFs.
        """
        print("=" * 70)
        print(f"{self.task_name} - {self.backend.upper()} Backend")
        print("=" * 70)
        
        cfg = self.config
        
        if self.backend == "corba":
            # Reset problem first
            if Client is not None:
                try:
                    Client().problem.resetProblem()
                except Exception:
                    pass
            
            from agimus_spacelab.backends import CorbaBackend
            self.planner = CorbaBackend()
            
            # Load robot
            print("\n1. Loading robot and objects...")
            self.robot = self.planner.load_robot(
                composite_name=[
                    "ur5-pokeball",
                ],
                robot_name=["ur5"],
                urdf_path=cfg.ROBOT_URDF,
                srdf_path=cfg.ROBOT_SRDF
            )
            
            # Load ball
            self.planner.load_object(
                name="pokeball",
                urdf_path=cfg.BALL_URDF,
                root_joint_type="freeflyer"
            )
            
            # Set joint bounds for freeflyer
            bounds = JointBounds.freeflyer_bounds()
            self.planner.set_joint_bounds(cfg.BALL_NAME, bounds)
            
            # Load environment
            self.planner.load_environment("ground", cfg.GROUND_URDF)
            self.planner.load_environment("box", cfg.BOX_URDF)
            
            # Get problem solver
            self.ps = self.planner.get_problem()
            
            # Position box walls
            box_x = cfg.BOX_X
            box_off = cfg.BOX_OFFSET
            self.ps.moveObstacle(
                'box/base_link_0', [box_x + box_off, 0, 0.04, 0, 0, 0, 1]
            )
            self.ps.moveObstacle(
                'box/base_link_1', [box_x - box_off, 0, 0.04, 0, 0, 0, 1]
            )
            self.ps.moveObstacle(
                'box/base_link_2', [box_x, box_off, 0.04, 0, 0, 0, 1]
            )
            self.ps.moveObstacle(
                'box/base_link_3', [box_x, -box_off, 0.04, 0, 0, 0, 1]
            )
            
            # Configure path planning
            self.ps.selectPathValidation("Discretized", validation_step)
            self.ps.selectPathProjector("Progressive", projector_step)
            
        else:  # pyhpp
            from agimus_spacelab.backends import PyHPPBackend
            self.planner = PyHPPBackend()
            
            print("\n1. Loading robot and objects...")
            
            # Load robot
            self.robot = self.planner.load_robot(
                robot_name="ur5",
                urdf_path=cfg.ROBOT_URDF,
                srdf_path=cfg.ROBOT_SRDF,
                root_joint_type="anchor"
            )
            
            # Load ball
            self.planner.load_object(
                name="pokeball",
                urdf_path=cfg.BALL_URDF,
                root_joint_type="freeflyer"
            )
            
            # Set joint bounds
            bounds = JointBounds.freeflyer_bounds()
            self.planner.set_joint_bounds(cfg.BALL_NAME, bounds)
            
            # Load environment
            self.planner.load_environment("ground", cfg.GROUND_URDF)
            
            # Load box at correct position
            box_pose = SE3(
                rotation=np.eye(3),
                translation=np.array([cfg.BOX_X, 0, 0.04])
            )
            self.planner.load_environment("box", cfg.BOX_URDF, pose=box_pose)
            
            self.ps = self.planner.get_problem()
            
            # Configure path validation
            from pyhpp.manipulation import createProgressiveProjector
            from pyhpp.core import createDiscretized
            
            device = self.robot.asPinDevice()
            problem = self.ps
            
            # Set path projector
            problem.pathProjector = createProgressiveProjector(
                problem.distance(), problem.steeringMethod(),
                projector_step
            )
            
            # Set path validation
            problem.pathValidation = createDiscretized(
                device, validation_step
            )
        
        print("   ✓ Robot and objects loaded")
    
    def setup_collision_management(self) -> None:
        """No special collision management needed for this task."""
        pass
    
    def create_constraints(self) -> None:
        """Create all transformation constraints."""
        cfg = self.config
        
        if self.backend == "pyhpp":
            self.pyhpp_constraints = self._create_pyhpp_constraints()
            return
        
        # CORBA constraints using ConstraintBuilder
        cb = ConstraintBuilder
        
        # 1. Grasp: gripper holds ball
        cb.create_grasp_constraint(
            self.ps, "grasp",
            cfg.GRIPPER_NAME, cfg.BALL_NAME,
            cfg.BALL_IN_GRIPPER, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 2. Placement: ball on ground
        cb.create_placement_constraint(
            self.ps, "placement",
            cfg.BALL_NAME, cfg.BALL_ON_GROUND,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 3. Placement complement: X, Y, yaw free
        cb.create_complement_constraint(
            self.ps, "placement",
            cfg.BALL_NAME, cfg.BALL_ON_GROUND,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 4. Gripper-ball aligned: gripper above ball
        cb.create_grasp_constraint(
            self.ps, "gripper_ball_aligned",
            cfg.GRIPPER_NAME, cfg.BALL_NAME,
            cfg.GRIPPER_ABOVE_BALL, cfg.GRASP_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 5. Ball near table constraint
        cb.create_placement_constraint(
            self.ps, "ball_near_table",
            cfg.BALL_NAME, cfg.BALL_NEAR_TABLE,
            cfg.PLACEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        # 6. Ball near table complement
        ball_near_table_comp = [cfg.BOX_X, 0.2, 0.1, 0, 0, 0, 1]
        cb.create_complement_constraint(
            self.ps, "ball_near_table",
            cfg.BALL_NAME, ball_near_table_comp,
            cfg.PLACEMENT_COMPLEMENT_MASK,
            robot=self.robot, backend=self.backend
        )
        
        print("   ✓ Created transformation constraints")
        
    def _create_pyhpp_constraints(self) -> Dict:
        """Create PyHPP-specific constraints."""
        cfg = self.config
        constraints = {}
        
        robot = self.planner.get_robot()
        
        # Get joint IDs
        joint_gripper = robot.model().getJointId(cfg.GRIPPER_NAME)
        joint_ball = robot.model().getJointId(cfg.BALL_NAME)
        Id = SE3.Identity()
        
        # 1. GRASP: gripper/ball fixed (all 6 DOF)
        q_grasp = Quaternion(0.5, 0.5, -0.5, 0.5)
        ball_in_gripper = SE3(q_grasp, np.array([0, 0.137, 0]))
        mask_full = Mask()
        mask_full[:] = (True,) * 6
        
        pc_grasp = RelativeTransformation.create(
            'grasp', robot.asPinDevice(),
            joint_gripper, joint_ball,
            ball_in_gripper, Id, mask_full
        )
        cts_grasp = ComparisonTypes()
        cts_grasp[:] = tuple([ComparisonType.EqualToZero] * 6)
        constraints['grasp'] = Implicit.create(pc_grasp, cts_grasp, mask_full)
        print("    ✓ grasp constraint")
        
        # 2. PLACEMENT: world/ball - fixed z, r, p
        q_placement = Quaternion(0, 0, 0, 1)
        ball_on_ground = SE3(
            q_placement, np.array([0, 0, cfg.BALL_RADIUS])
        )
        mask_placement = [False, False, True, True, True, False]
        
        pc_placement = Transformation.create(
            'placement', robot.asPinDevice(),
            joint_ball, Id, ball_on_ground, mask_placement
        )
        cts_placement = ComparisonTypes()
        cts_placement[:] = tuple([ComparisonType.EqualToZero] * 3)
        implicit_mask_placement = [True, True, True]
        constraints['placement'] = Implicit.create(
            pc_placement, cts_placement, implicit_mask_placement
        )
        print("    ✓ placement constraint")
        
        # 3. PLACEMENT/COMPLEMENT: world/ball - fixed x, y, yaw
        mask_placement_comp = [True, True, False, False, False, True]
        pc_placement_comp = Transformation.create(
            'placement/complement', robot.asPinDevice(),
            joint_ball, Id, ball_on_ground, mask_placement_comp
        )
        cts_placement_comp = ComparisonTypes()
        cts_placement_comp[:] = tuple([ComparisonType.Equality] * 3)
        implicit_mask_comp = [True, True, True]
        constraints['placement/complement'] = Implicit.create(
            pc_placement_comp, cts_placement_comp, implicit_mask_comp
        )
        print("    ✓ placement/complement constraint")
        
        # 4. GRIPPER_BALL_ALIGNED: gripper/ball - all fixed
        q_aligned = Quaternion(0.5, 0.5, -0.5, 0.5)
        gripper_above_ball = SE3(q_aligned, np.array([0, 0.2, 0]))
        mask_aligned = Mask()
        mask_aligned[:] = (True,) * 6
        
        pc_aligned = RelativeTransformation.create(
            'gripper_ball_aligned', robot.asPinDevice(),
            joint_gripper, joint_ball,
            gripper_above_ball, Id, mask_aligned
        )
        cts_aligned = ComparisonTypes()
        cts_aligned[:] = tuple([ComparisonType.EqualToZero] * 6)
        constraints['gripper_ball_aligned'] = Implicit.create(
            pc_aligned, cts_aligned, mask_aligned
        )
        print("    ✓ gripper_ball_aligned constraint")
        
        # 5. BALL_NEAR_TABLE: world/ball - fixed z, r, p
        q_table = Quaternion(0, 0, 0, 1)
        ball_near_table = SE3(q_table, np.array([cfg.BOX_X, 0, 0.1]))
        mask_table = [False, False, True, True, True, False]
        
        pc_table = Transformation.create(
            'ball_near_table', robot.asPinDevice(),
            joint_ball, Id, ball_near_table, mask_table
        )
        cts_table = ComparisonTypes()
        cts_table[:] = tuple([ComparisonType.EqualToZero] * 3)
        implicit_mask_table = [True, True, True]
        constraints['ball_near_table'] = Implicit.create(
            pc_table, cts_table, implicit_mask_table
        )
        print("    ✓ ball_near_table constraint")
        
        # 6. BALL_NEAR_TABLE/COMPLEMENT: world/ball - fixed x, y, yaw
        ball_near_table_comp = SE3(q_table, np.array([cfg.BOX_X, 0.2, 0.1]))
        mask_table_comp = [True, True, False, False, False, True]
        
        pc_table_comp = Transformation.create(
            'ball_near_table/complement', robot.asPinDevice(),
            joint_ball, Id, ball_near_table_comp, mask_table_comp
        )
        cts_table_comp = ComparisonTypes()
        cts_table_comp[:] = tuple([ComparisonType.Equality] * 3)
        implicit_mask_table_comp = [True, True, True]
        constraints['ball_near_table/complement'] = Implicit.create(
            pc_table_comp, cts_table_comp, implicit_mask_table_comp
        )
        print("    ✓ ball_near_table/complement constraint")
        
        return constraints
        
    def create_graph(self):
        """Create and configure constraint graph."""
        if self.backend == "corba":
            return self._create_corba_graph()
        else:
            return self._create_pyhpp_graph()
    
    def _create_corba_graph(self):
        """Create graph for CORBA backend."""
        print("    Building constraint graph...")
        cfg = self.config
        
        graph = ConstraintGraph(self.robot, "graph")
        
        # Create nodes (order matters for solver performance)
        graph.createNode(cfg.GRAPH_NODES)
        print(f"    ✓ Created {len(cfg.GRAPH_NODES)} states")
        
        # Create edges
        self._create_corba_edges(graph)
        print("    ✓ Created transitions")
        
        # Assign node constraints
        self._assign_corba_node_constraints(graph)
        print("    ✓ Added constraints to nodes")
        
        # Assign edge constraints
        self._assign_corba_edge_constraints(graph)
        print("    ✓ Added constraints to edges")
        
        # Set constant RHS
        self.ps.setConstantRightHandSide("placement", True)
        self.ps.setConstantRightHandSide("placement/complement", False)
        self.ps.setConstantRightHandSide("ball_near_table/complement", False)
        print("    ✓ Set constant right-hand side")
        
        # Initialize graph
        graph.initialize()
        print("    ✓ Graph initialized")
        
        return graph
    
    def _create_corba_edges(self, graph):
        """Create all state transitions for CORBA."""
        # Self-loops
        graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
        graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')
        
        # From placement
        graph.createEdge(
            'placement', 'gripper-above-ball', 'approach-ball', 1, 'placement'
        )
        
        # From gripper-above-ball
        graph.createEdge(
            'gripper-above-ball', 'placement', 'move-gripper-away', 1, 'placement'
        )
        graph.createEdge(
            'gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'placement'
        )
        
        # From grasp-placement
        graph.createEdge(
            'grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'placement'
        )
        graph.createEdge(
            'grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp'
        )
        
        # From ball-above-ground
        graph.createEdge(
            'ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'grasp'
        )
        graph.createEdge(
            'ball-above-ground', 'grasp', 'take-ball-away', 1, 'grasp'
        )
        
        # From grasp
        graph.createEdge(
            'grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp'
        )
    
    def _assign_corba_node_constraints(self, graph):
        """Assign constraints to states for CORBA."""
        graph.addConstraints(
            node="placement",
            constraints=Constraints(numConstraints=["placement"])
        )
        graph.addConstraints(
            node="gripper-above-ball",
            constraints=Constraints(
                numConstraints=["placement", "gripper_ball_aligned"]
            )
        )
        graph.addConstraints(
            node="grasp-placement",
            constraints=Constraints(numConstraints=["grasp", "placement"])
        )
        graph.addConstraints(
            node="ball-above-ground",
            constraints=Constraints(numConstraints=["grasp", "ball_near_table"])
        )
        graph.addConstraints(
            node="grasp",
            constraints=Constraints(numConstraints=["grasp"])
        )
    
    def _assign_corba_edge_constraints(self, graph):
        """Assign path constraints to edges for CORBA."""
        # Edges with placement/complement
        for edge in ["transit", "approach-ball", "move-gripper-away"]:
            graph.addConstraints(
                edge=edge,
                constraints=Constraints(numConstraints=["placement/complement"])
            )
        
        # Grasp-ball and move-gripper-up
        for edge in ["grasp-ball", "move-gripper-up"]:
            graph.addConstraints(
                edge=edge,
                constraints=Constraints(numConstraints=["placement/complement"])
            )
        
        # Take-ball-up and put-ball-down
        for edge in ["take-ball-up", "put-ball-down"]:
            graph.addConstraints(
                edge=edge,
                constraints=Constraints(
                    numConstraints=["ball_near_table/complement"]
                )
            )
        
        # Free motion edges
        for edge in ["transfer", "take-ball-away", "approach-ground"]:
            graph.addConstraints(edge=edge, constraints=Constraints())
    
    def _create_pyhpp_graph(self):
        """Create graph for PyHPP backend."""
        print("    Building constraint graph...")
        
        robot = self.planner.get_robot()
        problem = self.planner.get_problem()
        constraints = self.pyhpp_constraints
        
        graph = Graph("manipulation_graph", robot, problem)
        
        # Create states (order matters!)
        states = {}
        states['grasp'] = graph.createState("grasp", False, 0)
        states['ball-above-ground'] = graph.createState(
            "ball-above-ground", False, 0
        )
        states['grasp-placement'] = graph.createState(
            "grasp-placement", False, 0
        )
        states['gripper-above-ball'] = graph.createState(
            "gripper-above-ball", False, 0
        )
        states['placement'] = graph.createState("placement", False, 0)
        print("    ✓ Created 5 states")
        
        # Create edges
        edges = {}
        
        # Self-loops
        edges['transit'] = graph.createTransition(
            states['placement'], states['placement'],
            "transit", 1, states['placement']
        )
        edges['transfer'] = graph.createTransition(
            states['grasp'], states['grasp'],
            "transfer", 1, states['grasp']
        )
        
        # From placement
        edges['approach-ball'] = graph.createTransition(
            states['placement'], states['gripper-above-ball'],
            "approach-ball", 1, states['placement']
        )
        
        # From gripper-above-ball
        edges['move-gripper-away'] = graph.createTransition(
            states['gripper-above-ball'], states['placement'],
            "move-gripper-away", 1, states['placement']
        )
        edges['grasp-ball'] = graph.createTransition(
            states['gripper-above-ball'], states['grasp-placement'],
            "grasp-ball", 1, states['placement']
        )
        
        # From grasp-placement
        edges['move-gripper-up'] = graph.createTransition(
            states['grasp-placement'], states['gripper-above-ball'],
            "move-gripper-up", 1, states['placement']
        )
        edges['take-ball-up'] = graph.createTransition(
            states['grasp-placement'], states['ball-above-ground'],
            "take-ball-up", 1, states['grasp']
        )
        
        # From ball-above-ground
        edges['put-ball-down'] = graph.createTransition(
            states['ball-above-ground'], states['grasp-placement'],
            "put-ball-down", 1, states['grasp']
        )
        edges['take-ball-away'] = graph.createTransition(
            states['ball-above-ground'], states['grasp'],
            "take-ball-away", 1, states['grasp']
        )
        
        # From grasp
        edges['approach-ground'] = graph.createTransition(
            states['grasp'], states['ball-above-ground'],
            "approach-ground", 1, states['grasp']
        )
        print("    ✓ Created transitions")
        
        # Add constraints to states
        graph.addNumericalConstraint(
            states['placement'], constraints['placement']
        )
        graph.addNumericalConstraint(
            states['gripper-above-ball'], constraints['placement']
        )
        graph.addNumericalConstraint(
            states['gripper-above-ball'], constraints['gripper_ball_aligned']
        )
        graph.addNumericalConstraint(
            states['grasp-placement'], constraints['grasp']
        )
        graph.addNumericalConstraint(
            states['grasp-placement'], constraints['placement']
        )
        graph.addNumericalConstraint(
            states['ball-above-ground'], constraints['grasp']
        )
        graph.addNumericalConstraint(
            states['ball-above-ground'], constraints['ball_near_table']
        )
        graph.addNumericalConstraint(states['grasp'], constraints['grasp'])
        print("    ✓ Added constraints to states")
        
        # Add constraints to edges
        for edge in ['transit', 'approach-ball', 'move-gripper-away']:
            graph.addNumericalConstraintsToTransition(
                edges[edge], [constraints['placement/complement']]
            )
        for edge in ['grasp-ball', 'move-gripper-up']:
            graph.addNumericalConstraintsToTransition(
                edges[edge], [constraints['placement/complement']]
            )
        for edge in ['take-ball-up', 'put-ball-down']:
            graph.addNumericalConstraintsToTransition(
                edges[edge], [constraints['ball_near_table/complement']]
            )
        print("    ✓ Added constraints to edges")
        
        # Configure graph
        graph.maxIterations(100)
        graph.errorThreshold(0.00001)
        
        # Initialize graph
        graph.initialize()
        print("    ✓ Graph initialized")
        
        # Store for later use
        problem.constraintGraph(graph)
        self.pyhpp_states = states
        self.pyhpp_edges = edges
        
        return graph
        
    def generate_configurations(
        self, q_init: List[float]
    ) -> Dict[str, List[float]]:
        """Generate all waypoint configurations."""
        if self.backend == "pyhpp":
            return self._generate_configs_pyhpp(q_init)
        else:
            return self._generate_configs_corba(q_init)
    
    def _generate_configs_corba(self, q_init: List[float]) -> Dict:
        """Generate configurations for CORBA backend."""
        cfg = self.config
        configs = {}
        q1 = list(q_init)
        
        # 1. Project initial config on placement
        print("    1. Projecting onto 'placement' state...")
        res, q_proj, err = self.graph.applyNodeConstraints("placement", q1)
        configs["q_init"] = q_proj if res else q1
        if res:
            print("       ✓ q_init projected")
        else:
            print(f"       ⚠ Projection failed (error: {err})")
        
        # 2. Generate approach-ball config
        print("    2. Generating 'approach-ball' config...")
        for i in range(cfg.MAX_RANDOM_ATTEMPTS):
            q_rand = self.robot.shootRandomConfig()
            res, q_ab, err = self.graph.generateTargetConfig(
                "approach-ball", configs["q_init"], q_rand
            )
            if res:
                configs["q_ab"] = q_ab
                print("       ✓ q_ab generated")
                break
            if (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}...")
        else:
            print("       ⚠ Failed to generate approach-ball config")
            return configs
        
        # 3. Project onto gripper-above-ball
        print("    3. Projecting onto 'gripper-above-ball' state...")
        res, q_above, err = self.graph.applyNodeConstraints(
            "gripper-above-ball", configs["q_ab"]
        )
        configs["q_above"] = q_above if res else configs["q_ab"]
        if res:
            print("       ✓ q_above projected")
        
        # 4. Generate grasp-ball config
        print("    4. Generating 'grasp-ball' config...")
        for i in range(cfg.MAX_RANDOM_ATTEMPTS):
            q_rand = self.robot.shootRandomConfig()
            res, q_gb, err = self.graph.generateTargetConfig(
                "grasp-ball", configs["q_above"], q_rand
            )
            if res:
                configs["q_gb"] = q_gb
                print("       ✓ q_gb generated")
                break
            if (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}...")
        else:
            print("       ⚠ Failed to generate grasp-ball config")
            return configs
        
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        res, q_gp, err = self.graph.applyNodeConstraints(
            "grasp-placement", configs["q_gb"]
        )
        configs["q_grasp_place"] = q_gp if res else configs["q_gb"]
        if res:
            print("       ✓ q_grasp_place projected")
        
        # 6. Generate take-ball-up config
        print("    6. Generating 'take-ball-up' config...")
        for i in range(cfg.MAX_RANDOM_ATTEMPTS):
            q_rand = self.robot.shootRandomConfig()
            res, q_tbu, err = self.graph.generateTargetConfig(
                "take-ball-up", configs["q_grasp_place"], q_rand
            )
            if res:
                configs["q_tbu"] = q_tbu
                print("       ✓ q_tbu generated")
                break
            if (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}...")
        else:
            print("       ⚠ Failed to generate take-ball-up config")
            return configs
        
        # 7. Project onto ball-above-ground
        print("    7. Projecting onto 'ball-above-ground' state...")
        res, q_bag, err = self.graph.applyNodeConstraints(
            "ball-above-ground", configs["q_tbu"]
        )
        configs["q_ball_up"] = q_bag if res else configs["q_tbu"]
        if res:
            print("       ✓ q_ball_up projected")
        
        # 8. Generate take-ball-away config
        print("    8. Generating 'take-ball-away' config...")
        for i in range(cfg.MAX_RANDOM_ATTEMPTS):
            q_rand = self.robot.shootRandomConfig()
            res, q_tba, err = self.graph.generateTargetConfig(
                "take-ball-away", configs["q_ball_up"], q_rand
            )
            if res:
                configs["q_tba"] = q_tba
                print("       ✓ q_tba generated")
                break
            if (i + 1) % 200 == 0:
                print(f"       Attempt {i + 1}...")
        else:
            print("       ⚠ Failed to generate take-ball-away config")
            return configs
        
        # 9. Project onto grasp
        print("    9. Projecting onto 'grasp' state...")
        res, q_g, err = self.graph.applyNodeConstraints(
            "grasp", configs["q_tba"]
        )
        configs["q_grasp"] = q_g if res else configs["q_tba"]
        if res:
            print("       ✓ q_grasp projected")
        
        # 10. Generate goal (move ball to x=0.2)
        print("    10. Generating goal configuration...")
        q2 = list(q1)
        q2[6] = 0.2  # Ball x position (index 6 = first object DOF)
        res, q_goal, err = self.graph.applyNodeConstraints("placement", q2)
        configs["q_goal"] = q_goal if res else q2
        if res:
            print("       ✓ q_goal projected")
        
        return configs
    
    def _generate_configs_pyhpp(self, q_init: List[float]) -> Dict:
        """Generate configurations for PyHPP backend."""
        configs = {}
        states = self.pyhpp_states
        edges = self.pyhpp_edges
        
        problem = self.planner.get_problem()
        shooter = problem.configurationShooter()
        
        q1 = np.array(q_init)
        
        # 1. Project onto placement
        print("    1. Projecting onto 'placement' state...")
        result = self.graph.applyStateConstraints(states['placement'], q1)
        configs["q_init"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_init projected")
        
        # 2. Generate approach-ball
        print("    2. Generating 'approach-ball' config...")
        for i in range(100):
            q_rand = shooter.shoot()
            result = self.graph.generateTargetConfig(
                edges['approach-ball'], np.array(configs["q_init"]), q_rand
            )
            if result.success:
                configs["q_ab"] = result.configuration.tolist()
                print("       ✓ q_ab generated")
                break
        else:
            print("       ⚠ Failed")
            return configs
        
        # 3. Project onto gripper-above-ball
        print("    3. Projecting onto 'gripper-above-ball' state...")
        result = self.graph.applyStateConstraints(
            states['gripper-above-ball'], np.array(configs["q_ab"])
        )
        configs["q_above"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_above projected")
        
        # 4. Generate grasp-ball
        print("    4. Generating 'grasp-ball' config...")
        for i in range(100):
            q_rand = shooter.shoot()
            result = self.graph.generateTargetConfig(
                edges['grasp-ball'], np.array(configs["q_above"]), q_rand
            )
            if result.success:
                configs["q_gb"] = result.configuration.tolist()
                print("       ✓ q_gb generated")
                break
        else:
            print("       ⚠ Failed")
            return configs
        
        # 5. Project onto grasp-placement
        print("    5. Projecting onto 'grasp-placement' state...")
        result = self.graph.applyStateConstraints(
            states['grasp-placement'], np.array(configs["q_gb"])
        )
        configs["q_grasp_place"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_grasp_place projected")
        
        # 6. Generate take-ball-up
        print("    6. Generating 'take-ball-up' config...")
        for i in range(100):
            q_rand = shooter.shoot()
            result = self.graph.generateTargetConfig(
                edges['take-ball-up'], np.array(configs["q_grasp_place"]),
                q_rand
            )
            if result.success:
                configs["q_tbu"] = result.configuration.tolist()
                print("       ✓ q_tbu generated")
                break
        else:
            print("       ⚠ Failed")
            return configs
        
        # 7. Project onto ball-above-ground
        print("    7. Projecting onto 'ball-above-ground' state...")
        result = self.graph.applyStateConstraints(
            states['ball-above-ground'], np.array(configs["q_tbu"])
        )
        configs["q_ball_up"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_ball_up projected")
        
        # 8. Generate take-ball-away
        print("    8. Generating 'take-ball-away' config...")
        for i in range(100):
            q_rand = shooter.shoot()
            result = self.graph.generateTargetConfig(
                edges['take-ball-away'], np.array(configs["q_ball_up"]),
                q_rand
            )
            if result.success:
                configs["q_tba"] = result.configuration.tolist()
                print("       ✓ q_tba generated")
                break
        else:
            print("       ⚠ Failed")
            return configs
        
        # 9. Project onto grasp
        print("    9. Projecting onto 'grasp' state...")
        result = self.graph.applyStateConstraints(
            states['grasp'], np.array(configs["q_tba"])
        )
        configs["q_grasp"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_grasp projected")
        
        # 10. Generate goal
        print("    10. Generating goal configuration...")
        q2 = q1.copy()
        q2[6] = 0.2  # Move ball x
        result = self.graph.applyStateConstraints(states['placement'], q2)
        configs["q_goal"] = result.configuration.tolist()
        if result.success:
            print("       ✓ q_goal projected")
        
        return configs
    
    def setup(self, validation_step: float = 0.01,
              projector_step: float = 0.1):
        """
        Complete task setup: scene, constraints, graph.
        
        Overrides base class to use custom scene setup.
        """
        # 1. Custom scene setup (not using SceneBuilder)
        # self.setup_scene(validation_step, projector_step)
        self.planner, self.robot, self.ps = self.scene_builder.build(
            robot_names=self.robot_names,
            environment_names=self.environment_names,
            composite_names=self.composite_names,
            object_names=self.object_names,
            validation_step=validation_step,
            projector_step=projector_step
        )
        
        # Position box walls
        cfg = self.config
        box_x = cfg.BOX_X
        box_off = cfg.BOX_OFFSET
        self.scene_builder.move_obstacle(
            'box/base_link_0', [box_x + box_off, 0, 0.04], [0, 0, 0, 1]
        )
        self.scene_builder.move_obstacle(
            'box/base_link_1', [box_x - box_off, 0, 0.04], [0, 0, 0, 1]
        )
        self.scene_builder.move_obstacle(
            'box/base_link_2', [box_x, box_off, 0.04], [0, 0, 0, 1]
        )
        self.scene_builder.move_obstacle(
            'box/base_link_3', [box_x, -box_off, 0.04], [0, 0, 0, 1]
        )
        # 2. Create constraints
        print("\n2. Creating constraints...")
        self.create_constraints()
        
        # 3. Create graph
        print("\n3. Creating constraint graph...")
        self.graph = self.create_graph()
        
        # 4. Initialize config generator (simplified for this task)
        from agimus_spacelab.planning import ConfigGenerator
        self.config_gen = ConfigGenerator(
            self.robot, self.graph, self.planner, self.ps, backend=self.backend
        )
        
        print("\n   ✓ Task setup complete")


# ============================================================================
# Main Execution
# ============================================================================

def main(
    visualize: bool = True,
    solve: bool = False,
    backend: str = "corba"
):
    """
    Run the grasp ball task.
    
    Args:
        visualize: Show configurations in viewer
        solve: Attempt to solve planning problem
        backend: "corba" or "pyhpp" - which backend to use
    """
    # Validate backend availability
    if backend == "corba" and not HAS_CORBA:
        print("Error: CORBA backend not available")
        return None, None
    elif backend == "pyhpp" and not HAS_PYHPP:
        print("Error: PyHPP backend not available. Install pyhpp.")
        return None, None
    
    # Create and setup task
    task = GraspBallTask(backend=backend)
    task.setup(
        validation_step=ManipulationConfig.PATH_VALIDATION_STEP,
        projector_step=ManipulationConfig.PATH_PROJECTOR_STEP
    )
    
    # Run task
    result = task.run(visualize=visualize, solve=solve)
    
    # Visualize handles and grippers if viewer available
    if visualize and result.get('viewer'):
        print("\n" + "=" * 70)
        print("Visualizing Handles and Grippers")
        print("=" * 70)
        viewer = result['viewer']
        
        # Collect handle names (pokeball handle)
        handle_names = ["pokeball/handle"]
        
        # Visualize handles with approach arrows
        print("\n  Handles (green frames, cyan approach arrows):")
        try:
            visualize_all_handles(
                viewer, handle_names,
                show_approach=True,
                frame_color=[0, 0.8, 0, 1],   # Green
                arrow_color=[0, 1, 1, 1],      # Cyan
                axis_length=0.03,
                arrow_length=0.08
            )
            for handle_name in handle_names:
                print(f"    ✓ {handle_name}")
        except Exception as e:
            print(f"    ⚠ Handle visualization failed: {e}")
        
        # Visualize grippers with approach arrows
        gripper_names = ["ur5/gripper"]
        print("\n  Grippers (red frames, orange approach arrows):")
        try:
            visualize_all_grippers(
                viewer, gripper_names,
                show_approach=True,
                frame_color=[1, 0, 0, 1],      # Red
                arrow_color=[1, 0.5, 0, 1],    # Orange
                axis_length=0.03,
                arrow_length=0.08
            )
            for gripper_name in gripper_names:
                print(f"    ✓ {gripper_name}")
        except Exception as e:
            print(f"    ⚠ Gripper visualization failed: {e}")
        
        try:
            viewer.client.gui.refresh()
        except Exception:
            pass
    
    # Print summary
    print("\n" + "=" * 70)
    print("Task Complete!")
    print("=" * 70)
    
    if result:
        configs = result.get("configs", {})
        print(f"\nGenerated {len(configs)} configurations:")
        for name, q in configs.items():
            print(f"  {name}: {len(q)} DOF")
        
        print("\nTo visualize specific configs:")
        print("  task.planner.visualize(result['configs']['q_init'])")
        print("  task.planner.visualize(result['configs']['q_above'])")
        print("  task.planner.visualize(result['configs']['q_goal'])")
        
        if solve:
            print("\nTo replay path:")
            print("  task.planner.play_path(0)")
    
        # Visualize constraint graph
    print("\n📊 Generating constraint graph visualization...")
    # Pass state/edge dicts for PyHPP backend
    states_dict = getattr(task, 'pyhpp_states', None)
    edges_dict = getattr(task, 'pyhpp_edges', None)
    edge_topology = getattr(task, 'pyhpp_edge_topology', None)
    viz_path = visualize_constraint_graph(
        task.graph,
        output_path="constraint_graph",
        states_dict=states_dict,
        edges_dict=edges_dict,
        edge_topology=edge_topology
    )
    if viz_path:
        print(f"✓ Graph visualization saved to: {viz_path}")
    

    return task, result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="UR5 grasps pokeball from box"
    )
    parser.add_argument(
        "--no-viz", action="store_true", help="Disable visualization"
    )
    parser.add_argument(
        "--solve", action="store_true", help="Solve planning problem"
    )
    parser.add_argument(
        "--backend", type=str, default="corba",
        choices=["corba", "pyhpp"],
        help="Backend to use: corba (default) or pyhpp"
    )
    
    args = parser.parse_args()
    
    task, result = main(
        visualize=not args.no_viz,
        solve=args.solve,
        backend=args.backend
    )
    
    if task and result:
        print("\n" + "=" * 70)
        print("Available objects:")
        print("=" * 70)
        print("  task   - GraspBallTask instance")
        print("  result - Dictionary with configs, planner, robot, ps, graph")
