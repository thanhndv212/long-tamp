#!/usr/bin/env python3
"""
Visualization utilities for manipulation tasks.

Provides functions for visualizing constraint graphs and handle frames.
Both gepetto-viewer (CORBA) and pyhpp_viser (browser-based) viewers are
supported transparently — functions detect the viewer type at call time.
"""

from typing import Any, Dict, List, Optional, Tuple

from agimus_spacelab.logging import get_logger
from agimus_spacelab.planning.graph import GraphBuilder

logger = get_logger("visualization.viz")


def print_joint_info(robot):
    """Print all joints with their configuration ranks."""
    print("\nJoint Information:")
    joints = robot.getJointNames()
    for i, joint in enumerate(joints):
        rank = robot.rankInConfiguration[joint]
        print(f"  {i:3d}. {joint} (config rank: {rank})")


import numpy as np
from pinocchio import Quaternion

from agimus_spacelab.utils import se3_to_xyzquat, xyzquat_to_se3

# ---------------------------------------------------------------------------
# Viewer-type detection helpers
# ---------------------------------------------------------------------------


def _is_viser_viewer(viewer) -> bool:
    """Return True when *viewer* is a pyhpp_viser.Viewer instance."""
    try:
        from pyhpp_viser import Viewer as _ViserViewer

        return isinstance(viewer, _ViserViewer)
    except ImportError:
        return False


# Registry: {node_name: viser_frame_handle} for viser viewers.
# Used by clear_* functions to remove previously added frames/meshes.
_VISER_FRAME_REGISTRY: Dict[str, Any] = {}


def _viser_xyzquat_to_wxyz(pose: list) -> tuple:
    """Convert [x,y,z,qx,qy,qz,qw] pose (codebase-standard order — see
    utils.transforms.se3_to_xyzquat) to (position, wxyz) viser style."""
    pos = np.array(pose[:3], dtype=float)
    wxyz = np.array([pose[6], pose[3], pose[4], pose[5]], dtype=float)
    return pos, wxyz


def _handle_or_gripper_local_pose(robot, obj) -> tuple:
    """Return (joint_name, local_pose_xyzquat) for a Handle or Gripper.

    Neither `getHandlePositionInJoint` nor `getGripperPositionInJoint`
    exist on this pyhpp build's `Device` — only `getParentJointId()` (int)
    and `.localPosition` (pinocchio.SE3) do, so both are derived from those.
    """
    joint_name = robot.model().names[obj.getParentJointId()]
    local_pose = se3_to_xyzquat(obj.localPosition)
    return joint_name, local_pose


def _world_pose_from_joint(robot, joint_name: str, local_pose) -> list:
    """Compose a joint-local pose with that joint's current world pose.

    Only needed for viser: gepetto parents the added frame node under the
    link's own scene-graph group, so the local pose is enough there. viser
    has no such parenting for an ad-hoc `add_frame()` node, so the frame
    must be placed at its true world pose explicitly. Requires the robot's
    current configuration to already be the one you want frames drawn for.
    """
    if joint_name == "universe":
        return list(local_pose)
    from pyhpp.pinocchio import ComputationFlag

    robot.computeForwardKinematics(ComputationFlag.JOINT_POSITION)
    robot.computeFramesForwardKinematics()
    joint_pose = robot.getJointPosition(joint_name)  # [x,y,z,qx,qy,qz,qw]
    world_se3 = xyzquat_to_se3(joint_pose) * xyzquat_to_se3(local_pose)
    return se3_to_xyzquat(world_se3).tolist()


def _viser_add_frame(
    viewer,
    name: str,
    pose: list,
    axes_length: float = 0.015,
    axes_radius: float = 0.005,
):
    """Add a coordinate-frame to a viser scene and register it.

    Args:
        viewer: pyhpp_viser.Viewer instance
        name: Unique name for the frame in the viser scene
        pose: [x,y,z,qw,qx,qy,qz] in world/joint-local coordinates
        axes_length: Length of XYZ axes
        axes_radius: Radius of XYZ axes

    Returns:
        The created viser frame handle
    """
    pos, wxyz = _viser_xyzquat_to_wxyz(pose)
    handle = viewer.viewer.scene.add_frame(
        name,
        show_axes=True,
        axes_length=axes_length,
        axes_radius=axes_radius,
    )
    handle.position = pos
    handle.wxyz = wxyz
    _VISER_FRAME_REGISTRY[name] = handle
    return handle


def _make_arrow_mesh(length: float, radius: float):
    """Build a trimesh arrow (cylinder shaft + cone tip) along the +X axis.

    Returns:
        trimesh.Trimesh combined mesh
    """
    import trimesh

    shaft_len = length * 0.75
    tip_len = length * 0.25
    shaft = trimesh.creation.cylinder(
        radius=radius,
        height=shaft_len,
        transform=trimesh.transformations.translation_matrix([shaft_len / 2, 0, 0])
        @ trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]),
    )
    cone = trimesh.creation.cone(
        radius=radius * 2.0,
        height=tip_len,
        transform=trimesh.transformations.translation_matrix(
            [shaft_len + tip_len / 2, 0, 0]
        )
        @ trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]),
    )
    return trimesh.util.concatenate([shaft, cone])


def _viser_add_arrow(
    viewer,
    name: str,
    pose: list,
    direction: np.ndarray,
    color: list,
    length: float,
    radius: float,
):
    """Add an arrow mesh to a viser scene pointing along *direction*.

    The arrow is placed at the position encoded in *pose* and oriented along
    *direction* (in the same frame as pose).

    Returns:
        The created viser mesh handle
    """
    arrow_mesh = _make_arrow_mesh(length, radius)
    pos, _wxyz = _viser_xyzquat_to_wxyz(pose)

    # Build transform: rotate from +X to direction
    d = np.array(direction, dtype=float)
    norm = np.linalg.norm(d)
    if norm < 1e-9:
        d = np.array([1.0, 0.0, 0.0])
    else:
        d = d / norm

    x_axis = np.array([1.0, 0.0, 0.0])
    cross = np.cross(x_axis, d)
    cross_norm = np.linalg.norm(cross)
    if cross_norm < 1e-9:
        # Parallel: either same or opposite direction
        R = np.eye(3) if np.dot(x_axis, d) > 0 else -np.eye(3)
    else:
        cross = cross / cross_norm
        angle = np.arccos(np.clip(np.dot(x_axis, d), -1.0, 1.0))
        import pinocchio as pin

        R = pin.AngleAxis(angle, cross).toRotationMatrix()

    import pinocchio as pin

    arrow_wxyz = pin.Quaternion(R).coeffs()[[3, 0, 1, 2]]

    rgba = color[:4] if len(color) >= 4 else color + [1.0]
    handle = viewer.viewer.scene.add_mesh_simple(
        name,
        arrow_mesh.vertices.astype(np.float32),
        arrow_mesh.faces.astype(np.int32),
        color=rgba[:3],
        opacity=rgba[3],
    )
    handle.position = pos
    handle.wxyz = arrow_wxyz
    _VISER_FRAME_REGISTRY[name] = handle
    return handle


def _compute_arrow_orientation_gepetto(direction: np.ndarray) -> Quaternion:
    """Compute quaternion to orient a gepetto-viewer arrow along *direction*."""
    x_axis = direction / np.linalg.norm(direction)
    z_axis = np.array([0.0, 0.0, 1.0])
    y_axis = np.cross(z_axis, x_axis)
    if np.linalg.norm(y_axis) < 1e-6:
        y_axis = np.array([0.0, 1.0, 0.0])
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
    return Quaternion(rot_matrix)


def displayHandle(
    viewer,
    handle_name: str,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
) -> bool:
    """
    Display handle frame in the viewer.

    Works with both gepetto-viewer (CORBA) and pyhpp_viser (browser).
    For gepetto-viewer the frame is parented to the robot link so it moves
    with the robot.  For viser a static frame is added at the handle's
    local-joint pose; call ``viewer.display(q)`` to update geometry positions.

    Args:
        viewer: Viewer instance (gepetto or viser)
        handle_name: Full handle name (e.g., "box/handle2")
        frame_color: RGBA color [r, g, b, a] (default: green [0, 1, 0, 1])
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes

    Returns:
        True if successful
    """
    if frame_color is None:
        frame_color = [0, 1, 0, 1]  # Green

    try:
        robot = viewer.robot if hasattr(viewer, "robot") else viewer._robot
        handle_obj = robot.handles()[handle_name]
        joint, pose = _handle_or_gripper_local_pose(robot, handle_obj)
        hname = "handle__" + handle_name.replace("/", "_")

        if _is_viser_viewer(viewer):
            world_pose = _world_pose_from_joint(robot, joint, pose)
            _viser_add_frame(
                viewer,
                hname,
                world_pose,
                axes_length=axis_length,
                axes_radius=axis_radius,
            )
        else:
            viewer.client.gui.addXYZaxis(hname, frame_color, axis_radius, axis_length)
            if joint != "universe":
                link = robot.getLinkNames(joint)[0]
                viewer.client.gui.addToGroup(hname, robot.name + "/" + link)
            else:
                viewer.client.gui.addToGroup(hname, robot.name)
            viewer.client.gui.applyConfiguration(hname, list(pose))
        return True
    except Exception as e:
        logger.warning("Could not display handle %s: %s", handle_name, e)
        return False


def displayGripper(
    viewer,
    gripper_name: str,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
) -> bool:
    """
    Display gripper frame in the viewer.

    Works with both gepetto-viewer (CORBA) and pyhpp_viser (browser).

    Args:
        viewer: Viewer instance (gepetto or viser)
        gripper_name: Full gripper name (e.g., "pr2/l_gripper")
        frame_color: RGBA color [r, g, b, a] (default: green [0, 1, 0, 1])
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes

    Returns:
        True if successful
    """
    if frame_color is None:
        frame_color = [0, 1, 0, 1]  # Green

    try:
        robot = viewer.robot if hasattr(viewer, "robot") else viewer._robot
        gripper_obj = robot.grippers()[gripper_name]
        joint, pose = _handle_or_gripper_local_pose(robot, gripper_obj)
        gname = "gripper__" + gripper_name.replace("/", "_")

        if _is_viser_viewer(viewer):
            world_pose = _world_pose_from_joint(robot, joint, pose)
            _viser_add_frame(
                viewer,
                gname,
                world_pose,
                axes_length=axis_length,
                axes_radius=axis_radius,
            )
        else:
            viewer.client.gui.addXYZaxis(gname, frame_color, axis_radius, axis_length)
            if joint != "universe":
                link = robot.getLinkNames(joint)[0]
                viewer.client.gui.addToGroup(gname, robot.name + "/" + link)
            else:
                viewer.client.gui.addToGroup(gname, robot.name)
            viewer.client.gui.applyConfiguration(gname, list(pose))
        return True
    except Exception as e:
        logger.warning("Could not display gripper %s: %s", gripper_name, e)
        return False


def displayHandleApproach(
    viewer,
    handle_name: str,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None,
) -> bool:
    """
    Display approach direction arrow for a handle.

    The arrow will be attached to the same link as the handle frame,
    so it moves with the robot configuration.

    Args:
        viewer: Gepetto viewer instance
        handle_name: Full handle name (e.g., "box/handle2")
        arrow_color: RGBA color for arrow [r, g, b, a] (default: [0, 1, 1, 1] cyan)
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow

    Returns:
        True if successful
    """
    if arrow_color is None:
        arrow_color = [0, 1, 1, 1]  # Cyan

    try:
        robot = viewer.robot if hasattr(viewer, "robot") else viewer._robot
        joint, pose = robot.getHandlePositionInJoint(handle_name)

        # Determine approach direction in joint-local frame
        if approach_direction is not None:
            approach_vec = np.array(approach_direction, dtype=float)
        else:
            approach_vec = np.array(
                robot.getHandleApproachingDirection(handle_name), dtype=float
            )

        arrow_name = "handle__" + handle_name.replace("/", "_") + "_approach"

        if _is_viser_viewer(viewer):
            _viser_add_arrow(
                viewer,
                arrow_name,
                pose,
                approach_vec,
                arrow_color,
                arrow_length,
                arrow_radius,
            )
        else:
            handle_T = xyzquat_to_se3(pose)
            approach_world = handle_T.rotation @ approach_vec
            arrow_quat = _compute_arrow_orientation_gepetto(approach_world)
            arrow_pose = [
                pose[0],
                pose[1],
                pose[2],
                arrow_quat.w,
                arrow_quat.x,
                arrow_quat.y,
                arrow_quat.z,
            ]
            viewer.client.gui.addArrow(
                arrow_name, arrow_radius, arrow_length, arrow_color
            )
            viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)
            if joint != "universe":
                link = robot.getLinkNames(joint)[0]
                viewer.client.gui.addToGroup(arrow_name, robot.name + "/" + link)
            else:
                viewer.client.gui.addToGroup(arrow_name, robot.name)
        return True
    except Exception as e:
        logger.warning("Could not display approach arrow for %s: %s", handle_name, e)
        return False


def displayGripperApproach(
    viewer,
    gripper_name: str,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None,
) -> bool:
    """
    Display approach direction arrow for a gripper.

    The arrow will be attached to the same link as the gripper frame,
    so it moves with the robot configuration.

    Args:
        viewer: Gepetto viewer instance
        gripper_name: Full gripper name (e.g., "pr2/l_gripper")
        arrow_color: RGBA color for arrow [r, g, b, a] (default: [1, 0.5, 0, 1] orange)
        arrow_length: Length of approach arrow
        arrow_radius: Radius of approach arrow
        approach_direction: Approach direction in gripper frame (default: [1, 0, 0])

    Returns:
        True if successful
    """
    if arrow_color is None:
        arrow_color = [1, 0.5, 0, 1]  # Orange

    try:
        robot = viewer.robot if hasattr(viewer, "robot") else viewer._robot
        joint, pose = robot.getGripperPositionInJoint(gripper_name)

        approach_vec = (
            np.array(approach_direction, dtype=float)
            if approach_direction is not None
            else np.array([1.0, 0.0, 0.0])
        )

        arrow_name = "gripper__" + gripper_name.replace("/", "_") + "_approach"

        if _is_viser_viewer(viewer):
            _viser_add_arrow(
                viewer,
                arrow_name,
                pose,
                approach_vec,
                arrow_color,
                arrow_length,
                arrow_radius,
            )
        else:
            gripper_T = xyzquat_to_se3(pose)
            approach_world = gripper_T.rotation @ approach_vec
            arrow_quat = _compute_arrow_orientation_gepetto(approach_world)
            arrow_pose = [
                pose[0],
                pose[1],
                pose[2],
                arrow_quat.w,
                arrow_quat.x,
                arrow_quat.y,
                arrow_quat.z,
            ]
            viewer.client.gui.addArrow(
                arrow_name, arrow_radius, arrow_length, arrow_color
            )
            viewer.client.gui.applyConfiguration(arrow_name, arrow_pose)
            if joint != "universe":
                link = robot.getLinkNames(joint)[0]
                viewer.client.gui.addToGroup(arrow_name, robot.name + "/" + link)
            else:
                viewer.client.gui.addToGroup(arrow_name, robot.name)
        return True
    except Exception as e:
        logger.warning("Could not display approach arrow for %s: %s", gripper_name, e)
        return False


def visualize_all_handles(
    viewer,
    handle_names: List[str],
    show_approach: bool = True,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
) -> int:
    """
    Display multiple handles at once.

    Args:
        viewer: Gepetto viewer instance
        handle_names: List of handle names
        show_approach: Whether to display approach arrows
        frame_color: RGBA color for frames
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        arrow_color: RGBA color for arrows
        arrow_length: Length of approach arrows
        arrow_radius: Radius of approach arrows

    Returns:
        Number of successfully visualized handles
    """
    logger.info("Displaying %d handles...", len(handle_names))
    success_count = 0

    for handle_name in handle_names:
        logger.debug("  %s", handle_name)
        frame_ok = displayHandle(
            viewer,
            handle_name,
            frame_color=frame_color,
            axis_radius=axis_radius,
            axis_length=axis_length,
        )
        arrow_ok = True
        if show_approach:
            arrow_ok = displayHandleApproach(
                viewer,
                handle_name,
                arrow_color=arrow_color,
                arrow_length=arrow_length,
                arrow_radius=arrow_radius,
            )

        if frame_ok and arrow_ok:
            success_count += 1
            status = "frame and arrow" if show_approach else "frame"
            logger.debug("    ✓ %s added", status)
        elif frame_ok:
            logger.debug("    ✓ Frame added (arrow failed)")
        else:
            logger.debug("    ✗ Failed")

    if not _is_viser_viewer(viewer):
        viewer.client.gui.refresh()
    logger.info(
        "Successfully displayed %d/%d handles", success_count, len(handle_names)
    )
    return success_count


def visualize_all_grippers(
    viewer,
    gripper_names: List[str],
    show_approach: bool = True,
    frame_color: Optional[List[float]] = None,
    axis_radius: float = 0.005,
    axis_length: float = 0.015,
    arrow_color: Optional[List[float]] = None,
    arrow_length: float = 0.15,
    arrow_radius: float = 0.008,
    approach_direction: Optional[List[float]] = None,
) -> int:
    """
    Display multiple grippers at once.

    Args:
        viewer: Gepetto viewer instance
        gripper_names: List of gripper names
        show_approach: Whether to display approach arrows
        frame_color: RGBA color for frames
        axis_radius: Radius of XYZ axes
        axis_length: Length of XYZ axes
        arrow_color: RGBA color for arrows
        arrow_length: Length of approach arrows
        arrow_radius: Radius of approach arrows
        approach_direction: Approach direction in gripper frame

    Returns:
        Number of successfully visualized grippers
    """
    logger.info("Displaying %d grippers...", len(gripper_names))
    success_count = 0

    for gripper_name in gripper_names:
        logger.debug("  %s", gripper_name)
        frame_ok = displayGripper(
            viewer,
            gripper_name,
            frame_color=frame_color,
            axis_radius=axis_radius,
            axis_length=axis_length,
        )
        arrow_ok = True
        if show_approach:
            arrow_ok = displayGripperApproach(
                viewer,
                gripper_name,
                arrow_color=arrow_color,
                arrow_length=arrow_length,
                arrow_radius=arrow_radius,
                approach_direction=approach_direction,
            )

        if frame_ok and arrow_ok:
            success_count += 1
            status = "frame and arrow" if show_approach else "frame"
            logger.debug("    ✓ %s added", status)
        elif frame_ok:
            logger.debug("    ✓ Frame added (arrow failed)")
        else:
            logger.debug("    ✗ Failed")

    if not _is_viser_viewer(viewer):
        viewer.client.gui.refresh()
    logger.info(
        "Successfully displayed %d/%d grippers", success_count, len(gripper_names)
    )
    return success_count


def print_handle_info(viewer, handle_name: str) -> None:
    """
    Print detailed information about a handle.

    Args:
        viewer: Gepetto viewer instance
        handle_name: Full handle name
    """
    robot = viewer.robot
    handle_info = robot.getHandlePositionInJoint(handle_name)
    approach_dir = list(robot.getHandleApproachingDirection(handle_name))

    print(f"\nHandle: {handle_name}")
    print(f"  Joint: {handle_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {handle_info[1]}")
    print(f"  Approaching direction: {approach_dir}")


def print_gripper_info(viewer, gripper_name: str) -> None:
    """
    Print detailed information about a gripper.

    Args:
        viewer: Gepetto viewer instance
        gripper_name: Full gripper name
    """
    robot = viewer.robot
    gripper_info = robot.getGripperPositionInJoint(gripper_name)

    print(f"\nGripper: {gripper_name}")
    print(f"  Joint: {gripper_info[0]}")
    print(f"  Local pose (x,y,z,qw,qx,qy,qz): {gripper_info[1]}")


def remove_visualization(viewer, name: str) -> bool:
    """
    Remove a visualization element from viewer.

    Args:
        viewer: Gepetto viewer instance
        name: Name of element to remove (e.g., "handle__box_handle2")

    Returns:
        True if successful
    """
    try:
        viewer.client.gui.deleteNode(name, True)
        return True
    except Exception:
        return False


def clear_handle_visualizations(viewer) -> int:
    """
    Clear all handle visualization elements (handle__ prefix).

    Args:
        viewer: Gepetto viewer or viser viewer instance

    Returns:
        Number of elements removed
    """
    count = 0
    if _is_viser_viewer(viewer):
        to_remove = [
            k for k in list(_VISER_FRAME_REGISTRY.keys()) if k.startswith("handle__")
        ]
        for name in to_remove:
            try:
                _VISER_FRAME_REGISTRY.pop(name).remove()
                count += 1
            except Exception:
                pass
    else:
        try:
            nodes = viewer.client.gui.getNodeList()
            for node in nodes:
                if node.startswith("handle__"):
                    if remove_visualization(viewer, node):
                        count += 1
            viewer.client.gui.refresh()
        except Exception as e:
            logger.warning("Could not clear handle visualizations: %s", e)
    return count


def clear_gripper_visualizations(viewer) -> int:
    """
    Clear all gripper visualization elements (gripper__ prefix).

    Args:
        viewer: Gepetto viewer or viser viewer instance

    Returns:
        Number of elements removed
    """
    count = 0
    if _is_viser_viewer(viewer):
        to_remove = [
            k for k in list(_VISER_FRAME_REGISTRY.keys()) if k.startswith("gripper__")
        ]
        for name in to_remove:
            try:
                _VISER_FRAME_REGISTRY.pop(name).remove()
                count += 1
            except Exception:
                pass
    else:
        try:
            nodes = viewer.client.gui.getNodeList()
            for node in nodes:
                if node.startswith("gripper__"):
                    if remove_visualization(viewer, node):
                        count += 1
            viewer.client.gui.refresh()
        except Exception as e:
            logger.warning("Could not clear gripper visualizations: %s", e)
    return count


def clear_all_visualizations(viewer) -> int:
    """
    Clear all handle and gripper visualization elements.

    Args:
        viewer: Gepetto viewer instance

    Returns:
        Number of elements removed
    """
    handle_count = clear_handle_visualizations(viewer)
    gripper_count = clear_gripper_visualizations(viewer)
    return handle_count + gripper_count


def visualize_constraint_graph(
    graph_builder: GraphBuilder,
    output_path: str = "constraint_graph",
    include_subgraph: bool = True,
    show_png: bool = False,
    states_dict: Optional[Dict] = None,
    edges_dict: Optional[Dict] = None,
    edge_topology: Optional[Dict[str, Tuple[str, str]]] = None,
) -> Optional[str]:
    """
    Visualize constraint graph structure using NetworkX and Graphviz.

    Creates a visual representation of the constraint graph nodes and edges,
    with optional PNG generation and display.

    Args:
        graph: Graph instance (PyHPP)
        output_path: Base path for output files (without extension)
        include_subgraph: Include subgraph details if available
        show_png: If True, attempt to open the PNG after generation
        states_dict: Optional dict of states (for PyHPP backend)
        edges_dict: Optional dict of edges (for PyHPP backend)
        edge_topology: Optional dict mapping edge names to (from, to) tuples

    Returns:
        Path to the generated PNG file if successful, None otherwise
    """
    try:
        import warnings

        import matplotlib.pyplot as plt
        import networkx as nx

        warnings.filterwarnings("ignore", category=DeprecationWarning)

        # # Detect backend type and get graph structure accordingly
        # is_pyhpp = hasattr(graph, 'getStates')
        nodes = list(graph_builder.get_states().keys())
        edges = list(graph_builder.get_edges().keys())

        logger.info("📊 Constraint Graph Structure:")
        logger.info("  Nodes: %d", len(nodes))
        logger.info("  Edges: %d", len(edges))

        # Create directed graph
        G = nx.DiGraph()

        # Add nodes with labels
        for node in nodes:
            G.add_node(node, label=node)

        # Add edges with labels
        edge_labels = {}
        edge_topology = graph_builder.get_edge_topology()
        for edge_name in edges:
            from_node, to_node = edge_topology.get(edge_name, ("?", "?"))
            G.add_edge(from_node, to_node, label=edge_name)
            edge_labels[(from_node, to_node)] = edge_name
        # Set up the plot
        _fig, ax = plt.subplots(figsize=(14, 10))

        # Use hierarchical layout for better readability
        try:
            pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
        except Exception:
            pos = nx.spring_layout(G, seed=42)

        # Draw nodes
        node_colors = [
            "lightblue" if "free" in node.lower() else "lightgreen"
            for node in G.nodes()
        ]
        nx.draw_networkx_nodes(
            G, pos, node_color=node_colors, node_size=3000, alpha=0.9, ax=ax
        )

        # Draw node labels
        nx.draw_networkx_labels(G, pos, font_size=9, font_weight="bold", ax=ax)

        # Draw edges
        nx.draw_networkx_edges(
            G,
            pos,
            edge_color="gray",
            arrows=True,
            arrowsize=20,
            arrowstyle="->",
            width=2,
            ax=ax,
        )

        # Draw edge labels
        nx.draw_networkx_edge_labels(G, pos, edge_labels, font_size=7, ax=ax)

        # Add title and info
        title = "Constraint Graph Visualization"
        if include_subgraph:
            title += f"\n{len(nodes)} nodes, {len(edges)} edges"
        ax.set_title(title, fontsize=14, fontweight="bold", pad=20)
        ax.axis("off")

        plt.tight_layout()

        # Save to file
        png_path = f"{output_path}.png"
        plt.savefig(png_path, dpi=150, bbox_inches="tight")
        logger.info("✓ Saved graph visualization to: %s", png_path)

        # Log node/edge details
        logger.debug("  Nodes:")
        for node in nodes:
            logger.debug("    • %s", node)

        logger.debug("  Edges:")
        for from_node, to_node in G.edges():
            edge_name = edge_labels.get((from_node, to_node), "?")
            logger.debug("    • %s: %s → %s", edge_name, from_node, to_node)

        # Optionally display
        if show_png:
            try:
                plt.show()
            except Exception:
                logger.warning("Could not display PNG (no display available)")
        else:
            plt.close()

        return png_path

    except ImportError as e:
        logger.warning("Visualization requires networkx and matplotlib: %s", e)
        logger.warning("Install with: pip install networkx matplotlib")
        return None
    except Exception as e:
        logger.warning("Failed to visualize graph: %s", e)
        import traceback

        traceback.print_exc()
        return None


def visualize_constraint_graph_interactive(
    graph_builder: "GraphBuilder",
    window_size: Tuple[int, int] = (1200, 800),
    neighborhood_hops: Optional[int] = None,
    show_window: bool = True,
    blocking: bool = False,
) -> Optional[Any]:
    """
    Create an interactive constraint graph visualization using graph-tool.

    This function creates a live, interactive graph visualization that can be
    updated in real-time during path playback. The graph window supports:
    - Zooming and panning
    - Dragging nodes
    - Real-time state/edge highlighting

    Args:
        graph_builder: GraphBuilder instance with populated states/edges
        window_size: Window size (width, height) in pixels
        neighborhood_hops: If set, filter to show only N-hop neighborhood
        show_window: If True, display the window immediately
        blocking: If True, block until window is closed

    Returns:
        LiveConstraintGraphVisualizer instance, or None if graph-tool unavailable

    Example:
        >>> from agimus_spacelab.visualization.viz import visualize_constraint_graph_interactive
        >>> viz = visualize_constraint_graph_interactive(graph_builder)
        >>> # Later, during path playback:
        >>> planner.replay_sequence(visualizer=viz)
    """
    try:
        from agimus_spacelab.visualization.live_graph_viz import (
            HAS_GRAPH_TOOL,
            LiveConstraintGraphVisualizer,
        )

        if not HAS_GRAPH_TOOL:
            logger.warning(
                "graph-tool not available. Install with: "
                "conda install -c conda-forge graph-tool. "
                "Falling back to static visualization..."
            )
            return None

        # Create visualizer
        visualizer = LiveConstraintGraphVisualizer(
            graph_builder,
            window_size=window_size,
            neighborhood_hops=neighborhood_hops,
        )

        # Build the graph
        visualizer.build_graph()

        # Show window if requested
        if show_window:
            visualizer.show(blocking=blocking)

        return visualizer

    except ImportError as e:
        logger.warning(
            "Failed to import live graph visualization: %s. "
            "Ensure graph-tool is installed: conda install -c conda-forge graph-tool",
            e,
        )
        return None
    except Exception as e:
        logger.warning("Failed to create interactive visualization: %s", e)
        import traceback

        traceback.print_exc()
        return None


__all__ = [
    "print_joint_info",
    "visualize_all_handles",
    "visualize_constraint_graph",
    "visualize_constraint_graph_interactive",
]
