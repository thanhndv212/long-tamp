#!/usr/bin/env python3
"""
Visualization utilities for manipulation tasks.

Provides functions for visualizing constraint graphs and handle frames.
"""

from typing import List, Optional, Dict, Tuple, Any


def print_joint_info(robot):
    """Print all joints with their configuration ranks."""
    print("\nJoint Information:")
    joints = robot.jointNames
    for i, joint in enumerate(joints):
        rank = robot.rankInConfiguration[joint]
        print(f"  {i:3d}. {joint} (config rank: {rank})")


def visualize_handle_frames(
    robot, planner, q, handle_names: List[str]
):
    """
    Add visualization for handle frames and approaching directions.
    
    Args:
        robot: Robot instance
        planner: Planner with viewer
        q: Configuration to visualize
        handle_names: List of handle names to visualize
    """
    try:
        import numpy as np  # noqa: F401
        from pinocchio import SE3, Quaternion  # noqa: F401
        
        v = planner.viewer if hasattr(planner, 'viewer') else None
        if not v:
            print("   ⚠ No viewer available")
            return
            
        robot.setCurrentConfig(q)
        
        for handle_name in handle_names:
            try:
                # Get handle info
                approach_dir = list(
                    robot.getHandleApproachingDirection(handle_name)
                )
                handle_pos = robot.getHandlePositionInJoint(handle_name)
                
                # Add frame visualization
                safe_name = handle_name.replace('/', '_')
                frame_name = f"hpp-gui/{safe_name}_frame"
                arrow_name = f"hpp-gui/{safe_name}_approach"
                
                v.client.gui.addXYZaxis(
                    frame_name, [0, 0.8, 0, 1], 0.008, 0.1
                )
                print(f"  Added frame: {handle_name} "
                      f"(approach: {approach_dir})")
                
            except Exception as e:
                print(f"  ⚠ Failed to add frame for {handle_name}: {e}")
                
    except ImportError:
        print("   ⚠ Visualization requires pinocchio")


def visualize_constraint_graph(
    graph,
    output_path: str = "constraint_graph",
    include_subgraph: bool = True,
    show_png: bool = False,
    states_dict: Optional[Dict] = None,
    edges_dict: Optional[Dict] = None,
    edge_topology: Optional[Dict[str, Tuple[str, str]]] = None
) -> Optional[str]:
    """
    Visualize constraint graph structure using NetworkX and Graphviz.

    Creates a visual representation of the constraint graph nodes and edges,
    with optional PNG generation and display.

    Args:
        graph: ConstraintGraph instance (CORBA) or Graph instance (PyHPP)
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
        import networkx as nx
        import matplotlib.pyplot as plt
        import warnings
        warnings.filterwarnings('ignore', category=DeprecationWarning)

        # Detect backend type and get graph structure accordingly
        is_pyhpp = hasattr(graph, 'getStates')
        
        if is_pyhpp:
            # PyHPP backend - use provided dictionaries if available
            if states_dict is None or edges_dict is None:
                print("  ⚠ PyHPP graph requires states_dict "
                      "and edges_dict parameters")
                return None
            nodes = list(states_dict.keys())
            edges = list(edges_dict.keys())
        else:
            # CORBA backend - use nodes and edges dictionaries
            nodes = list(graph.nodes.keys())
            edges = list(graph.edges.keys())

        print("\n📊 Constraint Graph Structure:")
        print(f"  Nodes: {len(nodes)}")
        print(f"  Edges: {len(edges)}")

        # Create directed graph
        G = nx.DiGraph()

        # Add nodes with labels
        for node in nodes:
            G.add_node(node, label=node)

        # Add edges with labels
        edge_labels = {}
        for edge_name in edges:
            try:
                if is_pyhpp:
                    # PyHPP: use topology dictionary if available
                    if edge_topology and edge_name in edge_topology:
                        from_node, to_node = edge_topology[edge_name]
                    else:
                        print(f"  ⚠ No topology for edge '{edge_name}'")
                        continue
                else:
                    # CORBA: use getNodesConnectedByEdge
                    edge_info = graph.getNodesConnectedByEdge(edge_name)
                    from_node = edge_info[0]
                    to_node = edge_info[1]
                
                G.add_edge(from_node, to_node)
                edge_labels[(from_node, to_node)] = edge_name
            except Exception as e:
                print(f"  ⚠ Could not get info for edge "
                      f"'{edge_name}': {e}")

        # Set up the plot
        fig, ax = plt.subplots(figsize=(14, 10))

        # Use hierarchical layout for better readability
        try:
            pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
        except Exception:
            pos = nx.spring_layout(G, seed=42)

        # Draw nodes
        node_colors = [
            'lightblue' if 'free' in node.lower() else 'lightgreen'
            for node in G.nodes()
        ]
        nx.draw_networkx_nodes(
            G, pos, node_color=node_colors,
            node_size=3000, alpha=0.9, ax=ax
        )

        # Draw node labels
        nx.draw_networkx_labels(
            G, pos, font_size=9, font_weight='bold', ax=ax
        )

        # Draw edges
        nx.draw_networkx_edges(
            G, pos, edge_color='gray',
            arrows=True, arrowsize=20,
            arrowstyle='->', width=2, ax=ax
        )

        # Draw edge labels
        nx.draw_networkx_edge_labels(
            G, pos, edge_labels, font_size=7, ax=ax
        )

        # Add title and info
        title = "Constraint Graph Visualization"
        if include_subgraph:
            title += f"\n{len(nodes)} nodes, {len(edges)} edges"
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
        ax.axis('off')

        plt.tight_layout()

        # Save to file
        png_path = f"{output_path}.png"
        plt.savefig(png_path, dpi=150, bbox_inches='tight')
        print(f"  ✓ Saved graph visualization to: {png_path}")

        # Print node/edge details
        print("  Nodes:")
        for node in nodes:
            print(f"    • {node}")

        print("  Edges:")
        for from_node, to_node in G.edges():
            edge_name = edge_labels.get((from_node, to_node), "?")
            print(f"    • {edge_name}: {from_node} → {to_node}")

        # Optionally display
        if show_png:
            try:
                plt.show()
            except Exception:
                print("  ⚠ Could not display PNG (no display available)")
        else:
            plt.close()

        return png_path

    except ImportError as e:
        print(f"  ⚠ Visualization requires networkx and matplotlib: {e}")
        print("  Install with: pip install networkx matplotlib")
        return None
    except Exception as e:
        print(f"  ⚠ Failed to visualize graph: {e}")
        import traceback
        traceback.print_exc()
        return None


__all__ = [
    "print_joint_info",
    "visualize_handle_frames",
    "visualize_constraint_graph",
]
