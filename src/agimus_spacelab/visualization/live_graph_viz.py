#!/usr/bin/env python3
"""
Live constraint graph visualization using graph-tool.

Provides real-time graph visualization that updates during path playback
to show current robot state in the constraint graph.
"""

from typing import Dict, List, Tuple, Optional, Callable, Any
import threading
import time

# graph-tool imports
try:
    import graph_tool.all as gt
    from graph_tool.draw import GraphWindow, graph_draw

    HAS_GRAPH_TOOL = True
except ImportError:
    HAS_GRAPH_TOOL = False
    gt = None
    GraphWindow = None

from agimus_spacelab.planning.graph import GraphBuilder


class LiveConstraintGraphVisualizer:
    """
    Interactive constraint graph visualization with real-time state highlighting.

    Features:
    - Graph-tool based interactive window with SFDP layout
    - Real-time state/edge highlighting during path playback
    - Hierarchical node coloring (free vs grasp states)
    - Animated edge traversal
    - Neighborhood filtering for large graphs
    """

    def __init__(
        self,
        graph_builder: GraphBuilder,
        window_size: Tuple[int, int] = (1200, 800),
        update_interval: float = 0.05,
        neighborhood_hops: Optional[int] = None,
    ):
        """
        Initialize live graph visualizer.

        Args:
            graph_builder: GraphBuilder instance with populated states/edges
            window_size: Initial window size (width, height)
            update_interval: Minimum time between visual updates (seconds)
            neighborhood_hops: If set, show only N-hop neighborhood of current state
        """
        if not HAS_GRAPH_TOOL:
            raise ImportError(
                "graph-tool not available. Install with: "
                "conda install -c conda-forge graph-tool"
            )

        self.graph_builder = graph_builder
        self.window_size = window_size
        self.update_interval = update_interval
        self.neighborhood_hops = neighborhood_hops

        # graph-tool graph
        self.gt_graph: Optional[gt.Graph] = None
        self.window: Optional[GraphWindow] = None
        self.pos: Optional[gt.VertexPropertyMap] = None

        # Property maps for visualization
        self.vertex_text: Optional[gt.VertexPropertyMap] = None
        self.vertex_fill_color: Optional[gt.VertexPropertyMap] = None
        self.vertex_halo: Optional[gt.VertexPropertyMap] = None
        self.vertex_halo_color: Optional[gt.VertexPropertyMap] = None
        self.vertex_size: Optional[gt.VertexPropertyMap] = None
        self.edge_color: Optional[gt.EdgePropertyMap] = None
        self.edge_pen_width: Optional[gt.EdgePropertyMap] = None
        self.edge_text: Optional[gt.EdgePropertyMap] = None

        # State name -> vertex index mapping
        self.state_to_vertex: Dict[str, int] = {}
        self.vertex_to_state: Dict[int, str] = {}
        # Edge name -> edge descriptor mapping
        self.edge_name_to_edge: Dict[str, Any] = {}

        # Current highlight state
        self.current_state: Optional[str] = None
        self.current_edge: Optional[str] = None
        self.traversed_edges: List[str] = []

        # Threading for non-blocking updates
        self._update_lock = threading.Lock()
        self._last_update_time = 0.0

        # Color scheme (RGBA format)
        self.colors = {
            "free_state": [0.7, 0.85, 1.0, 1.0],  # Light blue
            "grasp_state": [0.7, 1.0, 0.7, 1.0],  # Light green
            "waypoint_state": [1.0, 0.9, 0.7, 1.0],  # Light orange
            "current_state": [1.0, 0.3, 0.3, 1.0],  # Red (highlight)
            "halo_current": [1.0, 0.0, 0.0, 0.8],  # Red halo
            "edge_default": [0.5, 0.5, 0.5, 0.6],  # Gray
            "edge_active": [1.0, 0.0, 0.0, 1.0],  # Red
            "edge_traversed": [0.0, 0.7, 0.0, 0.8],  # Green
        }

    def build_graph(self) -> None:
        """Build graph-tool graph from GraphBuilder data."""
        self.gt_graph = gt.Graph(directed=True)

        # Get data from graph builder
        states = self.graph_builder.get_states()
        edges = self.graph_builder.get_edges()
        edge_topology = self.graph_builder.get_edge_topology()

        # Create property maps
        self.vertex_text = self.gt_graph.new_vertex_property("string")
        self.vertex_fill_color = self.gt_graph.new_vertex_property(
            "vector<float>"
        )
        self.vertex_halo = self.gt_graph.new_vertex_property("bool")
        self.vertex_halo_color = self.gt_graph.new_vertex_property(
            "vector<float>"
        )
        self.vertex_size = self.gt_graph.new_vertex_property("float")

        self.edge_color = self.gt_graph.new_edge_property("vector<float>")
        self.edge_pen_width = self.gt_graph.new_edge_property("float")
        self.edge_text = self.gt_graph.new_edge_property("string")

        # Add vertices (states)
        print(
            f"Building graph with {len(states)} states and {len(edge_topology)} edges..."
        )
        for state_name in states.keys():
            v = self.gt_graph.add_vertex()
            v_idx = int(v)
            self.state_to_vertex[state_name] = v_idx
            self.vertex_to_state[v_idx] = state_name

            # Set vertex label (truncate long names)
            display_name = self._format_state_name(state_name)
            self.vertex_text[v] = display_name

            # Set vertex color based on state type
            self.vertex_fill_color[v] = self._get_state_color(state_name)

            self.vertex_halo[v] = False
            self.vertex_halo_color[v] = [0, 0, 0, 0]
            self.vertex_size[v] = 45  # Base size

        # Add edges
        edges_added = 0
        for edge_name, (from_state, to_state) in edge_topology.items():
            if (
                from_state not in self.state_to_vertex
                or to_state not in self.state_to_vertex
            ):
                print(
                    f"Warning: Edge {edge_name} references unknown states: {from_state} -> {to_state}"
                )
                continue

            v_from = self.gt_graph.vertex(self.state_to_vertex[from_state])
            v_to = self.gt_graph.vertex(self.state_to_vertex[to_state])

            e = self.gt_graph.add_edge(v_from, v_to)
            self.edge_name_to_edge[edge_name] = e

            self.edge_color[e] = self.colors["edge_default"]
            self.edge_pen_width[e] = 2.0

            # Truncate edge labels for display
            edge_label = self._format_edge_name(edge_name)
            self.edge_text[e] = edge_label

            edges_added += 1

        print(
            f"Graph built successfully: {self.gt_graph.num_vertices()} vertices, {self.gt_graph.num_edges()} edges"
        )

    def _format_state_name(self, state_name: str) -> str:
        """Format state name for display."""
        if len(state_name) < 40:
            return state_name
        # Truncate but keep meaningful parts
        if "grasps" in state_name:
            # Show just gripper-handle pairs
            parts = state_name.split(":")
            short_parts = [
                p.split("grasps")[0].strip()
                + " → "
                + p.split("grasps")[1].strip().split("/")[0]
                for p in parts
                if "grasps" in p
            ]
            return " : ".join(short_parts)[:37] + "..."
        return state_name[:37] + "..."

    def _format_edge_name(self, edge_name: str) -> str:
        """Format edge name for display."""
        if len(edge_name) < 30:
            return edge_name
        # Show key parts only
        if "|" in edge_name:
            parts = edge_name.split("|")
            if len(parts) >= 2:
                return parts[0][:15] + "|" + parts[-1][-10:]
        return edge_name[:27] + "..."

    def _get_state_color(self, state_name: str) -> List[float]:
        """Get color for a state based on its type."""
        state_lower = state_name.lower()
        if "free" in state_lower:
            return self.colors["free_state"]
        elif any(
            wp in state_name for wp in ["_pregrasp", "_preplace", "waypoint"]
        ):
            return self.colors["waypoint_state"]
        else:
            return self.colors["grasp_state"]

    def compute_layout(self) -> gt.VertexPropertyMap:
        """Compute graph layout using SFDP algorithm."""
        print(
            "Computing SFDP layout (this may take a moment for large graphs)..."
        )

        # SFDP works well for hierarchical constraint graphs
        # Use higher K for more spacing between nodes
        pos = gt.sfdp_layout(
            self.gt_graph,
            K=3.0,  # Optimal edge length (increase for more spacing)
            C=0.2,  # Relative strength of repulsive forces
            p=2.0,  # Repulsive exponent
            gamma=1.0,  # Attraction strength
            mu=0.1,  # Step size damping
            max_iter=1000,  # Maximum iterations
        )

        print("Layout computed successfully")
        return pos

    def show(self, blocking: bool = False) -> None:
        """
        Display the graph window.

        Args:
            blocking: If True, block until window is closed
        """
        if self.gt_graph is None:
            self.build_graph()

        if self.pos is None:
            self.pos = self.compute_layout()

        print("Opening interactive graph window...")

        # Create interactive window
        self.window = GraphWindow(
            self.gt_graph,
            self.pos,
            geometry=self.window_size,
            vertex_text=self.vertex_text,
            vertex_fill_color=self.vertex_fill_color,
            vertex_halo=self.vertex_halo,
            vertex_halo_color=self.vertex_halo_color,
            vertex_size=self.vertex_size,
            vertex_font_size=10,
            edge_color=self.edge_color,
            edge_pen_width=self.edge_pen_width,
            edge_text=self.edge_text,
            edge_font_size=8,
            edge_marker_size=12,
            output_size=self.window_size,
        )

        print(
            "Graph window opened. You can now zoom, pan, and interact with the graph."
        )

        if blocking:
            import gi

            gi.require_version("Gtk", "3.0")
            from gi.repository import Gtk

            self.window.connect("delete_event", lambda *args: Gtk.main_quit())
            Gtk.main()
        else:
            # Run GTK main loop in background thread to keep window responsive
            import gi
            import threading

            gi.require_version("Gtk", "3.0")
            from gi.repository import Gtk, GLib

            def gtk_main_loop():
                """Run GTK main loop in background."""
                Gtk.main()

            # Start GTK main loop in daemon thread
            self.gtk_thread = threading.Thread(target=gtk_main_loop, daemon=True)
            self.gtk_thread.start()

            # Give window time to appear
            time.sleep(0.5)

    def highlight_state(self, state_name: str) -> None:
        """
        Highlight the current state in the graph.

        Args:
            state_name: Name of state to highlight
        """
        with self._update_lock:
            # Throttle updates
            now = time.time()
            if now - self._last_update_time < self.update_interval:
                return
            self._last_update_time = now

            # Clear previous highlight
            if (
                self.current_state
                and self.current_state in self.state_to_vertex
            ):
                old_v = self.gt_graph.vertex(
                    self.state_to_vertex[self.current_state]
                )
                self.vertex_halo[old_v] = False
                self.vertex_size[old_v] = 45

            # Set new highlight
            self.current_state = state_name
            if state_name in self.state_to_vertex:
                v = self.gt_graph.vertex(self.state_to_vertex[state_name])
                self.vertex_halo[v] = True
                self.vertex_halo_color[v] = self.colors["halo_current"]
                self.vertex_size[v] = 60  # Larger for current state

                print(f"Current state: {state_name}")

            # Refresh display
            self._refresh_display()

    def highlight_edge(self, edge_name: str, traversed: bool = False) -> None:
        """
        Highlight an edge in the graph.

        Args:
            edge_name: Name of edge to highlight
            traversed: If True, mark as traversed (green); else active (red)
        """
        with self._update_lock:
            if edge_name not in self.edge_name_to_edge:
                print(f"Warning: Edge {edge_name} not found in graph")
                return

            e = self.edge_name_to_edge[edge_name]

            if traversed:
                self.edge_color[e] = self.colors["edge_traversed"]
                self.edge_pen_width[e] = 3.0
                if edge_name not in self.traversed_edges:
                    self.traversed_edges.append(edge_name)
                print(f"Edge traversed: {edge_name}")
            else:
                self.edge_color[e] = self.colors["edge_active"]
                self.edge_pen_width[e] = 5.0
                print(f"Edge active: {edge_name}")

            self.current_edge = edge_name

            # Refresh display
            self._refresh_display()

    def reset_highlights(self) -> None:
        """Reset all highlights to default state."""
        with self._update_lock:
            # Reset all vertices
            for v in self.gt_graph.vertices():
                self.vertex_halo[v] = False
                self.vertex_size[v] = 45

            # Reset all edges
            for e in self.gt_graph.edges():
                self.edge_color[e] = self.colors["edge_default"]
                self.edge_pen_width[e] = 2.0

            self.current_state = None
            self.current_edge = None
            self.traversed_edges = []

            print("Highlights reset")
            self._refresh_display()

    def _refresh_display(self) -> None:
        """Refresh the visualization display."""
        if self.window:
            try:
                self.window.graph.regenerate_surface()
                self.window.graph.queue_draw()

                # Process GTK events to keep window responsive
                import gi

                gi.require_version("Gtk", "3.0")
                from gi.repository import Gtk

                while Gtk.events_pending():
                    Gtk.main_iteration_do(False)
            except Exception as e:
                print(f"Warning: Failed to refresh display: {e}")

    def get_state_update_callback(self) -> Callable[[str], None]:
        """
        Get a callback function for state updates.

        Returns:
            Callback function that accepts state name string
        """
        return self.highlight_state

    def get_edge_update_callback(self) -> Callable[[str, bool], None]:
        """
        Get a callback function for edge updates.

        Returns:
            Callback function that accepts (edge_name, traversed) tuple
        """
        return self.highlight_edge


class LivePathPlayer:
    """
    PathPlayer wrapper that calls visualization callbacks during playback.
    """

    def __init__(
        self,
        path_player,  # Original PathPlayer
        graph_builder: GraphBuilder,
        visualizer: Optional[LiveConstraintGraphVisualizer] = None,
        state_callback: Optional[Callable[[str], None]] = None,
        edge_callback: Optional[Callable[[str, bool], None]] = None,
    ):
        """
        Initialize live path player.

        Args:
            path_player: Original hpp PathPlayer instance
            graph_builder: GraphBuilder for state detection
            visualizer: Optional LiveConstraintGraphVisualizer instance
            state_callback: Optional callback for state changes
            edge_callback: Optional callback for edge traversals
        """
        self.path_player = path_player
        self.graph_builder = graph_builder
        self.visualizer = visualizer

        self.state_callback = state_callback or (
            visualizer.highlight_state if visualizer else lambda s: None
        )
        self.edge_callback = edge_callback or (
            visualizer.highlight_edge if visualizer else lambda e, t: None
        )

        self._last_state: Optional[str] = None
        self._current_edge: Optional[str] = None

    def play_with_visualization(
        self,
        path_id: int,
        edge_name: Optional[str] = None,
    ) -> None:
        """
        Play path with live graph visualization updates.

        Args:
            path_id: Path index in ProblemSolver
            edge_name: Optional edge name being traversed
        """
        client = self.path_player.client
        publisher = self.path_player.publisher
        dt = self.path_player.dt
        speed = self.path_player.speed

        # Mark edge as active
        if edge_name:
            self._current_edge = edge_name
            self.edge_callback(edge_name, False)  # Active, not traversed

        length = self.path_player.end * client.problem.pathLength(path_id)
        t = self.path_player.start * client.problem.pathLength(path_id)

        states = self.graph_builder.get_states()

        print(f"Playing path {path_id} (length: {length:.2f})")

        while t < length:
            start_time = time.time()

            # Get configuration at time t
            q = client.problem.configAtParam(path_id, t)

            # Update robot visualization
            publisher.robotConfig = q
            publisher.publishRobots()
            publisher.client.gui.refresh()

            # Detect and update current state
            current_state = self._detect_state(q, states)
            if current_state and current_state != self._last_state:
                self.state_callback(current_state)
                self._last_state = current_state

            t += dt * speed

            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)

        # Mark edge as traversed
        if edge_name:
            self.edge_callback(edge_name, True)  # Traversed
            print(f"Path {path_id} completed")

    def _detect_state(
        self,
        q: List[float],
        states: Dict[str, int],
    ) -> Optional[str]:
        """
        Detect current state from configuration.

        This uses the constraint graph's applyNodeConstraints to find
        which state the configuration belongs to.
        """
        graph = self.graph_builder.get_graph()
        if graph is None:
            return None

        best_state = None
        best_error = float("inf")

        # Try each state and find the one with minimal projection error
        for state_name in states.keys():
            try:
                success, q_proj, error = graph.applyNodeConstraints(
                    state_name, list(q)
                )
                if success and error < best_error:
                    best_error = error
                    best_state = state_name
            except Exception:
                continue

        # Only return state if error is reasonably small
        if best_state and best_error < 1e-2:
            return best_state

        return None
