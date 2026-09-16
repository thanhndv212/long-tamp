#!/usr/bin/env python3
"""
Interactive menu-driven builder for GraspSequencePlanner runs.

Split out of grasp_sequence.py: this is a CLI/UI orchestration layer over
GraspSequencePlanner, not part of the planning algorithm itself.
"""

from __future__ import annotations

from typing import Any

from .grasp_sequence import GraspSequencePlanner


class InteractiveGraspSequenceBuilder:
    """Interactive builder for grasp sequence planning.

    Provides a menu-driven interface for:
    - Selecting grasps from available pairs
    - Configuring skip phases
    - Selecting frozen arms mode
    - Setting auto-save options
    - Running the sequence planner with interactive resume on failure

    Example:
        >>> builder = InteractiveGraspSequenceBuilder(task, cfg)
        >>> result = builder.run()
        >>> if result["success"]:
        ...     print("Sequence planning succeeded!")
    """

    def __init__(
        self,
        task: Any,
        task_config: Any,
        freeze_joint_substrings: list[str] | None = None,
    ):
        """Initialize interactive grasp sequence builder.

        Args:
            task: ManipulationTask instance with planner, graph_builder, etc.
            task_config: Task configuration with VALID_PAIRS.
            freeze_joint_substrings: Default joint substrings to freeze.
        """
        self.task = task
        self.task_config = task_config
        self.freeze_joint_substrings = freeze_joint_substrings or []
        self.ALL_ARM_KEYWORDS = list(getattr(task_config, "ALL_ARM_KEYWORDS", []))

        # Will be populated during run()
        self.grasp_sequence: list[tuple[str, str]] = []
        self.skip_phases: set[int] = set()
        self.skip_all_phases: bool = False
        self.frozen_arms_mode: str = "auto"
        self.per_phase_frozen_arms: dict[int, list[str]] | None = None
        self.auto_save_dir: str | None = None
        self.non_stop: bool = False

    def _get_available_grasps(self) -> list[tuple[str, str]]:
        """Get all possible grasps from config."""
        all_grasps = []
        valid_pairs = getattr(self.task_config, "VALID_PAIRS", {})
        for gripper, handles in valid_pairs.items():
            for handle in handles:
                all_grasps.append((gripper, handle))
        return all_grasps

    def select_sequence(self) -> bool:
        """Interactively select the grasp sequence.

        Returns:
            True if a sequence was selected, False if cancelled.
        """
        from long_tamp.utils.interactive import interactive_menu

        all_grasps = self._get_available_grasps()
        if not all_grasps:
            print("No valid grasps available in config.")
            return False

        # Format for display
        grasp_options = [f"{gripper} → {handle}" for gripper, handle in all_grasps] + [
            "[Done - Start Planning]",
            "[Done - Start Planning (non stop)]",
        ]

        self.grasp_sequence = []
        done_non_stop_idx = len(all_grasps) + 1

        while True:
            # Show current sequence
            if self.grasp_sequence:
                print("\nCurrent sequence:")
                for i, (g, h) in enumerate(self.grasp_sequence, 1):
                    print(f"  {i}. {g} → {h}")
            else:
                print("\nSequence is empty. Select grasps to add.")

            # Select next grasp
            selected = interactive_menu(
                "Select next grasp to add (or Done to plan):",
                grasp_options,
                multi_select=False,
            )

            if not selected:
                return False

            if selected[0] == len(all_grasps):  # Done - Start Planning
                break
            if selected[0] == done_non_stop_idx:
                self.non_stop = True
                break

            self.grasp_sequence.append(all_grasps[selected[0]])

        return len(self.grasp_sequence) > 0

    def configure_skip_phases(self) -> None:
        """Configure which phases to skip."""
        from long_tamp.cli.interactive_pickers import select_skip_phases

        if len(self.grasp_sequence) >= 1:
            self.skip_phases, self.skip_all_phases = select_skip_phases(
                self.grasp_sequence
            )

    def configure_frozen_arms(self) -> None:
        """Configure frozen arms mode."""
        from long_tamp.cli.interactive_pickers import select_frozen_arms_mode

        self.frozen_arms_mode, self.per_phase_frozen_arms = select_frozen_arms_mode(
            self.grasp_sequence,
            arm_keywords=self.ALL_ARM_KEYWORDS,
        )

    def configure_auto_save(self) -> None:
        """Configure auto-save directory."""
        from long_tamp.cli.interactive_pickers import select_auto_save_directory

        self.auto_save_dir = select_auto_save_directory()

    def _create_arm_selector_callback(self):
        """Create interactive arm selector callback for interactive mode."""
        from long_tamp.utils.interactive import interactive_menu

        def interactive_arm_selector(phase_idx, gripper, arm_keywords):
            """Callback for interactive arm selection per phase."""
            print(f"\n  Select arms to freeze for Phase {phase_idx + 1}:")
            print(f"  Active gripper: {gripper}")

            selected = interactive_menu(
                "Select arm(s) to freeze:",
                arm_keywords + ["[None - No Locking]"],
                multi_select=True,
            )

            arm_count = len(arm_keywords)
            if selected and selected[0] < arm_count:
                return [arm_keywords[i] for i in selected if i < arm_count]
            return []

        return interactive_arm_selector

    def run(self) -> dict[str, Any]:
        """Run the interactive grasp sequence planning workflow.

        Returns:
            Dictionary with 'success' and planning results.
        """
        print("\n=== Interactive Grasp Sequence Planning ===")

        # Ensure task is ready
        if not self._ensure_task_ready():
            return {"success": False, "error": "Task setup failed"}

        # Step 1: Select sequence
        if not self.select_sequence():
            return {"success": False, "error": "No grasps selected"}

        print(f"\nPlanning sequence of {len(self.grasp_sequence)} grasps...")

        # Step 2: Configure options
        self.configure_skip_phases()
        self.configure_frozen_arms()
        self.configure_auto_save()

        if self.non_stop:
            print(
                "Non stop mode enabled: will automatically resume on failure "
                "(Ctrl+C to stop)."
            )

        # Step 3: Get q_init
        q_init = self._get_q_init()
        if q_init is None:
            print("Error: q_init not available.")
            return {"success": False, "error": "q_init not available"}

        # Step 4: Create and run planner
        planner = None
        try:
            planner = GraspSequencePlanner(
                graph_builder=self.task.graph_builder,
                config_gen=self.task.config_gen,
                planner=self.task.planner,
                task_config=self.task_config,
                backend=self.task.backend,
                graph_constraints=getattr(self.task, "_graph_constraints", None),
                auto_save_dir=self.auto_save_dir,
                run_logger=getattr(self.task, "run_logger", None),
            )

            # Set interactive callback if in interactive mode
            if self.frozen_arms_mode == "interactive":
                planner.interactive_arm_selector_callback = (
                    self._create_arm_selector_callback()
                )

            result = planner.plan_sequence(
                grasp_sequence=self.grasp_sequence,
                q_init=q_init,
                frozen_arms_mode=self.frozen_arms_mode,
                per_phase_frozen_arms=self.per_phase_frozen_arms,
                skip_phases=self.skip_phases if self.skip_phases else None,
                verbose=True,
            )

            if result.get("success"):
                print("\n" + "=" * 70)
                print("Sequence planning succeeded!")
                print(planner.get_phase_summary())
                # Offer replay or browse after success
                self._offer_replay_or_browse(planner, q_init)
            else:
                self._handle_failure(planner, q_init)

            # Show saved files summary
            self._show_saved_files_summary(planner)

            return {
                "success": result.get("success", False),
                "planner": planner,
                "result": result,
            }

        except Exception as e:
            print(f"\nSequence planning error: {e}")
            import traceback

            traceback.print_exc()

            # In non-stop mode, auto-resume after failure
            if planner is not None:
                self._handle_failure(planner, q_init)
                self._show_saved_files_summary(planner)

                # Check if auto-resume succeeded
                resume_state = planner.get_resumable_state()
                if resume_state is None:
                    # No resumable state = all phases completed
                    return {
                        "success": True,
                        "planner": planner,
                    }

            return {"success": False, "error": str(e), "planner": planner}

    def _ensure_task_ready(self) -> bool:
        """Ensure task is set up for planning."""
        if not hasattr(self.task, "graph_builder") or self.task.graph_builder is None:
            print("Error: Task graph_builder not initialized.")
            return False
        if not hasattr(self.task, "planner") or self.task.planner is None:
            print("Error: Task planner not initialized.")
            return False
        return True

    def _get_q_init(self) -> list[float] | None:
        """Get initial configuration."""
        if hasattr(self.task, "config_gen") and self.task.config_gen is not None:
            q_init = self.task.config_gen.configs.get("q_init")
            if q_init is not None:
                return q_init
        return getattr(self.task, "q_init", None)

    def _handle_failure(
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
    ) -> None:
        """Handle planning failure with optional resume.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after resume.
        """

        if not hasattr(planner, "get_resumable_state"):
            print("Sequence planning failed.")
            return

        resume_state = planner.get_resumable_state()
        if not resume_state:
            print("Sequence planning failed. No resumable state.")
            return

        print("\n" + "=" * 70)
        print("Planning Failed - Partial Progress Saved")
        print("=" * 70)
        print(
            f"Failed at: Phase {resume_state['phase_idx'] + 1}, "
            f"Edge {resume_state['edge_idx'] + 1}"
        )
        print(f"Completed phases: {resume_state['completed_phases']}")
        print(f"Error: {resume_state['error']}")
        print(planner.get_phase_summary())

        if self.non_stop:
            self._auto_resume_loop(planner, q_init)
        else:
            self._interactive_resume_loop(planner, q_init)

    def _auto_resume_loop(
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
    ) -> None:
        """Auto-resume loop for non-stop mode.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after success.
        """
        print("\nNon stop mode: auto-resuming (Press Ctrl+C to stop)")

        while True:
            resume_state = planner.get_resumable_state()
            if not resume_state:
                break
            try:
                result = planner.resume_sequence(
                    retry_from_edge=-1,
                    timeout_per_edge=300.0,
                    max_iterations_per_edge=1000000,
                    frozen_arms_mode=self.frozen_arms_mode,
                    per_phase_frozen_arms=self.per_phase_frozen_arms,
                    skip_phases=self.skip_phases if self.skip_phases else None,
                    verbose=True,
                )
                if result.get("success"):
                    print("\n" + "=" * 70)
                    print("Resume succeeded!")
                    print(planner.get_phase_summary())
                    # Offer replay after successful resume
                    self._offer_replay_or_browse(planner, q_init)
                    break
            except KeyboardInterrupt:
                print("\nNon stop resume interrupted by user.")
                break
            except Exception as e:
                print(f"\nAuto-resume failed: {e}")

    def _interactive_resume_loop(
        self, planner: GraspSequencePlanner, q_init: list[float] | None = None
    ) -> None:
        """Interactive resume loop with menu options.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for replay/browse after success.
        """
        from long_tamp.utils.interactive import interactive_menu

        while True:
            resume_state = planner.get_resumable_state()
            if not resume_state:
                break

            options = [
                "[R] Replay completed paths",
                "[1] Retry from failed edge",
                "[2] Retry from start of failed phase",
                "[3] Retry with increased timeout",
                "[4] Retry with increased max iterations",
                "[Q] Quit to menu",
            ]

            selected = interactive_menu(
                "Resume Options:",
                options,
                multi_select=False,
            )

            if not selected or selected[0] == 5:  # Quit
                break

            if selected[0] == 0:  # Replay
                print("\nReplaying completed paths...")
                planner.replay_sequence()
                continue

            retry_edge = -1 if selected[0] in [0, 2, 4] else 0
            timeout = 120.0 if selected[0] == 2 else None
            max_iters = 10000 if selected[0] == 3 else None

            try:
                result = planner.resume_sequence(
                    retry_from_edge=retry_edge,
                    timeout_per_edge=timeout,
                    max_iterations_per_edge=max_iters,
                    frozen_arms_mode=self.frozen_arms_mode,
                    per_phase_frozen_arms=self.per_phase_frozen_arms,
                    skip_phases=self.skip_phases if self.skip_phases else None,
                    verbose=True,
                )

                if result.get("success"):
                    print("\n" + "=" * 70)
                    print("Resume succeeded!")
                    print(planner.get_phase_summary())
                    # Offer replay/browse after successful resume
                    self._offer_replay_or_browse(planner, q_init)
                    break
            except Exception as e:
                print(f"\nResume failed: {e}")

    def _collect_generated_configs(
        self, planner: GraspSequencePlanner, q_init: list[float]
    ) -> dict[str, list[float]]:
        """Collect all generated configurations from the planner.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration.

        Returns:
            Dictionary mapping configuration names to configuration vectors.
        """
        generated_configs = {}

        # Add q_init
        generated_configs["q_init (Initial)"] = q_init

        # Collect configs from each phase
        for phase_idx, phase_result in enumerate(planner.phase_results):
            if not phase_result:
                continue

            gripper = phase_result.get("gripper", "")
            handle = phase_result.get("handle", "")
            phase_label = f"Phase {phase_idx + 1}: {gripper} → {handle}"

            # Add edge configs (waypoints)
            edge_sequence = phase_result.get("edges", [])
            for edge_idx, edge_name in enumerate(edge_sequence):
                config_label = f"q_phase{phase_idx}_edge{edge_idx}"
                if planner.config_gen and config_label in planner.config_gen.configs:
                    # Create friendly name from edge name
                    edge_type = "waypoint"
                    if "pregrasp" in edge_name.lower():
                        edge_type = "pregrasp"
                    elif (
                        "grasp" in edge_name.lower() and "pre" not in edge_name.lower()
                    ):
                        edge_type = "grasp"
                    elif "placement" in edge_name.lower():
                        edge_type = "placement"

                    friendly_name = f"{phase_label} - Edge {edge_idx + 1} ({edge_type})"
                    generated_configs[friendly_name] = planner.config_gen.configs[
                        config_label
                    ]

            # Add final config for this phase
            if "final_config" in phase_result:
                final_label = f"{phase_label} - Final"
                generated_configs[final_label] = phase_result["final_config"]

        return generated_configs

    def _offer_replay_or_browse(
        self, planner: GraspSequencePlanner, q_init: list[float] | None
    ) -> None:
        """Offer replay for non-skipped phases or browse for skipped phases.

        Args:
            planner: The GraspSequencePlanner instance.
            q_init: Initial configuration for browsing.
        """
        from long_tamp.cli.interactive_pickers import browse_configurations
        from long_tamp.utils.interactive import interactive_menu

        # Check if all phases were skipped (config generation only)
        if self.skip_all_phases and planner.config_gen and q_init:
            print("\n" + "=" * 70)
            print("Configuration Generation Complete")
            print("=" * 70)

            generated_configs = self._collect_generated_configs(planner, q_init)
            print(
                f"\nGenerated {len(generated_configs)} configurations "
                f"across {len(planner.phase_results)} phases"
            )

            options = [
                "Browse configurations interactively",
                "Skip",
            ]
            selected = interactive_menu(
                "What would you like to do?",
                options,
                multi_select=False,
            )

            if selected and selected[0] == 0:
                browse_configurations(self.task, generated_configs)

        # Offer replay for non-skipped phases
        elif not self.skip_all_phases and planner.phase_results:
            num_phases = len(planner.phase_results)
            total_paths = sum(
                len(pr.get("paths", [])) for pr in planner.phase_results if pr
            )

            options = [
                f"Replay all paths ({total_paths} paths from {num_phases} phases)",
                "Replay individual phase",
                "Skip replay",
            ]
            selected = interactive_menu(
                "Replay completed paths?",
                options,
                multi_select=False,
            )

            if not selected or selected[0] == 2:
                return

            if selected[0] == 0:  # Replay all
                print("\nReplaying all completed paths...")
                planner.replay_sequence()

            elif selected[0] == 1:  # Replay individual phase
                phase_options = [
                    f"Phase {i + 1}: {pr.get('gripper', '')} → {pr.get('handle', '')}"
                    for i, pr in enumerate(planner.phase_results)
                    if pr
                ] + ["Back"]

                while True:
                    phase_selected = interactive_menu(
                        "Select phase to replay:",
                        phase_options,
                        multi_select=False,
                    )

                    if not phase_selected or phase_selected[0] >= len(
                        planner.phase_results
                    ):
                        break

                    phase_idx = phase_selected[0]
                    phase_result = planner.phase_results[phase_idx]
                    if phase_result and "paths" in phase_result:
                        print(f"\nReplaying Phase {phase_idx + 1}...")
                        for path in phase_result["paths"]:
                            try:
                                planner.planner.play_path(path)
                            except Exception as e:
                                print(f"  Failed to replay path: {e}")

    def _show_saved_files_summary(self, planner: GraspSequencePlanner) -> None:
        """Display summary of saved path files.

        Args:
            planner: The GraspSequencePlanner instance.
        """
        import os

        if self.auto_save_dir and hasattr(planner, "get_saved_path_files"):
            saved_files = planner.get_saved_path_files()
            if saved_files:
                print(f"\n=== Saved Path Files ({len(saved_files)} files) ===")
                print(f"Directory: {self.auto_save_dir}")
                for f in saved_files:
                    print(f"  - {os.path.basename(f)}")
                print(
                    "\n\u26a0 Note: These paths contain graph edge constraints "
                    "and can only be"
                )
                print(
                    "  replayed within the same session using "
                    "planner.replay_sequence()."
                )
                print(
                    "  They cannot be loaded in a new session without "
                    "recreating the graph."
                )




__all__ = ["InteractiveGraspSequenceBuilder"]
