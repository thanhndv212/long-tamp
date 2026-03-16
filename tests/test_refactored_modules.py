"""
Minimal tests for the refactored modules.

These tests verify the basic functionality of:
- Interactive menu system (utils/interactive.py)
- CLI argument parsing (cli/__init__.py)
- Path I/O utilities (planning/path_io.py)
"""

import pytest
from unittest.mock import MagicMock, patch
import sys
import tempfile
import os


# =============================================================================
# Test Interactive Menu (utils/interactive.py)
# =============================================================================


class TestInteractiveMenu:
    """Tests for the interactive menu system."""

    def test_import(self):
        """Test that interactive module can be imported."""
        from agimus_spacelab.utils.interactive import interactive_menu

        assert callable(interactive_menu)

    def test_empty_options_returns_empty_list(self):
        """Test that empty options returns empty list."""
        from agimus_spacelab.utils.interactive import interactive_menu

        result = interactive_menu("Test", [], multi_select=False)
        assert result == []

    def test_numbered_menu_fallback(self):
        """Test numbered menu fallback when getch unavailable."""
        from agimus_spacelab.utils.interactive import _numbered_menu

        # Test quit option
        with patch("builtins.input", return_value="q"):
            result = _numbered_menu("Test", ["A", "B", "C"])
            assert result == []

        # Test valid selection
        with patch("builtins.input", return_value="1"):
            result = _numbered_menu("Test", ["A", "B", "C"])
            assert result == [1]

        # Test multi-select
        with patch("builtins.input", return_value="0,2"):
            result = _numbered_menu("Test", ["A", "B", "C"], multi_select=True)
            assert result == [0, 2]

    def test_terminal_control_functions(self):
        """Test terminal control functions don't raise."""
        from agimus_spacelab.utils.interactive import (
            clear_line,
            move_cursor_up,
            hide_cursor,
            show_cursor,
        )

        # These should not raise
        clear_line()
        move_cursor_up(1)
        move_cursor_up(0)
        hide_cursor()
        show_cursor()


# =============================================================================
# Test CLI Arguments (cli/__init__.py)
# =============================================================================


class TestCLIArguments:
    """Tests for CLI argument parsing utilities."""

    def test_import(self):
        """Test that CLI module can be imported."""
        from agimus_spacelab.cli import (
            add_common_arguments,
            add_task_arguments,
            add_advanced_arguments,
            add_grasp_sequence_arguments,
        )

        assert callable(add_common_arguments)
        assert callable(add_task_arguments)

    def test_add_common_arguments(self):
        """Test adding common arguments to parser."""
        import argparse
        from agimus_spacelab.cli import add_common_arguments

        parser = argparse.ArgumentParser()
        add_common_arguments(parser)

        # Test defaults
        args = parser.parse_args([])
        assert args.backend == "corba"
        assert args.no_viz is False
        assert args.solve is False

        # Test explicit values
        args = parser.parse_args(["--backend", "pyhpp", "--no-viz", "--solve"])
        assert args.backend == "pyhpp"
        assert args.no_viz is True
        assert args.solve is True

    def test_add_task_arguments(self):
        """Test adding task arguments to parser."""
        import argparse
        from agimus_spacelab.cli import add_task_arguments

        parser = argparse.ArgumentParser()
        add_task_arguments(parser)

        args = parser.parse_args([])
        assert args.factory is False
        assert args.show_joints is False

        args = parser.parse_args(["--factory", "--show-joints"])
        assert args.factory is True
        assert args.show_joints is True

    def test_parse_grasp_sequence(self):
        """Test parsing grasp sequence strings."""
        from agimus_spacelab.cli import parse_grasp_sequence

        result = parse_grasp_sequence("g1:h1,g2:h2")
        assert result == [("g1", "h1"), ("g2", "h2")]

        # Test with spaces
        result = parse_grasp_sequence("gripper1:handle1 , gripper2:handle2")
        assert result == [("gripper1", "handle1"), ("gripper2", "handle2")]

        # Test invalid format (no colon)
        result = parse_grasp_sequence("invalid")
        assert result == []

    def test_parse_goal_pairs(self):
        """Test parsing goal pairs to state strings."""
        from agimus_spacelab.cli import parse_goal_pairs

        result = parse_goal_pairs(["g1:h1", "g2:h2"])
        assert result == ["g1 grasps h1", "g2 grasps h2"]

        # Test invalid
        result = parse_goal_pairs(["invalid"])
        assert result == []


# =============================================================================
# Test Path I/O (planning/path_io.py)
# =============================================================================


class TestPathIO:
    """Tests for path I/O utilities."""

    def test_import(self):
        """Test that path_io module can be imported."""
        from agimus_spacelab.planning.path_io import (
            PathLoadError,
            load_paths_from_directory,
            replay_paths,
            get_path_files,
            get_num_paths,
        )

        assert callable(load_paths_from_directory)
        assert callable(get_num_paths)

    def test_path_load_error(self):
        """Test PathLoadError exception."""
        from agimus_spacelab.planning.path_io import PathLoadError

        error = PathLoadError("Test message", requires_graph=True)
        assert error.message == "Test message"
        assert error.requires_graph is True

        error = PathLoadError("Simple error")
        assert error.requires_graph is False

    def test_get_path_files_nonexistent_dir(self):
        """Test get_path_files with nonexistent directory."""
        from agimus_spacelab.planning.path_io import get_path_files

        result = get_path_files("/nonexistent/path")
        assert result == {"native": [], "json": []}

    def test_get_path_files_empty_dir(self):
        """Test get_path_files with empty directory."""
        from agimus_spacelab.planning.path_io import get_path_files

        with tempfile.TemporaryDirectory() as tmpdir:
            result = get_path_files(tmpdir)
            assert result == {"native": [], "json": []}

    def test_get_path_files_with_files(self):
        """Test get_path_files finds files correctly."""
        from agimus_spacelab.planning.path_io import get_path_files

        with tempfile.TemporaryDirectory() as tmpdir:
            # Create test files
            open(os.path.join(tmpdir, "phase_01.path"), "w").close()
            open(os.path.join(tmpdir, "phase_02.path"), "w").close()
            open(os.path.join(tmpdir, "phase_01.json"), "w").close()

            result = get_path_files(tmpdir)
            assert len(result["native"]) == 2
            assert len(result["json"]) == 1

    def test_get_num_paths_empty_planner(self):
        """Test get_num_paths with mock planner."""
        from agimus_spacelab.planning.path_io import get_num_paths

        # Test with empty mock
        mock_planner = MagicMock(spec=[])
        result = get_num_paths(mock_planner)
        assert result == 0

        # Test with stored paths list
        mock_planner = MagicMock()
        mock_planner._stored_paths = [1, 2, 3]
        result = get_num_paths(mock_planner)
        assert result == 3

    def test_replay_paths(self):
        """Test replay_paths function."""
        from agimus_spacelab.planning.path_io import replay_paths

        mock_planner = MagicMock()
        mock_planner.play_path = MagicMock()

        result = replay_paths(mock_planner, [0, 1, 2], verbose=False)
        assert result["success"] is True
        assert result["played"] == 3
        assert result["failed"] == []
        assert mock_planner.play_path.call_count == 3

    def test_replay_paths_with_failure(self):
        """Test replay_paths handles failures."""
        from agimus_spacelab.planning.path_io import replay_paths

        mock_planner = MagicMock()
        mock_planner.play_path = MagicMock(
            side_effect=[None, Exception("fail"), None]
        )

        result = replay_paths(mock_planner, [0, 1, 2], verbose=False)
        assert result["success"] is False
        assert result["played"] == 2
        assert result["failed"] == [1]


# =============================================================================
# Test Config Loader (cli/config_loader.py)
# =============================================================================


class TestConfigLoader:
    """Tests for configuration loading utilities."""

    def test_import(self):
        """Test that config_loader module can be imported."""
        from agimus_spacelab.cli.config_loader import (
            load_task_config,
            get_default_config_dir,
        )

        assert callable(load_task_config)
        assert callable(get_default_config_dir)

    def test_get_default_config_dir(self):
        """Test get_default_config_dir returns correct path."""
        from agimus_spacelab.cli.config_loader import get_default_config_dir
        from pathlib import Path

        script_path = Path("/some/script/dir/script.py")
        result = get_default_config_dir(script_path)
        assert result == Path("/some/script/config")


# =============================================================================
# Test Interactive Pickers (cli/interactive_pickers.py)
# =============================================================================


class TestInteractivePickers:
    """Tests for domain-specific interactive pickers."""

    def test_import(self):
        """Test that interactive_pickers module can be imported."""
        from agimus_spacelab.cli.interactive_pickers import (
            select_grasp_pairs,
            select_frozen_arms,
            browse_configurations,
        )

        assert callable(select_grasp_pairs)
        assert callable(select_frozen_arms)
        assert callable(browse_configurations)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
