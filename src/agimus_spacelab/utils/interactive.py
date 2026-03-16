"""
Interactive terminal menu utilities for agimus_spacelab.

This module provides arrow-key navigable terminal menus with graceful fallback
to numbered selection when TTY features are unavailable.

Example usage:
    >>> from agimus_spacelab.utils.interactive import interactive_menu
    >>> selected = interactive_menu(
    ...     "Select an option:",
    ...     ["Option A", "Option B", "Option C"],
    ...     multi_select=False,
    ... )
    >>> print(f"Selected index: {selected}")
"""

from __future__ import annotations

import sys
from typing import List, Optional, Callable

__all__ = [
    "interactive_menu",
    "clear_line",
    "move_cursor_up",
    "hide_cursor",
    "show_cursor",
]


# =============================================================================
# Terminal Control Primitives
# =============================================================================


def clear_line() -> None:
    """Clear current line in terminal using ANSI escape code."""
    sys.stdout.write("\033[K")


def move_cursor_up(n: int = 1) -> None:
    """Move cursor up n lines using ANSI escape code."""
    if n > 0:
        sys.stdout.write(f"\033[{n}A")


def hide_cursor() -> None:
    """Hide terminal cursor using ANSI escape code."""
    sys.stdout.write("\033[?25l")
    sys.stdout.flush()


def show_cursor() -> None:
    """Show terminal cursor using ANSI escape code."""
    sys.stdout.write("\033[?25h")
    sys.stdout.flush()


# =============================================================================
# Getch Implementation
# =============================================================================


def _get_getch() -> Optional[Callable[[], str]]:
    """
    Get a getch function for reading single characters including escape sequences.

    Returns:
        Callable that reads a single character/escape sequence, or None if unavailable.
    """
    try:
        import tty
        import termios

        def getch() -> str:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                # Handle escape sequences (arrow keys)
                if ch == "\x1b":
                    ch += sys.stdin.read(2)
                return ch
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

        return getch
    except ImportError:
        return None


# =============================================================================
# Menu Functions
# =============================================================================


def _numbered_menu(
    title: str,
    options: List[str],
    multi_select: bool = False,
    selected: Optional[List[int]] = None,
) -> List[int]:
    """
    Fallback numbered menu when arrow keys aren't available.

    Args:
        title: Menu title to display.
        options: List of option strings.
        multi_select: If True, allow multiple selections with comma-separated input.
        selected: Initial selected indices (ignored in numbered mode).

    Returns:
        List of selected indices (empty list if cancelled).
    """
    print(f"\n{title}")
    for i, opt in enumerate(options):
        print(f"  [{i}] {opt}")

    if multi_select:
        print("\nEnter numbers separated by commas (e.g., 0,2,3),")
        print("or 'q' to quit:")
    else:
        print("\nEnter number, or 'q' to quit:")

    try:
        choice = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        return []

    if choice.lower() == "q":
        return []

    try:
        if multi_select:
            indices = [int(x.strip()) for x in choice.split(",")]
            return [i for i in indices if 0 <= i < len(options)]
        else:
            idx = int(choice)
            if 0 <= idx < len(options):
                return [idx]
    except ValueError:
        pass

    print("Invalid selection")
    return []


def interactive_menu(
    title: str,
    options: List[str],
    multi_select: bool = False,
    selected: Optional[List[int]] = None,
) -> List[int]:
    """
    Simple arrow-key menu for terminal selection.

    Provides an interactive menu with arrow-key navigation when TTY features
    are available, falling back to numbered selection otherwise.

    Args:
        title: Menu title to display.
        options: List of option strings to choose from.
        multi_select: If True, allow multiple selections with Space key.
        selected: Initial selected indices (for multi_select mode).

    Returns:
        List of selected indices. Empty list if user quits.
        For single-select mode, returns a single-item list.

    Example:
        >>> # Single selection
        >>> choice = interactive_menu("Pick one:", ["A", "B", "C"])
        >>> if choice:
        ...     print(f"Selected: {choice[0]}")

        >>> # Multi-selection
        >>> choices = interactive_menu(
        ...     "Pick multiple:",
        ...     ["X", "Y", "Z"],
        ...     multi_select=True,
        ...     selected=[0],  # Pre-select first item
        ... )
    """
    if not options:
        return []

    getch = _get_getch()
    if getch is None:
        # Fallback to numbered selection
        return _numbered_menu(title, options, multi_select, selected)

    cursor = 0
    checked = set(selected or [])

    hide_cursor()
    try:
        while True:
            # Print menu
            print(f"\n{title}")
            print("Use ↑/↓ to navigate, ", end="")
            if multi_select:
                print("Space to toggle, Enter to confirm, q to quit")
            else:
                print("Enter to select, q to quit")

            for i, opt in enumerate(options):
                prefix = "→ " if i == cursor else "  "
                if multi_select:
                    check = "[x]" if i in checked else "[ ]"
                    print(f"{prefix}{check} {opt}")
                else:
                    print(f"{prefix}{opt}")

            # Get input
            key = getch()

            # Move cursor up to redraw
            lines_to_clear = len(options) + 3
            move_cursor_up(lines_to_clear)

            if key == "\x1b[A":  # Up arrow
                cursor = (cursor - 1) % len(options)
            elif key == "\x1b[B":  # Down arrow
                cursor = (cursor + 1) % len(options)
            elif key == " " and multi_select:  # Space - toggle
                if cursor in checked:
                    checked.discard(cursor)
                else:
                    checked.add(cursor)
            elif key in ("\r", "\n"):  # Enter
                # Clear menu lines
                for _ in range(lines_to_clear):
                    clear_line()
                    print()
                move_cursor_up(lines_to_clear)

                if multi_select:
                    return sorted(checked)
                return [cursor]
            elif key in ("q", "Q", "\x03"):  # q or Ctrl+C
                for _ in range(lines_to_clear):
                    clear_line()
                    print()
                move_cursor_up(lines_to_clear)
                return []
    finally:
        show_cursor()
