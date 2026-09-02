"""
Unified API for manipulation planning.

This module provides a backend-agnostic interface for manipulation planning.
"""


def check_backend(backend: str) -> bool:
    """
    Check if a backend is available.

    Args:
        backend: Backend name ('pyhpp')

    Returns:
        bool: True if backend is available

    Raises:
        ValueError: If backend name is invalid
        ImportError: If backend is not available
    """
    if backend not in ["pyhpp"]:
        raise ValueError(f"Invalid backend: {backend}. Must be 'pyhpp'")

    try:
        import pyhpp  # noqa: F401

        return True
    except ImportError:
        raise ImportError("PyHPP backend not available. Install hpp-python.")


def create_planner(backend: str = "pyhpp", **kwargs):
    """
    Create a manipulation planner with the specified backend.

    This factory function returns an instance that inherits from
    PyHPPBackend based on the backend choice.

    Args:
        backend: Backend to use ('pyhpp')
        **kwargs: Additional arguments passed to backend constructor

    Returns:
        Instance of PyHPPPlanner

    Example:
        >>> planner = create_planner(backend="pyhpp")
        >>> planner.load_robot(robot_config)
        >>> planner.solve()
    """
    if not check_backend(backend):
        raise ValueError(
            f"Backend '{backend}' is not available. "
            f"Please install the required dependencies."
        )

    from long_tamp.backends import PyHPPBackend

    class PyHPPPlanner(PyHPPBackend):
        """Planner using PyHPP backend."""

        @property
        def backend(self) -> str:
            return "pyhpp"

    return PyHPPPlanner(**kwargs)


__all__ = [
    "check_backend",
    "create_planner",
]
