#!/usr/bin/env python3
"""
Test script to verify setInnerPlannerType method is available in TransitionPlanner
"""

# Quick verification that the method exists in the Python stubs
try:
    import hpp_stubs.manipulation._path_planners_idl as ppi

    TransitionPlanner = (
        ppi._0_hpp.manipulation_idl.pathPlanner_idl.TransitionPlanner
    )

    # Check if the method is defined
    objref_class = (
        ppi._0_hpp.manipulation_idl.pathPlanner_idl._objref_TransitionPlanner
    )
    if hasattr(objref_class, "setInnerPlannerType"):
        print("✓ setInnerPlannerType method found in TransitionPlanner objref")
        print(
            f"  Documentation: {TransitionPlanner.setInnerPlannerType__doc__}"
        )
    else:
        print("✗ setInnerPlannerType method NOT found in objref")

    # List all available methods
    methods = [m for m in dir(TransitionPlanner) if not m.startswith("_")]
    print(f"\nAvailable TransitionPlanner methods ({len(methods)}):")
    for method in sorted(methods):
        print(f"  - {method}")

except ImportError as e:
    print(f"✗ Could not import TransitionPlanner: {e}")

print("\n" + "=" * 70)
print("Usage example (in a real CORBA client context):")
print("=" * 70)
print(
    """
from hpp.corbaserver.manipulation import ProblemSolver

ps = ProblemSolver()
# ... set up robot, constraint graph, etc. ...

# Get or create a TransitionPlanner
tp = ps.client.manipulation.problem.createTransitionPlanner()

# Switch to DiffusingPlanner
tp.setInnerPlannerType("DiffusingPlanner")

# Or use BiRRTPlanner
tp.setInnerPlannerType("BiRRTPlanner")

# Available planner types:
# - "DiffusingPlanner"
# - "BiRRTPlanner" 
# - "VisibilityPrmPlanner"
# - "kPRM*"
# - "BiRRT*"
# - "SearchInRoadmap"
"""
)
