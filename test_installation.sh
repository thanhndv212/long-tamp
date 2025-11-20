#!/bin/bash
# Quick test script to verify agimus_spacelab installation
# Can be run on host or inside Docker container

set -e

echo "=== Agimus Spacelab Installation Test ==="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

test_passed=0
test_failed=0

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo -n "Testing $test_name... "
    if eval "$test_command" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ PASSED${NC}"
        ((test_passed++))
        return 0
    else
        echo -e "${RED}✗ FAILED${NC}"
        ((test_failed++))
        return 1
    fi
}

# Test Python version
echo "1. Checking Python version..."
python3 --version

# Test basic imports
echo ""
echo "2. Testing core imports..."
run_test "agimus_spacelab" "python3 -c 'import agimus_spacelab'"
run_test "agimus_spacelab.utils" "python3 -c 'from agimus_spacelab.utils import xyzrpy_to_xyzquat'"
run_test "agimus_spacelab.config" "python3 -c 'from agimus_spacelab.config import RobotJoints'"
run_test "agimus_spacelab.core" "python3 -c 'from agimus_spacelab.core import ManipulationTask'"

# Test backend availability
echo ""
echo "3. Testing backend availability..."

# Test CORBA availability
if python3 -c "import hpp.corbaserver" > /dev/null 2>&1; then
    echo -e "   CORBA backend: ${GREEN}✓ Available${NC}"
    run_test "agimus_spacelab.corba" "python3 -c 'from agimus_spacelab.corba import CorbaRobot'"
else
    echo -e "   CORBA backend: ${YELLOW}⚠ Not available (optional)${NC}"
fi

# Test PyHPP availability
if python3 -c "import pyhpp" > /dev/null 2>&1; then
    echo -e "   PyHPP backend: ${GREEN}✓ Available${NC}"
    run_test "agimus_spacelab.pyhpp" "python3 -c 'from agimus_spacelab.pyhpp import PyHPPRobot'"
else
    echo -e "   PyHPP backend: ${YELLOW}⚠ Not available (optional)${NC}"
fi

# Test unified API
echo ""
echo "4. Testing unified API..."
run_test "ManipulationPlanner" "python3 -c 'from agimus_spacelab import ManipulationPlanner'"

# Test utilities
echo ""
echo "5. Testing utility functions..."
python3 << 'EOF'
import numpy as np
from agimus_spacelab.utils import xyzrpy_to_xyzquat, xyzrpy_to_se3

# Test coordinate transformation
xyzrpy = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
xyzquat = xyzrpy_to_xyzquat(xyzrpy)
assert len(xyzquat) == 7, "xyzquat should have 7 elements"
assert np.isclose(np.linalg.norm(xyzquat[3:]), 1.0, atol=1e-6), "Quaternion should be unit"
print("   ✓ Coordinate transformations work")

se3 = xyzrpy_to_se3(xyzrpy)
print("   ✓ SE3 transformation works")
EOF

# Test configuration classes
echo ""
echo "6. Testing configuration classes..."
python3 << 'EOF'
from agimus_spacelab.config import (
    RobotJoints,
    InitialConfigurations,
    JointBounds,
    ManipulationConfig,
    RuleGenerator
)

# Test RobotJoints
joints = RobotJoints.all_robot_joints()
print(f"   ✓ Robot has {len(joints)} joints")

# Test InitialConfigurations
q_ur10 = InitialConfigurations.UR10
assert len(q_ur10) == 6, "UR10 should have 6 joints"
print("   ✓ Initial configurations defined")

# Test JointBounds
bounds = JointBounds.freeflyer_bounds()
assert len(bounds) == 14, "Freeflyer bounds should have 14 values"
print("   ✓ Joint bounds work")

# Test ManipulationConfig
grippers = ManipulationConfig.GRIPPERS
print(f"   ✓ Manipulation config has {len(grippers)} grippers")

# Test RuleGenerator
rules = RuleGenerator.generate_grasp_rules(ManipulationConfig)
print(f"   ✓ Rule generator created {len(rules)} rules")
EOF

# Summary
echo ""
echo "=== Test Summary ==="
echo -e "Passed: ${GREEN}${test_passed}${NC}"
echo -e "Failed: ${RED}${test_failed}${NC}"
echo ""

if [ $test_failed -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! Agimus Spacelab is ready to use.${NC}"
    exit 0
else
    echo -e "${YELLOW}⚠ Some tests failed, but core functionality may still work.${NC}"
    exit 1
fi
