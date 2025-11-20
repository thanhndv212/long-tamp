"""
Tests for agimus_spacelab core functionality.
"""

import pytest
import numpy as np
from agimus_spacelab import get_available_backends, check_backend


def test_get_available_backends():
    """Test getting available backends."""
    backends = get_available_backends()
    assert isinstance(backends, list)
    # At least one backend should be available in testing environment
    # (this may fail if neither is installed)


def test_check_backend():
    """Test backend availability checking."""
    # Should not raise for valid backend names
    check_backend("corba")
    check_backend("pyhpp")
    
    # Should raise for invalid backend
    with pytest.raises(ValueError):
        check_backend("invalid")


def test_invalid_backend_raises():
    """Test that invalid backend name raises error."""
    with pytest.raises(ValueError, match="Invalid backend"):
        check_backend("foo")


class TestConfigBuilder:
    """Tests for ConfigBuilder utility."""
    
    def test_import(self):
        """Test importing ConfigBuilder."""
        from agimus_spacelab.utils.config_utils import ConfigBuilder
        builder = ConfigBuilder()
        assert builder is not None
    
    def test_add_joint_config(self):
        """Test adding joint configurations."""
        from agimus_spacelab.utils.config_utils import ConfigBuilder
        
        builder = ConfigBuilder()
        builder.add_joint_config([1.0, 2.0, 3.0])
        q = builder.build()
        
        assert len(q) == 3
        np.testing.assert_array_equal(q, [1.0, 2.0, 3.0])
    
    def test_add_multiple_configs(self):
        """Test adding multiple configurations."""
        from agimus_spacelab.utils.config_utils import ConfigBuilder
        
        builder = ConfigBuilder()
        builder.add_joint_config([1.0, 2.0])
        builder.add_joint_config([3.0, 4.0])
        q = builder.build()
        
        assert len(q) == 4
        np.testing.assert_array_equal(q, [1.0, 2.0, 3.0, 4.0])


class TestBoundsManager:
    """Tests for BoundsManager utility."""
    
    def test_freeflyer_bounds(self):
        """Test creating freeflyer bounds."""
        from agimus_spacelab.utils.config_utils import BoundsManager
        
        bounds = BoundsManager.freeflyer_bounds()
        
        # Should have 14 values (3*2 for translation + 4*2 for quaternion)
        assert len(bounds) == 14
    
    def test_revolute_bounds(self):
        """Test creating revolute joint bounds."""
        from agimus_spacelab.utils.config_utils import BoundsManager
        
        bounds = BoundsManager.revolute_bounds(-1.0, 1.0)
        
        assert bounds == [-1.0, 1.0]


class TestTransformUtils:
    """Tests for transformation utilities."""
    
    def test_xyzrpy_to_xyzquat(self):
        """Test RPY to quaternion conversion."""
        from agimus_spacelab.utils import xyzrpy_to_xyzquat
        
        xyzrpy = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        xyzquat = xyzrpy_to_xyzquat(xyzrpy)
        
        assert len(xyzquat) == 7
        # Position should be preserved
        np.testing.assert_array_almost_equal(xyzquat[:3], [1.0, 2.0, 3.0])
    
    def test_normalize_quaternion(self):
        """Test quaternion normalization."""
        from agimus_spacelab.utils import normalize_quaternion
        
        q = [0.5, 0.5, 0.5, 0.5]
        q_norm = normalize_quaternion(q)
        
        # Should be unit quaternion
        assert abs(np.linalg.norm(q_norm) - 1.0) < 1e-10


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
