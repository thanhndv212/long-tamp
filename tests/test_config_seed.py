"""Target generation seeds IK with an unvalidated random draw.

The seed only starts the projection -- the projected result is what gets
collision-checked -- and ConfigGenerator overwrites every object's pose in
the seed anyway. Validating the seed first (rejection-sampling the full
configuration, objects included) cost millions of wasted collision checks
per mission.
"""

from long_tamp.planning.config import ConfigGenerator


class _Planner:
    def __init__(self, accepts_flag=True):
        self.accepts_flag = accepts_flag
        self.calls = []

    def random_config(self, *args, **kwargs):
        if kwargs and not self.accepts_flag:
            raise TypeError("unexpected keyword argument 'validate'")
        self.calls.append(kwargs)
        return [0.0]


def _gen(planner):
    gen = object.__new__(ConfigGenerator)
    gen.planner = planner
    return gen


def test_seed_skips_validation():
    planner = _Planner()
    _gen(planner)._random_seed()
    assert planner.calls == [{"validate": False}]


def test_seed_falls_back_for_backends_without_the_flag():
    planner = _Planner(accepts_flag=False)
    assert _gen(planner)._random_seed() == [0.0]
    assert planner.calls == [{}]
