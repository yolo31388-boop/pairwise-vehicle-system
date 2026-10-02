"""pytest fixtures shared by the test-suite.

The tests were written unittest-style (``self.assertIsNotNone``) but the
test class does not inherit from ``unittest.TestCase``.  Provide the
assertion helpers here so the suite runs under plain pytest.
"""

import unittest


def _make_assertion(name):
    method = getattr(unittest.TestCase, name)

    def assertion(self, *args, **kwargs):
        case = unittest.TestCase()
        return getattr(case, name)(*args, **kwargs)

    assertion.__name__ = name
    return assertion


def pytest_collection_modifyitems(items):
    from tests.test_vehicle import TestVehicleSystem

    for name in ("assertIsNotNone", "assertIsNone", "assertEqual",
                 "assertTrue", "assertFalse", "assertGreater",
                 "assertLess", "assertAlmostEqual", "assertIn"):
        if not hasattr(TestVehicleSystem, name):
            setattr(TestVehicleSystem, name, _make_assertion(name))
