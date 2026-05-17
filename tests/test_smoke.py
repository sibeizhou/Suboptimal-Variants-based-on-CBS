import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CODE_DIR = ROOT / "Final Project" / "code"
sys.path.insert(0, str(CODE_DIR))

from cbs import CBSSolver, detect_collisions
from run_experiments import import_mapf_instance


class SmokeTests(unittest.TestCase):
    def setUp(self):
        instance = CODE_DIR / "instances" / "test_40.txt"
        self.my_map, self.starts, self.goals = import_mapf_instance(str(instance))

    def assert_valid_solution(self, paths):
        self.assertEqual(len(paths), len(self.starts))
        self.assertEqual(detect_collisions(paths), [])
        for agent, path in enumerate(paths):
            self.assertEqual(path[0], self.starts[agent])
            self.assertEqual(path[-1], self.goals[agent])

    def test_cbs_solves_small_instance(self):
        solver = CBSSolver(self.my_map, self.starts, self.goals, ("CBS", 1, 1, 1))
        paths, _ = solver.find_solution(disjoint=False)
        self.assert_valid_solution(paths)

    def test_bounded_cbs_solves_small_instance(self):
        solver = CBSSolver(self.my_map, self.starts, self.goals, ("BCBS", 1, 1.5, 1.5))
        paths, _ = solver.find_solution(disjoint=False)
        self.assert_valid_solution(paths)

    def test_ecbs_solves_small_instance(self):
        solver = CBSSolver(self.my_map, self.starts, self.goals, ("ECBS", 1, 1.5, 1.5))
        paths, _ = solver.find_solution(disjoint=False)
        self.assert_valid_solution(paths)


if __name__ == "__main__":
    unittest.main()
