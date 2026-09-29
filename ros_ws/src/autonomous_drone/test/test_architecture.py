import ast
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / 'autonomous_drone'


class ArchitectureTests(unittest.TestCase):
    def test_console_entrypoint_modules_define_main(self):
        entrypoint_files = [
            PACKAGE_ROOT / 'node_interface.py',
            PACKAGE_ROOT / 'perception' / 'object_detector.py',
            PACKAGE_ROOT / 'perception' / 'object_avoidance.py',
            PACKAGE_ROOT / 'perception' / 'object_following.py',
            PACKAGE_ROOT / 'perception' / 'vslam_node.py',
            PACKAGE_ROOT / 'bridges' / 'sim_bridge.py',
            PACKAGE_ROOT / 'bridges' / 'udp_custom_receiver.py',
        ]
        for path in entrypoint_files:
            with self.subTest(path=path):
                tree = ast.parse(path.read_text(encoding='utf-8'))
                functions = {
                    node.name for node in tree.body if isinstance(node, ast.FunctionDef)
                }
                self.assertIn('main', functions)

    def test_ros_graph_endpoints_are_namespace_relative(self):
        endpoint_methods = {'create_publisher', 'create_subscription', 'create_client'}
        violations = []
        for path in PACKAGE_ROOT.rglob('*.py'):
            if 'tests' in path.parts:
                continue
            tree = ast.parse(path.read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                    continue
                if node.func.attr not in endpoint_methods or len(node.args) < 2:
                    continue
                topic = node.args[1]
                if isinstance(topic, ast.Constant) and isinstance(topic.value, str):
                    if topic.value.startswith('/'):
                        violations.append(f'{path.relative_to(PACKAGE_ROOT)}:{node.lineno}')
        self.assertEqual(violations, [])

    def test_flight_manager_uses_ros_clock_for_control_time(self):
        source = (PACKAGE_ROOT / 'node_interface.py').read_text(encoding='utf-8')
        self.assertNotIn('time.monotonic(', source)
        self.assertNotIn('time.time(', source)


if __name__ == '__main__':
    unittest.main()
