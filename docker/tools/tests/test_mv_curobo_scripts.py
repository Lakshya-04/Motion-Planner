"""Host syntax check of the cuRobo-pass queue scripts (they need docker, a GPU and the sim queue to run)."""
import subprocess
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]


class MvCuroboScripts(unittest.TestCase):
    def test_bash_syntax(self):
        for name in ("mv_curobo_run.sh", "mv_curobo_queue.sh"):
            r = subprocess.run(["bash", "-n", str(TOOLS / name)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f"{name}: {r.stderr}")

    def test_usage_guard(self):
        r = subprocess.run(["bash", str(TOOLS / "mv_curobo_run.sh")], capture_output=True, text=True, timeout=20)
        self.assertEqual(r.returncode, 2)


if __name__ == "__main__":
    unittest.main()
