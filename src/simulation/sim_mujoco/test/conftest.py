"""Make the package importable when the tests run from a source checkout (no install)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
