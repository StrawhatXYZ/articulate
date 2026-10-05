"""Make the in-source lab_simulation package importable from the tests."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
