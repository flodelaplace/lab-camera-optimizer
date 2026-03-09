"""
init_project.py
───────────────
Entry point for `lab-camera-init`.
Copies the example config files into the current working directory
so the user can start customising them immediately after pip install.
"""

import os
import shutil
import sys


def _find_bundled_configs():
    """
    Locate the bundled configs/ directory whether the package is:
      - installed via pip  (configs/ sits next to this file in site-packages)
      - run locally from the repo root
    """
    # Same directory as this file
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "configs")
    if os.path.isdir(candidate):
        return candidate

    # One level up (editable install or local run from a sub-dir)
    candidate = os.path.join(os.path.dirname(here), "configs")
    if os.path.isdir(candidate):
        return candidate

    return None


def main():
    bundled_configs = _find_bundled_configs()

    if bundled_configs is None:
        print("ERROR: bundled configs directory not found.")
        print("Please report this issue at:")
        print("  https://github.com/flodelaplace/lab-camera-optimizer/issues")
        sys.exit(1)

    # ── Destination: current working directory ────────────────────────────
    dest_configs = os.path.join(os.getcwd(), "configs")

    print("Lab Camera Optimizer — project initialisation")
    print("=" * 50)
    print(f"Copying example configs to: {dest_configs}\n")

    os.makedirs(dest_configs, exist_ok=True)

    copied = []
    skipped = []
    for fname in sorted(os.listdir(bundled_configs)):
        if not fname.endswith(".yaml"):
            continue
        src = os.path.join(bundled_configs, fname)
        dst = os.path.join(dest_configs, fname)
        if os.path.exists(dst):
            skipped.append(fname)
        else:
            shutil.copy2(src, dst)
            copied.append(fname)

    if copied:
        print("Copied:")
        for f in copied:
            print(f"  configs/{f}")
    if skipped:
        print("\nSkipped (already exist):")
        for f in skipped:
            print(f"  configs/{f}  ← not overwritten")

    # ── Create outputs directory ──────────────────────────────────────────
    outputs_dir = os.path.join(os.getcwd(), "outputs")
    os.makedirs(outputs_dir, exist_ok=True)
    print(f"\nCreated: outputs/")

    print("\nDone! Next steps:")
    print("  1. Edit a config:    cp configs/example_simple.yaml configs/my_lab.yaml")
    print("  2. Preview room:     lab-camera-preview   --config configs/my_lab.yaml")
    print("  3. Run optimiser:    lab-camera-optimizer --config configs/my_lab.yaml")


if __name__ == "__main__":
    main()

