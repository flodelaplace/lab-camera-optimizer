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
    Locate the bundled configs/ directory.
    After pip install, configs live inside the lab_camera_optimizer package:
      site-packages/lab_camera_optimizer/configs/
    When run locally from the repo, they are at:
      ./configs/  or  ./lab_camera_optimizer/configs/
    """
    import importlib.util

    # 1. Try via the installed lab_camera_optimizer package (pip install path)
    spec = importlib.util.find_spec("lab_camera_optimizer")
    if spec and spec.origin:
        pkg_dir = os.path.dirname(spec.origin)
        candidate = os.path.join(pkg_dir, "configs")
        if os.path.isdir(candidate):
            return candidate

    # 2. Local repo: configs/ next to this file
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in [
        os.path.join(here, "configs"),
        os.path.join(here, "lab_camera_optimizer", "configs"),
        os.path.join(os.path.dirname(here), "configs"),
    ]:
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

