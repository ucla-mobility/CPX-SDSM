#!/usr/bin/env python3
"""
Experiment runner for V2V simulation.

Runs simulations (Periodic, EventTriggered, Greedy, GreedyBSMImplied, HybridSDSM × seeds) in fast
batch mode (no ROS bridge needed). GreedyROS requires a running ROS 2 bridge.

Usage:
  python run_experiments.py                    # all runs (no bridge needed except GreedyROS)
  python run_experiments.py --algorithm Greedy
  python run_experiments.py --algorithm Greedy --seed 0
  python run_experiments.py --algorithm GreedyROS --seed 0  # needs ROS 2 bridge
  python run_experiments.py --dry-run
"""

import argparse
import atexit
import glob
import json
import os
import signal
import shutil
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ALGORITHMS = ["Periodic", "EventTriggered", "Greedy", "GreedyBSMImplied", "HybridSDSM", "GreedyROS"]
# Algorithms that require the ROS 2 bridge to be running
ROS_ALGORITHMS = {"GreedyROS"}
SEEDS = [0]
SIM_DURATION_S = 300
DEFAULT_NUM_VEHICLES = 400
BRIDGE_UDP_PORT = 50010
DOCKER_BRIDGE_NAME = "veins_ros_bridge"
BRIDGE_START_WAIT_S = 5
PROBE_SIM_DURATION_S = 20  # short run to detect setup failures early
PROBE_TIMEOUT_S = 120  # max wall time for probe


def _root() -> Path:
    return Path(__file__).resolve().parent


def _subprocess_env_with_sumo() -> dict[str, str]:
    """Build env for sim subprocess so SUMO is on PATH (avoids TraCI 'exited with code 1')."""
    env = os.environ.copy()
    sumo_exe = shutil.which("sumo")
    if sumo_exe:
        sumo_dir = str(Path(sumo_exe).resolve().parent)
        env["PATH"] = sumo_dir + os.pathsep + env.get("PATH", "")
        return env
    # Add all common SUMO locations so sim's child (SUMO) is found regardless of script's PATH
    extra = []
    for prefix in (
        "/usr/local/bin",
        "/opt/homebrew/bin",
        os.path.expanduser("~/bin"),
        "/Library/Frameworks/Python.framework/Versions/3.13/bin",
    ):
        if Path(prefix).exists():
            extra.append(prefix)
    if extra:
        env["PATH"] = os.pathsep.join(extra) + os.pathsep + env.get("PATH", "")
    return env


def _get_sumo_path() -> Path | None:
    """Return full path to sumo binary, or None if not found."""
    sumo_exe = shutil.which("sumo")
    if sumo_exe:
        return Path(sumo_exe).resolve()
    for prefix in (
        "/usr/local/bin",
        "/opt/homebrew/bin",
        os.path.expanduser("~/bin"),
        "/Library/Frameworks/Python.framework/Versions/3.13/bin",
    ):
        p = Path(prefix).joinpath("sumo")
        if p.exists():
            return p.resolve()
    return None


def _patch_omnetpp_ini_sumo(sim_dir: Path, sumo_path: Path) -> str | None:
    """Patch omnetpp.ini to use full path to sumo. Returns original content for restore, or None."""
    ini_path = sim_dir / "omnetpp.ini"
    try:
        content = ini_path.read_text(encoding="utf-8")
    except OSError:
        return None
    # Replace commandLine = "sumo with commandLine = "/full/path/sumo (same format as original)
    marker = '*.manager.commandLine = "sumo '
    if marker not in content:
        return None
    path_str = str(sumo_path.resolve())
    new_content = content.replace(marker, f'*.manager.commandLine = "{path_str} ', 1)
    if new_content == content:
        return None
    try:
        ini_path.write_text(new_content, encoding="utf-8")
    except OSError:
        return None
    return content


def _restore_omnetpp_ini(sim_dir: Path, original_content: str | None) -> None:
    """Restore omnetpp.ini to original content."""
    if not original_content:
        return
    try:
        (sim_dir / "omnetpp.ini").write_text(original_content, encoding="utf-8")
    except OSError:
        pass


def _write_sumo_override_ini(sim_dir: Path) -> Path | None:
    """Write sumo_override.ini with full path to sumo so TraCI always finds it. Returns path or None."""
    sumo_path = _get_sumo_path()
    if not sumo_path:
        return None
    override = sim_dir / "sumo_override.ini"
    path_str = str(sumo_path.resolve())
    # OMNeT++ ini: value must be inside double quotes (the quotes are part of the ini syntax, not part of the value)
    cmd = (
        path_str
        + " -c ../sumo/scenario.sumo.cfg --remote-port $port --start --quit-on-end"
    )
    content = f'[General]\n*.manager.commandLine = "{cmd}"\n'
    try:
        override.write_text(content, encoding="utf-8")
        return override
    except OSError:
        return None


# Substrings in stderr that indicate setup failure (probe exits immediately if seen)
PROBE_FAILURE_MARKERS = (
    "Cannot write output scalar file ''",
    "Cannot write output vector file",
    "Cannot write output vector index file",
    "Error launching TraCI server",
    "exited with code 1",
    "set $PATH correctly",
)


def _run_probe(
    binary: Path,
    sim_dir: Path,
    sim_results: Path,
    ned_path: str,
    run_env: dict[str, str],
    sumo_override: Path | None,
    log_fn,
) -> bool:
    """Run a short simulation to verify setup. Return True if OK, False if setup will fail."""
    _clean_config_results(sim_results, "Periodic")
    cmd = [
        str(binary.resolve()),
        "omnetpp.ini",
    ]
    if sumo_override and sumo_override.exists():
        cmd += ["-f", "sumo_override.ini"]
    cmd += [
        "-c",
        "Periodic",
        "-r",
        "0",
        "-u",
        "Cmdenv",
        "-n",
        ned_path,
        f"--sim-time-limit={PROBE_SIM_DURATION_S}s",
    ]
    try:
        result = subprocess.run(
            cmd,
            cwd=str(sim_dir),
            env=run_env,
            capture_output=True,
            text=True,
            timeout=PROBE_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        log_fn(
            "  Probe FAIL: simulation did not finish within "
            + str(PROBE_TIMEOUT_S)
            + "s (setup likely broken)."
        )
        return False
    except Exception as e:
        log_fn("  Probe FAIL: " + str(e))
        return False

    stderr = result.stderr or ""
    for marker in PROBE_FAILURE_MARKERS:
        if marker in stderr:
            log_fn(
                "  Probe FAIL: detected '"
                + marker
                + "' — setup will not work. Aborting to save time."
            )
            if result.stderr:
                log_fn(result.stderr[-1500:])
            return False

    if result.returncode != 0:
        log_fn(
            "  Probe FAIL: simulation exited with code "
            + str(result.returncode)
            + ". Aborting."
        )
        if result.stderr:
            log_fn(result.stderr[-1500:])
        return False

    log_fn(
        "  Probe OK: setup verified in "
        + str(PROBE_SIM_DURATION_S)
        + "s — running full experiments."
    )
    return True


def _kill_stale_sumo() -> None:
    """Kill any leftover sumo processes to free port 9998."""
    try:
        result = subprocess.run(
            ["pkill", "-f", "sumo.*remote-port"], capture_output=True, timeout=5
        )
    except Exception:
        pass
    # Brief pause to let the OS release the port
    time.sleep(0.5)


def _clean_config_results(sim_results: Path, algo: str) -> None:
    """Remove existing OMNeT output files AND CSV logs for this config so the next run can write cleanly."""
    # OMNeT++ scalar/vector files
    for name in (
        f"{algo}-scalars.sca",
        f"{algo}-vectors.vec",
        f"{algo}-vectors.vci",
        f"{algo}-scalars.sca-journal",
        f"{algo}-vectors.vec-journal",
    ):
        p = sim_results / name
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass
    # CSV logs from the app (all seeds for this algo) — stale files prevent clean writes
    for pattern in (f"{algo}-r*-*.csv",):
        for p in sim_results.glob(pattern):
            try:
                p.unlink()
            except OSError:
                pass


def _bridge_already_running() -> bool:
    """True if something is bound to the bridge UDP port (bridge running)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", BRIDGE_UDP_PORT))
        return False
    except OSError:
        return True


def _docker_bridge_running() -> bool:
    """True if our Docker bridge container is already running."""
    try:
        r = subprocess.run(
            ["docker", "ps", "--filter", f"name={DOCKER_BRIDGE_NAME}", "-q"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return bool(r.returncode == 0 and r.stdout.strip())
    except Exception:
        return False


def _start_bridge_docker(root: Path) -> bool:
    """Start ROS 2 bridge in Docker. Return True if we started it (caller must stop)."""
    if _docker_bridge_running():
        return False
    docker = shutil.which("docker")
    if not docker:
        return False
    ws_install = root / "ros2_ws" / "install" / "setup.bash"
    if not ws_install.exists():
        return False
    mount = str(root.resolve())
    cmd = [
        "docker",
        "run",
        "-d",
        "--rm",
        "-v",
        f"{mount}:/workspace",
        "-p",
        f"{BRIDGE_UDP_PORT}:{BRIDGE_UDP_PORT}/udp",
        "-p",
        "50000:50000/udp",
        "--name",
        DOCKER_BRIDGE_NAME,
        "ros:humble-ros-base",
        "bash",
        "-c",
        "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && ros2 run veins_ros_bridge udp_bridge_node",
    ]
    try:
        r = subprocess.run(
            cmd, cwd=str(root), capture_output=True, text=True, timeout=30
        )
        return r.returncode == 0
    except Exception:
        return False


def _start_bridge_native(root: Path) -> subprocess.Popen | None:
    """Start ROS 2 bridge natively (requires ROS 2 and ros2_ws built). Return process or None."""
    ws_setup = root / "ros2_ws" / "install" / "setup.bash"
    if not ws_setup.exists():
        return None
    ros_setup = os.environ.get("ROS_DISTRO")
    if ros_setup:
        ros_setup = f"/opt/ros/{ros_setup}/setup.bash"
    else:
        for distro in ("humble", "jazzy", "iron"):
            p = Path(f"/opt/ros/{distro}/setup.bash")
            if p.exists():
                ros_setup = str(p)
                break
    if not ros_setup or not Path(ros_setup).exists():
        return None
    script = f"source {ros_setup} && source {ws_setup.resolve()} && ros2 run veins_ros_bridge udp_bridge_node"
    try:
        proc = subprocess.Popen(
            ["bash", "-c", script],
            cwd=str(root),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        return proc
    except Exception:
        return None


def _bridge_running() -> bool:
    """True if the ROS 2 bridge is available (port in use or Docker container up)."""
    if _bridge_already_running():
        return True
    if _docker_bridge_running():
        return True
    return False


def ensure_ros2_bridge(root: Path) -> tuple[bool, subprocess.Popen | None]:
    """
    Ensure the ROS 2 UDP bridge is running. Start via Docker or native if needed.
    Returns (we_started_docker, native_proc_or_none).
    Caller must stop: docker stop DOCKER_BRIDGE_NAME or kill native_proc.
    """
    if _bridge_running():
        return (False, None)

    # Try Docker first
    if _start_bridge_docker(root):
        time.sleep(BRIDGE_START_WAIT_S)
        if _bridge_running():
            return (True, None)
        try:
            subprocess.run(
                ["docker", "stop", DOCKER_BRIDGE_NAME], capture_output=True, timeout=10
            )
        except Exception:
            pass

    # Try native
    proc = _start_bridge_native(root)
    if proc is not None:
        time.sleep(BRIDGE_START_WAIT_S)
        if _bridge_running():
            return (False, proc)
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            proc.kill()

    return (False, None)


def stop_ros2_bridge(
    we_started_docker: bool, native_proc: subprocess.Popen | None
) -> None:
    if we_started_docker:
        try:
            subprocess.run(
                ["docker", "stop", DOCKER_BRIDGE_NAME], capture_output=True, timeout=15
            )
        except Exception:
            pass
    if native_proc is not None:
        try:
            native_proc.terminate()
            native_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            native_proc.kill()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run V2V experiments with ROS 2 (Periodic, EventTriggered, Greedy × 3 seeds)."
    )
    parser.add_argument(
        "--algorithm", "-a", choices=ALGORITHMS, help="Run only this algorithm"
    )
    parser.add_argument(
        "--seed", "-s", type=int, choices=SEEDS, help="Run only this seed"
    )
    parser.add_argument("--dry-run", action="store_true", help="Print plan and exit")
    parser.add_argument(
        "--sim-duration",
        type=int,
        default=SIM_DURATION_S,
        help=f"Simulation duration in seconds (default: {SIM_DURATION_S})",
    )
    args = parser.parse_args()

    root = _root()
    binary = root / "src" / "veins_ros_v2v_ucla"
    if not binary.exists():
        for config in ("clang-release", "gcc-release", "release"):
            alt = root / "out" / config / "src" / "veins_ros_v2v_ucla"
            if alt.exists():
                binary = alt
                break
    if sys.platform.startswith("win") and not binary.suffix:
        exe = binary.with_suffix(".exe")
        if exe.exists():
            binary = exe
    sim_dir = root / "simulations"
    sim_results = sim_dir / "results"
    out_root = root / "results"
    log_file = out_root / "experiment_log.txt"

    algorithms = [args.algorithm] if args.algorithm else ALGORITHMS
    seeds = [args.seed] if args.seed is not None else SEEDS
    runs = [(a, s) for a in algorithms for s in seeds]
    if not runs:
        print("No runs selected.", file=sys.stderr)
        return 1

    if not binary.exists():
        print(f"ERROR: Binary not found: {binary}", file=sys.stderr)
        print("Build first: make -C src", file=sys.stderr)
        return 1

    veins_dir = os.environ.get("VEINS_DIR") or str(root.parent / "veins")
    veins_ned = Path(veins_dir) / "src" / "veins"
    ned_extra = ":" + str(veins_ned.resolve()) if veins_ned.exists() else ""
    ned_path = ".:../src:./apps:./networks" + ned_extra

    def log(msg: str) -> None:
        print(msg)
        out_root.mkdir(parents=True, exist_ok=True)
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(msg + "\n")

    if args.dry_run:
        log("DRY RUN — would run:")
        for i, (algo, seed) in enumerate(runs, 1):
            ros_tag = " (ROS bridge required)" if algo in ROS_ALGORITHMS else ""
            log(f"  {i}. {algo} seed={seed}{ros_tag}")
        log(f"Total: {len(runs)} run(s)")
        return 0

    # --- Only require ROS 2 bridge if any run needs it ---
    needs_bridge = any(algo in ROS_ALGORITHMS for algo, _ in runs)
    we_started_docker = False
    native_proc = None

    if needs_bridge:
        we_started_docker, native_proc = ensure_ros2_bridge(root)
        if not _bridge_running():
            stop_ros2_bridge(we_started_docker, native_proc)
            print(
                "ERROR: ROS 2 bridge is required for GreedyROS and could not be started.",
                file=sys.stderr,
            )
            print("", file=sys.stderr)
            print("Options:", file=sys.stderr)
            print(
                "  1. Docker: ensure 'ros2_ws' is built, then run:",
                file=sys.stderr,
            )
            print(
                '     docker run -d --rm -v "$(pwd):/workspace" -p 50010:50010/udp -p 50000:50000/udp \\',
                file=sys.stderr,
            )
            print(
                "       --name veins_ros_bridge ros:humble-ros-base bash -c \\",
                file=sys.stderr,
            )
            print(
                "       'source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && ros2 run veins_ros_bridge udp_bridge_node'",
                file=sys.stderr,
            )
            print(
                "  2. Native: source ROS 2 and ros2_ws, then in another terminal:",
                file=sys.stderr,
            )
            print(
                "     ros2 run veins_ros_bridge udp_bridge_node", file=sys.stderr
            )
            print("     Then re-run this script.", file=sys.stderr)
            return 1

    def cleanup():
        stop_ros2_bridge(we_started_docker, native_proc)

    atexit.register(cleanup)

    if needs_bridge:
        log("ROS 2 bridge is running. Starting experiments.")
    else:
        log("No ROS bridge needed. Starting experiments.")
    out_root.mkdir(parents=True, exist_ok=True)
    sim_results.mkdir(parents=True, exist_ok=True)
    with open(log_file, "a", encoding="utf-8") as f:
        f.write("\n--- " + datetime.now(timezone.utc).isoformat() + " ---\n")

    run_env = _subprocess_env_with_sumo()
    total = len(runs)
    start_all = datetime.now(timezone.utc)
    first_run_elapsed = None

    if not _get_sumo_path():
        log(
            "ERROR: SUMO not found. Install SUMO and ensure 'sumo' is in PATH or in /usr/local/bin, /opt/homebrew/bin, or ~/bin."
        )
        atexit.unregister(cleanup)
        stop_ros2_bridge(we_started_docker, native_proc)
        return 1
    sumo_path = _get_sumo_path()
    log("Using SUMO at " + str(sumo_path) + " (patching omnetpp.ini)")

    original_ini_content = _patch_omnetpp_ini_sumo(sim_dir, sumo_path)
    if original_ini_content is not None:
        atexit.register(lambda: _restore_omnetpp_ini(sim_dir, original_ini_content))

    sumo_override = None

    log("Running short probe (" + str(PROBE_SIM_DURATION_S) + "s) to verify setup...")
    if not _run_probe(
        binary, sim_dir, sim_results, ned_path, run_env, sumo_override, log
    ):
        atexit.unregister(cleanup)
        if original_ini_content is not None:
            _restore_omnetpp_ini(sim_dir, original_ini_content)
        stop_ros2_bridge(we_started_docker, native_proc)
        return 1

    for idx, (algo, seed) in enumerate(runs, 1):
        run_start = datetime.now(timezone.utc)
        log(f"[{idx}/{total}] Running {algo} seed={seed} ...")

        _kill_stale_sumo()
        _clean_config_results(sim_results, algo)

        cmd = [
            str(binary.resolve()),
            "omnetpp.ini",
        ]
        if sumo_override and sumo_override.exists():
            cmd += ["-f", "sumo_override.ini"]
        cmd += [
            "-c",
            algo,
            "-r",
            str(seed),
            "-u",
            "Cmdenv",
            "-n",
            ned_path,
            f"--sim-time-limit={args.sim_duration}s",
        ]
        try:
            result = subprocess.run(
                cmd,
                cwd=str(sim_dir),
                env=run_env,
                capture_output=True,
                text=True,
                timeout=3600,
            )
        except subprocess.TimeoutExpired:
            log("  FAIL: timeout")
            continue
        except Exception as e:
            log(f"  FAIL: {e}")
            continue

        elapsed = (datetime.now(timezone.utc) - run_start).total_seconds()
        if first_run_elapsed is None:
            first_run_elapsed = elapsed

        if result.returncode != 0:
            log(f"  FAIL (exit {result.returncode})")
            if result.stderr:
                log(result.stderr[-2000:])
            continue

        log(f"  OK ({elapsed:.0f}s)")

        dest_dir = out_root / algo / f"seed{seed}"
        dest_dir.mkdir(parents=True, exist_ok=True)
        prefix = f"{algo}-r{seed}"
        patterns = [
            f"{prefix}-rx.csv",
            f"{prefix}-txrx.csv",
            f"{prefix}-tx.csv",
            f"{prefix}-timeseries.csv",
            f"{prefix}-metadata.csv",
            f"{prefix}-summary.csv",
            f"{prefix}-vehicle-summary.csv",
            f"{prefix}-ros-events.jsonl",
            f"{algo}-scalars.sca",
            f"{algo}-vectors.vec",
            f"{algo}-vectors.vci",
        ]
        for name in patterns:
            src = sim_results / name
            if src.exists():
                shutil.copy2(src, dest_dir / name)

        meta = {
            "algorithm": algo,
            "seed": seed,
            "timestamp": run_start.isoformat(),
            "sim_duration_s": args.sim_duration,
            "num_vehicles": DEFAULT_NUM_VEHICLES,
            "config": algo,
            "params": {},
        }
        meta_csv = dest_dir / f"{prefix}-metadata.csv"
        if meta_csv.exists():
            try:
                with open(meta_csv, encoding="utf-8") as f:
                    for line in f:
                        if "," in line and not line.strip().startswith("key"):
                            parts = line.strip().split(",", 1)
                            if len(parts) == 2:
                                k, v = parts[0].strip(), parts[1].strip()
                                if k == "num_vehicles" and v.isdigit():
                                    meta["num_vehicles"] = int(v)
                                elif k == "sim_duration":
                                    try:
                                        meta["sim_duration_s"] = int(float(v))
                                    except ValueError:
                                        pass
            except Exception:
                pass
        with open(dest_dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        remaining = total - idx
        if remaining > 0 and first_run_elapsed:
            log(
                f"  Estimated time remaining: ~{first_run_elapsed * remaining / 60:.0f} min"
            )

    atexit.unregister(cleanup)
    stop_ros2_bridge(we_started_docker, native_proc)
    total_elapsed = (datetime.now(timezone.utc) - start_all).total_seconds()
    log(f"Done. Total time: {total_elapsed/60:.1f} min. Log: {log_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
