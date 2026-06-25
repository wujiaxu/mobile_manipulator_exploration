# One-Command Test Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one executable foreground Bash script that launches Isaac Sim, MoveIt, and RViz and stops the full stack on `Ctrl+C`.

**Architecture:** A repository-root supervisor script validates dependencies, starts Isaac in its own process group, waits for startup, then runs the existing MoveIt launch with RViz. Signal traps clean both process groups; `--dry-run` validates and prints commands without starting GPU or GUI processes.

**Tech Stack:** Bash, ROS 2 Humble, Isaac Sim 4.5, Python `unittest`

---

### Task 1: Add Launcher Contract

**Files:**
- Create: `isaac_sim/tests/test_start_moveit_test_contract.py`
- Test: `start_moveit_test.sh`

- [x] **Step 1: Write the failing contract**

Require an executable root script containing strict Bash mode, Isaac and MoveIt middleware values, both established commands, `--dry-run`, and cleanup traps. Run the dry run and require resolved Isaac and MoveIt command output.

- [x] **Step 2: Verify the test fails for the missing script**

Run:

```bash
/usr/bin/python3 -m unittest isaac_sim/tests/test_start_moveit_test_contract.py -v
```

Expected: failure reporting that `start_moveit_test.sh` does not exist.

### Task 2: Implement Foreground Supervisor

**Files:**
- Create: `start_moveit_test.sh`

- [x] **Step 1: Implement validation and dry run**

Resolve the workspace from `BASH_SOURCE`, validate ROS/Isaac/workspace setup paths, and print both commands when invoked with `--dry-run`.

- [x] **Step 2: Implement process startup and cleanup**

Use `setsid` for each child process group. Start Isaac with Fast DDS, wait 12 seconds while checking liveness, then start MoveIt/RViz with Cyclone DDS. Trap `INT`, `TERM`, and `EXIT`; send group `SIGINT`, wait up to five seconds, and send group `SIGTERM` only if still alive.

- [x] **Step 3: Make the script executable and run green tests**

Run:

```bash
chmod +x start_moveit_test.sh
bash -n start_moveit_test.sh
/usr/bin/python3 -m unittest isaac_sim/tests/test_start_moveit_test_contract.py -v
./start_moveit_test.sh --dry-run
```

Expected: syntax check succeeds, contracts pass, and dry run prints both resolved commands without starting processes.

### Task 3: Update Project Memory

**Files:**
- Modify: `CODEX_TASKS.md`
- Modify: `progress.md`

- [x] **Step 1: Record the command**

Document `./start_moveit_test.sh`, `Ctrl+C` cleanup, `--dry-run`, and the foreground terminal behavior.

- [x] **Step 2: Run focused regressions**

Run the new contract together with `test_moveit_config_contract.py`. Expected: all tests pass with zero failures.

Repository commits are omitted because this workspace has no usable Git repository.
