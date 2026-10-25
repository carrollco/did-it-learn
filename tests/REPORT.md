# Automated Test Suite Report

## Overview

- **Total tests**: **56**
  - **Python (`pytest` in `tests/py/`)**: **32 tests** — **27 passed**, **5 xfailed** (`strict=True`)
  - **Node (`node:test` in `tests/node/`)**: **24 tests** (across 4 suites) — **23 passed**, **1 todo** (expected failure)
- **External calls**: **0** (all Gemini client calls, token counters, and external subprocesses/secrets are faked or isolated; a synthetic `HELDOUT_SECRET` is used in all tests).

## Commands to Run the Suite

From `/workspace/repo`:

```bash
# Run all Python tests
python3 -m pytest tests/py -v

# Run all Node tests
node --test tests/node/drone.test.mjs
```

## Coverage by Test File

### 1. `tests/py/test_judge.py` (7 tests: 6 passed, 1 xfailed)
Covers `loop/judge.py`:
- **`test_verdict_rule_in_docstring_order`**: Verifies all three final verdicts (`"gains did not survive compression"`, `"nothing to certify"`, `"score certified"`) and confirms that `predicted_inflation > 2` takes precedence over `recipe_value <= 2` when both hold.
- **`test_verdict_boundary_at_tolerance_2`**: Tests exact `TOLERANCE = 2` boundary behavior (`predicted_inflation == 2.0` passes inflation check; `recipe_value == 2.0` triggers `"nothing to certify"`).
- **`test_verdict_stays_provisional_until_6_of_8_in_each_group`**: Confirms verdicts stay `"rebuilds pending"` when either the explorer's recipe group or the `none` control group has `< 6` valid rebuilds out of 8, and resolve once both reach 6 of 8.
- **`test_verdict_provisional_when_recipe_repro_group_not_created_yet` (`xfail`)**: Exposes Bug 5 (`loop/judge.py:83-84`), where a missing `repro/<explorer>` directory causes `verdict` to be omitted instead of `"rebuilds pending"`.
- **`test_unfinished_rebuilds_excluded_never_counted_as_zero`**: Verifies rebuilds with missing artifacts (`"no scene"`) or invalid artifacts (`valid: False`) are excluded from `val_median`, `test_median`, `val_mean`, `test_mean`, `rebuild_scores_val`, and `rebuild_scores_test`, and never counted as zero.
- **`test_artifact_memorized_needs_gap_of_8_or_more`**: Verifies `artifact_memorized` is `True` at a held-out gap of `8.0` and `60.5`, and `False` at `7.5` and below.
- **`test_best_round_round1_inflation_and_score_cache`**: Tests multi-round best-round selection (ties broken by later round), `round1_val` and `predicted_inflation_vs_round1`, `heldout_gap_trials`, `scores-cache.json` reuse, and `heldout_secret_sha256`.

### 2. `tests/py/test_compress.py` (6 tests: 6 passed)
Covers `loop/compress.py` and `loop/reproduce.py`:
- **`test_count_subtracts_empty_string_token_overhead`**: Replaces the Gemini client with a fake counter and verifies `compress.count()` subtracts the empty-string token overhead from the recipe's raw token count.
- **`test_over_budget_recipe_is_refused_then_shortened_recipe_accepted`**: Confirms recipes exceeding `--budget` are rejected with prompt feedback and retried until within budget.
- **`test_all_attempts_over_budget_marked_false_and_refused_by_reproduce`**: Verifies that 4 over-budget attempts produce `within_budget: False` and cause `loop/reproduce.py` to exit with an error.
- **`test_recipes_with_4_or_more_digit_numbers_are_refused`**: Verifies recipes containing 4+ digit numbers (e.g., `1024` or 10-digit seeds) are flagged in `leak` and rejected, whereas 1–3 digit numbers pass.
- **`test_persistent_4_digit_leak_marks_within_budget_false`**: Confirms a short recipe that still contains a 4-digit number after 4 tries has `within_budget: False`.
- **`test_explorer_record_selects_latest_best_round`**: Tests `explorer_record()` transcript formatting and tie-breaking across rounds.

### 3. `tests/py/test_serve.py` (14 tests: 11 passed, 3 xfailed)
Covers `loop/serve.py`:
- **`test_studied_seeds_refused_by_api_fly`**: Confirms `/api/fly` refuses all 16 validation seeds and all 4 smoke seeds (`101..104`) with HTTP `400`.
- **`test_non_integer_seed_returns_400` (6 parametrized cases)**: Confirms non-integer seeds (`abc`, `12.34`, `NaN`, `1e6`, `true`, `101px`) return HTTP `400`.
- **`test_api_fly_refuses_artifacts_outside_runs_and_baselines`**: Confirms `/api/fly` refuses paths outside `runs/` and `tasks/drone/baselines/`, including `../` traversal and symlinks inside `runs/` or `baselines/` that resolve outside.
- **`test_api_fly_refuses_sibling_prefix_directory` (`xfail`)**: Exposes Bug 4 (`loop/serve.py:25`), where `str(path).startswith(str(ROOT / "runs"))` allows sibling directories like `runs-evil/`.
- **`test_api_fly_succeeds_for_valid_controller_and_unstudied_seed`**: Verifies valid live flights return `200` with `{seed, result, track}`.
- **`test_private_git_and_tmp_blocked_except_tmp_live`**: Confirms `/private/...`, `/.git/...`, and `/tmp/...` return `404` while `/tmp/live/...` returns `200`.
- **`test_tmp_exact_path_without_trailing_slash_returns_404` (`xfail`)**: Exposes Bug 3 (`loop/serve.py:57`), where `GET /tmp` returns `301` instead of `404`.
- **`test_traversal_and_url_encoded_requests_to_private_git_tmp_blocked` (`xfail`)**: Exposes Bug 2 (`loop/serve.py:57`), where `/tmp/live/../scratch.txt`, `/runs/../private/secret.txt`, `/runs/../.git/HEAD`, and `/%70rivate/secret.txt` bypass the raw `u.path` check.
- **`test_byte_range_requests_return_206_and_correct_content_range`**: Tests prefix (`0-9`), middle (`10-25`), single-byte (`0-0`), open-ended (`20-`), suffix (`-12`), and out-of-bounds (`416`) byte-range requests.

### 4. `tests/py/test_kit.py` (5 tests: 4 passed, 1 xfailed)
Covers `loop/kit.py`:
- **`test_heldout_seeds_deterministic_and_64_distinct`**: Verifies `heldout_seeds` is deterministic, returns 64 distinct integers in `[100000, 900100000)`, varies with the secret, and is disjoint from validation seeds.
- **`test_sources_strips_validation_seeds_for_reproducers`**: Verifies `sources()` removes all 16 validation seeds from `task.mjs` for `drone` and `drone-hard` reproducers (and omits `val.mjs` for `marble` reproducers) while keeping smoke seeds and explorer validation seeds.
- **`test_sources_all_listed_files_exist_in_repo` (`xfail`)**: Exposes Bug 6 (`loop/kit.py:20,172,230` and `loop/tester.py:11-15`), where missing `raster.py`, `view.html`, and `tasks/drone-hard/baselines/stabilizer.js` cause `FileNotFoundError`.
- **`test_trials_passes_heldout_seeds_via_stdin_not_argv`**: Verifies `kit.trials()` sends held-out seeds on `stdin` via `--seeds-stdin` and never exposes them in `argv`.
- **`test_cost_and_polling_helpers_with_fake_client`**: Tests `kit.cost()`, `kit.run_interaction()`, and `kit.wait_interaction()` with a fake client and simulated network blips.

### 5. `tests/node/drone.test.mjs` (24 tests: 23 passed, 1 todo)
Covers `tasks/drone/sim.mjs`, `tasks/drone/host.mjs`, `tasks/drone/task.mjs`, and `tasks/drone/run.mjs`:
- **Determinism & physics parameters (3 tests)**: Identical `result` and `track` across repeated runs of passing (`stabilizer.js` on seed 104) and failing controllers; deterministic `trialParameters` and `initialState`.
- **Source validation & forbidden tokens (3 tests)**: Rejects all 14 forbidden tokens, rejects files `> 256 KiB`, and cleanly fails invalid exports.
- **Sandbox security & resource limits (7 tests: 6 passed, 1 todo)**:
  - Obfuscated controllers attempting to read files (`fs.readFileSync`), write files (`fs.writeFileSync`), read `process.env` (verified empty `{}` even when parent holds `HELDOUT_SECRET`), spawn child processes (`child_process.spawnSync`), or start workers (`worker_threads.Worker`) are stopped and scored as failed flights without crashing the judge.
  - Infinite loop (`while (true) {}`) is killed within the 50 ms step timeout and scored as a failure.
  - Obfuscated network socket test (`todo`) exposes Bug 1 (`tasks/drone/sim.mjs:10-13` & `tasks/drone/host.mjs:4-9`), where Node 22 `--experimental-permission` allows TCP connections via `process.getBuiltinModule('net')`.
- **Malformed actions & CLI runner (11 tests)**: Tests 9 malformed action shapes (`null`, `undefined`, array, primitive, missing fields, `NaN`, `Infinity`, `-Infinity`, string number), corrupted `stdout` protocol output, and `tasks/drone/run.mjs` CLI options (`--seeds`, `--seeds-stdin`, `--json`, and invalid seed handling).

## Bugs Found

See `tests/BUGS.md` for full details and reproduction steps:
1. **`tasks/drone/sim.mjs:10-13` & `tasks/drone/host.mjs:4-9`**: Node 22 `--experimental-permission` does not restrict network sockets, allowing obfuscated controllers to open TCP connections via `process.getBuiltinModule('net')` (also causing `tasks/drone/check.mjs:70` to fail).
2. **`loop/serve.py:57`**: `u.path.startswith(("/private", "/.git", "/tmp/"))` checks raw `u.path` before URL-unquoting and dot-segment normalization, allowing `/tmp/live/../scratch.txt`, `/runs/../private/...`, `/runs/../.git/...`, and `/%70rivate/...` to serve forbidden files.
3. **`loop/serve.py:57`**: The prefix tuple uses `"/tmp/"` with a trailing slash instead of `"/tmp"`, so `GET /tmp` returns `301` instead of `404`.
4. **`loop/serve.py:25`**: `str(path).startswith(str(ROOT / "runs"))` lacks a trailing path separator, permitting sibling directories such as `runs-evil/`.
5. **`loop/judge.py:83-84`**: When `repro/<explorer>` does not exist yet, `judge.py` skips setting `verdict` instead of setting `"rebuilds pending"` as documented.
6. **`loop/kit.py:20,172,230` & `loop/tester.py:11-15`**: `DRONE_FILES` and `HARNESS_FILES` reference `raster.py`, `view.html`, and `tasks/drone-hard/baselines/stabilizer.js` which are missing from `tasks/drone-hard/` and omitted from `tester.py`'s `FILES` list.
