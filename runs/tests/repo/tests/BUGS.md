# Bugs Found by the Automated Test Suite

Each bug below was discovered while testing the repository against its specifications and docstrings. Per instructions, no project files were modified; each bug is covered by an expected-failure test (`@pytest.mark.xfail(strict=True)` in `tests/py/` or `{ todo: ... }` in `tests/node/`).

---

## Bug 1: `tasks/drone` child sandbox allows opening network sockets on Node 22

- **Files and lines**:
  - `tasks/drone/sim.mjs`, lines 10–13 and 31–33
  - `tasks/drone/host.mjs`, lines 4–10
  - `tasks/drone/check.mjs`, lines 61–70
- **What is wrong**:
  - `tasks/drone/API.md` (lines 69–70) and `tasks/drone/check.mjs` (lines 61–70) require that untrusted controllers running in `host.mjs` have no network access (`ERR_ACCESS_DENIED`), even when bypassing the static `FORBIDDEN` regex filter via string concatenation (`const p = (() => {})['con'+'structor']('return pro'+'cess')()`).
  - On Node 22 (`v22.23.4`), `tasks/drone/sim.mjs:10-13` falls back to `--experimental-permission`, which restricts filesystem (`fs`), child processes (`child_process`), and worker threads (`worker_threads`), but **does not restrict network access** (`net`, `http`, `tls`, `dgram`).
  - Furthermore, `tasks/drone/host.mjs:4-10` does not disable `process.getBuiltinModule` or freeze/remove `Function.prototype.constructor` before importing the controller module. Consequently, a controller that obfuscates `'constructor'` and `'process'` can call `process.getBuiltinModule('net').connect(...)` and open real TCP connections during flight without being stopped.
- **How to reproduce**:
  - Run the built-in self-check:
    ```bash
    node tasks/drone/check.mjs
    ```
    It fails at `tasks/drone/check.mjs:70` with `AssertionError: 'controller error: unexpected network outcome ECONNREFUSED'` instead of `/network correctly denied/` (`ERR_ACCESS_DENIED`).
  - Or run the Node test suite:
    ```bash
    node --test tests/node/drone.test.mjs
    ```
    See test `controller that opens a network socket via obfuscated process.getBuiltinModule("net") is denied` (marked `todo`).

---

## Bug 2: `loop/serve.py` static path filter can be bypassed via dot-segments and percent-encoding

- **File and line**:
  - `loop/serve.py`, lines 57–59
- **What is wrong**:
  - `Handler.do_GET` checks `if u.path.startswith(("/private", "/.git", "/tmp/")) and not u.path.startswith("/tmp/live/"):` on the raw, un-normalized, URL-encoded path `u.path` before passing execution to `super().do_GET()`, where `SimpleHTTPRequestHandler.translate_path()` URL-decodes and normalizes dot-segments (`..`, `.`).
  - As a result:
    1. Any request starting with `/tmp/live/..` (e.g., `GET /tmp/live/../scratch.txt`) satisfies `u.path.startswith("/tmp/live/")`, bypasses the 404 guard, and serves arbitrary files inside `/tmp/`.
    2. Requests with dot-segments or URL-encoded characters such as `GET /runs/../private/secret.txt`, `GET /runs/../.git/HEAD`, or `GET /%70rivate/secret.txt` do not start with `("/private", "/.git", "/tmp/")` in raw `u.path`, so `SimpleHTTPRequestHandler` resolves them and serves files from `/private` and `/.git` with `200 OK`.
- **How to reproduce**:
  - Run:
    ```bash
    pytest tests/py/test_serve.py -k test_traversal_and_url_encoded_requests_to_private_git_tmp_blocked
    ```

---

## Bug 3: `loop/serve.py` does not block `GET /tmp` without a trailing slash

- **File and line**:
  - `loop/serve.py`, line 57
- **What is wrong**:
  - Line 57 checks `u.path.startswith(("/private", "/.git", "/tmp/"))`. Because `"/tmp/"` includes a trailing slash (unlike `"/private"` and `"/.git"`), a request for `GET /tmp` has `u.path == "/tmp"`, which does not match the prefix tuple and falls through to `SimpleHTTPRequestHandler` (returning `301 Moved Permanently` instead of `404 {"error": "not served"}`).
- **How to reproduce**:
  - Run:
    ```bash
    pytest tests/py/test_serve.py -k test_tmp_exact_path_without_trailing_slash_returns_404
    ```

---

## Bug 4: `loop/serve.py` artifact path check uses `str.startswith` without a path separator

- **File and line**:
  - `loop/serve.py`, lines 25–27
- **What is wrong**:
  - `fly()` validates the resolved path using:
    ```python
    if not str(path).startswith(str(ROOT / "runs")) and not str(path).startswith(str(ROOT / "tasks/drone/baselines")):
    ```
  - Because `str(ROOT / "runs")` and `str(ROOT / "tasks/drone/baselines")` do not end with `/` (and `Path.is_relative_to` is not used), a sibling directory sharing the prefix—such as `runs-evil/controller.js` or `tasks/drone/baselines-evil/controller.js`—passes the check even though it is outside `runs/` and `tasks/drone/baselines/`.
- **How to reproduce**:
  - Run:
    ```bash
    pytest tests/py/test_serve.py -k test_api_fly_refuses_sibling_prefix_directory
    ```

---

## Bug 5: `loop/judge.py` omits `verdict` instead of setting `"rebuilds pending"` when an explorer's reproducer directory does not exist yet

- **File and line**:
  - `loop/judge.py`, lines 14–15 and 82–84
- **What is wrong**:
  - The `loop/judge.py` docstring (lines 14–15) specifies:
    > No label is given until at least 6 of 8 recipe rebuilds and 6 of 8 no-recipe rebuilds have produced an artifact; until then the verdict is "rebuilds pending".
  - However, in `loop/judge.py:82-84`:
    ```python
    rp = out["reproducers"].get(ex)
    if not rp:
        continue
    ```
    When 0 recipe rebuilds exist because `repro/<explorer>` has not been created yet (whereas `repro/none` may exist), `judge.py` skips the loop body and leaves `out["explorers"][ex]` without any `"verdict"` key instead of setting `"verdict": "rebuilds pending"`.
- **How to reproduce**:
  - Run:
    ```bash
    pytest tests/py/test_judge.py -k test_verdict_provisional_when_recipe_repro_group_not_created_yet
    ```

---

## Bug 6: `loop/kit.py` `sources()` and `loop/tester.py` `FILES` have mismatched file lists causing `FileNotFoundError`

- **Files and lines**:
  - `loop/kit.py`, lines 9, 20, 172–173, and 230
  - `loop/tester.py`, lines 11–15
- **What is wrong**:
  - `loop/kit.py:172` defines `DRONE_FILES = [..., "raster.py", "view.html", ..., "baselines/stabilizer.js"]` and `loop/kit.py:9` defines `HARNESS_FILES = [..., "view.html", ...]`.
  - In `tasks/drone-hard/`, `raster.py`, `view.html`, and `baselines/stabilizer.js` do not exist in the repository.
  - Furthermore, `loop/tester.py:11-15` constructs the sandbox bundle `FILES` using `glob("*.mjs")` and `glob("*.md")`, omitting `tasks/drone/raster.py` and `harness/view.html`.
  - Consequently, calling `kit.sources("drone", "reproducer")`, `kit.sources("drone-hard", "reproducer")`, or `kit.sources("marble", "reproducer")` on the repository `ROOT` raises `FileNotFoundError` at `loop/kit.py:230` or `loop/kit.py:20`.
- **How to reproduce**:
  - Run:
    ```bash
    pytest tests/py/test_kit.py -k test_sources_all_listed_files_exist_in_repo
    ```
