# Agent guide

- Everything after the `pre-event` tag is event work. Never add code written before the
  event, and never copy code in from another project.
- The app calls `gemini-4-argon` through the Interactions API with `google-genai` 2.3.0 or
  later. Read the installed Gemini API skill before writing Gemini calls; do not use
  `thinking_budget`, `temperature`, `top_p`, `top_k` or explicit `cached_content`.
- Python lives in `.venv` (`python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`).
- Scratch files go in `tmp/` (gitignored). No API key is ever written to a file.
- Automated tests never call a paid model: replay saved responses instead.
