# Contributing

This repository documents a robot. Every number in it comes from a script that
can be run again, and a check fails the build when a document stops agreeing
with the data. Every change is expected to arrive the same way: measured,
tested, and small enough to review.

---

## The cycle

Work moves in one repeating loop. A change is not finished when it runs; it is
finished when it has been through all six steps.

| Step | What it means | How |
|------|---------------|-----|
| Develop | One thing at a time, small enough to describe in a commit subject | |
| Verify | Does it behave? | Run it, read the output |
| Test | Does it still behave for everyone else? | `docker compose build ci && docker compose run --rm ci` |
| Certify | Do the published numbers still match the data? | `python3 docs/scripts/check_docs.py` |
| Document | The README for what it does, `docs/mathematical-model/` for why | |
| Push | One change, one commit | |

Then start the next one. Prefer several small certified commits over one large
one: a batch that fails leaves nothing reviewable, and the repository is never
supposed to sit in a half-measured state.

---

## Verifying before you push

Run the test suite the way continuous integration runs it:

```bash
docker compose build ci && docker compose run --rm ci
```

**Build it first, every time.** The `ci` service mounts **nothing**, so it runs
the copy of the code that is inside the image. Skip the build and you are
testing whatever you last built — a new test file will not even be collected,
and the run goes green without it. Continuous integration builds the image on
every push, so it never has this problem; only local runs do.

That the service mounts nothing is the whole point. The `dev` service mounts the
workspace over the image, so a file that is missing from the image is still
present in the container, because the mount put it back. A missing file has
passed locally for days that way and failed on the first continuous integration
run. If you only have time for one check, make it this one.

`dev` is still the right service for iterating, because your edits are live:

```bash
docker compose run --rm dev bash
```

Documentation has its own gate, which continuous integration also runs:

```bash
python3 docs/scripts/check_docs.py
```

It fails if a headline number no longer matches the data file it came from; if
any figure in µm or N·m anywhere in the documents is absent from the data,
unless it is listed as a declaration with its reason; if the published gains,
torque shares, inertia table, mass table or kinematic figures disagree with
their data files; if a local link or image is missing; if a heading anchor does
not resolve; or if a path a script computes no longer exists.

What it cannot see is a claim with no number in it. A sentence that interprets
a table has to be re-read against the data by hand whenever the data changes.

---

## Commit messages

Write what a senior engineer would want to read six months later: what changed,
why it changed, and what was measured. The subject line is a sentence in the
imperative, under about 72 characters, with no trailing period. An example from
this repository's history, with the figures it had at the time:

```
Raise the control loop to 1 kHz

Bandwidth was never limited by the gains. The ceiling is the loop rate,
usable to about a quarter of it, so 200 Hz capped the achievable natural
frequency at 40 rad/s. At 1 kHz the same tuning reaches 160 rad/s and the
settled tracking error drops from 61.0 um to 6.3 um against a 300 um line.
```

**The message ends when the body ends.** No trailers of any kind: no
co-author lines, no attribution footers, no "generated with" notices, no links
back to a tool or a session, no emojis. One commit, one author — the person who
made the change. A tool or template that proposes adding such a line does not
change this; the repository convention is the one that applies here.

This is enforced, not merely requested:
`tests/test_repo_style.py::test_no_ai_attribution_in_the_history` fails the
build if such a trailer reaches the history. It skips inside the container,
which has no `.git`, and continuous integration runs it on the runner, which
does. The same rule covers documents and code comments, not only commits.

Check your own branch before pushing:

```bash
git log --format='%B' origin/humble..HEAD | grep -in 'co-authored\|generated with\|session'
git log --format='%an <%ae>' origin/humble..HEAD | sort -u    # one author
```

---

## Branches

| Branch | Role |
|--------|------|
| `humble` | Integration. Changes reach it by pull request, already green |
| `development` | Where work happens |

There is no `main`; the ROS 1 history it held is reachable with
`git show 19fa6ae`. Continuous integration runs on `humble`, `development` and
`feature/**`.

---

## What is not versioned

Some files are deliberately absent, each for a stated reason, and `.gitignore`
carries that reason next to the rule. The short version:

- **Regenerable caches** — `docs/data/*.npz`, `*.npy`, build output. The JSON
  summaries beside them *are* published, because those are the numbers the
  documentation quotes. Keep data loading lazy in the analysis scripts so a
  fresh clone can still import them.
- **Artwork that is not this repository's to publish** — the official ROS logo
  is CC BY-NC and this tree is Apache-2.0, so `fetch_artwork.sh` downloads it
  rather than vendoring it. A personal photograph and two organisations' logos
  are likewise absent; the trajectories derived from them are published, since
  those are the only copy of that work.
- **One large reproducible trajectory** — `ingeniero.npz`, about 4 MB, rebuilt by
  `export_trajectory.py` from an image the repository does carry.

If you add something regenerable, ignore it and write the regeneration command
in the comment, so the next person does not have to guess.

---

## Code style

`colcon test` runs ament's linters — flake8, pep257 and copyright — scoped to
each package. `setup.cfg` holds the flake8 configuration, and every per-file
exception in it carries the reason it exists. There are no exceptions under
`src/`; the ones that exist are for plotting and analysis scripts under `docs/`,
where import order is forced by the matplotlib backend and where `I` is the
inertia tensor rather than a bad variable name.

New files need the Apache-2.0 header. Inside a ROS package `test_copyright`
checks it; under `docs/scripts`, `tools` and `tests`,
`test_repo_style.py::test_every_script_carries_the_licence_header` does.

---

## Tests

| Suite | Runs | Covers |
|-------|------|--------|
| `colcon test` | Inside the workspace | Linters per package, trajectory contents, launch-file contents |
| `pytest tests/` | From the repository root | `docs/scripts` and the repository as a whole — kinematics, dynamics, toolpath, mass properties, collision meshes, the MoveIt configuration, the package architecture, documentation, repository style |

`colcon` never reaches `docs/scripts`, which is why the second suite exists.
A change to the analysis scripts that only runs `colcon test` is untested.

---

## Reporting something

Open an issue with the command you ran, what you expected, what happened, and
the output of `ros2 doctor --report` if it is environmental. If the model moves
in Gazebo but stands still in RViz, check `ros2 topic info /joint_states` first:
the publisher count must be one, and a leftover `joint_state_publisher` is the
usual cause.
