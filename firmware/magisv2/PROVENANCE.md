# Vendored MagisV2 firmware

| | |
|---|---|
| Upstream | https://github.com/DronaAviation/MagisV2 |
| Commit | `5982032ca1496779fed16fad537efe0bb7ebfd8d` (2026-05-19, Ashish Jaiswal) |
| Imported | 2026-09-24, with `git archive` of the paths below (byte-identical to upstream) |
| Paths | `LICENSE`, `README.md`, `Makefile`, `PlutoPilot.cpp`, `PlutoPilot.h`, `src/`, `lib/`, `support/` |
| Not imported | `docs/`, `graphify-out/`, `.agent/`, `.github/`, `.DS_Store` files at any depth (documentation / tooling / OS junk, not firmware; the two under `src/` are on disk but git-ignored) |
| License | GPL-3.0-or-later (see `LICENSE`) |

**Do not edit files in this directory.** Host-execution changes are kept
separately as patch files in `simulation_engine/pluto_x_magisv2/patches/`, applied to a copy
in the build tree, so this directory always matches upstream and can be
updated by re-running the import with a newer commit.

To verify the import is unmodified:

```bash
tmp=$(mktemp -d) && git clone -q https://github.com/DronaAviation/MagisV2.git "$tmp/src" && \
git -C "$tmp/src" archive 5982032ca1496779fed16fad537efe0bb7ebfd8d LICENSE README.md Makefile PlutoPilot.cpp PlutoPilot.h src lib support | tar -x -C "$tmp" --exclude=.DS_Store && \
rm -rf "$tmp/src" && diff -r --exclude=PROVENANCE.md --exclude=COLCON_IGNORE --exclude=.DS_Store "$tmp" firmware/magisv2 && echo unmodified
```
