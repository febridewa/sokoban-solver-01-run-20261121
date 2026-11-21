# Sokoban Solver — push-optimal, room by room

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/febridewa/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/febridewa/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the levels this competition draws:
a hallway with 1–4 rooms off it, 3–4 boxes and goals per room. `main.ts`
(Node 22, no dependencies) answers all 5 instances of a set in one run,
in well under a second.

## How it works

- **Room decomposition.** The hallway's dead squares are cut out of the
  floor and each remaining part holding a box is a room. A box pushed out
  of a room can never come back, so a level's push optimum is the sum of
  its rooms' optima and each room is solved on its own, with the other
  rooms' boxes standing as walls.
- **Push A\* per room.** A state is the boxes' squares plus the player's
  reachable region (named by its lowest square). The heuristic is the
  cheapest assignment of boxes to goals under shortest floor distances —
  admissible, since one push moves one box one square — so the search stays
  push-optimal while expanding far fewer states than plain breadth-first
  search.
- **Sound deadlock pruning.** A box on a dead square (no push sequence,
  other boxes ignored, can ever bring it to a goal) is lost, and a box in
  a frozen 2×2 block of walls and boxes holding a box off its goal can
  never move again. Both prunings never cut a solvable state.
- **Deterministic by construction.** Search is budgeted by work (node
  expansions), never by the clock, and every tie-break is fixed, so two
  runs write byte-identical outputs.

## Results

Practice set: **1,000,000** — every instance solved in exactly P\* pushes
(13 / 16 / 29 / 47 / 60), in ~0.4 s for the whole set. The same score on
seven further sets drawn from other seeds (all 5 instances at P\* each),
so the solver generalizes to the unseen final set rather than memorizing
the practice levels. The house baseline (plain-astar 1.0.0) scores
200,000 on the practice set.

## Run it

    mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts

(`practice/` is in the kit, not in this repository.) Each instance's
answer is written to `out/<id>/solution.txt` as LURD moves.

Referral: `6etrfz79`

## License

MIT — see [LICENSE](LICENSE).
