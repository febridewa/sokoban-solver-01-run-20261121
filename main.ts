// Sokoban solver: room decomposition + push-optimal A* per room.
//
// Strategy
// --------
// Every level this generator draws is a hallway with 1-4 rooms off it, each
// room holding 3-4 boxes and as many goals, and a box pushed out of a room
// can never come back. So a level's push optimum is the sum of its rooms'
// optima, and each room can be solved on its own with the other rooms' boxes
// standing as walls.
//
// Each room is solved with A* over pushes (a state is the boxes' squares
// plus the player's reachable region, named by its lowest square). The
// heuristic is the cheapest assignment of boxes to goals under shortest
// floor distances (admissible: one push moves one box one square), which
// keeps the search push-optimal while expanding far fewer states than a
// plain breadth-first search. Two deadlocks are pruned, both sound:
//  - a box on a dead square (no push sequence, other boxes ignored, can
//    ever bring it to a goal) is lost;
//  - a box in a frozen 2x2 block of walls and boxes, with a box off its
//    goal in the block, can never move again.
//
// Everything is budgeted by work (node expansions), never by the clock,
// and every tie-break is fixed, so two runs write byte-identical outputs.
// No network, no clock reads, no environment reads as a source of variation.

import { mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (node expansions), never the clock.
const ASTAR_BUDGET = 300_000;
const BFS_BUDGET = 250_000;

// Directions: [dx, dy, step letter, push letter].
const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

// Squares from which a lone box can be pushed onto some goal: pull the box
// back from the goals over open floor (a box at y pushed along d lands on
// x = y + d, which needs the player's square y - d to be floor).
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (let k = 0; k < 4; k++) {
      const d = k === 0 ? -w : k === 1 ? w : k === 2 ? -1 : 1;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// The player's reachable region from `from` with `occupied` in the way.
// Marks squares in `seen` with `stamp`; returns the region's lowest square.
function reachRegion(
  level: Level,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

// Whether the box just pushed to `t` sits in a frozen 2x2 block of walls
// and boxes holding a box off its goal.
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

// Shortest walking distance from every square to each goal, over floor
// (walls only, boxes ignored). BFS from each goal.
function floorDistances(level: Level): Int32Array[] {
  const { w, wall, goals } = level;
  const out: Int32Array[] = [];
  for (const g of goals) {
    const dist = new Int32Array(wall.length).fill(-1);
    const queue = [g];
    dist[g] = 0;
    for (let qi = 0; qi < queue.length; qi++) {
      const p = queue[qi]!;
      for (let k = 0; k < 4; k++) {
        const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
        if (!wall[q] && dist[q] < 0) {
          dist[q] = dist[p]! + 1;
          queue.push(q);
        }
      }
    }
    out.push(dist);
  }
  return out;
}

// All permutations of 0..n-1, in lexicographic order (deterministic).
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Array<boolean>(n).fill(false);
  const rec = (): void => {
    if (cur.length === n) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = false;
    }
  };
  rec();
  return out;
}

// Cheapest assignment of boxes to goals under floor distances; Infinity
// when some box cannot reach any goal.
function assignmentHeuristic(
  boxes: number[],
  dists: Int32Array[],
  perms: number[][],
): number {
  let best = Infinity;
  for (const perm of perms) {
    let sum = 0;
    for (let i = 0; i < boxes.length; i++) {
      const d = dists[perm[i]!]![boxes[i]!]!;
      if (d < 0) {
        sum = Infinity;
        break;
      }
      sum += d;
    }
    if (sum < best) best = sum;
  }
  return best;
}

// Shortest walk from `from` to `to` around walls and boxes, in lower LURD
// letters (direction order u, d, l, r breaks ties).
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const [dx, dy, letter] = DIRS[k]!;
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

interface PushSolution {
  pushes: number;
  moves: string;
}

// Push-optimal A* for one (sub-)level. State: sorted boxes + player's
// region low. Returns null when the budget runs out or no solution exists.
function solvePushAStar(level: Level, maxExpansions: number): PushSolution | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const n = level.boxes.length;
  if (n === 0) return { pushes: 0, moves: '' };
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const dists = floorDistances(level);
  const perms = permutations(n);
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const cap = maxExpansions + 2;
  const boxData = new Int32Array(cap * n);
  const playerData = new Int32Array(cap);
  const parentData = new Int32Array(cap);
  const pushData = new Int32Array(cap); // from | to<<8 | dir<<16
  const gData = new Int32Array(cap);
  const keyData = new Float64Array(cap); // state key (region low + boxes)
  const bestG = new Map<number, number>();
  const hCache = new Map<number, number>();

  const keyOf = (boxes: Int32Array, off: number, low: number): number => {
    let k = low;
    for (let i = 0; i < n; i++) k = k * size + boxes[off + i]!;
    return k;
  };
  const boxKey = (boxes: Int32Array, off: number): number => {
    let k = 0;
    for (let i = 0; i < n; i++) k = k * size + boxes[off + i]!;
    return k;
  };
  const heuristic = (off: number): number => {
    const bk = boxKey(boxData, off);
    const cached = hCache.get(bk);
    if (cached !== undefined) return cached;
    const boxes: number[] = [];
    for (let i = 0; i < n; i++) boxes.push(boxData[off + i]!);
    const h = assignmentHeuristic(boxes, dists, perms);
    hCache.set(bk, h);
    return h;
  };
  const isSolved = (off: number): boolean => {
    for (let i = 0; i < n; i++) if (!goal[boxData[off + i]!]!) return false;
    return true;
  };

  // Binary heap on [f, -g, seq]: lowest f first, deeper first on ties,
  // earliest inserted first after that. Parallel arrays, 1-based.
  const hf = new Float64Array(cap + 1);
  const hg = new Int32Array(cap + 1);
  const hseq = new Int32Array(cap + 1);
  const hidx = new Int32Array(cap + 1);
  let heapSize = 0;
  let seq = 0;
  const heapLess = (a: number, b: number): boolean =>
    hf[a]! < hf[b]! || (hf[a]! === hf[b]! && (hg[a]! > hg[b]! || (hg[a]! === hg[b]! && hseq[a]! < hseq[b]!)));
  const heapPush = (f: number, g: number, idx: number): void => {
    heapSize++;
    hf[heapSize] = f;
    hg[heapSize] = g;
    hseq[heapSize] = seq++;
    hidx[heapSize] = idx;
    let i = heapSize;
    while (i > 1) {
      const p = i >> 1;
      if (!heapLess(i, p)) break;
      const tf = hf[i]!; hf[i] = hf[p]!; hf[p] = tf;
      const tg = hg[i]!; hg[i] = hg[p]!; hg[p] = tg;
      const ts = hseq[i]!; hseq[i] = hseq[p]!; hseq[p] = ts;
      const ti = hidx[i]!; hidx[i] = hidx[p]!; hidx[p] = ti;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = hidx[1]!;
    hf[1] = hf[heapSize]!;
    hg[1] = hg[heapSize]!;
    hseq[1] = hseq[heapSize]!;
    hidx[1] = hidx[heapSize]!;
    heapSize--;
    for (let i = 1; ; ) {
      let c = 2 * i;
      if (c > heapSize) break;
      if (c + 1 <= heapSize && heapLess(c + 1, c)) c++;
      if (!heapLess(c, i)) break;
      const tf = hf[i]!; hf[i] = hf[c]!; hf[c] = tf;
      const tg = hg[i]!; hg[i] = hg[c]!; hg[c] = tg;
      const ts = hseq[i]!; hseq[i] = hseq[c]!; hseq[c] = ts;
      const ti = hidx[i]!; hidx[i] = hidx[c]!; hidx[c] = ti;
      i = c;
    }
    return top;
  };

  // Root state.
  const rootBoxes = level.boxes.slice().sort((a, b) => a - b);
  for (let i = 0; i < n; i++) boxData[i] = rootBoxes[i]!;
  playerData[0] = level.player;
  parentData[0] = -1;
  pushData[0] = -1;
  gData[0] = 0;
  for (const b of rootBoxes) occupied[b] = 1;
  const rootLow = reachRegion(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of rootBoxes) occupied[b] = 0;
  const h0 = heuristic(0);
  if (h0 === Infinity) return null;
  keyData[0] = keyOf(boxData, 0, rootLow);
  bestG.set(keyData[0]!, 0);
  heapPush(h0, 0, 0);
  let count = 1;
  let expanded = 0;
  let found = -1;

  const nextBoxes = new Int32Array(n);
  while (heapSize > 0 && expanded < maxExpansions) {
    const s = heapPop();
    const off = s * n;
    const g = gData[s]!;
    // Stale heap entry: a better path to this state was found since.
    if (bestG.get(keyData[s]!) !== g) continue;
    if (isSolved(off)) {
      found = s;
      break;
    }
    expanded++;
    const boxesOff = off;
    for (let i = 0; i < n; i++) occupied[boxData[boxesOff + i]!] = 1;
    // Region of the player in this state (stamp `here`).
    const here = ++stamp;
    reachRegion(level, occupied, playerData[s]!, seen, here, stack);
    for (let bi = 0; bi < n; bi++) {
      const b = boxData[boxesOff + bi]!;
      for (let k = 0; k < 4; k++) {
        const [dx, dy] = DIRS[k]!;
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozenBlock(level, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        for (let i = 0; i < n; i++) nextBoxes[i] = boxData[boxesOff + i]!;
        nextBoxes[bi] = t;
        // Insertion sort of one moved element (n <= 4, keep it simple).
        const sorted = Array.from(nextBoxes).sort((a, b2) => a - b2);
        const low = reachRegion(level, occupied, b, mark, ++stamp, stack);
        let key = low;
        for (const x of sorted) key = key * size + x;
        const prevBest = bestG.get(key);
        if (prevBest === undefined || g + 1 < prevBest) {
          if (count >= cap) return null;
          bestG.set(key, g + 1);
          const ni = count++;
          const noff = ni * n;
          for (let i = 0; i < n; i++) boxData[noff + i] = sorted[i]!;
          playerData[ni] = b;
          parentData[ni] = s;
          pushData[ni] = b | (t << 8) | (k << 16);
          gData[ni] = g + 1;
          keyData[ni] = key;
          const h = heuristic(noff);
          if (h !== Infinity) heapPush(g + 1 + h, g + 1, ni);
        }
        occupied[t] = 0;
        occupied[b] = 1;
      }
    }
    for (let i = 0; i < n; i++) occupied[boxData[boxesOff + i]!] = 0;
  }
  if (found < 0) return null;

  // Reconstruct the pushes, then the moves (walk + push letter each time).
  const pushes: { from: number; to: number; dir: number }[] = [];
  for (let s = found; s > 0; s = parentData[s]!) {
    const pk = pushData[s]!;
    pushes.push({ from: pk & 0xff, to: (pk >> 8) & 0xff, dir: (pk >> 16) & 0xff });
  }
  pushes.reverse();
  let moves = '';
  let player = level.player;
  const boxes = level.boxes.slice().sort((a, b) => a - b);
  for (const { from, to, dir } of pushes) {
    const d = DIRS[dir]![1] * w + DIRS[dir]![0];
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS[dir]![3];
    player = from;
    boxes[boxes.indexOf(from)] = to;
    boxes.sort((a, b) => a - b);
  }
  return { pushes: pushes.length, moves };
}

// Plain push-BFS fallback (same state model and pruning, no heuristic):
// expands in push order, so the first solution found is push-optimal.
function solvePushBfs(level: Level, maxStates: number): PushSolution | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const n = level.boxes.length;
  if (n === 0) return { pushes: 0, moves: '' };
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const cap = maxStates + 2;
  const boxData = new Int32Array(cap * n);
  const playerData = new Int32Array(cap);
  const parentData = new Int32Array(cap);
  const pushData = new Int32Array(cap);
  const index = new Map<number, number>();

  const rootBoxes = level.boxes.slice().sort((a, b) => a - b);
  for (let i = 0; i < n; i++) boxData[i] = rootBoxes[i]!;
  playerData[0] = level.player;
  parentData[0] = -1;
  pushData[0] = -1;
  for (const b of rootBoxes) occupied[b] = 1;
  const rootLow = reachRegion(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of rootBoxes) occupied[b] = 0;
  let rootKey = rootLow;
  for (const b of rootBoxes) rootKey = rootKey * size + b;
  index.set(rootKey, 0);
  let count = 1;
  let found = rootBoxes.every((b) => goal[b]) ? 0 : -1;
  const nextBoxes = new Int32Array(n);

  for (let head = 0; found < 0 && head < count; head++) {
    const off = head * n;
    for (let i = 0; i < n; i++) occupied[boxData[off + i]!] = 1;
    const here = ++stamp;
    reachRegion(level, occupied, playerData[head]!, seen, here, stack);
    for (let bi = 0; bi < n && found < 0; bi++) {
      const b = boxData[off + bi]!;
      for (let k = 0; k < 4; k++) {
        const [dx, dy] = DIRS[k]!;
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        if (frozenBlock(level, occupied, goal, t)) {
          occupied[t] = 0;
          occupied[b] = 1;
          continue;
        }
        for (let i = 0; i < n; i++) nextBoxes[i] = boxData[off + i]!;
        nextBoxes[bi] = t;
        const sorted = Array.from(nextBoxes).sort((a, b2) => a - b2);
        const low = reachRegion(level, occupied, b, mark, ++stamp, stack);
        let key = low;
        for (const x of sorted) key = key * size + x;
        occupied[t] = 0;
        occupied[b] = 1;
        if (index.has(key)) continue;
        if (count >= cap) {
          for (let i = 0; i < n; i++) occupied[boxData[off + i]!] = 0;
          return null;
        }
        index.set(key, count);
        const ni = count++;
        const noff = ni * n;
        for (let i = 0; i < n; i++) boxData[noff + i] = sorted[i]!;
        playerData[ni] = b;
        parentData[ni] = head;
        pushData[ni] = b | (t << 8) | (k << 16);
        if (sorted.every((x) => goal[x])) {
          found = ni;
          break;
        }
      }
    }
    for (let i = 0; i < n; i++) occupied[boxData[off + i]!] = 0;
  }
  if (found < 0) return null;

  const pushes: { from: number; to: number; dir: number }[] = [];
  for (let s = found; s > 0; s = parentData[s]!) {
    const pk = pushData[s]!;
    pushes.push({ from: pk & 0xff, to: (pk >> 8) & 0xff, dir: (pk >> 16) & 0xff });
  }
  pushes.reverse();
  let moves = '';
  let player = level.player;
  const boxes = level.boxes.slice().sort((a, b) => a - b);
  for (const { from, to, dir } of pushes) {
    const d = DIRS[dir]![1] * w + DIRS[dir]![0];
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS[dir]![3];
    player = from;
    boxes[boxes.indexOf(from)] = to;
    boxes.sort((a, b) => a - b);
  }
  return { pushes: pushes.length, moves };
}

function solveRoom(level: Level): PushSolution | null {
  return solvePushAStar(level, ASTAR_BUDGET) ?? solvePushBfs(level, BFS_BUDGET);
}

interface Room {
  boxes: number[];
  goals: number[];
  done: boolean;
}

// Split a level into rooms: cut the dead squares of the player's row (the
// hallway: a box there reaches no goal) out of the floor; each remaining
// part holding a box is a room. Null when the level is no set of rooms.
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (let k = 0; k < 4; k++) {
        const [dx, dy] = DIRS[k]!;
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function solve(text: string): string {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  if (rooms === null) {
    const solved = solveRoom(level);
    return solved === null ? '' : solved.moves;
  }
  let player = level.player;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveRoom(sub);
    if (solved === null) return '';
    moves += solved.moves;
    // Advance the player to the end of this room's moves.
    for (const ch of solved.moves.toLowerCase()) {
      player += ch === 'u' ? -level.w : ch === 'd' ? level.w : ch === 'l' ? -1 : 1;
    }
    room.done = true;
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  if (!statSync(join(I, id)).isDirectory()) continue;
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
