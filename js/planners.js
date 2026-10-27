/* ==========================================================================
   MobileRobotSim – Bahnplanungsalgorithmen
   Portierung der Verfahren aus mobRob_planningAlgorithms.m:
   Bug2, A*, RRT, RRT*, RRT (Dubins), Distance Transform, PRM, PRM (geglättet)
   Jeder Planer erhält ctx = { map, raw, start, goal, rng, p } und liefert
   { ok, states:[[x,y,th],...], viz:{...}, info:[...] }.
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { wrapAngle, MinHeap, Dubins } = MRS;
  const SQRT2 = Math.SQRT2;

  /* ------------------------------ Hilfsfunktionen ------------------------------ */

  function makeChecker(map) {
    const step = 0.5 / map.res; // halbe Zellgröße (feiner als ValidationDistance 0.1 m)
    const free = (x, y) => !map.occupiedAt(x, y);
    return {
      free,
      seg(a, b) {
        const dx = b[0] - a[0], dy = b[1] - a[1];
        const n = Math.max(1, Math.ceil(Math.hypot(dx, dy) / step));
        for (let i = 0; i <= n; i++) if (!free(a[0] + (dx * i) / n, a[1] + (dy * i) / n)) return false;
        return true;
      }
    };
  }

  function pathLength(states) {
    let L = 0;
    for (let i = 1; i < states.length; i++) L += Math.hypot(states[i][0] - states[i - 1][0], states[i][1] - states[i - 1][1]);
    return L;
  }

  /** Orientierung aus Segmentrichtung, letzter Zustand erhält die Zielorientierung. */
  function withHeadings(pts, goalTh) {
    const n = pts.length;
    return pts.map((p, i) => [p[0], p[1], i < n - 1 ? Math.atan2(pts[i + 1][1] - p[1], pts[i + 1][0] - p[0]) : goalTh]);
  }

  /** Lineare Interpolation (wie navPath/interpolate), Stützpunkte bleiben erhalten. */
  function densify(states, spacing) {
    if (states.length < 2) return states.slice();
    const out = [];
    for (let i = 0; i < states.length - 1; i++) {
      const a = states[i], b = states[i + 1];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      const n = Math.max(1, Math.round(L / spacing));
      const th = Math.atan2(b[1] - a[1], b[0] - a[0]);
      for (let k = 0; k < n; k++) out.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n, k === 0 ? (a[2] ?? th) : th]);
    }
    out.push(states[states.length - 1].slice());
    return out;
  }

  /** Zufällige Abkürzungen (shortcutPath aus dem MATLAB-Skript). */
  function shortcut(states, chk, trials, rng) {
    let s = states.slice();
    for (let k = 0; k < trials; k++) {
      const n = s.length;
      if (n < 3) break;
      const i = rng.int(0, n - 3);
      const j = rng.int(i + 2, n - 1);
      if (chk.seg(s[i], s[j])) s = s.slice(0, i + 1).concat(s.slice(j));
    }
    return s;
  }

  function finishPath(pts, ctx) {
    const st = withHeadings(pts, ctx.goal[2]);
    st[0][2] = st.length > 1 ? st[0][2] : ctx.start[2];
    return st;
  }

  /* ------------------------------------ Bug2 ----------------------------------- */
  function planBug2(ctx) {
    const { chk, start, goal, p } = ctx;
    const right = p.bug2Dir === 'right';
    const xs = [start[0], start[1]], xg = [goal[0], goal[1]];
    const A = xs[1] - xg[1], B = xg[0] - xs[0], C = xs[0] * xg[1] - xg[0] * xs[1];
    const den = Math.hypot(A, B) || 1;
    const sd = (q) => (A * q[0] + B * q[1] + C) / den;
    const step = p.bug2Step, maxSteps = 15000;
    const pts = [xs.slice()];
    let cur = xs.slice(), mode = 'GTG', hit = [Infinity, Infinity];
    let last = Math.atan2(xg[1] - xs[1], xg[0] - xs[0]);
    let reached = false, steps = 0, hits = 0;
    const scan = (fromDeg, toDeg) => {
      const dir = toDeg >= fromDeg ? 5 : -5;
      for (let a = fromDeg; dir > 0 ? a <= toDeg : a >= toDeg; a += dir) {
        const ang = last + (a * Math.PI) / 180;
        const tp = [cur[0] + step * Math.cos(ang), cur[1] + step * Math.sin(ang)];
        if (chk.free(tp[0], tp[1])) { cur = tp; pts.push(tp); last = ang; return true; }
      }
      return false;
    };
    for (steps = 1; steps <= maxSteps; steps++) {
      const dg = Math.hypot(xg[0] - cur[0], xg[1] - cur[1]);
      if (dg <= step) { pts.push(xg.slice()); reached = true; break; }
      if (mode === 'GTG') {
        const dir = [(xg[0] - cur[0]) / dg, (xg[1] - cur[1]) / dg];
        const nx = [cur[0] + step * dir[0], cur[1] + step * dir[1]];
        last = Math.atan2(dir[1], dir[0]);
        if (chk.free(nx[0], nx[1])) { cur = nx; pts.push(nx); }
        else {
          mode = 'WF'; hit = cur.slice(); hits++;
          if (!(right ? scan(0, -180) : scan(0, 180))) break;
        }
      } else {
        if (!(right ? scan(90, -180) : scan(-90, 180))) break;
        const cs = sd(cur), ps = sd(pts[pts.length - 2]);
        const crossed = Math.sign(cs) !== Math.sign(ps) || Math.abs(cs) < 0.5 * step;
        if (crossed) {
          const d = Math.hypot(xg[0] - cur[0], xg[1] - cur[1]);
          const dHit = Math.hypot(xg[0] - hit[0], xg[1] - hit[1]);
          const dToHit = Math.hypot(cur[0] - hit[0], cur[1] - hit[1]);
          if (d < dHit - 0.2 && dToHit > 0.3) {
            const dir = [(xg[0] - cur[0]) / d, (xg[1] - cur[1]) / d];
            if (chk.free(cur[0] + step * dir[0], cur[1] + step * dir[1])) { mode = 'GTG'; last = Math.atan2(dir[1], dir[0]); }
          }
        }
      }
    }
    return {
      ok: reached,
      states: finishPath(pts, ctx),
      viz: { mline: [xs, xg] },
      info: [`${Math.min(steps, maxSteps)} Schritte à ${step} m`, `${hits} Hindernis-Kontakte`, reached ? 'Ziel erreicht' : 'Ziel nicht erreicht (Abbruch)']
    };
  }

  /* ------------------------------- A* auf dem Raster ---------------------------- */
  function planAStar(ctx) {
    const { map, start, goal, p, chk, rng } = ctx;
    const W = map.W, H = map.H, res = map.res;
    const sx = Math.floor(start[0] * res), sy = Math.floor(start[1] * res);
    const gx = Math.floor(goal[0] * res), gy = Math.floor(goal[1] * res);
    if (map.occupied(sx, sy) || map.occupied(gx, gy)) return { ok: false, states: [], viz: {}, info: ['Start oder Ziel liegt im Hindernis'] };
    const N = W * H;
    const g = new Float64Array(N).fill(Infinity);
    const parent = new Int32Array(N).fill(-1);
    const closed = new Uint8Array(N);
    const heap = new MinHeap();
    const h = (ix, iy) => {
      const dx = Math.abs(ix - gx), dy = Math.abs(iy - gy);
      return dx + dy + (SQRT2 - 2) * Math.min(dx, dy);
    };
    const s = sy * W + sx, goalIdx = gy * W + gx;
    g[s] = 0;
    heap.push(h(sx, sy), s);
    const NB = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, SQRT2], [1, -1, SQRT2], [-1, 1, SQRT2], [-1, -1, SQRT2]];
    let expanded = 0, found = false;
    while (heap.size) {
      const c = heap.pop();
      if (closed[c]) continue;
      closed[c] = 1; expanded++;
      if (c === goalIdx) { found = true; break; }
      const cx = c % W, cy = (c / W) | 0;
      for (const [dx, dy, cost] of NB) {
        const nx = cx + dx, ny = cy + dy;
        if (map.occupied(nx, ny)) continue;
        if (dx && dy && (map.occupied(cx + dx, cy) || map.occupied(cx, cy + dy))) continue; // keine Ecken schneiden
        const ni = ny * W + nx;
        if (closed[ni]) continue;
        const ng = g[c] + cost;
        if (ng < g[ni]) { g[ni] = ng; parent[ni] = c; heap.push(ng + p.astarWeight * h(nx, ny), ni); }
      }
    }
    if (!found) return { ok: false, states: [], viz: { explored: closed, W, H }, info: [`${expanded} Zellen expandiert`, 'Kein Pfad gefunden'] };
    const cells = [];
    for (let c = goalIdx; c >= 0; c = parent[c]) cells.push(c);
    cells.reverse();
    let pts = cells.map((c) => [((c % W) + 0.5) / res, (((c / W) | 0) + 0.5) / res]);
    pts[0] = [start[0], start[1]];
    pts[pts.length - 1] = [goal[0], goal[1]];
    // kollineare Rasterpunkte entfernen
    const red = [pts[0]];
    for (let i = 1; i < pts.length - 1; i++) {
      const a = red[red.length - 1], b = pts[i], c = pts[i + 1];
      const cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
      if (Math.abs(cr) > 1e-9) red.push(b);
    }
    red.push(pts[pts.length - 1]);
    let st = finishPath(red, ctx);
    const info = [`${expanded} von ${N} Zellen expandiert`, `Raster ${W}×${H}, Gewicht ε = ${p.astarWeight}`];
    if (p.astarSmooth) { st = finishPath(shortcut(st, chk, 150, rng), ctx); info.push('Shortcut-Glättung (150 Versuche)'); }
    return { ok: true, states: st, viz: { explored: closed, W, H }, info };
  }

  /* --------------------------- Distance Transform ------------------------------- */
  /** Geodätische Distanz (8-Nachbarschaft, quasi-euklidisch) zur Zielzelle, in Zellen. */
  function geodesic(map, gx, gy) {
    const W = map.W, H = map.H, N = W * H;
    const D = new Float64Array(N).fill(Infinity);
    if (map.occupied(gx, gy)) return D;
    const heap = new MinHeap();
    const done = new Uint8Array(N);
    D[gy * W + gx] = 0;
    heap.push(0, gy * W + gx);
    const NB = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, SQRT2], [1, -1, SQRT2], [-1, 1, SQRT2], [-1, -1, SQRT2]];
    while (heap.size) {
      const c = heap.pop();
      if (done[c]) continue;
      done[c] = 1;
      const cx = c % W, cy = (c / W) | 0;
      for (const [dx, dy, cost] of NB) {
        const nx = cx + dx, ny = cy + dy;
        if (map.occupied(nx, ny)) continue;
        const ni = ny * W + nx, nd = D[c] + cost;
        if (nd < D[ni]) { D[ni] = nd; heap.push(nd, ni); }
      }
    }
    return D;
  }
  MRS.geodesic = geodesic;

  function planDistanceTransform(ctx) {
    const { map, raw, start, goal } = ctx;
    const W = map.W, res = map.res;
    const gx = Math.floor(goal[0] * res), gy = Math.floor(goal[1] * res);
    const sx = Math.floor(start[0] * res), sy = Math.floor(start[1] * res);
    const D = geodesic(map, gx, gy);
    if (!isFinite(D[sy * W + sx])) return { ok: false, states: [], viz: {}, info: ['Start vom Ziel aus nicht erreichbar'] };
    let cx = sx, cy = sy, steps = 0;
    const cells = [[cx, cy]];
    while (Math.hypot(cx - gx, cy - gy) > 1.5 && steps < 15000) {
      steps++;
      let best = Infinity, bx = cx, by = cy;
      for (let dy = -1; dy <= 1; dy++)
        for (let dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          const nx = cx + dx, ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= W || ny >= map.H) continue;
          const v = D[ny * W + nx];
          if (v < best) { best = v; bx = nx; by = ny; }
        }
      if (!isFinite(best)) break;
      cx = bx; cy = by;
      cells.push([cx, cy]);
    }
    let pts = cells.map(([x, y]) => [(x + 0.5) / res, (y + 0.5) / res]);
    pts = pts.filter((q, i) => i === 0 || Math.hypot(q[0] - pts[i - 1][0], q[1] - pts[i - 1][1]) > 1e-4);
    if (Math.hypot(pts[0][0] - start[0], pts[0][1] - start[1]) > 1e-3) pts.unshift([start[0], start[1]]);
    const lp = pts[pts.length - 1];
    if (Math.hypot(lp[0] - goal[0], lp[1] - goal[1]) > 1e-3) pts.push([goal[0], goal[1]]);
    // Anzeige-Distanzfeld auf der nicht aufgeblähten Karte (wie im MATLAB-Skript)
    const Ddisp = geodesic(raw, Math.floor(goal[0] * raw.res), Math.floor(goal[1] * raw.res));
    return {
      ok: true,
      states: finishPath(pts, ctx),
      viz: { distField: Ddisp, W: raw.W, H: raw.H },
      info: [`Gradientenabstieg: ${steps} Zellschritte`, `Distanz Start→Ziel: ${D[sy * W + sx].toFixed(1)} Zellen`]
    };
  }

  /* ------------------------------------ RRT ------------------------------------- */
  function rrtCore(ctx, opt) {
    const { map, chk, start, goal, rng } = ctx;
    const Wm = map.width, Hm = map.height;
    const xs = [start[0]], ys = [start[1]], par = [-1], cost = [0], children = [[]];
    const star = !!opt.star;
    const gamma = 2 * Math.sqrt(1.5) * Math.sqrt((Wm * Hm) / Math.PI);
    let goalIdx = -1, it = 0;
    for (it = 0; it < opt.maxIter && xs.length < opt.maxNodes; it++) {
      let sx, sy;
      if (rng() < opt.goalBias) { sx = goal[0]; sy = goal[1]; } else { sx = rng() * Wm; sy = rng() * Hm; }
      let ni = 0, nd = Infinity;
      for (let i = 0; i < xs.length; i++) {
        const d = (xs[i] - sx) ** 2 + (ys[i] - sy) ** 2;
        if (d < nd) { nd = d; ni = i; }
      }
      nd = Math.sqrt(nd);
      if (nd < 1e-9) continue;
      const f = Math.min(1, opt.maxConn / nd);
      const nw = [xs[ni] + (sx - xs[ni]) * f, ys[ni] + (sy - ys[ni]) * f];
      if (!chk.free(nw[0], nw[1]) || !chk.seg([xs[ni], ys[ni]], nw)) continue;
      let bestPar = ni, bestCost = cost[ni] + Math.hypot(nw[0] - xs[ni], nw[1] - ys[ni]);
      let near = [];
      if (star) {
        const n = xs.length + 1;
        const r = Math.min(opt.maxConn, gamma * Math.sqrt(Math.log(n) / n));
        for (let i = 0; i < xs.length; i++) {
          const d = Math.hypot(xs[i] - nw[0], ys[i] - nw[1]);
          if (d <= r) near.push([i, d]);
        }
        for (const [i, d] of near) {
          if (cost[i] + d < bestCost - 1e-12 && chk.seg([xs[i], ys[i]], nw)) { bestCost = cost[i] + d; bestPar = i; }
        }
      }
      const k = xs.length;
      xs.push(nw[0]); ys.push(nw[1]); par.push(bestPar); cost.push(bestCost); children.push([]);
      children[bestPar].push(k);
      if (star) {
        for (const [i, d] of near) {
          if (i === bestPar) continue;
          if (bestCost + d < cost[i] - 1e-12 && chk.seg(nw, [xs[i], ys[i]])) {
            const old = par[i];
            children[old].splice(children[old].indexOf(i), 1);
            par[i] = k; children[k].push(i);
            const delta = bestCost + d - cost[i];
            const stack = [i];
            while (stack.length) { const q = stack.pop(); cost[q] += delta; for (const c of children[q]) stack.push(c); }
          }
        }
      }
      if (Math.hypot(nw[0] - goal[0], nw[1] - goal[1]) < 0.1) { goalIdx = k; break; }
    }
    const edges = [];
    for (let i = 1; i < xs.length; i++) edges.push([xs[par[i]], ys[par[i]], xs[i], ys[i]]);
    const nodes = xs.map((x, i) => [x, ys[i]]);
    if (goalIdx < 0) return { ok: false, pts: [], edges, nodes, iter: it };
    const pts = [];
    for (let c = goalIdx; c >= 0; c = par[c]) pts.push([xs[c], ys[c]]);
    pts.reverse();
    pts[pts.length - 1] = [goal[0], goal[1]];
    return { ok: true, pts, edges, nodes, iter: it };
  }

  function planRRT(ctx) {
    const { p } = ctx;
    const r = rrtCore(ctx, { maxConn: p.rrtMaxConn, goalBias: p.rrtGoalBias, maxIter: p.rrtMaxIter, maxNodes: p.rrtMaxIter });
    const info = [`${r.nodes.length} Knoten, ${r.iter} Iterationen`, `MaxConnectionDistance = ${p.rrtMaxConn} m`];
    if (!r.ok) return { ok: false, states: [], viz: { edges: r.edges, nodes: r.nodes }, info: info.concat('Kein Pfad gefunden') };
    const st = densify(finishPath(r.pts, ctx), 0.5);
    return { ok: true, states: st, viz: { edges: r.edges, nodes: r.nodes }, info };
  }

  function planRRTStar(ctx) {
    const { p, chk } = ctx;
    let best = null;
    const runs = [];
    for (let k = 0; k < p.rrtsRuns; k++) {
      const rng = MRS.makeRng(p.seed + k);
      const r = rrtCore({ ...ctx, rng }, { star: true, maxConn: p.rrtsMaxConn, goalBias: p.rrtsGoalBias, maxIter: p.rrtsMaxIter, maxNodes: p.rrtsMaxIter });
      if (!r.ok) { runs.push(`Lauf ${k + 1}: kein Pfad`); continue; }
      const pts = shortcut(r.pts, chk, p.rrtsShortcut, rng);
      const L = pathLength(pts);
      runs.push(`Lauf ${k + 1}: ${L.toFixed(2)} m (${r.nodes.length} Knoten)`);
      if (!best || L < best.L) best = { L, pts, r };
    }
    const info = [`Best-of-${p.rrtsRuns} mit Shortcut (${p.rrtsShortcut} Versuche)`].concat(runs);
    if (!best) return { ok: false, states: [], viz: {}, info };
    const st = densify(finishPath(best.pts, ctx), 0.5);
    return { ok: true, states: st, viz: { edges: best.r.edges, nodes: best.r.nodes }, info };
  }

  /* -------------------------------- RRT (Dubins) -------------------------------- */
  function planRRTDubins(ctx) {
    const { map, chk, start, goal, rng, p } = ctx;
    const rho = p.dubinsRadius, maxConn = p.dubinsMaxConn;
    const Wm = map.width, Hm = map.height;
    const nodes = [start.slice()], par = [-1], edgePath = [null];
    const pathFree = (path) => {
      const n = Math.max(2, Math.ceil(path.length / 0.05));
      for (let i = 0; i <= n; i++) { const q = Dubins.sample(path, (path.length * i) / n); if (!chk.free(q[0], q[1])) return false; }
      return true;
    };
    const K = 12;
    let goalIdx = -1, it = 0;
    for (it = 0; it < p.dubinsMaxIter && nodes.length < p.dubinsMaxIter; it++) {
      const smp = rng() < p.dubinsGoalBias ? goal.slice() : [rng() * Wm, rng() * Hm, (rng() * 2 - 1) * Math.PI];
      if (!chk.free(smp[0], smp[1])) continue;
      // K nächste Knoten (euklidisch) als Kandidaten, dann kürzeste Dubins-Distanz
      const cand = [];
      for (let i = 0; i < nodes.length; i++) {
        const d = (nodes[i][0] - smp[0]) ** 2 + (nodes[i][1] - smp[1]) ** 2;
        if (cand.length < K) { cand.push([d, i]); cand.sort((a, b) => a[0] - b[0]); }
        else if (d < cand[K - 1][0]) { cand[K - 1] = [d, i]; cand.sort((a, b) => a[0] - b[0]); }
      }
      let bp = null, bi = -1;
      for (const [, i] of cand) {
        const dp = Dubins.shortest(nodes[i], smp, rho);
        if (dp && (!bp || dp.length < bp.length)) { bp = dp; bi = i; }
      }
      if (!bp || bp.length < 1e-6) continue;
      let target = smp, path = bp;
      if (bp.length > maxConn) { target = Dubins.sample(bp, maxConn); path = Dubins.shortest(nodes[bi], target, rho); }
      if (!path || !chk.free(target[0], target[1]) || !pathFree(path)) continue;
      const k = nodes.length;
      nodes.push(target); par.push(bi); edgePath.push(path);
      if (Math.hypot(target[0] - goal[0], target[1] - goal[1]) < 0.1 && Math.abs(wrapAngle(target[2] - goal[2])) < 0.05) { goalIdx = k; break; }
      const gp = Dubins.shortest(target, goal, rho);
      if (gp && gp.length <= maxConn && pathFree(gp)) {
        nodes.push(goal.slice()); par.push(k); edgePath.push(gp);
        goalIdx = nodes.length - 1;
        break;
      }
    }
    const edges = [];
    for (let i = 1; i < nodes.length; i++) {
      const e = edgePath[i], n = Math.max(2, Math.ceil(e.length / 0.25));
      const poly = [];
      for (let j = 0; j <= n; j++) poly.push(Dubins.sample(e, (e.length * j) / n));
      edges.push(poly);
    }
    const viz = { polyEdges: edges, nodes: nodes.map((q) => [q[0], q[1]]) };
    const info = [`${nodes.length} Knoten, ${it} Iterationen`, `Min. Wenderadius ${rho} m`];
    if (goalIdx < 0) return { ok: false, states: [], viz, info: info.concat('Kein Pfad gefunden') };
    const chain = [];
    for (let c = goalIdx; c > 0; c = par[c]) chain.push(c);
    chain.reverse();
    const st = [start.slice()];
    for (const c of chain) {
      const e = edgePath[c], n = Math.max(1, Math.ceil(e.length / 0.1));
      for (let j = 1; j <= n; j++) st.push(Dubins.sample(e, (e.length * j) / n));
    }
    st[st.length - 1] = goal.slice();
    info.push(`Segmente: ${chain.map((c) => edgePath[c].type).join(' · ')}`);
    return { ok: true, states: st, viz, info };
  }

  /* ------------------------------------ PRM ------------------------------------- */
  function prmTry(ctx, N, conn) {
    const { map, chk, start, goal, rng } = ctx;
    const Wm = map.width, Hm = map.height;
    const xs = [], ys = [];
    let guard = 0;
    while (xs.length < N && guard++ < N * 200) {
      const x = rng() * Wm, y = rng() * Hm;
      if (chk.free(x, y)) { xs.push(x); ys.push(y); }
    }
    // Spatial-Hash für Nachbarschaftssuche
    const cs = conn, gw = Math.ceil(Wm / cs) + 1;
    const buckets = new Map();
    const key = (x, y) => Math.floor(y / cs) * gw + Math.floor(x / cs);
    const insert = (i) => { const k = key(xs[i], ys[i]); if (!buckets.has(k)) buckets.set(k, []); buckets.get(k).push(i); };
    const neighbours = (x, y) => {
      const out = [], bx = Math.floor(x / cs), by = Math.floor(y / cs);
      for (let dy = -1; dy <= 1; dy++)
        for (let dx = -1; dx <= 1; dx++) {
          const b = buckets.get((by + dy) * gw + bx + dx);
          if (b) for (const i of b) out.push(i);
        }
      return out;
    };
    const adj = xs.map(() => []);
    const edges = [];
    for (let i = 0; i < xs.length; i++) insert(i);
    for (let i = 0; i < xs.length; i++) {
      for (const j of neighbours(xs[i], ys[i])) {
        if (j <= i) continue;
        const d = Math.hypot(xs[i] - xs[j], ys[i] - ys[j]);
        if (d > conn) continue;
        if (chk.seg([xs[i], ys[i]], [xs[j], ys[j]])) { adj[i].push([j, d]); adj[j].push([i, d]); edges.push([xs[i], ys[i], xs[j], ys[j]]); }
      }
    }
    const nodes = xs.map((x, i) => [x, ys[i]]);
    // Start und Ziel an die Roadmap anbinden
    const attach = (q) => {
      const idx = xs.length;
      xs.push(q[0]); ys.push(q[1]); adj.push([]);
      for (const j of neighbours(q[0], q[1])) {
        if (j === idx) continue;
        const d = Math.hypot(q[0] - xs[j], q[1] - ys[j]);
        if (d <= conn && chk.seg(q, [xs[j], ys[j]])) { adj[idx].push([j, d]); adj[j].push([idx, d]); }
      }
      insert(idx);
      return idx;
    };
    const s = attach(start), g = attach(goal);
    // A* auf dem Graphen
    const n = xs.length;
    const gc = new Float64Array(n).fill(Infinity), parent = new Int32Array(n).fill(-1), closed = new Uint8Array(n);
    const heap = new MinHeap();
    gc[s] = 0; heap.push(0, s);
    while (heap.size) {
      const c = heap.pop();
      if (closed[c]) continue;
      closed[c] = 1;
      if (c === g) break;
      for (const [j, d] of adj[c]) {
        const ng = gc[c] + d;
        if (ng < gc[j]) { gc[j] = ng; parent[j] = c; heap.push(ng + Math.hypot(xs[j] - goal[0], ys[j] - goal[1]), j); }
      }
    }
    if (!isFinite(gc[g])) return { ok: false, edges, nodes };
    const pts = [];
    for (let c = g; c >= 0; c = parent[c]) pts.push([xs[c], ys[c]]);
    pts.reverse();
    return { ok: true, pts, edges, nodes };
  }

  function planPRM(ctx, smooth) {
    const { p, chk, rng } = ctx;
    const nodeTrials = [p.prmNodes, Math.max(p.prmNodes, 2000), Math.max(p.prmNodes, 3500)];
    const connTrials = [p.prmConn, Math.max(p.prmConn, 4.0), Math.max(p.prmConn, 6.0)];
    const info = [];
    let r = null;
    for (let t = 0; t < nodeTrials.length; t++) {
      r = prmTry(ctx, nodeTrials[t], connTrials[t]);
      info.push(`Versuch ${t + 1}: ${nodeTrials[t]} Knoten, d_max = ${connTrials[t]} m → ${r.ok ? 'Pfad gefunden' : 'kein Pfad'}`);
      if (r.ok) break;
    }
    const viz = { edges: r.edges, nodes: r.nodes };
    if (!r.ok) return { ok: false, states: [], viz, info };
    info.push(`${r.edges.length} Kanten in der Roadmap`);
    let st = finishPath(r.pts, ctx);
    if (smooth) st = shortcut(st, chk, 250, rng);
    st = densify(finishPath(st, ctx), 0.5);
    if (smooth) { st = finishPath(shortcut(st, chk, 150, rng), ctx); info.push('Shortcut-Glättung: 250 + 150 Versuche'); }
    return { ok: true, states: st, viz, info };
  }

  /* --------------------------------- Registry ----------------------------------- */
  MRS.ALGORITHMS = [
    { id: 'bug2', name: 'Bug2', fn: planBug2, params: ['bug2Dir', 'bug2Step'],
      desc: 'Reaktiver Planer: fährt entlang der M-Linie zum Ziel, folgt bei Kontakt der Hinderniskontur, bis die M-Linie näher am Ziel wieder erreicht wird.' },
    { id: 'astar', name: 'A* (Raster)', fn: planAStar, params: ['astarWeight', 'astarSmooth'],
      desc: 'Graphsuche auf dem 8-Nachbarschafts-Raster der aufgeblähten Karte mit Oktil-Heuristik. Optimal für ε = 1.' },
    { id: 'rrt', name: 'RRT', fn: planRRT, params: ['rrtMaxConn', 'rrtGoalBias', 'rrtMaxIter'],
      desc: 'Rapidly-exploring Random Tree: wächst durch zufällige Stichproben, findet schnell einen (nicht optimalen) Pfad.' },
    { id: 'rrtstar', name: 'RRT*', fn: planRRTStar, params: ['rrtsMaxConn', 'rrtsGoalBias', 'rrtsMaxIter', 'rrtsRuns', 'rrtsShortcut'],
      desc: 'RRT mit Elternwahl und Rewiring, Best-of-N-Läufe und Shortcut-Nachbearbeitung (zufällige Abkürzungen zwischen Pfadpunkten).' },
    { id: 'rrtdubins', name: 'RRT (Dubins)', fn: planRRTDubins, params: ['dubinsRadius', 'dubinsMaxConn', 'dubinsGoalBias', 'dubinsMaxIter'],
      desc: 'RRT im Dubins-Zustandsraum: Verbindungen aus Kreisbögen und Geraden mit minimalem Wenderadius, berücksichtigt die Zielorientierung.' },
    { id: 'dt', name: 'Distance Transform', fn: planDistanceTransform, params: ['dtArrows'],
      desc: 'Geodätische Distanztransformation vom Ziel aus; der Pfad folgt dem steilsten Abstieg vom Start.' },
    { id: 'prm', name: 'PRM', fn: (c) => planPRM(c, false), params: ['prmNodes', 'prmConn'],
      desc: 'Probabilistic Roadmap: zufällige freie Knoten werden zu einem Graphen verbunden, darauf wird der kürzeste Weg gesucht.' },
    { id: 'prmsmooth', name: 'PRM (geglättet)', fn: (c) => planPRM(c, true), params: ['prmNodes', 'prmConn'],
      desc: 'PRM-Pfad mit zweistufiger Shortcut-Glättung (250 + 150 Versuche): Zwischenpunkte werden entfernt, wo eine direkte Verbindung kollisionsfrei ist.' }
  ];

  MRS.PARAM_DEFS = {
    bug2Dir: { label: 'Umfahrungsrichtung', type: 'select', options: [['right', 'rechts'], ['left', 'links']], def: 'right' },
    bug2Step: { label: 'Schrittweite [m]', type: 'number', def: 0.15, min: 0.05, max: 0.5, step: 0.01 },
    astarWeight: { label: 'Heuristik-Gewicht ε', type: 'number', def: 1, min: 1, max: 5, step: 0.1 },
    astarSmooth: { label: 'Shortcut-Glättung', type: 'checkbox', def: false },
    rrtMaxConn: { label: 'Max. Verbindungsdistanz [m]', type: 'number', def: 2.5, min: 0.2, max: 10, step: 0.1 },
    rrtGoalBias: { label: 'Goal Bias', type: 'number', def: 0.05, min: 0, max: 1, step: 0.01 },
    rrtMaxIter: { label: 'Max. Iterationen', type: 'number', def: 10000, min: 100, max: 100000, step: 100 },
    rrtsMaxConn: { label: 'Max. Verbindungsdistanz [m]', type: 'number', def: 1.5, min: 0.2, max: 10, step: 0.1 },
    rrtsGoalBias: { label: 'Goal Bias', type: 'number', def: 0.15, min: 0, max: 1, step: 0.01 },
    rrtsMaxIter: { label: 'Max. Iterationen / Knoten', type: 'number', def: 6000, min: 100, max: 50000, step: 100 },
    rrtsRuns: { label: 'Anzahl Läufe (Best-of-N)', type: 'number', def: 3, min: 1, max: 20, step: 1 },
    rrtsShortcut: { label: 'Shortcut-Versuche', type: 'number', def: 150, min: 0, max: 2000, step: 10 },
    dubinsRadius: { label: 'Min. Wenderadius [m]', type: 'number', def: 0.75, min: 0.1, max: 3, step: 0.05 },
    dubinsMaxConn: { label: 'Max. Verbindungslänge [m]', type: 'number', def: 2.5, min: 0.5, max: 10, step: 0.1 },
    dubinsGoalBias: { label: 'Goal Bias', type: 'number', def: 0.1, min: 0, max: 1, step: 0.01 },
    dubinsMaxIter: { label: 'Max. Iterationen', type: 'number', def: 10000, min: 100, max: 100000, step: 100 },
    dtArrows: { label: 'Gradientenpfeile anzeigen', type: 'checkbox', def: true },
    prmNodes: { label: 'Anzahl Knoten', type: 'number', def: 1200, min: 50, max: 10000, step: 50 },
    prmConn: { label: 'Max. Verbindungsdistanz [m]', type: 'number', def: 2.5, min: 0.2, max: 10, step: 0.1 }
  };

  /** Führt einen Planer aus und misst die Planungszeit. */
  MRS.runPlanner = function (algoId, raw, inflated, start, goal, p) {
    const algo = MRS.ALGORITHMS.find((a) => a.id === algoId);
    const ctx = { map: inflated, raw, start, goal, p, rng: MRS.makeRng(p.seed), chk: makeChecker(inflated) };
    const t0 = performance.now();
    let r;
    if (!ctx.chk.free(start[0], start[1]) || !ctx.chk.free(goal[0], goal[1])) {
      r = { ok: false, states: [], viz: {}, info: ['Start oder Ziel liegt im (aufgeblähten) Hindernis'] };
    } else r = algo.fn(ctx);
    const t1 = performance.now();
    r.planTime = (t1 - t0) / 1000;
    r.length = r.states.length > 1 ? pathLength(r.states) : NaN;
    r.algoId = algoId;
    r.algoName = algo.name;
    return r;
  };
  MRS.pathLength = pathLength;
})();
