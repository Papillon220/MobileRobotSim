/* ==========================================================================
   MobileRobotSim – Lidar-Simulation und 2D-SLAM
   Angelehnt an mobRob_Lidar.m:
   - Lidar-Raycasting auf der wahren Belegungskarte
   - Occupancy-Grid-Mapping (Log-Odds, Bresenham-Strahlen)
   - Scan-to-Map-Matching über ein Distanzfeld (Grid-Search grob/fein)
   - EKF-Posenschätzung: Prädiktion mit verrauschter Odometrie,
     Korrektur mit der Scan-Matching-Pose (Gating, begrenzte Innovation)
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { wrapAngle } = MRS;

  /* ------------------------------- Karten ------------------------------------ */
  /** Demo-Szene aus createDemoScene() in mobRob_Lidar.m (x ∈ [0,10], y ∈ [-4,6] → y + 4). */
  function buildDemoScene(res) {
    const m = new MRS.GridMap(10, 10, res);
    const R = (x0, x1, y0, y1) => m.fillRect(x0, y0 + 4, x1, y1 + 4);
    R(0.0, 10.0, -4.0, -3.82); R(0.0, 10.0, 5.82, 6.0); R(0.0, 0.18, -4.0, 6.0); R(9.82, 10.0, -4.0, 6.0);
    R(2.10, 5.70, -0.15, 0.15); R(5.55, 5.85, -0.15, 4.40); R(1.35, 2.10, 1.95, 2.25);
    R(2.10, 7.70, 4.35, 4.65); R(7.45, 7.75, 1.55, 4.65);
    return m;
  }

  MRS.SLAM_MAPS = [
    {
      id: 'demo', name: 'Demo-Szene', res: 20,
      desc: 'Raum 10 m × 10 m mit Innenwänden; vordefinierte Rundfahrt entlang der Außenwände.',
      build: buildDemoScene, start: [9, 2, Math.PI / 2],
      route: [[9, 2], [9, 9.2], [1.2, 9.2], [1.2, 2], [5, 2]]
    }
  ].concat(MRS.MAPS.map((m) => ({
    id: m.id, name: m.name, desc: m.desc, res: 10, build: m.build, route: m.route,
    // mit vordefinierter Route: Start am ersten Routenpunkt, ausgerichtet auf den zweiten
    start: m.route ? [m.route[0][0], m.route[0][1], Math.atan2(m.route[1][1] - m.route[0][1], m.route[1][0] - m.route[0][0])] : m.start
  })));

  /* ------------------------------ Lidar ------------------------------------- */
  /** Exakter Strahl-Zellen-Schnitt (Amanatides & Woo). Liefert Distanz bis zur ersten belegten Zelle. */
  function castRay(map, x, y, ang, maxR) {
    const res = map.res, cell = 1 / res;
    let ix = Math.floor(x * res), iy = Math.floor(y * res);
    if (map.occupied(ix, iy)) return [0, true];
    const dx = Math.cos(ang), dy = Math.sin(ang);
    const sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
    const adx = Math.abs(dx), ady = Math.abs(dy);
    let tx = adx > 1e-12 ? (dx > 0 ? (ix + 1) * cell - x : x - ix * cell) / adx : Infinity;
    let ty = ady > 1e-12 ? (dy > 0 ? (iy + 1) * cell - y : y - iy * cell) / ady : Infinity;
    const tdx = adx > 1e-12 ? cell / adx : Infinity, tdy = ady > 1e-12 ? cell / ady : Infinity;
    for (;;) {
      let t;
      if (tx < ty) { t = tx; tx += tdx; ix += sx; } else { t = ty; ty += tdy; iy += sy; }
      if (t > maxR) return [maxR, false];
      if (map.occupied(ix, iy)) return [t, true];
    }
  }
  MRS.castRay = castRay;

  function gauss(rng) {
    let u = 0, v = 0;
    while (u === 0) u = rng();
    while (v === 0) v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  /** Winkel der Lidar-Strahlen relativ zur Fahrtrichtung. */
  function beamAngles(fovDeg, n) {
    const fov = (Math.min(360, Math.max(1, fovDeg)) * Math.PI) / 180;
    const out = new Float64Array(n);
    if (fovDeg >= 360) for (let i = 0; i < n; i++) out[i] = -Math.PI + (2 * Math.PI * i) / n;
    else for (let i = 0; i < n; i++) out[i] = n === 1 ? 0 : -fov / 2 + (fov * i) / (n - 1);
    return out;
  }

  /* --------------------------- Occupancy Grid -------------------------------- */
  const L_FREE = -0.4, L_OCC = 0.85, L_MIN = -4, L_MAX = 4, L_OCC_TH = 0.6;

  class OccGrid {
    constructor(width, height, res) {
      this.res = res;
      this.W = Math.round(width * res);
      this.H = Math.round(height * res);
      this.width = width; this.height = height;
      this.L = new Float32Array(this.W * this.H);
      this.feat = new Int32Array(this.W * this.H).fill(-1);
      this._d2 = new Float32Array(this.W * this.H);
      this.version = 0;
    }

    _add(ix, iy, v) {
      if (ix < 0 || iy < 0 || ix >= this.W || iy >= this.H) return;
      const i = iy * this.W + ix;
      const n = this.L[i] + v;
      this.L[i] = n < L_MIN ? L_MIN : n > L_MAX ? L_MAX : n;
    }

    /** Scan an Pose integrieren: freie Zellen entlang des Strahls, belegte Zelle am Treffer. */
    integrate(pose, scan) {
      const res = this.res;
      const x0 = Math.floor(pose[0] * res), y0 = Math.floor(pose[1] * res);
      let bx0 = x0, bx1 = x0, by0 = y0, by1 = y0;
      for (let k = 0; k < scan.r.length; k++) {
        const a = pose[2] + scan.ang[k], r = scan.r[k];
        const x1 = Math.floor((pose[0] + r * Math.cos(a)) * res), y1 = Math.floor((pose[1] + r * Math.sin(a)) * res);
        // Bresenham
        let x = x0, y = y0;
        const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0);
        const sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        let err = dx + dy;
        for (;;) {
          if (x === x1 && y === y1) break;
          this._add(x, y, L_FREE);
          const e2 = 2 * err;
          if (e2 >= dy) { err += dy; x += sx; }
          if (e2 <= dx) { err += dx; y += sy; }
        }
        this._add(x1, y1, scan.hit[k] ? L_OCC : L_FREE);
        bx0 = Math.min(bx0, x1); bx1 = Math.max(bx1, x1); by0 = Math.min(by0, y1); by1 = Math.max(by1, y1);
      }
      // geänderter Bereich seit der letzten Darstellung (für inkrementelles Zeichnen)
      const d = this.dirty;
      this.dirty = d ? [Math.min(d[0], bx0), Math.min(d[1], by0), Math.max(d[2], bx1), Math.max(d[3], by1)] : [bx0, by0, bx1, by1];
      this.version++;
    }

    /**
     * Nächste belegte Zelle je Zelle aktualisieren (Grundlage des Scan-Matchings).
     * Optional nur im Fenster ±half [m] um (cx, cy): Ein Scan verändert die Karte nur innerhalb der
     * Lidar-Reichweite, und das Matching tastet nur dort ab – bei großen Karten spart das viel Zeit.
     */
    updateDistance(cx, cy, half) {
      const L = this.L, W = this.W, H = this.H;
      let x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1;
      if (half != null) {
        x0 = Math.max(0, Math.floor((cx - half) * this.res)); x1 = Math.min(W - 1, Math.ceil((cx + half) * this.res));
        y0 = Math.max(0, Math.floor((cy - half) * this.res)); y1 = Math.min(H - 1, Math.ceil((cy + half) * this.res));
      }
      computeFeatures(W, H, (i) => L[i] > L_OCC_TH, (i) => L[i] < 0, this.feat, this._d2, x0, y0, x1, y1);
    }

    stats(world) {
      let known = 0, correct = 0, N = 0;
      // große Karten: Stichprobe auf einem gröberen Raster (≈ 250 000 Zellen) genügt für Anteile
      const step = Math.max(1, Math.round(Math.sqrt((this.W * this.H) / 250000)));
      for (let iy = 0; iy < this.H; iy += step)
        for (let ix = 0; ix < this.W; ix += step) {
          N++;
          const l = this.L[iy * this.W + ix];
          if (Math.abs(l) < 0.5) continue;
          known++;
          const occ = world.occupiedAt((ix + 0.5) / this.res, (iy + 0.5) / this.res);
          if ((l > 0) === occ) correct++;
        }
      return { coverage: known / N, accuracy: known ? correct / known : NaN };
    }
  }
  MRS.OccGrid = OccGrid;

  /**
   * Feature-Transformation: für jede Zelle die nächstgelegene belegte Zelle (zweistufiges
   * Propagationsverfahren, 8er-Nachbarschaft). Damit lässt sich der exakte Abstand eines
   * Punktes zur Oberfläche der belegten Zelle berechnen (kein Interpolations-Bias).
   */
  function computeFeatures(W, H, isOcc, isFree, F, D2, x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1) {
    // Nur Oberflächenzellen (belegt mit freiem 4er-Nachbarn) dienen als Merkmale. Sonst kosteten
    // Endpunkte im Inneren dicker Wände nichts und die Orientierung wäre schlecht beobachtbar.
    // Berechnet wird im Fenster [x0, x1] × [y0, y1]; Merkmale außerhalb bleiben unberücksichtigt.
    for (let y = y0; y <= y1; y++)
      for (let x = x0; x <= x1; x++) {
        const i = y * W + x;
        const surf = isOcc(i) && ((x > 0 && isFree(i - 1)) || (x < W - 1 && isFree(i + 1)) || (y > 0 && isFree(i - W)) || (y < H - 1 && isFree(i + W)));
        if (surf) { F[i] = i; D2[i] = 0; } else { F[i] = -1; D2[i] = 1e12; }
      }
    const relax = (i, x, y, n) => {
      const f = F[n];
      if (f < 0) return;
      const fx = f % W, fy = (f / W) | 0;
      const d2 = (fx - x) * (fx - x) + (fy - y) * (fy - y);
      if (d2 < D2[i]) { D2[i] = d2; F[i] = f; }
    };
    for (let y = y0; y <= y1; y++)
      for (let x = x0; x <= x1; x++) {
        const i = y * W + x;
        if (D2[i] === 0) continue;
        if (x > x0) relax(i, x, y, i - 1);
        if (y > y0) {
          relax(i, x, y, i - W);
          if (x > x0) relax(i, x, y, i - W - 1);
          if (x < x1) relax(i, x, y, i - W + 1);
        }
      }
    for (let y = y1; y >= y0; y--)
      for (let x = x1; x >= x0; x--) {
        const i = y * W + x;
        if (D2[i] === 0) continue;
        if (x < x1) relax(i, x, y, i + 1);
        if (y < y1) {
          relax(i, x, y, i + W);
          if (x < x1) relax(i, x, y, i + W + 1);
          if (x > x0) relax(i, x, y, i + W - 1);
        }
      }
  }

  /** Distanzfeld der wahren Karte (für „Lokalisierung in bekannter Karte“ wie im MATLAB-Skript). */
  function knownDistanceField(world) {
    const F = new Int32Array(world.W * world.H);
    computeFeatures(world.W, world.H, (i) => world.occ[i] === 1, (i) => world.occ[i] === 0, F, new Float32Array(world.W * world.H));
    return { W: world.W, H: world.H, res: world.res, feat: F };
  }

  /* ----------------------------- Scan Matching ------------------------------- */
  const D_CLAMP = 0.3;

  /** Abstand des Punktes (x, y) zur nächsten belegten Zelle (als Quadrat), begrenzt auf D_CLAMP. */
  function sampleDist(df, x, y) {
    const res = df.res, W = df.W, F = df.feat;
    const gx = x * res, gy = y * res;
    const ix = Math.floor(gx - 0.5), iy = Math.floor(gy - 0.5);
    if (ix < 0 || iy < 0 || ix >= W - 1 || iy >= df.H - 1) return D_CLAMP;
    let best = D_CLAMP * res;
    // die vier umliegenden Zellen liefern Kandidaten für die nächste belegte Zelle
    for (let k = 0; k < 4; k++) {
      const f = F[(iy + (k >> 1)) * W + ix + (k & 1)];
      if (f < 0) continue;
      const fx = f % W, fy = (f / W) | 0;
      const dx = gx < fx ? fx - gx : gx > fx + 1 ? gx - fx - 1 : 0;
      const dy = gy < fy ? fy - gy : gy > fy + 1 ? gy - fy - 1 : 0;
      const d = Math.sqrt(dx * dx + dy * dy);
      if (d < best) best = d;
    }
    return best / res;
  }

  /** Ist die Zelle unter (x, y) noch unbekannt? (nur für die SLAM-Karte, die bekannte Karte ist vollständig) */
  function isUnknown(df, x, y) {
    if (!df.L) return false;
    const ix = Math.floor(x * df.res), iy = Math.floor(y * df.res);
    if (ix < 0 || iy < 0 || ix >= df.W || iy >= df.H) return true;
    const l = df.L[iy * df.W + ix];
    return l > -0.5 && l < 0.5;
  }

  const P_UNKNOWN = 0.35;
  // EKF: Grundrauschen der Orientierung je Scan bzw. Messunsicherheit der Scan-Matching-Orientierung [°]
  const Q_TH = 0.1, R_TH = 0.5;
  // Karte erst nach 0,5 m bzw. 6° Bewegung fortschreiben: Der Scan wird so gegen Kartenteile aus älteren
  // Posen abgeglichen, statt gegen die eigene, gerade erst eingetragene Schätzung (weniger Drift).
  const MAP_UPD_DIST = 0.5, MAP_UPD_ANG = (6 * Math.PI) / 180;

  /**
   * Korrelatives Scan-Matching auf einem Likelihood-Feld (vgl. refinePoseByDistanceField im
   * MATLAB-Skript, Hector-SLAM, AMCL): Jeder Scan-Endpunkt erhält den Wert exp(−d²/2σ²), d = Abstand
   * zur nächsten Wandoberfläche der Karte. Endpunkte in noch unbekanntem Gebiet erhalten einen
   * konstanten Wert und erzeugen damit keinen systematischen „Zug“ (Korridor-Effekt).
   * Gittersuche um die prädizierte Pose: grob, danach mehrfach verfeinert.
   */
  function matchScan(scan, pred, df, win) {
    let bx = [], by = [];
    const stride = Math.max(1, Math.floor(scan.r.length / 72));
    for (let k = 0; k < scan.r.length; k += stride) {
      if (!scan.hit[k]) continue;
      bx.push(scan.r[k] * Math.cos(scan.ang[k]));
      by.push(scan.r[k] * Math.sin(scan.ang[k]));
    }
    if (bx.length < 10) return null;
    // Nur Endpunkte verwenden, die an der prädizierten Pose in bereits kartiertem Gebiet liegen
    // (feste Auswahl für die ganze Suche). Sonst „zieht“ das Matching Endpunkte jenseits der
    // Kartengrenze in bekanntes Gebiet zurück – in Fluren entstünde ein systematischer Versatz
    // entgegen der Fahrtrichtung.
    {
      const c = Math.cos(pred[2]), s = Math.sin(pred[2]), kx = [], ky = [];
      for (let k = 0; k < bx.length; k++) {
        const px = pred[0] + c * bx[k] - s * by[k], py = pred[1] + s * bx[k] + c * by[k];
        if (!isUnknown(df, px, py) || sampleDist(df, px, py) < 0.1) { kx.push(bx[k]); ky.push(by[k]); }
      }
      if (kx.length >= 10) { bx = kx; by = ky; }
    }
    const n = bx.length;
    const rx = new Float64Array(n), ry = new Float64Array(n);
    const evalTheta = (th) => { const c = Math.cos(th), s = Math.sin(th); for (let k = 0; k < n; k++) { rx[k] = c * bx[k] - s * by[k]; ry[k] = s * bx[k] + c * by[k]; } };
    const sigma0 = Math.max(0.03, 0.6 / df.res);
    // Kosten = 1 − mittlere Likelihood (0 = perfekte Deckung)
    const costAt = (x, y, sigma) => {
      const k2 = 1 / (2 * sigma * sigma);
      let s = 0;
      for (let k = 0; k < n; k++) {
        const px = x + rx[k], py = y + ry[k];
        const d = sampleDist(df, px, py);
        let p = Math.exp(-d * d * k2);
        if (p < P_UNKNOWN && isUnknown(df, px, py)) p = P_UNKNOWN;
        s += p;
      }
      return 1 - s / n;
    };
    const search = (center, tw, rw, steps) => {
      // Breite des Feldes an die Schrittweite der Suchstufe anpassen (grob → glatter)
      const sigma = Math.max(sigma0, (tw / steps) * 0.8);
      let b = { pose: center.slice(), cost: Infinity };
      for (let j = -steps; j <= steps; j++) {
        const th = center[2] + (rw * j) / steps;
        evalTheta(th);
        for (let u = -steps; u <= steps; u++)
          for (let v = -steps; v <= steps; v++) {
            const x = center[0] + (tw * u) / steps, y = center[1] + (tw * v) / steps;
            const pen = 0.1 * Math.hypot(x - pred[0], y - pred[1]) + 0.05 * Math.abs(th - pred[2]);
            const c = costAt(x, y, sigma) + pen;
            if (c < b.cost) b = { pose: [x, y, th], cost: c };
          }
      }
      return b;
    };
    // Gittersuche grob (5 × 5 × 5), danach fünfmal fein (3 × 3 × 3) mit jeweils halbierter Schrittweite
    let best = search(pred, win.trans, win.rot, 2);
    best = search(best.pose, win.trans / 4, win.rot / 4, 1);
    best = search(best.pose, win.trans / 8, win.rot / 8, 1);
    best = search(best.pose, win.trans / 16, win.rot / 16, 1);
    best = search(best.pose, win.trans / 32, win.rot / 32, 1);
    best = search(best.pose, win.trans / 64, win.rot / 64, 1);
    best.pose[2] = wrapAngle(best.pose[2]);
    evalTheta(best.pose[2]);
    // Güte: Anteil der Endpunkte nahe einer bekannten Wand (Inlier), RMS-Abstand der Inlier,
    // Anteil der Endpunkte in bereits bekanntem Gebiet (Überlappung mit der Karte)
    let nin = 0, sin = 0, nknown = 0;
    for (let k = 0; k < n; k++) {
      const px = best.pose[0] + rx[k], py = best.pose[1] + ry[k];
      const d = sampleDist(df, px, py);
      if (d < 0.1) { nin++; sin += d * d; }
      if (d < 0.1 || !isUnknown(df, px, py)) nknown++;
    }
    best.overlap = nknown / n;
    best.inlier = nknown ? nin / nknown : 0;
    best.rms = nin ? Math.sqrt(sin / nin) : D_CLAMP;
    best.n = n;
    return best;
  }

  /* ---------------------------- 3×3-Matrixhilfen ----------------------------- */
  const mat = {
    mul: (A, B) => A.map((r, i) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j])),
    T: (A) => [0, 1, 2].map((i) => [A[0][i], A[1][i], A[2][i]]),
    add: (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j])),
    sub: (A, B) => A.map((r, i) => r.map((v, j) => v - B[i][j])),
    diag: (a, b, c) => [[a, 0, 0], [0, b, 0], [0, 0, c]],
    I: () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    inv(A) {
      const [[a, b, c], [d, e, f], [g, h, i]] = A;
      const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
      if (Math.abs(det) < 1e-18) return null;
      return [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]
      ];
    },
    vec: (A, v) => A.map((r) => r[0] * v[0] + r[1] * v[1] + r[2] * v[2])
  };

  /* --------------------------- Routenerzeugung ------------------------------- */
  function downsample(map, res) {
    const out = new MRS.GridMap(map.width, map.height, res);
    const f = map.res / res;
    for (let iy = 0; iy < map.H; iy++)
      for (let ix = 0; ix < map.W; ix++)
        if (map.occ[iy * map.W + ix]) out.occ[Math.min(out.H - 1, Math.floor(iy / f)) * out.W + Math.min(out.W - 1, Math.floor(ix / f))] = 1;
    return out;
  }

  function segFree(map, a, b) {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(1, Math.ceil(L / (0.5 / map.res)));
    for (let i = 0; i <= n; i++) if (map.occupiedAt(a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n)) return false;
    return true;
  }

  /**
   * Strukturierte Erkundungsroute: Rasterpunkte im Abstand `spacing` werden per
   * „nächster unbesuchter Punkt (geodätisch)“ verbunden; anschließend String-Pulling.
   */
  function buildCoverageRoute(world, start, spacing = 2.5) {
    const clear = world.inflate(0.45);
    const coarse = downsample(clear, 5);
    const fine = world.inflate(0.35);
    const cand = [];
    for (let y = spacing / 2; y < world.height; y += spacing)
      for (let x = spacing / 2; x < world.width; x += spacing) if (!coarse.occupiedAt(x, y)) cand.push([x, y]);
    const cellOf = (p) => [Math.floor(p[0] * coarse.res), Math.floor(p[1] * coarse.res)];
    let cur = [start[0], start[1]];
    const cells = [cur];
    const visited = new Uint8Array(cand.length);
    for (let guard = 0; guard < cand.length + 1; guard++) {
      const [cx, cy] = cellOf(cur);
      const D = MRS.geodesic(coarse, cx, cy);
      let bi = -1, bd = Infinity;
      cand.forEach((p, i) => {
        if (visited[i]) return;
        const [px, py] = cellOf(p);
        const d = D[py * coarse.W + px];
        if (d < bd) { bd = d; bi = i; }
      });
      if (bi < 0 || !isFinite(bd)) break;
      // Pfad vom Ziel zum aktuellen Punkt per Gradientenabstieg
      let [x, y] = cellOf(cand[bi]);
      const seg = [];
      for (let s = 0; s < 5000 && D[y * coarse.W + x] > 0; s++) {
        seg.push([(x + 0.5) / coarse.res, (y + 0.5) / coarse.res]);
        let best = D[y * coarse.W + x], nx = x, ny = y;
        for (let dy = -1; dy <= 1; dy++)
          for (let dx = -1; dx <= 1; dx++) {
            const qx = x + dx, qy = y + dy;
            if (qx < 0 || qy < 0 || qx >= coarse.W || qy >= coarse.H) continue;
            const v = D[qy * coarse.W + qx];
            if (v < best) { best = v; nx = qx; ny = qy; }
          }
        if (nx === x && ny === y) break;
        x = nx; y = ny;
      }
      seg.reverse();
      for (const p of seg) {
        cells.push(p);
        cand.forEach((c, i) => { if (!visited[i] && Math.hypot(c[0] - p[0], c[1] - p[1]) < spacing * 0.45) visited[i] = 1; });
      }
      visited[bi] = 1;
      cur = cand[bi];
    }
    // String-Pulling
    const out = [cells[0]];
    let i = 0;
    while (i < cells.length - 1) {
      let j = Math.min(cells.length - 1, i + 60);
      while (j > i + 1 && !segFree(fine, cells[i], cells[j])) j--;
      out.push(cells[j]);
      i = j;
    }
    return out;
  }
  MRS.buildCoverageRoute = buildCoverageRoute;

  /* ------------------------------ Simulation --------------------------------- */
  class SlamSim {
    /**
     * cfg: { lidar:{rate,range,fov,beams,noise}, odo:{sigV,sigW,biasW},
     *        method:'slam'|'known'|'odom', mapRes, win:{trans,rot}, robotR }
     */
    constructor(world, start, cfg, seed = 7) {
      this.world = world;
      this.cfg = cfg;
      this.rng = MRS.makeRng(seed);
      this.truePose = start.slice();
      this.estPose = start.slice();
      this.odoPose = start.slice();
      this.P = mat.diag(0.015 ** 2, 0.015 ** 2, (Math.PI / 180) ** 2);
      this.t = 0;
      this.scans = 0;
      this.grid = new OccGrid(world.width, world.height, cfg.mapRes);
      this.known = cfg.method === 'known' ? knownDistanceField(world) : null;
      this.truePath = [[start[0], start[1]]];
      this.estPath = [[start[0], start[1]]];
      this.odoPath = [[start[0], start[1]]];
      this.log = { t: [], eSlam: [], eOdo: [], hSlam: [], hOdo: [], rms: [] };
      this.lastScan = null;
      this.lastMatch = null;
      this.lastEstAtScan = start.slice();
      this.status = 'bereit';
      this.matchMs = 0;
      this.rejected = 0;
      this.bumped = false;
      this.dist = 0;
      this.mapStats = { coverage: 0, accuracy: NaN };
      this.lastDistortion = 0;
      // Startscan als Momentaufnahme an der bekannten Startpose, danach rotierender Strahl
      this._startRev(0, true);
      this.rev.T = 0; // die erste reguläre Umdrehung beginnt sofort
      this._lidarStep(0, 0, this.truePose, this.truePose, this.odoPose, this.odoPose);
    }

    /* ------------------------ rotierender Lidar-Strahl ------------------------ */
    /**
     * Neue Umdrehung des Lidar-Kopfes ab Zeitpunkt t0. Ein einzelner Laserstrahl dreht mit der
     * Scan-Rate; innerhalb des Öffnungswinkels wird in gleichen Winkelschritten gemessen.
     * Messpunkt k entsteht zum Zeitpunkt t0 + Ts·(k+1)/n (Ts = Dauer des Sektors).
     * Beim idealen Modell (Momentaufnahme) werden alle Punkte zum selben Zeitpunkt gemessen.
     */
    _startRev(t0, snapshot = false) {
      const lid = this.cfg.lidar;
      const n = Math.max(2, Math.round(lid.beams));
      const T = 1 / Math.max(0.1, lid.rate);
      const Ts = T * Math.min(1, lid.fov / 360);
      const ideal = snapshot || lid.model === 'ideal';
      const times = new Float64Array(n);
      for (let k = 0; k < n; k++) times[k] = snapshot ? t0 : ideal ? t0 + Ts : t0 + (Ts * (k + 1)) / n;
      this.rev = {
        t0, T, n, times, idx: 0, ideal,
        ang: beamAngles(lid.fov, n), range: lid.range,
        r: new Float64Array(n), hit: new Uint8Array(n),
        tp: new Float64Array(3 * n), op: new Float64Array(3 * n)
      };
      const sw = this.sweep;
      if (!sw || sw.n !== n) {
        this.sweep = { n, wx: new Float64Array(n), wy: new Float64Array(n), sx: new Float64Array(n), sy: new Float64Array(n), hit: new Uint8Array(n), valid: new Uint8Array(n), last: -1, order: [] };
      }
    }

    /** Alle Messpunkte, deren Aufnahmezeitpunkt im Intervall (tOld, tNew] liegt, an der jeweils interpolierten Pose messen. */
    _lidarStep(tOld, tNew, trueOld, trueNew, odoOld, odoNew) {
      const lerp = (a, b, f) => [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, wrapAngle(a[2] + wrapAngle(b[2] - a[2]) * f)];
      const span = tNew - tOld;
      for (let guard = 0; guard < 100000; guard++) {
        const rev = this.rev;
        if (rev.idx >= rev.n) {
          if (rev.t0 + rev.T > tNew + 1e-12) break;
          this._startRev(rev.t0 + rev.T);
          continue;
        }
        const k = rev.idx, tk = rev.times[k];
        if (tk > tNew + 1e-12) break;
        const f = span > 0 ? Math.min(1, Math.max(0, (tk - tOld) / span)) : 1;
        const tp = lerp(trueOld, trueNew, f), op = lerp(odoOld, odoNew, f);
        const lid = this.cfg.lidar;
        const a = tp[2] + rev.ang[k];
        const [d, h] = castRay(this.world, tp[0], tp[1], a, rev.range);
        let r = rev.range, hit = 0;
        if (h) {
          r = Math.max(0.01, d + (lid.noise > 0 ? lid.noise * gauss(this.rng) : 0));
          if (r < rev.range) hit = 1; else r = rev.range;
        }
        rev.r[k] = r; rev.hit[k] = hit;
        rev.tp.set(tp, 3 * k); rev.op.set(op, 3 * k);
        // Anzeige: Messpunkt in Weltkoordinaten (Radar-artige Aktualisierung)
        const sw = this.sweep;
        sw.wx[k] = tp[0] + r * Math.cos(a); sw.wy[k] = tp[1] + r * Math.sin(a);
        sw.sx[k] = tp[0]; sw.sy[k] = tp[1];
        sw.hit[k] = hit; sw.valid[k] = 1; sw.last = k;
        rev.idx++;
        if (rev.idx === rev.n) this._finishRev();
      }
    }

    /**
     * Umdrehung abgeschlossen: Messpunkte in das Sensor-Koordinatensystem zum Zeitpunkt des
     * letzten Messpunkts überführen. Mit Deskewing wird die Eigenbewegung während der Umdrehung
     * aus der Odometrie herausgerechnet, sonst werden die Rohwerte (Winkel, Distanz) verwendet.
     */
    _finishRev() {
      const rev = this.rev, n = rev.n, deskew = !rev.ideal && this.cfg.lidar.deskew;
      const ref = n - 1;
      const o0 = [rev.op[3 * ref], rev.op[3 * ref + 1], rev.op[3 * ref + 2]];
      const t0 = [rev.tp[3 * ref], rev.tp[3 * ref + 1], rev.tp[3 * ref + 2]];
      const toRef = (P, R, px, py) => {
        // Punkt (px, py) im Sensorsystem der Pose P → Sensorsystem der Referenzpose R
        const c = Math.cos(P[2]), s = Math.sin(P[2]);
        const wx = P[0] + c * px - s * py - R[0], wy = P[1] + s * px + c * py - R[1];
        const cr = Math.cos(R[2]), sr = Math.sin(R[2]);
        return [cr * wx + sr * wy, -sr * wx + cr * wy];
      };
      const ang = new Float64Array(n), r = new Float64Array(n);
      let maxShift = 0, maxResid = 0;
      for (let k = 0; k < n; k++) {
        const a = rev.ang[k], rr = rev.r[k];
        const px = rr * Math.cos(a), py = rr * Math.sin(a);
        // tatsächliche Verzerrung: Rohpunkt gegenüber dem mit der wahren Bewegung korrigierten Punkt
        let qt = null;
        if (rev.hit[k] && !rev.ideal) {
          qt = toRef([rev.tp[3 * k], rev.tp[3 * k + 1], rev.tp[3 * k + 2]], t0, px, py);
          maxShift = Math.max(maxShift, Math.hypot(qt[0] - px, qt[1] - py));
        }
        if (deskew) {
          const q = toRef([rev.op[3 * k], rev.op[3 * k + 1], rev.op[3 * k + 2]], o0, px, py);
          ang[k] = Math.atan2(q[1], q[0]); r[k] = Math.hypot(q[0], q[1]);
          if (qt) maxResid = Math.max(maxResid, Math.hypot(qt[0] - q[0], qt[1] - q[1]));
        } else { ang[k] = a; r[k] = rr; }
      }
      // Verzerrung: roh (ohne Korrektur) und verbleibend nach der Korrektur
      this.lastDistortion = maxShift;
      this.lastDistortionResid = deskew ? maxResid : maxShift;
      this._process({ ang, r, hit: rev.hit.slice(), range: rev.range, pose: t0, deskewed: deskew });
    }

    collides(x, y) {
      const r = this.cfg.robotR, w = this.world;
      if (w.occupiedAt(x, y)) return true;
      for (let k = 0; k < 12; k++) {
        const a = (k * Math.PI) / 6;
        if (w.occupiedAt(x + r * Math.cos(a), y + r * Math.sin(a))) return true;
      }
      return false;
    }

    /** Abstand nach vorn (Nahbereichssensor/Bumper, unabhängig vom Lidar-Öffnungswinkel). */
    frontClearance(halfDeg = 35, maxR = 2) {
      let m = maxR;
      for (let a = -halfDeg; a <= halfDeg; a += 7) {
        const [d] = castRay(this.world, this.truePose[0], this.truePose[1], this.truePose[2] + (a * Math.PI) / 180, maxR);
        m = Math.min(m, d);
      }
      return m - this.cfg.robotR;
    }

    step(dt, v, w) {
      const integ = (p, vv, ww) => {
        const th = p[2];
        if (Math.abs(ww) < 1e-9) return [p[0] + vv * Math.cos(th) * dt, p[1] + vv * Math.sin(th) * dt, th];
        const th2 = th + ww * dt;
        return [p[0] + (vv / ww) * (Math.sin(th2) - Math.sin(th)), p[1] - (vv / ww) * (Math.cos(th2) - Math.cos(th)), wrapAngle(th2)];
      };
      const trueOld = this.truePose, odoOld = this.odoPose, tOld = this.t;
      let np = integ(this.truePose, v, w);
      this.bumped = false;
      if (v !== 0 && this.collides(np[0], np[1])) { v = 0; np = integ(this.truePose, 0, w); this.bumped = true; }
      this.dist += Math.hypot(np[0] - this.truePose[0], np[1] - this.truePose[1]);
      this.truePose = np;
      // Verrauschte Odometrie (Radencoder + Gyro-Drift)
      const o = this.cfg.odo;
      const moving = Math.abs(v) > 1e-6 || Math.abs(w) > 1e-6;
      const vm = v * (1 + (o.sigV / 100) * gauss(this.rng));
      const wm = w + (moving ? o.sigW * gauss(this.rng) : 0) + o.biasW;
      this.odoPose = integ(this.odoPose, vm, wm);
      this.estPose = integ(this.estPose, vm, wm);
      this._odoInc = this._odoInc || [0, 0];
      this._odoInc[0] += Math.abs(vm * dt);
      this._odoInc[1] += Math.abs(wm * dt);
      this.t += dt;
      const lp = this.truePath[this.truePath.length - 1];
      if (Math.hypot(np[0] - lp[0], np[1] - lp[1]) > 0.02) {
        this.truePath.push([np[0], np[1]]);
        this.odoPath.push([this.odoPose[0], this.odoPose[1]]);
      }
      this._lidarStep(tOld, this.t, trueOld, this.truePose, odoOld, this.odoPose);
    }

    /** Verarbeitung eines vollständigen Scans: EKF-Prädiktion, Scan-Matching, Korrektur, Kartierung. */
    _process(scan) {
      const cfg = this.cfg;
      this.lastScan = scan;
      this.scans++;
      const t0 = performance.now();
      // EKF-Prädiktion: Kovarianz wächst mit zurückgelegtem Weg/Winkel seit dem letzten Scan
      const inc = this._odoInc || [0, 0];
      this._odoInc = [0, 0];
      const prev = this.lastEstAtScan, pred = this.estPose;
      const dxw = pred[0] - prev[0], dyw = pred[1] - prev[1];
      const F = [[1, 0, -dyw], [0, 1, dxw], [0, 0, 1]];
      const Q = mat.diag(0.002 ** 2 + inc[0] * 0.014 ** 2, 0.002 ** 2 + inc[0] * 0.014 ** 2, (Q_TH * Math.PI / 180) ** 2 + inc[1] * (0.8 * Math.PI / 180) ** 2 + inc[0] * (0.5 * Math.PI / 180) ** 2);
      let P = mat.add(mat.mul(mat.mul(F, this.P), mat.T(F)), Q);
      this.lastMatch = null;
      if (cfg.method !== 'odom' && this.scans > 1) {
        const df = cfg.method === 'known' ? this.known : this.grid;
        const m = matchScan(scan, pred, df, cfg.win);
        if (m) {
          this.lastMatch = m;
          const reliable = m.inlier >= 0.5 && m.rms < 0.06 && m.overlap >= 0.3;
          if (reliable) {
            const R = mat.diag(0.01 ** 2, 0.01 ** 2, (R_TH * Math.PI / 180) ** 2);
            const innov = [m.pose[0] - pred[0], m.pose[1] - pred[1], wrapAngle(m.pose[2] - pred[2])];
            const S = mat.add(P, R);
            const Si = mat.inv(S);
            const nis = Si ? innov.reduce((s, v, i) => s + v * mat.vec(Si, innov)[i], 0) : Infinity;
            // Chi²-Gating (3 Freiheitsgrade); Schwelle gegenüber 7,81 (95 %) erweitert, da R klein gewählt ist
            if (nis <= 7.81 * 4) {
              const K = mat.mul(P, Si);
              const d = mat.vec(K, innov);
              this.estPose = [pred[0] + d[0], pred[1] + d[1], wrapAngle(pred[2] + d[2])];
              P = mat.mul(mat.sub(mat.I(), K), P);
              this.status = 'Scan-Matching ok';
            } else { this.rejected++; this.status = 'Messung verworfen (Gating)'; }
          } else { this.rejected++; this.status = 'Matching unsicher'; }
        } else this.status = 'zu wenige Treffer';
      } else if (cfg.method === 'odom') this.status = 'nur Odometrie';
      this.P = P.map((r, i) => r.map((v, j) => 0.5 * (v + P[j][i])));
      this.lastEstAtScan = this.estPose.slice();
      // Kartierung an der geschätzten Pose – nur nach ausreichender Bewegung (Update-Schwellen wie
      // in gängigen SLAM-Systemen). Sonst würde die Karte im Stand der Odometrie-Drift „nachgeführt“.
      const lm = this.lastMapPose;
      if (!lm || Math.hypot(this.estPose[0] - lm[0], this.estPose[1] - lm[1]) > MAP_UPD_DIST || Math.abs(wrapAngle(this.estPose[2] - lm[2])) > MAP_UPD_ANG) {
        this.grid.integrate(this.estPose, scan);
        // Distanzfeld nur im Bereich des Scans (Reichweite + Suchfenster + Rand) neu berechnen
        if (cfg.method === 'slam') this.grid.updateDistance(this.estPose[0], this.estPose[1], scan.range + 1.5);
        this.lastMapPose = this.estPose.slice();
        this.mapUpdates = (this.mapUpdates || 0) + 1;
      }
      this.matchMs = performance.now() - t0;
      this.estPath.push([this.estPose[0], this.estPose[1]]);
      // Protokoll
      const tp = this.truePose, ep = this.estPose, op = this.odoPose;
      const L = this.log;
      L.t.push(this.t);
      L.eSlam.push(Math.hypot(ep[0] - tp[0], ep[1] - tp[1]));
      L.eOdo.push(Math.hypot(op[0] - tp[0], op[1] - tp[1]));
      L.hSlam.push(Math.abs(wrapAngle(ep[2] - tp[2])) * 180 / Math.PI);
      L.hOdo.push(Math.abs(wrapAngle(op[2] - tp[2])) * 180 / Math.PI);
      L.rms.push(this.lastMatch ? this.lastMatch.rms : null);
      if (this.scans % 25 === 1) this.mapStats = this.grid.stats(this.world);
    }
  }
  MRS.SlamSim = SlamSim;

  /* ------------------------------ Fahrstrategien ------------------------------ */
  /** Strukturierte Fahrt: Route mit Pure Pursuit abfahren, an Ecken auf der Stelle drehen.
   *  Wie im MATLAB-Skript (vorgegebene Trajektorie) folgt der Roboter der Route anhand der wahren
   *  Pose; die Schätzung (SLAM/Odometrie) läuft davon entkoppelt und kann so frei bewertet werden. */
  class RouteDriver {
    constructor(route, v, wMax, useEst = false) {
      this.route = route;
      this.useEst = useEst;
      this.v = v; this.wMax = wMax;
      this.pp = new MRS.PurePursuit(route, { lookahead: 0.5, vDesired: v, wMax });
      this.done = false;
    }
    progress() {
      const pp = this.pp;
      let done = 0, total = 0;
      pp.segLen.forEach((l, i) => { total += l; if (i < pp.projIdx) done += l; });
      done += Math.hypot(pp.projPt[0] - this.route[pp.projIdx][0], pp.projPt[1] - this.route[pp.projIdx][1]);
      return total > 0 ? Math.min(1, done / total) : 1;
    }
    /**
     * Ecken mit deutlichem Richtungswechsel (> 30°) exakt anfahren: auf den Eckpunkt zufahren, dort auf
     * der Stelle in die neue Richtung drehen und erst dann weiterfahren. Pure Pursuit allein würde die
     * Ecke abkürzen – in engen Gängen (z. B. zwischen Tischreihen) führte das zu Wandkontakt.
     * Rückgabe: Stellgrößen [v, ω] während des Eckmanövers, sonst null.
     */
    _corner(p) {
      const R = this.route, pp = this.pp;
      if (this.next == null || pp.projIdx >= this.next) { this.next = pp.projIdx + 1; this.inCorner = false; this.atVertex = false; }
      const i = this.next;
      if (i >= R.length - 1) return null;
      const a = R[i - 1], c = R[i], b = R[i + 1];
      const hIn = Math.atan2(c[1] - a[1], c[0] - a[0]), hOut = Math.atan2(b[1] - c[1], b[0] - c[0]);
      const d = Math.hypot(c[0] - p[0], c[1] - p[1]);
      if (!this.inCorner && !(Math.abs(wrapAngle(hOut - hIn)) > Math.PI / 6 && d < pp.L + 0.05)) return null;
      this.inCorner = true;
      const clampW = (w) => Math.max(-this.wMax, Math.min(this.wMax, w));
      if (!this.atVertex) {
        // erreicht, wenn nah genug oder der Eckpunkt bereits (in Anfahrrichtung) hinter dem Roboter liegt
        const along = (c[0] - p[0]) * Math.cos(hIn) + (c[1] - p[1]) * Math.sin(hIn);
        if (d > 0.03 && along > 0.005) {
          const al = wrapAngle(Math.atan2(c[1] - p[1], c[0] - p[0]) - p[2]);
          if (Math.abs(al) > 0.35 && d > 0.1) return [0, Math.sign(al) * this.wMax * 0.8];
          return [Math.min(this.v, 0.1 + 1.5 * d), clampW(2.5 * al)];
        }
        this.atVertex = true;
      }
      const e = wrapAngle(hOut - p[2]);
      if (Math.abs(e) > 0.02) return [0, Math.sign(e) * Math.max(0.15, Math.min(this.wMax, 3 * Math.abs(e)))];
      // Ecke abgeschlossen: Pure Pursuit ab dem Eckpunkt auf dem nächsten Segment fortsetzen
      pp.projIdx = i; pp.projPt = c.slice();
      this.next = i + 1; this.inCorner = false; this.atVertex = false;
      return null;
    }

    command(sim, dt) {
      if (this.done) return [0, 0];
      // Erholung nach Hinderniskontakt: kurz zurücksetzen; bei wiederholtem Kontakt Wegpunkt überspringen
      if (this.backoff > 0) { this.backoff -= dt; return [-0.2, 0]; }
      if (sim.bumped) {
        this.bumps = (this.bumps || 0) + 1;
        this.backoff = 1.0;
        if (this.bumps % 3 === 0 && this.pp.projIdx < this.route.length - 2) {
          this.pp.projIdx++;
          this.pp.projPt = this.route[this.pp.projIdx].slice();
          this.skipped = (this.skipped || 0) + 1;
        }
        return [-0.2, 0];
      }
      const p = this.useEst ? sim.estPose : sim.truePose, last = this.route[this.route.length - 1];
      const cmd = this._corner(p);
      if (cmd) return cmd;
      const r = this.pp.step(p);
      if (this.pp.projIdx >= this.route.length - 2 && Math.hypot(p[0] - last[0], p[1] - last[1]) < 0.15) { this.done = true; return [0, 0]; }
      const lp = this.pp.lookPt;
      const alpha = wrapAngle(Math.atan2(lp[1] - p[1], lp[0] - p[0]) - p[2]);
      if (Math.abs(alpha) > Math.PI / 4) return [0, Math.sign(alpha) * this.wMax * 0.8];
      return [this.v, r.w];
    }
  }

  /** Zufallsfahrt: geradeaus, bei Hindernis(-kontakt) Drehung um zufälligen Winkel in zufällige Richtung. */
  class RandomDriver {
    constructor(v, wMax, dSafe, rng) {
      this.v = v; this.wMax = wMax; this.dSafe = dSafe; this.rng = rng;
      this.state = 'drive'; this.turns = 0;
    }
    command(sim, dt) {
      const fc = sim.frontClearance();
      if (this.state === 'drive') {
        if (fc < this.dSafe || sim.bumped) {
          this.state = 'turn';
          this.dir = this.rng() < 0.5 ? -1 : 1;
          this.target = Math.PI / 2 + this.rng() * Math.PI * 0.75;
          this.turned = 0;
          this.turns++;
        } else return [this.v, 0];
      }
      this.turned += this.wMax * dt;
      if (this.turned >= this.target && fc > this.dSafe + 0.3) { this.state = 'drive'; return [this.v, 0]; }
      return [0, this.dir * this.wMax];
    }
  }
  MRS.RouteDriver = RouteDriver;

  /**
   * Verbindet Wegpunkte zu einer fahrbaren Bahn. Mit plan = true wird zwischen je zwei Punkten
   * ein kollisionsfreier Weg mit A* (Raster, Shortcut-Glättung) auf der aufgeblähten Karte gesucht;
   * gelingt das nicht, wird die direkte Verbindung verwendet.
   */
  function connectWaypoints(world, pts, plan) {
    if (pts.length < 2) return pts.map((p) => [p[0], p[1]]);
    const out = [[pts[0][0], pts[0][1]]];
    const wpIdx = [0];
    let failed = 0;
    if (plan && !world._wpInfl) world._wpInfl = world.inflate(0.25);
    for (let i = 1; i < pts.length; i++) {
      const a = pts[i - 1], b = pts[i];
      let seg = null;
      if (plan) {
        const r = MRS.runPlanner('astar', world, world._wpInfl, [a[0], a[1], 0], [b[0], b[1], 0], { astarWeight: 1, astarSmooth: true, seed: 1 });
        if (r.ok) seg = r.states.slice(1).map((q) => [q[0], q[1]]);
        else failed++;
      }
      for (const q of seg || [[b[0], b[1]]]) out.push(q);
      wpIdx.push(out.length - 1);
    }
    out.failed = failed;
    out.wpIdx = wpIdx; // Index jedes Wegpunkts in der Bahn
    return out;
  }
  MRS.connectWaypoints = connectWaypoints;
  MRS.RandomDriver = RandomDriver;
})();
