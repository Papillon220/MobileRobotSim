/* ==========================================================================
   MobileRobotSim – Belegungskarten (Occupancy Grids)
   Koordinaten: x nach rechts, y nach oben, Ursprung links unten, Einheit m.
   Zelle (ix, iy) überdeckt [ix/res, (ix+1)/res] x [iy/res, (iy+1)/res].
   ========================================================================== */
(function () {
  'use strict';
  const MRS = (window.MRS = window.MRS || {});

  class GridMap {
    constructor(widthM, heightM, res) {
      this.res = res;
      this.W = Math.round(widthM * res);
      this.H = Math.round(heightM * res);
      this.width = this.W / res;
      this.height = this.H / res;
      this.occ = new Uint8Array(this.W * this.H);
    }

    clone() {
      const m = new GridMap(this.width, this.height, this.res);
      m.occ.set(this.occ);
      return m;
    }

    /** Erzeugt eine Karte aus Textzeilen ('1' = belegt), erste Zeile = oben. */
    static fromRows(rows, baseRes, res) {
      const f = Math.round(res / baseRes);
      const Hb = rows.length, Wb = rows[0].length;
      const m = new GridMap(Wb / baseRes, Hb / baseRes, res);
      for (let r = 0; r < Hb; r++) {
        const iyb = Hb - 1 - r;
        for (let c = 0; c < Wb; c++) {
          if (rows[r][c] !== '1') continue;
          for (let dy = 0; dy < f; dy++)
            for (let dx = 0; dx < f; dx++)
              m.occ[(iyb * f + dy) * m.W + c * f + dx] = 1;
        }
      }
      return m;
    }

    /** Erzeugt eine Karte aus lauflängenkodierten Zeilen (Format siehe map-atrium.js). */
    static fromRLE(d, res) {
      const A = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_';
      const f = res / d.res;
      const m = new GridMap(d.W / d.res, d.H / d.res, res);
      d.rows.split('|').forEach((row, r) => {
        const iyb = d.H - 1 - r;
        const y0 = Math.floor(iyb * f), y1 = Math.max(y0, Math.ceil((iyb + 1) * f) - 1);
        let c = 0, occ = false, n = 0, sh = 0;
        for (let i = 0; i < row.length; i++) {
          const v = A.indexOf(row[i]);
          n += (v & 31) << sh;
          if (v >= 32) { sh += 5; continue; }
          if (occ) {
            const x0 = Math.floor(c * f), x1 = Math.max(x0, Math.ceil((c + n) * f) - 1);
            for (let y = y0; y <= y1; y++) m.occ.fill(1, y * m.W + x0, y * m.W + Math.min(x1, m.W - 1) + 1);
          }
          c += n; occ = !occ; n = 0; sh = 0;
        }
      });
      return m;
    }

    occupied(ix, iy) {
      if (ix < 0 || iy < 0 || ix >= this.W || iy >= this.H) return true;
      return this.occ[iy * this.W + ix] === 1;
    }

    occupiedAt(x, y) {
      return this.occupied(Math.floor(x * this.res), Math.floor(y * this.res));
    }

    /** Rechteck in Weltkoordinaten füllen (Zellmittelpunkte im Rechteck). */
    fillRect(x0, y0, x1, y1) {
      const r = this.res;
      const ix0 = Math.max(0, Math.ceil(Math.min(x0, x1) * r - 0.5));
      const ix1 = Math.min(this.W - 1, Math.floor(Math.max(x0, x1) * r - 0.5));
      const iy0 = Math.max(0, Math.ceil(Math.min(y0, y1) * r - 0.5));
      const iy1 = Math.min(this.H - 1, Math.floor(Math.max(y0, y1) * r - 0.5));
      for (let iy = iy0; iy <= iy1; iy++)
        for (let ix = ix0; ix <= ix1; ix++) this.occ[iy * this.W + ix] = 1;
      return this;
    }

    clearRect(x0, y0, x1, y1) {
      const r = this.res;
      const ix0 = Math.max(0, Math.ceil(Math.min(x0, x1) * r - 0.5));
      const ix1 = Math.min(this.W - 1, Math.floor(Math.max(x0, x1) * r - 0.5));
      const iy0 = Math.max(0, Math.ceil(Math.min(y0, y1) * r - 0.5));
      const iy1 = Math.min(this.H - 1, Math.floor(Math.max(y0, y1) * r - 0.5));
      for (let iy = iy0; iy <= iy1; iy++)
        for (let ix = ix0; ix <= ix1; ix++) this.occ[iy * this.W + ix] = 0;
      return this;
    }

    fillCircle(cx, cy, rad) {
      const r = this.res;
      const ix0 = Math.max(0, Math.floor((cx - rad) * r)), ix1 = Math.min(this.W - 1, Math.ceil((cx + rad) * r));
      const iy0 = Math.max(0, Math.floor((cy - rad) * r)), iy1 = Math.min(this.H - 1, Math.ceil((cy + rad) * r));
      for (let iy = iy0; iy <= iy1; iy++)
        for (let ix = ix0; ix <= ix1; ix++) {
          const x = (ix + 0.5) / r - cx, y = (iy + 0.5) / r - cy;
          if (x * x + y * y <= rad * rad) this.occ[iy * this.W + ix] = 1;
        }
      return this;
    }

    border(t) {
      this.fillRect(0, 0, this.width, t);
      this.fillRect(0, this.height - t, this.width, this.height);
      this.fillRect(0, 0, t, this.height);
      this.fillRect(this.width - t, 0, this.width, this.height);
      return this;
    }

    /** Hindernisse um Radius rad [m] aufblähen (entspricht inflate() in MATLAB). */
    inflate(rad) {
      const out = this.clone();
      const rc = rad * this.res;
      const R = Math.ceil(rc);
      const offs = [];
      for (let dy = -R; dy <= R; dy++)
        for (let dx = -R; dx <= R; dx++) if (dx * dx + dy * dy <= rc * rc + 1e-9) offs.push(dx, dy);
      const W = this.W, H = this.H, src = this.occ, dst = out.occ;
      for (let iy = 0; iy < H; iy++)
        for (let ix = 0; ix < W; ix++) {
          if (!src[iy * W + ix]) continue;
          // nur Randzellen müssen aufgebläht werden
          if (ix > 0 && iy > 0 && ix < W - 1 && iy < H - 1 &&
              src[iy * W + ix - 1] && src[iy * W + ix + 1] && src[(iy - 1) * W + ix] && src[(iy + 1) * W + ix]) continue;
          for (let k = 0; k < offs.length; k += 2) {
            const x = ix + offs[k], y = iy + offs[k + 1];
            if (x >= 0 && y >= 0 && x < W && y < H) dst[y * W + x] = 1;
          }
        }
      return out;
    }
  }
  MRS.GridMap = GridMap;

  /* ---------- Karte 1: complexMap aus der MATLAB-Toolbox (2 Zellen/m) ---------- */
  const COMPLEX_MAP_ROWS = [
    "1111111111111111111111111111111111111111111111111111",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1111111111111100001111111000010000111111111111100001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000001111100001111111000010000111111100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000011111111111111111111111111111111111100000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000000000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1000000000000000001000000000010000000000100000000001",
    "1111111111111100001111111000010000111111111111111111",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1000000000000000000000000000000000000000000000000001",
    "1111100001111100001111111000010000111111100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1000000001000000001000000000010000000000100000000001",
    "1111111111111111111111111111111111111111111111111111"
  ];

  /* ---------- Karte: generiertes Labyrinth (Recursive Backtracker) ---------- */
  function buildMaze(res) {
    const n = 10, cell = 2, t = 0.3, off = t / 2;
    const m = new GridMap(n * cell + t, n * cell + t, res);
    const rng = MRS.makeRng(7);
    const east = [], north = [], seen = [];
    for (let i = 0; i < n * n; i++) { east.push(true); north.push(true); seen.push(false); }
    const stack = [0];
    seen[0] = true;
    while (stack.length) {
      const c = stack[stack.length - 1], cx = c % n, cy = (c / n) | 0;
      const nb = [];
      if (cx < n - 1 && !seen[c + 1]) nb.push([c + 1, 'E', c]);
      if (cx > 0 && !seen[c - 1]) nb.push([c - 1, 'E', c - 1]);
      if (cy < n - 1 && !seen[c + n]) nb.push([c + n, 'N', c]);
      if (cy > 0 && !seen[c - n]) nb.push([c - n, 'N', c - n]);
      if (!nb.length) { stack.pop(); continue; }
      const [nx, dir, owner] = nb[Math.floor(rng() * nb.length)];
      if (dir === 'E') east[owner] = false; else north[owner] = false;
      seen[nx] = true;
      stack.push(nx);
    }
    m.border(t);
    for (let cy = 0; cy < n; cy++)
      for (let cx = 0; cx < n; cx++) {
        const c = cy * n + cx;
        const x0 = off + cx * cell, y0 = off + cy * cell;
        if (east[c] && cx < n - 1) m.fillRect(x0 + cell - off, y0 - off, x0 + cell + off, y0 + cell + off);
        if (north[c] && cy < n - 1) m.fillRect(x0 - off, y0 + cell - off, x0 + cell + off, y0 + cell + off);
      }
    return m;
  }

  function buildOffice(res) {
    const m = new GridMap(24, 16, res);
    const t = 0.2;
    m.border(t);
    // Flur zwischen y = 7 und y = 9
    m.fillRect(0, 6.9, 24, 7.1);
    m.fillRect(0, 8.9, 24, 9.1);
    // Raumtrennwände
    for (const x of [6, 12, 18]) {
      m.fillRect(x - 0.1, 0, x + 0.1, 7);
      m.fillRect(x - 0.1, 9, x + 0.1, 16);
    }
    // Türen (1.2 m breit)
    const doorsBottom = [4.2, 7.0, 15.5, 19.2], doorsTop = [1.0, 9.8, 13.0, 21.6];
    for (const d of doorsBottom) m.clearRect(d, 6.8, d + 1.2, 7.2);
    for (const d of doorsTop) m.clearRect(d, 8.8, d + 1.2, 9.2);
    // Möbel
    m.fillRect(1.0, 4.0, 3.5, 5.2);   // Tisch Raum 1
    m.fillRect(7.5, 1.0, 8.5, 4.5);   // Regal Raum 2
    m.fillRect(9.8, 2.5, 11.2, 5.5);
    m.fillCircle(15, 3.5, 1.0);        // runder Tisch
    m.fillRect(19.5, 1.5, 22.5, 2.5);
    m.fillRect(20.5, 4.0, 23.8, 5.0);
    m.fillRect(2.0, 11.5, 4.5, 13.5);
    m.fillRect(7.5, 13.5, 10.5, 15.8);
    m.fillCircle(9.0, 11.2, 0.7);
    m.fillRect(13.5, 10.5, 14.5, 14.5);
    m.fillRect(15.8, 12.0, 17.8, 13.0);
    m.fillRect(19.2, 10.2, 20.4, 12.8);
    // Hindernisse im Flur
    m.fillRect(10.0, 7.6, 10.8, 8.4);
    m.fillCircle(16.5, 8.0, 0.35);
    return m;
  }

  function buildObstacles(res) {
    const m = new GridMap(20, 20, res);
    m.border(0.2);
    const rects = [
      [3, 3.5, 6, 4.5], [8, 1.5, 9, 6], [11, 3, 15, 4], [16.5, 1, 17.5, 5],
      [2, 7, 3, 11], [5, 8, 9, 9], [11, 6.5, 12, 11], [14, 8, 18.5, 9],
      [3, 13, 7, 14], [9, 12.5, 10, 17.5], [12, 14, 16, 15], [17, 11, 18, 15],
      [5.5, 16, 7, 18.8]
    ];
    for (const r of rects) m.fillRect(r[0], r[1], r[2], r[3]);
    const circ = [[6.5, 11.2, 1.0], [14.2, 11.8, 1.1], [13.5, 17.6, 0.8], [3.2, 16.8, 0.7], [17.5, 17.2, 0.6], [15.2, 6.0, 0.7]];
    for (const c of circ) m.fillCircle(c[0], c[1], c[2]);
    return m;
  }

  function buildUTrap(res) {
    const m = new GridMap(20, 14, res);
    m.border(0.2);
    const th = 0.4;
    m.fillRect(12 - th, 3.5, 12, 10.5);     // Rückwand
    m.fillRect(7, 3.5, 12, 3.5 + th);       // unterer Arm
    m.fillRect(7, 10.5 - th, 12, 10.5);     // oberer Arm
    m.fillRect(15.5, 0, 16, 4.5);
    m.fillRect(15.5, 9.5, 16, 14);
    return m;
  }

  function buildNarrow(res) {
    const m = new GridMap(20, 12, res);
    m.border(0.2);
    m.fillRect(9.8, 0, 10.2, 5.4);
    m.fillRect(9.8, 6.4, 10.2, 12);
    m.fillRect(3, 4, 7, 5);
    m.fillRect(4, 8, 5, 12);
    m.fillRect(13, 2, 14, 7);
    m.fillRect(15.5, 8.5, 19, 9.3);
    m.fillCircle(16.5, 4.5, 1.0);
    return m;
  }

  const D2R = Math.PI / 180;
  MRS.MAPS = [
    {
      id: 'complex', name: 'Labyrinth',
      desc: 'Verwinkeltes Labyrinth mit vielen Sackgassen und engen Gängen, 26 m × 20,5 m.',
      build: (res) => GridMap.fromRows(COMPLEX_MAP_ROWS, 2, res),
      start: [2, 2, 0], goal: [18, 18, -90 * D2R]
    },
    {
      id: 'maze', name: 'Generiertes Labyrinth 10×10',
      desc: 'Zufallslabyrinth (Recursive Backtracker, Seed 7) mit 2 m breiten Zellen.',
      build: buildMaze, start: [1.15, 1.15, 0], goal: [19.15, 19.15, 90 * D2R]
    },
    {
      id: 'office', name: 'Bürogebäude',
      desc: 'Acht Räume an einem Flur, verbunden über Türen, mit Möbeln.',
      build: buildOffice, start: [2, 2, 0], goal: [22, 13.5, 180 * D2R]
    },
    {
      id: 'atrium', name: 'Bürogebäude mit Atrium',
      desc: 'Etage eines Hochschul- und Bürogebäudes, 107,8 m × 72,3 m: Hörsäle, Seminarräume und Büros um ein Atrium mit Wendeltreppe, Flure mit Stützen. Vordefinierte Route: 222 m Rundgang um das Atrium mit Abstechern in den Bürotrakt und einen Hörsaal, dessen Reihen einzeln abgefahren werden.',
      build: (res) => GridMap.fromRLE(MRS.ATRIUM_MAP, res),
      start: [28.05, 37.75, 27 * D2R], goal: [82.75, 22.95, -90 * D2R],
      // Rundgang (geschlossen): Nordflur, Bürotrakt, Diagonale am Bürotrakt, Hörsaal Ost (schlangenförmig
      // durch alle Gänge zwischen den Tischreihen), Südflur, Westseite. Mindestabstand ≈ 0,35 m, in den
      // Gängen zwischen den Reihen ≈ 0,25 m.
      route: [[27.87, 37.89], [51.95, 50.64], [67.83, 49.89], [69.31, 48.44], [73.24, 52.91], [82.17, 44.15],
        [82.08, 43.17], [78.36, 39.77], [78.82, 39.38], [89.22, 28.05], [91.27, 30.46], [94.0, 27.75],
        [96.24, 26.6], [98.45, 22.64], [97.6, 20.95], [91.25, 27.11], [90.05, 25.91], [96.4, 19.74],
        [95.3, 18.65], [91.56, 20.83], [88.58, 22.79], [86.13, 23.83], [84.94, 25.87], [70.43, 11.34],
        [32.6, 11.44], [32.81, 11.75], [33.13, 14.92], [32.05, 16.01], [28.03, 26.44], [27.87, 37.89]]
    },
    {
      id: 'obstacles', name: 'Hindernisfeld',
      desc: 'Offene Fläche mit verstreuten rechteckigen und runden Hindernissen.',
      build: buildObstacles, start: [1.2, 1.2, 0], goal: [18.8, 18.8, 90 * D2R]
    },
    {
      id: 'utrap', name: 'U-Falle',
      desc: 'U-förmiges Hindernis direkt auf der Verbindungslinie Start–Ziel (lokale Minima, Bug2).',
      build: buildUTrap, start: [3, 7, 0], goal: [18, 7, 0]
    },
    {
      id: 'narrow', name: 'Enger Durchgang',
      desc: 'Trennwand mit nur 1 m breiter Öffnung – schwierig für stichprobenbasierte Planer.',
      build: buildNarrow, start: [2, 2, 0], goal: [18, 10.5, 0]
    }
  ];
})();
