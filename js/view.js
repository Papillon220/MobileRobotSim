/* ==========================================================================
   MobileRobotSim – Canvas-Darstellung: Weltansicht (Karte) und Zeitverlaufs-Plots
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;

  const C = (MRS.COLORS = {
    planned: '#008300',     // geplante Bahn (grün)
    driven: '#e34948',      // gefahrene Bahn (rot)
    faulty: '#8a8984',      // fehlerbehaftete Bahn (grau)
    tree: 'rgba(42,120,214,0.22)',
    treeNode: 'rgba(42,120,214,0.65)',
    explored: [158, 197, 244],
    mline: '#d55181',
    wall: '#2b2a28',
    inflated: '#dcdbd5',
    free: '#ffffff',
    grid: '#e1e0d9',
    axis: '#c3c2b7',
    muted: '#6f6d68',
    ink: '#0b0b0b',
    robot: '#1c5cab',
    series: ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
  });

  function niceStep(span, target) {
    const raw = span / Math.max(1, target);
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const f = raw / p;
    return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
  }
  MRS.niceStep = niceStep;

  /* ------------------------------- Weltansicht -------------------------------- */
  class WorldView {
    constructor(canvas, opts = {}) {
      this.c = canvas;
      this.ctx = canvas.getContext('2d');
      this.m = { l: 44, r: 14, t: 12, b: 32, ...(opts.margin || {}) };
      this.x0 = 0; this.y0 = 0; this.w = 10; this.h = 10;
      this.onResize = null;
      new ResizeObserver(() => this.resize()).observe(canvas);
      this.resize();
    }

    setWorld(x0, y0, w, h) {
      this.x0 = x0; this.y0 = y0; this.w = w; this.h = h;
      this._fit();
    }

    resize() {
      const r = this.c.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      this.dpr = dpr;
      this.cw = Math.max(50, r.width);
      this.ch = Math.max(50, r.height);
      this.c.width = Math.round(this.cw * dpr);
      this.c.height = Math.round(this.ch * dpr);
      this._fit();
      if (this.onResize) this.onResize();
    }

    _fit() {
      const m = this.m, aw = this.cw - m.l - m.r, ah = this.ch - m.t - m.b;
      this.s = Math.max(1e-6, Math.min(aw / this.w, ah / this.h));
      this.ox = m.l + (aw - this.w * this.s) / 2 - this.x0 * this.s;
      this.oy = m.t + (ah - this.h * this.s) / 2 + (this.h + this.y0) * this.s;
    }

    X(x) { return this.ox + x * this.s; }
    Y(y) { return this.oy - y * this.s; }
    toWorld(px, py) { return [(px - this.ox) / this.s, (this.oy - py) / this.s]; }

    begin(bg = '#fcfcfb') {
      const g = this.ctx;
      g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      g.fillStyle = bg;
      g.fillRect(0, 0, this.cw, this.ch);
      return g;
    }

    image(img, x0, y0, w, h, smooth = false) {
      const g = this.ctx;
      g.imageSmoothingEnabled = smooth;
      g.drawImage(img, this.X(x0), this.Y(y0 + h), w * this.s, h * this.s);
      g.imageSmoothingEnabled = true;
    }

    /** Achsen mit Teilstrichen in Metern; grid = true zeichnet Gitterlinien. */
    axes(grid = false) {
      const g = this.ctx;
      const xs = this.X(this.x0), xe = this.X(this.x0 + this.w), ys = this.Y(this.y0), ye = this.Y(this.y0 + this.h);
      const st = niceStep(Math.max(this.w, this.h), Math.max(4, Math.min(12, (xe - xs) / 70)));
      g.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
      g.fillStyle = C.muted;
      g.strokeStyle = grid ? C.grid : C.axis;
      g.lineWidth = 1;
      g.textAlign = 'center'; g.textBaseline = 'top';
      const dec = st < 1 ? 1 : 0;
      for (let v = Math.ceil(this.x0 / st) * st; v <= this.x0 + this.w + 1e-9; v += st) {
        const px = Math.round(this.X(v)) + 0.5;
        g.beginPath();
        if (grid) { g.moveTo(px, ye); g.lineTo(px, ys); } else { g.moveTo(px, ys); g.lineTo(px, ys + 4); }
        g.stroke();
        g.fillText(v.toFixed(dec), px, ys + 6);
      }
      g.textAlign = 'right'; g.textBaseline = 'middle';
      for (let v = Math.ceil(this.y0 / st) * st; v <= this.y0 + this.h + 1e-9; v += st) {
        const py = Math.round(this.Y(v)) + 0.5;
        g.beginPath();
        if (grid) { g.moveTo(xs, py); g.lineTo(xe, py); } else { g.moveTo(xs - 4, py); g.lineTo(xs, py); }
        g.stroke();
        g.fillText(v.toFixed(dec), xs - 7, py);
      }
      g.fillStyle = C.muted;
      g.textAlign = 'right'; g.textBaseline = 'bottom';
      g.fillText('x [m]', xe, this.ch - 2);
      g.save();
      g.translate(11, ye); g.rotate(-Math.PI / 2);
      g.textAlign = 'right'; g.textBaseline = 'middle';
      g.fillText('y [m]', 0, 0);
      g.restore();
    }

    polyline(pts, color, width = 2, dash = null, alpha = 1, halo = false) {
      if (!pts || pts.length < 2) return;
      if (halo) this.polyline(pts, 'rgba(255,255,255,0.9)', width + 3, dash, alpha);
      const g = this.ctx;
      g.save();
      g.globalAlpha = alpha;
      g.strokeStyle = color;
      g.lineWidth = width;
      g.lineJoin = 'round'; g.lineCap = 'round';
      if (dash) g.setLineDash(dash);
      g.beginPath();
      // null-Einträge unterbrechen die Linie (z. B. „Stift angehoben“)
      let up = true;
      for (const p of pts) {
        if (!p) { up = true; continue; }
        if (up) g.moveTo(this.X(p[0]), this.Y(p[1])); else g.lineTo(this.X(p[0]), this.Y(p[1]));
        up = false;
      }
      g.stroke();
      g.restore();
    }

    /** Pose-Marker: Kreis (Start) bzw. Kreuz (Ziel) mit Orientierungspfeil. */
    poseMarker(p, kind, color) {
      const g = this.ctx, x = this.X(p[0]), y = this.Y(p[1]);
      g.save();
      g.strokeStyle = color; g.fillStyle = color; g.lineWidth = 2.5;
      const L = 18;
      g.beginPath(); g.moveTo(x, y); g.lineTo(x + L * Math.cos(p[2]), y - L * Math.sin(p[2])); g.stroke();
      const hx = x + L * Math.cos(p[2]), hy = y - L * Math.sin(p[2]);
      g.beginPath();
      g.moveTo(hx, hy);
      g.lineTo(hx - 6 * Math.cos(p[2] - 0.5), hy + 6 * Math.sin(p[2] - 0.5));
      g.lineTo(hx - 6 * Math.cos(p[2] + 0.5), hy + 6 * Math.sin(p[2] + 0.5));
      g.closePath(); g.fill();
      if (kind === 'start') {
        g.fillStyle = '#ffffff';
        g.beginPath(); g.arc(x, y, 7, 0, 2 * Math.PI); g.fill(); g.stroke();
      } else {
        g.lineWidth = 3;
        g.beginPath(); g.moveTo(x - 7, y - 7); g.lineTo(x + 7, y + 7); g.moveTo(x + 7, y - 7); g.lineTo(x - 7, y + 7); g.stroke();
      }
      g.restore();
    }

    /** Zwei-Rad-Roboter (Differentialantrieb), mind. minPx Radius auf dem Bildschirm. */
    robot(p, color, trackWidth = 0.2, minPx = 9, alpha = 1) {
      const g = this.ctx;
      const scale = Math.max(1, minPx / ((trackWidth / 2) * this.s));
      const r = (trackWidth / 2) * this.s * scale;
      const x = this.X(p[0]), y = this.Y(p[1]);
      g.save();
      g.globalAlpha = alpha;
      g.translate(x, y);
      g.rotate(-p[2]);
      // Räder
      g.fillStyle = '#2b2a28';
      const ww = r * 0.85, wh = r * 0.32;
      g.fillRect(-ww / 2, -r - wh / 2, ww, wh);
      g.fillRect(-ww / 2, r - wh / 2, ww, wh);
      // Körper
      g.fillStyle = color;
      g.strokeStyle = '#ffffff';
      g.lineWidth = 1.5;
      g.beginPath(); g.arc(0, 0, r * 0.92, 0, 2 * Math.PI); g.fill(); g.stroke();
      // Fahrtrichtung
      g.fillStyle = '#ffffff';
      g.beginPath(); g.moveTo(r * 0.75, 0); g.lineTo(-r * 0.25, -r * 0.42); g.lineTo(-r * 0.25, r * 0.42); g.closePath(); g.fill();
      g.restore();
    }

    dot(p, color, rad = 3.5) {
      const g = this.ctx;
      g.save();
      g.fillStyle = color;
      g.strokeStyle = '#ffffff'; g.lineWidth = 1.5;
      g.beginPath(); g.arc(this.X(p[0]), this.Y(p[1]), rad, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.restore();
    }
  }
  MRS.WorldView = WorldView;

  /** Belegungskarte als Bild (1 Pixel je Zelle); optional aufgeblähte Zone grau. */
  MRS.renderMapImage = function (raw, inflated) {
    const cv = document.createElement('canvas');
    cv.width = raw.W; cv.height = raw.H;
    const g = cv.getContext('2d');
    const img = g.createImageData(raw.W, raw.H);
    const hex = (h) => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
    const cw = hex(C.wall), ci = hex(C.inflated), cf = hex(C.free);
    for (let iy = 0; iy < raw.H; iy++)
      for (let ix = 0; ix < raw.W; ix++) {
        const i = iy * raw.W + ix, o = ((raw.H - 1 - iy) * raw.W + ix) * 4;
        const col = raw.occ[i] ? cw : inflated && inflated.occ[i] ? ci : cf;
        img.data[o] = col[0]; img.data[o + 1] = col[1]; img.data[o + 2] = col[2]; img.data[o + 3] = 255;
      }
    g.putImageData(img, 0, 0);
    return cv;
  };

  /** Distanzfeld als Grünverlauf (dunkel nahe Ziel, hell fern), Wände schwarz. */
  MRS.renderDistanceImage = function (raw, D) {
    const cv = document.createElement('canvas');
    cv.width = raw.W; cv.height = raw.H;
    const g = cv.getContext('2d');
    const img = g.createImageData(raw.W, raw.H);
    let mn = Infinity, mx = -Infinity;
    for (const v of D) if (isFinite(v)) { if (v < mn) mn = v; if (v > mx) mx = v; }
    for (let iy = 0; iy < raw.H; iy++)
      for (let ix = 0; ix < raw.W; ix++) {
        const i = iy * raw.W + ix, o = ((raw.H - 1 - iy) * raw.W + ix) * 4;
        let r = 0, gg = 0, b = 0;
        if (!raw.occ[i] && isFinite(D[i])) {
          const t = (D[i] - mn) / (mx - mn || 1);
          r = 255 * (0.0 + 0.3 * t); gg = 255 * (0.2 + 0.8 * t); b = 255 * (0.0 + 0.3 * t);
        } else if (!raw.occ[i]) { r = gg = b = 235; }
        img.data[o] = r; img.data[o + 1] = gg; img.data[o + 2] = b; img.data[o + 3] = 255;
      }
    g.putImageData(img, 0, 0);
    cv.dRange = [mn, mx];
    return cv;
  };

  /** Expandierte Zellen (A*) als halbtransparentes Bild. */
  MRS.renderMaskImage = function (mask, W, H, rgb, alpha) {
    const cv = document.createElement('canvas');
    cv.width = W; cv.height = H;
    const g = cv.getContext('2d');
    const img = g.createImageData(W, H);
    for (let iy = 0; iy < H; iy++)
      for (let ix = 0; ix < W; ix++) {
        if (!mask[iy * W + ix]) continue;
        const o = ((H - 1 - iy) * W + ix) * 4;
        img.data[o] = rgb[0]; img.data[o + 1] = rgb[1]; img.data[o + 2] = rgb[2]; img.data[o + 3] = alpha;
      }
    g.putImageData(img, 0, 0);
    return cv;
  };

  /* ---------------------------- Zeitverlaufs-Plot ----------------------------- */
  class TimePlot {
    /**
     * opts: { title, unit, series: [{label, color, dash}] }
     * Daten werden per draw(t, arrays[], visible[]) übergeben.
     */
    constructor(canvas, opts) {
      this.c = canvas;
      this.g = canvas.getContext('2d');
      this.o = opts;
      this.hoverX = null;
      this.window = 30;
      this.last = null;
      new ResizeObserver(() => this._resize()).observe(canvas);
      canvas.addEventListener('mousemove', (e) => { const r = canvas.getBoundingClientRect(); this.hoverX = e.clientX - r.left; this.redraw(); });
      canvas.addEventListener('mouseleave', () => { this.hoverX = null; this.redraw(); });
      this._resize();
    }

    _resize() {
      const r = this.c.getBoundingClientRect();
      this.dpr = window.devicePixelRatio || 1;
      this.w = Math.max(50, r.width); this.h = Math.max(50, r.height);
      this.c.width = Math.round(this.w * this.dpr); this.c.height = Math.round(this.h * this.dpr);
      this.redraw();
    }

    redraw() { if (this.last) this.draw(...this.last); else this.draw([], [], []); }

    draw(t, arrays, visible) {
      this.last = [t, arrays, visible];
      const g = this.g, W = this.w, H = this.h, o = this.o;
      g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      g.fillStyle = '#fcfcfb'; g.fillRect(0, 0, W, H);
      const m = { l: 46, r: 10, t: 24, b: 22 };
      const pw = W - m.l - m.r, ph = H - m.t - m.b;
      g.font = '600 12px system-ui, -apple-system, "Segoe UI", sans-serif';
      g.fillStyle = C.ink; g.textAlign = 'left'; g.textBaseline = 'middle';
      g.fillText(o.title, m.l, 11);
      g.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
      g.fillStyle = C.muted; g.textAlign = 'right';
      g.fillText(o.unit, W - m.r, 11);

      const n = t.length;
      const tmax = n ? t[n - 1] : 0;
      const t0 = this.window > 0 ? Math.max(0, tmax - this.window) : 0;
      const t1 = this.window > 0 ? Math.max(t0 + this.window, tmax) : Math.max(tmax, 1);
      let i0 = 0;
      { let lo = 0, hi = n - 1; while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] < t0) lo = mid + 1; else hi = mid; } i0 = Math.max(0, lo - 1); }
      let ymin = Infinity, ymax = -Infinity;
      arrays.forEach((a, k) => {
        if (!visible[k] || !a) return;
        for (let i = i0; i < n; i++) { const v = a[i]; if (v == null || !isFinite(v)) continue; if (v < ymin) ymin = v; if (v > ymax) ymax = v; }
      });
      if (!isFinite(ymin)) { ymin = -1; ymax = 1; }
      if (ymax - ymin < 1e-6) { ymin -= 0.5; ymax += 0.5; }
      const pad = (ymax - ymin) * 0.08; ymin -= pad; ymax += pad;
      const ys = niceStep(ymax - ymin, Math.max(2, Math.floor(ph / 28)));
      ymin = Math.floor(ymin / ys) * ys; ymax = Math.ceil(ymax / ys) * ys;
      const X = (tt) => m.l + ((tt - t0) / (t1 - t0)) * pw;
      const Y = (v) => m.t + ph - ((v - ymin) / (ymax - ymin)) * ph;

      // Gitter + Achsen
      g.strokeStyle = C.grid; g.lineWidth = 1;
      g.fillStyle = C.muted; g.textAlign = 'right'; g.textBaseline = 'middle';
      const dec = ys < 0.1 ? 2 : ys < 1 ? 1 : 0;
      for (let v = ymin; v <= ymax + 1e-9; v += ys) {
        const py = Math.round(Y(v)) + 0.5;
        g.beginPath(); g.moveTo(m.l, py); g.lineTo(m.l + pw, py); g.stroke();
        g.fillText(v.toFixed(dec), m.l - 6, py);
      }
      const xs = niceStep(t1 - t0, Math.max(2, Math.floor(pw / 70)));
      g.textAlign = 'center'; g.textBaseline = 'top';
      for (let v = Math.ceil(t0 / xs) * xs; v <= t1 + 1e-9; v += xs) {
        const px = Math.round(X(v)) + 0.5;
        g.strokeStyle = C.grid;
        g.beginPath(); g.moveTo(px, m.t); g.lineTo(px, m.t + ph); g.stroke();
        if (px < m.l + pw - 22) g.fillText(v.toFixed(xs < 1 ? 1 : 0), px, m.t + ph + 5);
      }
      g.strokeStyle = C.axis;
      g.beginPath(); g.moveTo(m.l, m.t + ph + 0.5); g.lineTo(m.l + pw, m.t + ph + 0.5); g.stroke();
      g.textAlign = 'right'; g.textBaseline = 'bottom';
      g.fillText('t [s]', W - m.r, H - 1);

      // Linien (dezimiert auf ca. 2 Punkte pro Pixel)
      g.save();
      g.beginPath(); g.rect(m.l, m.t - 2, pw, ph + 4); g.clip();
      const stride = Math.max(1, Math.floor((n - i0) / (2 * pw)));
      arrays.forEach((a, k) => {
        if (!visible[k] || !a) return;
        const s = o.series[k];
        g.strokeStyle = s.color; g.lineWidth = 2; g.lineJoin = 'round';
        g.setLineDash(s.dash || []);
        g.beginPath();
        let started = false;
        for (let i = i0; i < n; i += stride) {
          const v = a[i];
          if (v == null || !isFinite(v)) { started = false; continue; }
          if (!started) { g.moveTo(X(t[i]), Y(v)); started = true; } else g.lineTo(X(t[i]), Y(v));
        }
        if (started && n - 1 >= i0 && a[n - 1] != null) g.lineTo(X(t[n - 1]), Y(a[n - 1]));
        g.stroke();
      });
      g.restore();
      g.setLineDash([]);

      // Hover: Fadenkreuz + Tooltip
      if (this.hoverX != null && n > 1 && this.hoverX >= m.l && this.hoverX <= m.l + pw) {
        const th = t0 + ((this.hoverX - m.l) / pw) * (t1 - t0);
        let lo = i0, hi = n - 1;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] < th) lo = mid + 1; else hi = mid; }
        const i = lo;
        const px = X(t[i]);
        g.strokeStyle = C.axis; g.lineWidth = 1;
        g.beginPath(); g.moveTo(px + 0.5, m.t); g.lineTo(px + 0.5, m.t + ph); g.stroke();
        const lines = [`t = ${t[i].toFixed(2)} s`];
        arrays.forEach((a, k) => {
          if (!visible[k] || !a || a[i] == null) return;
          g.fillStyle = o.series[k].color; g.strokeStyle = '#fcfcfb'; g.lineWidth = 2;
          g.beginPath(); g.arc(px, Y(a[i]), 4, 0, 2 * Math.PI); g.fill(); g.stroke();
          lines.push([o.series[k].label, a[i].toFixed(3), o.series[k].color]);
        });
        g.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
        const tw = Math.max(...lines.map((l) => g.measureText(typeof l === 'string' ? l : `${l[0]}: ${l[1]}`).width)) + 26;
        const bh = lines.length * 15 + 8;
        let bx = px + 10; if (bx + tw > W - 4) bx = px - 10 - tw;
        const by = m.t + 4;
        g.fillStyle = 'rgba(255,255,255,0.96)'; g.strokeStyle = 'rgba(11,11,11,0.15)'; g.lineWidth = 1;
        g.beginPath(); g.roundRect ? g.roundRect(bx, by, tw, bh, 6) : g.rect(bx, by, tw, bh); g.fill(); g.stroke();
        g.textAlign = 'left'; g.textBaseline = 'middle';
        lines.forEach((l, j) => {
          const yy = by + 11 + j * 15;
          if (typeof l === 'string') { g.fillStyle = C.ink; g.fillText(l, bx + 8, yy); return; }
          g.fillStyle = l[2]; g.fillRect(bx + 8, yy - 1, 10, 3);
          g.fillStyle = C.ink; g.fillText(`${l[0]}: ${l[1]}`, bx + 22, yy);
        });
      }
    }
  }
  MRS.TimePlot = TimePlot;
})();
