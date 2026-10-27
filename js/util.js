/* ==========================================================================
   MobileRobotSim – Hilfsfunktionen (Zufallszahlen, Winkel, Heap, Dubins)
   ========================================================================== */
(function () {
  'use strict';
  const MRS = (window.MRS = window.MRS || {});

  /** Reproduzierbarer Zufallszahlengenerator (mulberry32), entspricht rng(seed). */
  MRS.makeRng = function (seed) {
    let a = (seed >>> 0) || 1;
    const f = function () {
      a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    /** Ganzzahl im geschlossenen Intervall [lo, hi] (wie randi). */
    f.int = (lo, hi) => lo + Math.floor(f() * (hi - lo + 1));
    return f;
  };

  MRS.wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  MRS.deg = (r) => (r * 180) / Math.PI;
  MRS.rad = (d) => (d * Math.PI) / 180;

  /** Binärer Min-Heap mit (Priorität, Wert)-Paaren. */
  class MinHeap {
    constructor() { this.k = []; this.v = []; }
    get size() { return this.k.length; }
    push(key, val) {
      const k = this.k, v = this.v;
      let i = k.length;
      k.push(key); v.push(val);
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (k[p] <= key) break;
        k[i] = k[p]; v[i] = v[p]; i = p;
      }
      k[i] = key; v[i] = val;
    }
    pop() {
      const k = this.k, v = this.v;
      const top = v[0];
      const lk = k.pop(), lv = v.pop();
      const n = k.length;
      if (n > 0) {
        let i = 0;
        for (;;) {
          let c = 2 * i + 1;
          if (c >= n) break;
          if (c + 1 < n && k[c + 1] < k[c]) c++;
          if (k[c] >= lk) break;
          k[i] = k[c]; v[i] = v[c]; i = c;
        }
        k[i] = lk; v[i] = lv;
      }
      return top;
    }
  }
  MRS.MinHeap = MinHeap;

  /* ---------------- Dubins-Pfade (kürzeste Verbindung mit min. Wenderadius) --------------- */
  const TWO_PI = 2 * Math.PI;
  const mod2pi = (a) => { a %= TWO_PI; return a < 0 ? a + TWO_PI : a; };
  const TYPES = {
    LSL: ['L', 'S', 'L'], RSR: ['R', 'S', 'R'], LSR: ['L', 'S', 'R'],
    RSL: ['R', 'S', 'L'], RLR: ['R', 'L', 'R'], LRL: ['L', 'R', 'L']
  };

  function dubinsShortest(q0, q1, rho) {
    const dx = q1[0] - q0[0], dy = q1[1] - q0[1];
    const d = Math.hypot(dx, dy) / rho;
    const th = mod2pi(Math.atan2(dy, dx));
    const a = mod2pi(q0[2] - th), b = mod2pi(q1[2] - th);
    const sa = Math.sin(a), sb = Math.sin(b), ca = Math.cos(a), cb = Math.cos(b);
    const cab = Math.cos(a - b), d2 = d * d;
    let best = null;
    const cand = (type, t, p, q) => {
      const L = t + p + q;
      if (!best || L < best.L) best = { type, seg: [t, p, q], L };
    };
    let p2, tmp, p;
    // LSL
    p2 = 2 + d2 - 2 * cab + 2 * d * (sa - sb);
    if (p2 >= 0) { tmp = Math.atan2(cb - ca, d + sa - sb); cand('LSL', mod2pi(tmp - a), Math.sqrt(p2), mod2pi(b - tmp)); }
    // RSR
    p2 = 2 + d2 - 2 * cab + 2 * d * (sb - sa);
    if (p2 >= 0) { tmp = Math.atan2(ca - cb, d - sa + sb); cand('RSR', mod2pi(a - tmp), Math.sqrt(p2), mod2pi(tmp - b)); }
    // LSR
    p2 = -2 + d2 + 2 * cab + 2 * d * (sa + sb);
    if (p2 >= 0) { p = Math.sqrt(p2); tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2, p); cand('LSR', mod2pi(tmp - a), p, mod2pi(tmp - b)); }
    // RSL
    p2 = -2 + d2 + 2 * cab - 2 * d * (sa + sb);
    if (p2 >= 0) { p = Math.sqrt(p2); tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2, p); cand('RSL', mod2pi(a - tmp), p, mod2pi(b - tmp)); }
    // RLR
    tmp = (6 - d2 + 2 * cab + 2 * d * (sa - sb)) / 8;
    if (Math.abs(tmp) <= 1) {
      p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(a - Math.atan2(ca - cb, d - sa + sb) + p / 2);
      cand('RLR', t, p, mod2pi(a - b - t + p));
    }
    // LRL
    tmp = (6 - d2 + 2 * cab + 2 * d * (sb - sa)) / 8;
    if (Math.abs(tmp) <= 1) {
      p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(-a - Math.atan2(ca - cb, d + sa - sb) + p / 2);
      cand('LRL', t, p, mod2pi(b - a - t + p));
    }
    if (!best) return null;
    return { q0: q0.slice(), rho, type: best.type, seg: best.seg, length: best.L * rho };
  }

  function segStep(t, q, kind) {
    if (kind === 'L') return [q[0] + Math.sin(q[2] + t) - Math.sin(q[2]), q[1] - Math.cos(q[2] + t) + Math.cos(q[2]), q[2] + t];
    if (kind === 'R') return [q[0] - Math.sin(q[2] - t) + Math.sin(q[2]), q[1] + Math.cos(q[2] - t) - Math.cos(q[2]), q[2] - t];
    return [q[0] + Math.cos(q[2]) * t, q[1] + Math.sin(q[2]) * t, q[2]];
  }

  /** Zustand [x, y, theta] nach Bogenlänge s entlang des Dubins-Pfades. */
  function dubinsSample(path, s) {
    const tp = Math.max(0, Math.min(s, path.length)) / path.rho;
    const k = TYPES[path.type], [p1, p2] = path.seg;
    const qi = [0, 0, path.q0[2]];
    let q;
    if (tp < p1) q = segStep(tp, qi, k[0]);
    else {
      const q1 = segStep(p1, qi, k[0]);
      if (tp < p1 + p2) q = segStep(tp - p1, q1, k[1]);
      else q = segStep(tp - p1 - p2, segStep(p2, q1, k[1]), k[2]);
    }
    return [q[0] * path.rho + path.q0[0], q[1] * path.rho + path.q0[1], MRS.wrapAngle(q[2])];
  }

  MRS.Dubins = { shortest: dubinsShortest, sample: dubinsSample };

  /** Formatierung von Zahlen im deutschen Format. */
  MRS.fmt = (v, d = 2) => (v == null || !isFinite(v) ? '–' : v.toLocaleString('de-DE', { minimumFractionDigits: d, maximumFractionDigits: d }));

  /**
   * Demo-Vollbild für einen Bereich el (Klasse „demo“): Vollbild anfordern, Bildschirm wach halten,
   * Hinweis zum Beenden kurz einblenden. Beenden mit Esc, Doppelklick oder Verlassen des Vollbilds;
   * danach wird onExit aufgerufen. Rückgabe: { exit() }.
   * Optional speed = { values: [...], get(), set(v) }: Tempo mit den Tasten + / − oder über eine kleine
   * Leiste, die bei Mausbewegung kurz eingeblendet wird.
   */
  MRS.enterDemo = function (el, onExit, speed) {
    const st = { active: true, wake: null };
    const hint = el.querySelector('.demo-exit');
    let bar = null, barTimer = 0;
    // Autorenangaben aus dem Seitenkopf auch im Vollbild – als eigene Kopfzeile über der Ansicht
    const author = document.querySelector('.app-header .author');
    const badge = author ? author.cloneNode(true) : null;
    if (badge) { badge.classList.add('demo-author'); el.prepend(badge); }
    const showBar = () => {
      if (!bar) return;
      bar.querySelector('b').textContent = speed.get().toLocaleString('de-DE') + '×';
      bar.classList.add('show'); el.classList.add('demo-pointer');
      clearTimeout(barTimer);
      barTimer = setTimeout(() => { bar.classList.remove('show'); el.classList.remove('demo-pointer'); }, 2500);
    };
    const stepSpeed = (dir) => {
      const vals = speed.values, cur = speed.get();
      // nächster Wert in der gewünschten Richtung (auch wenn der aktuelle Wert nicht in der Liste steht)
      const next = dir > 0 ? vals.find((v) => v > cur + 1e-9) : vals.slice().reverse().find((v) => v < cur - 1e-9);
      if (next != null) speed.set(next);
      showBar();
    };
    const onFs = () => { if (!document.fullscreenElement) st.exit(); };
    const onKey = (e) => {
      if (e.key === 'Escape') { e.preventDefault(); st.exit(); }
      else if (speed && (e.key === '+' || e.key === '=')) { e.preventDefault(); stepSpeed(1); }
      else if (speed && e.key === '-') { e.preventDefault(); stepSpeed(-1); }
    };
    if (speed) {
      bar = document.createElement('div');
      bar.className = 'demo-speed';
      bar.innerHTML = '<span>Tempo</span><button type="button" aria-label="langsamer">−</button><b></b><button type="button" aria-label="schneller">+</button>';
      const [minus, plus] = bar.querySelectorAll('button');
      minus.addEventListener('click', () => stepSpeed(-1));
      plus.addEventListener('click', () => stepSpeed(1));
      bar.addEventListener('dblclick', (e) => e.stopPropagation()); // schnelles Klicken beendet nicht die Demo
      el.append(bar);
      el.addEventListener('mousemove', showBar);
    }
    st.exit = () => {
      if (!st.active) return;
      st.active = false;
      document.removeEventListener('fullscreenchange', onFs);
      window.removeEventListener('keydown', onKey, true);
      el.removeEventListener('dblclick', st.exit);
      if (bar) { el.removeEventListener('mousemove', showBar); bar.remove(); clearTimeout(barTimer); el.classList.remove('demo-pointer'); }
      clearTimeout(st.timer);
      if (hint) hint.classList.remove('show');
      if (badge) badge.remove();
      el.classList.remove('demo');
      document.body.classList.remove('demo-active');
      if (st.wake) st.wake.release().catch(() => {});
      if (document.fullscreenElement) document.exitFullscreen().catch(() => {});
      onExit();
    };
    el.classList.add('demo');
    document.body.classList.add('demo-active');
    document.addEventListener('fullscreenchange', onFs);
    window.addEventListener('keydown', onKey, true);
    el.addEventListener('dblclick', st.exit);
    if (el.requestFullscreen) el.requestFullscreen().catch(() => {});
    // Bildschirm nicht abschalten lassen (sofern vom Browser unterstützt)
    if (navigator.wakeLock) navigator.wakeLock.request('screen').then((l) => { if (st.active) st.wake = l; else l.release(); }).catch(() => {});
    if (hint) { hint.classList.add('show'); st.timer = setTimeout(() => hint.classList.remove('show'), 4000); }
    return st;
  };

  /** Fahrtaste normieren: W/A/S/D werden auf die entsprechenden Pfeiltasten abgebildet. */
  const WASD = { w: 'ArrowUp', a: 'ArrowLeft', s: 'ArrowDown', d: 'ArrowRight' };
  MRS.driveKey = (e) => (e.ctrlKey || e.metaKey || e.altKey ? e.key : WASD[e.key.toLowerCase()] || e.key);
})();
