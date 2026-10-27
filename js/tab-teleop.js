/* ==========================================================================
   MobileRobotSim – Reiter 2: Manuelle Steuerung eines Differentialantriebs
   Pfeiltasten bzw. W/A/S/D → (v, ω); optionale Fehlersimulation mit Offset auf ω.
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { fmt, deg, wrapAngle, COLORS } = MRS;
  const $ = (id) => document.getElementById(id);
  const SUB_DT = 0.001;   // Integrationsschritt [s]
  const LOG_DT = 0.02;    // Abtastung der Plots [s]

  const S = {
    active: false, paused: false, lastTs: 0,
    keys: new Set(),
    vT: 0, wT: 0, v: 0, w: 0,
    pose: [0, 0, 0], fpose: null, faultOn: false,
    t: 0, nextLog: 0, dist: 0,
    path: [], fpath: [],
    log: null,
    bounds: null,
    startTh: 0,   // Startorientierung (Manöver)
    man: null,    // laufendes Manöver { plan, name }
    pen: true     // Bahn zeichnen (bei Manövern zeitweise „Stift angehoben“)
  };
  let view, plots;

  function newLog() {
    return { t: [], x: [], y: [], th: [], thw: [], v: [], w: [], fx: [], fy: [], fth: [], fthw: [], fw: [] };
  }

  /* ------------------------------ Initialisierung ------------------------------ */
  function init() {
    view = new MRS.WorldView($('teleCanvas'));
    view.onResize = draw;
    const red = COLORS.driven, gray = COLORS.faulty;
    const mk = (id, title, unit) => new MRS.TimePlot($(id), {
      title, unit,
      series: [{ label: 'ideal', color: red }, { label: 'mit Offset', color: gray, dash: [6, 4] }]
    });
    plots = {
      x: mk('plotX', 'Position x', 'm'),
      y: mk('plotY', 'Position y', 'm'),
      th: mk('plotTh', 'Orientierung θ', '°'),
      v: mk('plotV', 'Eingang v', 'm/s'),
      w: mk('plotW', 'Eingang ω', 'rad/s')
    };

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', (e) => { if (S.keys.delete(MRS.driveKey(e))) updateKeypad(); });
    window.addEventListener('blur', () => { S.keys.clear(); updateKeypad(); });

    // Bildschirm-Tasten (Maus/Touch)
    document.querySelectorAll('#keypad button').forEach((b) => {
      const k = b.dataset.key;
      const down = (e) => { e.preventDefault(); press(k); };
      const up = (e) => { e.preventDefault(); if (S.keys.delete(k)) updateKeypad(); };
      b.addEventListener('pointerdown', down);
      b.addEventListener('pointerup', up);
      b.addEventListener('pointerleave', up);
      b.addEventListener('pointercancel', up);
    });

    $('tMode').addEventListener('change', () => {
      document.getElementById('tab-teleop').classList.toggle('mode-step', $('tMode').value === 'step');
      S.vT = 0; S.wT = 0;
    });
    $('fOn').addEventListener('change', () => setFault($('fOn').checked));
    $('fOffR').addEventListener('input', () => { $('fOff').value = $('fOffR').value; });
    $('fOff').addEventListener('change', () => { $('fOffR').value = $('fOff').value; });
    $('tWin').addEventListener('change', () => { for (const p of Object.values(plots)) p.window = parseFloat($('tWin').value); drawPlots(); });
    $('tThUnit').addEventListener('change', drawPlots);
    $('tPause').addEventListener('click', () => { S.paused = !S.paused; $('tPause').textContent = S.paused ? '▶ Weiter' : '❚❚ Pause'; });
    $('tReset').addEventListener('click', () => reset());
    $('mSel').addEventListener('change', () => { $('mSize').value = MANEUVERS[$('mSel').value].def; updateManeuverUI(); });
    $('mStart').addEventListener('click', () => (S.man ? stopManeuver() : startManeuver()));
    $('mDemo').addEventListener('click', startDemo);
    updateManeuverUI();
    $('teleCanvas').addEventListener('pointerdown', () => $('teleCanvas').focus());

    reset();
    requestAnimationFrame(loop);
  }

  function reset(th0 = 0) {
    S.man = null; S.pen = true;
    updateManeuverBtn();
    S.startTh = th0;
    S.pose = [0, 0, th0];
    S.v = S.w = S.vT = S.wT = 0;
    S.t = 0; S.nextLog = 0; S.dist = 0;
    S.path = [[0, 0]];
    S.fpose = S.faultOn ? [0, 0, th0] : null;
    S.fpath = S.faultOn ? [[0, 0]] : [];
    S.unTh = th0; S.funTh = th0;
    S.log = newLog();
    S.bounds = [-5, -5, 5, 5];
    view.setWorld(-5, -5, 10, 10);
    logSample();
    draw(); drawPlots(); updateReadouts();
  }

  function setFault(on) {
    S.faultOn = on;
    $('faultCard').classList.toggle('off', !on);
    if (on) {
      // Fehlerbehafteter Roboter startet an der aktuellen Pose
      S.fpose = S.pose.slice();
      S.funTh = S.unTh;
      S.fpath = [[S.pose[0], S.pose[1]]];
    } else {
      S.fpose = null;
    }
  }

  /* --------------------------------- Tastatur ---------------------------------- */
  const KEYS = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '];

  function onKeyDown(e) {
    if (S.demo) return; // Demo: keine manuelle Steuerung (Esc beendet über MRS.enterDemo)
    const k = MRS.driveKey(e);
    if (!S.active || !KEYS.includes(k)) return;
    const t = e.target;
    if (t && (t.tagName === 'SELECT' || (t.tagName === 'INPUT' && t.type !== 'range' && t.type !== 'checkbox'))) return;
    e.preventDefault();
    if (e.repeat && $('tMode').value === 'hold') return;
    press(k);
  }

  function press(k) {
    $('focusHint').hidden = true;
    if (S.man) stopManeuver();
    const step = $('tMode').value === 'step';
    const vmax = parseFloat($('tVmax').value) || 1, wmax = parseFloat($('tWmax').value) || 1;
    if (k === ' ') { S.vT = 0; S.wT = 0; S.v = 0; S.w = 0; S.keys.clear(); updateKeypad(); flash(' '); return; }
    if (step) {
      const dv = parseFloat($('tDv').value) || 0.1, dw = parseFloat($('tDw').value) || 0.2;
      if (k === 'ArrowUp') S.vT = Math.min(vmax, S.vT + dv);
      if (k === 'ArrowDown') S.vT = Math.max(-vmax, S.vT - dv);
      if (k === 'ArrowLeft') S.wT = Math.min(wmax, S.wT + dw);
      if (k === 'ArrowRight') S.wT = Math.max(-wmax, S.wT - dw);
      S.vT = Math.round(S.vT * 1000) / 1000; S.wT = Math.round(S.wT * 1000) / 1000;
      flash(k);
    } else {
      S.keys.add(k);
      updateKeypad();
    }
  }

  function flash(k) {
    const b = document.querySelector(`#keypad button[data-key="${k === ' ' ? ' ' : k}"]`);
    if (!b) return;
    b.classList.add('down');
    setTimeout(() => { if (!S.keys.has(k)) b.classList.remove('down'); }, 120);
  }

  function updateKeypad() {
    document.querySelectorAll('#keypad button').forEach((b) => b.classList.toggle('down', S.keys.has(b.dataset.key)));
  }

  /* -------------------------------- Simulation --------------------------------- */
  /** Exakte Integration der Unicycle-Kinematik bei konstantem (v, ω) über dt. */
  function integrate(p, v, w, dt) {
    const th = p[2];
    if (Math.abs(w) < 1e-9) return [p[0] + v * Math.cos(th) * dt, p[1] + v * Math.sin(th) * dt, th];
    const th2 = th + w * dt;
    return [p[0] + (v / w) * (Math.sin(th2) - Math.sin(th)), p[1] - (v / w) * (Math.cos(th2) - Math.cos(th)), th2];
  }

  const ramp = (cur, tgt, rate, dt) => (tgt > cur ? Math.min(tgt, cur + rate * dt) : Math.max(tgt, cur - rate * dt));

  function stepSim(dt) {
    const vmax = parseFloat($('tVmax').value) || 1, wmax = parseFloat($('tWmax').value) || 1;
    const av = parseFloat($('tAv').value) || 1, aw = parseFloat($('tAw').value) || 1;
    if ($('tMode').value === 'hold') {
      S.vT = (S.keys.has('ArrowUp') ? vmax : 0) - (S.keys.has('ArrowDown') ? vmax : 0);
      S.wT = (S.keys.has('ArrowLeft') ? wmax : 0) - (S.keys.has('ArrowRight') ? wmax : 0);
    } else {
      S.vT = Math.max(-vmax, Math.min(vmax, S.vT));
      S.wT = Math.max(-wmax, Math.min(wmax, S.wT));
    }
    const off = S.faultOn ? parseFloat($('fOff').value) || 0 : 0;
    let n = Math.round(dt / SUB_DT);
    while (n-- > 0) {
      if (S.man) {
        // Manöver: Sollwerte exakt vorgeben (ohne Rampe), Runde für Runde bis zum Anhalten
        const m = S.man, plan = m.plan;
        let tau = S.t - m.t0;
        if (tau >= plan.T - 1e-12) {
          if (S.demo) { S.demoNext = true; break; } // Demo: jeder Durchlauf beginnt neu im Ursprung
          m.t0 += plan.T; m.lap++; tau -= plan.T;
        }
        if (tau + SUB_DT <= plan.T) [S.v, S.w] = plan.cmd(tau, SUB_DT);
        else {
          // Schritt über das Rundenende: Ende dieser und Anfang der nächsten Runde anteilig
          const d1 = plan.T - tau, d2 = SUB_DT - d1;
          const c1 = plan.cmd(tau, d1), c2 = plan.cmd(0, d2);
          S.v = (c1[0] * d1 + c2[0] * d2) / SUB_DT;
          S.w = (c1[1] * d1 + c2[1] * d2) / SUB_DT;
        }
        S.vT = S.v; S.wT = S.w;
        S.pen = plan.pen(tau + SUB_DT / 2);
      } else {
        S.v = ramp(S.v, S.vT, av, SUB_DT);
        S.w = ramp(S.w, S.wT, aw, SUB_DT);
      }
      const np = integrate(S.pose, S.v, S.w, SUB_DT);
      S.dist += Math.hypot(np[0] - S.pose[0], np[1] - S.pose[1]);
      S.unTh += S.w * SUB_DT;
      S.pose = [np[0], np[1], wrapAngle(np[2])];
      if (S.fpose) {
        const fp = integrate(S.fpose, S.v, S.w + off, SUB_DT);
        S.funTh += (S.w + off) * SUB_DT;
        S.fpose = [fp[0], fp[1], wrapAngle(fp[2])];
      }
      S.t += SUB_DT;
      if (S.t >= S.nextLog - 1e-9) logSample();
    }
  }

  function logSample() {
    const L = S.log;
    const off = S.faultOn ? parseFloat($('fOff').value) || 0 : 0;
    L.t.push(S.t);
    L.x.push(S.pose[0]); L.y.push(S.pose[1]); L.th.push(S.unTh); L.thw.push(S.pose[2]);
    L.v.push(S.v); L.w.push(S.w);
    if (S.fpose) {
      L.fx.push(S.fpose[0]); L.fy.push(S.fpose[1]); L.fth.push(S.funTh); L.fthw.push(S.fpose[2]); L.fw.push(S.w + off);
    } else {
      L.fx.push(null); L.fy.push(null); L.fth.push(null); L.fthw.push(null); L.fw.push(null);
    }
    S.nextLog = S.t + LOG_DT;
    // Pfade nur bei Bewegung erweitern; angehobener Stift unterbricht die Linie
    // ideale Bahn eines Manövers ab der 2. Runde deckungsgleich → nicht erneut speichern
    if (!(S.man && S.man.lap > 1)) extendPath(S.path, S.pose);
    if (S.fpose) extendPath(S.fpath, S.fpose);
    growBounds(S.pose);
    if (S.fpose) growBounds(S.fpose);
  }

  function extendPath(path, p) {
    const lp = path[path.length - 1];
    if (!S.pen) { if (lp) path.push(null); return; }
    if (!lp || Math.hypot(p[0] - lp[0], p[1] - lp[1]) > 0.005) path.push([p[0], p[1]]);
  }

  /* --------------------------------- Manöver ----------------------------------- */
  /**
   * Fahrplan aus Abschnitten konstanter Sollwerte { T, v, w, pen }.
   * cmd() mittelt über den Integrationsschritt, damit Abschnittsgrenzen exakt eingehalten werden.
   */
  function segmentPlan(segs, th0 = 0) {
    let T = 0;
    const b = segs.map((s) => { const a = T; T += s.T; return { ...s, a, e: T }; });
    return {
      T, th0,
      cmd(t, dt) {
        let v = 0, w = 0;
        for (const s of b) {
          const o = Math.min(t + dt, s.e) - Math.max(t, s.a);
          if (o > 0) { v += s.v * o; w += s.w * o; }
        }
        return [v / dt, w / dt];
      },
      pen(t) { const s = b.find((x) => t < x.e); return s ? s.pen !== false : true; }
    };
  }

  /** Fahrplan entlang einer parametrischen Kurve f(u), u ∈ [u0, u1]: ω = v · κ(s) (Krümmungsvorsteuerung). */
  function curvePlan(f, u0, u1, v, N = 8000) {
    const s = [0], phi = [];
    let prev = f(u0);
    for (let i = 1; i <= N; i++) {
      const p = f(u0 + ((u1 - u0) * i) / N);
      const a = Math.atan2(p[1] - prev[1], p[0] - prev[0]);
      phi.push(phi.length ? phi[phi.length - 1] + wrapAngle(a - phi[phi.length - 1]) : a);
      s.push(s[i - 1] + Math.hypot(p[0] - prev[0], p[1] - prev[1]));
      prev = p;
    }
    // Krümmung an den inneren Stützstellen aus der Richtungsänderung benachbarter Sehnen
    const kap = [], sk = [];
    for (let i = 0; i < N - 1; i++) {
      kap.push((phi[i + 1] - phi[i]) / ((s[i + 2] - s[i]) / 2));
      sk.push(s[i + 1]);
    }
    const kappa = (x) => {
      let lo = 0, hi = sk.length - 1;
      if (x <= sk[0]) return kap[0];
      if (x >= sk[hi]) return kap[hi];
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (sk[m] <= x) lo = m; else hi = m; }
      return kap[lo] + ((kap[hi] - kap[lo]) * (x - sk[lo])) / (sk[hi] - sk[lo]);
    };
    // Starttangente: Richtung der ersten Sehne, zurückgerechnet auf den Kurvenanfang
    const th0 = phi[0] - (kap[0] * s[1]) / 2;
    return { T: s[N] / v, th0, cmd: (t, dt) => [v, v * kappa(v * (t + dt / 2))], pen: () => true };
  }

  /**
   * Turtle-Grafik für Fahrten aus Geraden, Drehungen auf der Stelle und Kreisbögen.
   * Führt die exakte Sollpose mit; moveTo fährt bei Bedarf rückwärts, statt um 180° zu wenden.
   */
  function turtle(v, wt) {
    const segs = [];
    let x = 0, y = 0, h = 0, pen = true;
    const T = {
      segs,
      pen(p) { pen = p; return T; },
      /** Drehung auf der Stelle um d (> 0: links). */
      turn(d) {
        if (Math.abs(d) > 1e-9) segs.push({ T: Math.abs(d) / wt, v: 0, w: Math.sign(d) * wt, pen });
        h += d; return T;
      },
      /** Gerade der Länge L (< 0: rückwärts). */
      go(L) {
        if (Math.abs(L) > 1e-9) segs.push({ T: Math.abs(L) / v, v: Math.sign(L) * v, w: 0, pen });
        x += L * Math.cos(h); y += L * Math.sin(h); return T;
      },
      /** Kreisbogen mit Radius R über den Winkel a (> 0: links, < 0: rechts). */
      arc(R, a) {
        const sg = Math.sign(a);
        segs.push({ T: (R * Math.abs(a)) / v, v, w: (sg * v) / R, pen });
        const cx = x - R * sg * Math.sin(h), cy = y + R * sg * Math.cos(h);
        h += a;
        x = cx + R * sg * Math.sin(h); y = cy - R * sg * Math.cos(h);
        return T;
      },
      /** Zum Punkt fahren; mit rev = true rückwärts, wenn das weniger Drehung erfordert. */
      moveTo(tx, ty, rev = true) {
        const d = Math.hypot(tx - x, ty - y);
        if (d < 1e-9) return T;
        const diff = wrapAngle(Math.atan2(ty - y, tx - x) - h);
        if (rev && Math.abs(diff) > Math.PI / 2 + 1e-9) return T.turn(wrapAngle(diff + Math.PI)).go(-d);
        return T.turn(diff).go(d);
      },
      /** Zurück zur Startpose (Ursprung, Ausrichtung in x-Richtung). */
      home() { return T.moveTo(0, 0).turn(wrapAngle(-h)); }
    };
    return T;
  }

  /** Schriftzug „DHBW“ mit Buchstabenhöhe H; Grundlinie auf der x-Achse, Beginn im Ursprung. */
  function dhbwPlan(H, v, wt) {
    const t = turtle(v, wt), gap = 0.3;
    let x0 = 0;
    const P = (x, y) => [x0 + x * H, y * H];
    // D: Stamm, oberer Balken, Halbkreis rechts herum, unterer Balken zurück
    t.moveTo(...P(0, 1), false).moveTo(...P(0.1, 1), false).arc(0.5 * H, -Math.PI).moveTo(...P(0, 0), false);
    // H: linker Stamm, zurück zur Mitte, Querbalken, rechter Stamm
    x0 += (0.6 + gap) * H;
    t.pen(false).moveTo(...P(0, 0)).pen(true)
      .moveTo(...P(0, 1)).moveTo(...P(0, 0.5)).moveTo(...P(0.6, 0.5)).moveTo(...P(0.6, 1)).moveTo(...P(0.6, 0));
    // B: Stamm, oberer (schmalerer) und unterer Bogen
    x0 += (0.6 + gap) * H;
    t.pen(false).moveTo(...P(0, 0)).pen(true)
      .moveTo(...P(0, 1)).moveTo(...P(0.3, 1), false).arc(0.25 * H, -Math.PI).moveTo(...P(0, 0.5), false)
      .moveTo(...P(0.35, 0.5), false).arc(0.25 * H, -Math.PI).moveTo(...P(0, 0), false);
    // W: vier Striche
    x0 += (0.6 + gap) * H;
    t.pen(false).moveTo(...P(0, 1)).pen(true)
      .moveTo(...P(0.225, 0), false).moveTo(...P(0.45, 0.6), false).moveTo(...P(0.675, 0), false).moveTo(...P(0.9, 1), false);
    // mit angehobenem Stift zurück zum Start, damit sich der Schriftzug wiederholen lässt
    t.pen(false).home();
    return segmentPlan(t.segs);
  }

  const MANEUVERS = {
    eight: {
      size: 'Halbe Breite a [m]', def: 2, turns: false,
      desc: 'Lemniskate (Bernoulli) mit dem Kreuzungspunkt im Ursprung. Start dort unter 45°, zuerst die rechte Schleife im Uhrzeigersinn, dann die linke gegen den Uhrzeigersinn. Die Drehrate folgt stetig der Bahnkrümmung (ω = v·κ, an den Scheiteln 3·v/a); Rechts- und Linksdrehung heben sich insgesamt auf.',
      plan: (a, v) => curvePlan((u) => { const c = Math.cos(u), d = 1 + c * c; return [(a * Math.sin(u)) / d, (a * Math.sin(u) * c) / d]; }, 0, 2 * Math.PI, v)
    },
    circle: {
      size: 'Radius R [m]', def: 1.5, turns: false,
      desc: 'Konstante Geschwindigkeit v und Drehrate ω = v/R. Start im Ursprung in x-Richtung, Mittelpunkt bei (0, R). Nach einer vollen Runde steht der Roboter im Idealfall wieder an der Startpose.',
      plan: (R, v) => segmentPlan([{ T: (2 * Math.PI * R) / v, v, w: v / R }])
    },
    squareL: {
      size: 'Seitenlänge [m]', def: 3, turns: true,
      desc: 'Vier Geraden mit 90°-Drehungen auf der Stelle nach links. Zusammen mit der Fahrt im Uhrzeigersinn ein üblicher Test zur Kalibrierung der Odometrie: Systematische Fehler zeigen sich als Abstand zwischen Start- und Endpunkt, der je nach Umlaufrichtung unterschiedlich ausfällt.',
      plan: (L, v, w) => { const t = turtle(v, w); for (let i = 0; i < 4; i++) t.go(L).turn(Math.PI / 2); return segmentPlan(t.segs); }
    },
    squareR: {
      size: 'Seitenlänge [m]', def: 3, turns: true,
      desc: 'Vier Geraden mit 90°-Drehungen auf der Stelle nach rechts. Ein Drehratenfehler wirkt hier gegen die Umlaufrichtung statt mit ihr – im Vergleich zum Quadrat gegen den Uhrzeigersinn lassen sich so unterschiedliche Fehlerursachen trennen.',
      plan: (L, v, w) => { const t = turtle(v, w); for (let i = 0; i < 4; i++) t.go(L).turn(-Math.PI / 2); return segmentPlan(t.segs); }
    },
    outback: {
      size: 'Strecke [m]', def: 4, turns: true,
      desc: 'Geradeaus, 180°-Wende auf der Stelle, zurück und erneute Wende. Im Idealfall steht der Roboter danach exakt an der Startpose.',
      plan: (L, v, w) => { const t = turtle(v, w); t.go(L).turn(Math.PI).go(L).turn(Math.PI); return segmentPlan(t.segs); }
    },
    dhbw: {
      size: 'Buchstabenhöhe [m]', def: 2, turns: true,
      desc: 'Der Roboter schreibt „DHBW“ aus Geraden, Halbkreisen und Drehungen auf der Stelle; manche Strecken werden rückwärts gefahren. Zwischen den Buchstaben wird der „Stift angehoben“: Die Bahn ist dort unterbrochen.',
      plan: (H, v, w) => dhbwPlan(H, v, w)
    }
  };

  function updateManeuverUI() {
    const m = MANEUVERS[$('mSel').value];
    $('mDesc').textContent = m.desc;
    $('mSizeLbl').textContent = m.size;
    $('mWLbl').hidden = !m.turns;
  }

  function updateManeuverBtn() {
    $('mStart').textContent = S.man ? '■ Manöver anhalten' : '▶ Manöver fahren';
    $('mStart').classList.toggle('go', !S.man);
  }

  function startManeuver() {
    const def = MANEUVERS[$('mSel').value];
    const size = Math.max(0.1, parseFloat($('mSize').value) || def.def);
    const v = Math.max(0.01, parseFloat($('mV').value) || 0.5);
    const w = Math.max(0.05, parseFloat($('mW').value) || 1);
    const plan = def.plan(size, v, w);
    reset(plan.th0);
    S.man = { plan, name: $('mSel').selectedOptions[0].textContent, t0: 0, lap: 1 };
    S.keys.clear(); updateKeypad();
    if (S.paused) $('tPause').click();
    $('focusHint').hidden = true;
    updateManeuverBtn();
  }

  function stopManeuver() {
    // ideale Bahn an der aktuellen Pose fortsetzen (ab Runde 2 wurde sie nicht mehr erweitert)
    if (S.man && S.man.lap > 1) S.path.push(null, [S.pose[0], S.pose[1]]);
    S.man = null; S.pen = true;
    S.v = S.w = S.vT = S.wT = 0;
    updateManeuverBtn();
  }

  /* ------------------------------ Demo-Vollbild ------------------------------- */
  /** DHBW-Schriftzug im Vollbild endlos abfahren (nur Karte und Trajektorien, keine Bedienelemente). */
  function startDemo() {
    if (S.demo) return;
    if ($('mSel').value !== 'dhbw') { $('mSel').value = 'dhbw'; $('mSize').value = MANEUVERS.dhbw.def; updateManeuverUI(); }
    S.demo = { runs: 0, speed: 1 };
    for (const p of Object.values(plots)) p.window = 0; // Verlauf des ganzen Durchlaufs
    S.demo.ctl = MRS.enterDemo($('teleCenter'), onDemoExit, {
      values: [0.25, 0.5, 1, 2, 3, 5],
      get: () => S.demo.speed,
      set: (v) => { S.demo.speed = v; }
    });
    demoRun();
  }

  /** Neuer Durchlauf: Zustand zurücksetzen, Ansicht auf den Schriftzug einpassen. */
  function demoRun() {
    S.demo.runs++;
    startManeuver();
    const H = Math.max(0.1, parseFloat($('mSize').value) || MANEUVERS.dhbw.def), m = 1.2;
    S.bounds = [-m, -m, 3.6 * H + m, H + m];
    view.setWorld(S.bounds[0], S.bounds[1], S.bounds[2] - S.bounds[0], S.bounds[3] - S.bounds[1]);
    draw(); drawPlots();
  }

  /** Nach dem Beenden (Esc, Doppelklick, Vollbild verlassen): Ausgangszustand wiederherstellen. */
  function onDemoExit() {
    S.demo = null; S.demoNext = false;
    stopManeuver();
    for (const p of Object.values(plots)) p.window = parseFloat($('tWin').value);
    drawPlots();
  }

  function growBounds(p) {
    const b = S.bounds, m = 1.0;
    let ch = false;
    if (p[0] - m < b[0]) { b[0] = p[0] - m; ch = true; }
    if (p[1] - m < b[1]) { b[1] = p[1] - m; ch = true; }
    if (p[0] + m > b[2]) { b[2] = p[0] + m; ch = true; }
    if (p[1] + m > b[3]) { b[3] = p[1] + m; ch = true; }
    if (ch) {
      // in 2-m-Schritten wachsen, damit die Ansicht ruhig bleibt
      b[0] = Math.floor(b[0] / 2) * 2; b[1] = Math.floor(b[1] / 2) * 2;
      b[2] = Math.ceil(b[2] / 2) * 2; b[3] = Math.ceil(b[3] / 2) * 2;
      view.setWorld(b[0], b[1], b[2] - b[0], b[3] - b[1]);
    }
  }

  let plotAcc = 0;
  function loop(ts) {
    requestAnimationFrame(loop);
    const dtReal = Math.min(0.05, (ts - (S.lastTs || ts)) / 1000);
    S.lastTs = ts;
    if (!S.active || S.paused) return;
    stepSim(dtReal * (S.demo ? S.demo.speed : 1)); // Demo: einstellbares Tempo
    if (S.demoNext) { S.demoNext = false; demoRun(); }
    draw();
    updateReadouts();
    plotAcc += dtReal;
    if (plotAcc >= 1 / 30) { plotAcc = 0; drawPlots(); }
  }

  /* --------------------------------- Zeichnen ---------------------------------- */
  function draw() {
    if (!view) return;
    view.begin('#fcfcfb');
    view.axes(true);
    const g = view.ctx;
    // Ursprungsachsen hervorheben
    g.strokeStyle = COLORS.axis; g.lineWidth = 1.2;
    g.beginPath();
    g.moveTo(view.X(view.x0), view.Y(0)); g.lineTo(view.X(view.x0 + view.w), view.Y(0));
    g.moveTo(view.X(0), view.Y(view.y0)); g.lineTo(view.X(0), view.Y(view.y0 + view.h));
    g.stroke();
    view.poseMarker([0, 0, S.startTh], 'start', '#2b2a28');
    const tw = parseFloat($('tTrack').value) || 0.2;
    if (S.fpose) {
      view.polyline(S.fpath, COLORS.faulty, 2, [6, 4]);
      view.robot(S.fpose, '#8a8984', tw, 9, 0.85);
    }
    view.polyline(S.path, COLORS.driven, S.demo ? 6 : 2.2); // Demo: kräftigere Linie für die Fernwirkung
    view.robot(S.pose, COLORS.robot, tw, 10);
    const lines = [`v = ${fmt(S.v, 2)} m/s   (Soll ${fmt(S.vT, 2)})`, `ω = ${fmt(S.w, 2)} rad/s (Soll ${fmt(S.wT, 2)})`];
    if (S.demo && S.man) lines.push(`DHBW-Demo – Durchlauf ${S.demo.runs}, ${Math.min(100, Math.round((100 * S.t) / S.man.plan.T))} % · Tempo ${S.demo.speed.toLocaleString('de-DE')}×`);
    else if (S.man) lines.push(`Manöver: ${S.man.name} – Runde ${S.man.lap},${Math.min(100, Math.round((100 * (S.t - S.man.t0)) / S.man.plan.T))} %`);
    if (S.faultOn) lines.push(`Fehler: Δω = ${fmt(parseFloat($('fOff').value) || 0, 3)} rad/s`);
    if (S.paused) lines.push('⏸ pausiert');
    $('teleHud').textContent = lines.join('\n');
  }

  function drawPlots() {
    const L = S.log;
    const unit = $('tThUnit').value;
    const k = unit === 'rad' ? 1 : 180 / Math.PI;
    const th = unit === 'wrap' ? L.thw : L.th, fth = unit === 'wrap' ? L.fthw : L.fth;
    const thS = th.map((v) => v * k), fthS = fth.map((v) => (v == null ? null : v * k));
    plots.th.o.unit = unit === 'rad' ? 'rad' : '°';
    const has = L.fx.some((v) => v != null);
    plots.x.draw(L.t, [L.x, L.fx], [true, has]);
    plots.y.draw(L.t, [L.y, L.fy], [true, has]);
    plots.th.draw(L.t, [thS, fthS], [true, has]);
    plots.v.draw(L.t, [L.v], [true]);
    plots.w.draw(L.t, [L.w, L.fw], [true, has]);
  }

  function updateReadouts() {
    const tw = parseFloat($('tTrack').value) || 0.2;
    $('sT').textContent = fmt(S.t, 2);
    $('sX').textContent = fmt(S.pose[0], 2);
    $('sY').textContent = fmt(S.pose[1], 2);
    $('sTh').textContent = fmt(deg(S.pose[2]), 1);
    $('sV').textContent = fmt(S.v, 2);
    $('sW').textContent = fmt(S.w, 2);
    $('sWheels').textContent = `${fmt(S.v - (S.w * tw) / 2, 2)} / ${fmt(S.v + (S.w * tw) / 2, 2)}`;
    $('sDist').textContent = fmt(S.dist, 2);
    if (S.fpose) {
      const off = parseFloat($('fOff').value) || 0;
      $('fX').textContent = `${fmt(S.fpose[0], 2)} / ${fmt(S.fpose[1], 2)}`;
      $('fTh').textContent = fmt(deg(S.fpose[2]), 1);
      $('fW').textContent = fmt(S.w + off, 3);
      $('fErr').textContent = fmt(Math.hypot(S.fpose[0] - S.pose[0], S.fpose[1] - S.pose[1]), 3);
      $('fErrTh').textContent = fmt(deg(wrapAngle(S.fpose[2] - S.pose[2])), 1);
    } else {
      for (const id of ['fX', 'fTh', 'fW', 'fErr', 'fErrTh']) $(id).textContent = '–';
    }
  }

  MRS.TeleopTab = {
    init,
    setActive(a) {
      S.active = a;
      S.keys.clear(); updateKeypad();
      if (a) { view.resize(); for (const p of Object.values(plots)) p._resize(); $('teleCanvas').focus({ preventScroll: true }); }
    }
  };
})();
