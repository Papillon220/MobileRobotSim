/* ==========================================================================
   MobileRobotSim – Reiter 1: Bahnplanung, Pure-Pursuit-Pfadverfolgung, Vergleich
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { fmt, rad, deg, COLORS } = MRS;
  const $ = (id) => document.getElementById(id);
  const RES = 10; // Kartenauflösung [Zellen/m], wie im MATLAB-Skript

  // Feste Farbe je Algorithmus für eingeblendete Vergleichsläufe (Farbe folgt dem Algorithmus)
  const ALGO_COLORS = {
    bug2: '#2a78d6', astar: '#eb6834', rrt: '#1baf7a', rrtstar: '#eda100',
    rrtdubins: '#e87ba4', dt: '#6250d6', prm: '#8b5a2b', prmsmooth: '#0d366b'
  };

  const S = {
    mapDef: null, raw: null, inflated: null, imgInfl: null, imgPlain: null, dtImg: null,
    start: [0, 0, 0], goal: [0, 0, 0],
    result: null, resultKey: null, run: null,
    sim: null, running: false, paused: false, acc: 0, lastTs: 0,
    runs: [], nextId: 1,
    clickMode: null, drag: null, mouse: null,
    overlay: null, paramValues: {}, batch: false
  };
  let view;

  /* ------------------------------ Initialisierung ------------------------------ */
  function init() {
    view = new MRS.WorldView($('planCanvas'));
    view.onResize = () => { S.overlay = null; draw(); };

    for (const m of MRS.MAPS) $('mapSel').add(new Option(m.name, m.id));
    for (const a of MRS.ALGORITHMS) $('algoSel').add(new Option(a.name, a.id));
    for (const [k, d] of Object.entries(MRS.PARAM_DEFS)) S.paramValues[k] = d.def;
    $('algoSel').value = 'prmsmooth';

    $('mapSel').addEventListener('change', () => loadMap($('mapSel').value));
    $('inflateR').addEventListener('change', () => { buildInflation(); invalidatePlan(); });
    $('trackW').addEventListener('change', draw);
    $('showInflation').addEventListener('change', draw);
    $('showTree').addEventListener('change', () => { S.overlay = null; draw(); });
    $('algoSel').addEventListener('change', onAlgoChange);
    for (const id of ['sx', 'sy', 'sth', 'gx', 'gy', 'gth']) $(id).addEventListener('change', readPoses);
    $('btnDefaultPoses').addEventListener('click', () => { setPoses(S.mapDef.start, S.mapDef.goal); invalidatePlan(); });
    document.querySelectorAll('#clickMode button[data-mode]').forEach((b) =>
      b.addEventListener('click', () => setClickMode(S.clickMode === b.dataset.mode ? null : b.dataset.mode)));

    $('btnPlan').addEventListener('click', () => plan());
    $('btnStart').addEventListener('click', start);
    $('btnPause').addEventListener('click', togglePause);
    $('btnReset').addEventListener('click', resetSim);
    $('btnBatch').addEventListener('click', runBatch);
    $('btnClear').addEventListener('click', () => { S.runs = []; S.run = null; renderComparison(); renderCurrent(); draw(); });
    $('btnCsv').addEventListener('click', exportCsv);
    $('cmpMetric').addEventListener('change', renderComparison);
    $('cmpThisMap').addEventListener('change', renderComparison);

    const cv = $('planCanvas');
    cv.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);
    cv.addEventListener('mouseleave', () => { S.mouse = null; updateHud(); });

    onAlgoChange();
    loadMap(MRS.MAPS[0].id);
    updateButtons();
  }

  /* ---------------------------------- Karte ------------------------------------ */
  function loadMap(id) {
    stopSim();
    S.mapDef = MRS.MAPS.find((m) => m.id === id);
    $('mapDesc').textContent = S.mapDef.desc;
    S.raw = S.mapDef.build(RES);
    S.imgPlain = MRS.renderMapImage(S.raw, null);
    buildInflation();
    view.setWorld(0, 0, S.raw.width, S.raw.height);
    setPoses(S.mapDef.start, S.mapDef.goal);
    invalidatePlan();
  }

  function buildInflation() {
    const r = Math.max(0, parseFloat($('inflateR').value) || 0);
    S.inflated = S.raw.inflate(r);
    S.imgInfl = MRS.renderMapImage(S.raw, S.inflated);
  }

  function setPoses(s, g) {
    S.start = s.slice(); S.goal = g.slice();
    $('sx').value = s[0].toFixed(2); $('sy').value = s[1].toFixed(2); $('sth').value = Math.round(deg(s[2]));
    $('gx').value = g[0].toFixed(2); $('gy').value = g[1].toFixed(2); $('gth').value = Math.round(deg(g[2]));
  }

  function readPoses() {
    const v = (id) => parseFloat($(id).value) || 0;
    S.start = [v('sx'), v('sy'), rad(v('sth'))];
    S.goal = [v('gx'), v('gy'), rad(v('gth'))];
    invalidatePlan();
  }

  function invalidatePlan() {
    stopSim();
    S.result = null; S.resultKey = null; S.sim = null; S.run = null; S.dtImg = null; S.overlay = null;
    renderCurrent();
    renderComparison();
    updateButtons();
    draw();
  }

  /* -------------------------------- Parameter ---------------------------------- */
  function onAlgoChange() {
    const algo = MRS.ALGORITHMS.find((a) => a.id === $('algoSel').value);
    $('algoDesc').textContent = algo.desc;
    const box = $('algoParams');
    box.innerHTML = '';
    for (const key of algo.params) {
      const d = MRS.PARAM_DEFS[key];
      const lab = document.createElement('label');
      let inp;
      if (d.type === 'select') {
        inp = document.createElement('select');
        for (const [v, t] of d.options) inp.add(new Option(t, v));
        inp.value = S.paramValues[key];
        lab.append(d.label, inp);
      } else if (d.type === 'checkbox') {
        lab.className = 'chk';
        inp = document.createElement('input');
        inp.type = 'checkbox';
        inp.checked = !!S.paramValues[key];
        lab.append(inp, ' ' + d.label);
      } else {
        inp = document.createElement('input');
        inp.type = 'number';
        Object.assign(inp, { min: d.min, max: d.max, step: d.step, value: S.paramValues[key] });
        lab.append(d.label, inp);
      }
      inp.addEventListener('change', () => {
        S.paramValues[key] = d.type === 'checkbox' ? inp.checked : d.type === 'select' ? inp.value : parseFloat(inp.value);
        if (key === 'dtArrows') { S.overlay = null; draw(); }
      });
      box.append(lab);
    }
    // Bug2 benötigt im MATLAB-Skript mehr Fahrzeit (250 s statt 150 s)
    const tm = $('simTmax');
    if (algo.id === 'bug2' && tm.value === '150') tm.value = '250';
    else if (algo.id !== 'bug2' && tm.value === '250') tm.value = '150';
  }

  function plannerParams() {
    return { ...S.paramValues, seed: parseInt($('seed').value, 10) || 0 };
  }

  function ctrlParams() {
    const v = (id) => parseFloat($(id).value);
    return {
      lookahead: v('ppL'), vDesired: v('ppV'), wMax: v('ppW'), headingGain: v('ppKh'),
      overshootMode: parseInt($('ppMode').value, 10), rotateThresh: rad(v('ppRot')),
      terminalDist: v('ppTerm'), goalLinGain: v('ppKg'), goalTolPos: v('ppTolP'), goalTolHead: rad(v('ppTolH')),
      dt: v('simDt'), maxTime: v('simTmax')
    };
  }

  /** Regelparameter für eine Bahn; Zeitlimit wird bei sehr langen Bahnen verlängert. */
  function simParams(result) {
    const P = ctrlParams();
    P.maxTime = Math.max(P.maxTime, Math.ceil((3 * result.length) / P.vDesired));
    return P;
  }

  function planKey() {
    return JSON.stringify([S.mapDef.id, $('inflateR').value, S.start, S.goal, $('algoSel').value, plannerParams()]);
  }

  /* --------------------------------- Planung ----------------------------------- */
  function showBusy(text) {
    $('busyText').textContent = text;
    $('busy').hidden = !text;
  }

  const nextFrame = () => new Promise((r) => requestAnimationFrame(() => setTimeout(r, 0)));

  async function plan(algoId = $('algoSel').value) {
    stopSim();
    const algo = MRS.ALGORITHMS.find((a) => a.id === algoId);
    showBusy(`Plane mit ${algo.name} …`);
    await nextFrame();
    let r;
    try {
      r = MRS.runPlanner(algoId, S.raw, S.inflated, S.start, S.goal, plannerParams());
    } catch (e) {
      console.error(e);
      r = { ok: false, states: [], viz: {}, info: ['Fehler: ' + e.message], planTime: NaN, length: NaN, algoId, algoName: algo.name };
    }
    showBusy(null);
    S.result = r;
    S.resultKey = planKey();
    S.sim = null;
    S.overlay = null;
    S.dtImg = r.viz && r.viz.distField ? MRS.renderDistanceImage(S.raw, r.viz.distField) : null;
    S.run = {
      id: S.nextId++, mapId: S.mapDef.id, mapName: S.mapDef.name, algoId, algoName: algo.name,
      length: r.ok ? r.length : NaN, planTime: r.planTime * 1000, travel: NaN, dist: NaN,
      status: r.ok ? 'geplant' : 'kein Pfad', ok: r.ok, reached: false, collisions: 0,
      planned: r.states.map((q) => [q[0], q[1]]), driven: null, show: false
    };
    S.runs.push(S.run);
    renderCurrent();
    renderComparison();
    updateButtons();
    draw();
    return r;
  }

  /* --------------------------- Simulation (Pfadverfolgung) --------------------------- */
  async function start() {
    if (S.running && !S.paused) return;
    if (S.running && S.paused) { togglePause(); return; }
    if (!S.result || S.resultKey !== planKey()) await plan();
    if (!S.result || !S.result.ok) return;
    if (!S.run || S.run.driven) {
      // erneutes Abfahren derselben Bahn → neuer Eintrag im Vergleich
      const prev = S.run;
      S.run = { ...prev, id: S.nextId++, travel: NaN, dist: NaN, status: 'geplant', reached: false, driven: null, show: false };
      S.runs.push(S.run);
    }
    const P = simParams(S.result);
    S.sim = new MRS.PathFollowSim(S.result.states, S.start, S.goal, P, S.raw, (parseFloat($('trackW').value) || 0.2) / 2);
    S.run.status = 'fährt';
    S.running = true; S.paused = false; S.acc = 0; S.lastTs = performance.now();
    renderCurrent(); renderComparison(); updateButtons();
    const speed = parseFloat($('simSpeed').value);
    if (speed === 0) { while (!S.sim.done) S.sim.step(); finishSim(); return; }
    requestAnimationFrame(loop);
  }

  function loop(ts) {
    if (!S.running) return;
    if (S.paused) { S.lastTs = ts; requestAnimationFrame(loop); return; }
    const speed = parseFloat($('simSpeed').value) || 50;
    const realDt = Math.min(0.1, (ts - S.lastTs) / 1000);
    S.lastTs = ts;
    S.acc += realDt * speed;
    const dt = S.sim.prm.dt;
    let n = 0;
    while (S.acc >= dt && !S.sim.done && n < 5000) { S.sim.step(); S.acc -= dt; n++; }
    draw();
    renderCurrent();
    if (S.sim.done) { finishSim(); return; }
    requestAnimationFrame(loop);
  }

  function finishSim() {
    const sim = S.sim, run = S.run;
    S.running = false; S.paused = false;
    run.travel = sim.t;
    run.dist = sim.dist;
    run.reached = sim.goalReached;
    run.collisions = sim.collisions;
    run.status = sim.goalReached ? 'Ziel erreicht' : 'Zeitlimit';
    run.driven = sim.path.slice();
    renderCurrent(); renderComparison(); updateButtons(); draw();
  }

  function togglePause() {
    if (!S.running) return;
    S.paused = !S.paused;
    updateButtons();
  }

  function stopSim() {
    if (S.running && S.run && !S.run.driven) S.run.status = 'abgebrochen';
    S.running = false; S.paused = false;
  }

  function resetSim() {
    stopSim();
    S.sim = null;
    renderCurrent(); renderComparison(); updateButtons(); draw();
  }

  async function runBatch() {
    if (S.batch) return;
    S.batch = true;
    updateButtons();
    const keepSpeed = $('simSpeed').value;
    for (const a of MRS.ALGORITHMS) {
      $('algoSel').value = a.id;
      onAlgoChange();
      const r = await plan(a.id);
      if (r.ok) {
        S.sim = new MRS.PathFollowSim(r.states, S.start, S.goal, simParams(r), S.raw, (parseFloat($('trackW').value) || 0.2) / 2);
        S.running = true;
        showBusy(`Fahre ${a.name} ab …`);
        await nextFrame();
        while (!S.sim.done) S.sim.step();
        showBusy(null);
        finishSim();
      }
      await nextFrame();
    }
    $('simSpeed').value = keepSpeed;
    S.batch = false;
    updateButtons();
  }

  function updateButtons() {
    const b = S.batch;
    $('btnPlan').disabled = b || (S.running && !S.paused);
    $('btnStart').disabled = b || (S.running && !S.paused);
    $('btnStart').textContent = S.running && S.paused ? '▶ Fortsetzen' : '▶ Start';
    $('btnPause').disabled = b || !S.running;
    $('btnPause').textContent = S.paused ? '▶ Weiter' : '❚❚ Pause';
    $('btnReset').disabled = b;
    $('btnBatch').disabled = b || S.running;
    $('mapSel').disabled = b;
  }

  /* -------------------------------- Mausbedienung -------------------------------- */
  function setClickMode(m) {
    S.clickMode = m;
    document.querySelectorAll('#clickMode button[data-mode]').forEach((b) => b.classList.toggle('active', b.dataset.mode === m));
    $('planCanvas').parentElement.classList.toggle('pick', !!m);
  }

  function evWorld(e) {
    const r = $('planCanvas').getBoundingClientRect();
    return { px: e.clientX - r.left, py: e.clientY - r.top, w: view.toWorld(e.clientX - r.left, e.clientY - r.top) };
  }

  function onMouseDown(e) {
    if (!S.clickMode || S.running) return;
    const p = evWorld(e);
    const [x, y] = p.w;
    if (x < 0 || y < 0 || x > S.raw.width || y > S.raw.height) return;
    const target = S.clickMode === 'start' ? S.start : S.goal;
    target[0] = x; target[1] = y;
    S.drag = { which: S.clickMode, px: p.px, py: p.py };
    setPoses(S.start, S.goal);
    draw();
  }

  function onMouseMove(e) {
    const p = evWorld(e);
    const inside = e.target === $('planCanvas');
    S.mouse = inside ? p.w : null;
    if (S.drag) {
      const dx = p.px - S.drag.px, dy = p.py - S.drag.py;
      if (Math.hypot(dx, dy) > 6) {
        const target = S.drag.which === 'start' ? S.start : S.goal;
        target[2] = Math.atan2(-dy, dx);
        setPoses(S.start, S.goal);
      }
      draw();
    } else if (inside && !S.running) updateHud();
  }

  function onMouseUp() {
    if (!S.drag) return;
    S.drag = null;
    setClickMode(null);
    invalidatePlan();
  }

  /* --------------------------------- Zeichnen ---------------------------------- */
  function buildOverlay() {
    const cv = document.createElement('canvas');
    cv.width = view.c.width; cv.height = view.c.height;
    const r = S.result;
    if (!r || !r.viz) return (S.overlay = cv);
    const g = cv.getContext('2d');
    g.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
    const X = (x) => view.X(x), Y = (y) => view.Y(y);
    const v = r.viz;
    const showTree = $('showTree').checked;
    if (showTree && v.explored) {
      const img = MRS.renderMaskImage(v.explored, v.W, v.H, COLORS.explored, 110);
      g.imageSmoothingEnabled = false;
      g.drawImage(img, X(0), Y(S.raw.height), S.raw.width * view.s, S.raw.height * view.s);
    }
    if (showTree && v.edges) {
      // Transparenz an die Kantendichte anpassen, damit dichte Roadmaps nicht flächig wirken
      const area = S.raw.width * S.raw.height;
      const a = Math.max(0.05, Math.min(0.3, (0.3 * 25 * area) / Math.max(1, v.edges.length) / 20));
      g.strokeStyle = `rgba(42,120,214,${a.toFixed(3)})`; g.lineWidth = 0.8;
      g.beginPath();
      for (const e of v.edges) { g.moveTo(X(e[0]), Y(e[1])); g.lineTo(X(e[2]), Y(e[3])); }
      g.stroke();
    }
    if (showTree && v.polyEdges) {
      g.strokeStyle = COLORS.tree; g.lineWidth = 0.8;
      g.beginPath();
      for (const pl of v.polyEdges) { g.moveTo(X(pl[0][0]), Y(pl[0][1])); for (let i = 1; i < pl.length; i++) g.lineTo(X(pl[i][0]), Y(pl[i][1])); }
      g.stroke();
    }
    if (showTree && v.nodes) {
      g.fillStyle = COLORS.treeNode;
      const s = 2.2;
      for (const n of v.nodes) g.fillRect(X(n[0]) - s / 2, Y(n[1]) - s / 2, s, s);
    }
    if (v.distField && S.paramValues.dtArrows) {
      const D = v.distField, W = v.W, H = v.H, step = 4;
      const len = Math.min(0.32, (step / RES) * 0.8) * view.s;
      g.strokeStyle = 'rgba(0,0,0,0.55)'; g.lineWidth = 0.8;
      g.beginPath();
      for (let iy = 2; iy < H - 1; iy += step)
        for (let ix = 2; ix < W - 1; ix += step) {
          const c = D[iy * W + ix];
          if (!isFinite(c) || c === 0) continue;
          const l = D[iy * W + ix - 1], rr = D[iy * W + ix + 1], d = D[(iy - 1) * W + ix], u = D[(iy + 1) * W + ix];
          const gx = isFinite(l) && isFinite(rr) ? (rr - l) / 2 : isFinite(rr) ? rr - c : isFinite(l) ? c - l : 0;
          const gy = isFinite(d) && isFinite(u) ? (u - d) / 2 : isFinite(u) ? u - c : isFinite(d) ? c - d : 0;
          const n = Math.hypot(gx, gy);
          if (n < 1e-9) continue;
          const ux = -gx / n, uy = -gy / n;
          const x0 = X((ix + 0.5) / RES), y0 = Y((iy + 0.5) / RES);
          const x1 = x0 + ux * len, y1 = y0 - uy * len;
          g.moveTo(x0, y0); g.lineTo(x1, y1);
          const a = Math.atan2(y1 - y0, x1 - x0), hl = Math.max(2, len * 0.35);
          g.moveTo(x1, y1); g.lineTo(x1 - hl * Math.cos(a - 0.45), y1 - hl * Math.sin(a - 0.45));
          g.moveTo(x1, y1); g.lineTo(x1 - hl * Math.cos(a + 0.45), y1 - hl * Math.sin(a + 0.45));
        }
      g.stroke();
    }
    return (S.overlay = cv);
  }

  function draw() {
    if (!S.raw) return;
    const g = view.begin();
    const r = S.result;
    const bg = r && S.dtImg ? S.dtImg : $('showInflation').checked ? S.imgInfl : S.imgPlain;
    view.image(bg, 0, 0, S.raw.width, S.raw.height);
    if (!S.overlay || S.overlay.width !== view.c.width || S.overlay.height !== view.c.height) buildOverlay();
    g.drawImage(S.overlay, 0, 0, view.cw, view.ch);

    // eingeblendete Vergleichsläufe
    for (const run of S.runs) {
      if (!run.show || run.mapId !== S.mapDef.id || run === S.run) continue;
      const col = ALGO_COLORS[run.algoId];
      view.polyline(run.planned, col, 1.5, [6, 4], 0.9);
      if (run.driven) view.polyline(run.driven, col, 2.2, null, 0.9);
    }

    if (r && r.viz && r.viz.mline) view.polyline(r.viz.mline, COLORS.mline, 1.8, [7, 5]);
    if (r && r.states.length > 1) view.polyline(r.states, r.ok ? COLORS.planned : '#b8860b', 3.6, r.ok ? null : [4, 4], 1, true);
    const sim = S.sim;
    if (sim) {
      view.polyline(sim.path, COLORS.driven, 1.8);
      if (S.running) view.dot(sim.pp.lookPt, '#eda100', 4);
    }
    view.poseMarker(S.start, 'start', '#2b2a28');
    view.poseMarker(S.goal, 'goal', '#2b2a28');
    const tw = parseFloat($('trackW').value) || 0.2;
    view.robot(sim ? sim.pose : S.start, COLORS.robot, tw);
    view.axes();
    updateHud();
    $('legTree').hidden = !(r && r.viz && (r.viz.edges || r.viz.polyEdges || r.viz.explored));
    $('legMline').hidden = !(r && r.viz && r.viz.mline);
    $('legDt').hidden = !(r && r.viz && r.viz.distField);
  }

  function updateHud() {
    const lines = [];
    const sim = S.sim;
    if (sim) {
      const L = sim.log, n = L.t.length - 1;
      lines.push(`t = ${fmt(sim.t, 1)} s`, `v = ${fmt(L.v[n], 2)} m/s`, `ω = ${fmt(L.w[n], 2)} rad/s`, `Modus: ${sim.done ? (sim.goalReached ? 'Ziel erreicht' : 'Zeitlimit') : sim.mode || '–'}`);
    }
    if (S.mouse && S.mouse[0] >= 0 && S.mouse[1] >= 0 && S.mouse[0] <= S.raw.width && S.mouse[1] <= S.raw.height) {
      const free = !S.inflated.occupiedAt(S.mouse[0], S.mouse[1]);
      lines.push(`Maus: (${fmt(S.mouse[0], 2)} | ${fmt(S.mouse[1], 2)}) m${free ? '' : ' – belegt'}`);
    }
    $('planHud').textContent = lines.join('\n');
  }

  /* ------------------------------ Ergebnisanzeige ------------------------------ */
  function statusHtml(run) {
    if (!run) return '–';
    const cls = run.status === 'Ziel erreicht' ? 'ok' : run.status === 'kein Pfad' || run.status === 'Zeitlimit' ? 'fail' : 'pending';
    const col = run.collisions ? ` <span class="fail" title="Kontakt des Roboters mit einem Hindernis">· ${run.collisions}× Kollision</span>` : '';
    return `<span class="${cls}">${run.status}</span>${col}`;
  }

  function statusIcon(run) {
    const col = run.collisions ? ' <span class="fail">⚠</span>' : '';
    if (run.status === 'Ziel erreicht') return `<span class="ok">✓</span>${col}`;
    if (run.status === 'kein Pfad') return '<span class="fail">✗ Pfad</span>';
    if (run.status === 'Zeitlimit') return `<span class="fail">⏱</span>${col}`;
    return `<span class="pending">${run.status === 'fährt' ? '…' : '○'}</span>`;
  }

  function renderCurrent() {
    const run = S.run, r = S.result, sim = S.sim;
    $('rAlgo').textContent = run ? run.algoName : '–';
    $('rLen').textContent = run ? fmt(run.length, 2) : '–';
    $('rTime').textContent = run ? fmt(run.planTime, 1) : '–';
    const live = sim && S.running;
    $('rTravel').textContent = run ? (live ? fmt(sim.t, 1) : fmt(run.travel, 1)) : '–';
    $('rDist').textContent = run ? (live ? fmt(sim.dist, 2) : fmt(run.dist, 2)) : '–';
    $('rStatus').innerHTML = statusHtml(run);
    const ul = $('planInfo');
    ul.innerHTML = '';
    if (r) for (const s of r.info || []) { const li = document.createElement('li'); li.textContent = s; ul.append(li); }
  }

  const METRICS = {
    length: { label: 'Bahnlänge', unit: 'm', d: 2 },
    planTime: { label: 'Planungszeit', unit: 'ms', d: 1 },
    travel: { label: 'Fahrdauer', unit: 's', d: 1 },
    dist: { label: 'Gefahrene Strecke', unit: 'm', d: 2 }
  };

  function filteredRuns() {
    return $('cmpThisMap').checked && S.mapDef ? S.runs.filter((r) => r.mapId === S.mapDef.id) : S.runs.slice();
  }

  function renderComparison() {
    const runs = filteredRuns();
    const metric = $('cmpMetric').value, M = METRICS[metric];
    const bars = $('bars');
    bars.innerHTML = '';
    if (!runs.length) {
      bars.innerHTML = '<p class="empty">Noch keine Ergebnisse. Bahn planen und mit „Start“ abfahren – jeder Lauf wird hier eingetragen.</p>';
    } else {
      const vals = runs.map((r) => r[metric]).filter((v) => isFinite(v));
      const max = vals.length ? Math.max(...vals) : 1;
      const best = vals.length ? Math.min(...vals) : NaN;
      for (const run of runs) {
        const v = run[metric];
        const row = document.createElement('div');
        row.className = 'bar-row' + (isFinite(v) && v === best ? ' best' : '') + (isFinite(v) ? '' : ' na');
        row.innerHTML = `<span class="lbl">#${run.id} ${run.algoName}</span><span class="track"><span class="fill" style="width:${isFinite(v) ? Math.max(0.5, (v / max) * 100) : 0}%"></span></span><span class="val">${isFinite(v) ? fmt(v, M.d) : '–'}</span>`;
        row.addEventListener('mousemove', (e) => showTip(e, tipText(run)));
        row.addEventListener('mouseleave', hideTip);
        bars.append(row);
      }
    }
    // Tabelle
    const tb = $('cmpTable').tBodies[0];
    tb.innerHTML = '';
    const bestOf = (k) => { const v = runs.filter((r) => r.ok).map((r) => r[k]).filter(isFinite); return v.length ? Math.min(...v) : NaN; };
    const bL = bestOf('length'), bP = bestOf('planTime'), bT = runs.filter((r) => r.reached).map((r) => r.travel).reduce((a, b) => Math.min(a, b), Infinity);
    for (const run of runs) {
      const tr = document.createElement('tr');
      if (run === S.run) tr.className = 'current';
      const sameMap = run.mapId === S.mapDef.id;
      tr.innerHTML =
        `<td><input type="checkbox" ${run.show ? 'checked' : ''} ${sameMap && run.ok ? '' : 'disabled'} aria-label="In Karte einblenden"></td>` +
        `<td>${run.id}</td>` +
        `<td><span class="swatch" style="background:${ALGO_COLORS[run.algoId]}"></span>${run.algoName}${$('cmpThisMap').checked ? '' : `<br><small>${run.mapName}</small>`}</td>` +
        `<td class="num ${run.length === bL ? 'best' : ''}">${fmt(run.length, 2)}</td>` +
        `<td class="num ${run.planTime === bP ? 'best' : ''}">${fmt(run.planTime, 1)}</td>` +
        `<td class="num ${run.reached && run.travel === bT ? 'best' : ''}">${fmt(run.travel, 1)}</td>` +
        `<td title="${run.status}${run.collisions ? ` – ${run.collisions}× Kollision` : ''}">${statusIcon(run)}</td>` +
        `<td><button class="del" title="Eintrag löschen" aria-label="Eintrag löschen">✕</button></td>`;
      tr.querySelector('input').addEventListener('change', (e) => { run.show = e.target.checked; draw(); });
      tr.querySelector('.del').addEventListener('click', () => {
        S.runs = S.runs.filter((x) => x !== run);
        if (S.run === run) S.run = null;
        renderComparison(); renderCurrent(); draw();
      });
      tb.append(tr);
    }
  }

  function tipText(run) {
    return `#${run.id} ${run.algoName} – ${run.mapName}\n` +
      `Bahnlänge:     ${fmt(run.length, 2)} m\n` +
      `Planungszeit:  ${fmt(run.planTime, 1)} ms\n` +
      `Fahrdauer:     ${fmt(run.travel, 1)} s\n` +
      `Gef. Strecke:  ${fmt(run.dist, 2)} m\n` +
      `Status:        ${run.status}${run.collisions ? ` (${run.collisions}× Kollision)` : ''}`;
  }

  function showTip(e, text) {
    const t = $('tooltip');
    t.textContent = text;
    t.hidden = false;
    const w = t.offsetWidth, h = t.offsetHeight;
    let x = e.clientX + 14, y = e.clientY + 14;
    if (x + w > window.innerWidth - 8) x = e.clientX - w - 14;
    if (y + h > window.innerHeight - 8) y = e.clientY - h - 14;
    t.style.left = x + 'px'; t.style.top = y + 'px';
  }
  function hideTip() { $('tooltip').hidden = true; }

  function exportCsv() {
    const head = ['Nr', 'Karte', 'Algorithmus', 'Bahnlaenge_m', 'Planungszeit_ms', 'Fahrdauer_s', 'Gefahrene_Strecke_m', 'Status', 'Kollisionen'];
    const rows = S.runs.map((r) => [r.id, r.mapName, r.algoName, r.length, r.planTime, r.travel, r.dist, r.status, r.collisions]
      .map((v) => (typeof v === 'number' ? (isFinite(v) ? v.toFixed(4) : '') : `"${String(v).replace(/"/g, '""')}"`)).join(';'));
    const blob = new Blob(['﻿' + [head.join(';')].concat(rows).join('\r\n')], { type: 'text/csv;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'MobileRobotSim_Vergleich.csv';
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  MRS.PlanningTab = { init, onShow: () => { view.resize(); } };
})();
