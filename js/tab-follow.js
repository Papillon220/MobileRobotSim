/* ==========================================================================
   MobileRobotSim – Reiter 4: Vergleich von Pfadfolgeregelungen
   Regelbasierte Folgeregelung (Stateflow) und Pure Pursuit fahren dieselben
   Wegpunkte gleichzeitig ab; Kennzahlen und Verläufe werden gegenübergestellt.
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { fmt } = MRS;
  const $ = (id) => document.getElementById(id);
  const DT = 0.001; // Integrationsschritt des Fahrzeugmodells [s]
  const C_RULE = '#eb6834', C_PP = '#2a78d6', C_REF = '#8a8984', C_WP = '#6250d6';

  const S = {
    active: false, running: false, lastTs: 0, acc: 0,
    def: null, world: null, img: null, worlds: {}, pre: {},
    wpMode: 'pre', manual: [], wps: [],
    th0: null, drag: false, suppressClick: false, // manuelle Startausrichtung (null = automatisch)
    sim: null, recorded: false, hist: [], runId: 1
  };
  let view, plots;

  /* ------------------------------ Initialisierung ------------------------------ */
  function init() {
    for (const m of MRS.SLAM_MAPS) $('fwMap').add(new Option(m.name, m.id));
    view = new MRS.WorldView($('fwCanvas'));
    view.onResize = draw;
    const series = [{ label: 'Regelbasiert', color: C_RULE }, { label: 'Pure Pursuit', color: C_PP }];
    plots = {
      ct: new MRS.TimePlot($('fwPlotCt'), { title: 'Querabweichung', unit: 'cm', series }),
      v: new MRS.TimePlot($('fwPlotV'), { title: 'Lineargeschwindigkeit v', unit: 'm/s', series }),
      w: new MRS.TimePlot($('fwPlotW'), { title: 'Drehrate ω', unit: 'rad/s', series })
    };
    for (const p of Object.values(plots)) p.window = 0;

    $('fwMap').addEventListener('change', loadMap);
    document.querySelectorAll('#fwWpMode button').forEach((b) => b.addEventListener('click', () => setWpMode(b.dataset.mode)));
    $('fwUndo').addEventListener('click', () => { S.manual.pop(); if (!S.manual.length) S.th0 = null; waypointsChanged(); });
    $('fwClear').addEventListener('click', () => { S.manual = []; S.th0 = null; waypointsChanged(); });
    $('fwTh0Auto').addEventListener('click', () => setTh0(null));
    $('fwCopyPre').addEventListener('click', () => { S.manual = predefined().map((p) => p.slice()); waypointsChanged(); });
    for (const id of ['fwLoop', 'fwDense']) $(id).addEventListener('change', waypointsChanged);
    $('fwCanvas').addEventListener('click', onCanvasClick);
    $('fwCanvas').addEventListener('pointerdown', onPointerDown);
    $('fwCanvas').addEventListener('pointermove', onPointerMove);
    for (const ev of ['pointerup', 'pointercancel']) $('fwCanvas').addEventListener(ev, onPointerUp);
    $('fwCanvas').addEventListener('wheel', onWheel, { passive: false });
    $('fwStart').addEventListener('click', onStart);
    $('fwReset').addEventListener('click', reset);
    $('fwHistClear').addEventListener('click', () => { S.hist = []; renderHistory(); });
    $('fwShowHelpers').addEventListener('change', draw);
    $('fwWin').addEventListener('change', () => { for (const p of Object.values(plots)) p.window = parseFloat($('fwWin').value); drawPlots(); });
    for (const id of ['fwUseRule', 'fwUsePP', 'fwRuleV', 'fwRuleK', 'fwRuleTol', 'fwRuleWg', 'fwPPV', 'fwPPL', 'fwPPW', 'fwPPTol', 'fwTs', 'fwTmax', 'fwWlim', 'fwRobot'])
      $(id).addEventListener('change', () => { if (!S.running) reset(); });

    loadMap();
    requestAnimationFrame(loop);
  }

  /* ------------------------------- Wegpunkte ---------------------------------- */
  function loadMap() {
    S.running = false;
    S.def = MRS.SLAM_MAPS.find((m) => m.id === $('fwMap').value);
    $('fwMapDesc').textContent = S.def.desc;
    if (!S.worlds[S.def.id]) {
      const w = S.def.build(S.def.res);
      S.worlds[S.def.id] = { world: w, img: MRS.renderMapImage(w, w.inflate(0.1)) };
    }
    S.world = S.worlds[S.def.id].world;
    S.img = S.worlds[S.def.id].img;
    S.manual = [];
    S.th0 = null;
    view.setWorld(0, 0, S.world.width, S.world.height);
    waypointsChanged();
  }

  /** Vordefinierte Wegpunkte: Demo-Route bzw. Eckpunkte eines geglätteten A*-Pfades Start → Ziel. */
  function predefined() {
    const id = S.def.id;
    if (S.pre[id]) return S.pre[id];
    let pts;
    if (S.def.route) pts = S.def.route.map((p) => p.slice());
    else {
      const m = MRS.MAPS.find((x) => x.id === id);
      // möglichst großer Wandabstand; bei engen Durchgängen schrittweise verringern
      pts = null;
      for (const infl of [0.35, 0.25, 0.15]) {
        const r = MRS.runPlanner('astar', S.world, S.world.inflate(infl), m.start, m.goal, { astarWeight: 1, astarSmooth: true, seed: 1 });
        if (r.ok) { pts = r.states.map((q) => [q[0], q[1]]); break; }
      }
      if (!pts) pts = [[m.start[0], m.start[1]], [m.goal[0], m.goal[1]]];
    }
    return (S.pre[id] = pts);
  }

  function setWpMode(m) {
    S.wpMode = m;
    document.querySelectorAll('#fwWpMode button').forEach((b) => b.classList.toggle('active', b.dataset.mode === m));
    $('tab-follow').classList.toggle('wp-man', m === 'manual');
    $('tab-follow').classList.toggle('wp-pre', m === 'pre');
    waypointsChanged();
  }

  function waypointsChanged() {
    const base = S.wpMode === 'pre' ? predefined() : S.manual;
    // Rundkurs: Rückfahrt zum ersten Wegpunkt anhängen (sofern der letzte nicht bereits dort liegt)
    const route = base.slice();
    if ($('fwLoop').checked && base.length >= 2) {
      const a = base[0], b = base[base.length - 1];
      if (Math.hypot(a[0] - b[0], a[1] - b[1]) > 1e-6) route.push(a);
    }
    S.wps = $('fwDense').checked && route.length >= 2 ? Array.from(MRS.connectWaypoints(S.world, route, true)) : route.map((p) => p.slice());
    const extra = S.wps.length - route.length;
    $('fwCount').textContent = base.length + (extra > 0 ? ` (+${extra} Zwischenpunkte)` : '');
    updateTh0Info();
    reset();
  }

  function canvasPos(e) {
    const r = $('fwCanvas').getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  /* --------------------------- Manuelle Startausrichtung --------------------------- */
  const ARROW_PX = 36;   // Länge des Ausrichtungspfeils [px]
  const WHEEL_PX = 48;   // Mausrad wirkt innerhalb dieses Abstands um den Startpunkt [px]

  /** Startausrichtung editierbar? (manueller Modus, Startpunkt gesetzt, Fahrt noch nicht begonnen) */
  function th0Editable() {
    return S.wpMode === 'manual' && S.manual.length >= 1 && !S.running && !(S.sim && S.sim.t > 0);
  }

  /** Aktuelle Startausrichtung: manuell gesetzt oder Richtung zum zweiten Punkt der Bahn. */
  function startHeading() {
    if (S.wpMode === 'manual' && S.th0 != null) return S.th0;
    const w = S.wps;
    return w.length > 1 ? Math.atan2(w[1][1] - w[0][1], w[1][0] - w[0][0]) : 0;
  }

  /** Bildschirmposition von Startpunkt und Pfeilspitze. */
  function arrowGeom() {
    const p = S.manual[0], th = startHeading();
    const x = view.X(p[0]), y = view.Y(p[1]);
    return { x, y, hx: x + ARROW_PX * Math.cos(th), hy: y - ARROW_PX * Math.sin(th) };
  }

  function setTh0(th) {
    S.th0 = th == null ? null : MRS.wrapAngle(th);
    updateTh0Info();
    reset();
  }

  function updateTh0Info() {
    const d = Math.round(MRS.deg(startHeading()));
    $('fwTh0Info').textContent = `Startausrichtung: ${d}°` + (S.th0 == null ? ' (automatisch)' : '');
    $('fwTh0Auto').disabled = S.th0 == null;
  }

  function onHandle(px, py) {
    if (!th0Editable()) return false;
    const a = arrowGeom();
    return Math.hypot(px - a.hx, py - a.hy) <= 12;
  }

  function onPointerDown(e) {
    const [px, py] = canvasPos(e);
    if (!onHandle(px, py)) return;
    e.preventDefault();
    S.drag = true;
    $('fwCanvas').setPointerCapture(e.pointerId);
    $('fwCanvas').style.cursor = 'grabbing';
  }

  function onPointerMove(e) {
    const [px, py] = canvasPos(e);
    if (S.drag) {
      const a = arrowGeom();
      if (Math.hypot(px - a.x, py - a.y) > 4) setTh0(Math.atan2(a.y - py, px - a.x));
      return;
    }
    $('fwCanvas').style.cursor = onHandle(px, py) ? 'grab' : '';
  }

  function onPointerUp(e) {
    if (!S.drag) return;
    S.drag = false;
    S.suppressClick = true; // abschließenden Klick nicht als neuen Wegpunkt werten
    setTimeout(() => { S.suppressClick = false; }, 0);
    $('fwCanvas').releasePointerCapture(e.pointerId);
    const [px, py] = canvasPos(e);
    $('fwCanvas').style.cursor = onHandle(px, py) ? 'grab' : '';
  }

  function onWheel(e) {
    if (!th0Editable()) return;
    const [px, py] = canvasPos(e);
    const a = arrowGeom();
    if (Math.hypot(px - a.x, py - a.y) > WHEEL_PX) return; // sonst normal scrollen
    e.preventDefault();
    const step = MRS.rad(e.shiftKey ? 1 : 5);
    // auf ganze Schritte runden, Rad nach oben = gegen den Uhrzeigersinn
    const th = Math.round(startHeading() / step) * step;
    setTh0(th + (e.deltaY < 0 ? step : -step));
  }

  function onCanvasClick(e) {
    if (S.suppressClick) { S.suppressClick = false; return; }
    if (S.wpMode !== 'manual' || S.running) return;
    const [x, y] = view.toWorld(...canvasPos(e));
    if (x < 0 || y < 0 || x > S.world.width || y > S.world.height) return;
    S.manual.push([x, y]);
    waypointsChanged();
  }

  /* -------------------------------- Simulation -------------------------------- */
  function cfgFromUI() {
    const v = (id) => parseFloat($(id).value);
    return {
      ts: Math.max(0.001, v('fwTs') || 0.05), maxTime: v('fwTmax') || 300, wLimit: Math.max(0, v('fwWlim') || 0),
      robotR: (v('fwRobot') || 0.2) / 2,
      th0: S.wpMode === 'manual' ? S.th0 : null,
      useRule: $('fwUseRule').checked, usePP: $('fwUsePP').checked,
      rule: { v: v('fwRuleV'), k: v('fwRuleK'), tol: v('fwRuleTol'), wGoal: v('fwRuleWg') },
      pp: { v: v('fwPPV'), lookahead: v('fwPPL'), wMax: v('fwPPW'), tol: v('fwPPTol') }
    };
  }

  function reset() {
    S.running = false;
    S.recorded = false;
    S.acc = 0;
    const cfg = cfgFromUI();
    S.sim = S.wps.length >= 2 && (cfg.useRule || cfg.usePP) ? new MRS.FollowSim(S.world, S.wps, cfg) : null;
    updateStartBtn();
    draw(); drawPlots(); renderCompare();
  }

  function onStart() {
    if (S.running) { S.running = false; updateStartBtn(); return; }
    if (!S.sim || S.sim.done) reset();
    if (!S.sim) { $('fwHud').textContent = 'Bitte mindestens zwei Wegpunkte festlegen\nund einen Regler aktivieren.'; return; }
    S.running = true;
    updateStartBtn();
  }

  function updateStartBtn() {
    $('fwStart').textContent = S.running ? '❚❚ Pause' : S.sim && S.sim.t > 0 && !S.sim.done ? '▶ Weiter' : '▶ Start';
    $('fwStart').classList.toggle('go', !S.running);
  }

  function loop(ts) {
    requestAnimationFrame(loop);
    const real = Math.min(0.05, (ts - (S.lastTs || ts)) / 1000);
    S.lastTs = ts;
    if (!S.active || !S.running || !S.sim) return;
    const speed = parseFloat($('fwSpeed').value);
    const t0 = performance.now();
    if (speed === 0) {
      while (!S.sim.done && performance.now() - t0 < 40) S.sim.step(DT);
    } else {
      S.acc = Math.min(S.acc + real * speed, 1);
      while (S.acc >= DT && !S.sim.done) {
        S.sim.step(DT);
        S.acc -= DT;
        if (performance.now() - t0 > 40) { S.acc = 0; break; }
      }
    }
    if (S.sim.done) { S.running = false; recordRun(); updateStartBtn(); }
    draw(); drawPlots(); renderCompare();
  }

  /* --------------------------------- Zeichnen --------------------------------- */
  function draw() {
    if (!view || !S.world) return;
    const g = view.begin();
    view.image(S.img, 0, 0, S.world.width, S.world.height);
    view.polyline(S.wps, C_REF, 1.6, [7, 5]);
    const sim = S.sim;
    if (sim) for (const a of sim.agents) view.polyline(a.path, a.color, 2.2, null, 0.95);
    // Wegpunkte
    g.save();
    g.font = '600 11px system-ui, -apple-system, "Segoe UI", sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    const base = S.wpMode === 'pre' ? predefined() : S.manual;
    base.forEach((p, i) => {
      const x = view.X(p[0]), y = view.Y(p[1]);
      g.fillStyle = S.world.occupiedAt(p[0], p[1]) ? '#b42323' : C_WP;
      g.strokeStyle = '#fff'; g.lineWidth = 2;
      g.beginPath(); g.arc(x, y, 9, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.fillStyle = '#fff';
      g.fillText(i === 0 ? 'S' : String(i), x, y + 0.5);
    });
    g.restore();
    if (sim) {
      const tw = parseFloat($('fwRobot').value) || 0.2;
      for (const a of sim.agents) {
        if ($('fwShowHelpers').checked && !a.ctrl.done) {
          if (a.id === 'rule') {
            const gp = a.ctrl.goal;
            g.save(); g.strokeStyle = C_RULE; g.lineWidth = 2.5;
            g.beginPath(); g.arc(view.X(gp[0]), view.Y(gp[1]), 13, 0, 2 * Math.PI); g.stroke(); g.restore();
          } else view.dot(a.ctrl.pp.lookPt, C_PP, 4);
        }
        view.robot(a.pose, a.color, tw, 8);
      }
    } else if (S.wps.length) view.poseMarker([S.wps[0][0], S.wps[0][1], startHeading()], 'start', '#2b2a28');
    if (th0Editable()) {
      // Ausrichtungspfeil mit Griff an der Spitze
      const a = arrowGeom(), th = startHeading();
      g.save();
      g.strokeStyle = C_WP; g.fillStyle = C_WP; g.lineWidth = 2.5;
      g.beginPath(); g.moveTo(a.x + 10 * Math.cos(th), a.y - 10 * Math.sin(th)); g.lineTo(a.hx, a.hy); g.stroke();
      g.beginPath(); g.arc(a.hx, a.hy, 6, 0, 2 * Math.PI);
      g.fillStyle = S.drag ? C_WP : '#fff'; g.fill(); g.stroke();
      g.restore();
    }
    view.axes();
    const hud = [];
    if (sim) {
      hud.push(`t = ${fmt(sim.t, 1)} s`);
      for (const a of sim.agents) hud.push(`${a.name}: ${a.ctrl.info()}`);
    } else hud.push(S.wpMode === 'manual' ? 'Wegpunkte per Klick setzen' : 'Keine Wegpunkte');
    $('fwHud').textContent = hud.join('\n');
  }

  function agentById(id) { return S.sim && S.sim.agents.find((a) => a.id === id); }

  function drawPlots() {
    const r = agentById('rule'), p = agentById('pp');
    const ref = r || p;
    const t = ref ? ref.log.t : [];
    const arr = (a, k, f = 1) => (a ? a.log[k].map((x) => x * f) : null);
    const vis = [!!r, !!p];
    plots.ct.draw(t, [arr(r, 'ct', 100), arr(p, 'ct', 100)], vis);
    plots.v.draw(t, [arr(r, 'v'), arr(p, 'v')], vis);
    plots.w.draw(t, [arr(r, 'w'), arr(p, 'w')], vis);
  }

  /* -------------------------------- Kennzahlen -------------------------------- */
  function metrics(a) {
    if (!a) return null;
    const last = S.wps[S.wps.length - 1];
    return {
      status: a.ctrl.done ? 'Ziel erreicht' : S.sim.done ? 'Zeitlimit' : 'fährt',
      time: a.tDone,
      dist: a.dist,
      ctMean: a.ctN ? (a.ctSum / a.ctN) * 100 : NaN,
      ctMax: a.ctMax * 100,
      wMax: a.wMaxAbs,
      contacts: a.contacts,
      endErr: Math.hypot(a.pose[0] - last[0], a.pose[1] - last[1]) * 100
    };
  }

  const ROWS = [
    ['status', 'Status', null],
    ['time', 'Fahrzeit bis Ziel [s]', 1],
    ['dist', 'Gefahrene Strecke [m]', 2],
    ['ctMean', '⌀ Querabweichung [cm]', 1],
    ['ctMax', 'max. Querabweichung [cm]', 1],
    ['wMax', 'max. |ω| [rad/s]', 2],
    ['contacts', 'Wandkontakte', 0],
    ['endErr', 'Abstand zum Ziel [cm]', 1]
  ];

  function renderCompare() {
    const mr = metrics(agentById('rule')), mp = metrics(agentById('pp'));
    const tb = $('fwCmp').tBodies[0];
    tb.innerHTML = '';
    for (const [k, label, d] of ROWS) {
      const a = mr ? mr[k] : null, b = mp ? mp[k] : null;
      const tr = document.createElement('tr');
      const cell = (v, other) => {
        if (v == null) return '<td class="num">–</td>';
        if (d === null) return `<td class="num"><span class="${v === 'Ziel erreicht' ? 'ok' : v === 'Zeitlimit' ? 'fail' : 'pending'}">${v}</span></td>`;
        const best = isFinite(v) && isFinite(other) && v < other - 1e-9 && mr && mp && mr.status !== 'fährt' && mp.status !== 'fährt';
        return `<td class="num${best ? ' best' : ''}">${isFinite(v) ? fmt(v, d) : '–'}</td>`;
      };
      tr.innerHTML = `<td>${label}</td>${cell(a, b)}${cell(b, a)}`;
      tb.append(tr);
    }
  }

  function recordRun() {
    if (S.recorded || !S.sim) return;
    S.recorded = true;
    const id = S.runId++;
    for (const a of S.sim.agents) {
      const m = metrics(a);
      S.hist.push({ id, name: a.name, color: a.color, map: S.def.name, ...m });
    }
    renderHistory();
  }

  function renderHistory() {
    const tb = $('fwHist').tBodies[0];
    tb.innerHTML = '';
    for (const h of S.hist.slice().reverse()) {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${h.id}</td><td><span class="dot-sw" style="--c:${h.color}"></span>${h.name}</td><td>${h.map}</td>` +
        `<td class="num">${isFinite(h.time) ? fmt(h.time, 1) : '–'}</td><td class="num">${fmt(h.ctMean, 1)}</td><td class="num">${fmt(h.ctMax, 1)}</td><td class="num">${h.contacts}</td>`;
      tb.append(tr);
    }
  }

  MRS.FollowTab = {
    init,
    setActive(a) {
      S.active = a;
      if (a) { view.resize(); for (const p of Object.values(plots)) p._resize(); draw(); drawPlots(); }
      else if (S.running) { S.running = false; updateStartBtn(); }
    }
  };
})();
