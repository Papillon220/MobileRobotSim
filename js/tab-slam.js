/* ==========================================================================
   MobileRobotSim – Reiter 3: Lidar-SLAM
   Wahre Umgebung mit Lidar links, dynamisch entstehende SLAM-Karte rechts,
   Schätzfehler-Verläufe darunter. Fahrmodi: manuell, strukturiert, zufällig, Wegpunkte.
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { fmt, deg, COLORS } = MRS;
  const $ = (id) => document.getElementById(id);
  const DT = 0.01;               // Simulationsschritt [s]
  const C_TRUE = COLORS.driven, C_EST = '#2a78d6', C_ODO = COLORS.faulty, C_ROUTE = '#9085e9', C_SCAN = '#d55181';
  const MODE_HINTS = {
    manual: 'Pfeiltasten bzw. W/A/S/D (oder Tasten unten) gedrückt halten: ▲▼ bzw. W/S fahren, ◀▶ bzw. A/D drehen.',
    route: 'Abfahren einer Erkundungsroute (Demo-Szene: Rundfahrt entlang der Außenwände; Bürogebäude mit Atrium: Rundgang um das Atrium; sonst automatisch erzeugte Rundfahrt über ein Raster freier Punkte). Die wahre Bahn ist vorgegeben und unabhängig von der Schätzung; so lässt sich die Güte der Schätzung bei identischer Fahrt vergleichen.',
    random: 'Geradeausfahrt; bei Hindernis vor dem Roboter Drehung um einen zufälligen Winkel in zufällige Richtung.',
    waypoints: 'Klick in eine der beiden Karten setzt einen Wegpunkt. „Start“ fährt die Wegpunkte ab der aktuellen Pose der Reihe nach ab.'
  };

  const S = {
    active: false, running: false, lastTs: 0,
    def: null, world: null, worldImg: null, routes: {}, worlds: {},
    sim: null, driver: null, mode: 'manual',
    keys: new Set(), v: 0, w: 0,
    mapImg: null, mapImgVersion: -1, plotAcc: 0,
    wps: [], wpDone: 0
  };
  let vWorld, vMap, plots;

  /* ------------------------------ Initialisierung ------------------------------ */
  function init() {
    for (const m of MRS.SLAM_MAPS) $('slMap').add(new Option(m.name, m.id));
    vWorld = new MRS.WorldView($('slWorld'), { margin: { l: 38, r: 10, t: 10, b: 28 } });
    vMap = new MRS.WorldView($('slMapCv'), { margin: { l: 38, r: 10, t: 10, b: 28 } });
    vWorld.onResize = draw;
    vMap.onResize = draw;
    const mk = (id, title, unit, series) => new MRS.TimePlot($(id), { title, unit, series });
    plots = {
      e: mk('slPlotE', 'Positionsfehler', 'm', [{ label: 'Schätzung', color: C_EST }, { label: 'Odometrie', color: C_ODO, dash: [6, 4] }]),
      h: mk('slPlotH', 'Orientierungsfehler', '°', [{ label: 'Schätzung', color: C_EST }, { label: 'Odometrie', color: C_ODO, dash: [6, 4] }]),
      r: mk('slPlotR', 'Scan-Matching-Residuum (RMS)', 'mm', [{ label: 'Residuum', color: '#6250d6' }])
    };
    for (const p of Object.values(plots)) p.window = 60;

    $('slMap').addEventListener('change', reset);
    $('slMethod').addEventListener('change', reset);
    $('slRes').addEventListener('change', reset);
    $('slReset').addEventListener('click', reset);
    $('slDemo').addEventListener('click', startDemo);
    $('slStart').addEventListener('click', onStart);
    $('slSpacing').addEventListener('change', () => { S.routes = {}; if (S.mode === 'route') reset(); });
    $('slWin').addEventListener('change', () => { for (const p of Object.values(plots)) p.window = parseFloat($('slWin').value); drawPlots(); });
    $('slShowTrue').addEventListener('change', draw);
    $('slMatlabLidar').addEventListener('click', () => {
      Object.entries({ slFov: 90, slBeams: 180, slRange: 5.5, slRate: 25, slModel: 'ideal' }).forEach(([id, v]) => { $(id).value = v; });
      applyLiveCfg();
    });
    $('slShowOdo').addEventListener('change', () => { $('slLegOdo').hidden = !$('slShowOdo').checked; draw(); });
    $('slShowBeams').addEventListener('change', draw);
    document.querySelectorAll('#slMode button').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
    for (const id of ['slRate', 'slRange', 'slFov', 'slBeams', 'slNoise', 'slSigV', 'slSigW', 'slBias', 'slWinT', 'slWinR', 'slModel', 'slDeskew'])
      $(id).addEventListener('change', applyLiveCfg);
    updateLidarInfo();
    for (const id of ['slV', 'slW', 'slSafe']) $(id).addEventListener('change', makeDriver);
    // Wegpunkte
    for (const cv of [$('slWorld'), $('slMapCv')]) cv.addEventListener('click', onCanvasClick);
    $('slWpUndo').addEventListener('click', () => { S.wps.pop(); waypointsChanged(); });
    $('slWpClear').addEventListener('click', () => { S.wps = []; waypointsChanged(); });
    for (const id of ['slWpLoop', 'slWpPlan', 'slWpEst']) $(id).addEventListener('change', waypointsChanged);

    window.addEventListener('keydown', onKey);
    window.addEventListener('keyup', (e) => { if (S.keys.delete(MRS.driveKey(e))) updateKeypad(); });
    window.addEventListener('blur', () => { S.keys.clear(); updateKeypad(); });
    document.querySelectorAll('#slKeypad button').forEach((b) => {
      const k = b.dataset.key;
      const up = (e) => { e.preventDefault(); if (S.keys.delete(k)) updateKeypad(); };
      b.addEventListener('pointerdown', (e) => { e.preventDefault(); pressKey(k); });
      b.addEventListener('pointerup', up);
      b.addEventListener('pointerleave', up);
      b.addEventListener('pointercancel', up);
    });

    setMode('manual', true);
    reset();
    requestAnimationFrame(loop);
  }

  /* ---------------------------------- Setup ------------------------------------ */
  function cfgFromUI() {
    const v = (id) => parseFloat($(id).value);
    const resSel = $('slRes').value;
    return {
      lidar: { rate: v('slRate') || 25, range: v('slRange') || 5.5, fov: v('slFov') || 90, beams: Math.max(2, Math.round(v('slBeams') || 180)), noise: Math.max(0, v('slNoise') || 0), model: $('slModel').value, deskew: $('slDeskew').checked },
      odo: { sigV: Math.max(0, v('slSigV') || 0), sigW: Math.max(0, v('slSigW') || 0), biasW: v('slBias') || 0 },
      method: $('slMethod').value,
      mapRes: resSel === 'auto' ? S.def.res : parseInt(resSel, 10),
      win: { trans: v('slWinT') || 0.1, rot: ((v('slWinR') || 5) * Math.PI) / 180 },
      robotR: 0.1
    };
  }

  /** Abgeleitete Kenngrößen des rotierenden Lidars anzeigen. */
  function updateLidarInfo() {
    const v = (id) => parseFloat($(id).value);
    const rate = v('slRate') || 25, fov = Math.min(360, v('slFov') || 90), n = Math.max(2, Math.round(v('slBeams') || 180));
    const res = fov >= 360 ? fov / n : fov / (n - 1);
    const sectorMs = (1000 / rate) * (fov / 360);
    // Messrate des Lasers: n Messungen in der Zeit, die der Strahl für den Sektor benötigt
    const pulseRate = n / (sectorMs / 1000);
    $('slLidarInfo').textContent = `Winkelauflösung ${fmt(res, 2)}° · Umdrehung ${fmt(1000 / rate, 0)} ms · Messrate ${fmt(pulseRate / 1000, 1)} kHz`;
    $('slDeskew').disabled = $('slModel').value === 'ideal';
  }

  function applyLiveCfg() {
    updateLidarInfo();
    if (!S.sim) return;
    const c = cfgFromUI();
    S.sim.cfg.lidar = c.lidar;
    S.sim.cfg.odo = c.odo;
    S.sim.cfg.win = c.win;
    draw();
  }

  function reset() {
    setRunning(false);
    S.def = MRS.SLAM_MAPS.find((m) => m.id === $('slMap').value);
    $('slMapDesc').textContent = S.def.desc;
    if (!S.worlds[S.def.id]) {
      const w = S.def.build(S.def.res);
      S.worlds[S.def.id] = { world: w, img: MRS.renderMapImage(w, null) };
    }
    S.world = S.worlds[S.def.id].world;
    S.worldImg = S.worlds[S.def.id].img;
    S.sim = new MRS.SlamSim(S.world, S.def.start, cfgFromUI(), 7);
    S.v = S.w = 0;
    S.mapImg = null; S.mapImgVersion = -1;
    vWorld.setWorld(0, 0, S.world.width, S.world.height);
    vMap.setWorld(0, 0, S.world.width, S.world.height);
    makeDriver();
    draw(); drawPlots(); updateReadouts();
  }

  function getRoute() {
    const key = S.def.id;
    if (!S.routes[key]) S.routes[key] = S.def.route || MRS.buildCoverageRoute(S.world, S.def.start, parseFloat($('slSpacing').value) || 2.5);
    return S.routes[key];
  }

  function makeDriver() {
    const v = parseFloat($('slV').value) || 0.4, w = parseFloat($('slW').value) || 1;
    if (S.mode === 'route') {
      if (!S.routes[S.def.id] && !S.def.route) {
        $('slBusy').hidden = false;
        setTimeout(() => { getRoute(); $('slBusy').hidden = true; makeDriver(); draw(); }, 30);
        S.driver = null;
        return;
      }
      S.driver = new MRS.RouteDriver(getRoute(), v, w);
    } else if (S.mode === 'waypoints') {
      S.driver = null;
      const rest = S.wps.slice(S.wpDone);
      if (!rest.length) return;
      const start = $('slWpEst').checked ? S.sim.estPose : S.sim.truePose;
      const pts = [start].concat(rest);
      if ($('slWpLoop').checked && S.wps.length > 1) pts.push(S.wps[0]);
      const path = MRS.connectWaypoints(S.world, pts, $('slWpPlan').checked);
      if (path.length < 2) return;
      S.driver = new MRS.RouteDriver(path, v, w, $('slWpEst').checked);
      S.driver.failed = path.failed;
      S.driver.wpMap = path.wpIdx.slice(1, 1 + rest.length);
      S.driver.wpStart = S.wpDone;
    } else if (S.mode === 'random') {
      S.driver = new MRS.RandomDriver(v, w, parseFloat($('slSafe').value) || 0.5, MRS.makeRng(Date.now() & 0xffff));
    } else S.driver = null;
  }

  function setMode(m, silent) {
    const prev = S.mode;
    S.mode = m;
    const tab = $('tab-slam');
    tab.classList.remove('m-manual', 'm-route', 'm-random', 'm-waypoints');
    tab.classList.add('m-' + m);
    document.querySelectorAll('#slMode button').forEach((b) => b.classList.toggle('active', b.dataset.mode === m));
    $('slModeHint').textContent = MODE_HINTS[m];
    $('slLegRoute').hidden = m !== 'route' && m !== 'waypoints';
    if (silent) return;
    // Die Route beginnt an der Startpose → Wechsel in den strukturierten Modus startet neu
    if (m === 'route' && prev !== 'route') reset();
    else { makeDriver(); draw(); }
  }

  function onStart() {
    if (S.running) { setRunning(false); return; }
    // Abgeschlossene Wegpunktfahrt: neu ab der aktuellen Pose planen
    if (S.mode === 'waypoints' && (!S.driver || S.driver.done)) { S.wpDone = 0; makeDriver(); }
    if (S.mode === 'waypoints' && !S.driver) { $('slModeHint').textContent = 'Bitte zuerst Wegpunkte per Klick in eine der Karten setzen.'; return; }
    if (S.mode === 'route' && S.driver && S.driver.done) return;
    setRunning(true);
  }

  /* -------------------------------- Wegpunkte ---------------------------------- */
  function onCanvasClick(e) {
    if (S.mode !== 'waypoints') return;
    const view = e.currentTarget.id === 'slWorld' ? vWorld : vMap;
    const r = e.currentTarget.getBoundingClientRect();
    const [x, y] = view.toWorld(e.clientX - r.left, e.clientY - r.top);
    if (x < 0 || y < 0 || x > S.world.width || y > S.world.height) return;
    S.wps.push([x, y]);
    waypointsChanged();
  }

  /** Anzahl der bereits erreichten Wegpunkte aus dem Fortschritt der laufenden Fahrt. */
  function updateWpDone() {
    const d = S.driver;
    if (S.mode !== 'waypoints' || !d || !d.wpMap) return;
    const reached = d.done ? d.wpMap.length : d.wpMap.filter((i) => d.pp.projIdx >= i).length;
    S.wpDone = Math.min(S.wps.length, d.wpStart + reached);
  }

  function waypointsChanged() {
    $('slWpCount').textContent = S.wps.length;
    // Während der Fahrt: ab aktueller Pose mit den noch offenen Wegpunkten weiterfahren
    if (S.running) updateWpDone(); else S.wpDone = 0;
    S.wpDone = Math.min(S.wpDone, S.wps.length);
    if (S.mode === 'waypoints') makeDriver();
    if (S.mode === 'waypoints' && !S.driver && S.running) setRunning(false);
    draw();
  }

  function drawWaypoints(view) {
    if (S.mode !== 'waypoints') return;
    if (S.driver && S.driver.route) view.polyline(S.driver.route, C_ROUTE, 1.6, [6, 5], 0.95);
    const g = view.ctx;
    g.save();
    g.font = '600 11px system-ui, -apple-system, "Segoe UI", sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    S.wps.forEach((p, i) => {
      const x = view.X(p[0]), y = view.Y(p[1]);
      const blocked = S.world.occupiedAt(p[0], p[1]);
      const reached = S.running || (S.driver && S.driver.done) ? i < S.wpDone : false;
      g.fillStyle = blocked ? '#b42323' : reached ? '#8a8984' : '#6250d6';
      g.strokeStyle = '#ffffff'; g.lineWidth = 2;
      g.beginPath(); g.arc(x, y, 9, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.fillStyle = '#ffffff';
      g.fillText(String(i + 1), x, y + 0.5);
    });
    g.restore();
  }

  function setRunning(r) {
    S.running = r;
    $('slStart').textContent = r ? '❚❚ Pause' : '▶ Start';
    $('slStart').classList.toggle('go', !r);
  }

  /* --------------------------------- Tastatur ---------------------------------- */
  const KEYS = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
  function onKey(e) {
    if (S.demo) return; // Demo: keine manuelle Steuerung
    const k = MRS.driveKey(e);
    if (!S.active || !KEYS.includes(k)) return;
    const t = e.target;
    if (t && (t.tagName === 'SELECT' || (t.tagName === 'INPUT' && t.type !== 'range' && t.type !== 'checkbox'))) return;
    e.preventDefault();
    if (e.repeat) return;
    pressKey(k);
  }
  function pressKey(k) {
    if (S.mode !== 'manual') setMode('manual');
    S.keys.add(k);
    if (!S.running) setRunning(true);
    updateKeypad();
  }
  function updateKeypad() {
    document.querySelectorAll('#slKeypad button').forEach((b) => b.classList.toggle('down', S.keys.has(b.dataset.key)));
  }

  /* ------------------------------- Simulation --------------------------------- */
  function manualCommand() {
    const vmax = parseFloat($('slV').value) || 0.4, wmax = parseFloat($('slW').value) || 1;
    const vT = (S.keys.has('ArrowUp') ? vmax : 0) - (S.keys.has('ArrowDown') ? vmax : 0);
    const wT = (S.keys.has('ArrowLeft') ? wmax : 0) - (S.keys.has('ArrowRight') ? wmax : 0);
    const ramp = (c, t, r) => (t > c ? Math.min(t, c + r * DT) : Math.max(t, c - r * DT));
    S.v = ramp(S.v, vT, 1.5);
    S.w = ramp(S.w, wT, 5);
    return [S.v, S.w];
  }

  function loop(ts) {
    requestAnimationFrame(loop);
    const real = Math.min(0.05, (ts - (S.lastTs || ts)) / 1000);
    S.lastTs = ts;
    if (!S.active || !S.sim) return;
    // Demo: nach kurzer Pause am Rundenende neu beginnen
    if (S.demo && S.demo.restartAt && ts >= S.demo.restartAt) { S.demo.restartAt = 0; demoRun(); }
    if (S.running) {
      const speed = parseFloat($('slSpeed').value) || 1;
      const t0 = performance.now();
      // Zeitakkumulator: auch bei Zeitlupe (< 1×) wird die Simulationszeit exakt fortgeschrieben
      S.acc = Math.min((S.acc || 0) + real * speed, 0.5);
      while (S.acc >= DT) {
        S.acc -= DT;
        let cmd;
        if (S.mode === 'manual') cmd = manualCommand();
        else if (S.driver) cmd = S.driver.command(S.sim, DT);
        else cmd = [0, 0];
        S.sim.step(DT, cmd[0], cmd[1]);
        if (performance.now() - t0 > 35) { S.acc = 0; break; } // Rechenzeitbudget je Frame
      }
      updateWpDone();
      if (S.driver && S.driver.done) {
        setRunning(false);
        if (S.demo) S.demo.restartAt = ts + DEMO_PAUSE_MS;
      }
      draw();
      updateReadouts();
      S.plotAcc += real;
      if (S.plotAcc > 0.1) { S.plotAcc = 0; drawPlots(); }
    }
  }

  /* --------------------------------- Zeichnen ---------------------------------- */
  function updateMapImage() {
    const g = S.sim.grid;
    if (S.mapImg && S.mapImgVersion === g.version) return;
    const W = g.W, H = g.H;
    // nur den seit der letzten Darstellung geänderten Bereich neu zeichnen (große Karten)
    let x0 = 0, y0 = 0, x1 = W - 1, y1 = H - 1;
    if (!S.mapImg || S.mapImg.width !== W || S.mapImg.height !== H) {
      S.mapImg = document.createElement('canvas');
      S.mapImg.width = W; S.mapImg.height = H;
      S.mapImgData = S.mapImg.getContext('2d').createImageData(W, H);
    } else if (g.dirty) {
      [x0, y0, x1, y1] = [Math.max(0, g.dirty[0]), Math.max(0, g.dirty[1]), Math.min(W - 1, g.dirty[2]), Math.min(H - 1, g.dirty[3])];
    }
    g.dirty = null;
    const d = S.mapImgData.data, L = g.L;
    for (let iy = y0; iy <= y1; iy++)
      for (let ix = x0; ix <= x1; ix++) {
        const l = L[iy * W + ix];
        // unbekannt = Grau, frei → weiß, belegt → schwarz (wie flipud(gray) im MATLAB-Skript)
        let v;
        if (l < 0) v = 200 + 55 * Math.min(1, -l / 2);
        else v = 200 * (1 - Math.min(1, l / 2.5));
        const o = ((H - 1 - iy) * W + ix) * 4;
        d[o] = v; d[o + 1] = v; d[o + 2] = l === 0 ? v - 4 : v; d[o + 3] = 255;
      }
    if (x1 >= x0 && y1 >= y0) S.mapImg.getContext('2d').putImageData(S.mapImgData, 0, 0, x0, H - 1 - y1, x1 - x0 + 1, y1 - y0 + 1);
    S.mapImgVersion = g.version;
  }

  function fovCone(view, pose, lid, fill, stroke) {
    const g = view.ctx;
    const fov = (Math.min(360, lid.fov) * Math.PI) / 180;
    g.save();
    g.fillStyle = fill; g.strokeStyle = stroke; g.lineWidth = 1;
    g.beginPath();
    if (lid.fov < 360) g.moveTo(view.X(pose[0]), view.Y(pose[1]));
    const n = 48;
    for (let i = 0; i <= n; i++) {
      const a = pose[2] - fov / 2 + (fov * i) / n;
      const x = view.X(pose[0] + lid.range * Math.cos(a)), y = view.Y(pose[1] + lid.range * Math.sin(a));
      if (i === 0 && lid.fov >= 360) g.moveTo(x, y); else g.lineTo(x, y);
    }
    g.closePath(); g.fill(); g.stroke();
    g.restore();
  }

  function draw() {
    const sim = S.sim;
    if (!sim || !vWorld) return;
    const lid = sim.cfg.lidar;
    // ---- linke Ansicht: wahre Umgebung ----
    let g = vWorld.begin();
    vWorld.image(S.worldImg, 0, 0, S.world.width, S.world.height);
    if (S.mode === 'route' && S.driver && S.driver.route) vWorld.polyline(S.driver.route, C_ROUTE, 1.5, [6, 5], 0.9);
    drawWaypoints(vWorld);
    fovCone(vWorld, sim.truePose, lid, 'rgba(250,178,25,0.22)', 'rgba(230,150,0,0.6)');
    const sc = sim.lastScan;
    drawSweep(vWorld, sim);
    if ($('slShowOdo').checked) vWorld.polyline(sim.odoPath, C_ODO, 1.6, [5, 4]);
    vWorld.polyline(sim.estPath, C_EST, 2);
    vWorld.polyline(sim.truePath, C_TRUE, 2, [7, 4]);
    vWorld.robot(sim.truePose, C_TRUE, 0.2, 8);
    if (S.demo) drawDemoLabels(vWorld);
    vWorld.axes();

    // ---- rechte Ansicht: SLAM-Karte ----
    updateMapImage();
    g = vMap.begin();
    vMap.image(S.mapImg, 0, 0, sim.grid.width, sim.grid.height);
    fovCone(vMap, sim.estPose, lid, 'rgba(250,178,25,0.12)', 'rgba(230,150,0,0.35)');
    if ($('slShowTrue').checked) vMap.polyline(sim.truePath, C_TRUE, 1.6, [7, 4]);
    vMap.polyline(sim.estPath, C_EST, 2.2);
    if (sc) {
      const p = sim.estPose;
      g.fillStyle = C_SCAN;
      for (let k = 0; k < sc.r.length; k++) {
        if (!sc.hit[k]) continue;
        const a = p[2] + sc.ang[k];
        g.fillRect(vMap.X(p[0] + sc.r[k] * Math.cos(a)) - 1.5, vMap.Y(p[1] + sc.r[k] * Math.sin(a)) - 1.5, 3, 3);
      }
    }
    drawWaypoints(vMap);
    vMap.robot(sim.estPose, C_EST, 0.2, 8);
    vMap.axes();
    const m = sim.lastMatch;
    $('slHud').textContent = (S.demo ? `Demo – Runde ${S.demo.runs}, ${fmt((S.driver ? S.driver.progress() : 0) * 100, 0)} % · Tempo ${parseFloat($('slSpeed').value).toLocaleString('de-DE')}×\n` : '') +
      `t = ${fmt(sim.t, 1)} s\n${sim.status}` + (m ? `\nResiduum ${fmt(m.rms * 1000, 1)} mm` : '');
  }

  /**
   * Radar-artige Darstellung des rotierenden Lidars: jeder Messpunkt bleibt an der Stelle, an der er
   * gemessen wurde, bis der Strahl ihn in der nächsten Umdrehung überschreibt. Der aktuelle Strahl
   * wird hervorgehoben, die zuletzt gemessenen Strahlen „leuchten nach“.
   */
  function drawSweep(view, sim) {
    const sw = sim.sweep;
    if (!sw) return;
    const g = view.ctx, n = sw.n;
    g.save();
    g.fillStyle = '#e34948';
    for (let k = 0; k < n; k++) {
      if (!sw.valid[k] || !sw.hit[k]) continue;
      g.fillRect(view.X(sw.wx[k]) - 1.5, view.Y(sw.wy[k]) - 1.5, 3, 3);
    }
    if ($('slShowBeams').checked && sw.last >= 0) {
      const rev = sim.rev, trail = Math.max(1, Math.round(n / 10));
      const line = (k, alpha, width) => {
        g.strokeStyle = `rgba(0,131,0,${alpha})`; g.lineWidth = width;
        g.beginPath(); g.moveTo(view.X(sw.sx[k]), view.Y(sw.sy[k])); g.lineTo(view.X(sw.wx[k]), view.Y(sw.wy[k])); g.stroke();
      };
      if (rev.ideal) {
        // Momentaufnahme: alle Strahlen gleichzeitig
        const step = Math.max(1, Math.round(n / 60));
        for (let k = 0; k < n; k += step) if (sw.valid[k]) line(k, 0.3, 1);
      } else {
        for (let j = trail; j >= 1; j--) {
          const k = (sw.last - j + n) % n;
          if (sw.valid[k]) line(k, 0.4 * (1 - j / (trail + 1)), 1);
        }
        line(sw.last, 0.95, 2.2);
      }
    }
    g.restore();
  }

  function drawPlots() {
    const L = S.sim.log;
    const has = $('slMethod').value !== 'odom';
    plots.e.draw(L.t, [L.eSlam, L.eOdo], [true, true]);
    plots.h.draw(L.t, [L.hSlam, L.hOdo], [true, true]);
    plots.r.draw(L.t, [L.rms.map((v) => (v == null ? null : v * 1000))], [has]);
  }

  function updateReadouts() {
    const s = S.sim, tp = s.truePose, ep = s.estPose, op = s.odoPose;
    const pose = (p) => `${fmt(p[0], 2)} | ${fmt(p[1], 2)} | ${fmt(deg(p[2]), 1)}°`;
    $('slT').textContent = fmt(s.t, 1);
    $('slScans').textContent = s.scans;
    $('slPTrue').textContent = pose(tp);
    $('slPEst').textContent = pose(ep);
    $('slErr').textContent = fmt(Math.hypot(ep[0] - tp[0], ep[1] - tp[1]), 3);
    $('slErrOdo').textContent = fmt(Math.hypot(op[0] - tp[0], op[1] - tp[1]), 3);
    $('slDist').textContent = fmt(s.dist, 2);
    const okStatus = s.status === 'Scan-Matching ok';
    $('slStatus').innerHTML = `<span class="${okStatus ? 'ok' : s.status === 'nur Odometrie' ? '' : 'pending'}">${s.status}</span>`;
    const m = s.lastMatch;
    $('slRms').textContent = m ? fmt(m.rms * 1000, 1) : '–';
    $('slInl').textContent = m ? fmt(m.inlier * 100, 0) : '–';
    $('slRej').textContent = s.rejected;
    $('slMs').textContent = fmt(s.matchMs, 1);
    $('slDist0').textContent = fmt(s.lastDistortion * 1000, 0);
    $('slDist1').textContent = fmt((s.lastDistortionResid ?? s.lastDistortion) * 1000, 0);
    $('slSig').textContent = fmt(Math.sqrt(Math.max(s.P[0][0], s.P[1][1])) * 1000, 1);
    $('slCov').textContent = fmt(s.mapStats.coverage * 100, 0);
    $('slAcc').textContent = fmt(s.mapStats.accuracy * 100, 1);
    let drv = 'Manuell';
    if (S.mode === 'route') drv = S.driver ? (S.driver.done ? 'Route beendet' : `Route ${fmt(S.driver.progress() * 100, 0)} %`) + (S.driver.skipped ? ` · ${S.driver.skipped} Wegpunkt(e) übersprungen` : '') : 'Route wird berechnet';
    if (S.mode === 'random') drv = S.driver ? `Zufall · ${S.driver.turns} Richtungswechsel` : 'Zufall';
    if (S.mode === 'waypoints') {
      drv = !S.driver ? 'Wegpunkte: keine gesetzt' : S.driver.done ? 'Alle Wegpunkte erreicht' : `Wegpunktfahrt ${fmt(S.driver.progress() * 100, 0)} %`;
      if (S.driver && S.driver.failed) drv += ` · ${S.driver.failed}× keine A*-Verbindung`;
      if (S.driver && S.driver.skipped) drv += ` · ${S.driver.skipped} übersprungen`;
    }
    if (s.bumped) drv += ' · Kontakt!';
    $('slDrive').textContent = drv;
  }

  /* ------------------------------ Demo-Vollbild ------------------------------- */
  /** Einstellungen der Demo: Bürogebäude mit Atrium, Rundgang, 90°-Lidar mit 10 Hz. */
  const DEMO_CFG = {
    slMap: 'atrium', slMethod: 'slam', slRes: '20',
    slFov: 90, slRate: 10, slBeams: 180, slRange: 10, slNoise: 0.01, slModel: 'rotating', slDeskew: true,
    slSigV: 1, slSigW: 0.005, slBias: 0.0005, slWinT: 0.1, slWinR: 5, // gut kalibrierter Gyro
    slV: 0.6, slW: 1, slSpeed: '2', slWin: '0',
    slShowOdo: true, slShowTrue: true, slShowBeams: true
  };
  const DEMO_PAUSE_MS = 4000; // fertige Karte am Rundenende kurz stehen lassen

  /** Schriftzüge der Demo in den freien Ecken über der Karte (Schriftgröße mit dem Kartenmaßstab). */
  function drawDemoLabels(view) {
    const g = view.ctx, W = S.world.width, H = S.world.height;
    const fs = Math.max(12, 3.2 * view.s); // ≈ 3,2 m Schrifthöhe in Kartenmaßstab
    g.save();
    g.font = `500 ${fs}px system-ui, -apple-system, "Segoe UI", Roboto, sans-serif`;
    g.fillStyle = '#ffffff';
    g.textBaseline = 'top';
    g.shadowColor = 'rgba(0, 0, 0, 0.75)'; g.shadowBlur = fs * 0.12; g.shadowOffsetX = g.shadowOffsetY = fs * 0.05;
    const top = view.Y(H) + 0.04 * H * view.s;
    g.textAlign = 'left';
    ['Elektrotechnik und', 'Informationstechnik'].forEach((t, i) => g.fillText(t, view.X(0.035 * W), top + i * fs * 1.15));
    g.textAlign = 'right';
    g.fillText('DHBW Stuttgart', view.X(0.965 * W), top);
    g.restore();
  }

  function startDemo() {
    if (S.demo) return;
    // aktuelle Einstellungen sichern, nach der Demo wiederherstellen
    const saved = {};
    for (const id of Object.keys(DEMO_CFG)) { const el = $(id); saved[id] = el.type === 'checkbox' ? el.checked : el.value; }
    S.demo = { runs: 0, saved, mode: S.mode, restartAt: 0 };
    applyFields(DEMO_CFG);
    setMode('route', true);
    S.demo.ctl = MRS.enterDemo($('slCenter'), onDemoExit, {
      values: [0.1, 0.25, 0.5, 1, 2, 5, 10],
      get: () => parseFloat($('slSpeed').value),
      set: (v) => { $('slSpeed').value = String(v); }
    });
    demoRun();
  }

  function applyFields(vals) {
    for (const [id, v] of Object.entries(vals)) { const el = $(id); if (el.type === 'checkbox') el.checked = v; else el.value = v; }
    $('slLegOdo').hidden = !$('slShowOdo').checked;
    for (const p of Object.values(plots)) p.window = parseFloat($('slWin').value);
    updateLidarInfo();
  }

  /** Neue Runde: SLAM von vorn, Rundgang ab dem Startpunkt. */
  function demoRun() {
    S.demo.runs++;
    reset();
    setRunning(true);
  }

  /** Nach dem Beenden (Esc, Doppelklick, Vollbild verlassen): vorherige Einstellungen wiederherstellen. */
  function onDemoExit() {
    const d = S.demo;
    S.demo = null;
    setRunning(false);
    applyFields(d.saved);
    setMode(d.mode, true);
    reset();
  }

  MRS.SlamTab = {
    init,
    state: () => S, // für Tests/Fehlersuche
    setActive(a) {
      S.active = a;
      S.keys.clear(); updateKeypad();
      if (a) { vWorld.resize(); vMap.resize(); for (const p of Object.values(plots)) p._resize(); draw(); drawPlots(); }
      else setRunning(false);
    }
  };
})();
