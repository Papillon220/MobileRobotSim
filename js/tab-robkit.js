/* ==========================================================================
   MobileRobotSim – Reiter „ROBKIT1 Control“: Roboter ROBKIT1 per Web Bluetooth steuern,
   kartieren und aufzeichnen oder ohne Hardware simulieren
   (Unterreiter Steuerung, Simulation, Replay, Messwerte, Parameter, Aufzeichnung)
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;

  // ============================================================================
  // Protocol (see PROTOCOL.md and src/rk_app.c)
  // ============================================================================
  const UUID = {
    service: "00000000-0001-11e1-9ab4-0002a5d5c51b",
    telem:   "00000001-000a-11e1-ac36-0002a5d5c51b",
    cmd:     "00000002-000a-11e1-ac36-0002a5d5c51b",
  };
  const PKT = { STATE: 0x01, TOF: 0x02, PARAM: 0x03, WP_ACK: 0x04, HEADING: 0x05 };
  const CMD = { MODE: 0x10, JOY: 0x11, ODOM_RESET: 0x12, STREAM: 0x13, PARAM_SET: 0x20, PARAM_GET: 0x21,
                WP_CLEAR: 0x30, WP_SET: 0x31, WP_COMMIT: 0x32 };
  const MODE_NAMES = ["Stopp", "Fernsteuerung", "Autopilot", "Follow me", "Wegpunkte"];
  const NAV_NAMES = ["aus", "vorwärts", "drehen", "zurück"];
  const WP_NAMES = ["aus", "fährt", "fertig", "blockiert"];
  const TRACK_CM = 11.5;      // wheel distance (RK.TrackWidth)
  const TOF_FOV_DEG = 45;
  const TOF_OFFSET_CM = 6;    // sensor in front of the wheel axis
  const WP_MAX = 16;

  const cmd = {
    mode: (m) => [CMD.MODE, m, 1],
    joy: (s, st) => [CMD.JOY, s & 0xff, st & 0xff],
    odomReset: () => [CMD.ODOM_RESET],
    stream: (hz) => [CMD.STREAM, hz > 0 ? 1 : 0, Math.max(1, hz)],
    paramSet: (id, v) => { const b = new DataView(new ArrayBuffer(6)); b.setUint8(0, CMD.PARAM_SET); b.setUint8(1, id); b.setFloat32(2, v, true); return new Uint8Array(b.buffer); },
    paramGet: (id) => [CMD.PARAM_GET, id],
    wpClear: () => [CMD.WP_CLEAR],
    wpSet: (first, pts) => {
      const b = new DataView(new ArrayBuffer(3 + 4 * pts.length));
      b.setUint8(0, CMD.WP_SET); b.setUint8(1, first); b.setUint8(2, pts.length);
      pts.forEach((p, i) => { b.setInt16(3 + 4 * i, Math.round(p.x), true); b.setInt16(5 + 4 * i, Math.round(p.y), true); });
      return new Uint8Array(b.buffer);
    },
    wpCommit: (n) => [CMD.WP_COMMIT, n],
  };

  // ============================================================================
  // Helpers
  // ============================================================================
  // ids of this tab carry the prefix "rk" (element rkMapCanvas for "mapCanvas"), they must not collide with the other tabs
  const $ = (id) => document.getElementById("rk" + id[0].toUpperCase() + id.slice(1));
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  function toast(msg, ms = 2600) {
    const t = $("toast"); t.textContent = msg; t.classList.add("show");
    clearTimeout(toast.h); toast.h = setTimeout(() => t.classList.remove("show"), ms);
  }
  function download(name, text, type = "text/csv") {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([text], { type }));
    a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  }
  function fitCanvas(c) {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(c.clientWidth * dpr)), h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    return dpr;
  }
  const wrap180 = (a) => ((a + 180) % 360 + 360) % 360 - 180;
  const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");

  // ============================================================================
  // Links: Web Bluetooth and demo robot share one interface
  // ============================================================================
  class BleLink {
    constructor(onPacket, onClose) { this.onPacket = onPacket; this.onClose = onClose; this.queue = Promise.resolve(); }
    async connect(anyDevice) {
      const opts = anyDevice ? { acceptAllDevices: true, optionalServices: [UUID.service] }
                             : { filters: [{ namePrefix: "ROBKIT" }], optionalServices: [UUID.service] };
      this.device = await navigator.bluetooth.requestDevice(opts);
      this.device.addEventListener("gattserverdisconnected", () => this.onClose());
      const server = await this.device.gatt.connect();
      const svc = await server.getPrimaryService(UUID.service);
      this.telem = await svc.getCharacteristic(UUID.telem);
      this.cmd = await svc.getCharacteristic(UUID.cmd);
      this.telem.addEventListener("characteristicvaluechanged", (e) => this.onPacket(e.target.value));
      await this.telem.startNotifications();
      return this.device.name || "ROBKIT";
    }
    // GATT operations must not overlap: all writes go through one promise chain
    write(bytes, fast = false) {
      const buf = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
      this.queue = this.queue.then(() => {
        if (!this.cmd) return;
        if (fast && this.cmd.writeValueWithoutResponse) return this.cmd.writeValueWithoutResponse(buf);
        return this.cmd.writeValueWithResponse ? this.cmd.writeValueWithResponse(buf) : this.cmd.writeValue(buf);
      }).catch((e) => console.warn("write", e));
      return this.queue;
    }
    disconnect() { if (this.device && this.device.gatt.connected) this.device.gatt.disconnect(); }
  }

  // Simulated robot (tab "Simulation"): true motion with wheel slip, estimated pose from the
  // encoders (wheel radius error) and optionally a gyroscope, ToF ray casting, remote drive,
  // a simple autopilot and the waypoint drive (Pure Pursuit as in the model).
  // Rooms in cm, the robot starts at (0, 0) looking along +x.
  const box = (x, y, w, h) => [[x, y, x + w, y], [x + w, y, x + w, y + h], [x + w, y + h, x, y + h], [x, y + h, x, y]];
  const rect = (x0, y0, x1, y1) => box(x0, y0, x1 - x0, y1 - y0);
  const SIM_ROOMS = {
    lab: [...rect(-40, -50, 260, 150), ...box(80, 80, 30, 30), ...box(160, -10, 25, 30), ...box(5, 95, 15, 15)],
    corridor: [[-40, -40, 400, -40], [-40, 40, 200, 40], [280, 40, 400, 40], [-40, -40, -40, 40], [400, -40, 400, 40],
      [200, 40, 200, 260], [280, 40, 280, 260], [200, 260, 280, 260]],
    parcours: [...rect(-40, -60, 360, 240), ...box(60, -60, 20, 160), ...box(160, 80, 20, 160), ...box(250, -10, 40, 40),
      ...box(90, 170, 30, 30)],
    empty: rect(-50, -150, 350, 150),
    // flat: left room (start), door to the right room, kitchen behind a second door
    flat: [...rect(-50, -50, 450, 350),
      [150, -50, 150, 120], [150, 200, 150, 350],                    // wall with door 120 ... 200
      [150, 150, 300, 150], [370, 150, 450, 150],                    // wall with door 300 ... 370
      ...box(30, 250, 60, 60), ...box(250, 260, 70, 50), ...box(330, -20, 70, 40), ...box(-40, 120, 40, 80)],
    office: [...rect(-50, -100, 550, 300),
      ...box(100, 80, 120, 60), ...box(300, 80, 120, 60), ...box(150, -100, 100, 40), ...box(330, -60, 60, 60),
      ...box(470, 120, 30, 30), ...box(-50, 200, 80, 100)],
    maze: [...rect(-40, -40, 360, 360),
      [-40, 40, 200, 40], [120, 40, 120, 200], [40, 120, 40, 280], [40, 200, 120, 200],
      [200, 120, 280, 120], [280, 40, 280, 280], [200, 120, 200, 280], [120, 280, 200, 280], [280, 280, 360, 280]],
    octagon: (() => {
      const pts = Array.from({ length: 8 }, (_, k) => [150 + 210 * Math.cos((22.5 + 45 * k) * Math.PI / 180), 210 * Math.sin((22.5 + 45 * k) * Math.PI / 180)]);
      return [...pts.map((p, k) => [...p, ...pts[(k + 1) % 8]]), ...box(150, -20, 40, 40)];
    })(),
    custom: [],
  };

  // Office building with atrium: occupancy grid MRS.ATRIUM_MAP (js/map-atrium.js)
  class GridWorld {
    constructor(d) {
      const A = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_";
      this.W = d.W; this.H = d.H; this.cell = 100 / d.res;                 // cm per cell
      this.width = this.W * this.cell; this.height = this.H * this.cell;    // cm, origin bottom left
      this.occ = new Uint8Array(this.W * this.H);
      d.rows.split("|").forEach((row, r) => {
        const iy = this.H - 1 - r;
        let c = 0, occ = false, n = 0, sh = 0;
        for (let i = 0; i < row.length; i++) {
          const v = A.indexOf(row[i]);
          n += (v & 31) << sh;
          if (v >= 32) { sh += 5; continue; }
          if (occ) this.occ.fill(1, iy * this.W + c, iy * this.W + Math.min(this.W, c + n));
          c += n; occ = !occ; n = 0; sh = 0;
        }
      });
    }
    occAt(x, y) {
      const ix = Math.floor(x / this.cell), iy = Math.floor(y / this.cell);
      return ix < 0 || iy < 0 || ix >= this.W || iy >= this.H || this.occ[iy * this.W + ix] === 1;
    }
    // distance to the first occupied cell (steps of 2.5 cm), at most max cm
    ray(x, y, a, max = 400) {
      const dx = Math.cos(a), dy = Math.sin(a);
      for (let t = 0; t < max; t += 2.5) if (this.occAt(x + t * dx, y + t * dy)) return t;
      return max;
    }
    // image of the world (1 pixel per cell), repainted after a theme change
    image() {
      const key = css("--map-occ") + css("--map-free");
      if (this.img && this.imgKey === key) return this.img;
      const hex = (h) => { const v = parseInt(h.replace("#", ""), 16); return [(v >> 16) & 255, (v >> 8) & 255, v & 255]; };
      const cw = hex(css("--map-occ")), cf = hex(css("--map-free"));
      const cv = document.createElement("canvas"); cv.width = this.W; cv.height = this.H;
      const g = cv.getContext("2d"), im = g.createImageData(this.W, this.H);
      for (let iy = 0; iy < this.H; iy++) for (let ix = 0; ix < this.W; ix++) {
        const o = ((this.H - 1 - iy) * this.W + ix) * 4, col = this.occ[iy * this.W + ix] ? cw : cf;
        im.data[o] = col[0]; im.data[o + 1] = col[1]; im.data[o + 2] = col[2]; im.data[o + 3] = 255;
      }
      g.putImageData(im, 0, 0);
      this.img = cv; this.imgKey = key;
      return cv;
    }
  }
  // description and default start pose (cm, rad counter-clockwise) per room
  const SIM_INFO = {
    lab: { desc: "Raum 3 m × 2 m mit drei Kisten." },
    atrium: { desc: "Etage eines Hochschul- und Bürogebäudes, 107,8 m × 72,3 m: Hörsäle, Seminarräume und Büros um ein Atrium mit Wendeltreppe, Flure mit Stützen. Die Karte des Roboters wird dafür auf 160 m × 160 m (10-cm-Raster) gestellt, so ist das ganze Gebäude abfahrbar.",
      start: { x: 2805, y: 3775, psi: 27 * Math.PI / 180 } },
    flat: { desc: "Wohnung mit drei Zimmern, verbunden über Türen, mit Möbeln." },
    office: { desc: "Büro 6 m × 4 m mit Tischen und Schränken." },
    maze: { desc: "Labyrinth 4 m × 4 m mit 80 cm breiten Gängen." },
    octagon: { desc: "Achteckiger Raum mit einer Säule in der Mitte." },
    corridor: { desc: "Flur mit Abzweig nach links." },
    parcours: { desc: "Parcours mit Trennwänden und Hindernissen." },
    empty: { desc: "Leerer Raum 4 m × 3 m." },
    custom: { desc: "Eigene Karte aus einer JSON-Datei." },
  };
  let atriumWorld = null;
  function simRoom(id) {
    const start = (SIM_INFO[id] && SIM_INFO[id].start) || { x: 0, y: 0, psi: 0 };
    if (id === "atrium") return { world: atriumWorld || (atriumWorld = new GridWorld(MRS.ATRIUM_MAP)), start };
    return { walls: SIM_ROOMS[id] || SIM_ROOMS.lab, start };
  }

  class DemoLink {
    constructor(onPacket, onClose) {
      this.onPacket = onPacket; this.onClose = onClose;
      this.mode = 0; this.joy = [0, 0]; this.t = 0; this.cnt = 0; this.frame = 0;
      this.tofHz = 4; this.lastTof = 0; this.lastState = 0;
      this.wp = []; this.wpEdit = []; this.wpIdx = 0; this.wpState = 0; this.nav = 0;
      this.params = [["DriveVmax", 15], ["DriveWmax", 10], ["StopDist", 180], ["ClearDist", 350], ["Vmin", 6], ["Vmax", 14],
        ["Kv", 0.02], ["Ksteer", 3], ["Wturn", 6], ["TurnTimeout", 4], ["CliffDist", 350], ["BackTime", 0.8], ["Vback", 8],
        ["FollowDist", 200], ["Kd", 0.08], ["Ka", 0.5], ["FollowVmax", 14], ["FollowWmax", 8], ["FlipRows", 0], ["FlipCols", 0],
        ["WpVel", 10], ["WpLookahead", 20], ["WpTurnAngle", 60], ["WpTol", 5], ["WpStopDist", 150],
        ["PoseSource", 1], ["MagGain", 0], ["NavVariant", 1], ["GyroZupt", 0], ["GyroScale", 1]];
      this.wpMinRadius = 12;      // RK.WpMinRadius (not tunable)
      this.rays = []; this.biasEst = 0;   // gyro offset learned by the fusion (kept when the robot is put back)
      this.configure(sim.settings(), true);
    }
    configure(s, newRoom) {
      this.cfg = s;
      if (newRoom) { const r = simRoom(s.room); this.walls = r.walls || null; this.world = r.world || null; this.resetPose(); }
    }
    // robot at the start pose of the tab "Simulation"; the odometry starts there with (0, 0, 0)
    resetPose() {
      const st = sim.startPose();
      this.p = { x: st.x, y: st.y, psi: st.psi, vl: 0, vr: 0 };   // true pose (cm, rad counter-clockwise)
      this.est = { x: st.x, y: st.y, psi: st.psi };              // estimated pose (world frame)
      this.psiEnc = st.psi; this.psiGyro = st.psi; this.psiFus = st.psi; this.yawRate = 0;
      this.origin = { ...this.est };
      this.dist = 0; sim.clearTrails();
    }
    async connect() {
      let last = performance.now();
      this.timer = setInterval(() => {      // 20 ms steps, several per tick when faster than real time
        const now = performance.now(), n = Math.min(40, Math.round((now - last) / 20 * this.cfg.speed));
        if (n > 0) last = now;
        for (let k = 0; k < n; k++) this.step(0.02);
      }, 20);
      return "Simulation";
    }
    disconnect() { clearInterval(this.timer); this.onClose(); }
    par(name) { return this.params.find((p) => p[0] === name)[1]; }
    write(bytes) {
      const m = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
      const dv = new DataView(m.buffer, m.byteOffset, m.byteLength);
      switch (m[0]) {
        case CMD.MODE: this.mode = m[1]; this.joy = [0, 0]; this.nav = 0; break;
        case CMD.JOY: this.joy = [dv.getInt8(1), dv.getInt8(2)]; break;
        case CMD.ODOM_RESET: this.origin = { ...this.est }; break;
        case CMD.STREAM: this.tofHz = m[1] ? m[2] : 0; break;
        case CMD.PARAM_SET: if (m[1] < this.params.length) { this.params[m[1]][1] = dv.getFloat32(2, true); this.sendParam(m[1]); } break;
        case CMD.PARAM_GET: if (m[1] === 0xff) this.params.forEach((_, i) => this.sendParam(i)); else this.sendParam(m[1]); break;
        case CMD.WP_CLEAR: this.wpEdit = []; break;
        case CMD.WP_SET: for (let k = 0; k < m[2]; k++) this.wpEdit[m[1] + k] = { x: dv.getInt16(3 + 4 * k, true), y: dv.getInt16(5 + 4 * k, true) }; break;
        case CMD.WP_COMMIT: {
          this.wp = this.wpEdit.slice(0, m[1]); this.wpIdx = 0;
          if (this.mode === 4 && this.wp.length) this.startPath();
          const b = new DataView(new ArrayBuffer(6)); b.setUint8(0, PKT.WP_ACK); b.setUint8(1, this.wp.length); b.setUint32(2, ++this.seq || (this.seq = 1), true);
          this.emit(b); break;
        }
      }
      return Promise.resolve();
    }
    sendParam(i) {
      const b = new DataView(new ArrayBuffer(20)); const [name, v] = this.params[i];
      b.setUint8(0, PKT.PARAM); b.setUint8(1, i); b.setUint8(2, this.params.length); b.setUint8(3, i === this.params.length - 1 ? 1 : 0);
      b.setFloat32(4, v, true); for (let k = 0; k < Math.min(12, name.length); k++) b.setUint8(8 + k, name.charCodeAt(k));
      this.emit(b);
    }
    emit(dv) { setTimeout(() => this.onPacket(dv), 5); }
    odom() {   // estimated pose in the motor board convention relative to the last reset
      const o = this.origin || { x: 0, y: 0, psi: 0 };
      const dx = this.est.x - o.x, dy = this.est.y - o.y, c = Math.cos(o.psi), s = Math.sin(o.psi);
      return { x: c * dx + s * dy, y: -s * dx + c * dy, th: -wrap180((this.est.psi - o.psi) * 180 / Math.PI) };
    }
    ray(x, y, a) {
      if (this.world) return this.world.ray(x, y, a);
      let best = 400; const dx = Math.cos(a), dy = Math.sin(a);
      for (const [x1, y1, x2, y2] of this.walls) {
        const ex = x2 - x1, ey = y2 - y1, den = dx * ey - dy * ex;
        if (Math.abs(den) < 1e-9) continue;
        const t = ((x1 - x) * ey - (y1 - y) * ex) / den, u = ((x1 - x) * dy - (y1 - y) * dx) / den;
        if (t > 0 && u >= 0 && u <= 1) best = Math.min(best, t);
      }
      return best;
    }
    // distance per ToF column (cm), column 1 = left
    scan() {
      const p = this.p, sx = p.x + TOF_OFFSET_CM * Math.cos(p.psi), sy = p.y + TOF_OFFSET_CM * Math.sin(p.psi), d = [];
      for (let c = 1; c <= 8; c++) d.push(this.ray(sx, sy, p.psi + (4.5 - c) * TOF_FOV_DEG / 8 * Math.PI / 180));
      return { sx, sy, d };
    }
    autopilot(front, scan) {
      // forward, slower close to obstacles. Standard (NavVariant 1): turn on the spot to the freer side until the
      // corridor is clear. Random Turn (2): turn by a random angle (45 ... 180 deg, time controlled), then go on
      const stop = this.par("StopDist"), clear = this.par("ClearDist");
      if (this.par("NavVariant") >= 1.5) {
        if ((this.nav !== 2 && front < stop) || (this.nav === 2 && this.t >= this.turnEnd && front < stop)) this.randomTurn();
        else if (this.nav === 2 && this.t >= this.turnEnd) this.nav = 1;
      } else {
        if (this.nav !== 2 && front < stop) {
          this.nav = 2; this.turnDir = scan.d.slice(0, 4).reduce((a, b) => a + b) > scan.d.slice(4).reduce((a, b) => a + b) ? 1 : -1;
        }
        if (this.nav === 2 && front > clear) this.nav = 1;
      }
      if (this.nav === 0) this.nav = 1;
      if (this.nav === 2) { const w = this.par("Wturn"); return [-w * this.turnDir, w * this.turnDir]; }
      const v = Math.min(this.par("Vmax"), Math.max(this.par("Vmin"), this.par("Vmin") + this.par("Kv") * (front - stop)));
      return [v, v];
    }
    step(dt) {
      this.t += dt;
      const p = this.p, scan = this.scan(), front = Math.min(...scan.d.slice(1, 7)) * 10;
      let l = 0, r = 0;
      if (this.mode === 1) {
        const v = this.joy[0] / 100 * this.par("DriveVmax"), w = this.joy[1] / 100 * this.par("DriveWmax");
        l = v + w; r = v - w;
      } else if (this.mode === 2) {
        [l, r] = this.autopilot(front, scan);
      } else if (this.mode === 4) {
        if (this.wpState === 0 && this.wp.length) this.startPath();
        if (this.wpState === 1 || this.wpState === 3) {
          const o = this.odom(), tg = this.wp[this.wpIdx], last = this.wpIdx === this.wp.length - 1;
          const dx = tg.x - o.x, dy = tg.y - o.y, psi = -o.th * Math.PI / 180;
          const behind = dx * Math.cos(psi) + dy * Math.sin(psi) < 0, dist = Math.hypot(dx, dy);
          if (dist < (last ? this.par("WpTol") : this.par("WpLookahead")) || (behind && dist < this.par("WpLookahead"))) {
            if (last) this.wpState = 2; else { this.wpIdx++; this.setPath(o); }
          }
          if (this.wpState !== 2) {
            const pp = this.purePursuit(o), turn = Math.abs(pp.alpha) > this.par("WpTurnAngle") * Math.PI / 180;
            const blocked = front < this.par("WpStopDist") && !turn;
            this.wpState = blocked ? 3 : 1;
            if (!blocked) {
              const v = turn ? 0 : this.par("WpVel");
              const w = turn ? Math.sign(pp.alpha) * this.par("Wturn") : this.par("WpVel") * pp.kappa * TRACK_CM / 2;
              l = v - w; r = v + w;
            }
          }
        }
      }
      if (this.mode !== 4) this.wpState = 0;
      if (this.mode !== 2) this.nav = 0;
      this.cmdL = l; this.cmdR = r;
      const k = dt / 0.1;   // wheel speed control (PT1, 0.1 s)
      p.vl += k * (l - p.vl); p.vr += k * (r - p.vr);

      // true motion: the wheels slip, more when turning
      const turnShare = Math.abs(p.vr - p.vl) / (Math.abs(p.vl) + Math.abs(p.vr) + 1e-6);
      const slip = this.cfg.slip * (0.3 + turnShare) * (0.5 + Math.random());
      const gl = p.vl * (1 - slip), gr = p.vr * (1 - slip);
      const v = (gl + gr) / 2, w = (gr - gl) / TRACK_CM;
      const nx = p.x + v * Math.cos(p.psi) * dt, ny = p.y + v * Math.sin(p.psi) * dt;
      if (this.ray(p.x, p.y, Math.atan2(ny - p.y, nx - p.x)) > 10 || v < 0) { p.x = nx; p.y = ny; this.dist += Math.abs(v * dt); }
      p.psi += w * dt;

      // estimate: encoders see the wheel rotation (right wheel radius error), heading from encoders, gyro or both
      const el = p.vl * dt, er = p.vr * dt / (1 + this.cfg.scale);
      const ed = (el + er) / 2;
      this.psiEnc += (er - el) / TRACK_CM;
      const gyro = (w + this.cfg.gyroOffset * Math.PI / 180 + (Math.random() - 0.5) * 0.002) * this.par("GyroScale");   // measured yaw rate: offset, noise, scale
      this.psiGyro += gyro * dt;
      // fusion: gyro heading, its offset is learned while the encoders report standstill (time constant 2 s) and subtracted
      if (Math.abs(el) + Math.abs(er) < 0.01 * dt) this.biasEst += (gyro - this.biasEst) * dt / 2;
      this.psiFus += (gyro - this.biasEst) * dt;
      this.yawRate = -gyro * 180 / Math.PI;
      // parameter GyroZupt (stage 1 of the localization): the gyro heading learns its offset at standstill
      this.est.psi = { enc: this.psiEnc, gyro: this.par("GyroZupt") >= 0.5 ? this.psiFus : this.psiGyro, fused: this.psiFus }[this.cfg.heading];
      this.est.x += ed * Math.cos(this.est.psi); this.est.y += ed * Math.sin(this.est.psi);
      this.cnt = (this.cnt + 1) & 0xff;

      const now = this.t * 1000;
      if (now - this.lastState >= 100) { this.lastState = now; this.sendState(front); sim.sample(this); }
      if (this.tofHz > 0 && now - this.lastTof >= 1000 / this.tofHz) { this.lastTof = now; this.sendTof(scan); }
    }
    randomTurn() {
      const a = (45 + Math.random() * 135) * Math.PI / 180;     // RK.RandTurnMin ... RK.RandTurnMax
      this.nav = 2; this.turnDir = Math.random() < 0.5 ? 1 : -1;
      this.turnEnd = this.t + a * TRACK_CM / (2 * this.par("Wturn"));
    }
    startPath() {
      const o = this.odom();
      this.start = { x: o.x, y: o.y }; this.wpIdx = 0; this.wpState = 1; this.setPath(o);
    }
    setPath() {
      const anchor = this.wpIdx === 0 ? this.start : this.wp[this.wpIdx - 1];
      this.path = [anchor, ...this.wp.slice(this.wpIdx)]; this.projIdx = 0; this.proj = { ...anchor };
    }
    purePursuit(o) {
      const P = this.path, L = this.par("WpLookahead");
      const closest = (a, b) => {
        const ex = b.x - a.x, ey = b.y - a.y, n = ex * ex + ey * ey;
        const u = n > 0 ? Math.max(0, Math.min(1, ((o.x - a.x) * ex + (o.y - a.y) * ey) / n)) : 0;
        const q = { x: a.x + u * ex, y: a.y + u * ey }; return [q, Math.hypot(q.x - o.x, q.y - o.y)];
      };
      let [best, dmin] = closest(this.proj, P[this.projIdx + 1] || this.proj);
      let along = Math.hypot(best.x - (P[this.projIdx + 1] || best).x, best.y - (P[this.projIdx + 1] || best).y);
      for (let i = this.projIdx + 1; i < P.length - 1 && along <= L; i++) {
        along += Math.hypot(P[i + 1].x - P[i].x, P[i + 1].y - P[i].y);
        const [q, d] = closest(P[i], P[i + 1]);
        if (d < dmin) { dmin = d; best = q; this.projIdx = i; }
      }
      this.proj = best;
      let start = best, idx = this.projIdx, rest = L, look = P[P.length - 1];
      while (idx < P.length - 1) {
        const end = P[idx + 1], seg = Math.hypot(end.x - start.x, end.y - start.y);
        if (seg >= rest) { look = { x: start.x + (end.x - start.x) * rest / seg, y: start.y + (end.y - start.y) * rest / seg }; break; }
        rest -= seg; start = end; idx++;
      }
      const psi = -o.th * Math.PI / 180;
      const alpha = wrap180((Math.atan2(look.y - o.y, look.x - o.x) - psi) * 180 / Math.PI) * Math.PI / 180;
      const kMax = 1 / this.wpMinRadius;
      return { alpha, kappa: Math.max(-kMax, Math.min(kMax, 2 * Math.sin(alpha) / L)) };
    }
    sendState(front) {
      const o = this.odom(), b = new DataView(new ArrayBuffer(20));
      b.setUint8(0, PKT.STATE); b.setUint16(1, Math.round(this.t * 100) & 0xffff, true);
      b.setInt16(3, Math.round(o.x * 10), true); b.setInt16(5, Math.round(o.y * 10), true); b.setInt16(7, Math.round(o.th * 10), true);
      b.setInt8(9, Math.round(this.cmdL || 0)); b.setInt8(10, Math.round(this.cmdR || 0));
      b.setUint8(11, this.mode); b.setUint8(12, this.nav); b.setUint8(13, 0); b.setUint8(14, this.wpIdx); b.setUint8(15, this.mode === 4 ? this.wpState : 0);
      b.setUint16(16, Math.min(65535, Math.round(front)), true); b.setUint8(18, Math.round(7.6 * 20)); b.setUint8(19, this.cnt);
      this.emit(b);
      // headings like the firmware: encoders only, used pose, compass (true heading +- 3 deg), relative to the reset
      const ref = this.origin ? this.origin.psi : 0, deg = (psi) => -wrap180((psi - ref) * 180 / Math.PI);
      const h = new DataView(new ArrayBuffer(14));
      h.setUint8(0, PKT.HEADING);
      h.setInt16(1, Math.round(deg(this.psiEnc) * 10), true); h.setInt16(3, Math.round(deg(this.est.psi) * 10), true);
      h.setInt16(5, Math.round((deg(this.p.psi) + (Math.random() - 0.5) * 6) * 10), true); h.setInt16(7, Math.round(this.yawRate * 10), true);
      h.setUint8(9, 1 | (this.cfg.heading !== "enc" ? 2 : 0));
      this.emit(h);
    }
    sendTof(scan) {
      const floor = [57, 34, 23], z = new Uint8Array(64);
      this.rays = [];
      scan.d.forEach((dist, i) => {
        const c = i + 1, d = dist + (Math.random() - 0.5), a = this.p.psi + (4.5 - c) * TOF_FOV_DEG / 8 * Math.PI / 180;
        const len = Math.min(dist, Number($("mapRange").value));    // drawn inside the field of view (mapping range)
        this.rays.push([scan.sx, scan.sy, scan.sx + len * Math.cos(a), scan.sy + len * Math.sin(a)]);
        for (let r = 1; r <= 8; r++) {
          const v = r <= 5 ? d : Math.min(d, floor[r - 6]);
          z[(r - 1) * 8 + (c - 1)] = v > 254 ? 0xff : Math.max(1, Math.round(v));
        }
      });
      this.frame = (this.frame + 1) & 0xff;
      for (let part = 0; part < 4; part++) {
        const b = new DataView(new ArrayBuffer(20));
        b.setUint8(0, PKT.TOF); b.setUint8(1, this.frame); b.setUint8(2, part);
        for (let k = 0; k < 16; k++) b.setUint8(4 + k, z[part * 16 + k]);
        this.emit(b);
      }
    }
  }

  // ============================================================================
  // Robot state, measured wheel speeds, history
  // ============================================================================
  const S = {
    link: null, demo: false, connected: false,
    t: 0, tRaw: null, tOff: 0,
    x: 0, y: 0, th: 0, cmdL: 0, cmdR: 0, measL: 0, measR: 0,
    mode: 0, nav: 0, flags: 0, wpIdx: 0, wpState: 0, front: 0, vbat: 0, cnt: -1,
    lastOdom: null, tof: null, tofParts: {}, params: [], trail: [],
  };
  const REC = { on: false, rows: [], tof: [], t0: 0 };

  function onPacket(dv) {
    const type = dv.getUint8(0);
    if (type === PKT.STATE && dv.byteLength >= 20) onState(dv);
    else if (type === PKT.TOF && dv.byteLength >= 20) onTofPart(dv);
    else if (type === PKT.PARAM && dv.byteLength >= 8) onParam(dv);
    else if (type === PKT.HEADING && dv.byteLength >= 10) onHeading(dv);
    else if (type === PKT.WP_ACK && dv.byteLength >= 2) {
      $("wpInfo").textContent = `Übertragen: ${dv.getUint8(1)} Wegpunkte ✓`;
      mission.acked = true;
    }
  }

  function onState(dv) {
    const raw = dv.getUint16(1, true);            // 10 ms, wraps after 655 s
    if (S.tRaw !== null && raw < S.tRaw) S.tOff += 65536;
    S.tRaw = raw; S.t = (raw + S.tOff) / 100;
    S.x = dv.getInt16(3, true) / 10; S.y = dv.getInt16(5, true) / 10; S.th = dv.getInt16(7, true) / 10;
    S.cmdL = dv.getInt8(9); S.cmdR = dv.getInt8(10);
    S.mode = dv.getUint8(11); S.nav = dv.getUint8(12); S.flags = dv.getUint8(13);
    S.wpIdx = dv.getUint8(14); S.wpState = dv.getUint8(15);
    S.front = dv.getUint16(16, true); S.vbat = dv.getUint8(18) / 20;
    const cnt = dv.getUint8(19);
    if (cnt !== S.cnt) {               // new odometry sample -> measured wheel speeds
      // heading change from the encoders: the pose heading (gyro) creeps in 0.1 deg steps while standing
      const o = { t: S.t, x: S.x, y: S.y, th: S.th, thW: S.hasHeading ? S.thEnc : S.th };
      if (S.lastOdom && o.t > S.lastOdom.t) {
        const dt = o.t - S.lastOdom.t, psi = -S.lastOdom.th * Math.PI / 180;
        const v = ((o.x - S.lastOdom.x) * Math.cos(psi) + (o.y - S.lastOdom.y) * Math.sin(psi)) / dt;
        const w = -wrap180(o.thW - S.lastOdom.thW) * Math.PI / 180 / dt;    // rad/s, counter-clockwise
        S.measL = 0.5 * S.measL + 0.5 * (v - w * TRACK_CM / 2);
        S.measR = 0.5 * S.measR + 0.5 * (v + w * TRACK_CM / 2);
      }
      S.lastOdom = o; S.cnt = cnt;
    }
    const last = S.trail[S.trail.length - 1];
    const mp = loc.toMap(S.x, S.y, S.th);    // path in the map frame
    if (!last || Math.hypot(last[0] - mp.x, last[1] - mp.y) > 0.5) { S.trail.push([mp.x, mp.y]); if (S.trail.length > 20000) S.trail.splice(0, 5000); }
    charts.push(S.t); traj.push();
    updateMission();
    if (REC.on) {
      REC.rows.push([+S.t.toFixed(2), +S.x.toFixed(1), +S.y.toFixed(1), +S.th.toFixed(1), S.cmdL, S.cmdR, +S.measL.toFixed(1), +S.measR.toFixed(1),
        S.mode, S.nav, S.wpIdx, S.wpState, S.front, +S.vbat.toFixed(2), S.flags & 1, (S.flags >> 1) & 1,
        S.hasHeading ? S.thEnc : "", S.hasHeading ? S.thMag : "", S.hasHeading ? S.yawRate : ""]);
      recView.addState(REC.rows[REC.rows.length - 1]);
    }
    ui.dirty = true;
  }

  // headings from encoders, gyroscope and compass (Pose Estimation of the model)
  function onHeading(dv) {
    S.thEnc = dv.getInt16(1, true) / 10; S.thGyro = dv.getInt16(3, true) / 10; S.thMag = dv.getInt16(5, true) / 10;
    S.yawRate = dv.getInt16(7, true) / 10; S.imuFlags = dv.getUint8(9); S.hasHeading = true;
    charts.heading.push(S.t, [S.th, S.thEnc, S.thMag]);
  }

  function onTofPart(dv) {
    const frame = dv.getUint8(1), part = dv.getUint8(2);
    if (part > 3) return;
    let f = S.tofParts[frame];
    if (!f) { S.tofParts = { [frame]: (f = { n: 0, z: new Uint8Array(64), got: [] }) }; }
    if (f.got[part]) return;
    f.got[part] = true; f.n++;
    for (let k = 0; k < 16; k++) f.z[part * 16 + k] = dv.getUint8(4 + k);
    if (f.n === 4) {
      S.tof = f.z; delete S.tofParts[frame];
      if ($("mapping").checked) loc.frame(S.tof, S.x, S.y, S.th);
      if (REC.on && $("recTof").checked) { REC.tof.push([+S.t.toFixed(2), +S.x.toFixed(1), +S.y.toFixed(1), +S.th.toFixed(1), ...S.tof]); recView.addTof(REC.tof[REC.tof.length - 1]); }
      ui.tofDirty = true; ui.mapDirty = true;
    }
  }

  function onParam(dv) {
    const id = dv.getUint8(1), n = dv.getUint8(2), v = dv.getFloat32(4, true);
    let name = "";
    for (let k = 8; k < Math.min(20, dv.byteLength); k++) { const c = dv.getUint8(k); if (!c) break; name += String.fromCharCode(c); }
    const isNew = !S.params[id];
    S.params[id] = { id, name, value: v };
    S.params.length = Math.max(S.params.length, n);
    params.render(id, isNew);
    if (name === "GyroZupt") $("locZupt").checked = v >= 0.5;
  }

  // ============================================================================
  // Occupancy grid (log odds), square around the origin: 12 m ... 48 m with 5 cm cells, 160 m with 10 cm cells
  // ============================================================================
  class OccGrid {
    // settings(): { rows: [ToF rows used], maxR: range limit in cm, size: cells per side, res: cell size in cm }
    constructor(settings) {
      this.settings = settings;
      this.alloc(settings().size || 240, settings().res || 5);
    }
    alloc(n, res = 5) {
      this.n = n; this.res = res; this.L = new Float32Array(n * n);
      this.off = document.createElement("canvas"); this.off.width = this.off.height = n;
      this.img = this.off.getContext("2d").createImageData(n, n);
      this.invalidate();
    }
    // the image is repainted only where cells changed (box), completely after clear or a theme change
    invalidate() { this.full = true; this.box = null; this.dirty = true; }
    clear() { const { size = 240, res = 5 } = this.settings(); if (size !== this.n || res !== this.res) this.alloc(size, res); else { this.L.fill(0); this.invalidate(); } }
    cell(x, y) { return [Math.floor(x / this.res) + this.n / 2, Math.floor(y / this.res) + this.n / 2]; }
    grow(i, j) {
      const b = this.box || (this.box = [i, j, i, j]);
      if (i < b[0]) b[0] = i; if (j < b[1]) b[1] = j; if (i > b[2]) b[2] = i; if (j > b[3]) b[3] = j;
    }
    add(i, j, d) { if (i < 0 || j < 0 || i >= this.n || j >= this.n) return; const k = j * this.n + i; this.L[k] = Math.max(-4, Math.min(4, this.L[k] + d)); }
    // rays of a ToF image (row major, cm, 255 = invalid) at pose x, y (cm), theta (deg, + = right):
    // sensor position, end point (at most the range) and whether a wall was hit
    rays(z, x, y, th) {
      const { rows, maxR } = this.settings(), out = [];
      const psi = -th * Math.PI / 180, sx = x + TOF_OFFSET_CM * Math.cos(psi), sy = y + TOF_OFFSET_CM * Math.sin(psi);
      for (let c = 1; c <= 8; c++) {
        let d = 255;
        for (const r of rows) { const v = z[(r - 1) * 8 + (c - 1)]; if (v !== 255) d = Math.min(d, v); }
        const len = Math.min(d === 255 ? maxR : d, maxR), a = psi + (4.5 - c) * TOF_FOV_DEG / 8 * Math.PI / 180;
        out.push({ sx, sy, ex: sx + len * Math.cos(a), ey: sy + len * Math.sin(a), hit: d < maxR });
      }
      return out;
    }
    integrate(z, x, y, th) {
      for (const r of this.rays(z, x, y, th)) {
        const [i0, j0] = this.cell(r.sx, r.sy), [i1, j1] = this.cell(r.ex, r.ey);
        this.line(i0, j0, i1, j1, (i, j) => this.add(i, j, -0.35));
        if (r.hit) this.add(i1, j1, 1.2);
        this.grow(i0, j0); this.grow(i1, j1);
      }
      this.dirty = true;
    }
    line(x0, y0, x1, y1, f) {    // Bresenham without the end cell
      const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
      let err = dx + dy;
      while (!(x0 === x1 && y0 === y1)) {
        f(x0, y0);
        const e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
      }
    }
    // image of the grid, repainted only after changes
    image() {
      if (!this.dirty) return this.off;
      const hex = (h) => { const v = parseInt(h.replace("#", ""), 16); return [(v >> 16) & 255, (v >> 8) & 255, v & 255]; };
      const free = hex(css("--map-free")), occ = hex(css("--map-occ")), unk = hex(css("--map-unk")), d = this.img.data, n = this.n;
      const b = this.full ? [0, 0, n - 1, n - 1] : this.box;
      if (b) {
        const i0 = Math.max(0, b[0]), j0 = Math.max(0, b[1]), i1 = Math.min(n - 1, b[2]), j1 = Math.min(n - 1, b[3]);
        for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
          const l = this.L[j * n + i], o = ((n - 1 - j) * n + i) * 4;    // image row 0 = max y
          let c;
          if (Math.abs(l) < 0.05) c = unk;
          else { const p = 1 / (1 + Math.exp(-l)); const a = l > 0 ? occ : free, w = Math.abs(p - 0.5) * 2; c = [0, 1, 2].map((k) => unk[k] + (a[k] - unk[k]) * w); }
          d[o] = c[0]; d[o + 1] = c[1]; d[o + 2] = c[2]; d[o + 3] = 255;
        }
        if (i1 >= i0 && j1 >= j0) this.off.getContext("2d").putImageData(this.img, 0, 0, i0, n - 1 - j1, i1 - i0 + 1, j1 - j0 + 1);
      }
      this.full = false; this.box = null; this.dirty = false;
      return this.off;
    }
    csv() {
      const n = this.n, lines = [`# occupancy probability, cell ${this.res} cm, x from ${-n / 2 * this.res} cm (column 1), y from ${(n / 2 - 1) * this.res} cm (row 1) downwards; empty = unknown`];
      for (let j = n - 1; j >= 0; j--) {
        const row = [];
        for (let i = 0; i < n; i++) { const l = this.L[j * n + i]; row.push(Math.abs(l) < 0.05 ? "" : (1 / (1 + Math.exp(-l))).toFixed(2)); }
        lines.push(row.join(","));
      }
      return lines.join("\n");
    }
  }
  // map size "cells" or "cells:cell size in cm"
  const mapSettings = () => {
    const [size, res = 5] = $("mapSize").value.split(":").map(Number);
    return { rows: $("mapRows").value.split(",").map(Number), maxR: Number($("mapRange").value), size, res };
  };
  const map = new OccGrid(mapSettings);

  // ============================================================================
  // Localization against the map (card "Lokalisierung", each stage optional):
  //  1 gyro offset at standstill: on the robot (parameter GyroZupt)
  //  2 scan matching: several ToF images are collected and fitted to the map
  //  3 rectangular rooms: the dominant wall direction is kept at multiples of 90 deg
  // corr maps the odometry frame of the robot into the map frame (= odometry
  // frame at the last reset): p_map = R(corr.psi) p_odo + (corr.x, corr.y).
  // The map, the path and the waypoints lie in the map frame.
  // ============================================================================
  const LOC = {
    BATCH_MIN: 4, BATCH_MAX: 12, BATCH_MOVE_CM: 10, BATCH_TURN_DEG: 15,   // images per matching
    SEARCH_CM: 15, SEARCH_DEG: 6, MIN_POINTS: 10, MIN_KNOWN: 0.35, MIN_GAIN: 1.08,
  MATCH_GAIN: 0.5, MATCH_MAX_CM: 5, MATCH_MAX_DEG: 2,                    // damped: single wrong matches do little harm
    RECT_GAIN: 0.3, RECT_MAX_DEG: 10, RECT_MIN_R: 0.5, RECT_WALLS: 40,
  };
  const wrap45 = (a) => ((a + Math.PI / 4) % (Math.PI / 2) + Math.PI / 2) % (Math.PI / 2) - Math.PI / 4;
  class Localizer {
    constructor(grid) { this.grid = grid; this.reset(); }
    reset() { this.corr = { x: 0, y: 0, psi: 0 }; this.sentCorr = null; this.clearMap(); this.stats = { shift: 0, fit: 0, skip: 0 }; }
    clearMap() { this.buf = []; this.walls = []; this.rect = null; }
    get matching() { return $("locMatch").checked; }
    get rectangular() { return $("locRect").checked; }
    get active() { return this.matching || this.rectangular; }
    // odometry pose (cm, deg + = right) -> map pose
    toMap(x, y, th) {
      const c = Math.cos(this.corr.psi), s = Math.sin(this.corr.psi);
      return { x: c * x - s * y + this.corr.x, y: s * x + c * y + this.corr.y, th: wrap180(th - this.corr.psi * 180 / Math.PI) };
    }
    toOdo(x, y) {
      const c = Math.cos(this.corr.psi), s = Math.sin(this.corr.psi), dx = x - this.corr.x, dy = y - this.corr.y;
      return { x: c * dx + s * dy, y: -s * dx + c * dy };
    }
    // correction: rotate the map pose by dpsi about (px, py), then shift by (dx, dy)
    shift(dx, dy, dpsi, px, py) {
      const c = Math.cos(dpsi), s = Math.sin(dpsi), tx = this.corr.x - px, ty = this.corr.y - py;
      this.corr = { x: c * tx - s * ty + px + dx, y: s * tx + c * ty + py + dy, psi: this.corr.psi + dpsi };
      for (const w of this.walls) w[0] += dpsi;
      if (this.onShift) this.onShift();
    }
    // new ToF image at the odometry pose
    frame(z, x, y, th) {
      if (!this.matching) {
        this.flush();
        if (this.rectangular) this.rectUpdate([{ z, x, y, th }]);
        const m = this.toMap(x, y, th); this.grid.integrate(z, m.x, m.y, m.th);
        return;
      }
      this.buf.push({ z, x, y, th });
      const a = this.buf[0], n = this.buf.length;
      const moved = Math.hypot(x - a.x, y - a.y), turned = Math.abs(wrap180(th - a.th));
      if (n >= LOC.BATCH_MAX || (n >= LOC.BATCH_MIN && (moved >= LOC.BATCH_MOVE_CM || turned >= LOC.BATCH_TURN_DEG))) this.flush();
    }
    // collected images: correct the pose, then enter them into the map
    flush() {
      const fr = this.buf;
      if (!fr.length) return;
      this.buf = [];
      if (this.rectangular) this.rectUpdate(fr);
      if (this.matching) this.match(fr);
      for (const f of fr) { const m = this.toMap(f.x, f.y, f.th); this.grid.integrate(f.z, m.x, m.y, m.th); }
    }
    // stage 2: search the shift (dx, dy, dpsi about the robot) that puts the hits onto the walls of the map
    match(fr) {
      const pts = [];
      for (const f of fr) { const m = this.toMap(f.x, f.y, f.th); for (const r of this.grid.rays(f.z, m.x, m.y, m.th)) if (r.hit) pts.push([r.ex, r.ey]); }
      const last = fr[fr.length - 1], pc = this.toMap(last.x, last.y, last.th);
      if (pts.length < LOC.MIN_POINTS) { this.stats.skip++; return; }
      const g = this.grid, n = g.n, L = g.L, res = g.res;
      const idx = (x, y) => { const i = Math.floor(x / res) + n / 2, j = Math.floor(y / res) + n / 2; return i < 3 || j < 3 || i >= n - 3 || j >= n - 3 ? -1 : j * n + i; };
      // cell value: occupancy or half of the best neighbour (wider capture range), known free space negative;
      // interpolated between the cell centres, so that the score changes smoothly with the shift
      const cellVal = (k) => { const v = Math.max(L[k], 0.5 * Math.max(L[k - 1], L[k + 1], L[k - n], L[k + n])); return v > 0 ? v : 0.5 * L[k]; };
      const val = (x, y) => {
        const fx = x / res + n / 2 - 0.5, fy = y / res + n / 2 - 0.5, i = Math.floor(fx), j = Math.floor(fy);
        if (i < 3 || j < 3 || i >= n - 4 || j >= n - 4) return 0;
        const ax = fx - i, ay = fy - j, k = j * n + i;
        return (1 - ay) * ((1 - ax) * cellVal(k) + ax * cellVal(k + 1)) + ay * ((1 - ax) * cellVal(k + n) + ax * cellVal(k + n + 1));
      };
      // only where the map already has walls nearby (otherwise: new area, nothing to match)
      let known = 0;
      for (const [x, y] of pts) {
        const k = idx(x, y); if (k < 0) continue;
        search: for (let dj = -3; dj <= 3; dj++) for (let di = -3; di <= 3; di++) if (L[k + dj * n + di] > 1) { known++; break search; }
      }
      if (known < LOC.MIN_KNOWN * pts.length) { this.stats.skip++; return; }
      const score = (dx, dy, dp) => {
        const c = Math.cos(dp), s = Math.sin(dp); let sum = 0;
        for (const [x, y] of pts) { const rx = x - pc.x, ry = y - pc.y; sum += val(c * rx - s * ry + pc.x + dx, s * rx + c * ry + pc.y + dy); }
        // small preference for small corrections (corridors are ambiguous along the walls)
        return sum - 0.02 * pts.length * ((dx * dx + dy * dy) / LOC.SEARCH_CM ** 2 + (dp * 180 / Math.PI / LOC.SEARCH_DEG) ** 2);
      };
      const s0 = score(0, 0, 0), rad = Math.PI / 180;
      let best = [0, 0, 0, s0];
      const grid = (cx, cy, cp, rc, sc, rp, sp) => {
        for (let p = -rp; p <= rp + 1e-9; p += sp) for (let dx = -rc; dx <= rc + 1e-9; dx += sc) for (let dy = -rc; dy <= rc + 1e-9; dy += sc) {
          const v = score(cx + dx, cy + dy, cp + p * rad);
          if (v > best[3]) best = [cx + dx, cy + dy, cp + p * rad, v];
        }
      };
      grid(0, 0, 0, LOC.SEARCH_CM, 3, LOC.SEARCH_DEG, 1.5);            // coarse
      grid(best[0], best[1], best[2], 3, 1, 1.5, 0.5);                    // fine around the best
      if (best[3] < s0 * LOC.MIN_GAIN + 0.05 * pts.length) { this.stats.fit++; return; }   // fits already (or no clear improvement)
      this.stats.shift++;
      const lim = (v, m) => Math.max(-m, Math.min(m, LOC.MATCH_GAIN * v));
    this.shift(lim(best[0], LOC.MATCH_MAX_CM), lim(best[1], LOC.MATCH_MAX_CM), lim(best[2], LOC.MATCH_MAX_DEG * rad), pc.x, pc.y);
    }
    // stage 3: straight wall pieces (runs of neighbouring hits, line fit) -> dominant direction modulo 90 deg
    rectUpdate(fr) {
      let last = null;
      for (const f of fr) {
        const m = this.toMap(f.x, f.y, f.th), rs = this.grid.rays(f.z, m.x, m.y, m.th);
        let run = [];
        const fit = () => {
          if (run.length >= 3) {
            const k = run.length, mx = run.reduce((a, p) => a + p[0], 0) / k, my = run.reduce((a, p) => a + p[1], 0) / k;
            let sxx = 0, syy = 0, sxy = 0;
            for (const [x, y] of run) { sxx += (x - mx) ** 2; syy += (y - my) ** 2; sxy += (x - mx) * (y - my); }
            sxx /= k; syy /= k; sxy /= k;
            const minor = (sxx + syy) / 2 - Math.sqrt(((sxx - syy) / 2) ** 2 + sxy * sxy);
            const span = Math.hypot(run[k - 1][0] - run[0][0], run[k - 1][1] - run[0][1]);
            if (span >= 10 && Math.sqrt(Math.max(0, minor)) < 1.5) this.walls.push([0.5 * Math.atan2(2 * sxy, sxx - syy), span]);
          }
          run = [];
        };
        for (const r of rs) {
          const q = run[run.length - 1];
          if (r.hit && (!q || Math.hypot(r.ex - q[0], r.ey - q[1]) < 25)) run.push([r.ex, r.ey]);
          else { fit(); if (r.hit) run.push([r.ex, r.ey]); }
        }
        fit();
        last = m;
      }
      if (this.walls.length > LOC.RECT_WALLS) this.walls.splice(0, this.walls.length - LOC.RECT_WALLS);
      if (this.walls.length < 8) return;
      let C = 0, S = 0, W = 0;
      for (const [a, w] of this.walls) { C += w * Math.cos(4 * a); S += w * Math.sin(4 * a); W += w; }
      if (Math.hypot(C, S) / W < LOC.RECT_MIN_R) return;              // no clear direction (round or cluttered)
      const dir = Math.atan2(S, C) / 4;
      if (this.rect === null) { this.rect = dir; return; }             // reference: the first clear direction
      const e = wrap45(dir - this.rect);
      if (Math.abs(e) > LOC.RECT_MAX_DEG * Math.PI / 180) return;
      this.shift(0, 0, -LOC.RECT_GAIN * e, last.x, last.y);
    }
    statsText() {
      return `${this.stats.shift} korrigiert, ${this.stats.fit} passend, ${this.stats.skip} ohne Bezug` + (this.rect !== null ? `, Wandrichtung ${(this.rect * 180 / Math.PI).toFixed(1)}°` : "");
    }
  }
  const loc = new Localizer(map);
  // the waypoints lie in the map frame: after a correction the robot gets the open ones again in its odometry frame
  loc.onShift = () => {
    if (!S.link || !mission.active || mission.paused || !mission.acked || !loc.sentCorr || performance.now() - loc.sentAt < 3000) return;
    const a = loc.sentCorr, b = loc.corr, ca = Math.cos(a.psi), sa = Math.sin(a.psi), cb = Math.cos(b.psi), sb = Math.sin(b.psi);
    const moved = Math.hypot(ca * S.x - sa * S.y + a.x - (cb * S.x - sb * S.y + b.x), sa * S.x + ca * S.y + a.y - (sb * S.x + cb * S.y + b.y));
    if (moved >= 3 || Math.abs(b.psi - a.psi) >= 1.5 * Math.PI / 180) sendWaypoints(true);
  };
  function locUi() {
    const c = loc.corr, m = loc.toMap(S.x, S.y, S.th);
    $("locCorr").textContent = loc.active || c.psi || c.x ? `${c.x.toFixed(1)} / ${c.y.toFixed(1)} cm, ${(c.psi * 180 / Math.PI).toFixed(1)}°` : "–";
    $("locStats").textContent = loc.active ? loc.statsText() : "aus";
    $("locPose").textContent = S.connected ? `x ${m.x.toFixed(1)} cm, y ${m.y.toFixed(1)} cm, θ ${m.th.toFixed(1)}°` : "–";
  }

  // ============================================================================
  // World views (control map, simulation, replay): pan, zoom, drawing
  // ============================================================================
  class WorldView {
    // onPress(x, y): optional, returns {move(x, y), end()} to take over a drag (e.g. setting the start pose)
    constructor(canvas, { onTap, onUserMove, onView, onPress } = {}) {
      this.c = canvas; this.scale = 2; this.cx = 0; this.cy = 0;    // px per cm, view centre in cm
      this.m = { l: 40, r: 8, t: 8, b: 24 };                         // margins for the axes (MobileRobotSim)
      const pts = new Map(); let moved = false, pinch = null, press = null;
      canvas.addEventListener("pointerdown", (e) => {
        try { canvas.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
        if (onPress && !pts.size) { press = onPress(...this.s2w(e.offsetX, e.offsetY)); if (press) return; }
        pts.set(e.pointerId, [e.offsetX, e.offsetY]); moved = false; pinch = null;
      });
      canvas.addEventListener("pointermove", (e) => {
        if (press) { press.move(...this.s2w(e.offsetX, e.offsetY)); this.dirty = true; return; }
        if (!pts.has(e.pointerId)) return;
        const prev = pts.get(e.pointerId); pts.set(e.pointerId, [e.offsetX, e.offsetY]);
        if (pts.size === 2) {
          const [a, b] = [...pts.values()], d = Math.hypot(a[0] - b[0], a[1] - b[1]);
          if (pinch) this.zoom(d / pinch);
          pinch = d; moved = true; onUserMove && onUserMove(); onView && onView();
        } else {
          const dx = e.offsetX - prev[0], dy = e.offsetY - prev[1];
          moved = moved || Math.abs(dx) + Math.abs(dy) > 3;
          if (moved) { this.cx -= dx / this.scale; this.cy += dy / this.scale; onUserMove && onUserMove(); onView && onView(); }
        }
        this.dirty = true;
      });
      canvas.addEventListener("pointerup", (e) => {
        if (press) { press.end(); press = null; this.dirty = true; return; }
        if (pts.size === 1 && !moved && onTap) onTap(...this.s2w(e.offsetX, e.offsetY));
        pts.delete(e.pointerId); pinch = null; this.dirty = true;
      });
      canvas.addEventListener("pointercancel", (e) => { pts.delete(e.pointerId); pinch = null; press = null; });
      // mouse wheel: zoom around the mouse pointer
      canvas.addEventListener("wheel", (e) => {
        e.preventDefault();
        const [wx, wy] = this.s2w(e.offsetX, e.offsetY);
        this.zoom(e.deltaY < 0 ? 1.2 : 1 / 1.2);
        const [nx, ny] = this.s2w(e.offsetX, e.offsetY);
        this.cx += wx - nx; this.cy += wy - ny;
        onView && onView();
      }, { passive: false });
      this.dirty = true;
    }
    zoom(f) { this.scale = Math.min(20, Math.max(0.02, this.scale * f)); this.dirty = true; }
    w2s(x, y) { return [this.c.clientWidth / 2 + (x - this.cx) * this.scale, this.c.clientHeight / 2 - (y - this.cy) * this.scale]; }
    s2w(px, py) { return [this.cx + (px - this.c.clientWidth / 2) / this.scale, this.cy - (py - this.c.clientHeight / 2) / this.scale]; }
    // layers: world (GridWorld), walls [[x1,y1,x2,y2]], grid (OccGrid), fov {x, y, psi, range}, rays [[x1,y1,x2,y2]],
    // trails [{pts, color, width, dash}], waypoints {list, cur}, start {x, y, psi}, robots [{x, y, th, color, alpha}]
    // frame {x, y, psi}: optional, layers grid/trails/waypoints/start/robots are given in this frame
    // (odometry) and drawn in world coordinates
    draw(layers) {
      if (layers.frame) layers = this.toWorld(layers);
      const c = this.c, dpr = fitCanvas(c), g = c.getContext("2d");
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      const W = c.clientWidth, H = c.clientHeight;
      g.fillStyle = css(layers.walls || layers.world ? "--map-unk" : "--surface"); g.fillRect(0, 0, W, H);
      if (layers.world) {            // occupancy grid of the true world (e.g. office building)
        const w = layers.world, [gx, gy] = this.w2s(0, w.height);
        g.imageSmoothingEnabled = false;
        g.drawImage(w.image(), gx, gy, w.width * this.scale, w.height * this.scale);
        g.imageSmoothingEnabled = true;
      }
      if (layers.walls) {            // rooms from wall segments: free inside the bounding box
        const b = wallBox(layers.walls);
        if (b) { const [x0, y0] = this.w2s(b[0], b[3]), [x1, y1] = this.w2s(b[2], b[1]); g.fillStyle = css("--map-free"); g.fillRect(x0, y0, x1 - x0, y1 - y0); }
      }
      if (layers.grid) {
        const gr = layers.grid, half = gr.n / 2 * gr.res, f = layers.gridFrame || { x: 0, y: 0, psi: 0 }, [ox, oy] = this.w2s(f.x, f.y);
        g.save(); g.translate(ox, oy); g.rotate(-f.psi);
        g.imageSmoothingEnabled = f.psi !== 0;      // rotated: smooth, otherwise sharp cells
        g.drawImage(gr.image(), -half * this.scale, -half * this.scale, gr.n * gr.res * this.scale, gr.n * gr.res * this.scale);
        g.restore(); g.imageSmoothingEnabled = true;
      }
      this.gridLines(g, W, H);
      if (layers.walls) {
        g.strokeStyle = css("--map-occ"); g.lineWidth = Math.max(2, 3 * this.scale); g.lineCap = "square"; g.beginPath();
        for (const [a, b, cc, d] of layers.walls) { const [sx, sy] = this.w2s(a, b), [ex, ey] = this.w2s(cc, d); g.moveTo(sx, sy); g.lineTo(ex, ey); }
        g.stroke(); g.lineCap = "butt";
      }
      if (layers.fov) {              // ToF field of view
        const f = layers.fov, [sx, sy] = this.w2s(f.x, f.y), r = f.range * this.scale, h = TOF_FOV_DEG / 2 * Math.PI / 180;
        g.fillStyle = css("--fov"); g.beginPath(); g.moveTo(sx, sy); g.arc(sx, sy, r, -f.psi - h, -f.psi + h); g.closePath(); g.fill();
      }
      if (layers.rays) {
        g.strokeStyle = css("--ray"); g.globalAlpha = 0.7; g.lineWidth = 1; g.beginPath();
        for (const r of layers.rays) { const [sx, sy] = this.w2s(r[0], r[1]), [ex, ey] = this.w2s(r[2], r[3]); g.moveTo(sx, sy); g.lineTo(ex, ey); }
        g.stroke(); g.globalAlpha = 1;
      }
      for (const t of layers.trails || []) {
        if (t.pts.length < 2) continue;
        g.strokeStyle = css(t.color); g.lineWidth = t.width || 2.2; g.setLineDash(t.dash || []); g.lineJoin = "round"; g.lineCap = "round"; g.beginPath();
        t.pts.forEach(([x, y], i) => { const [sx, sy] = this.w2s(x, y); i ? g.lineTo(sx, sy) : g.moveTo(sx, sy); });
        g.stroke(); g.setLineDash([]); g.lineCap = "butt";
      }
      if (layers.start) this.startMarker(g, layers.start);
      if (layers.waypoints && layers.waypoints.list.length) this.drawWaypoints(g, layers.waypoints, layers.robots && layers.robots[layers.robots.length - 1]);
      for (const r of layers.robots || []) this.robot(g, r);
      this.axes(g, W, H);
      this.dirty = false;
    }
    // layers in the frame {x, y, psi} -> world coordinates (heading th in deg, + = clockwise)
    toWorld(layers) {
      const f = layers.frame, c = Math.cos(f.psi), s = Math.sin(f.psi), deg = f.psi * 180 / Math.PI;
      const P = (x, y) => [f.x + c * x - s * y, f.y + s * x + c * y];
      const out = { ...layers, frame: null, gridFrame: f };
      if (layers.trails) out.trails = layers.trails.map((t) => ({ ...t, pts: t.pts.map(([x, y]) => P(x, y)) }));
      if (layers.waypoints) out.waypoints = { ...layers.waypoints, list: layers.waypoints.list.map((p) => { const [x, y] = P(p.x, p.y); return { ...p, x, y }; }) };
      if (layers.robots) out.robots = layers.robots.map((r) => { const [x, y] = P(r.x, r.y); return { ...r, x, y, th: r.th - deg }; });
      if (layers.start) { const [x, y] = P(layers.start.x, layers.start.y); out.start = { x, y, psi: layers.start.psi + f.psi }; }
      return out;
    }
    // light grid lines at the tick positions
    ticks(W) { return niceStep(Math.max(1, (W - this.m.l - this.m.r) / this.scale / 8)); }
    gridLines(g, W, H) {
      const st = this.ticks(W), [x0, y1] = this.s2w(0, 0), [x1, y0] = this.s2w(W, H);
      g.strokeStyle = css("--grid"); g.lineWidth = 1; g.globalAlpha = 0.7; g.beginPath();
      for (let x = Math.ceil(x0 / st) * st; x <= x1; x += st) { const sx = Math.round(this.w2s(x, 0)[0]) + 0.5; g.moveTo(sx, 0); g.lineTo(sx, H); }
      for (let y = Math.ceil(y0 / st) * st; y <= y1; y += st) { const sy = Math.round(this.w2s(0, y)[1]) + 0.5; g.moveTo(0, sy); g.lineTo(W, sy); }
      g.stroke(); g.globalAlpha = 1;
    }
    // axes in metres on the left and bottom margin, as in MobileRobotSim
    axes(g, W, H) {
      const m = this.m, st = this.ticks(W), [x0, y1] = this.s2w(m.l, m.t), [x1, y0] = this.s2w(W - m.r, H - m.b);
      g.fillStyle = css("--surface"); g.fillRect(0, 0, m.l, H); g.fillRect(0, H - m.b, W, m.b);
      g.strokeStyle = css("--axis"); g.lineWidth = 1; g.beginPath();
      g.moveTo(m.l + 0.5, 0); g.lineTo(m.l + 0.5, H - m.b + 0.5); g.lineTo(W, H - m.b + 0.5); g.stroke();
      g.fillStyle = css("--muted"); g.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
      const dec = st < 10 ? 2 : st < 100 ? 1 : 0, lab = (v) => { const s = (v / 100).toFixed(dec); return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s; };
      g.textAlign = "center"; g.textBaseline = "top";
      for (let x = Math.ceil(x0 / st) * st; x <= x1; x += st) {
        const sx = Math.round(this.w2s(x, 0)[0]) + 0.5;
        if (sx < m.l + 12 || sx > W - 30) continue;
        g.beginPath(); g.moveTo(sx, H - m.b); g.lineTo(sx, H - m.b + 4); g.stroke(); g.fillText(lab(x), sx, H - m.b + 6);
      }
      g.textAlign = "right"; g.textBaseline = "middle";
      for (let y = Math.ceil(y0 / st) * st; y <= y1; y += st) {
        const sy = Math.round(this.w2s(0, y)[1]) + 0.5;
        if (sy > H - m.b - 6 || sy < 18) continue;
        g.beginPath(); g.moveTo(m.l - 4, sy); g.lineTo(m.l, sy); g.stroke(); g.fillText(lab(y), m.l - 6, sy);
      }
      g.textAlign = "right"; g.textBaseline = "bottom"; g.fillText("x [m]", W - 2, H - 1);
      g.save(); g.translate(11, 4); g.rotate(-Math.PI / 2); g.textAlign = "right"; g.textBaseline = "middle"; g.fillText("y [m]", 0, 0); g.restore();
      g.textAlign = "start"; g.textBaseline = "alphabetic";
    }
    // two-wheel robot (differential drive) as in MobileRobotSim, at least 9 px radius
    robot(g, r) {
      const [x, y] = this.w2s(r.x, r.y), rr = Math.max(9, (TRACK_CM / 2 + 1) * this.scale), psi = -r.th * Math.PI / 180;
      g.save(); g.globalAlpha = r.alpha ?? 1; g.translate(x, y); g.rotate(-psi);
      g.fillStyle = css("--map-occ"); const ww = rr * 0.85, wh = rr * 0.32;
      g.fillRect(-ww / 2, -rr - wh / 2, ww, wh); g.fillRect(-ww / 2, rr - wh / 2, ww, wh);
      g.fillStyle = css(r.color); g.strokeStyle = "#ffffff"; g.lineWidth = 1.5;
      g.beginPath(); g.arc(0, 0, rr * 0.92, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.fillStyle = "#ffffff"; g.beginPath(); g.moveTo(rr * 0.75, 0); g.lineTo(-rr * 0.25, -rr * 0.42); g.lineTo(-rr * 0.25, rr * 0.42); g.closePath(); g.fill();
      g.restore();
    }
    // start pose: ring with direction arrow (MobileRobotSim)
    startMarker(g, p) {
      const [x, y] = this.w2s(p.x, p.y), L = 20, hx = x + L * Math.cos(p.psi), hy = y - L * Math.sin(p.psi), col = css("--ink");
      g.save(); g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 2.5;
      g.beginPath(); g.moveTo(x, y); g.lineTo(hx, hy); g.stroke();
      g.beginPath(); g.moveTo(hx, hy); g.lineTo(hx - 7 * Math.cos(p.psi - 0.5), hy + 7 * Math.sin(p.psi - 0.5)); g.lineTo(hx - 7 * Math.cos(p.psi + 0.5), hy + 7 * Math.sin(p.psi + 0.5)); g.closePath(); g.fill();
      g.fillStyle = css("--surface"); g.beginPath(); g.arc(x, y, 7, 0, 2 * Math.PI); g.fill(); g.stroke();
      g.restore();
    }
    // waypoints: reached ones grey, the current target ringed and connected to the robot
    drawWaypoints(g, { list, cur }, robot) {
      const wc = css("--map-wp"), gray = css("--muted"), white = css("--surface");
      g.setLineDash([6, 5]); g.lineWidth = 1.5;
      for (let i = 1; i < list.length; i++) {
        const [ax, ay] = this.w2s(list[i - 1].x, list[i - 1].y), [bx, by] = this.w2s(list[i].x, list[i].y);
        g.strokeStyle = list[i].reached ? gray : wc; g.globalAlpha = list[i].reached ? 0.5 : 1;
        g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke();
      }
      if (cur >= 0 && robot && list[cur]) {
        const [bx, by] = this.w2s(list[cur].x, list[cur].y), [ax, ay] = this.w2s(robot.x, robot.y);
        g.strokeStyle = wc; g.globalAlpha = 0.8; g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke();
      }
      g.setLineDash([]); g.globalAlpha = 1;
      list.forEach((p, i) => {
        const [sx, sy] = this.w2s(p.x, p.y), isCur = i === cur;
        g.globalAlpha = p.reached ? 0.45 : 1;
        g.fillStyle = p.reached ? gray : wc; g.strokeStyle = white; g.lineWidth = 2;
        g.beginPath(); g.arc(sx, sy, 6, 0, 2 * Math.PI); g.fill(); g.stroke();
        if (isCur) { g.strokeStyle = wc; g.lineWidth = 2; g.beginPath(); g.arc(sx, sy, 11, 0, 2 * Math.PI); g.stroke(); }
        g.fillStyle = p.reached ? gray : css("--ink"); g.font = "600 12px system-ui"; g.fillText(String(i + 1), sx + 9, sy - 8);
        g.globalAlpha = 1;
      });
    }
  }
  // bounding box of wall segments (cached per room)
  const wallBoxes = new WeakMap();
  function wallBox(walls) {
    if (!walls.length) return null;
    if (!wallBoxes.has(walls)) {
      const b = [Infinity, Infinity, -Infinity, -Infinity];
      for (const [x1, y1, x2, y2] of walls) { b[0] = Math.min(b[0], x1, x2); b[1] = Math.min(b[1], y1, y2); b[2] = Math.max(b[2], x1, x2); b[3] = Math.max(b[3], y1, y2); }
      wallBoxes.set(walls, b);
    }
    return wallBoxes.get(walls);
  }

  // control map
  const wps = [];
  const mapCanvas = $("mapCanvas");
  const view = new WorldView(mapCanvas, {
    onUserMove: () => { $("follow").checked = false; ui.mapDirty = true; },
    onTap: (x, y) => {
      if (!$("wpEdit").checked) return;
      if (wps.length >= WP_MAX) toast(`Höchstens ${WP_MAX} Wegpunkte`);
      else { wps.push({ x: Math.round(x), y: Math.round(y), reached: false }); renderWps(); }
    },
  });
  function drawMap() {
    const rp = loc.toMap(S.x, S.y, S.th);
    if ($("follow").checked && S.connected) { view.cx = rp.x; view.cy = rp.y; }
    view.draw({
      grid: map,
      trails: [{ pts: S.trail, color: "--map-trail" }],
      waypoints: { list: wps, cur: curWp() },
      robots: [{ ...rp, color: "--robot" }],
    });
  }

  // Waypoint drive: the robot gets the not yet reached waypoints wps[base ...];
  // its wpIndex counts within this part. Pause = mode 0, "Weiter" sends the
  // remaining waypoints again (the robot starts from its current position).
  const mission = { active: false, started: false, acked: false, paused: false, base: 0, count: 0 };

  function curWp() {
    return mission.active && mission.started && !mission.paused && S.mode === 4 && (S.wpState === 1 || S.wpState === 3)
      ? mission.base + S.wpIdx : -1;
  }

  // progress from the telemetry (only after the robot has taken over the new list)
  function updateMission() {
    if (!mission.active || mission.paused || !mission.acked || S.mode !== 4) return;
    if (S.wpState === 1 || S.wpState === 3) {
      mission.started = true;
      for (let k = 0; k < Math.min(S.wpIdx, mission.count); k++) if (wps[mission.base + k]) wps[mission.base + k].reached = true;
    } else if (S.wpState === 2 && mission.started) {
      for (let k = 0; k < mission.count; k++) if (wps[mission.base + k]) wps[mission.base + k].reached = true;
      mission.active = false;
      toast("Alle Wegpunkte erreicht");
    }
  }

  function missionText() {
    if (!S.connected) return ["nicht verbunden", ""];
    if (mission.paused) return [`Pause – ${wps.filter((p) => !p.reached).length} Wegpunkte offen`, "warn"];
    if (S.mode !== 4) return [`Modus: ${MODE_NAMES[S.mode] ?? S.mode}`, ""];
    const n = wps.length, cur = curWp();
    if (S.wpState === 3) return [`Blockiert: Hindernis ${Math.round(S.front / 10)} cm vor dem Roboter (Ziel ${cur + 1} von ${n})`, "warn"];
    if (S.wpState === 1) return [`Fährt zu Wegpunkt ${cur + 1} von ${n}`, "ok"];
    if (S.wpState === 2) return ["Fertig: alle Wegpunkte erreicht", "ok"];
    return ["Wegpunkte: keine Liste – Punkte setzen und „Senden & starten“", ""];
  }

  function renderWps() {
    const ul = $("wpList"); ul.innerHTML = "";
    if (!wps.length) { ul.innerHTML = '<li class="muted">Keine Wegpunkte. Schalter oben aktivieren und auf die Karte tippen.</li>'; }
    const cur = curWp();
    wps.forEach((p, i) => {
      const li = document.createElement("li");
      li.className = p.reached ? "done" : (i === cur ? "cur" : "");
      li.innerHTML = `<span>${i + 1}: x = ${Math.round(p.x)} cm, y = ${Math.round(p.y)} cm${p.reached ? " ✓" : ""}</span>`;
      const b = document.createElement("button"); b.textContent = "✕"; b.title = "Entfernen";
      b.onclick = () => {
        wps.splice(i, 1);
        if (mission.active && i < mission.base + mission.count) { mission.active = false; mission.paused = false; toast("Liste geändert – zum Weiterfahren neu senden"); }
        renderWps();
      };
      li.appendChild(b); ul.appendChild(li);
    });
    const pb = $("wpPause");
    pb.disabled = !S.link || !(mission.active || mission.paused);
    pb.textContent = mission.paused ? "Weiter" : "Pause";
    ui.mapDirty = true;
  }

  async function sendWaypoints(quiet = false) {
    if (!S.link) return toast("Nicht verbunden");
    if (!wps.length) return toast("Keine Wegpunkte gesetzt");
    if (wps.every((p) => p.reached)) wps.forEach((p) => (p.reached = false));    // all done: start again
    const base = wps.findIndex((p) => !p.reached), part = wps.slice(base);
    Object.assign(mission, { active: true, started: false, acked: false, paused: false, base, count: part.length });
    renderWps();
    await S.link.write(cmd.wpClear());
    const odo = part.map((p) => loc.toOdo(p.x, p.y));     // the robot drives in its odometry frame
    loc.sentCorr = { ...loc.corr }; loc.sentAt = performance.now();
    for (let i = 0; i < odo.length; i += 4) await S.link.write(cmd.wpSet(i, odo.slice(i, i + 4)));
    await S.link.write(cmd.wpCommit(part.length));
    requestMode(4);
    if (!quiet) toast(base ? `Weiter mit Wegpunkt ${base + 1} (${part.length} offen)` : `${part.length} Wegpunkte gesendet, Fahrt startet`);
  }

  function pauseOrResume() {
    if (!S.link) return;
    if (mission.paused) { sendWaypoints(); return; }
    requestMode(0);
    mission.paused = true;
    renderWps(); ui.dirty = true;
  }

  // ============================================================================
  // Charts (last 30 s)
  // ============================================================================
  class Chart {
    constructor(canvas, legend, series, opts = {}) {
      this.c = canvas; this.series = series; this.opts = opts; this.data = series.map(() => []);
      legend.innerHTML = series.map((s) => `<span><i class="sw line${s.dash ? " dashed" : ""}" style="--c:var(${s.color})"></i>${s.label}</span>`).join("");
    }
    push(t, values) { values.forEach((v, i) => { if (!Number.isFinite(v)) return; const d = this.data[i]; d.push([t, v]); while (!this.opts.full && d.length && d[0][0] < t - 35) d.shift(); }); }
    clear() { this.data = this.series.map(() => []); }
    draw() {
      const dpr = fitCanvas(this.c), g = this.c.getContext("2d");
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      const W = this.c.clientWidth, H = this.c.clientHeight, padL = 44, padB = 18, padT = 6, padR = 8;
      g.clearRect(0, 0, W, H);
      const all = this.data.flat();
      if (!all.length) { g.fillStyle = css("--muted"); g.font = "13px system-ui"; g.fillText("keine Daten", padL, H / 2); return; }
      // full: whole recording (time from its start), otherwise the last 30 s
      let tMax = -Infinity, tMin = Infinity;
      for (const p of all) { if (p[0] > tMax) tMax = p[0]; if (p[0] < tMin) tMin = p[0]; }
      if (!this.opts.full) tMin = tMax - 30;
      const span = Math.max(1e-6, tMax - tMin);
      let lo = Infinity, hi = -Infinity;
      for (const p of all) { if (p[1] < lo) lo = p[1]; if (p[1] > hi) hi = p[1]; }
      if (this.opts.min !== undefined) lo = Math.min(lo, this.opts.min);
      if (this.opts.max !== undefined) hi = Math.max(hi, this.opts.max);
      if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
      const m = (hi - lo) * 0.08; lo -= m; hi += m;
      const X = (t) => padL + (t - tMin) / span * (W - padL - padR), Y = (v) => padT + (hi - v) / (hi - lo) * (H - padT - padB);
      g.strokeStyle = css("--line"); g.fillStyle = css("--muted"); g.font = "11px system-ui"; g.lineWidth = 1;
      const step = niceStep((hi - lo) / 4);
      for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
        const y = Y(v); g.beginPath(); g.moveTo(padL, y); g.lineTo(W - padR, y); g.stroke(); g.fillText(fmt(v), 2, y + 4);
      }
      if (lo < 0 && hi > 0) { g.strokeStyle = css("--muted"); g.beginPath(); g.moveTo(padL, Y(0)); g.lineTo(W - padR, Y(0)); g.stroke(); }
      g.fillStyle = css("--muted");
      g.save(); g.beginPath(); g.rect(padL, 0, W - padL - padR, H - padB); g.clip();
      this.series.forEach((s, i) => {
        const d = this.data[i]; if (d.length < 2) return;
        g.strokeStyle = css(s.color); g.lineWidth = s.dash ? 1.5 : 2; g.setLineDash(s.dash ? [5, 4] : []);
        g.beginPath(); d.forEach(([t, v], k) => (k ? g.lineTo(X(t), Y(v)) : g.moveTo(X(t), Y(v)))); g.stroke();
      });
      g.restore(); g.setLineDash([]);
    }
  }
  function niceStep(x) { const p = Math.pow(10, Math.floor(Math.log10(x))), f = x / p; return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p; }
  function fmt(v) { return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(0) : v.toFixed(1); }

  const charts = {
    list: [],
    init() {
      this.wheels = new Chart($("chWheels"), $("lgWheels"), [
        { label: "links Soll", color: "--s1", dash: true }, { label: "links Ist", color: "--s1" },
        { label: "rechts Soll", color: "--s2", dash: true }, { label: "rechts Ist", color: "--s2" }]);
      this.pos = new Chart($("chPos"), $("lgPos"), [{ label: "x", color: "--s1" }, { label: "y", color: "--s3" }]);
      this.heading = new Chart($("chTheta"), $("lgTheta"), [{ label: "θ Pose (verwendet)", color: "--s4" },
        { label: "θ nur Encoder", color: "--s2", dash: true }, { label: "θ Kompass", color: "--s3", dash: true }]);
      this.front = new Chart($("chFront"), $("lgFront"), [{ label: "Abstand", color: "--s2" }], { min: 0 });
      this.list = [this.wheels, this.pos, this.heading, this.front];
    },
    push(t) {
      this.wheels.push(t, [S.cmdL, S.measL, S.cmdR, S.measR]);
      this.pos.push(t, [S.x, S.y]); this.front.push(t, [Math.min(S.front, 2000)]);
      if (!S.hasHeading) this.heading.push(t, [S.th, NaN, NaN]);      // firmware without heading packet
    },
    clear() { this.list.forEach((c) => c.clear()); },
    draw() { this.list.forEach((c) => c.draw()); },
  };

  // ============================================================================
  // Trajectories below the maps (tabs "Steuerung" and "Simulation"), as in MobileRobotSim:
  // title and unit in the plot, time window, crosshair with values
  // ============================================================================
  class TimePlot {
    // opts: { title, unit, series: [{ key, label, color, dash, simOnly }] }
    constructor(canvas, opts) {
      this.c = canvas; this.o = opts; this.hoverX = null;
      canvas.addEventListener("mousemove", (e) => { this.hoverX = e.offsetX; this.draw(); });
      canvas.addEventListener("mouseleave", () => { this.hoverX = null; this.draw(); });
    }
    draw() {
      if (!this.c.offsetParent) return;                    // hidden
      const T = traj.T, t = T.t, n = t.length, o = this.o, win = traj.window();
      const dpr = fitCanvas(this.c), g = this.c.getContext("2d"), W = this.c.clientWidth, H = this.c.clientHeight;
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.fillStyle = css("--surface"); g.fillRect(0, 0, W, H);
      const m = { l: 46, r: 10, t: 24, b: 22 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
      const font = '11px system-ui, -apple-system, "Segoe UI", sans-serif', ink = css("--ink"), muted = css("--muted");
      g.font = '600 12px system-ui, -apple-system, "Segoe UI", sans-serif'; g.fillStyle = ink; g.textAlign = "left"; g.textBaseline = "middle";
      g.fillText(o.title, m.l, 11);
      g.font = font; g.fillStyle = muted; g.textAlign = "right"; g.fillText(o.unit, W - m.r, 11);
      const series = o.series.filter((s) => !s.simOnly || sim.running);
      const tmax = n ? t[n - 1] : 0, t0 = win > 0 ? Math.max(0, tmax - win) : 0, t1 = win > 0 ? Math.max(t0 + win, tmax) : Math.max(tmax, 1);
      let i0 = 0; { let lo = 0, hi = n - 1; while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] < t0) lo = mid + 1; else hi = mid; } i0 = Math.max(0, lo - 1); }
      let ymin = Infinity, ymax = -Infinity;
      for (const s of series) { const a = T[s.key]; for (let i = i0; i < n; i++) { const v = a[i]; if (v == null || !Number.isFinite(v)) continue; if (v < ymin) ymin = v; if (v > ymax) ymax = v; } }
      if (!Number.isFinite(ymin)) { ymin = -1; ymax = 1; }
      if (ymax - ymin < 1e-6) { ymin -= 0.5; ymax += 0.5; }
      const pad = (ymax - ymin) * 0.08; ymin -= pad; ymax += pad;
      const ys = niceStep((ymax - ymin) / Math.max(2, Math.floor(ph / 28)));
      ymin = Math.floor(ymin / ys) * ys; ymax = Math.ceil(ymax / ys) * ys;
      const X = (tt) => m.l + (tt - t0) / (t1 - t0) * pw, Y = (v) => m.t + ph - (v - ymin) / (ymax - ymin) * ph;
      g.strokeStyle = css("--grid"); g.lineWidth = 1; g.fillStyle = muted; g.textAlign = "right"; g.textBaseline = "middle";
      const dec = ys < 0.1 ? 2 : ys < 1 ? 1 : 0;
      for (let v = ymin; v <= ymax + 1e-9; v += ys) { const py = Math.round(Y(v)) + 0.5; g.beginPath(); g.moveTo(m.l, py); g.lineTo(m.l + pw, py); g.stroke(); g.fillText(v.toFixed(dec), m.l - 6, py); }
      const xs = niceStep((t1 - t0) / Math.max(2, Math.floor(pw / 70)));
      g.textAlign = "center"; g.textBaseline = "top";
      for (let v = Math.ceil(t0 / xs) * xs; v <= t1 + 1e-9; v += xs) {
        const px = Math.round(X(v)) + 0.5;
        g.beginPath(); g.moveTo(px, m.t); g.lineTo(px, m.t + ph); g.stroke();
        if (px < m.l + pw - 22) g.fillText(v.toFixed(xs < 1 ? 1 : 0), px, m.t + ph + 5);
      }
      g.strokeStyle = css("--axis"); g.beginPath(); g.moveTo(m.l, m.t + ph + 0.5); g.lineTo(m.l + pw, m.t + ph + 0.5); g.stroke();
      g.textAlign = "right"; g.textBaseline = "bottom"; g.fillText("t [s]", W - m.r, H - 1);
      // lines (about 2 points per pixel)
      g.save(); g.beginPath(); g.rect(m.l, m.t - 2, pw, ph + 4); g.clip();
      const stride = Math.max(1, Math.floor((n - i0) / (2 * pw)));
      for (const s of series) {
        const a = T[s.key]; g.strokeStyle = css(s.color); g.lineWidth = 2; g.lineJoin = "round"; g.setLineDash(s.dash ? [6, 4] : []);
        g.beginPath(); let on = false;
        for (let i = i0; i < n; i += stride) { const v = a[i]; if (v == null || !Number.isFinite(v)) { on = false; continue; } if (on) g.lineTo(X(t[i]), Y(v)); else { g.moveTo(X(t[i]), Y(v)); on = true; } }
        g.stroke();
      }
      g.restore(); g.setLineDash([]);
      // crosshair with the values
      if (this.hoverX != null && n > 1 && this.hoverX >= m.l && this.hoverX <= m.l + pw) {
        const th = t0 + (this.hoverX - m.l) / pw * (t1 - t0);
        let lo = i0, hi = n - 1; while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] < th) lo = mid + 1; else hi = mid; }
        const i = lo, px = X(t[i]), lines = [`t = ${t[i].toFixed(1)} s`];
        g.strokeStyle = css("--axis"); g.beginPath(); g.moveTo(px + 0.5, m.t); g.lineTo(px + 0.5, m.t + ph); g.stroke();
        for (const s of series) {
          const v = T[s.key][i]; if (v == null || !Number.isFinite(v)) continue;
          g.fillStyle = css(s.color); g.strokeStyle = css("--surface"); g.lineWidth = 2;
          g.beginPath(); g.arc(px, Y(v), 4, 0, 2 * Math.PI); g.fill(); g.stroke();
          lines.push([s.label, v.toFixed(1), css(s.color)]);
        }
        g.font = font;
        const tw = Math.max(...lines.map((l) => g.measureText(typeof l === "string" ? l : `${l[0]}: ${l[1]}`).width)) + 26, bh = lines.length * 15 + 8;
        let bx = px + 10; if (bx + tw > W - 4) bx = px - 10 - tw;
        g.fillStyle = css("--hud"); g.strokeStyle = css("--line-2"); g.lineWidth = 1;
        g.beginPath(); g.roundRect ? g.roundRect(bx, m.t + 4, tw, bh, 6) : g.rect(bx, m.t + 4, tw, bh); g.fill(); g.stroke();
        g.textAlign = "left"; g.textBaseline = "middle";
        lines.forEach((l, j) => {
          const yy = m.t + 15 + j * 15;
          if (typeof l === "string") { g.fillStyle = ink; g.fillText(l, bx + 8, yy); return; }
          g.fillStyle = l[2]; g.fillRect(bx + 8, yy - 1, 10, 3); g.fillStyle = ink; g.fillText(`${l[0]}: ${l[1]}`, bx + 22, yy);
        });
      }
      g.textAlign = "start"; g.textBaseline = "alphabetic";
    }
  }

  const traj = {
    T: null, t0: null, last: 0, plots: [],
    window() { return Number($("tpWin").value); },
    clear() {
      this.T = Object.fromEntries(["t", "x", "y", "th", "thEnc", "thMag", "tx", "ty", "tth", "cl", "ml", "cr", "mr", "front", "ePos", "eTh"].map((k) => [k, []]));
      this.t0 = null;
    },
    init() {
      this.clear();
      const mk = (id, title, unit, series) => new TimePlot($(id), { title, unit, series });
      const est = "--map-trail", tru = "--true-trail";
      this.plots = [
        mk("tpX", "Position x", "cm", [{ key: "x", label: "geschätzt", color: est }, { key: "tx", label: "wahr", color: tru, dash: true, simOnly: true }]),
        mk("tpY", "Position y", "cm", [{ key: "y", label: "geschätzt", color: est }, { key: "ty", label: "wahr", color: tru, dash: true, simOnly: true }]),
        mk("tpTh", "Kurs θ (+ = rechts)", "°", [{ key: "th", label: "Pose", color: est }, { key: "thEnc", label: "Encoder", color: "--s2", dash: true },
          { key: "thMag", label: "Kompass", color: "--s3", dash: true }, { key: "tth", label: "wahr", color: tru, dash: true, simOnly: true }]),
        mk("tpWheels", "Radgeschwindigkeit", "cm/s", [{ key: "cl", label: "links Soll", color: "--s1", dash: true }, { key: "ml", label: "links Ist", color: "--s1" },
          { key: "cr", label: "rechts Soll", color: "--s2", dash: true }, { key: "mr", label: "rechts Ist", color: "--s2" }]),
        mk("tpFront", "Abstand vorne", "mm", [{ key: "front", label: "Abstand", color: "--s4" }]),
        mk("tpErrPos", "Positionsfehler der Schätzung", "cm", [{ key: "ePos", label: "Fehler", color: "--s5", simOnly: true }]),
        mk("tpErrTh", "Kursfehler der Schätzung", "°", [{ key: "eTh", label: "Fehler", color: "--s5", simOnly: true }]),
      ];
    },
    push() {
      const T = this.T;
      if (this.t0 === null || S.t < this.t0 + (T.t[T.t.length - 1] ?? 0)) { this.clear(); this.t0 = S.t; }
      T.t.push(S.t - this.t0); T.x.push(S.x); T.y.push(S.y); T.th.push(S.th);
      T.thEnc.push(S.hasHeading ? S.thEnc : null); T.thMag.push(S.hasHeading ? S.thMag : null);
      T.cl.push(S.cmdL); T.ml.push(S.measL); T.cr.push(S.cmdR); T.mr.push(S.measR); T.front.push(Math.min(S.front, 4000));
      const l = sim.link;
      if (l) {
        const o = sim.world2odo(l.p.x, l.p.y), fr = sim.frame();
        T.tx.push(o.x); T.ty.push(o.y); T.tth.push(-wrap180((l.p.psi - fr.psi) * 180 / Math.PI));
        T.ePos.push(Math.hypot(l.est.x - l.p.x, l.est.y - l.p.y)); T.eTh.push(wrap180((l.est.psi - l.p.psi) * 180 / Math.PI));
      } else { T.tx.push(null); T.ty.push(null); T.tth.push(null); T.ePos.push(null); T.eTh.push(null); }
      if (T.t.length > 72000) for (const k in T) T[k].splice(0, 12000);       // at most 2 h at 10 Hz
    },
    draw(now) {
      if (now - this.last < 100) return;
      this.last = now; this.plots.forEach((p) => p.draw());
    },
  };

  // ============================================================================
  // ToF heat map
  // ============================================================================
  function drawTof() { drawTofOn($("tofCanvas"), S.tof); }
  function drawTofOn(c, z) {
    const dpr = fitCanvas(c), g = c.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = c.clientWidth, s = W / 8;
    g.clearRect(0, 0, W, W);
    for (let r = 0; r < 8; r++) for (let col = 0; col < 8; col++) {
      const v = z ? z[r * 8 + col] : 255;
      if (v === 255) { g.fillStyle = css("--panel-2"); }
      else { const q = Math.min(1, v / 200); g.fillStyle = `hsl(${12 + q * 200}, 75%, ${38 + q * 18}%)`; }   // near = warm, far = cool
      g.fillRect(col * s + 1, r * s + 1, s - 2, s - 2);
      g.fillStyle = v === 255 ? css("--muted") : "#fff"; g.font = `600 ${Math.round(s * 0.3)}px system-ui`;
      g.textAlign = "center"; g.textBaseline = "middle";
      g.fillText(v === 255 ? "–" : String(v), col * s + s / 2, r * s + s / 2);
    }
    g.textAlign = "start"; g.textBaseline = "alphabetic";
  }

  // ============================================================================
  // Parameters
  // ============================================================================
  const params = {
    render(id, isNew) {
      const body = $("paramBody");
      if (isNew || !body.querySelector(`tr[data-id="${id}"]`)) { this.rebuild(); return; }
      const p = S.params[id];
      body.querySelector(`tr[data-id="${id}"] .cur`).textContent = fmtParam(p.value);
    },
    rebuild() {
      const body = $("paramBody"); body.innerHTML = "";
      S.params.forEach((p) => {
        if (!p) return;
        const tr = document.createElement("tr"); tr.dataset.id = p.id;
        tr.innerHTML = `<td>${p.name}</td><td class="cur">${fmtParam(p.value)}</td><td class="v"><input type="number" step="any" inputmode="decimal"></td><td><button class="btn">Setzen</button></td>`;
        const inp = tr.querySelector("input"), btn = tr.querySelector("button");
        const set = () => {
          const v = parseFloat(inp.value.replace(",", "."));
          if (!Number.isFinite(v)) return toast("Ungültiger Wert");
          S.link && S.link.write(cmd.paramSet(p.id, v)); inp.value = ""; toast(`${p.name} = ${v}`);
        };
        btn.onclick = set; inp.onkeydown = (e) => { if (e.key === "Enter") set(); };
        body.appendChild(tr);
      });
    },
    matlab() {
      return S.params.filter(Boolean).map((p) => p.name === "NavVariant"
        ? `% Free-Nav-Variante (Block "Nav Variant"): ${fmtParam(p.value)}`
        : `RK.${p.name} = ${fmtParam(p.value)};`).join("\n");
    },
  };
  function fmtParam(v) { return Number.isInteger(v) ? String(v) : String(Math.round(v * 1e4) / 1e4); }

  // ============================================================================
  // Joystick and keyboard
  // ============================================================================
  const joy = { speed: 0, steer: 0, active: false, sent: [0, 0], keys: new Set(), lastSend: 0 };
  (function joystick() {
    // the robot picture is the touch area: dragging from its centre = speed (up) and steering (sideways)
    const pad = $("joy");
    const move = (e) => {
      const r = pad.getBoundingClientRect();
      let nx = (e.clientX - r.left - r.width / 2) / (r.width / 2), ny = (e.clientY - r.top - r.height / 2) / (r.height / 2);
      const m = Math.hypot(nx, ny); if (m > 1) { nx /= m; ny /= m; }
      joy.speed = Math.round(-ny * 100); joy.steer = Math.round(nx * 100);
    };
    pad.addEventListener("pointerdown", (e) => {
      if (S.mode !== 1) { toast("Joystick nur im Modus Fernsteuerung"); return; }
      e.preventDefault(); pad.setPointerCapture(e.pointerId); joy.active = true; pad.classList.add("driving"); move(e);
    });
    pad.addEventListener("pointermove", (e) => { if (joy.active) move(e); });
    const end = () => { joy.active = false; joy.speed = 0; joy.steer = 0; pad.classList.remove("driving"); };
    pad.addEventListener("pointerup", end); pad.addEventListener("pointercancel", end);
    const keyMap = { ArrowUp: "f", KeyW: "f", ArrowDown: "b", KeyS: "b", ArrowLeft: "l", KeyA: "l", ArrowRight: "r", KeyD: "r" };
    // only text fields keep the keys (checkboxes and buttons keep the focus after a click)
    const typing = (t) => t.tagName === "SELECT" || (t.tagName === "INPUT" && !["checkbox", "radio", "button"].includes(t.type));
    window.addEventListener("keydown", (e) => {
      if (!ui.active || typing(e.target)) return;
      if (e.code === "Space") { e.preventDefault(); stop(); return; }
      if (keyMap[e.code] && S.mode === 1) { e.preventDefault(); joy.keys.add(keyMap[e.code]); }
    });
    window.addEventListener("keyup", (e) => { if (keyMap[e.code]) joy.keys.delete(keyMap[e.code]); });
    window.addEventListener("blur", () => joy.keys.clear());       // no stuck key after a window change
  })();
  // Joystick: always the latest value, with acknowledged writes (no lost packets),
  // a new value is only sent when the previous write is finished (no backlog)
  function joyTick(now) {
    if (!S.link || S.mode !== 1 || joy.busy) return;
    let sp = joy.speed, st = joy.steer;
    if (!joy.active && joy.keys.size) {
      sp = (joy.keys.has("f") ? 70 : 0) - (joy.keys.has("b") ? 70 : 0);
      st = (joy.keys.has("r") ? 100 : 0) - (joy.keys.has("l") ? 100 : 0);
    } else if (!joy.active && pad.active) {
      sp = pad.speed; st = pad.steer;
    }
    if (sp === joy.sent[0] && st === joy.sent[1]) return;
    joy.busy = true; joy.sent = [sp, st]; joy.lastSend = now;
    S.link.write(cmd.joy(sp, st)).finally(() => { joy.busy = false; });
  }
  // Mode commands are repeated until the telemetry shows the new mode
  const modeReq = { mode: null, tries: 0, timer: null };
  function requestMode(m) {
    clearTimeout(modeReq.timer);
    Object.assign(modeReq, { mode: m, tries: 0 });
    const attempt = () => {
      if (!S.link || modeReq.mode !== m) return;
      if (S.mode === m && modeReq.tries > 0) { modeReq.mode = null; return; }
      if (modeReq.tries >= 4) { toast(`Modus „${MODE_NAMES[m]}“ wurde nicht übernommen`); modeReq.mode = null; return; }
      modeReq.tries++;
      S.link.write(cmd.mode(m));
      modeReq.timer = setTimeout(attempt, 600);
    };
    attempt();
  }
  function setMode(m) {
    if (!S.link) return toast("Nicht verbunden");
    if (m === 4 && (mission.paused || !mission.active) && wps.some((p) => !p.reached)) { sendWaypoints(); return; }
    if (m !== 4) { mission.active = false; mission.paused = false; renderWps(); }
    requestMode(m);
  }
  function stop() {
    if (!S.link) return;
    requestMode(0); S.link.write(cmd.joy(0, 0)); joy.sent = [0, 0];
    if (mission.active) { mission.paused = true; renderWps(); }     // waypoint drive can continue with "Weiter"
  }

  // ============================================================================
  // UI update loop
  // ============================================================================
  const ui = { dirty: true, tofDirty: true, mapDirty: true, tab: "control", active: false };
  function renderStatus() {
    const [txt, cls] = missionText(), ms = $("mapStatus");
    ms.textContent = txt; ms.className = `map-status ${cls}`;
    $("chipMode").innerHTML = `Modus <b>${MODE_NAMES[S.mode] ?? S.mode}</b>`;
    const batt = $("chipBatt"); batt.innerHTML = `Akku <b>${S.connected ? S.vbat.toFixed(2) + " V" : "–"}</b>`;
    batt.classList.toggle("alarm", S.connected && S.vbat > 0 && S.vbat < 6.8);
    document.querySelectorAll("#rkModeBtns button").forEach((b) => {
      const m = Number(b.dataset.mode);
      b.classList.toggle("sel", S.connected && m === S.mode);
      b.classList.toggle("pending", modeReq.mode === m && S.mode !== m);
    });
    $("joy").classList.toggle("disabled", S.mode !== 1);
    $("joyInfo").textContent = S.mode === 1 ? `Geschwindigkeit ${joy.sent[0]} %, Lenkung ${joy.sent[1]} %` : "Nur im Modus Fernsteuerung aktiv";
    locUi(); navVariantUi();
    if (!S.connected) return;
    $("stMode").textContent = MODE_NAMES[S.mode] ?? S.mode;
    $("stNav").textContent = S.mode === 2 ? (NAV_NAMES[S.nav] ?? S.nav) : "–";
    $("stWp").textContent = S.mode === 4 ? `${WP_NAMES[S.wpState] ?? S.wpState}, Ziel ${S.wpIdx + 1}` : "–";
    $("stFront").textContent = `${S.front} mm`;
    $("stPose").textContent = `x ${S.x.toFixed(1)} cm, y ${S.y.toFixed(1)} cm, θ ${S.th.toFixed(1)}°`;
    $("stCmd").textContent = `L ${S.cmdL} / R ${S.cmdR} cm/s`;
    $("stMeas").textContent = `L ${S.measL.toFixed(1)} / R ${S.measR.toFixed(1)} cm/s`;
    $("stHead").textContent = S.hasHeading ? `${S.th.toFixed(1)}° / ${S.thEnc.toFixed(1)}° / ${S.thMag.toFixed(1)}°` : "–";
    $("stImu").textContent = !S.hasHeading ? "nur Encoder (Firmware ohne IMU)"
      : (S.imuFlags & 2) ? `Gyroskop (Drehrate ${S.yawRate.toFixed(1)} °/s)` : (S.imuFlags & 1) ? "Encoder (Gyro aus: RK.PoseSource = 0)" : "Encoder (keine IMU-Daten)";
    const f = []; if (S.flags & 1) f.push("Klippe"); if (S.flags & 2) f.push("Ziel erkannt"); if (S.mode === 4 && S.wpState === 3) f.push("Hindernis");
    $("stFlags").textContent = f.length ? f.join(", ") : "–";
    if (REC.on) recUi();
  }
  let lastWpRender = "";
  function frame(now) {
    padTick();
    joyTick(now);
    if (!ui.active) { requestAnimationFrame(frame); return; }    // other tab of MobileRobotSim: the link keeps running
    if (ui.dirty) {
      renderStatus(); ui.dirty = false;
      const key = `${S.mode}/${S.wpState}/${S.wpIdx}/${mission.active}/${mission.paused}/${wps.filter((p) => p.reached).length}`;
      if (key !== lastWpRender) { lastWpRender = key; renderWps(); }
      ui.mapDirty = true;
    }
    if ((ui.tab === "control" || ui.tab === "sim") && ui.tofDirty) { drawTof(); ui.tofDirty = false; }
    if (ui.tab === "control" && (ui.mapDirty || view.dirty)) { drawMap(); ui.mapDirty = false; }
    if (ui.tab === "sim") { if (ui.mapDirty) { sim.dirty = true; ui.mapDirty = false; } sim.draw(); }
    if (ui.tab === "control" || ui.tab === "sim") traj.draw(now);
    if (ui.tab === "replay") replay.tick(now);
    if (ui.tab === "rec") recView.draw(now);
    if (ui.tab === "plots" && (!frame.last || now - frame.last > 100)) { charts.draw(); frame.last = now; }
    requestAnimationFrame(frame);
  }

  // ============================================================================
  // Tab "Simulation": ground truth view of the simulated robot
  // ============================================================================
  const sim = {
    link: null, running: false, dirty: true, trueTrail: [], estTrail: [], corTrail: [],
    get follow() { return $("simFollow").checked; },
    set follow(v) { $("simFollow").checked = v; },
    // right: the map the robot builds (estimated pose, odometry frame), like in the tab "Steuerung"
    estView: new WorldView($("simEstCanvas"), {
      onUserMove: () => { sim.follow = false; sim.dirty = true; },
      onView: () => sim.syncFrom(sim.estView),
      onTap: (x, y) => {           // drawn in world coordinates, waypoints in the odometry frame
        if (!$("wpEdit").checked) return;
        if (wps.length >= WP_MAX) return toast(`Höchstens ${WP_MAX} Wegpunkte`);
        const o = sim.world2odo(x, y); wps.push({ x: Math.round(o.x), y: Math.round(o.y), reached: false }); renderWps(); sim.dirty = true;
      },
    }),
    // left: true world
    view: new WorldView($("simCanvas"), {
      onUserMove: () => { sim.follow = false; sim.dirty = true; },
      onView: () => sim.syncFrom(sim.view),
      onPress: (x, y) => sim.pressStart(x, y),
      onTap: (x, y) => {           // same as the control map: tap sets a waypoint (odometry frame)
        if (!$("wpEdit").checked) return;
        if (wps.length >= WP_MAX) return toast(`Höchstens ${WP_MAX} Wegpunkte`);
        const o = sim.world2odo(x, y); wps.push({ x: Math.round(o.x), y: Math.round(o.y), reached: false }); renderWps(); sim.dirty = true;
      },
    }),
    // odometry frame (origin = start or last odometry reset of the estimate) <-> world
    frame() { return (this.link && this.link.origin) || this.startPose(); },
    // map pose (heading in deg) -> world pose (psi in rad, counter-clockwise)
    mapToWorld(m) { const w = this.odo2world(m.x, m.y); return { x: w.x, y: w.y, psi: this.frame().psi - m.th * Math.PI / 180 }; },
    odo2world(x, y) { const o = this.frame(), c = Math.cos(o.psi), s = Math.sin(o.psi); return { x: o.x + c * x - s * y, y: o.y + s * x + c * y }; },
    world2odo(x, y) { const o = this.frame(), c = Math.cos(o.psi), s = Math.sin(o.psi), dx = x - o.x, dy = y - o.y; return { x: c * dx + s * dy, y: -s * dx + c * dy }; },
    settings() {
      return { room: $("simRoom").value, heading: $("simHeading").value, scale: Number($("simScale").value) / 100,
        slip: Number($("simSlip").value), speed: Number($("simSpeed").value), gyroOffset: Number($("simGyroOff").value) || 0 };
    },
    // start pose from the fields (m, deg) in cm and rad
    startPose() {
      const v = (id) => Number($(id).value) || 0;
      return { x: v("simStartX") * 100, y: v("simStartY") * 100, psi: v("simStartTh") * Math.PI / 180 };
    },
    showStart(p) {
      $("simStartX").value = (p.x / 100).toFixed(2); $("simStartY").value = (p.y / 100).toFixed(2);
      $("simStartTh").value = Math.round(wrap180(p.psi * 180 / Math.PI));
    },
    blocked(p) {
      const r = simRoom($("simRoom").value);
      if (r.world) return r.world.occAt(p.x, p.y);
      return false;
    },
    // new start pose: the robot is put there, odometry and map start again
    applyStart() {
      const p = this.startPose();
      if (this.blocked(p)) { toast("Die Startposition liegt in einem Hindernis"); this.showStart(simRoom($("simRoom").value).start); return; }
      this.centerOn(p); this.dirty = true;
      if (this.link) { this.resetPose(); toast("Roboter an die Startpose gesetzt, Odometrie und Karte neu"); }
    },
    // "Start per Klick": click = position, drag = direction
    picking: false,
    setPicking(on) { this.picking = on; $("simPickStart").classList.toggle("active", on); $("simCanvas").classList.toggle("picking", on); },
    pressStart(x, y) {
      if (!this.picking) return null;
      const p = { x, y, psi: this.startPose().psi };
      this.showStart(p); this.dirty = true;
      return {
        move: (mx, my) => { if (Math.hypot(mx - x, my - y) * this.view.scale > 6) { p.psi = Math.atan2(my - y, mx - x); this.showStart(p); this.dirty = true; } },
        end: () => { this.setPicking(false); this.applyStart(); },
      };
    },
    centerOn(p) { this.view.cx = p.x; this.view.cy = p.y; this.syncFrom(this.view); },
    roomChanged() {
      const id = $("simRoom").value, r = simRoom(id);
      $("simRoomDesc").textContent = (SIM_INFO[id] || {}).desc || "";
      this.showStart(r.start);
      if (r.world && $("mapSize").value !== "1600:10") { $("mapSize").value = "1600:10"; map.clear(); }   // whole building (160 m)
      this.view.scale = 1.2; this.centerOn(r.start);
    },
    attach(link) { this.link = link; this.running = true; this.follow = true; this.view.scale = this.estView.scale = 1.2; this.ui(); },
    detach() { this.link = null; this.running = false; this.ui(); },
    zoom(f) { this.view.zoom(f); this.syncFrom(this.view); },
    // both maps are drawn in world coordinates (the robot's map placed with its start pose, as in MobileRobotSim)
    // and always show the same section
    syncFrom(v) {
      const w = v === this.view ? this.estView : this.view;
      w.cx = v.cx; w.cy = v.cy; w.scale = v.scale; w.dirty = v.dirty = this.dirty = true;
    },
    ui() {
      $("simStart").textContent = this.running ? "■ Simulation beenden" : "▶ Simulation starten";
      $("simStart").classList.toggle("go", !this.running);
      document.querySelectorAll("#tab-robkit .sim-only").forEach((e) => (e.hidden = !this.running));
      $("simReset").disabled = !this.running;
      $("demoBtn").textContent = this.running ? "Simulation läuft" : "Simulation";
    },
    applySettings(id) {
      this.dirty = true;
      if (!this.link) return;
      this.link.configure(this.settings(), id === "simRoom");
      if (id === "simRoom") { this.restartOdom(); toast("Neue Umgebung: Roboter steht am Start, Karte gelöscht"); }
    },
    resetPose() {
      if (!this.link) return;
      this.link.resetPose(); this.restartOdom();
    },
    // odometry starts again at the start pose: no wheel speed from the jump, new map
    restartOdom() {
      this.link.write(cmd.odomReset());
      S.lastOdom = null; S.measL = S.measR = 0;
      map.clear(); loc.reset(); S.trail = []; ui.mapDirty = true;
    },
    clearTrails() { this.trueTrail = []; this.estTrail = []; this.corTrail = []; this.dirty = true; },
    sample(l) {       // every 100 ms from the simulation
      const add = (t, x, y) => { const q = t[t.length - 1]; if (!q || Math.hypot(q[0] - x, q[1] - y) > 0.5) { t.push([x, y]); if (t.length > 20000) t.splice(0, 5000); } };
      add(this.trueTrail, l.p.x, l.p.y); add(this.estTrail, l.est.x, l.est.y);
      const cw = this.mapToWorld(loc.toMap(S.x, S.y, S.th)); add(this.corTrail, cw.x, cw.y);
      this.dirty = true;
    },
    draw() {
      if (!this.dirty && !this.view.dirty && !this.estView.dirty) return;
      const l = this.link;
      const st = $("simStatus");
      const room = simRoom($("simRoom").value), start = this.startPose();
      if (!l) {
        this.view.draw({ walls: room.walls, world: room.world, start,
          robots: [{ x: start.x, y: start.y, th: -start.psi * 180 / Math.PI, color: "--true-robot", alpha: 0.5 }] });
        this.estView.draw({ frame: start, grid: map, start: { x: 0, y: 0, psi: 0 } });
        st.textContent = this.picking ? "Startpose: in die linke Karte klicken (Ziehen = Richtung)" : "Simulation aus – „▶ Simulation starten“";
        st.className = "map-status map-status-bar"; this.dirty = false; return;
      }
      if (this.follow) { this.view.cx = l.p.x; this.view.cy = l.p.y; this.syncFrom(this.view); }
      this.estView.draw({
        frame: this.frame(), grid: map, trails: [{ pts: S.trail, color: "--map-trail" }],
        waypoints: { list: wps, cur: curWp() },
        start: { x: 0, y: 0, psi: 0 },
        robots: [{ ...loc.toMap(S.x, S.y, S.th), color: "--robot" }],
      });
      const fov = { x: l.p.x + TOF_OFFSET_CM * Math.cos(l.p.psi), y: l.p.y + TOF_OFFSET_CM * Math.sin(l.p.psi), psi: l.p.psi, range: Number($("mapRange").value) };
      this.view.draw({
        walls: l.walls, world: l.world, fov, rays: l.rays, start: l.origin || start,
        waypoints: { list: wps.map((p) => ({ ...this.odo2world(p.x, p.y), reached: p.reached })), cur: curWp() },
        trails: [{ pts: this.estTrail, color: "--map-trail" }, ...(loc.active ? [{ pts: this.corTrail, color: "--s3" }] : []),
                 { pts: this.trueTrail, color: "--true-trail", dash: [6, 4] }],
        robots: [{ x: l.est.x, y: l.est.y, th: -l.est.psi * 180 / Math.PI, color: "--robot", alpha: 0.55 },
                 { x: l.p.x, y: l.p.y, th: -l.p.psi * 180 / Math.PI, color: "--true-robot" }],
      });
      const ePos = Math.hypot(l.est.x - l.p.x, l.est.y - l.p.y), eTh = wrap180((l.est.psi - l.p.psi) * 180 / Math.PI);
      $("simErrPos").textContent = `${ePos.toFixed(1)} cm`;
      $("simErrTh").textContent = `${eTh.toFixed(1)}°`;
      $("simDist").textContent = `${(l.dist / 100).toFixed(2)} m`;
      const mw = this.mapToWorld(loc.toMap(S.x, S.y, S.th));
      $("simErrPosMap").textContent = loc.active ? `${Math.hypot(mw.x - l.p.x, mw.y - l.p.y).toFixed(1)} cm` : "– (Lokalisierung aus)";
      $("simErrThMap").textContent = loc.active ? `${wrap180((mw.psi - l.p.psi) * 180 / Math.PI).toFixed(1)}°` : "–";
      const [mt, mc] = missionText();
      st.textContent = `${mt} · Schätzfehler ${ePos.toFixed(0)} cm / ${eTh.toFixed(1)}° · ${{ enc: "Kurs aus Encodern", gyro: "Kurs aus Gyro", fused: "Kurs aus Encoder + Gyro" }[l.cfg.heading]}`;
      st.className = `map-status map-status-bar ${mc || (Math.abs(eTh) > 10 || ePos > 20 ? "warn" : "ok")}`;
      this.dirty = false;
    },
  };

  // ============================================================================
  // Tab "Replay": recorded drives with the map built again from pose and ToF
  // ============================================================================
  const replay = {
    data: null, grid: new OccGrid(mapSettings), follow: true, playing: false, t: 0, t0: 0, t1: 0,
    tofIdx: 0, stateIdx: 0, trail: [], last: null, dirty: true,
    view: new WorldView($("replayCanvas"), { onUserMove: () => { replay.follow = false; } }),
    async loadFiles(files) {
      try {
        let d = { state: [], tof: [], waypoints: [] }, name = [...files].map((f) => f.name).join(", ");
        for (const f of files) {
          const text = await f.text();
          if (f.name.endsWith(".json")) { const j = JSON.parse(text); d = { state: j.state || [], tof: j.tof || d.tof, waypoints: j.waypoints || [] }; continue; }
          const lines = text.trim().split(/\r?\n/), head = lines.shift().split(",");
          const rows = lines.map((l) => l.split(","));
          if (head.includes("z11_cm")) d.tof = rows.map((r) => [...r.slice(0, 4).map(Number), ...r.slice(4).map((v) => (v === "" ? 255 : Number(v)))]);
          else d.state = rows.map((r) => r.map(Number));
        }
        this.load(d, name);
      } catch (e) { toast(`Datei nicht lesbar: ${e.message}`, 5000); }
    },
    load(d, name) {
      if (!d.state || !d.state.length) return toast("Keine Zustandsdaten in der Aufzeichnung");
      this.data = d;
      this.t0 = d.state[0][0]; this.t1 = d.state[d.state.length - 1][0];
      this.follow = true; this.playing = false; this.view.scale = 1.2; this.restart(); this.t = this.t0;
      $("rpPlay").disabled = false; $("rpSeek").disabled = false;
      $("rpDur").textContent = `${(this.t1 - this.t0).toFixed(1)} s`;
      $("rpCount").textContent = `${d.state.length} / ${d.tof.length}`;
      $("replayStatus").textContent = `Geladen: ${name}`;
      this.advanceTo(this.t0); this.ui();
    },
    restart() { this.grid.clear(); (this.loc || (this.loc = new Localizer(this.grid))).reset(); this.tofIdx = 0; this.stateIdx = 0; this.trail = []; this.dirty = true; },
    advanceTo(t) {
      const d = this.data;
      while (this.tofIdx < d.tof.length && d.tof[this.tofIdx][0] <= t) {
        const r = d.tof[this.tofIdx++]; this.loc.frame(r.slice(4), r[1], r[2], r[3]);
      }
      while (this.stateIdx < d.state.length && d.state[this.stateIdx][0] <= t) {
        const r = d.state[this.stateIdx++], q = this.trail[this.trail.length - 1], m = this.loc.toMap(r[1], r[2], r[3]);
        if (!q || Math.hypot(q[0] - m.x, q[1] - m.y) > 0.5) this.trail.push([m.x, m.y]);
      }
      this.t = t; this.dirty = true;
    },
    seek(frac) {
      if (!this.data) return;
      const t = this.t0 + frac * (this.t1 - this.t0);
      if (t < this.t) this.restart();
      this.advanceTo(t); this.ui();
    },
    toggle() {
      if (!this.data) return;
      if (!this.playing && this.t >= this.t1) { this.restart(); this.t = this.t0; }
      this.playing = !this.playing; this.last = null; this.ui();
    },
    tick(now) {
      if (this.playing) {
        if (this.last !== null) this.advanceTo(Math.min(this.t1, this.t + (now - this.last) / 1000 * Number($("rpSpeed").value)));
        this.last = now;
        if (this.t >= this.t1) { this.playing = false; this.ui(); }
        else if (!this.uiLast || now - this.uiLast > 200) { this.ui(); this.uiLast = now; }
      }
      if (this.dirty || this.view.dirty) this.draw();
    },
    ui() {
      $("rpPlay").textContent = this.playing ? "❚❚ Pause" : "▶ Abspielen";
      if (!this.data) return;
      $("rpSeek").value = Math.round(1000 * (this.t - this.t0) / Math.max(1e-6, this.t1 - this.t0));
      $("rpTime").textContent = `${(this.t - this.t0).toFixed(1).replace(".", ",")} / ${(this.t1 - this.t0).toFixed(1).replace(".", ",")} s`;
    },
    draw() {
      const d = this.data;
      if (!d) { this.view.draw({}); this.dirty = false; return; }
      const s = d.state[Math.max(0, this.stateIdx - 1)];
      const [, x, y, th, cl, cr, ml, mr, mode, , wi, wst, front] = s;
      const mp = this.loc.toMap(x, y, th);
      if (this.follow) { this.view.cx = mp.x; this.view.cy = mp.y; }
      const wpList = (d.waypoints || []).map((p) => ({ ...p, reached: false }));
      this.view.draw({ grid: this.grid, trails: [{ pts: this.trail, color: "--map-trail" }],
        waypoints: { list: wpList, cur: mode === 4 && (wst === 1 || wst === 3) ? wi : -1 }, robots: [{ ...mp, color: "--robot" }] });
      $("rpMode").textContent = MODE_NAMES[mode] ?? mode;
      $("rpPose").textContent = `x ${x.toFixed(1)}, y ${y.toFixed(1)} cm, θ ${th.toFixed(1)}°`;
      $("rpCmd").textContent = `L ${cl} / R ${cr} cm/s`;
      $("rpMeas").textContent = `L ${ml} / R ${mr} cm/s`;
      $("rpFront").textContent = `${front} mm`;
      $("rpWp").textContent = mode === 4 ? `${WP_NAMES[wst] ?? wst}, Ziel ${wi + 1}` : "–";
      const tf = d.tof[Math.max(0, this.tofIdx - 1)];
      drawTofOn($("rpTof"), tf ? tf.slice(4) : null);
      this.dirty = false;
    },
  };

  // ============================================================================
  // PS4 gamepad (Gamepad API, standard mapping)
  // ============================================================================
  const pad = { index: null, speed: 0, steer: 0, active: false, prev: [] };
  window.addEventListener("gamepadconnected", (e) => { pad.index = e.gamepad.index; padUi(e.gamepad); toast(`Gamepad verbunden: ${e.gamepad.id.split("(")[0]}`); });
  window.addEventListener("gamepaddisconnected", (e) => { if (e.gamepad.index === pad.index) { pad.index = null; pad.active = false; padUi(null); } });
  function padUi(gp) {
    $("padInfo").classList.toggle("on", !!gp);
    $("padState").textContent = gp ? gp.id.split("(")[0].trim() : "nicht verbunden – per USB oder Bluetooth koppeln und eine Taste drücken";
    $("chipPad").hidden = !gp;
  }
  function padTick() {
    if (pad.index === null || !navigator.getGamepads) return;
    const gp = navigator.getGamepads()[pad.index];
    if (!gp) return;
    const dz = (v) => (Math.abs(v) < 0.12 ? 0 : (v - Math.sign(v) * 0.12) / 0.88);
    const btn = (i) => !!(gp.buttons[i] && gp.buttons[i].pressed);
    const boost = gp.buttons[7] ? gp.buttons[7].value : 0;                 // R2 (analog)
    const scale = 0.6 + 0.4 * boost;
    pad.speed = Math.round(-dz(gp.axes[1] || 0) * 100 * scale);            // left stick vertical
    pad.steer = Math.round(dz(gp.axes[2] || 0) * 100 * scale);             // right stick horizontal
    pad.active = pad.speed !== 0 || pad.steer !== 0;
    const edge = (i) => btn(i) && !pad.prev[i];
    if (edge(0)) stop();                                                   // cross
    if (edge(1)) setMode(1);                                               // circle
    if (edge(2)) setMode(2);                                               // square
    if (edge(3)) setMode(4);                                               // triangle
    if (edge(9) && (mission.active || mission.paused)) pauseOrResume();    // options
    if (edge(4)) toggleRec();                                              // L1
    pad.prev = gp.buttons.map((b) => b.pressed);
  }

  // ============================================================================
  // Tab "Aufzeichnung": whole recording as signal plots, map and path
  // ============================================================================
  const recView = {
    grid: new OccGrid(mapSettings), trail: [], dirty: true, last: 0, follow: true,
    view: new WorldView($("recCanvas"), { onUserMove: () => { recView.follow = false; } }),
    charts: null,
    init() {
      const full = { full: true };
      this.charts = {
        wheels: new Chart($("rcWheels"), $("rlWheels"), [
          { label: "links Soll", color: "--s1", dash: true }, { label: "links Ist", color: "--s1" },
          { label: "rechts Soll", color: "--s2", dash: true }, { label: "rechts Ist", color: "--s2" }], full),
        pos: new Chart($("rcPos"), $("rlPos"), [{ label: "x", color: "--s1" }, { label: "y", color: "--s3" }], full),
        head: new Chart($("rcHead"), $("rlHead"), [{ label: "θ Pose", color: "--s4" },
          { label: "θ Encoder", color: "--s2", dash: true }, { label: "θ Kompass", color: "--s3", dash: true }], full),
        front: new Chart($("rcFront"), $("rlFront"), [{ label: "Abstand vorne [mm]", color: "--s2" },
          { label: "Modus × 200", color: "--s1", dash: true }], { full: true, min: 0 }),
      };
    },
    reset() {
      this.grid.clear(); this.trail = []; this.follow = true;
      Object.values(this.charts).forEach((c) => c.clear());
      this.dirty = true;
    },
    // state row: t, x, y, th, cmdL, cmdR, measL, measR, mode, nav, wpIdx, wpState, front, vbat, cliff, target, thEnc, thMag, yaw
    addState(r) {
      const t = r[0] - REC.t0, q = this.trail[this.trail.length - 1];
      if (!q || Math.hypot(q[0] - r[1], q[1] - r[2]) > 0.5) this.trail.push([r[1], r[2]]);
      this.charts.wheels.push(t, [r[4], r[6], r[5], r[7]]);
      this.charts.pos.push(t, [r[1], r[2]]);
      this.charts.head.push(t, [r[3], r[16] === "" ? NaN : r[16], r[17] === "" ? NaN : r[17]]);
      this.charts.front.push(t, [Math.min(r[12], 2000), r[8] * 200]);
      this.dirty = true;
    },
    addTof(r) { this.grid.integrate(r.slice(4), r[1], r[2], r[3]); this.dirty = true; },
    // the recording view is drawn at most 4 times per second
    draw(now) {
      if (!this.dirty && !this.view.dirty) return;
      if (now - this.last < 250 && !this.view.dirty) return;
      this.last = now;
      const tr = this.trail, end = tr[tr.length - 1];
      if (this.follow && end) { this.view.cx = end[0]; this.view.cy = end[1]; }
      const lastRow = REC.rows[REC.rows.length - 1];
      this.view.draw({
        grid: this.grid, trails: [{ pts: tr, color: "--map-trail" }],
        waypoints: { list: (REC.wps || []).map((p) => ({ ...p, reached: false })), cur: -1 },
        robots: lastRow ? [{ x: lastRow[1], y: lastRow[2], th: lastRow[3], color: "--robot" }] : [],
      });
      if (tr.length) {     // start marker
        const g = this.view.c.getContext("2d"), [sx, sy] = this.view.w2s(tr[0][0], tr[0][1]);
        g.fillStyle = css("--go"); g.beginPath(); g.arc(sx, sy, 6, 0, 2 * Math.PI); g.fill();
      }
      Object.values(this.charts).forEach((c) => c.draw());
      this.dirty = false;
    },
  };
  recView.init();

  // ============================================================================
  // Control elements shared by the tabs "Steuerung" and "Simulation"
  // ============================================================================
  // Mode, STOP, waypoints, joystick and state/ToF exist once and move to the tab
  // that is shown, so both tabs are operated the same way.
  function placeSharedControls(tab) {
    const toSim = tab === "sim", left = toSim ? $("simLeft") : $("ctlLeft");
    for (const sel of [".c-mode", ".c-wp", ".c-joy"]) left.appendChild(document.querySelector(`#tab-robkit ${sel}`));
    const side = toSim ? $("simSide") : $("ctlSide");
    side.insertBefore(document.querySelector("#tab-robkit .c-info"), side.firstChild);
    side.insertBefore(document.querySelector("#tab-robkit .c-loc"), document.querySelector("#tab-robkit .c-info").nextSibling);
    (toSim ? $("simMain") : $("ctlMain")).appendChild(document.querySelector("#tab-robkit .c-plots"));     // trajectories below the map(s)
  }
  placeSharedControls("control");


  // ============================================================================
  // Connection
  // ============================================================================
  async function connect(demo) {
    if (S.link) { S.link.disconnect(); return; }
    const Link = demo ? DemoLink : BleLink;
    const link = new Link(onPacket, onDisconnected);
    try {
      $("chipLink").textContent = "verbinde …";
      const name = await link.connect(false);
      S.link = link; S.demo = demo; S.connected = true; S.tRaw = null; S.tOff = 0; S.lastOdom = null; S.cnt = -1; traj.clear(); loc.reset();
      S.params = []; $("paramBody").innerHTML = "";
      $("chipLink").textContent = name; $("chipLink").classList.add("on");
      $("connectBtn").textContent = "Trennen"; $("demoBtn").disabled = true; $("connectBtn").disabled = false;
      if (demo) { $("connectBtn").textContent = "Simulation beenden"; sim.attach(link); }
      await link.write(cmd.stream(Number($("tofRate").value)));
      await link.write(cmd.paramGet(0xff));
      toast(`Verbunden mit ${name}`);
    } catch (e) {
      $("chipLink").textContent = "nicht verbunden";
      if (e && e.name !== "NotFoundError") toast(`Verbindung fehlgeschlagen: ${e.message || e}`, 5000);
    }
    ui.dirty = true;
  }
  function onDisconnected() {
    S.link = null; S.connected = false;
    sim.detach();
    $("chipLink").textContent = "nicht verbunden"; $("chipLink").classList.remove("on");
    $("connectBtn").textContent = "Verbinden"; $("demoBtn").disabled = false;
    if (REC.on) toggleRec();
    if (mission.active) mission.paused = true;     // the robot stops on disconnect; continue with "Weiter"
    renderWps();
    toast("Verbindung getrennt"); ui.dirty = true;
  }

  // ============================================================================
  // Recording
  // ============================================================================
  const REC_HEAD = "t_s,x_cm,y_cm,theta_deg,leftCmd_cmps,rightCmd_cmps,leftMeas_cmps,rightMeas_cmps,opMode_enum,navState_enum,wpIndex_cnt,wpState_enum,frontDist_mm,vbat_V,cliff_bool,targetFound_bool,thetaEnc_deg,thetaMag_deg,yawRate_dps";
  const TOF_HEAD = "t_s,x_cm,y_cm,theta_deg," + Array.from({ length: 64 }, (_, i) => `z${Math.floor(i / 8) + 1}${i % 8 + 1}_cm`).join(",");
  function toggleRec() {
    if (!REC.on && !S.connected) return toast("Erst verbinden oder Simulation starten");
    REC.on = !REC.on;
    if (REC.on) { REC.rows = []; REC.tof = []; REC.t0 = S.t; REC.wps = wps.map((p) => ({ x: p.x, y: p.y })); recView.reset(); }
    recUi();
    if (!REC.on && REC.rows.length) toast(`Aufnahme beendet: ${REC.rows.length} Zustände, ${REC.tof.length} ToF-Bilder – Verläufe im Reiter „Aufzeichnung“`);
  }
  function recUi() {
    const has = !REC.on && REC.rows.length > 0, dur = REC.on ? S.t - REC.t0 : (REC.rows.length ? REC.rows[REC.rows.length - 1][0] - REC.rows[0][0] : 0);
    const mmss = `${Math.floor(dur / 60)}:${String(Math.floor(dur % 60)).padStart(2, "0")}`;
    $("recHead").textContent = REC.on ? `■ Stopp ${mmss}` : "● Aufnahme";
    $("recHead").classList.toggle("rec-on", REC.on);
    $("recBtn").textContent = REC.on ? "■ Aufnahme stoppen" : "● Aufnahme starten";
    $("recBtn").classList.toggle("rec-on", REC.on);
    $("recCsv").disabled = !has; $("recTofCsv").disabled = !has || !REC.tof.length;
    $("recJson").disabled = !has; $("recToReplay").disabled = !has; $("rpUseCurrent").disabled = !has;
    $("recRows").textContent = REC.rows.length; $("recFrames").textContent = REC.tof.length; $("recDur").textContent = `${dur.toFixed(0)} s`;
  }
  const tofCsv = (r) => [...r.slice(0, 4), ...r.slice(4).map((v) => (v === 255 ? "" : v))].join(",");
  function recordingJson() {
    return JSON.stringify({
      format: "robkit-replay", version: 1, created: new Date().toISOString(),
      stateColumns: REC_HEAD.split(","), state: REC.rows,
      tofColumns: ["t_s", "x_cm", "y_cm", "theta_deg", "z_cm[64] row major, 255 = invalid"], tof: REC.tof,
      waypoints: REC.wps || [],
    });
  }

  // ============================================================================
  // Wiring
  // ============================================================================
  document.querySelectorAll("#rkTabs button").forEach((b) => b.addEventListener("click", () => {
    document.querySelectorAll("#rkTabs button").forEach((x) => x.classList.toggle("active", x === b));
    document.querySelectorAll("#tab-robkit .rk-pane").forEach((s) => s.classList.toggle("active", s.dataset.pane === b.dataset.tab));
    ui.tab = b.dataset.tab; ui.mapDirty = ui.tofDirty = true; sim.dirty = replay.dirty = view.dirty = recView.dirty = true;
    if (ui.tab === "sim" || ui.tab === "control") placeSharedControls(ui.tab);
  }));
  $("connectBtn").onclick = () => connect(false);
  $("demoBtn").onclick = () => connect(true);
  document.querySelectorAll("#rkModeBtns button").forEach((b) => (b.onclick = () => setMode(Number(b.dataset.mode))));
  // autopilot variant: parameter NavVariant of the robot (1 Standard, 2 Random Turn), also while driving
  const navVariantParam = () => S.params.find((q) => q && q.name === "NavVariant");
  function navVariantUi() {
    const p = navVariantParam();
    document.querySelectorAll("#rkNavVariant button").forEach((b) => {
      b.classList.toggle("on", !!p && Math.round(p.value) === Number(b.dataset.v)); b.disabled = !S.link || !p;
    });
  }
  document.querySelectorAll("#rkNavVariant button").forEach((b) => (b.onclick = () => {
    const p = navVariantParam();
    if (S.link && p) S.link.write(cmd.paramSet(p.id, Number(b.dataset.v)));
  }));
  $("stopBtn").onclick = stop;
  $("tofRate").onchange = () => S.link && S.link.write(cmd.stream(Number($("tofRate").value)));
  $("zoomIn").onclick = () => { view.zoom(1.3); ui.mapDirty = true; };
  $("zoomOut").onclick = () => { view.zoom(1 / 1.3); ui.mapDirty = true; };
  $("follow").onchange = () => (ui.mapDirty = true);
  $("wpSend").onclick = () => sendWaypoints();
  $("wpPause").onclick = pauseOrResume;
  $("wpClear").onclick = () => { wps.length = 0; mission.active = false; mission.paused = false; renderWps(); };
  $("mapClear").onclick = () => { map.clear(); loc.clearMap(); S.trail = []; ui.mapDirty = true; };
  // path only: the map stays (in the tab "Simulation" also the true and estimated path)
  $("trailClear").onclick = () => { S.trail = []; sim.clearTrails(); ui.mapDirty = true; };
  // localization: stage 1 on the robot (parameter GyroZupt), stages 2 and 3 in the app
  $("locZupt").onchange = (e) => {
    const p = S.params.find((q) => q && q.name === "GyroZupt");
    if (!S.link || !p) {
      e.target.checked = !e.target.checked;
      return toast(S.link ? "Die Firmware kennt den Parameter GyroZupt nicht – neue Firmware flashen" : "Nicht verbunden");
    }
    S.link.write(cmd.paramSet(p.id, e.target.checked ? 1 : 0));
  };
  ["locMatch", "locRect"].forEach((id) => ($(id).onchange = () => { if (!loc.matching) loc.flush(); ui.dirty = ui.mapDirty = true; sim.dirty = replay.dirty = true; }));
  $("odomReset").onclick = () => {
    if (!S.link) return toast("Nicht verbunden");
    if (!confirm("Odometrie auf (0, 0, 0) setzen? Karte, Spur und Wegpunkte beziehen sich danach auf die neue Lage und werden gelöscht.")) return;
    S.link.write(cmd.odomReset()); map.clear(); loc.reset(); S.trail = []; wps.length = 0; mission.active = false; mission.paused = false;
    renderWps(); charts.clear(); S.lastOdom = null;
  };
  $("mapPng").onclick = () => mapCanvas.toBlob((b) => { const a = document.createElement("a"); a.href = URL.createObjectURL(b); a.download = `robkit_karte_${stamp()}.png`; a.click(); });
  $("mapCsv").onclick = () => download(`robkit_karte_${stamp()}.csv`, map.csv());
  $("paramRead").onclick = () => S.link ? S.link.write(cmd.paramGet(0xff)) : toast("Nicht verbunden");
  $("paramCopy").onclick = async () => {
    const txt = params.matlab(); if (!txt) return toast("Noch keine Parameter gelesen");
    try { await navigator.clipboard.writeText(txt); toast("In die Zwischenablage kopiert"); } catch { download("robkit_params_app.m", txt, "text/plain"); }
  };
  $("recBtn").onclick = toggleRec;
  $("recCsv").onclick = () => download(`robkit_log_${stamp()}.csv`, [REC_HEAD, ...REC.rows.map((r) => r.join(","))].join("\n"));
  $("recTofCsv").onclick = () => download(`robkit_tof_${stamp()}.csv`, [TOF_HEAD, ...REC.tof.map(tofCsv)].join("\n"));
  $("recJson").onclick = () => download(`robkit_replay_${stamp()}.json`, recordingJson(), "application/json");
  $("recHead").onclick = toggleRec;
  $("rpUseCurrent").onclick = () => replay.load({ state: REC.rows, tof: REC.tof, waypoints: REC.wps || [] }, "aktuelle Aufnahme");
  $("recToReplay").onclick = () => { $("rpUseCurrent").click(); document.querySelector('button[data-tab="replay"]').click(); };
  $("rpFile").onchange = (e) => replay.loadFiles(e.target.files);
  $("rpPlay").onclick = () => replay.toggle();
  $("rpSeek").oninput = (e) => replay.seek(Number(e.target.value) / 1000);
  $("simStart").onclick = () => (sim.running ? S.link && S.link.disconnect() : connect(true));
  $("simPickStart").onclick = () => { sim.setPicking(!sim.picking); sim.dirty = true; };
  $("simDefStart").onclick = () => { sim.showStart(simRoom($("simRoom").value).start); sim.applyStart(); };
  ["simStartX", "simStartY", "simStartTh"].forEach((id) => ($(id).onchange = () => sim.applyStart()));
  $("mapSize").onchange = () => { map.clear(); loc.clearMap(); S.trail = []; ui.mapDirty = true; sim.dirty = true; toast(`Kartengröße ${$("mapSize").selectedOptions[0].text}, Karte gelöscht`); };
  $("tpWin").onchange = () => { traj.last = 0; };
  $("simReset").onclick = () => sim.resetPose();
  $("simZoomIn").onclick = () => sim.zoom(1.3);
  $("simZoomOut").onclick = () => sim.zoom(1 / 1.3);
  $("simFollow").onchange = () => { sim.dirty = true; };
  // own map: JSON array of wall segments [[x1, y1, x2, y2], ...] in cm or {"walls": [...]}; start at (0, 0) looking along +x
  $("simMapFile").onchange = async (e) => {
    try {
      const j = JSON.parse(await e.target.files[0].text()), w = Array.isArray(j) ? j : j.walls;
      if (!Array.isArray(w) || !w.every((s) => Array.isArray(s) && s.length === 4 && s.every(Number.isFinite))) throw new Error("erwartet [[x1, y1, x2, y2], ...] in cm");
      SIM_ROOMS.custom = w;
      const opt = $("simRoom").querySelector('option[value="custom"]'); opt.disabled = false;
      $("simRoom").value = "custom"; sim.roomChanged(); sim.applySettings("simRoom");
      toast(`Eigene Karte geladen: ${w.length} Wände`);
    } catch (err) { toast(`Karte nicht lesbar: ${err.message}`, 5000); }
  };
  ["simRoom", "simHeading", "simScale", "simSlip", "simGyroOff", "simSpeed"].forEach((id) => ($(id).onchange = () => { if (id === "simRoom") sim.roomChanged(); sim.applySettings(id); }));
  window.addEventListener("resize", () => { ui.mapDirty = ui.tofDirty = true; sim.dirty = replay.dirty = true; });
  // light / dark: switch in the header of MobileRobotSim (js/main.js); colours from the CSS variables
  function themeChanged() {
    [map, replay.grid, recView.grid].forEach((gr) => gr.invalidate());
    sim.dirty = replay.dirty = recView.dirty = view.dirty = true; ui.dirty = ui.mapDirty = ui.tofDirty = true; traj.last = 0;
  }

  function checkBluetooth() {
    if (navigator.bluetooth) return;
    const b = $("banner"); b.hidden = false;
    b.textContent = window.isSecureContext
      ? "Dieser Browser unterstützt kein Web Bluetooth. Bitte Chrome oder Edge verwenden (Android, Windows, macOS, Linux). iPhone/iPad: App „Bluefy“. Der Demo-Modus funktioniert trotzdem."
      : "Web Bluetooth braucht HTTPS oder localhost. Seite über https:// oder http://localhost öffnen. Der Demo-Modus funktioniert trotzdem.";
    $("connectBtn").disabled = true;
  }
  function init() {
    checkBluetooth(); charts.init(); traj.init(); renderWps();
    sim.roomChanged(); sim.ui(); placeSharedControls("control");
    requestAnimationFrame(frame);
  }

  MRS.RobkitTab = {
    init,
    onTheme: themeChanged,
    setActive(a) {
      ui.active = a;
      if (a) themeChanged();     // draw everything again (sizes may have changed while hidden)
      else joy.keys.clear();
    },
  };
})();
