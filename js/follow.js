/* ==========================================================================
   MobileRobotSim – Vergleich von Pfadfolgeregelungen
   1) Regelbasierte Folgeregelung (Stateflow-Diagramm):
        FollowWaypoints: goalPoint = waypoints(idx,:); [angle,range] = trackGoal(goalPoint);
                         v = 0.2; w = 1.5*angle
        [range < 0.1]  → idx == numPts ? ReachedGoal : idx = idx + 1
        ReachedGoal:     v = 0; w = 0.3
   2) Pure Pursuit (controllerPurePursuit) mit Zieltoleranz
   Beide Roboter: Differentialantrieb, Regler mit Abtastzeit Ts (Halteglied),
   Fahrzeugmodell exakt integriert (1 ms).
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { wrapAngle } = MRS;

  /** trackGoal: Winkel zum Ziel relativ zur Fahrtrichtung und Abstand. */
  function trackGoal(pose, goal) {
    const dx = goal[0] - pose[0], dy = goal[1] - pose[1];
    return [wrapAngle(Math.atan2(dy, dx) - pose[2]), Math.hypot(dx, dy)];
  }
  MRS.trackGoal = trackGoal;

  class RuleBasedFollower {
    /** targets: anzufahrende Wegpunkte (ohne Startpunkt); p: { v, k, tol, wGoal } */
    constructor(targets, p) {
      this.targets = targets;
      this.p = p;
      this.idx = 0;
      this.state = 'FollowWaypoints';
      this.done = false;
    }
    get goal() { return this.targets[Math.min(this.idx, this.targets.length - 1)]; }
    command(pose) {
      const p = this.p;
      if (this.state === 'ReachedGoal') return [0, p.wGoal];
      let [angle, range] = trackGoal(pose, this.goal);
      if (range < p.tol) {
        if (this.idx === this.targets.length - 1) {
          this.state = 'ReachedGoal';
          this.done = true;
          return [0, p.wGoal];
        }
        this.idx++;
        [angle, range] = trackGoal(pose, this.goal);
      }
      return [p.v, p.k * angle];
    }
    info() { return this.state === 'ReachedGoal' ? 'ReachedGoal' : `FollowWaypoints · Ziel ${this.idx + 1}/${this.targets.length}`; }
  }

  class PurePursuitFollower {
    /** path: Wegpunkte inkl. Startpunkt; p: { lookahead, v, wMax, tol } */
    constructor(path, p) {
      this.p = p;
      this.path = path;
      this.pp = new MRS.PurePursuit(path, { lookahead: p.lookahead, vDesired: p.v, wMax: p.wMax });
      this.done = false;
    }
    command(pose) {
      if (this.done) return [0, 0];
      const last = this.path[this.path.length - 1];
      if (this.pp.projIdx >= this.path.length - 2 && Math.hypot(pose[0] - last[0], pose[1] - last[1]) < this.p.tol) {
        this.done = true;
        return [0, 0];
      }
      const r = this.pp.step(pose);
      return [r.v, r.w];
    }
    info() { return this.done ? 'Ziel erreicht' : `Segment ${this.pp.projIdx + 1}/${this.path.length - 1}`; }
  }

  /** Abstand eines Punktes zur Referenz-Polylinie (Querabweichung). */
  function crossTrack(pts, x, y) {
    let best = Infinity;
    for (let i = 0; i < pts.length - 1; i++) {
      const a = pts[i], b = pts[i + 1];
      const dx = b[0] - a[0], dy = b[1] - a[1], l2 = dx * dx + dy * dy;
      const t = l2 > 0 ? Math.max(0, Math.min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / l2)) : 0;
      const d = Math.hypot(x - a[0] - t * dx, y - a[1] - t * dy);
      if (d < best) best = d;
    }
    return best;
  }

  class FollowSim {
    /**
     * world: Belegungskarte, wps: Wegpunkte (erster = Start), cfg: { ts, maxTime, wLimit, robotR, th0 (optional),
     * rule:{v,k,tol,wGoal}, pp:{lookahead,v,wMax,tol}, useRule, usePP }
     */
    constructor(world, wps, cfg) {
      this.world = world;
      this.wps = wps;
      this.cfg = cfg;
      this.t = 0;
      const th0 = cfg.th0 != null ? cfg.th0 : wps.length > 1 ? Math.atan2(wps[1][1] - wps[0][1], wps[1][0] - wps[0][0]) : 0;
      const start = [wps[0][0], wps[0][1], th0];
      const mk = (id, name, color, ctrl) => ({
        id, name, color, ctrl, pose: start.slice(), cmd: [0, 0], path: [[start[0], start[1]]],
        log: { t: [0], ct: [0], v: [0], w: [0] }, dist: 0, tDone: NaN, contacts: 0, inContact: false,
        ctSum: 0, ctN: 0, ctMax: 0, wMaxAbs: 0
      });
      this.agents = [];
      if (cfg.useRule) this.agents.push(mk('rule', 'Regelbasiert', '#eb6834', new RuleBasedFollower(wps.slice(1), cfg.rule)));
      if (cfg.usePP) this.agents.push(mk('pp', 'Pure Pursuit', '#2a78d6', new PurePursuitFollower(wps, cfg.pp)));
      this.nextCtrl = 0;
      this.done = false;
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

    step(dt) {
      if (this.done) return;
      const cfg = this.cfg;
      // Regler mit Abtastzeit Ts, Stellgrößen werden bis zum nächsten Abtastschritt gehalten
      if (this.t >= this.nextCtrl - 1e-9) {
        for (const a of this.agents) {
          let [v, w] = a.ctrl.command(a.pose);
          if (cfg.wLimit > 0) w = Math.max(-cfg.wLimit, Math.min(cfg.wLimit, w));
          a.cmd = [v, w];
          if (a.ctrl.done && !isFinite(a.tDone)) a.tDone = this.t;
        }
        this.nextCtrl = this.t + cfg.ts;
      }
      this.t += dt;
      for (const a of this.agents) {
        const [v, w] = a.cmd;
        const p = a.pose, th = p[2];
        let np;
        if (Math.abs(w) < 1e-9) np = [p[0] + v * Math.cos(th) * dt, p[1] + v * Math.sin(th) * dt, th];
        else {
          const th2 = th + w * dt;
          np = [p[0] + (v / w) * (Math.sin(th2) - Math.sin(th)), p[1] - (v / w) * (Math.cos(th2) - Math.cos(th)), wrapAngle(th2)];
        }
        const hit = v !== 0 && this.collides(np[0], np[1]);
        if (hit) np = [p[0], p[1], np[2]]; // Wand blockiert die Translation
        if (hit && !a.inContact) a.contacts++;
        a.inContact = hit;
        a.dist += Math.hypot(np[0] - p[0], np[1] - p[1]);
        a.pose = np;
        const lp = a.path[a.path.length - 1];
        if (Math.hypot(np[0] - lp[0], np[1] - lp[1]) > 0.01) a.path.push([np[0], np[1]]);
        a.wMaxAbs = Math.max(a.wMaxAbs, Math.abs(w));
      }
      // Protokoll alle 50 ms
      const L0 = this.agents[0] && this.agents[0].log;
      if (L0 && this.t - L0.t[L0.t.length - 1] >= 0.05 - 1e-9) {
        for (const a of this.agents) {
          const ct = crossTrack(this.wps, a.pose[0], a.pose[1]);
          if (!a.ctrl.done) { a.ctSum += ct; a.ctN++; a.ctMax = Math.max(a.ctMax, ct); }
          a.log.t.push(this.t); a.log.ct.push(ct); a.log.v.push(a.cmd[0]); a.log.w.push(a.cmd[1]);
        }
      }
      if (this.agents.every((a) => a.ctrl.done) || this.t >= cfg.maxTime) this.done = true;
    }
  }

  MRS.FollowSim = FollowSim;
  MRS.crossTrack = crossTrack;
})();
