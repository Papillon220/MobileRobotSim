/* ==========================================================================
   MobileRobotSim – Pure-Pursuit-Regler und Pfadfolge-Simulation
   Nachbildung von controllerPurePursuit + Schleife aus mobRob_planningAlgorithms.m
   Fahrzeug: differentialDriveKinematics, Eingänge (v, omega).
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  const { wrapAngle } = MRS;

  class PurePursuit {
    constructor(waypoints, opt) {
      this.wp = waypoints.map((p) => [p[0], p[1]]);
      this.L = opt.lookahead;
      this.v = opt.vDesired;
      this.wMax = opt.wMax;
      this.segLen = [];
      for (let i = 0; i < this.wp.length - 1; i++) this.segLen.push(Math.hypot(this.wp[i + 1][0] - this.wp[i][0], this.wp[i + 1][1] - this.wp[i][1]));
      this.projIdx = 0;
      this.projPt = this.wp[0].slice();
      this.lookPt = this.wp[0].slice();
    }

    /** Nächstgelegenen Punkt auf dem Pfad suchen (nur vorwärts, begrenztes Suchfenster). */
    _project(x, y) {
      const wp = this.wp, n = wp.length;
      if (n < 2) return;
      const window = Math.max(2 * this.L, 1.0);
      let best = Infinity, bi = this.projIdx, bp = this.projPt, acc = 0;
      for (let i = this.projIdx; i < n - 1; i++) {
        const a = wp[i], b = wp[i + 1];
        const dx = b[0] - a[0], dy = b[1] - a[1], l2 = dx * dx + dy * dy;
        let t = l2 > 0 ? ((x - a[0]) * dx + (y - a[1]) * dy) / l2 : 0;
        if (i === this.projIdx && l2 > 0) {
          const t0 = ((this.projPt[0] - a[0]) * dx + (this.projPt[1] - a[1]) * dy) / l2;
          t = Math.max(t, t0);
        }
        t = Math.max(0, Math.min(1, t));
        const px = a[0] + t * dx, py = a[1] + t * dy;
        const d = Math.hypot(x - px, y - py);
        if (d < best) { best = d; bi = i; bp = [px, py]; }
        acc += i === this.projIdx ? Math.hypot(b[0] - this.projPt[0], b[1] - this.projPt[1]) : this.segLen[i];
        if (acc > window) break;
      }
      this.projIdx = bi;
      this.projPt = bp;
    }

    _lookahead() {
      const wp = this.wp, n = wp.length;
      let rem = this.L, i = this.projIdx, p = this.projPt;
      while (i < n - 1) {
        const b = wp[i + 1];
        const d = Math.hypot(b[0] - p[0], b[1] - p[1]);
        if (d >= rem) { const f = rem / d; return [p[0] + (b[0] - p[0]) * f, p[1] + (b[1] - p[1]) * f]; }
        rem -= d; p = b; i++;
      }
      return wp[n - 1].slice();
    }

    step(pose) {
      this._project(pose[0], pose[1]);
      const lp = (this.lookPt = this._lookahead());
      const alpha = wrapAngle(Math.atan2(lp[1] - pose[1], lp[0] - pose[0]) - pose[2]);
      const v = this.v;
      let w = (2 * v * Math.sin(alpha)) / this.L;
      w = Math.max(-this.wMax, Math.min(this.wMax, w));
      return { v, w };
    }
  }
  MRS.PurePursuit = PurePursuit;

  /**
   * Geschlossene Regelschleife wie im MATLAB-Skript:
   * Pure Pursuit + Überschwingreduktion + Endanflug-Regelung, Euler-Integration.
   */
  class PathFollowSim {
    constructor(states, start, goal, prm, rawMap, robotRadius) {
      this.prm = prm;
      this.goal = goal;
      this.pp = new PurePursuit(states, prm);
      this.pose = start.slice();
      this.t = 0;
      this.k = 0;
      this.path = [[start[0], start[1]]];
      this.log = { t: [0], v: [0], w: [0] };
      this.dist = 0;
      this.done = false;
      this.goalReached = false;
      this.collisions = 0;
      this.inCollision = false;
      this.raw = rawMap;
      this.rr = robotRadius;
      this.maxSteps = Math.round(prm.maxTime / prm.dt);
    }

    _collides(x, y) {
      const m = this.raw, r = this.rr;
      if (m.occupiedAt(x, y)) return true;
      for (let k = 0; k < 8; k++) {
        const a = (k * Math.PI) / 4;
        if (m.occupiedAt(x + r * Math.cos(a), y + r * Math.sin(a))) return true;
      }
      return false;
    }

    step() {
      if (this.done) return;
      const P = this.prm, pose = this.pose, g = this.goal;
      const ref = this.pp.step(pose);
      const vRef = ref.v, wRef = ref.w;
      const goalDist = Math.hypot(pose[0] - g[0], pose[1] - g[1]);
      const goalHeadErr = wrapAngle(g[2] - pose[2]);
      const sat = (e) => Math.sign(e) * Math.min(P.wMax, P.headingGain * Math.abs(e));
      let v, w;
      this.mode = 'Pfadfolge';
      if (goalDist <= P.terminalDist) {
        const tgt = Math.atan2(g[1] - pose[1], g[0] - pose[0]);
        const e = wrapAngle(tgt - pose[2]);
        this.mode = 'Endanflug';
        if (goalDist <= P.goalTolPos) {
          v = 0;
          if (Math.abs(goalHeadErr) > P.goalTolHead) w = sat(goalHeadErr);
          else { w = 0; this.goalReached = true; }
          this.mode = 'Ausrichten';
        } else if (Math.abs(e) > P.rotateThresh) { v = 0; w = sat(e); }
        else { v = Math.min(P.vDesired, P.goalLinGain * goalDist); w = sat(e); }
      } else {
        switch (P.overshootMode) {
          case 1: {
            if (vRef > 0) {
              const ePath = wrapAngle(Math.atan((wRef * P.lookahead) / vRef));
              if (Math.abs(ePath) > P.rotateThresh) { v = 0; w = sat(ePath); this.mode = 'Drehen auf der Stelle'; }
              else { v = vRef; w = wRef; }
            } else { v = vRef; w = wRef; }
            break;
          }
          case 2:
            v = vRef * Math.exp(-1.5 * Math.abs(wRef));
            w = wRef;
            break;
          default:
            v = vRef; w = wRef;
        }
      }
      if (this.goalReached) { this.done = true; this._log(v, w); return; }
      // Kinematik des Differentialantriebs: [v cos θ, v sin θ, ω]
      const dt = P.dt;
      const nx = pose[0] + v * Math.cos(pose[2]) * dt;
      const ny = pose[1] + v * Math.sin(pose[2]) * dt;
      this.dist += Math.hypot(nx - pose[0], ny - pose[1]);
      this.pose = [nx, ny, pose[2] + w * dt];
      this.k++;
      this.t = this.k * dt;
      this.path.push([nx, ny]);
      this._log(v, w);
      const col = this._collides(nx, ny);
      if (col && !this.inCollision) this.collisions++;
      this.inCollision = col;
      const d = Math.hypot(nx - g[0], ny - g[1]);
      if (d <= P.goalTolPos && Math.abs(wrapAngle(g[2] - this.pose[2])) <= P.goalTolHead) this.goalReached = true;
      if (this.goalReached || this.k >= this.maxSteps) this.done = true;
    }

    _log(v, w) {
      this.log.t.push(this.t);
      this.log.v.push(v);
      this.log.w.push(w);
    }
  }
  MRS.PathFollowSim = PathFollowSim;
})();
