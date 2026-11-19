/* Understudy teach-and-replay simulator.
   A planar two-link arm (shoulder + elbow). A therapist demonstration defines the path.
   During replay a simulated weak patient tries to follow it; the robot adds force
   only when the hand drifts outside a small deadband (assist-as-needed). */
(function () {
  "use strict";
  var WORLD_W = 1.6, WORLD_H = 1.0;
  var SHOULDER = { x: 0.32, y: 0.45 };
  var L1 = 0.45, L2 = 0.43, REACH = L1 + L2 - 0.004;
  var CM_PER_UNIT = 68;

  function bez(p0, p1, p2, t) {
    var u = 1 - t;
    return { x: u * u * p0.x + 2 * u * t * p1.x + t * t * p2.x, y: u * u * p0.y + 2 * u * t * p1.y + t * t * p2.y };
  }
  function defaultPath() {
    var pts = [], a = { x: 0.62, y: 0.82 }, c = { x: 1.12, y: 0.86 }, b = { x: 1.08, y: 0.30 };
    for (var i = 0; i <= 60; i++) pts.push(bez(a, c, b, i / 60));
    return pts;
  }
  function resample(pts, n) {
    var d = [0];
    for (var i = 1; i < pts.length; i++) d.push(d[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y));
    var total = d[d.length - 1], out = [], j = 0;
    if (total === 0) return [pts[0]];
    for (var k = 0; k < n; k++) {
      var s = (k / (n - 1)) * total;
      while (j < d.length - 2 && d[j + 1] < s) j++;
      var seg = d[j + 1] - d[j] || 1, f = (s - d[j]) / seg;
      out.push({ x: pts[j].x + (pts[j + 1].x - pts[j].x) * f, y: pts[j].y + (pts[j + 1].y - pts[j].y) * f });
    }
    return out;
  }
  function clampReach(p) {
    var dx = p.x - SHOULDER.x, dy = p.y - SHOULDER.y, r = Math.hypot(dx, dy);
    if (r > REACH) return { x: SHOULDER.x + dx / r * REACH, y: SHOULDER.y + dy / r * REACH };
    if (r < 0.08) return { x: SHOULDER.x + (dx / (r || 1)) * 0.08, y: SHOULDER.y + (dy / (r || 1)) * 0.08 };
    return p;
  }
  function elbowFor(h) {
    var dx = h.x - SHOULDER.x, dy = h.y - SHOULDER.y, d = Math.min(Math.hypot(dx, dy), REACH);
    var base = Math.atan2(dy, dx);
    var cosA = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
    var off = Math.acos(Math.max(-1, Math.min(1, cosA)));
    var e1 = { x: SHOULDER.x + L1 * Math.cos(base + off), y: SHOULDER.y + L1 * Math.sin(base + off) };
    var e2 = { x: SHOULDER.x + L1 * Math.cos(base - off), y: SHOULDER.y + L1 * Math.sin(base - off) };
    return e1.y > e2.y ? e1 : e2; // elbow hangs down
  }
  function minJerk(t) { return t * t * t * (10 - 15 * t + 6 * t * t); }

  function createSim(canvas, opts) {
    opts = opts || {};
    var ctx = canvas.getContext("2d");
    var S = {
      path: resample(defaultPath(), 240), strength: opts.strength != null ? opts.strength : 0.35,
      assist: true, clock: 0, hand: null, vel: { x: 0, y: 0 }, trail: [], mode: "replay",
      teachPts: [], force: { x: 0, y: 0 }, assistEma: 0, errEma: 0, paused: false, auto: null, frame: 0
    };
    S.hand = { x: S.path[0].x, y: S.path[0].y };
    var scale = 1, W = 0, H = 0, dpr = 1;

    function colors() {
      var cs = getComputedStyle(canvas);
      function v(n, f) { return (cs.getPropertyValue(n) || "").trim() || f; }
      return {
        grid: v("--sim-grid", "rgba(0,0,0,.06)"), path: v("--sim-path", "#3355ff"), hand: v("--sim-hand", "#0e1a24"),
        arm: v("--sim-arm", "#7d8b99"), force: v("--sim-force", "#e2702b"), text: v("--sim-text", "#4a5866"), bg: v("--sim-bg", "transparent")
      };
    }
    function resize() {
      var r = canvas.getBoundingClientRect();
      dpr = window.devicePixelRatio || 1;
      W = Math.max(1, r.width); H = Math.max(1, r.height);
      canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
      scale = Math.min(W / WORLD_W, H / WORLD_H);
    }
    function toPx(p) { var ox = (W - WORLD_W * scale) / 2, oy = (H - WORLD_H * scale) / 2; return { x: ox + p.x * scale, y: oy + p.y * scale }; }
    function toWorld(x, y) { var ox = (W - WORLD_W * scale) / 2, oy = (H - WORLD_H * scale) / 2; return { x: (x - ox) / scale, y: (y - oy) / scale }; }

    var CYCLE = 4.0, HOLD = 0.7;
    function targetAt(clock) {
      var period = 2 * (CYCLE + HOLD), t = clock % period, u;
      if (t < CYCLE) u = minJerk(t / CYCLE);
      else if (t < CYCLE + HOLD) u = 1;
      else if (t < 2 * CYCLE + HOLD) u = 1 - minJerk((t - CYCLE - HOLD) / CYCLE);
      else u = 0;
      var idx = u * (S.path.length - 1), i = Math.floor(idx), f = idx - i, a = S.path[i], b = S.path[Math.min(i + 1, S.path.length - 1)];
      return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f };
    }

    function step(dt) {
      if (S.mode === "auto-teach") {
        S.auto.t += dt;
        var k = Math.min(1, S.auto.t / S.auto.dur), n = Math.max(1, Math.floor(minJerk(k) * (S.auto.src.length - 1)));
        S.teachPts = S.auto.src.slice(0, n + 1);
        S.hand = S.teachPts[S.teachPts.length - 1];
        S.force = { x: 0, y: 0 };
        if (k >= 1) finishTeach(S.auto.src);
        return;
      }
      if (S.mode === "teach") { S.force = { x: 0, y: 0 }; return; }
      S.clock += dt;
      var tg = targetAt(S.clock), h = S.hand, s = S.strength;
      var ex = tg.x - h.x, ey = tg.y - h.y, e = Math.hypot(ex, ey);
      var Kp = 70, D = 15, g = 4.2;
      var tremor = Math.sin(S.clock * 31) * 0.35 + Math.sin(S.clock * 23.7) * 0.3;
      var fpx = s * Kp * ex - D * S.vel.x + tremor * (1 - s);
      var fpy = s * Kp * ey - D * S.vel.y + g * (1 - s) * 0.55;
      var frx = 0, fry = 0, db = 0.018;
      if (S.assist && e > db) {
        var mag = Math.min(160 * (e - db), 9);
        frx = mag * ex / e; fry = mag * ey / e;
      }
      S.force = { x: frx, y: fry };
      S.vel.x += (fpx + frx) * dt; S.vel.y += (fpy + fry) * dt;
      var nx = h.x + S.vel.x * dt, ny = h.y + S.vel.y * dt, cl = clampReach({ x: nx, y: ny });
      if (cl.x !== nx || cl.y !== ny) { S.vel.x *= 0.5; S.vel.y *= 0.5; }
      if (cl.y > 0.97) { cl.y = 0.97; S.vel.y = Math.min(0, S.vel.y); }
      S.hand = cl;
      var active = Math.hypot(s * Kp * ex, s * Kp * ey) + 0.6 * s, fr = Math.hypot(frx, fry);
      var share = fr / (fr + active + 1e-6);
      S.assistEma += (share - S.assistEma) * 0.02;
      S.errEma += (e * CM_PER_UNIT - S.errEma) * 0.02;
      S.trail.push({ x: S.hand.x, y: S.hand.y });
      if (S.trail.length > 420) S.trail.shift();
    }

    function line(pts, color, width, dash) {
      if (pts.length < 2) return;
      ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = "round"; ctx.lineJoin = "round";
      if (dash) ctx.setLineDash(dash);
      ctx.beginPath(); var p = toPx(pts[0]); ctx.moveTo(p.x, p.y);
      for (var i = 1; i < pts.length; i++) { p = toPx(pts[i]); ctx.lineTo(p.x, p.y); }
      ctx.stroke(); ctx.restore();
    }
    function draw() {
      var c = colors(), u = scale;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      if (c.bg !== "transparent") { ctx.fillStyle = c.bg; ctx.fillRect(0, 0, W, H); }
      // 10 cm grid
      ctx.strokeStyle = c.grid; ctx.lineWidth = 1;
      var stepU = 10 / CM_PER_UNIT;
      for (var gx = 0; gx <= WORLD_W + 1e-6; gx += stepU) { var a = toPx({ x: gx, y: 0 }), b = toPx({ x: gx, y: WORLD_H }); ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke(); }
      for (var gy = 0; gy <= WORLD_H + 1e-6; gy += stepU) { var a2 = toPx({ x: 0, y: gy }), b2 = toPx({ x: WORLD_W, y: gy }); ctx.beginPath(); ctx.moveTo(a2.x, a2.y); ctx.lineTo(b2.x, b2.y); ctx.stroke(); }

      var teaching = S.mode === "teach" || S.mode === "auto-teach";
      if (teaching) line(S.teachPts, c.path, Math.max(2, u * 0.012));
      else {
        line(S.path, c.path, Math.max(2, u * 0.01), [u * 0.02, u * 0.018]);
        line(S.trail, c.hand, Math.max(1.5, u * 0.006));
      }
      // arm
      var h = S.hand, el = elbowFor(h), sp = toPx(SHOULDER), ep = toPx(el), hp = toPx(h);
      ctx.save(); ctx.strokeStyle = c.arm; ctx.lineCap = "round"; ctx.lineWidth = u * 0.05;
      ctx.beginPath(); ctx.moveTo(sp.x, sp.y); ctx.lineTo(ep.x, ep.y); ctx.lineTo(hp.x, hp.y); ctx.stroke();
      ctx.fillStyle = c.hand;
      [sp, ep].forEach(function (p) { ctx.beginPath(); ctx.arc(p.x, p.y, u * 0.022, 0, Math.PI * 2); ctx.fill(); });
      // forearm cuff (robot attachment)
      var cx = ep.x + (hp.x - ep.x) * 0.62, cy = ep.y + (hp.y - ep.y) * 0.62, ang = Math.atan2(hp.y - ep.y, hp.x - ep.x);
      ctx.translate(cx, cy); ctx.rotate(ang); ctx.fillStyle = teaching ? c.path : c.force;
      ctx.fillRect(-u * 0.05, -u * 0.035, u * 0.1, u * 0.07); ctx.restore();
      ctx.fillStyle = c.hand; ctx.beginPath(); ctx.arc(hp.x, hp.y, u * 0.026, 0, Math.PI * 2); ctx.fill();
      // assist force arrow
      var f = Math.hypot(S.force.x, S.force.y);
      if (f > 0.05) {
        var len = u * 0.03 * f, dx = S.force.x / f, dy = S.force.y / f, tx = hp.x + dx * len, ty = hp.y + dy * len;
        ctx.save(); ctx.strokeStyle = c.force; ctx.fillStyle = c.force; ctx.lineWidth = Math.max(2, u * 0.012); ctx.lineCap = "round";
        ctx.beginPath(); ctx.moveTo(hp.x, hp.y); ctx.lineTo(tx, ty); ctx.stroke();
        var hs = u * 0.03; ctx.beginPath(); ctx.moveTo(tx + dx * hs * 0.6, ty + dy * hs * 0.6);
        ctx.lineTo(tx - dy * hs * 0.5, ty + dx * hs * 0.5); ctx.lineTo(tx + dy * hs * 0.5, ty - dx * hs * 0.5); ctx.fill(); ctx.restore();
      }
      // scale bar
      var s0 = toPx({ x: 0.06, y: 0.95 }), s1 = toPx({ x: 0.06 + 10 / CM_PER_UNIT, y: 0.95 });
      ctx.strokeStyle = c.text; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(s0.x, s0.y); ctx.lineTo(s1.x, s1.y); ctx.stroke();
      ctx.fillStyle = c.text; ctx.font = Math.max(10, u * 0.032) + "px 'IBM Plex Mono', ui-monospace, monospace";
      ctx.fillText("10 cm", s1.x + 8, s1.y + 4);
    }

    var last = null, raf = 0;
    function loop(ts) {
      if (last == null) last = ts;
      var dt = Math.min(0.05, (ts - last) / 1000); last = ts;
      if (!S.paused) { var n = 4; for (var i = 0; i < n; i++) step(dt / n); }
      draw();
      if (opts.onMetrics && (S.frame++ % 6 === 0)) opts.onMetrics({ mode: S.mode, assist: S.assistEma, errorCm: S.errEma, strength: S.strength, assistOn: S.assist });
      raf = requestAnimationFrame(loop);
    }
    function finishTeach(pts) {
      var len = 0; for (var i = 1; i < pts.length; i++) len += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
      if (len > 0.15) S.path = resample(pts, 240);
      S.mode = "replay"; S.clock = 0; S.trail = []; S.vel = { x: 0, y: 0 }; S.hand = { x: S.path[0].x, y: S.path[0].y };
      S.assistEma = 0; S.errEma = 0; S.teachPts = [];
    }

    if (opts.interactive !== false) {
      canvas.style.touchAction = "none";
      canvas.addEventListener("pointerdown", function (ev) {
        var r = canvas.getBoundingClientRect(), p = clampReach(toWorld(ev.clientX - r.left, ev.clientY - r.top));
        S.mode = "teach"; S.teachPts = [p]; S.hand = p; canvas.setPointerCapture(ev.pointerId);
      });
      canvas.addEventListener("pointermove", function (ev) {
        if (S.mode !== "teach") return;
        var r = canvas.getBoundingClientRect(), p = clampReach(toWorld(ev.clientX - r.left, ev.clientY - r.top));
        S.teachPts.push(p); S.hand = p;
      });
      function end() { if (S.mode === "teach") finishTeach(S.teachPts); }
      canvas.addEventListener("pointerup", end); canvas.addEventListener("pointercancel", end);
    }
    window.addEventListener("resize", resize);
    resize(); raf = requestAnimationFrame(loop);

    return {
      setStrength: function (v) { S.strength = v; },
      setAssist: function (on) { S.assist = !!on; },
      setPaused: function (p) { S.paused = !!p; },
      resetPath: function () { finishTeach(defaultPath()); },
      autoTeach: function (dur) {
        var src = [], a = { x: 0.58, y: 0.86 }, c = { x: 0.80, y: 0.22 }, b = { x: 1.12, y: 0.42 };
        for (var i = 0; i <= 80; i++) { var p = bez(a, c, b, i / 80); p.y += Math.sin(i / 6) * 0.006; src.push(clampReach(p)); }
        S.mode = "auto-teach"; S.auto = { src: src, t: 0, dur: dur || 3 }; S.teachPts = [src[0]];
      },
      resize: resize
    };
  }
  window.UnderstudySim = { create: createSim };
})();
