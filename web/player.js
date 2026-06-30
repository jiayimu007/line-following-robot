/*
 * Line-Following Robot - trajectory player (animation only)
 * ---------------------------------------------------------
 * This file contains NO simulation. It loads a trajectory JSON produced by the
 * C++ engine (web/data/<track>.json, written by `make web`) and animates it on
 * a <canvas>. Every number it draws - the track polyline, the car pose, the two
 * sensor dots, the L/R on/off read-out - was computed by the engine and stored
 * in that JSON. The player only:
 *
 *   - fetches the JSON,
 *   - draws the polyline once into a static layer,
 *   - advances a frame cursor at a slider-controlled rate,
 *   - linearly interpolates the car position between two stored frames so the
 *     motion is smooth, and draws the precomputed sensor dots and states.
 *
 * There is no sensor model, no comparator, no controller and no kinematics here.
 * If the demo needs more behaviour, export it from the C++ engine - do not add
 * algorithm to this file. (Interpolation below is pure rendering: a weighted
 * average of two already-computed poses, never a motion update.)
 */

(function () {
  "use strict";

  var canvas = document.getElementById("view");
  var ctx = canvas.getContext("2d");

  // --- colours, matched to the Python GIF renderer (render.py) ---------------
  var COL = {
    bg: "#f4f5f8",
    line: "#181818",
    carFill: "#1f6feb",
    carEdge: "#0b3d91",
    nose: "#facc15",
    on: "#22c55e",
    off: "#ef4444",
    dotEdge: "#ffffff"
  };

  // --- player state ----------------------------------------------------------
  var data = null;        // the loaded JSON for the current track
  var staticLayer = null; // offscreen canvas holding background + line
  var cursor = 0;         // fractional frame index into data.frames
  var playing = true;
  var speed = 1.0;        // frames advanced per ~16ms at slider = 1.0
  var track = "oval";
  var lastTs = 0;
  var raf = null;

  // --- DOM -------------------------------------------------------------------
  var playBtn = document.getElementById("play");
  var restartBtn = document.getElementById("restart");
  var speedInput = document.getElementById("speed");
  var speedVal = document.getElementById("speed-val");
  var frameVal = document.getElementById("frame-val");
  var badgeL = document.getElementById("badge-left");
  var badgeR = document.getElementById("badge-right");
  var trackBtns = Array.prototype.slice.call(
    document.querySelectorAll("button[data-track]"));

  function t(key) {
    var d = (window.I18N && window.I18N[window.LANG]) || {};
    return d[key] != null ? d[key] : key;
  }

  // --- loading the engine's exported data ------------------------------------
  function load(name) {
    track = name;
    return fetch("data/" + name + ".json", { cache: "no-cache" })
      .then(function (r) {
        if (!r.ok) throw new Error("HTTP " + r.status);
        return r.json();
      })
      .then(function (json) {
        data = json;
        cursor = 0;
        buildStaticLayer();
        // size the canvas to the field the engine reported
        canvas.width = data.width;
        canvas.height = data.height;
        draw();
      })
      .catch(function (err) {
        ctx.fillStyle = "#161b22";
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#ff8d8d";
        ctx.font = "16px monospace";
        ctx.fillText("Could not load data/" + name + ".json", 20, 40);
        ctx.fillText(String(err), 20, 64);
        ctx.fillText("Run `make web` to generate the data files.", 20, 88);
      });
  }

  // Pre-render the static layer (background + the track polyline) once per
  // track, so each animated frame only stamps the moving car on top.
  function buildStaticLayer() {
    var w = data.width, h = data.height;
    staticLayer = document.createElement("canvas");
    staticLayer.width = w;
    staticLayer.height = h;
    var s = staticLayer.getContext("2d");

    s.fillStyle = COL.bg;
    s.fillRect(0, 0, w, h);

    var poly = data.polyline;
    if (poly && poly.length) {
      s.strokeStyle = COL.line;
      s.lineWidth = data.trackWidth;
      s.lineJoin = "round";
      s.lineCap = "round";
      s.beginPath();
      s.moveTo(poly[0][0], poly[0][1]);
      for (var i = 1; i < poly.length; i++) s.lineTo(poly[i][0], poly[i][1]);
      s.closePath();
      s.stroke();
    }
  }

  // --- drawing one frame -----------------------------------------------------
  // Reads two stored frames and blends their poses. No motion is computed; this
  // is a straight weighted average of values the engine already wrote.
  function draw() {
    if (!data || !staticLayer) return;
    ctx.drawImage(staticLayer, 0, 0);

    var frames = data.frames;
    var n = frames.length;
    if (n === 0) return;

    var i0 = Math.floor(cursor) % n;
    var i1 = (i0 + 1) % n;
    var frac = cursor - Math.floor(cursor);
    var a = frames[i0];
    var b = frames[i1];

    // Linear blend of two precomputed poses (rendering smoothing only).
    var x = a.x + (b.x - a.x) * frac;
    var y = a.y + (b.y - a.y) * frac;
    var theta = blendAngle(a.theta, b.theta, frac);

    // Sensor dots and states: use the discrete frame the cursor sits on, so the
    // on/off read-out is exactly what the engine recorded (never interpolated).
    drawCar(x, y, theta);
    drawSensor(a.sLx, a.sLy, a.onL);
    drawSensor(a.sRx, a.sRy, a.onR);

    updateReadout(a.onL, a.onR);
    frameVal.textContent = (i0 + 1) + " / " + n;
  }

  // Shortest-arc blend so the heading does not spin the long way around at the
  // figure-8 crossing. Still pure interpolation between two stored angles.
  function blendAngle(a, b, frac) {
    var d = b - a;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    return a + d * frac;
  }

  function rot(x, y, dx, dy, c, s) {
    return [x + dx * c - dy * s, y + dx * s + dy * c];
  }

  function drawCar(x, y, theta) {
    var c = Math.cos(theta), s = Math.sin(theta);
    var hl = data.carLength / 2, hw = data.carWidth / 2;
    var body = [
      rot(x, y, -hl, -hw, c, s), rot(x, y, hl, -hw, c, s),
      rot(x, y, hl, hw, c, s), rot(x, y, -hl, hw, c, s)
    ];
    ctx.beginPath();
    ctx.moveTo(body[0][0], body[0][1]);
    for (var i = 1; i < body.length; i++) ctx.lineTo(body[i][0], body[i][1]);
    ctx.closePath();
    ctx.fillStyle = COL.carFill;
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = COL.carEdge;
    ctx.stroke();

    // nose triangle so the heading is obvious
    var nose = [
      rot(x, y, hl, 0, c, s), rot(x, y, hl - 8, -6, c, s), rot(x, y, hl - 8, 6, c, s)
    ];
    ctx.beginPath();
    ctx.moveTo(nose[0][0], nose[0][1]);
    ctx.lineTo(nose[1][0], nose[1][1]);
    ctx.lineTo(nose[2][0], nose[2][1]);
    ctx.closePath();
    ctx.fillStyle = COL.nose;
    ctx.fill();
  }

  function drawSensor(px, py, on) {
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fillStyle = on ? COL.on : COL.off;
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = COL.dotEdge;
    ctx.stroke();
  }

  function updateReadout(onL, onR) {
    badgeL.textContent = t("left") + ": " + (onL ? t("on") : t("off"));
    badgeR.textContent = t("right") + ": " + (onR ? t("on") : t("off"));
    badgeL.className = "badge " + (onL ? "on" : "off");
    badgeR.className = "badge " + (onR ? "on" : "off");
  }

  // --- animation loop --------------------------------------------------------
  function tick(ts) {
    raf = requestAnimationFrame(tick);
    if (!data) return;
    if (!lastTs) lastTs = ts;
    var dt = ts - lastTs;
    lastTs = ts;

    if (playing) {
      // advance the frame cursor at roughly `speed` engine-frames per 16ms
      cursor += speed * (dt / 16.0);
      var n = data.frames.length;
      if (cursor >= n) cursor -= n; // loop
    }
    draw();
  }

  // --- controls --------------------------------------------------------------
  function setPlaying(p) {
    playing = p;
    playBtn.textContent = playing ? t("pause") : t("play");
  }

  playBtn.addEventListener("click", function () { setPlaying(!playing); });

  restartBtn.addEventListener("click", function () {
    cursor = 0;
    setPlaying(true);
    draw();
  });

  speedInput.addEventListener("input", function () {
    speed = parseFloat(speedInput.value);
    speedVal.textContent = speed.toFixed(2) + "×";
  });

  trackBtns.forEach(function (btn) {
    btn.addEventListener("click", function () {
      trackBtns.forEach(function (b) { b.classList.remove("active"); });
      btn.classList.add("active");
      load(btn.getAttribute("data-track"));
    });
  });

  // Re-render the read-out / button labels when the UI language toggles.
  window.refreshDynamicLang = function () {
    setPlaying(playing);
    if (data) {
      var i0 = Math.floor(cursor) % data.frames.length;
      var f = data.frames[i0];
      updateReadout(f.onL, f.onR);
    }
  };

  // --- boot ------------------------------------------------------------------
  speed = parseFloat(speedInput.value);
  speedVal.textContent = speed.toFixed(2) + "×";
  setPlaying(true);
  load("oval").then(function () {
    raf = requestAnimationFrame(tick);
  });
})();
