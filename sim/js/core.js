/* ForestStark Sim Lab — shared kit.
   Maths, soil physics (same constants and formulas as the Gazebo systems in fs_gazebo),
   canvas plotting, and the three.js scene parts and robot models. */
(() => {
const FS = window.FS = {};

/* ---------------- maths ---------------- */
const clamp = FS.clamp = (v, a, b) => v < a ? a : v > b ? b : v;
FS.lerp = (a, b, u) => a + (b - a) * u;
FS.wrap = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };
FS.ease = u => u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u);
FS.deg = r => r * 180 / Math.PI;
FS.rad = d => d * Math.PI / 180;
FS.rng = seed => () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
FS.gauss = r => { let u = 0; while (!u) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r()); };
FS.noise2 = (x, z) => Math.sin(x * .31 + 1.3) * Math.cos(z * .37 - .4) * .55 + Math.sin(x * .83 + z * .61) * .28 + Math.sin(x * 1.9 - z * 1.47 + 2.1) * .12 + Math.cos(x * 3.3 + z * 2.9) * .05;
FS.fmtT = s => { s = Math.max(0, Math.floor(s)); const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, ss = s % 60; return (h ? h + ':' + String(m).padStart(2, '0') : m) + ':' + String(ss).padStart(2, '0'); };
FS.n = (v, d = 0) => Number.isFinite(v) ? v.toLocaleString('en-IN', { minimumFractionDigits: d, maximumFractionDigits: d }) : '—';
FS.esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
FS.G = 9.81;

/* Ornstein–Uhlenbeck process: the simulated driver's weave and GNSS wander. */
FS.OU = (sigma, tau, r) => { let x = 0; return dt => { const a = Math.exp(-dt / tau); x = a * x + sigma * Math.sqrt(1 - a * a) * FS.gauss(r); return x; }; };

/* Polyline with arc length; lateral > 0 to the left of travel. */
FS.Path = class {
  constructor(pts) {
    this.p = pts.map(q => [q[0], q[1]]);
    this.s = [0]; this.d = [];
    for (let i = 1; i < this.p.length; i++) {
      const dx = this.p[i][0] - this.p[i - 1][0], dy = this.p[i][1] - this.p[i - 1][1], L = Math.hypot(dx, dy) || 1e-9;
      this.d.push([dx / L, dy / L, L]); this.s.push(this.s[i - 1] + L);
    }
    this.length = this.s[this.s.length - 1];
  }
  seg(s) { let lo = 0, hi = this.d.length - 1; while (lo < hi) { const m = (lo + hi + 1) >> 1; if (this.s[m] <= s) lo = m; else hi = m - 1; } return lo; }
  at(s) { s = clamp(s, 0, this.length); const i = this.seg(s), u = s - this.s[i], d = this.d[i]; return [this.p[i][0] + d[0] * u, this.p[i][1] + d[1] * u, Math.atan2(d[1], d[0])]; }
  project(x, y, hint = null, win = 80) {
    const n = this.d.length, lo = hint == null ? 0 : Math.max(0, hint - win), hi = hint == null ? n : Math.min(n, hint + win);
    let best = 1e18, bs = 0, bl = 0, bi = lo;
    for (let i = lo; i < hi; i++) {
      const a = this.p[i], d = this.d[i], rx = x - a[0], ry = y - a[1], t = clamp(rx * d[0] + ry * d[1], 0, d[2]);
      const fx = a[0] + d[0] * t, fy = a[1] + d[1] * t, e = (x - fx) ** 2 + (y - fy) ** 2;
      if (e < best) { best = e; bs = this.s[i] + t; bl = d[0] * ry - d[1] * rx; bi = i; }
    }
    return { s: bs, lat: bl, i: bi };
  }
  offset(dist) {
    return new FS.Path(this.p.map((q, i) => {
      const a = this.d[Math.max(0, i - 1)], b = this.d[Math.min(this.d.length - 1, i)];
      let nx = -(a[1] + b[1]), ny = a[0] + b[0]; const L = Math.hypot(nx, ny) || 1; nx /= L; ny /= L;
      const m = 1 / Math.max(.3, nx * -b[1] + ny * b[0]);
      return [q[0] + nx * dist * m, q[1] + ny * dist * m];
    }));
  }
};

/* ---------------- soil physics: mirrors fs_gazebo/include/fs_gazebo/SoilModel.hh ---------------- */
// Bekker constants (Wong) + specific boring energy, MJ/m^3.
FS.SOILS = {
  dry_sand:         { label: 'Dry sand',         n: 1.10, kc: 0.99,  kphi: 1528.43, c: 1.04,  phi: 28, K: .025, eb: 2.5 },
  sandy_loam:       { label: 'Sandy loam',       n: 0.90, kc: 52.53, kphi: 1127.97, c: 4.83,  phi: 20, K: .025, eb: 4.0 },
  clayey_soil:      { label: 'Clayey soil',      n: 0.50, kc: 13.19, kphi: 692.15,  c: 4.14,  phi: 13, K: .020, eb: 9.0 },
  heavy_clay:       { label: 'Heavy clay',       n: 0.13, kc: 12.70, kphi: 1555.95, c: 68.95, phi: 34, K: .010, eb: 14.0 },
  lateritic_gravel: { label: 'Lateritic gravel', n: 0.80, kc: 30.00, kphi: 4000.0,  c: 10.0,  phi: 35, K: .015, eb: 22.0 },
};
const S = FS.soil = {
  modulus: (s, b) => (s.kc / Math.max(b, 1e-3) + s.kphi) * 1e3,
  sinkage: (s, W, b, l) => W <= 0 ? 0 : Math.pow(W / (b * l) / S.modulus(s, b), 1 / s.n),
  compaction: (s, z, b) => z <= 0 ? 0 : b * S.modulus(s, b) * Math.pow(z, s.n + 1) / (s.n + 1),
  shearCap: (s, W, b, l) => Math.max(0, b * l * s.c * 1e3 + Math.max(W, 0) * Math.tan(s.phi * Math.PI / 180)),
  shearFrac: (i, l, K) => { i = clamp(Math.abs(i), 0, 1); const a = i * l / Math.max(K, 1e-4); return a < 1e-6 ? a / 2 : 1 - (1 - Math.exp(-a)) / a; },
  drawbar: (s, loads, b, l, i) => loads.reduce((acc, W) => acc + S.shearCap(s, W, b, l) * S.shearFrac(i, l, s.K) - S.compaction(s, S.sinkage(s, W, b, l), b), 0),
  slipFor: (s, loads, b, l, need) => {
    if (S.drawbar(s, loads, b, l, 1) < need) return 1;
    let lo = 0, hi = 1; for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (S.drawbar(s, loads, b, l, m) >= need) hi = m; else lo = m; } return hi;
  },
  moisture: m => clamp(1.6 - 2.4 * m, .7, 1.6),
  /* Auger in a material of specific energy e (J/m^3): power- and thrust-limited feed. */
  bore: (head, e) => {
    const A = Math.PI * head.D * head.D / 4, epm = Math.max(e, 1) * A;
    const fp = head.eta * head.P / epm; let feed = Math.min(head.feedMax, fp), thrust = .012 * epm, tl = false;
    if (thrust > head.thrustMax) { feed *= head.thrustMax / thrust; thrust = head.thrustMax; tl = true; }
    const power = epm * feed / head.eta;
    return { feed, thrust, power, torque: power * head.eta / (head.rpm * 2 * Math.PI / 60), thrustLimited: tl, powerLimited: fp < head.feedMax };
  },
};
/* The three heads, as in the robot URDFs. */
FS.HEADS = {
  katar:  { D: .15, P: 8000, eta: .75, thrustMax: 8000, feedMax: .15, rpm: 90,  free: .5,  retract: .6, rest: .75, place: 1.2, close: 1.5, pump: .5 },
  jaal:   { D: .12, P: 4000, eta: .85, thrustMax: 3000, feedMax: .15, rpm: 140, free: .25, retract: .3, rest: .58, place: 1.0, close: 1.2, pump: .4 },
  pahadi: { D: .12, P: 2000, eta: .80, thrustMax: 2500, feedMax: .12, rpm: 110, free: .25, retract: .3, rest: .08, place: 2.0, close: 1.5, pump: .3 },
};
/* One planting cycle as a list of phases [name, seconds] plus energy, from the soil. */
FS.cycle = (headName, soil, moisture, depth, water, extra = {}) => {
  const h = { ...FS.HEADS[headName], ...(extra.head || {}) };
  const e = soil.eb * 1e6 * S.moisture(moisture), r = S.bore(h, e);
  const bore = depth / Math.max(r.feed, 1e-4);
  const phases = [];
  if (extra.locate) phases.push(['locate', extra.locate]);
  phases.push(['lower', h.rest / h.free], ['bore', bore], ['retract', (h.rest + depth) / h.retract],
    ['place', h.place], ['close', h.close], ['water', water / h.pump]);
  if (extra.halfmoon) phases.push(['half-moon', extra.halfmoon]);
  if (extra.stow) phases.push(['stow', extra.stow]);
  return { phases, total: phases.reduce((a, p) => a + p[1], 0), bore, energy: e * Math.PI * h.D * h.D / 4 * depth / h.eta / 1000, rate: r };
};

/* Force-angle tip-over margins (Papadopoulos & Rey). contacts: [[x,y,z]...] gravity-aligned (z up). */
FS.convexHull = pts => {
  const p = pts.map((q, i) => [q[0], q[1], i]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p.map(q => q[2]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1)).map(q => q[2]);
};
FS.forceAngle = (contacts, cg, force) => {
  const idx = FS.convexHull(contacts); if (idx.length < 3) return { idx, margins: [] };
  const H = idx.map(i => contacts[i]), cen = [0, 1, 2].map(k => H.reduce((a, q) => a + q[k], 0) / H.length);
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s], nrm = a => { const L = Math.hypot(...a) || 1; return scl(a, 1 / L); };
  const perp = (v, a) => sub(v, scl(a, dot(v, a)));
  const margins = H.map((p1, i) => {
    const a = nrm(sub(H[(i + 1) % H.length], p1)), l = perp(sub(cg, p1), a), fp = perp(force, a);
    const up = nrm(scl(fp, -1)); let w = perp(sub(cen, p1), a); w = nrm(sub(w, scl(up, dot(w, up))));
    return Math.atan2(dot(l, w), dot(l, up));
  });
  return { idx, margins };
};

/* ---------------- theme ---------------- */
const TH = FS.TH = {};
FS.readTheme = () => {
  const cs = getComputedStyle(document.documentElement);
  ['bg', 'panel', 'soft', 'ink', 'muted', 'line', 'accent', 'mark', 'leaf', 'water', 'warn', 'bad', 'ok', 'c4', 'c5', 'plot', 'sky']
    .forEach(k => TH[k] = cs.getPropertyValue('--' + k).trim());
  TH.dark = (cs.getPropertyValue('color-scheme') || '').includes('dark');
};

/* ---------------- canvas plotting ---------------- */
FS.fit = cv => {
  const r = cv.getBoundingClientRect(), dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = Math.max(1, Math.round(r.width)), h = Math.max(1, Math.round(r.height));
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const c = cv.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
  return { c, w, h };
};
const FONT = { mono: '"IBM Plex Mono",ui-monospace,monospace', body: '"IBM Plex Sans",system-ui,sans-serif', display: '"Barlow Condensed","Arial Narrow",sans-serif' };
FS.text = (c, s, x, y, o = {}) => {
  c.save(); c.fillStyle = o.color || TH.ink; c.font = `${o.weight || 400} ${o.size || 11}px ${FONT[o.font || 'mono']}`;
  c.textAlign = o.align || 'left'; c.textBaseline = o.base || 'alphabetic';
  if (o.halo) { c.lineWidth = 3; c.strokeStyle = TH.plot; c.lineJoin = 'round'; c.strokeText(s, x, y); }
  c.fillText(s, x, y); c.restore();
};
FS.line = (c, pts, color, w = 1.5, dash = null) => {
  if (pts.length < 2) return; c.save(); c.strokeStyle = color; c.lineWidth = w; c.lineJoin = 'round'; c.lineCap = 'round';
  if (dash) c.setLineDash(dash); c.beginPath(); c.moveTo(pts[0][0], pts[0][1]); for (let i = 1; i < pts.length; i++) c.lineTo(pts[i][0], pts[i][1]); c.stroke(); c.restore();
};
FS.poly = (c, pts, fill, stroke, w = 1) => {
  if (pts.length < 2) return; c.save(); c.beginPath(); c.moveTo(pts[0][0], pts[0][1]); for (let i = 1; i < pts.length; i++) c.lineTo(pts[i][0], pts[i][1]); c.closePath();
  if (fill) { c.fillStyle = fill; c.fill(); } if (stroke) { c.strokeStyle = stroke; c.lineWidth = w; c.stroke(); } c.restore();
};
FS.circle = (c, x, y, r, fill, stroke, w = 1) => { c.save(); c.beginPath(); c.arc(x, y, r, 0, Math.PI * 2); if (fill) { c.fillStyle = fill; c.fill(); } if (stroke) { c.strokeStyle = stroke; c.lineWidth = w; c.stroke(); } c.restore(); };
FS.arrow = (c, x1, y1, x2, y2, color, w = 1.6, hs = 7) => {
  const a = Math.atan2(y2 - y1, x2 - x1), L = Math.hypot(x2 - x1, y2 - y1); if (L < 1) return;
  FS.line(c, [[x1, y1], [x2 - Math.cos(a) * hs * .6, y2 - Math.sin(a) * hs * .6]], color, w);
  FS.poly(c, [[x2, y2], [x2 - Math.cos(a - .42) * hs, y2 - Math.sin(a - .42) * hs], [x2 - Math.cos(a + .42) * hs, y2 - Math.sin(a + .42) * hs]], color);
};
FS.alpha = (col, a) => {
  if (!col) return col; if (col.startsWith('#') && col.length === 7) return col + Math.round(a * 255).toString(16).padStart(2, '0');
  return col;
};
FS.niceTicks = (lo, hi, n = 5) => {
  if (!(hi > lo)) hi = lo + 1; const span = hi - lo, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 2.5, 5, 10].map(k => k * mag).find(s => s >= step0) || mag * 10;
  const t = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) t.push(+v.toFixed(10)); return { t, step };
};
/* Axes frame: returns scale functions. box = {x,y,w,h}; xr/yr = [lo,hi]. */
FS.frame = (c, box, xr, yr, o = {}) => {
  const X = v => box.x + (v - xr[0]) / (xr[1] - xr[0]) * box.w, Y = v => box.y + box.h - (v - yr[0]) / (yr[1] - yr[0]) * box.h;
  const xt = FS.niceTicks(xr[0], xr[1], o.xn || 5), yt = FS.niceTicks(yr[0], yr[1], o.yn || 4);
  c.save(); c.strokeStyle = FS.alpha(TH.line, 1); c.lineWidth = 1;
  yt.t.forEach(v => { const y = Math.round(Y(v)) + .5; c.beginPath(); c.moveTo(box.x, y); c.lineTo(box.x + box.w, y); c.stroke();
    FS.text(c, o.yfmt ? o.yfmt(v) : FS.n(v, yt.step < 1 ? (yt.step < .1 ? 2 : 1) : 0), box.x - 6, y + 3.5, { color: TH.muted, align: 'right', size: 10.5 }); });
  if (!o.noX) xt.t.forEach(v => { const x = Math.round(X(v)) + .5; FS.text(c, o.xfmt ? o.xfmt(v) : FS.n(v, xt.step < 1 ? 1 : 0), x, box.y + box.h + 14, { color: TH.muted, align: 'center', size: 10.5 }); });
  c.strokeStyle = TH.muted; c.beginPath(); c.moveTo(box.x + .5, box.y); c.lineTo(box.x + .5, box.y + box.h + .5); c.lineTo(box.x + box.w, box.y + box.h + .5); c.stroke(); c.restore();
  if (o.xlabel) FS.text(c, o.xlabel, box.x + box.w, box.y + box.h + 28, { color: TH.muted, align: 'right', size: 10.5 });
  if (o.ylabel) FS.text(c, o.ylabel, box.x - 6, box.y - 8, { color: TH.muted, align: 'left', size: 10.5 });
  return { X, Y };
};
FS.series = (c, X, Y, data, color, o = {}) => {
  if (data.length < 2) return; const pts = data.map(d => [X(d[0]), Y(d[1])]);
  if (o.area) { const base = Y(o.base ?? 0); FS.poly(c, [[pts[0][0], base], ...pts, [pts[pts.length - 1][0], base]], FS.alpha(color, .16)); }
  FS.line(c, pts, color, o.w || 1.8, o.dash);
  if (o.dot !== false) { const p = pts[pts.length - 1]; FS.circle(c, p[0], p[1], 3.2, color, TH.plot, 1.5); }
};
FS.legend = (c, items, x, y, maxW = Infinity) => {
  let cx = x, cy = y; c.font = `400 10.5px ${FONT.mono}`;
  items.forEach(([label, color, dash]) => {
    const wd = 26 + c.measureText(label).width; if (cx + wd > x + maxW && cx > x) { cx = x; cy += 14; }
    FS.line(c, [[cx, cy - 3.5], [cx + 14, cy - 3.5]], color, 2.2, dash); FS.text(c, label, cx + 18, cy, { color: TH.muted, size: 10.5 }); cx += wd;
  });
  return cy;
};
/* Swatch legend (filled squares) that wraps; returns the y of its last line. */
FS.swatches = (c, items, x, y, maxW) => {
  let cx = x, cy = y; c.font = `400 10px ${FONT.mono}`;
  items.forEach(([label, color]) => { const wd = 18 + c.measureText(label).width; if (cx + wd > x + maxW && cx > x) { cx = x; cy += 14; }
    FS.poly(c, [[cx, cy - 8], [cx + 9, cy - 8], [cx + 9, cy + 1], [cx, cy + 1]], color); FS.text(c, label, cx + 12, cy, { color: TH.muted, size: 10 }); cx += wd; });
  return cy;
};
FS.chip = (c, s, x, y, color) => { c.font = `500 10px ${FONT.mono}`; const w = c.measureText(s).width + 10; FS.poly(c, [[x, y - 11], [x + w, y - 11], [x + w, y + 4], [x, y + 4]], FS.alpha(color, .14), color); FS.text(c, s, x + 5, y, { color, size: 10, weight: 500 }); return w; };

/* ---------------- three.js kit ---------------- */
const T3 = FS.T3 = { ok: false };
FS.init3D = host => {
  if (!window.THREE) return false;
  try { T3.renderer = new THREE.WebGLRenderer({ antialias: true }); } catch (e) { return false; }
  const r = T3.renderer;
  r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2)); r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
  r.domElement.className = 'gl'; host.insertBefore(r.domElement, host.firstChild);
  T3.scene = new THREE.Scene(); T3.camera = new THREE.PerspectiveCamera(40, 1.6, .1, 600);
  T3.controls = FS.orbit(T3.camera, r.domElement);
  T3.scene.add(new THREE.HemisphereLight(0xEAF2F6, 0x6B5A40, .78));
  const sun = T3.sun = new THREE.DirectionalLight(0xFFF1DC, 1.05);
  sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -18, right: 18, top: 18, bottom: -18, near: 1, far: 120 }); sun.shadow.bias = -.0006;
  T3.scene.add(sun, sun.target); T3.world = new THREE.Group(); T3.scene.add(T3.world);
  T3.ok = true; FS.makeMaterials(); return true;
};
FS.orbit = (cam, el) => {
  const o = { target: new THREE.Vector3(), minDistance: 3, maxDistance: 180, maxPolar: Math.PI * .485, dT: 0, dP: 0, zoom: 1, pan: new THREE.Vector3(), user: 0 };
  const sph = new THREE.Spherical(), off = new THREE.Vector3(), right = new THREE.Vector3(), upv = new THREE.Vector3();
  o.update = () => {
    off.copy(cam.position).sub(o.target); sph.setFromVector3(off); sph.theta += o.dT;
    sph.phi = clamp(sph.phi + o.dP, .08, o.maxPolar); sph.radius = clamp(sph.radius * o.zoom, o.minDistance, o.maxDistance);
    o.target.add(o.pan); cam.position.copy(o.target).add(off.setFromSpherical(sph)); cam.lookAt(o.target);
    o.dT *= .8; o.dP *= .8; if (Math.abs(o.dT) < 1e-5) o.dT = 0; if (Math.abs(o.dP) < 1e-5) o.dP = 0; o.zoom = 1; o.pan.set(0, 0, 0);
  };
  const panBy = (dx, dy) => { const d = cam.position.distanceTo(o.target), k = 2 * d * Math.tan(cam.fov * Math.PI / 360) / (el.clientHeight || 1);
    right.setFromMatrixColumn(cam.matrix, 0); upv.setFromMatrixColumn(cam.matrix, 1); o.pan.addScaledVector(right, -dx * k).addScaledVector(upv, dy * k); };
  const pts = new Map(); let mode = null, lastPinch = 0, lastMid = null;
  el.style.touchAction = 'none'; el.addEventListener('contextmenu', e => e.preventDefault());
  el.addEventListener('pointerdown', e => { el.setPointerCapture(e.pointerId); pts.set(e.pointerId, { x: e.clientX, y: e.clientY }); o.user = performance.now();
    mode = pts.size === 2 ? 'pinch' : (e.button === 2 || e.shiftKey ? 'pan' : 'rotate');
    if (mode === 'pinch') { const [a, b] = [...pts.values()]; lastPinch = Math.hypot(a.x - b.x, a.y - b.y); lastMid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }; } });
  el.addEventListener('pointermove', e => { const p = pts.get(e.pointerId); if (!p) return; const dx = e.clientX - p.x, dy = e.clientY - p.y; p.x = e.clientX; p.y = e.clientY; o.user = performance.now();
    if (mode === 'rotate') { const h = el.clientHeight || 1; o.dT -= Math.PI * dx / h; o.dP -= Math.PI * dy / h; }
    else if (mode === 'pan') panBy(dx, dy);
    else if (mode === 'pinch' && pts.size === 2) { const [a, b] = [...pts.values()], d = Math.hypot(a.x - b.x, a.y - b.y), mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      if (lastPinch > 0) o.zoom *= lastPinch / d; lastPinch = d; panBy(mid.x - lastMid.x, mid.y - lastMid.y); lastMid = mid; } });
  const end = e => { pts.delete(e.pointerId); mode = pts.size === 1 ? 'rotate' : null; };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
  el.addEventListener('wheel', e => { e.preventDefault(); o.user = performance.now(); o.zoom *= Math.exp(e.deltaY * .0012); }, { passive: false });
  return o;
};
const MAT = FS.MAT = {};
FS.makeMaterials = () => {
  const m = (c, o = {}) => new THREE.MeshStandardMaterial(Object.assign({ color: c, roughness: .65, metalness: .05 }, o));
  Object.assign(MAT, {
    yellow: m(0xE2A514, { roughness: .5 }), graphite: m(0x303633, { roughness: .75 }), steel: m(0x8F999B, { metalness: .55, roughness: .35 }),
    rubber: m(0x1B1D1C, { roughness: .95 }), glass: m(0xA9CCD6, { transparent: true, opacity: .28, roughness: .1 }), tankBlue: m(0x2E6B8E, { roughness: .5 }),
    tray: m(0x262A28, { roughness: .8 }), leaf: m(0x4F9A3F, { roughness: .8, flatShading: true }), leaf2: m(0x3C7F36, { roughness: .8, flatShading: true }),
    leafI: m(0xFFFFFF, { roughness: .8, flatShading: true }), stem: m(0x6C4A2C), soilDug: m(0x5E432B, { roughness: 1 }), solar: m(0x1E3354, { metalness: .35, roughness: .3 }),
    lens: m(0x101214, { roughness: .2, metalness: .4 }), beacon: m(0xFF7A1A, { emissive: 0xFF5A00, emissiveIntensity: .7 }), lamp: m(0xFFF6D8, { emissive: 0xFFF0C0, emissiveIntensity: .5 }),
    shade: m(0x355E3B, { transparent: true, opacity: .55, side: THREE.DoubleSide, roughness: 1 }), skin: m(0xB98058), cloth: m(0x3A5A8C), cloth2: m(0x8C5A3A),
    hopper: m(0xB07C46, { transparent: true, opacity: .85 }), white: m(0xEDEDE6), rock: m(0x8A8378, { roughness: .95, flatShading: true }), seed: m(0x7B5235, { roughness: 1 }),
    asphalt: m(0x4A4C4E, { roughness: .95 }), gravel: m(0xA79C88, { roughness: 1 }), paint: m(0xEDEDE6, { roughness: .8 }), orange: m(0xE06A1B), milestone: m(0xE8B321),
    trunk: m(0x5A4330, { roughness: .9 }), tree: m(0x5E7D3C, { roughness: .9, flatShading: true }), dry: m(0x8F8A4C, { roughness: 1 }), cab: m(0xE9ECE8, { roughness: .5 }),
    mound: m(0x9E6B45, { roughness: 1, flatShading: true }), cow: m(0xD9D1C2, { roughness: .9 }), concrete: m(0xB8B6AE, { roughness: .95 }),
    ring: new THREE.MeshBasicMaterial({ color: 0xFFFFFF, transparent: true, opacity: .9, side: THREE.DoubleSide }),
    zone: new THREE.MeshBasicMaterial({ color: 0xC0392B, transparent: true, opacity: .22, side: THREE.DoubleSide, depthWrite: false }),
    zoneHidden: new THREE.MeshBasicMaterial({ color: 0x7B3FA0, transparent: true, opacity: .16, side: THREE.DoubleSide, depthWrite: false }),
    safety: new THREE.MeshBasicMaterial({ color: 0xE2A514, transparent: true, opacity: .25, side: THREE.DoubleSide, depthWrite: false }),
  });
};
FS.B = (w, h, d) => new THREE.BoxGeometry(w, h, d);
FS.C = (rt, rb, h, s = 20) => new THREE.CylinderGeometry(rt, rb, h, s);
FS.add = (parent, geo, m, x = 0, y = 0, z = 0) => { const me = new THREE.Mesh(geo, m); me.position.set(x, y, z); me.castShadow = !(m && m.transparent); me.receiveShadow = true; parent.add(me); return me; };
const { add } = FS;
FS.wheel = (parent, r, w, x, y, z, rimMat, lugs) => {
  const B = FS.B, C = FS.C, g = new THREE.Group(); g.position.set(x, y, z); parent.add(g);
  const spin = new THREE.Group(); g.add(spin);
  let t = add(spin, C(r, r, w, 28), MAT.rubber); t.rotation.x = Math.PI / 2;
  t = add(spin, C(r * .62, r * .62, w * 1.04, 20), rimMat); t.rotation.x = Math.PI / 2;
  t = add(spin, C(r * .18, r * .18, w * 1.12, 12), MAT.steel); t.rotation.x = Math.PI / 2;
  for (let i = 0; i < 2; i++) { const sp = add(spin, B(r * 1.15, r * .09, w * 1.08), MAT.graphite); sp.rotation.z = i * Math.PI / 2; }
  for (let i = 0; i < (lugs || 0); i++) { const a = i / lugs * Math.PI * 2, l = add(spin, B(r * .16, r * .1, w * 1.02), MAT.rubber, Math.cos(a) * r, Math.sin(a) * r, 0); l.rotation.z = a; }
  return { g, spin, r };
};
FS.person = (parent, x, y, z, rotY, shirt) => {
  const B = FS.B, C = FS.C, g = new THREE.Group(); g.position.set(x, y, z); g.rotation.y = rotY || 0; parent.add(g);
  [-.1, .1].forEach(s => add(g, B(.14, .82, .16), MAT.graphite, 0, .41, s));
  add(g, C(.16, .18, .62, 12), shirt || MAT.cloth, 0, 1.13, 0);
  [-.25, .25].forEach(s => add(g, B(.1, .56, .1), shirt || MAT.cloth, 0, 1.13, s));
  add(g, new THREE.SphereGeometry(.115, 14, 10), MAT.skin, 0, 1.58, 0); return g;
};
FS.augerSet = (parent, len, flights) => {
  const B = FS.B, C = FS.C, aug = new THREE.Group(); parent.add(aug);
  add(aug, C(.034, .034, len, 8), MAT.steel, 0, -len / 2, 0);
  for (let i = 0; i < flights; i++) { const f = add(aug, B(.2, .018, .05), MAT.steel, 0, -.1 - i * (len - .15) / flights, 0); f.rotation.y = i * .7; }
  add(aug, C(.034, .002, .08, 8), MAT.steel, 0, -len - .04, 0); return aug;
};
FS.groundMesh = (w, d, sx, sz, hFn, colFn, cx = 0, cz = 0) => {
  const g = new THREE.PlaneGeometry(w, d, sx, sz); g.rotateX(-Math.PI / 2); g.translate(cx, 0, cz);
  const pos = g.attributes.position, cols = [], c = new THREE.Color();
  for (let i = 0; i < pos.count; i++) { const x = pos.getX(i), z = pos.getZ(i), h = hFn(x, z); pos.setY(i, h); c.setHex(colFn(x, z, h)); cols.push(c.r, c.g, c.b); }
  g.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3)); g.computeVertexNormals();
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1 })); m.receiveShadow = true; T3.world.add(m); return m;
};
FS.mixHex = (a, b, u) => { const ca = new THREE.Color(a), cb = new THREE.Color(b); return ca.lerp(cb, clamp(u, 0, 1)).getHex(); };
FS.ribbon = (path, lat0, lat1, y, mat) => {
  const pos = [], idx = [];
  path.p.forEach((q, i) => { const d = path.d[Math.min(i, path.d.length - 1)], nx = -d[1], ny = d[0];
    pos.push(q[0] + nx * lat0, y, -(q[1] + ny * lat0), q[0] + nx * lat1, y, -(q[1] + ny * lat1));
    if (i) { const a = 2 * i - 2; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); } });
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setIndex(idx); g.computeVertexNormals();
  const m = new THREE.Mesh(g, mat); m.receiveShadow = true; T3.world.add(m); return m;
};
FS.bigTree = (x, z, s, y = 0) => {
  add(T3.world, FS.C(.12 * s, .2 * s, 2.1 * s, 7), MAT.trunk, x, y + 1.05 * s, z);
  [[0, 2.5, 0, 1.4], [.8, 2.2, .3, 1.0], [-.7, 2.3, -.4, 1.1], [.1, 3.1, .2, .9]].forEach(([dx, dy, dz, rr]) => add(T3.world, new THREE.IcosahedronGeometry(rr * s, 1), MAT.tree, x + dx * s, y + dy * s, z + dz * s));
};
FS.makeLabel = (text, sub) => {
  const cv = document.createElement('canvas'); cv.width = 512; cv.height = sub ? 168 : 112; const x = cv.getContext('2d'), r = 14, w = cv.width, h = cv.height;
  x.fillStyle = 'rgba(26,35,31,.92)'; x.beginPath(); x.moveTo(r, 0); x.arcTo(w, 0, w, h, r); x.arcTo(w, h, 0, h, r); x.arcTo(0, h, 0, 0, r); x.arcTo(0, 0, w, 0, r); x.fill();
  x.fillStyle = '#F2F4EF'; x.font = '600 64px "Barlow Condensed","Arial Narrow",sans-serif'; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(text.toUpperCase(), w / 2, sub ? 62 : h / 2);
  if (sub) { x.fillStyle = '#E2A514'; x.font = '500 30px "IBM Plex Mono",monospace'; x.fillText(sub, w / 2, 126); }
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), depthTest: false })); sp.scale.set(2.4, 2.4 * cv.height / cv.width, 1); sp.renderOrder = 10; return sp;
};
/* Instanced pools: planted saplings and planned-spot rings. World coords: x east, y north -> three (x, h, -y). */
FS.PlantPool = class {
  constructor(cap) {
    this.cap = cap; this.n = 0; this.grow = [];
    const C = FS.C;
    const geo = { disc: C(.24, .27, .03, 14), stem: C(.012, .017, .48, 6).translate(0, .24, 0), c1: new THREE.IcosahedronGeometry(.14, 0).translate(0, .5, 0),
      c2: new THREE.IcosahedronGeometry(.1, 0).translate(.07, .38, .04), ball: new THREE.SphereGeometry(.05, 8, 6).translate(0, .04, 0), flag: FS.B(.14, .09, .01).translate(-.08, .46, 0),
      stick: C(.008, .008, .5, 4).translate(-.15, .25, 0), moon: new THREE.TorusGeometry(.45, .06, 5, 18, Math.PI).rotateX(Math.PI / 2).rotateY(-Math.PI / 2) };
    this.meshes = { disc: [geo.disc, MAT.soilDug], stem: [geo.stem, MAT.stem], c1: [geo.c1, MAT.leaf], c2: [geo.c2, MAT.leaf2],
      ball: [geo.ball, MAT.seed], flag: [geo.flag, MAT.orange], stick: [geo.stick, MAT.stem], moon: [geo.moon, MAT.soilDug] };
    for (const k in this.meshes) { const [g, m] = this.meshes[k], im = new THREE.InstancedMesh(g, m, cap); im.count = 0; im.castShadow = k !== 'disc' && k !== 'moon'; im.receiveShadow = true; im.frustumCulled = false; this.meshes[k] = im; T3.world.add(im); }
    this.items = [];
  }
  add(x, y, h, kind = 'sapling', yaw = 0) {
    if (this.items.length >= this.cap) return; const parts = kind === 'seed' ? ['ball', 'stick', 'flag'] : kind === 'halfmoon' ? ['moon'] : ['disc', 'stem', 'c1', 'c2'];
    const it = { x, y, h, parts, yaw, u: 0, idx: {} };
    parts.forEach(p => { const im = this.meshes[p]; it.idx[p] = im.count; im.count++; });
    this.items.push(it); this.grow.push(it); this.set(it, .05);
  }
  set(it, s) {
    const m = new THREE.Matrix4(), q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), it.yaw), v = new THREE.Vector3(it.x, it.h, -it.y), sc = new THREE.Vector3(s, s, s);
    m.compose(v, q, sc); it.parts.forEach(p => { this.meshes[p].setMatrixAt(it.idx[p], m); this.meshes[p].instanceMatrix.needsUpdate = true; });
  }
  update(dt) { for (let i = this.grow.length - 1; i >= 0; i--) { const it = this.grow[i]; it.u = Math.min(1, it.u + dt * 2.2); this.set(it, .05 + .95 * (1 - Math.pow(1 - it.u, 3))); if (it.u >= 1) this.grow.splice(i, 1); } }
};
FS.RingPool = class {
  constructor(spots, hFn) {
    const g = new THREE.RingGeometry(.13, .2, 18); g.rotateX(-Math.PI / 2);
    this.im = new THREE.InstancedMesh(g, MAT.ring, spots.length); this.im.frustumCulled = false; T3.world.add(this.im);
    const m = new THREE.Matrix4(); spots.forEach((s, i) => { m.makeTranslation(s.x, hFn(s.x, s.y) + .03, -s.y); this.im.setMatrixAt(i, m); this.color(i, s.skip ? 'skip' : 'todo'); });
  }
  color(i, state) { const c = { todo: 0xE2A514, done: 0x3E9A4C, skip: 0xC0392B, seed: 0x8B5A2B, warn: 0xB4600F }[state] || 0xE2A514; this.im.setColorAt(i, new THREE.Color(c)); if (this.im.instanceColor) this.im.instanceColor.needsUpdate = true; }
};

/* ---------------- robot models (sizes as in fs_description) ---------------- */
FS.buildKatar = () => {
  const B = FS.B, C = FS.C, r = { root: new THREE.Group(), spins: [] }, root = r.root;
  const T = new THREE.Group(); root.add(T);
  [-.8, .8].forEach(z => { const w = FS.wheel(T, .72, .4, 0, .72, z, MAT.yellow, 16); r.spins.push(w); });
  r.steer = [-.68, .68].map(z => { const piv = new THREE.Group(); piv.position.set(2.05, .42, z); T.add(piv); const w = FS.wheel(piv, .42, .24, 0, 0, 0, MAT.yellow, 0); r.spins.push(w); return piv; });
  add(T, B(2.5, .3, .6), MAT.graphite, 1.05, .64, 0); add(T, B(1.55, .7, .78), MAT.yellow, 1.88, 1.12, 0); add(T, B(.06, .58, .68), MAT.graphite, 2.67, 1.08, 0);
  [-.25, .25].forEach(z => add(T, B(.04, .09, .14), MAT.lamp, 2.7, 1.3, z)); add(T, C(.05, .05, .8, 10), MAT.graphite, 2.25, 1.85, .26);
  [-1, 1].forEach(s => add(T, B(1.15, .06, .5), MAT.yellow, 0, 1.5, s * .8)); add(T, B(1.25, .08, 1.2), MAT.graphite, 0, 1.12, 0);
  [-.55, .6].forEach(x => [-.57, .57].forEach(z => add(T, B(.06, 1.46, .06), MAT.graphite, x, 1.89, z)));
  add(T, B(1.35, .08, 1.3), MAT.yellow, .02, 2.64, 0); add(T, B(.02, 1.0, 1.08), MAT.glass, .6, 2.08, 0);
  [-.57, .57].forEach(z => add(T, B(1.1, .85, .02), MAT.glass, .02, 2.12, z));
  FS.person(T, -.12, .44 + .72 - .3, 0, 0, MAT.cloth); add(T, C(.07, .07, .12, 12), MAT.beacon, .3, 2.74, .42); add(T, B(.5, .12, .14), MAT.steel, -.85, .72, 0);
  // trailer on a hitch pivot 1.15 m behind the rear axle
  const H = r.hitch = new THREE.Group(); H.position.set(-1.15, 0, 0); root.add(H);
  const R = new THREE.Group(); R.position.x = 1.15; H.add(R);
  add(R, B(.9, .1, .1), MAT.steel, -1.6, .74, 0); add(R, B(3.7, .14, 2.0), MAT.graphite, -3.85, .86, 0);
  [-.98, .98].forEach(z => add(R, B(3.7, .12, .04), MAT.yellow, -3.85, .99, z));
  [-4.25, -5.1].forEach(x => [-.93, .93].forEach(z => r.spins.push(FS.wheel(R, .42, .26, x, .42, z, MAT.graphite, 0))));
  const tk = add(R, C(.55, .55, 1.2, 24), MAT.tankBlue, -2.75, 1.5, 0); tk.rotation.z = Math.PI / 2;
  r.waterLevel = add(R, B(1.18, .02, .3), MAT.white, -2.75, 2.06, 0);
  [-1, 1].forEach(s => { const zc = s * .68;
    [-3.48, -4.55, -5.62].forEach(x => [zc - .22, zc + .22].forEach(z => add(R, B(.05, 1.86, .05), MAT.steel, x, 1.86, z)));
    [.98, 1.5, 2.02].forEach(y => { add(R, B(2.14, .03, .46), MAT.steel, -4.55, y - .03, zc); add(R, B(2.06, .05, .4), MAT.tray, -4.55, y + .01, zc); }); });
  r.trays = []; [-1, 1].forEach(s => [.98, 1.5, 2.02].forEach(y => r.trays.push(add(R, B(2.0, .07, .36), MAT.leaf, -4.55, y + .07, s * .68))));
  add(R, B(2.2, .02, 2.0), MAT.shade, -4.55, 2.8, 0); FS.person(R, -4.25, .93, 0, Math.PI / 2, MAT.cloth2);
  add(R, B(2.8, .08, .08), MAT.steel, -2.75, .95, -1.0);
  // boom: sled slides along x, carriage across (left = -z), auger feeds down
  const sled = r.sled = new THREE.Group(); sled.position.set(-2.0, 0, 0); R.add(sled);
  add(sled, B(.12, 1.15, .12), MAT.yellow, 0, 1.5, -1.0); add(sled, B(.12, .12, 2.75), MAT.yellow, 0, 2.02, -2.35);
  const br = add(sled, B(.06, .06, .95), MAT.steel, 0, 1.75, -1.4); br.rotation.x = .71;
  const carr = r.carr = new THREE.Group(); carr.position.set(0, 1.95, -2.5); sled.add(carr);
  add(carr, B(.32, .3, .32), MAT.yellow); add(carr, C(.09, .09, .2, 12), MAT.graphite, 0, .25, 0); add(carr, B(.04, .8, .2), MAT.graphite, .17, -.5, 0);
  r.aug = FS.augerSet(carr, .95, 9); r.aug.position.y = -.25;
  r.setTool = d => { r.aug.position.y = -.25 - d; }; r.spinTool = dt => { r.aug.rotation.y += dt * 9; };
  return r;
};
FS.buildJaal = () => {
  const B = FS.B, C = FS.C, r = { root: new THREE.Group(), spins: [], steer: [] }, root = r.root;
  add(root, B(2.8, .45, 1.3), MAT.yellow, 0, .6, 0); add(root, B(2.84, .1, 1.34), MAT.graphite, 0, .42, 0); add(root, B(2.7, .05, 1.24), MAT.graphite, 0, .85, 0);
  [[.95, 1], [.95, -1], [-.95, 1], [-.95, -1]].forEach(([x, s]) => {
    const piv = new THREE.Group(); piv.position.set(x, .38, s * .83); root.add(piv); add(piv, B(.16, .3, .12), MAT.graphite, 0, .24, 0);
    r.spins.push(FS.wheel(piv, .38, .22, 0, 0, 0, MAT.graphite, 10)); r.steer.push(piv); add(root, B(.86, .05, .3), MAT.yellow, x, .86, s * .83);
  });
  [1, -1].forEach(s => { add(root, B(.1, .2, 1.2), MAT.graphite, s * 1.45, .5, 0); add(root, B(.02, .05, 1.0), MAT.lens, s * 1.505, .55, 0); });
  r.drums = [.55, -.43].map(cx => { const d = new THREE.Group(); d.position.set(cx, .88, 0); root.add(d);
    add(d, C(.3, .34, .06, 20), MAT.graphite); add(d, C(.05, .05, 1.72, 8), MAT.steel, 0, .86, 0);
    for (let k = 0; k < 5; k++) add(d, C(.48, .48, .025, 36), MAT.graphite, 0, .05 + k * .33, 0);
    const fill = add(d, C(.43, .43, 1.6, 24), MAT.leaf, 0, .84, 0); d.userData.fill = fill; add(d, C(.12, .12, .05, 12), MAT.yellow, 0, 1.74, 0); return d; });
  [-1.12, 1.12].forEach(x => [-.6, .6].forEach(z => add(root, B(.05, 1.9, .05), MAT.graphite, x, 1.83, z)));
  add(root, B(2.54, .03, 1.4), MAT.graphite, 0, 2.77, 0); add(root, B(2.5, .05, 1.36), MAT.solar, 0, 2.8, 0);
  add(root, C(.03, .03, .5, 8), MAT.graphite, 1.15, 3.05, 0); add(root, C(.09, .09, .02, 20), MAT.yellow, 1.15, 3.3, 0);
  r.lidar = add(root, C(.085, .085, .11, 20), MAT.lens, 1.15, 3.36, 0); add(root, C(.06, .06, .1, 12), MAT.beacon, -1.0, 2.88, -.5);
  [-.45, .45].forEach(z => add(root, B(.08, .8, .08), MAT.graphite, -1.38, 1.25, z)); add(root, B(.1, .1, 1.0), MAT.graphite, -1.38, 1.68, 0); add(root, B(.34, .1, .12), MAT.yellow, -1.53, 1.62, 0);
  const carr = new THREE.Group(); carr.position.set(-1.65, 1.5, 0); root.add(carr);
  add(carr, B(.24, .26, .24), MAT.yellow); add(carr, C(.07, .07, .16, 12), MAT.graphite, 0, .2, 0); add(carr, C(.015, .015, .25, 6), MAT.tankBlue, -.11, -.22, .1);
  r.aug = FS.augerSet(carr, .72, 7); r.aug.position.y = -.2;
  r.setTool = d => { r.aug.position.y = -.2 - d; }; r.spinTool = dt => { r.aug.rotation.y += dt * 14; };
  const sf = new THREE.RingGeometry(1.5 + 1.45, 3.0 + 1.45, 48, 1, -Math.PI / 2.4, Math.PI / 1.2); sf.rotateX(-Math.PI / 2);
  r.safety = add(root, sf, MAT.safety, 0, .03, 0); r.safety.visible = false;
  return r;
};
FS.buildPahadi = () => {
  const B = FS.B, C = FS.C, r = { F: new THREE.Group(), Rm: new THREE.Group(), joint: new THREE.Group(), spins: [] };
  const track = (P, z) => { add(P, B(1.1, .42, .28), MAT.rubber, 0, .21, z);
    [-.55, .55].forEach(x => { const c = add(P, C(.21, .21, .28, 16), MAT.rubber, x, .21, z); c.rotation.x = Math.PI / 2; });
    [-.4, 0, .4].forEach(x => { const w = new THREE.Group(); w.position.set(x, .21, z); P.add(w); const c = add(w, C(.12, .12, .3, 12), MAT.steel); c.rotation.x = Math.PI / 2; add(w, B(.2, .03, .305), MAT.graphite); r.spins.push({ spin: w, r: .12 }); });
    add(P, B(1.5, .04, .32), MAT.graphite, 0, .45, z); };
  [r.F, r.Rm].forEach(P => { [-.5, .5].forEach(z => track(P, z)); });
  add(r.F, B(1.3, .4, .72), MAT.yellow, 0, .63, 0); add(r.F, B(1.32, .05, .74), MAT.graphite, 0, .84, 0);
  [-.25, .25].forEach(z => add(r.F, B(.03, .07, .12), MAT.lamp, .66, .72, z)); add(r.F, C(.025, .025, .6, 6), MAT.graphite, -.45, 1.15, .26); add(r.F, C(.09, .09, .03, 18), MAT.graphite, -.45, 1.48, .26);
  add(r.Rm, B(1.3, .4, .72), MAT.graphite, 0, .63, 0); add(r.Rm, B(1.32, .05, .74), MAT.yellow, 0, .84, 0);
  [.38, -.06].forEach(tx => [.88, 1.3].forEach(y => { add(r.Rm, B(.42, .05, .55), MAT.tray, tx, y, 0); }));
  r.trays = [.38, -.06].flatMap(tx => [.88, 1.3].map(y => add(r.Rm, B(.4, .06, .5), MAT.leaf, tx, y + .05, 0)));
  [.6, -.28].forEach(x => [-.3, .3].forEach(z => add(r.Rm, B(.04, .86, .04), MAT.steel, x, 1.27, z)));
  add(r.Rm, B(.95, .02, .66), MAT.shade, .16, 1.72, 0); add(r.Rm, C(.18, .18, .42, 16), MAT.hopper, -.47, 1.06, 0); add(r.Rm, C(.05, .05, .09, 10), MAT.beacon, -.47, 1.35, 0);
  add(r.joint, C(.07, .07, .5, 10), MAT.steel, 0, .62, 0); const bl = add(r.joint, C(.2, .2, .3, 14), MAT.graphite, 0, .62, 0); bl.rotation.z = Math.PI / 2;
  // arm: base on the front module (x .25), yaw, shoulder, elbow, wrist; lengths .85/.85, tool .50
  const base = new THREE.Group(); base.position.set(.25, .86, 0); r.F.add(base); add(base, C(.13, .15, .08, 16), MAT.graphite, 0, .04, 0);
  const yaw = new THREE.Group(); yaw.position.y = .08; base.add(yaw); add(yaw, B(.16, .14, .16), MAT.yellow, 0, .07, 0);
  const sh = new THREE.Group(); sh.position.y = .12; yaw.add(sh); let j = add(sh, C(.07, .07, .2, 12), MAT.graphite); j.rotation.x = Math.PI / 2; add(sh, B(.85, .09, .11), MAT.yellow, .425, 0, 0);
  const el = new THREE.Group(); el.position.x = .85; sh.add(el); j = add(el, C(.06, .06, .18, 12), MAT.graphite); j.rotation.x = Math.PI / 2; add(el, B(.85, .08, .09), MAT.yellow, .425, 0, 0);
  const wr = new THREE.Group(); wr.position.x = .85; el.add(wr); j = add(wr, C(.05, .05, .15, 12), MAT.graphite); j.rotation.x = Math.PI / 2; add(wr, B(.14, .12, .12), MAT.graphite, .07, 0, 0);
  const bit = new THREE.Group(); wr.add(bit); const tg = C(.035, .05, .36, 8); tg.rotateZ(-Math.PI / 2); tg.translate(.32, 0, 0); add(bit, tg, MAT.steel);
  for (let i = 0; i < 4; i++) { const f = add(bit, B(.02, .1, .03), MAT.steel, .2 + i * .07, 0, 0); f.rotation.x = i * .8; }
  // three's rotation.z is the opposite sense to the URDF pitch (+y): negate
  r.pose = q => { yaw.rotation.y = q[0]; sh.rotation.z = -q[1]; el.rotation.z = -q[2]; wr.rotation.z = -q[3]; };
  r.spinTool = dt => { bit.rotation.x += dt * 12; }; r.bit = bit; r.wr = wr;
  r.setTool = d => { bit.position.x = d; };
  return r;
};

/* Pahadi arm IK, same as fs_autonomy/ik.py. target in front-module frame (x fwd, y left, z up). */
FS.armIK = (tx, ty, tz, clear = .08) => {
  const bx = .25, sz = .85, l1 = .85, l2 = .85, tool = .5;
  const yaw = Math.atan2(ty, tx - bx), r = Math.hypot(tx - bx, ty), wz = tz + clear + tool - sz, d = Math.hypot(r, wz);
  if (d > l1 + l2 - 1e-3) return null;
  const q2 = Math.acos(clamp((d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2), -1, 1)), phi = Math.atan2(-wz, r);
  const q1 = phi - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2)); return [yaw, q1, q2, Math.PI / 2 - q1 - q2];
};
FS.ARM_STOW = [0, -1.35, 2.6, .3];
})();
