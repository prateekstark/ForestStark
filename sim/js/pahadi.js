/* C · Pahadi — articulated tracked planter on a hillside. Same terrain model, contour planner,
   turns, tool choice, stability and traction physics as fs_autonomy (pahadi_navigator,
   stability_monitor, traction_monitor) and fs::Terramechanics. */
(() => {
const { clamp, SOILS, soil: S, TH, Path } = FS;
FS.SIMS = FS.SIMS || {};
const HALF = .85, B_TR = .28, L_SEG = .37, GAUGE = .5, WR = .21, MAST = 1.27;

function terrain(p) {
  const t = Math.tan(FS.rad(p.slope));
  const h = (x, y) => y * t + .35 * FS.noise2(x * .35, y * .35) + .9 * FS.noise2(x * .06 + 3, y * .06) - 1.2 * Math.exp(-(((x - 31) / 3) ** 2)) / (1 + Math.exp(-(y + 10) / 4));
  const nrock = (x, y) => FS.noise2(x * .22 + 7, y * .22 - 2);
  const thrHi = .55 - .9 * p.rock * .45, thrMid = .55 - .9 * p.rock;
  const depth = (x, y) => { const n = nrock(x, y); if (n > thrHi) return .03 + .08 * clamp(1 - (n - thrHi) / .3, 0, 1); if (n > thrMid) return .13 + .11 * clamp((thrHi - n) / (thrHi - thrMid), 0, 1); return clamp(.45 + .35 * FS.noise2(x * .12, y * .12 + 4), .3, .9); };
  const normal = (x, y) => { const e = .3, dx = (h(x + e, y) - h(x - e, y)) / (2 * e), dy = (h(x, y + e) - h(x, y - e)) / (2 * e), L = Math.hypot(dx, dy, 1); return [-dx / L, -dy / L, 1 / L]; };
  const r = FS.rng(23), boulders = [];
  while (boulders.length < 45) { const x = -44 + r() * 88, y = -44 + r() * 88, rr = .3 + r() ** 2 * 1.1; if (boulders.every(b => Math.hypot(b.x - x, b.y - y) > b.r + rr + 1.5)) boulders.push({ x, y, r: rr }); }
  return { h, depth, normal, boulders, nrock };
}
function contour(T, h0, x0, x1) {
  let pts = [];
  for (let x = x0; x <= x1 + 1e-9; x += .5) { let lo = -40, hi = 40; if (!(T.h(x, lo) <= h0 && h0 <= T.h(x, hi))) continue; for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (T.h(x, m) < h0) lo = m; else hi = m; } pts.push([x, (lo + hi) / 2]); }
  const smooth = (q, k) => q.map((v, i) => { if (!i || i === q.length - 1) return v; const a = Math.max(0, i - k), b = Math.min(q.length, i + k + 1), s = q.slice(a, b); return [s.reduce((u, w) => u + w[0], 0) / s.length, s.reduce((u, w) => u + w[1], 0) / s.length]; });
  pts = smooth(pts, 2);
  for (let it = 0; it < 4; it++) { pts = pts.map(([x, y]) => { for (const b of T.boulders) { const d = Math.hypot(x - b.x, y - b.y), R = b.r + 1.7; if (d < R) { x = b.x + (x - b.x) / (d || 1e-6) * R; y = b.y + (y - b.y) / (d || 1e-6) * R; } } return [x, y]; }); pts = smooth(pts, 5); }
  return pts;
}
function arc(cx, cy, r, a0, a1) { const n = Math.max(2, Math.ceil(Math.abs(a1 - a0) * r / .2) + 1), o = []; for (let i = 0; i < n; i++) { const a = a0 + (a1 - a0) * i / (n - 1); o.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]); } return o; }
function uTurn(E, hd, S2, R) {   // same as fs_autonomy planners.u_turn
  const c = Math.cos(hd), s = Math.sin(hd), rx = S2[0] - E[0], ry = S2[1] - E[1], ly = -s * rx + c * ry, side = ly >= 0 ? 1 : -1, d = Math.abs(ly);
  const x0 = Math.max(c * rx + s * ry, 0);   // next row starts further on: straight first; short: straight after
  let pts;
  if (d >= 2 * R) pts = arc(0, side * d / 2, d / 2, -side * Math.PI / 2, side * Math.PI / 2);
  else { const xc = Math.sqrt(Math.max(4 * R * R - (d / 2 + R) ** 2, 0)), t1 = Math.atan2(d / 2 + R, xc), ain = Math.atan2(-(d / 2 + R), -xc); let aout = Math.atan2(d / 2 + R, -xc); if (aout < ain) aout += 2 * Math.PI;
    pts = arc(0, -R, R, Math.PI / 2, t1).concat(arc(xc, d / 2, R, ain, aout), arc(0, d + R, R, Math.atan2(-(d / 2 + R), xc), -Math.PI / 2)).map(([x, y]) => [x, side * y]); }
  pts = (x0 > 0 ? [[0, 0]] : []).concat(pts.map(([x, y]) => [x + x0, y]));
  return pts.map(([x, y]) => [E[0] + c * x - s * y, E[1] + s * x + c * y]).concat([S2]);
}

FS.SIMS.C = {
  key: 'C', name: 'Pahadi', deva: 'पहाड़ी', gloss: 'of the hills', mode: 'Tracked · contour rows on a slope',
  desc: 'Two rubber-track modules joined by a steered articulation, with a 4-axis arm. It plants along contour lines and switches to the pick or to seed balls where rock leaves no soil.',
  speeds: [1, 4, 12, 30], defaultSpeed: 12, dt: .05,
  params: [
    { id: 'slope', label: 'Hillside slope', type: 'range', min: 5, max: 32, step: 1, value: 14, unit: '°' },
    { id: 'side', label: 'Arm plants on the', type: 'opts', value: 'downhill', opts: [['downhill', 'Downhill side'], ['uphill', 'Uphill side']], help: 'Reaching downhill moves the centre of gravity toward the tipping edge.' },
    { id: 'soil', label: 'Soil', type: 'opts', value: 'lateritic_gravel', opts: [['lateritic_gravel', 'Laterite'], ['sandy_loam', 'Sandy loam'], ['dry_sand', 'Dry sand']] },
    { id: 'rock', label: 'Thin-soil cover', type: 'range', min: 0, max: .7, step: .05, value: .25, unit: '' },
    { id: 'moist', label: 'Soil moisture', type: 'range', min: .04, max: .35, step: .01, value: .10, unit: 'm³/m³' },
    { id: 'power', label: 'Auger power', type: 'range', min: 1, max: 6, step: .5, value: 2, unit: 'kW' },
    { id: 'payload', label: 'Payload at start', type: 'range', min: 0, max: 100, step: 10, value: 100, unit: '%' },
    { id: 'speed', label: 'Track speed', type: 'range', min: .15, max: .7, step: .05, value: .35, unit: 'm/s' },
    { id: 'rowsp', label: 'Contour spacing', type: 'range', min: 3, max: 8, step: .5, value: 4, unit: 'm' },
    { id: 'halfmoon', label: 'Dig half-moons', type: 'opts', value: true, opts: [[true, 'Yes'], [false, 'No']] },
    { id: 'tilt', label: 'GNSS tilt compensation', type: 'opts', value: true, opts: [[true, 'On'], [false, 'Off']], help: 'Off: the 1.27 m antenna mast leans downhill with the machine and every hole lands short.' },
  ],
  create(p) { return new Pahadi(p); },
  model: `
  <div class="eq"><h5>Track–soil contact (Bekker–Wong)</h5>
    <div class="f"><var>z</var> = (<var>p</var> / (<var>k</var><sub>c</sub>/<var>b</var> + <var>k</var><sub>φ</sub>))<sup>1/<var>n</var></sup>,  <var>R</var><sub>c</sub> = <var>b</var>(<var>k</var><sub>c</sub>/<var>b</var> + <var>k</var><sub>φ</sub>) <var>z</var><sup><var>n</var>+1</sup>/(<var>n</var>+1)</div>
    <div class="f"><var>F</var> = (<var>A c</var> + <var>W</var> tan <var>φ</var>)[1 − <var>K</var>/(<var>i l</var>)(1 − e<sup>−<var>i l</var>/<var>K</var></sup>)]</div>
    <p>Each track is three 0.28 × 0.37 m segments. Slip <var>i</var> solves DP(<var>i</var>) = grade + internal resistance; side slope loads the downhill track. In Gazebo the same law is applied per contact by fs::Terramechanics, with a bristle state so the machine can stand still on the slope.</p></div>
  <div class="eq"><h5>Tip-over: force-angle stability measure</h5>
    <div class="f"><var>β</var><sub>i</sub> = atan2(<var>l</var><sub>i</sub>·<var>ŵ</var>, <var>l</var><sub>i</sub>·<var>û</var>) for each support edge <var>i</var></div>
    <p>Support polygon: convex hull of the 12 contacts. CG from link masses placed by the live arm pose and articulation, minus plugs and water used, with the auger thrust as an upward force at the bit. Navigation halves speed below 12° and holds below 6° (Papadopoulos &amp; Rey, 1996).</p></div>
  <div class="eq"><h5>Contour planning and steering</h5>
    <div class="f"><var>h</var>(<var>x</var>, <var>y</var><sub>c</sub>(<var>x</var>)) = <var>h</var><sub>0</sub> − <var>k</var> Δ<var>h</var>,  Δ<var>h</var> = spacing · tan θ</div>
    <div class="f"><var>γ</var> = −2 atan(0.85 <var>κ</var>),  |<var>γ</var>| ≤ 30° → <var>R</var><sub>min</sub> = 3.2 m</div>
    <p>Rows are iso-elevation lines, pushed clear of boulders and smoothed, joined by U or omega turns no tighter than 3.4 m. The articulation steers; both modules follow one arc.</p></div>
  <div class="eq"><h5>Tool choice and the arm</h5>
    <div class="f">rock depth ≥ 0.25 m: auger · 0.12–0.25: pick (45 MJ/m³ in rock, η 0.5) · &lt; 0.12: seed balls</div>
    <div class="f"><var>q</var><sub>2</sub> = acos((<var>d</var>² − <var>l</var><sub>1</sub>² − <var>l</var><sub>2</sub>²)/(2<var>l</var><sub>1</sub><var>l</var><sub>2</sub>)),  <var>q</var><sub>3</sub> = π/2 − <var>q</var><sub>1</sub> − <var>q</var><sub>2</sub></div>
    <p>Arm 0.85 + 0.85 m, tool 0.50 m, reach 1.25 m beside the track. GNSS antenna on a 1.27 m mast: without tilt compensation it sits 1.27 sin(roll) m downhill of where the planar model thinks.</p></div>`,
};

class Pahadi {
  constructor(p) {
    this.p = p; this.T = terrain(p); this.soil = SOILS[p.soil]; this.r = FS.rng(5); this.t = 0;
    const T = this.T, x0 = -34, x1 = 30, n = 3, dh = p.rowsp * Math.tan(FS.rad(p.slope)), hStart = T.h(-2, -2 + (n - 1) * p.rowsp / 2);
    this.rows = []; for (let k = 0; k < n; k++) { let c = contour(T, hStart - k * dh, x0, x1); if (c.length < 2) continue; if (k % 2) c = c.reverse(); this.rows.push(new Path(c)); }
    const spawn = [-40, -4]; let pts = [spawn];
    const r0 = this.rows[0].at(0); const bez = (P0, h0, P3, h3, k) => Array.from({ length: 40 }, (_, i) => { const t = i / 39, a = [P0[0] + k * Math.cos(h0), P0[1] + k * Math.sin(h0)], b = [P3[0] - k * Math.cos(h3), P3[1] - k * Math.sin(h3)];
      return [0, 1].map(j => (1 - t) ** 3 * P0[j] + 3 * (1 - t) ** 2 * t * a[j] + 3 * (1 - t) * t * t * b[j] + t ** 3 * P3[j]); });
    pts = bez(spawn, Math.PI / 2, [r0[0], r0[1]], r0[2], 5);
    this.rows.forEach((row, i) => { pts = pts.concat(row.p.slice(1)); if (i + 1 < this.rows.length) { const e = row.at(row.length), s2 = this.rows[i + 1].at(0); pts = pts.concat(uTurn([e[0], e[1]], e[2], [s2[0], s2[1]], 3.4).slice(1)); } });
    this.path = new Path(pts);
    this.spots = []; let id = 0; const rg = FS.rng(1);
    this.rows.forEach((row, ri) => { for (let s = 2; s < row.length - 2; s += 2.5) { const [x, y, hd] = row.at(s), nl = [-Math.sin(hd), Math.cos(hd)], nn = T.normal(x, y), dn = Math.hypot(nn[0], nn[1]) || 1, down = [nn[0] / dn, nn[1] / dn];
      const sg = ((nl[0] * down[0] + nl[1] * down[1] > 0) === (p.side === 'downhill')) ? 1 : -1, px = x + sg * 1.25 * nl[0], py = y + sg * 1.25 * nl[1];
      if (T.boulders.some(b => Math.hypot(b.x - px, b.y - py) < b.r + .4)) continue;
      const probe = T.depth(px, py) + FS.gauss(rg) * .03, tool = probe >= .25 ? 'auger' : probe >= .12 ? 'pick' : 'seedball';
      this.spots.push({ id: id++, x: px, y: py, s: this.path.project(x, y).s, side: sg, tool, depth: T.depth(px, py), state: 'todo' }); } });
    this.spots.sort((a, b) => a.s - b.s);
    Object.assign(this, { s: 0, hint: 0, v: 0, k: 0, state: 'drive', arm: FS.ARM_STOW.slice(), armGoal: FS.ARM_STOW.slice(), tool: 0, thrust: 0, cyc: null,
      planted: 0, seeded: 0, skipped: 0, energy: 0, plugs: 200 * p.payload / 100, water: 150 * p.payload / 100, seeds: 3000, errs: [], events: [],
      series: { t: [], margin: [], slip: [], lat: [] }, lastSample: -9, done: false, gamma: 0, slip: 0, lat: 0, margin: 30, roll: 0, pitch: 0, hold: 0 });
    this.place(0); this.stab();
    const tools = {}; this.spots.forEach(s => tools[s.tool] = (tools[s.tool] || 0) + 1);
    const minR = (() => { let m = 1e9; for (let i = 1; i < this.path.p.length - 1; i++) { const [a, b, c] = [this.path.p[i - 1], this.path.p[i], this.path.p[i + 1]], ab = Math.hypot(b[0] - a[0], b[1] - a[1]), bc = Math.hypot(c[0] - b[0], c[1] - b[1]), ca = Math.hypot(a[0] - c[0], a[1] - c[1]), cr = Math.abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])); if (ab > .05 && bc > .05 && cr > 1e-9) m = Math.min(m, ab * bc * ca / (2 * cr)); } return m; })();
    this.ev('ok', `${this.rows.length} contour rows, ${FS.n(this.path.length)} m of path, ${this.spots.length} spots: ${Object.entries(tools).map(([k, v]) => v + ' ' + k).join(', ')}; tightest curve ${minR.toFixed(1)} m`);
  }
  ev(level, text) { this.events.unshift({ t: this.t, level, text }); if (this.events.length > 60) this.events.pop(); }

  /* module poses on the terrain: front centre at s, rear centre 1.7 m back along the path */
  place(s) {
    const T = this.T, [fx, fy] = this.path.at(s), [rx, ry] = this.path.at(Math.max(0, s - 2 * HALF));
    const jx = (fx + rx) / 2, jy = (fy + ry) / 2, yawF = Math.atan2(fy - jy, fx - jx) || this.path.at(s)[2], yawR = Math.atan2(jy - ry, jx - rx) || yawF;
    const mod = (x, y, yaw) => { const c = Math.cos(yaw), sn = Math.sin(yaw), hf = T.h(x + c * .65, y + sn * .65), hb = T.h(x - c * .65, y - sn * .65), hl = T.h(x - sn * .5, y + c * .5), hr = T.h(x + sn * .5, y - c * .5);
      return { x, y, yaw, z: (hf + hb + hl + hr) / 4, pitch: Math.atan2(hf - hb, 1.3), roll: Math.atan2(hr - hl, 1.0) }; };
    this.F = mod(fx, fy, yawF); this.R = mod(rx, ry, yawR); this.gamma = FS.wrap(yawR - yawF);
  }
  /* body frame of a module -> world (x east, y north, z up) */
  toWorld(m, lx, ly, lz) {
    const cr = Math.cos(m.roll), sr = Math.sin(m.roll), cp = Math.cos(m.pitch), sp = Math.sin(m.pitch), cy = Math.cos(m.yaw), sy = Math.sin(m.yaw);
    let x = lx, y = ly, z = lz; [y, z] = [cr * y - sr * z, sr * y + cr * z]; [x, z] = [cp * x - sp * z, sp * x + cp * z]; [x, y] = [cy * x - sy * y, sy * x + cy * y];
    return [m.x + x, m.y + y, m.z + z];
  }
  armPoints(q) {   // arm link CoMs and bit, in the front module frame
    const [yw, q1, q2, q3] = q, bx = .25, sz = .85, c = Math.cos(yw), s = Math.sin(yw), pt = (r, z) => [bx + r * c, r * s, z];
    const e = [.85 * Math.cos(q1), sz - .85 * Math.sin(q1)], w = [e[0] + .85 * Math.cos(q1 + q2), e[1] - .85 * Math.sin(q1 + q2)], tip = [w[0] + (.5 + this.tool) * Math.cos(q1 + q2 + q3), w[1] - (.5 + this.tool) * Math.sin(q1 + q2 + q3)];
    return { up: pt(e[0] / 2, (sz + e[1]) / 2), fore: pt((e[0] + w[0]) / 2, (e[1] + w[1]) / 2), tool: pt((w[0] + tip[0]) / 2, (w[1] + tip[1]) / 2), elbow: pt(...e), wrist: pt(...w), tip: pt(...tip), base: pt(0, sz) };
  }
  stab() {
    const F = this.F, R = this.R, A = this.armPoints(this.arm);
    const items = [[480, this.toWorld(F, 0, 0, .3)], [10, this.toWorld(F, .25, 0, .65)], [15, this.toWorld(F, ...A.base)], [20, this.toWorld(F, ...A.up)], [15, this.toWorld(F, ...A.fore)], [24, this.toWorld(F, ...A.tool)],
      [390, this.toWorld(R, 0, 0, .35)], [this.plugs * .25, this.toWorld(R, .16, 0, 1.1)], [this.water, this.toWorld(R, 0, 0, .45)], [this.seeds * .01, this.toWorld(R, -.47, 0, 1.06)]];
    if (this.thrust > 1) items.push([-this.thrust / FS.G, this.toWorld(F, ...A.tip)]);   // auger reaction lifts the bit side
    const M = items.reduce((a, q) => a + q[0], 0), cg = [0, 1, 2].map(k => items.reduce((a, q) => a + q[0] * q[1][k], 0) / M);
    // ground contacts: both ends of every track segment, at the outer edge of each track
    const contacts = [];
    [F, R].forEach(m => [-.4, 0, .4].forEach(x => [-1, 1].forEach(e => [-1, 1].forEach(side => contacts.push(this.toWorld(m, x + e * L_SEG / 2, side * (GAUGE + B_TR / 2), 0))))));
    const res = FS.forceAngle(contacts, cg, [0, 0, -M * FS.G]);
    const k = res.margins.reduce((bi, m, i, a) => m < a[bi] ? i : bi, 0);
    Object.assign(this, { M, cg, contacts, hull: res.idx, margins: res.margins, margin: FS.deg(res.margins[k] ?? .5), critEdge: k, roll: FS.deg(F.roll), pitch: FS.deg(F.pitch), armPts: A });
    // traction: loads per track side with lateral transfer, then slip for the grade along the heading
    const W = M * FS.G, mid = (F.pitch + R.pitch) / 2, rollAvg = (F.roll + R.roll) / 2, hcg = .55;
    const ycg = ((cg[0] - F.x) * -Math.sin(F.yaw) + (cg[1] - F.y) * Math.cos(F.yaw));
    const Nn = W * Math.cos(rollAvg) * Math.cos(mid), dN = (W * Math.sin(rollAvg) * hcg + Nn * ycg) / (2 * GAUGE);
    const Nl = clamp(Nn / 2 + dN, 0, Nn), Nr = Nn - Nl; this.Nl = Nl; this.Nr = Nr;
    const loads = [...Array(6).fill(Nl / 6), ...Array(6).fill(Nr / 6)], need = W * Math.sin(mid) + .03 * W + (Math.abs(this.gamma) > .05 ? .04 * W : 0);
    this.loads = loads; this.need = need; this.slip = this.v > 0 ? S.slipFor(this.soil, loads, B_TR, L_SEG, Math.max(need, 0)) : 0;
    const latCap = loads.reduce((a, w) => a + .9 * S.shearCap(this.soil, w, B_TR, L_SEG), 0); this.lat = Math.abs(W * Math.sin(rollAvg)) / latCap;
    this.sink = S.sinkage(this.soil, Math.max(Nl, Nr) / 6, B_TR, L_SEG);
  }

  step(dt) {
    if (this.done) return; this.t += dt; const p = this.p;
    this.stab();
    if (this.state === 'plant') this.runCycle(dt);
    else {
      let vcmd = p.speed; if (this.margin < 6) { vcmd = 0; if (this.state !== 'hold') this.ev('bad', `stability CRITICAL: ${this.margin.toFixed(1)}° — holding`); this.state = 'hold'; }
      else { if (this.state === 'hold') this.state = 'drive'; if (this.margin < 12) vcmd *= .5; }
      const sp = this.spots[this.k];
      if (sp) { const gap = sp.s - (this.s + .25); if (gap < -1.5) { sp.state = 'skip'; this.skipped++; this.k++; this.ev('warn', `passed spot ${sp.id}`); } else if (gap <= .05) { this.startCycle(sp); vcmd = 0; } else vcmd = Math.min(vcmd, Math.max(.08, .5 * gap)); }
      else if (this.s >= this.path.length - .3) { vcmd = 0; if (!this.done) { this.done = true; this.ev('ok', `contours complete: ${this.planted} planted, ${this.seeded} seeded, ${this.skipped} skipped`); } }
      this.v += clamp(vcmd - this.v, -.8 * dt, .5 * dt);
      const vg = this.v * (1 - (this.slip >= 1 ? 1 : this.slip)); if (this.slip >= 1 && this.v > .05 && !this.stuck) { this.stuck = true; this.ev('bad', 'immobilised: tracks cannot develop the pull this grade needs'); } if (this.slip < 1) this.stuck = false;
      this.s = Math.min(this.path.length, this.s + vg * dt); this.place(this.s);
      this.arm = this.arm.map((q, i) => q + clamp(FS.ARM_STOW[i] - q, -.9 * dt, .9 * dt));
    }
    if (this.t - this.lastSample > 1) { this.lastSample = this.t; const s = this.series; s.t.push(this.t); s.margin.push(this.margin); s.slip.push(this.slip * 100); s.lat.push(this.lat * 100); if (s.t.length > 600) for (const k in s) s[k].shift(); }
  }
  startCycle(sp) {
    const F = this.F, gz = this.T.h(sp.x, sp.y);
    // target in the front module frame (inverse of toWorld)
    let dx = sp.x - F.x, dy = sp.y - F.y, dz = gz - F.z; const cy = Math.cos(-F.yaw), sy = Math.sin(-F.yaw); [dx, dy] = [cy * dx - sy * dy, sy * dx + cy * dy];
    const cp = Math.cos(-F.pitch), spp = Math.sin(-F.pitch); [dx, dz] = [cp * dx - spp * dz, spp * dx + cp * dz]; const cr = Math.cos(-F.roll), sr = Math.sin(-F.roll); [dy, dz] = [cr * dy - sr * dz, sr * dy + cr * dz];
    const q = FS.armIK(dx, dy, dz, .06);
    if (!q) { sp.state = 'skip'; this.skipped++; this.k++; this.ev('warn', `spot ${sp.id} out of arm reach`); return; }
    const head = { P: this.p.power * 1000 }, depth = sp.depth, soilE = this.soil.eb * 1e6 * S.moisture(this.p.moist);
    let tool = sp.tool, phases = [['reach', 4.5]], energy = 0;
    const h = { ...FS.HEADS.pahadi, ...head }, A = Math.PI * h.D * h.D / 4;
    const boreT = (e, d, eta) => { const r = S.bore({ ...h, eta }, e); return { t: d / Math.max(r.feed, 1e-4), E: e * A * d / eta / 1000, thrust: r.thrust }; };
    if (tool === 'auger' && depth < .25) { const a = boreT(soilE, depth, h.eta); phases.push(['bore', a.t], ['refusal', 1.5]); energy += a.E; tool = 'pick'; this.ev('warn', `spot ${sp.id}: auger refused at ${(depth * 100).toFixed(0)} cm, switching to the pick`); }
    if (tool === 'auger') { const a = boreT(soilE, .25, h.eta); phases.push(['bore', a.t]); energy += a.E; this.cycThrust = a.thrust; }
    else if (tool === 'pick') { const a = boreT(soilE, Math.min(depth, .25), h.eta), b = boreT(45e6, Math.max(0, .25 - depth), .5); phases.push(['pick', a.t + b.t]); energy += a.E + b.E; this.cycThrust = Math.max(a.thrust, b.thrust); }
    else phases.push(['seed', 4]);
    if (tool !== 'seedball') phases.push(['retract', .4 / .3], ['place', 2], ['close', 1.5], ['water', 1 / .3]);
    if (this.p.halfmoon && tool !== 'seedball') phases.push(['half-moon', 5.5]);
    phases.push(['stow', 4]);
    // GNSS: estimate error (RTK) plus the mast lever arm if not tilt-compensated
    const lean = this.p.tilt ? 0 : MAST * Math.sin(F.roll), nl = [-Math.sin(F.yaw), Math.cos(F.yaw)];
    const err = [FS.gauss(this.r) * .02 + lean * nl[0] * -1, FS.gauss(this.r) * .02 + lean * nl[1] * -1];
    this.cyc = { sp, q, tool, phases, t: 0, total: phases.reduce((a, x) => a + x[1], 0), energy, err, cur: 'reach' };
    this.state = 'plant'; this.v = 0;
  }
  runCycle(dt) {
    const c = this.cyc; c.t += dt; let acc = 0, cur = c.phases.length - 1; for (let i = 0; i < c.phases.length; i++) { if (c.t < acc + c.phases[i][1]) { cur = i; break; } acc += c.phases[i][1]; }
    const [name, dur] = c.phases[cur], u = (c.t - acc) / dur; c.cur = name;
    const goal = name === 'stow' ? FS.ARM_STOW : c.q; this.arm = this.arm.map((q, i) => q + clamp(goal[i] - q, -1.0 * dt, 1.0 * dt));
    this.tool = name === 'bore' || name === 'pick' ? .06 + .25 * u : name === 'retract' ? .31 * (1 - u) : 0;
    this.thrust = name === 'bore' || name === 'pick' ? this.cycThrust : 0;
    if (c.t >= c.total) {
      const sp = c.sp, ax = sp.x + c.err[0], ay = sp.y + c.err[1]; this.errs.push(Math.hypot(...c.err));
      if (c.tool === 'seedball') { this.seeded++; this.seeds -= 3; sp.state = 'seed'; } else { this.planted++; this.plugs = Math.max(0, this.plugs - 1); this.water = Math.max(0, this.water - 1); sp.state = 'done'; }
      this.energy += c.energy; sp.ax = ax; sp.ay = ay; this.newPlant = { sp, kind: c.tool === 'seedball' ? 'seed' : 'sapling', half: this.p.halfmoon && c.tool !== 'seedball' };
      this.k++; this.state = 'drive'; this.tool = 0; this.thrust = 0; this.lastCycle = c;
    }
  }

  /* ---------- 3D ---------- */
  build3D() {
    const W = FS.T3.world, T = this.T, M = FS.MAT, C = FS.C;
    FS.groundMesh(96, 96, 160, 160, (x, z) => T.h(x, -z), (x, z) => { const d = T.depth(x, -z); return d < .12 ? FS.mixHex(0x9A9184, 0xA89B85, .5 + .5 * FS.noise2(x, z)) : d < .25 ? FS.mixHex(0xA8916D, 0x9C8F78, .5) : FS.mixHex(0xB89C70, 0x8E8A55, .35 + .6 * FS.noise2(x * 1.3 + 5, z * 1.3)); });
    T.boulders.forEach((b, i) => { const m = FS.add(W, new THREE.IcosahedronGeometry(1, 0), M.rock, b.x, T.h(b.x, b.y) + b.r * .25, -b.y); m.scale.set(b.r * (1 + (i * 7 % 5) / 8), b.r * .7, b.r * (1 + (i * 3 % 4) / 7)); m.rotation.y = i * 1.3; });
    this.rings = new FS.RingPool(this.spots, (x, y) => T.h(x, y)); this.spots.forEach((s, i) => this.rings.color(i, s.tool === 'seedball' ? 'seed' : s.tool === 'pick' ? 'warn' : 'todo'));
    this.pool = new FS.PlantPool(this.spots.length * 2 + 4);
    const pl = new THREE.BufferGeometry().setFromPoints(this.path.p.map(([x, y]) => new THREE.Vector3(x, T.h(x, y) + .05, -y)));
    const line = new THREE.Line(pl, new THREE.LineDashedMaterial({ color: 0xE2A514, dashSize: .4, gapSize: .3 })); line.computeLineDistances(); W.add(line);
    this.m = FS.buildPahadi(); W.add(this.m.F, this.m.Rm, this.m.joint);
    [this.m.F, this.m.Rm].forEach(g => g.rotation.order = 'YZX');
  }
  sync3D(dt) {
    const m = this.m; if (!m) return; const set = (g, mod) => { g.position.set(mod.x, mod.z, -mod.y); g.rotation.set(-mod.roll, mod.yaw, mod.pitch); };
    set(m.F, this.F); set(m.Rm, this.R); m.joint.position.set((this.F.x + this.R.x) / 2, (this.F.z + this.R.z) / 2, -(this.F.y + this.R.y) / 2); m.joint.rotation.y = (this.F.yaw + this.R.yaw) / 2;
    m.spins.forEach(w => w.spin.rotation.z -= this.v * dt / w.r);
    m.pose(this.arm); m.setTool(this.tool); if (this.thrust > 0) m.spinTool(dt);
    m.trays.forEach((t, i) => t.visible = this.plugs > i * 50);
    if (this.newPlant) { const { sp, kind, half } = this.newPlant, h = this.T.h(sp.ax, sp.ay); this.pool.add(sp.ax, sp.ay, h, kind);
      if (half) { const n = this.T.normal(sp.ax, sp.ay); this.pool.add(sp.ax, sp.ay, h, 'halfmoon', Math.atan2(n[1], n[0])); }
      this.rings.color(this.spots.indexOf(sp), 'done'); this.newPlant = null; }
    this.pool.update(dt);
  }
  camTarget() { return [this.F.x, this.F.y, this.F.z + .8]; }
  camOffset() { return [-5.5, 4.6 + this.F.z * 0, -7.5]; }
  progress() { return this.k / Math.max(1, this.spots.length); }
  hud() { return { title: `C · Pahadi — ${this.p.slope}° slope, contour rows`, rows: [['Planted · seeded', `${this.planted} · ${this.seeded}`], ['Tip-over margin', `${this.margin.toFixed(1)}°`],
    ['Roll · pitch', `${this.roll.toFixed(1)}° · ${this.pitch.toFixed(1)}°`], ['Track slip', `${(this.slip * 100).toFixed(1)} %`], ['Head', this.state === 'plant' ? this.cyc.cur : this.state]] }; }
  tele() {
    const mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN, lv = this.margin < 6 ? 'bad' : this.margin < 12 ? 'warn' : 'ok';
    return [['Stability'], ['Tip-over margin', `${this.margin.toFixed(1)}°`, lv], ['Critical edge', this.edgeName()], ['Roll · pitch', `${this.roll.toFixed(1)}° · ${this.pitch.toFixed(1)}°`],
      ['Mass · CG height', `${FS.n(this.M)} kg · ${FS.n(this.cg[2] - this.F.z, 2)} m`], ['Articulation', `${FS.deg(this.gamma).toFixed(1)}°`],
      ['Traction'], ['Slip', `${(this.slip * 100).toFixed(1)} %`, this.slip > .5 ? 'bad' : this.slip > .2 ? 'warn' : 'ok'], ['Lateral shear used', `${(this.lat * 100).toFixed(0)} %`, this.lat > .85 ? 'bad' : this.lat > .6 ? 'warn' : 'ok'],
      ['Track loads L · R', `${FS.n(this.Nl / 1000, 1)} · ${FS.n(this.Nr / 1000, 1)} kN`], ['Sinkage', `${FS.n(this.sink * 1000, 1)} mm`],
      ['Work'], ['Planted · seeded · skipped', `${this.planted} · ${this.seeded} · ${this.skipped}`], ['Rate', this.t > 120 ? `${FS.n((this.planted + this.seeded) / this.t * 3600)} /h` : '—'],
      ['Placement error', this.errs.length ? `${FS.n(mean(this.errs) * 100, 1)} cm` : '—', this.errs.length && mean(this.errs) > .15 ? 'warn' : null],
      ['Energy per hole', this.planted ? `${FS.n(this.energy / (this.planted + this.seeded), 0)} kJ` : '—'], ['Plugs · water', `${FS.n(this.plugs)} · ${FS.n(this.water)} L`]];
  }
  edgeName() { if (!this.hull || !this.hull.length) return '—'; const i = this.hull[this.critEdge], j = this.hull[(this.critEdge + 1) % this.hull.length], a = this.contacts[i], b = this.contacts[j];
    const mx = (a[0] + b[0]) / 2 - this.F.x, my = (a[1] + b[1]) / 2 - this.F.y, c = Math.cos(-this.F.yaw), s = Math.sin(-this.F.yaw), bx = c * mx - s * my, by = s * mx + c * my;
    const ang = FS.deg(Math.atan2(by, bx)); return Math.abs(ang) < 30 ? 'front' : Math.abs(ang) > 150 ? 'rear' : ang > 0 ? 'left track' : 'right track'; }

  figs() {
    return [
      { id: 'xsec', title: 'Tip-over: cross-section', tag: 'looking along the track, live', draw: (c, w, h) => this.drawSection(c, w, h),
        caption: 'Slope, tracks, arm and centre of gravity, looking forward. The plumb line from the CG must stay inside the downhill track; the angle to the edge is the margin. Reach downhill, steepen the slope or start with full trays to watch it shrink.' },
      { id: 'poly', title: 'Support polygon', tag: 'top view, gravity-aligned', draw: (c, w, h) => this.drawPolygon(c, w, h),
        caption: 'Convex hull of the 12 track contacts with the CG projected straight down. Edge labels are the force-angle margins in degrees.' },
      { id: 'trac', title: 'Drawbar pull vs slip', tag: 'Janosi–Hanamoto on this soil', draw: (c, w, h) => this.drawTraction(c, w, h),
        caption: 'Net pull the tracks develop at each slip, against what this heading needs (grade plus internal resistance). Where they cross is the operating slip; if the curve never reaches the line the machine is immobilised.' },
      { id: 'map', title: 'Terrain and plan', tag: 'contours, rock, tools', wide: true, draw: (c, w, h) => this.drawMap(c, w, h),
        caption: '0.5 m contour lines over depth to rock (darker: thinner soil). The dashed path is the drive line with its omega turns; spots are coloured by tool: auger, pick, seed balls.' },
      { id: 'ts', title: 'Margin and slip over time', tag: 'last 10 minutes', draw: (c, w, h) => this.drawSeries(c, w, h),
        caption: 'Margin dips each time the arm reaches out and while the auger pushes; slip rises in the turns, where the machine climbs or descends.' },
    ];
  }
  drawSection(c, w, h) {
    // Gravity-aligned plane across the heading: u = left of travel, z = up. Uphill drawn on the left.
    const F = this.F, nl = [-Math.sin(F.yaw), Math.cos(F.yaw)], sc = Math.min(w / 4.4, h / 2.6), cx = w / 2, gy = h * .66;
    const UZ = q => [(q[0] - F.x) * nl[0] + (q[1] - F.y) * nl[1], q[2] - F.z], P = ([u, z]) => [cx - u * sc, gy - z * sc], W = (...l) => P(UZ(this.toWorld(F, ...l)));
    const up = this.T.h(F.x + nl[0], F.y + nl[1]) > this.T.h(F.x - nl[0], F.y - nl[1]) ? 1 : -1;
    const ground = []; for (let u = -2.6; u <= 2.6; u += .1) ground.push(P([u, this.T.h(F.x + nl[0] * u, F.y + nl[1] * u) - F.z]));
    FS.poly(c, ground.concat([[ground[ground.length - 1][0], h], [ground[0][0], h]]), FS.alpha(TH.accent, .14)); FS.line(c, ground, TH.muted, 1.2);
    [-1, 1].forEach(sd => FS.poly(c, [W(0, sd * (GAUGE - B_TR / 2), 0), W(0, sd * (GAUGE + B_TR / 2), 0), W(0, sd * (GAUGE + B_TR / 2), .42), W(0, sd * (GAUGE - B_TR / 2), .42)], TH.ink));
    FS.poly(c, [W(0, -.36, .22), W(0, .36, .22), W(0, .36, .62), W(0, -.36, .62)], TH.mark, TH.ink);
    const A = this.armPts, chain = [A.base, A.elbow, A.wrist].map(q => W(...q)), tip = W(...A.tip);
    FS.line(c, chain, TH.accent, 5); FS.line(c, [chain[2], tip], TH.steel || TH.muted, 3); FS.circle(c, tip[0], tip[1], 3, TH.accent);
    const g = P(UZ(this.cg)), downSide = -up, edge = W(0, downSide * (GAUGE + B_TR / 2), 0);
    FS.line(c, [g, [g[0], gy + 8]], TH.bad, 1.2, [3, 3]); FS.arrow(c, g[0], g[1], g[0], g[1] + 48, TH.bad, 2);
    FS.line(c, [g, edge], TH.bad, 1, [5, 4]); FS.circle(c, edge[0], edge[1], 4.5, TH.bad, TH.plot, 1.5);
    FS.circle(c, g[0], g[1], 6, TH.bad, TH.plot, 2); FS.text(c, 'CG', g[0] + 9, g[1] - 6, { color: TH.bad, size: 11, weight: 500 });
    FS.text(c, 'tipping edge', edge[0] + (downSide > 0 ? -8 : 8), edge[1] + 16, { align: downSide > 0 ? 'right' : 'left', size: 10, color: TH.bad });
    const lv = this.margin < 6 ? TH.bad : this.margin < 12 ? TH.warn : TH.ok;
    FS.text(c, `${this.margin.toFixed(1)}° to tip over`, 12, 24, { size: 17, font: 'display', weight: 600, color: lv });
    FS.text(c, `roll ${this.roll.toFixed(1)}° · arm ${this.p.side} · thrust ${FS.n(this.thrust)} N`, 12, 41, { size: 10.5, color: TH.muted });
    FS.text(c, up > 0 ? 'uphill' : 'downhill', 12, h - 10, { size: 10.5, color: TH.muted }); FS.text(c, up > 0 ? 'downhill' : 'uphill', w - 12, h - 10, { size: 10.5, color: TH.muted, align: 'right' });
  }
  drawPolygon(c, w, h) {
    if (!this.hull || this.hull.length < 3) return; const F = this.F, cs = Math.cos(-F.yaw), sn = Math.sin(-F.yaw);
    const L = q => { const dx = q[0] - F.x, dy = q[1] - F.y; return [cs * dx - sn * dy, sn * dx + cs * dy]; };
    const pts = this.contacts.map(L), sc = Math.min(w / 3.6, h / 2.2), cx = w / 2 + .85 * sc / 2, cy = h / 2, P = q => [cx + q[0] * sc, cy - q[1] * sc];
    const hull = this.hull.map(i => P(pts[i])); FS.poly(c, hull, FS.alpha(TH.leaf, .12), TH.leaf, 1.5);
    pts.forEach(q => { const s = P(q); FS.circle(c, s[0], s[1], 2.2, TH.muted); });
    this.hull.forEach((i, k) => { const a = P(pts[i]), b = P(pts[this.hull[(k + 1) % this.hull.length]]), m = FS.deg(this.margins[k]);
      if (Math.hypot(b[0] - a[0], b[1] - a[1]) > 25) FS.text(c, `${m.toFixed(0)}°`, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + (k === this.critEdge ? -6 : 12), { align: 'center', size: 10.5, color: k === this.critEdge ? (m < 12 ? TH.warn : TH.ink) : TH.muted, halo: true }); });
    const g = P(L(this.cg)); FS.circle(c, g[0], g[1], 5, TH.bad, TH.plot, 1.5); FS.text(c, 'CG', g[0] + 8, g[1] + 4, { size: 10.5, color: TH.bad });
    const tip = P(L(this.toWorld(F, ...this.armPts.tip))); FS.circle(c, tip[0], tip[1], 3.5, TH.accent); FS.text(c, 'bit', tip[0] + 6, tip[1] + 4, { size: 10, color: TH.accent });
    FS.arrow(c, cx + 1.2 * sc, cy, cx + 1.6 * sc, cy, TH.muted, 1.2); FS.text(c, 'travel', cx + 1.2 * sc, cy - 6, { size: 10, color: TH.muted });
  }
  drawTraction(c, w, h) {
    const box = { x: 50, y: 22, w: w - 66, h: h - 56 }, loads = this.loads || [1000], s = this.soil;
    const curve = Array.from({ length: 51 }, (_, i) => [i / 50 * 100, S.drawbar(s, loads, B_TR, L_SEG, i / 50) / 1000]);
    const ymax = Math.max(...curve.map(q => q[1]), this.need / 1000) * 1.15, ymin = Math.min(0, ...curve.map(q => q[1]));
    const { X, Y } = FS.frame(c, box, [0, 100], [ymin, ymax], { xlabel: 'slip %', ylabel: 'kN' });
    FS.series(c, X, Y, curve, TH.leaf, { dot: false, w: 2 });
    FS.line(c, [[X(0), Y(this.need / 1000)], [X(100), Y(this.need / 1000)]], TH.warn, 1.5, [5, 4]);
    FS.text(c, `needed ${FS.n(this.need / 1000, 2)} kN`, X(100) - 4, Y(this.need / 1000) - 6, { align: 'right', size: 10.5, color: TH.warn });
    if (this.slip < 1) FS.circle(c, X(this.slip * 100), Y(this.need / 1000), 5, TH.accent, TH.plot, 1.5);
    else FS.text(c, 'immobilised', X(50), Y(this.need / 1000) + 18, { align: 'center', color: TH.bad, size: 12, weight: 500 });
    FS.text(c, `${s.label}: c ${s.c} kPa, φ ${s.phi}°, K ${s.K * 100} cm`, box.x + 6, box.y + 12, { size: 10.5, color: TH.muted });
  }
  drawMap(c, w, h) {
    const T = this.T, x0 = -44, x1 = 40, y0 = -18, y1 = 14, sc = Math.min((w - 20) / (x1 - x0), (h - 20) / (y1 - y0)), P = (x, y) => [10 + (x - x0) * sc, h - 10 - (y - y0) * sc];
    const step = 1.2; for (let x = x0; x < x1; x += step) for (let y = y0; y < y1; y += step) { const d = T.depth(x, y); if (d < .25) { const q = P(x, y + step); c.fillStyle = FS.alpha(d < .12 ? TH.muted : TH.accent, d < .12 ? .45 : .22); c.fillRect(q[0], q[1], step * sc + .5, step * sc + .5); } }
    const hmin = T.h(0, y0) - 2, hmax = T.h(0, y1) + 2; for (let lev = Math.ceil(hmin * 2) / 2; lev < hmax; lev += .5) { const pts = []; for (let x = x0; x <= x1; x += 1) { let lo = y0 - 5, hi = y1 + 5; if (!(T.h(x, lo) <= lev && lev <= T.h(x, hi))) { if (pts.length) break; continue; } for (let k = 0; k < 22; k++) { const m = (lo + hi) / 2; if (T.h(x, m) < lev) lo = m; else hi = m; } pts.push(P(x, (lo + hi) / 2)); }
      FS.line(c, pts, FS.alpha(TH.muted, Math.abs(lev % 2.5) < .01 ? .7 : .25), Math.abs(lev % 2.5) < .01 ? 1.2 : .8); }
    T.boulders.forEach(b => { const q = P(b.x, b.y); FS.circle(c, q[0], q[1], b.r * sc, FS.alpha(TH.ink, .35)); });
    FS.line(c, this.path.p.map(q => P(q[0], q[1])), TH.accent, 1.2, [5, 4]);
    const col = { auger: TH.leaf, pick: TH.warn, seedball: TH.c4 };
    this.spots.forEach(s => { const q = P(s.x, s.y); if (s.state === 'done' || s.state === 'seed') FS.circle(c, q[0], q[1], 3.2, col[s.tool]); else FS.circle(c, q[0], q[1], 3, null, s.state === 'skip' ? TH.bad : col[s.tool], 1.4); });
    const F = this.F, R = this.R; FS.line(c, [P(R.x, R.y), P(F.x, F.y)], TH.ink, 5); FS.circle(c, ...P(F.x, F.y), 4, TH.mark, TH.ink);
    FS.poly(c, [[6, 4], [300, 4], [300, 24], [6, 24]], FS.alpha(TH.plot, .9));
    FS.legend(c, [['auger', TH.leaf], ['pick', TH.warn], ['seed balls', TH.c4], ['path', TH.accent, [5, 4]]], 14, 18);
    FS.text(c, 'uphill ↑', w - 10, 16, { align: 'right', size: 10.5, color: TH.muted });
  }
  drawSeries(c, w, h) {
    const s = this.series; if (s.t.length < 2) { FS.text(c, 'collecting…', 50, 40, { color: TH.muted }); return; }
    const box = { x: 40, y: 24, w: w - 84, h: h - 56 }, t0 = s.t[0] / 60, t1 = Math.max(s.t[s.t.length - 1] / 60, t0 + 1);
    const { X, Y } = FS.frame(c, box, [t0, t1], [0, Math.max(40, ...s.margin) * 1.05], { xlabel: 'min', ylabel: 'margin °' });
    FS.poly(c, [[box.x, Y(6)], [box.x + box.w, Y(6)], [box.x + box.w, Y(0)], [box.x, Y(0)]], FS.alpha(TH.bad, .08));
    FS.series(c, X, Y, s.t.map((t, i) => [t / 60, s.margin[i]]), TH.leaf, { area: true });
    const Y2 = v => box.y + box.h - v / 100 * box.h; FS.series(c, X, Y2, s.t.map((t, i) => [t / 60, s.slip[i]]), TH.warn, { w: 1.4 });
    [0, 50, 100].forEach(v => FS.text(c, `${v}%`, box.x + box.w + 6, Y2(v) + 3.5, { size: 10, color: TH.warn }));
    FS.legend(c, [['tip-over margin', TH.leaf], ['track slip', TH.warn]], box.x + 6, box.y + 12);
  }
}
})();
