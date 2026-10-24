/* A · Katar — roadside planting trailer. Same site, logic and constants as the ROS stack
   (fs_gazebo roadside site, katar_driver, katar_planter, planting_head). */
(() => {
const { clamp, SOILS, soil: S, TH, Path } = FS;
FS.SIMS = FS.SIMS || {};

const shoulderY = x => 6 * Math.sin(x / 45) + 2.5 * Math.sin(x / 19 + 1);
const CL = new Path(Array.from({ length: 207 }, (_, i) => { const x = -30 + i * 2; return [x, shoulderY(x)]; }));
const LINE = CL.offset(2.5);
const ZONES = [
  { name: 'culvert', s0: 150, s1: 158, lat0: -1.5, lat1: 6, visible: true },
  { name: 'telecom cable', s0: 232, s1: 290, lat0: 1.9, lat1: 3.2, visible: false },
  { name: 'field access', s0: 318, s1: 324, lat0: 1.3, lat1: 12, visible: true },
];
const POSTS = [232, 251, 270, 289];
const L1 = 2.05, HITCH = 1.15, L2 = 3.525;   // wheelbase, hitch behind rear axle, hitch to tandem centre
const M_TRACTOR = 2200;

const verge = (sc, soilName) => {
  if (sc > 150 && sc < 225) return 'clayey_soil';
  if (sc > 280 && sc < 335 && FS.noise2(sc * .05, 1) > 0) return 'lateritic_gravel';
  return soilName;
};

FS.SIMS.A = {
  key: 'A', name: 'Katar', deva: 'कतार', gloss: 'row', mode: 'Tractor trailer · roadside row',
  desc: 'A planting trailer behind a 40 hp tractor. A person drives; spacing, boring, placing and watering are automatic. A boom sled corrects where the driver stops.',
  speeds: [1, 4, 12, 30], defaultSpeed: 12, dt: 0.05,
  params: [
    { id: 'mode', label: 'Planting mode', type: 'opts', value: 'stop', opts: [['stop', 'Stop and go'], ['move', 'On the move']],
      help: 'On the move: the sled runs back at ground speed while the bit is in the soil.' },
    { id: 'spacing', label: 'Spacing', type: 'range', min: 2, max: 5, step: .5, value: 3, unit: 'm' },
    { id: 'speed', label: 'Driver set speed', type: 'range', min: 1, max: 4, step: .5, value: 2.5, unit: 'km/h' },
    { id: 'rail', label: 'Sled travel', type: 'range', min: .6, max: 2.4, step: .2, value: 2.4, unit: 'm' },
    { id: 'react', label: 'Driver reaction to STOP', type: 'range', min: .3, max: 2, step: .1, value: .8, unit: 's' },
    { id: 'weave', label: 'Driver weave (1σ)', type: 'range', min: 0, max: .6, step: .05, value: .25, unit: 'm' },
    { id: 'gnss', label: 'Positioning', type: 'opts', value: 'rtk', opts: [['rtk', 'RTK ±2 cm'], ['std', 'GNSS ±1.5 m'], ['odo', 'Odometry only']] },
    { id: 'survey', label: 'Utility survey done', type: 'opts', value: 'yes', opts: [['yes', 'Yes'], ['no', 'No']],
      help: 'Without it only visible hazards are avoided; the buried cable is not.' },
    { id: 'soil', label: 'Verge soil', type: 'opts', value: 'sandy_loam', opts: [['sandy_loam', 'Sandy loam'], ['dry_sand', 'Dry sand'], ['heavy_clay', 'Heavy clay']] },
    { id: 'moist', label: 'Soil moisture', type: 'range', min: .05, max: .35, step: .01, value: .14, unit: 'm³/m³' },
  ],
  create(p) { return new Katar(p); },
  model: `
  <div class="eq"><h5>Tractor–trailer kinematics</h5>
    <div class="f"><var>θ̇</var><sub>1</sub> = <var>v</var> tan <var>δ</var> / <var>L</var><sub>1</sub></div>
    <div class="f"><var>θ̇</var><sub>2</sub> = (<var>v</var>/<var>L</var><sub>2</sub>) sin(<var>θ</var><sub>1</sub>−<var>θ</var><sub>2</sub>) − (<var>h</var>/<var>L</var><sub>2</sub>) <var>θ̇</var><sub>1</sub> cos(<var>θ</var><sub>1</sub>−<var>θ</var><sub>2</sub>)</div>
    <p>Off-axle hitch: <var>L</var><sub>1</sub> 2.05 m, hitch <var>h</var> 1.15 m behind the rear axle, <var>L</var><sub>2</sub> 3.53 m to the tandem centre. The driver steers by pure pursuit on the shoulder with an Ornstein–Uhlenbeck weave and reacts to the cab STOP cue after a delay. Gazebo: AckermannSteering + a revolute hitch.</p></div>
  <div class="eq"><h5>Spacing trigger and boom correction</h5>
    <div class="f"><var>d</var><sub>stop</sub> = <var>v t</var><sub>react</sub> + <var>v</var>²/(2<var>a</var>) + 0.15</div>
    <div class="f"><var>q</var><sub>sled</sub> = <var>s</var><sub>spot</sub> − <var>s</var><sub>bit</sub> ∈ [−1.8, 0.6] m,  <var>q</var><sub>carriage</sub> = lateral − 1.0 ∈ [0.5, 2.5] m</div>
    <div class="f">on the move: <var>v</var> ≤ 0.85 · rail / <var>t</var><sub>ground</sub>,  <var>v</var> ≤ spacing / (<var>t</var><sub>cycle</sub> + 3)</div>
    <p>Stations come from the estimate (RTK, plain GNSS wander σ 1.5 m τ 30 s, or odometry with 2 % scale error), never the truth. The sled and carriage absorb what the driver cannot.</p></div>
  <div class="eq"><h5>Auger in soil (fs::AugerSoil)</h5>
    <div class="f"><var>e</var> = <var>e</var><sub>b</sub>·(1.6 − 2.4<var>w</var>),  feed = min(<var>f</var><sub>max</sub>, η<var>P</var>/(<var>e A</var>)),  <var>F</var><sub>t</sub> = 0.012 <var>e A</var></div>
    <div class="f"><var>E</var><sub>hole</sub> = <var>e A d</var> / η</div>
    <p>Ø 0.15 m, 8 kW at the bit, η 0.75, thrust ≤ 8 kN, hole 0.30 m. <var>e</var><sub>b</sub>: dry sand 2.5, sandy loam 4, clay 9, heavy clay 14, laterite 22 MJ/m³ (at 15 % moisture).</p></div>
  <div class="eq"><h5>Trailer load and tractor power</h5>
    <div class="f"><var>T</var><sub>tongue</sub> = Σ <var>m<sub>i</sub> g</var> (<var>x</var><sub>axle</sub> − <var>x<sub>i</sub></var>) / <var>x</var><sub>axle</sub></div>
    <div class="f"><var>P</var> = (Σ<var>R</var><sub>c</sub> + <var>m a</var>) <var>v</var>/0.85 + <var>P</var><sub>hyd</sub>/0.8 + 3 kW,  fuel = 250 g/kWh</div>
    <p>Compaction resistance <var>R</var><sub>c</sub> from Bekker sinkage of every tyre on the compacted shoulder. 1,000 L of water at 1.6 m behind the hitch makes the trailer nose-heavy at the start; as it empties the tongue load drops.</p></div>`,
};

class Katar {
  constructor(p) {
    this.p = p; this.r = FS.rng(7); this.t = 0;
    this.soil = S.atMoisture(SOILS[p.soil], p.moist);
    const sStart = LINE.project(...CL.at(40).slice(0, 2)).s, sEnd = LINE.project(...CL.at(380).slice(0, 2)).s;
    this.spots = [];
    const trees = []; const tr = FS.rng(7);
    for (let s = 55; s < 370; s += 13) { const lat = tr() > .18 ? 4 + tr() * 6 : 2.2 + tr() * .7; if (ZONES.some(z => s > z.s0 - 4 && s < z.s1 + 4)) continue;
      const [x, y, h] = CL.at(s + (tr() - .5) * 6); trees.push({ x: x - Math.sin(h) * lat, y: y + Math.cos(h) * lat }); }
    this.trees = trees;
    for (let s = sStart, k = 0; s <= sEnd; s += p.spacing, k++) {
      const [x, y] = LINE.at(s), pc = CL.project(x, y), sp = { id: k, s, x, y, sc: pc.s, lat: pc.lat, skip: false, note: '', hidden: '' };
      for (const z of ZONES) if (pc.s >= z.s0 && pc.s <= z.s1 && pc.lat >= z.lat0 && pc.lat <= z.lat1) {
        if (p.survey === 'yes' || z.visible) { sp.skip = true; sp.note = 'no-dig: ' + z.name; }
        else if (POSTS.some(m => Math.abs(pc.s - m) < 1.5)) { sp.skip = true; sp.note = 'cable marker post'; }
        else sp.hidden = z.name;
      }
      if (trees.some(t => Math.hypot(t.x - x, t.y - y) < 2)) { sp.skip = true; sp.note = 'existing tree'; }
      sp.soil = verge(pc.s, p.soil); this.spots.push(sp);
    }
    const [x0, y0, h0] = CL.at(8);
    Object.assign(this, { x: x0, y: y0, th: h0, th2: h0, v: 0, delta: 0, hint: null, lhint: null });
    this.weave = FS.OU(p.weave, 8, this.r); this.wv = 0;
    this.est = 0; this.gnssWander = FS.OU(1.5, 30, this.r);
    this.limit = -1; this.limits = [[-1e9, -1]];
    this.k = 0; this.state = 'drive'; this.stateT = 0; this.cyc = null;
    this.sled = 0; this.carr = 1.5; this.tool = 0; this.toolGoal = 0;
    this.planted = 0; this.skipped = 0; this.strikes = 0; this.water = 1000; this.plugs = 1200; this.fuel = 0; this.energy = 0;
    this.hist = []; this.errs = []; this.cycles = []; this.lastPlantS = null; this.spacings = [];
    this.events = []; this.series = { t: [], v: [], P: [], tongue: [] }; this.lastSample = -9;
    this.done = false; this.driveT = 0; this.sinceStop = 0;
    this.headPred = FS.cycle('katar', this.soil, p.moist, .3, 1);
    this.vGov = Math.min(p.speed / 3.6, .85 * p.rail / (this.headPred.total - 1), p.spacing / (this.headPred.total + 3));
    this.railMin = p.mode === 'move' ? .6 - p.rail : -Math.min(1.8, p.rail * .75); this.railMax = p.mode === 'move' ? .6 : Math.min(.6, p.rail * .25);
    this.ev('ok', `${this.spots.length} spots at ${p.spacing} m, ${this.spots.filter(s => s.skip).length} no-dig; predicted cycle ${this.headPred.total.toFixed(1)} s`);
    if (p.mode === 'move') this.ev('ok', `governed speed ${(this.vGov * 3.6).toFixed(2)} km/h (sled ${p.rail} m, ${(this.headPred.total - 1).toFixed(1)} s in the ground)`);
  }
  ev(level, text) { this.events.unshift({ t: this.t, level, text }); if (this.events.length > 60) this.events.pop(); }

  /* ---------- geometry ---------- */
  trailerPose() { const hx = this.x - HITCH * Math.cos(this.th), hy = this.y - HITCH * Math.sin(this.th); return { hx, hy, th: this.th2 }; }
  bitXY(q, c) { const T = this.trailerPose(), ax = -.85 + q, ay = 1.0 + c, cs = Math.cos(T.th), sn = Math.sin(T.th); return [T.hx + cs * ax - sn * ay, T.hy + sn * ax + cs * ay]; }
  stationOf(x, y) { const r = LINE.project(x, y, this.lhint); this.lhint = r.i; return r; }
  massItems() {   // x measured rearward from the hitch, kg
    return [['Frame + racks', 1600, 2.7], ['Water', this.water, 1.6], ['Tank shell', 80, 1.6], ['Plants', this.plugs * .2, 3.4], ['Loader', 70, 3.1], ['Boom', 210, .85 - this.sled]];
  }
  tongue() { const ax = L2, items = this.massItems(); const W = items.reduce((a, m) => a + m[1], 0); const T = items.reduce((a, m) => a + m[1] * (ax - m[2]) / ax, 0); return { W, T, frac: T / W }; }

  step(dt) {
    if (this.done) return; const p = this.p; this.t += dt;
    // ---- driver
    const r = CL.project(this.x, this.y, this.hint); this.hint = r.i; this.sc = r.s;
    this.wv = this.weave(dt);
    const [gx0, gy0, gh] = CL.at(r.s + 5), gx = gx0 - Math.sin(gh) * this.wv, gy = gy0 + Math.cos(gh) * this.wv;
    const alpha = FS.wrap(Math.atan2(gy - this.y, gx - this.x) - this.th), kappa = 2 * Math.sin(alpha) / Math.max(Math.hypot(gx - this.x, gy - this.y), .1);
    this.delta = clamp(Math.atan(kappa * L1), -.55, .55);
    let lim = -1; for (const [ts, v] of this.limits) if (ts <= this.t - p.react) lim = v;
    let target = lim < 0 ? p.speed / 3.6 : Math.min(p.speed / 3.6, lim);
    if (r.s > 388) target = 0;
    const acc = target > this.v ? .4 : .9; this.v += clamp(target - this.v, -acc * dt, acc * dt);
    const w1 = this.v * Math.tan(this.delta) / L1;
    this.x += this.v * Math.cos(this.th) * dt; this.y += this.v * Math.sin(this.th) * dt; this.th += w1 * dt;
    this.th2 += ((this.v / L2) * Math.sin(this.th - this.th2) - (HITCH / L2) * w1 * Math.cos(this.th - this.th2)) * dt;
    // ---- localization error along the line
    if (p.gnss === 'rtk') this.est = FS.gauss(this.r) * .02;
    else if (p.gnss === 'std') this.est = this.gnssWander(dt);
    else this.est += .02 * this.v * dt + FS.gauss(this.r) * .004 * Math.sqrt(dt);
    // ---- power, fuel
    const tg = this.tongue(), loads = [M_TRACTOR * FS.G / 2 + tg.T * FS.G / 2, M_TRACTOR * FS.G / 2 + tg.T * FS.G / 2];
    const lat = SOILS.lateritic_gravel, rc = loads.reduce((a, W) => a + S.compaction(lat, S.sinkage(lat, W / 2, .4, .36), .4), 0)
      + 4 * S.compaction(lat, S.sinkage(lat, (tg.W - tg.T) * FS.G / 4, .26, .2), .26);
    this.Rc = rc;
    const Phyd = this.state === 'planting' && this.cyc && this.cyc.cur === 'bore' ? this.cyc.rate.power : 0;
    this.Pw = rc * this.v / .85 + Phyd / .8 + 3000;
    this.fuel += this.Pw * dt / 3.6e6 * .25 / .84;
    if (this.state !== 'planting') this.driveT += dt;
    this.planter(dt);
    if (this.t - this.lastSample > 2) { this.lastSample = this.t; const s = this.series; s.t.push(this.t); s.v.push(this.v * 3.6); s.P.push(this.Pw / 1000); s.tongue.push(tg.frac * 100); if (s.t.length > 400) for (const k in s) s[k].shift(); }
  }

  planter(dt) {
    const p = this.p;
    if (this.k >= this.spots.length) { if (!this.done) { this.done = true; this.limits.push([this.t, -1]);
      this.ev('ok', `row complete: ${this.planted} planted, ${this.skipped} skipped, ${(this.planted / Math.max(this.t, 1) * 3600).toFixed(0)}/h`); } return; }
    const sp = this.spots[this.k];
    const home = p.mode === 'move' ? this.railMax : 0;
    const bt = this.bitXY(home, 1.5), sTip = this.stationOf(bt[0], bt[1]).s + this.est, d = sp.s - sTip;
    this.dSpot = d;
    if (this.state === 'planting') { this.runCycle(dt, sp); return; }
    if (sp.skip) { if (d < 0) { this.skipped++; this.k++; this.ev('warn', `spot ${sp.id} skipped: ${sp.note}`); } return; }
    if (p.mode === 'move') {
      this.setLimit(this.vGov);
      if (d <= 0) { if (d < this.railMin - this.railMax) { this.skipped++; this.k++; this.ev('bad', `spot ${sp.id} missed: passed before the head was free`); } else this.startCycle(sp, true); }
      return;
    }
    if (this.state === 'drive') {
      const v = Math.max(this.v, 0), stop = v * p.react + v * v / (2 * .9) + .15;
      if (d < stop) { this.setLimit(0); this.state = 'stopping'; this.stateT = this.t; }
    } else if (this.state === 'stopping' && Math.abs(this.v) < .03 && this.t - this.stateT > 1) {
      if (d >= this.railMin && d <= this.railMax) this.startCycle(sp, false);
      else if (d > this.railMax) { this.setLimit(-1); this.state = 'drive'; }
      else { this.skipped++; this.k++; this.ev('bad', `spot ${sp.id} overshot by ${(-d).toFixed(2)} m (sled range ${this.railMin.toFixed(1)} m)`); this.setLimit(-1); this.state = 'drive'; }
    }
  }
  setLimit(v) { if (this.limit !== v) { this.limit = v; this.limits.push([this.t, v]); if (this.limits.length > 20) this.limits.shift(); } }

  startCycle(sp, track) {
    const T = this.trailerPose(), cs = Math.cos(T.th), sn = Math.sin(T.th), dx = sp.x - T.hx, dy = sp.y - T.hy;
    const ax = cs * dx + sn * dy, ay = -sn * dx + cs * dy, qs = ax + .85 - (this.p.mode === 'stop' ? this.est : 0), qc = ay - 1.0;
    if (qc < .5 || qc > 2.5) { this.skipped++; this.k++; this.ev('warn', `spot ${sp.id}: outside boom reach (${(qc + 1).toFixed(2)} m)`); this.setLimit(-1); this.state = 'drive'; return; }
    const locate = Math.max(Math.abs(qs - this.sled) / .9, Math.abs(qc - this.carr) / .6, .3);
    this.cyc = FS.cycle('katar', SOILS[sp.soil], this.p.moist, .3, 1, { locate });
    this.cyc.t = 0; this.cyc.i = 0; this.cyc.cur = 'locate'; this.cyc.track = track; this.cyc.qs0 = this.sled; this.cyc.qc0 = this.carr; this.cyc.qs = qs; this.cyc.qc = qc;
    this.cyc.drive = this.driveT; this.driveT = 0; this.state = 'planting'; this.curSpot = sp;
  }
  runCycle(dt, sp) {
    const c = this.cyc; c.t += dt; let acc = 0, cur = c.phases.length - 1;
    for (let i = 0; i < c.phases.length; i++) { if (c.t < acc + c.phases[i][1]) { cur = i; break; } acc += c.phases[i][1]; }
    const name = c.phases[cur][0], u = (c.t - acc) / c.phases[cur][1]; c.cur = name;
    if (name === 'locate') { this.sled = FS.lerp(c.qs0, c.qs, FS.ease(u)); this.carr = FS.lerp(c.qc0, c.qc, FS.ease(u)); }
    if (c.track) { const T = this.trailerPose(), cs = Math.cos(T.th), sn = Math.sin(T.th), dx = sp.x - T.hx, dy = sp.y - T.hy; this.sled = clamp(cs * dx + sn * dy + .85, this.railMin - .05, .6); }
    const h = FS.HEADS.katar;
    this.tool = name === 'lower' ? u * h.rest : name === 'bore' ? h.rest + u * .3 : name === 'retract' ? (h.rest + .3) * (1 - u) : 0;
    if (c.t >= c.total) this.finish(sp);
  }
  finish(sp) {
    const c = this.cyc, [bx, by] = this.bitXY(this.sled, this.carr), err = Math.hypot(bx - sp.x, by - sp.y);
    this.planted++; this.plugs--; this.water -= 1; this.energy += c.energy; this.errs.push(err);
    if (this.lastPlantS != null) this.spacings.push(this.stationOf(bx, by).s - this.lastPlantS);
    this.lastPlantS = this.stationOf(bx, by).s;
    this.cycles.push({ drive: c.drive, phases: c.phases }); if (this.cycles.length > 40) this.cycles.shift();
    this.hist.push({ x: bx, y: by, ok: true, id: sp.id });
    if (sp.hidden) { this.strikes++; this.ev('bad', `UTILITY STRIKE at spot ${sp.id}: bored into the ${sp.hidden}. A survey would have prevented it.`); }
    this.newPlant = { x: bx, y: by, id: sp.id }; this.k++; this.state = 'drive'; this.tool = 0;
    if (this.p.mode === 'stop') this.setLimit(-1);
    if (c.track) this.sled = this.railMax;
  }

  /* ---------- 3D ---------- */
  build3D() {
    const W = FS.T3.world, B = FS.B, C = FS.C, M = FS.MAT;
    FS.groundMesh(460, 90, 115, 22, () => 0, (x, z) => FS.mixHex(0xA49B6C, 0x8A8857, .5 + .5 * FS.noise2(x * .5, z * .5)), 175, 0);
    FS.ribbon(CL, -8.5, -1.3, .012, M.asphalt); FS.ribbon(CL, -1.3, 1.3, .008, M.gravel); FS.ribbon(CL, -1.48, -1.36, .016, M.paint);
    this.trees.forEach((t, i) => FS.bigTree(t.x, -t.y, .8 + (i * 37 % 10) / 20));
    for (let s = 100; s < 380; s += 100) { const [x, y, h] = CL.at(s); const g = new THREE.Group(); g.position.set(x - Math.sin(h) * 1.6, 0, -(y + Math.cos(h) * 1.6)); g.rotation.y = h; W.add(g);
      FS.add(g, B(.14, .55, .32), M.white, 0, .275, 0); const tp = FS.add(g, C(.16, .16, .145, 16), M.milestone, 0, .55, 0); tp.rotation.x = Math.PI / 2; }
    POSTS.forEach(s => { const [x, y, h] = CL.at(s); FS.add(W, C(.05, .05, 1, 8), M.orange, x - Math.sin(h) * 1.75, .5, -(y + Math.cos(h) * 1.75)); });
    { const [x, y, h] = CL.at(154); const g = new THREE.Group(); g.position.set(x - Math.sin(h) * 2, 0, -(y + Math.cos(h) * 2)); g.rotation.y = h; W.add(g);
      [-4, 4].forEach(dx => FS.add(g, B(.4, .9, 7), M.concrete, dx, .45, 0)); }
    ZONES.forEach(z => { const seg = new Path(CL.p.filter((_, i) => CL.s[i] >= z.s0 - 1 && CL.s[i] <= z.s1 + 1)); if (seg.p.length > 1) FS.ribbon(seg, z.lat0, z.lat1, .02, z.visible ? M.zone : M.zoneHidden); });
    this.rings = new FS.RingPool(this.spots, () => 0); this.pool = new FS.PlantPool(this.spots.length + 4);
    const l = FS.makeLabel('Buried cable', this.p.survey === 'yes' ? 'surveyed · avoided' : 'not surveyed'); const [cx, cy] = CL.at(260); l.position.set(cx, 3.2, -(cy + 3)); W.add(l);
    this.m = FS.buildKatar(); W.add(this.m.root);
  }
  sync3D(dt) {
    const m = this.m; if (!m) return;
    m.root.position.set(this.x, 0, -this.y); m.root.rotation.y = this.th; m.hitch.rotation.y = this.th2 - this.th;
    m.steer.forEach(s => s.rotation.y = this.delta);
    m.spins.forEach(w => w.spin.rotation.z -= this.v * dt / w.r);
    m.sled.position.x = -2.0 + this.sled; m.carr.position.z = -(1.0 + this.carr); m.setTool(this.tool); if (this.state === 'planting' && this.cyc && ['lower', 'bore', 'retract'].includes(this.cyc.cur)) m.spinTool(dt);
    m.waterLevel.position.y = .98 + 1.08 * this.water / 1000;
    m.trays.forEach((tr, i) => tr.visible = this.plugs > i * 200);
    if (this.newPlant) { this.pool.add(this.newPlant.x, this.newPlant.y, 0); this.rings.color(this.newPlant.id, 'done'); this.newPlant = null; }
    if (this.k > 0 && this.spots[this.k - 1] && this.spots[this.k - 1].skip) this.rings.color(this.k - 1, 'skip');
    this.pool.update(dt);
  }
  camTarget() { return [this.x - 3 * Math.cos(this.th), this.y - 3 * Math.sin(this.th), 1]; }
  camOffset() { return [Math.cos(this.th) * 4 - Math.sin(this.th) * -11, 7, -(Math.sin(this.th) * 4 + Math.cos(this.th) * -11)]; }
  progress() { return this.k / this.spots.length; }

  hud() {
    const ph = this.state === 'planting' ? this.cyc.cur : this.state;
    return { title: `A · Katar — roadside row, ${this.p.spacing} m`, rows: [
      ['Planted', `${this.planted} / ${this.spots.filter(s => !s.skip).length}`], ['Station', `${FS.n(this.sc || 0)} m`],
      ['Speed', `${(this.v * 3.6).toFixed(2)} km/h`], ['Head', ph], ['Rate', this.t > 60 ? `${FS.n(this.planted / this.t * 3600)}/h` : '—']] };
  }
  tele() {
    const tg = this.tongue(), mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN;
    const sp = this.spacings, sd = sp.length > 1 ? Math.sqrt(mean(sp.map(v => (v - mean(sp)) ** 2))) : NaN;
    return [
      ['Run'], ['Planted · skipped', `${this.planted} · ${this.skipped}`], ['Utility strikes', String(this.strikes), this.strikes ? 'bad' : 'ok'],
      ['Planting rate', this.t > 60 ? `${FS.n(this.planted / this.t * 3600)} /h` : '—'], ['Mean cycle', this.cycles.length ? `${FS.n(mean(this.cycles.map(c => c.phases.reduce((a, q) => a + q[1], 0) + c.drive)), 1)} s` : '—'],
      ['Placement'], ['Error mean', this.errs.length ? `${FS.n(mean(this.errs) * 100, 1)} cm` : '—', this.errs.length && mean(this.errs) > .25 ? 'warn' : null],
      ['Spacing mean ± σ', sp.length ? `${FS.n(mean(sp), 2)} ± ${FS.n(sd * 100, 0)} cm` : '—'], ['Station estimate error', `${FS.n(this.est * 100, 0)} cm`, Math.abs(this.est) > .5 ? 'warn' : null],
      ['Rig'], ['Driver weave', `${FS.n(this.wv * 100, 0)} cm`], ['Articulation', `${FS.n(FS.deg(this.th - this.th2), 1)}°`],
      ['Sled · reach', `${FS.n(this.sled, 2)} m · ${FS.n(this.carr + 1, 2)} m`], ['Tongue load', `${FS.n(tg.frac * 100, 1)} %`, tg.frac < .05 ? 'warn' : tg.frac > .3 ? 'warn' : 'ok'],
      ['Tractor power', `${FS.n(this.Pw / 1000, 1)} kW`], ['Fuel · per 100', `${FS.n(this.fuel, 1)} L · ${this.planted ? FS.n(this.fuel / this.planted * 100, 1) : '—'} L`],
      ['Water · plugs', `${FS.n(this.water)} L · ${FS.n(this.plugs)}`],
    ];
  }

  /* ---------- diagrams ---------- */
  figs() {
    return [
      { id: 'plan', title: 'Plan view', tag: 'top-down, travel to the right', wide: true, draw: (c, w, h) => this.drawPlan(c, w, h),
        caption: 'Road, gravel shoulder and verge with the planting line 2.5 m in. The blue band is what the boom can reach (1.5–3.5 m from the trailer axis, sled travel along it). Red: no-dig zones; purple: the buried cable, red only if surveyed.' },
      { id: 'cycle', title: 'Where each plant’s time goes', tag: 'seconds per planting', draw: (c, w, h) => this.drawCycle(c, w, h),
        caption: 'Bars are the last 12 plantings, phase by phase. Drive time is the travel and stopping between spots. In hard soil the bore bar grows; in on-the-move mode the drive bar vanishes but speed is capped.' },
      { id: 'load', title: 'Trailer load', tag: 'side elevation · live', draw: (c, w, h) => this.drawLoad(c, w, h),
        caption: 'Weights act down at their positions; the hitch (tongue) and the tandem axle carry them. As the 1,000 L tank empties the tongue share falls; below about 5 % a trailer starts to sway.' },
      { id: 'spacing', title: 'Spacing achieved', tag: 'distance between consecutive holes', draw: (c, w, h) => this.drawSpacing(c, w, h),
        caption: 'With RTK the sled corrects every stop to within a few centimetres. Switch to plain GNSS or odometry to see the spread widen.' },
      { id: 'power', title: 'Speed and tractor power', tag: 'last 13 minutes', draw: (c, w, h) => this.drawPower(c, w, h),
        caption: 'Stop-and-go shows the saw-tooth of driving and waiting; power peaks while the hydraulic auger cuts.' },
    ];
  }
  drawPlan(c, w, h) {
    const sc = clamp(w / 52, 9, 18), cx = w * .42, cy = h * .58, th = this.th;
    const P = (x, y) => { const dx = x - this.x, dy = y - this.y, cs = Math.cos(-th), sn = Math.sin(-th); return [cx + (cs * dx - sn * dy) * sc, cy - (sn * dx + cs * dy) * sc]; };
    const band = (path, a, b, fill) => { const s0 = clamp(this.sc - w / sc, 0, path.length), s1 = clamp(this.sc + w / sc, 0, path.length), L = [], R = [];
      for (let s = s0; s <= s1; s += 1) { const [x, y, hh] = path.at(s); L.push(P(x - Math.sin(hh) * a, y + Math.cos(hh) * a)); R.push(P(x - Math.sin(hh) * b, y + Math.cos(hh) * b)); }
      FS.poly(c, L.concat(R.reverse()), fill); };
    c.fillStyle = FS.alpha(TH.leaf, .07); c.fillRect(0, 0, w, h);
    band(CL, -8.5, -1.3, FS.alpha(TH.muted, .35)); band(CL, -1.3, 1.3, FS.alpha(TH.accent, .14));
    ZONES.forEach(z => { if (Math.abs(z.s0 - this.sc) < 60) { const seg = new Path(CL.p.filter((_, i) => CL.s[i] >= z.s0 && CL.s[i] <= z.s1));
      if (seg.p.length > 1) { const o0 = seg.offset(z.lat0).p.map(q => P(q[0], q[1])), o1 = seg.offset(z.lat1).p.map(q => P(q[0], q[1]));
        const vis = z.visible || this.p.survey === 'yes'; FS.poly(c, o0.concat(o1.reverse()), FS.alpha(vis ? TH.bad : TH.c4, .18), vis ? TH.bad : TH.c4, 1); } } });
    const s0 = this.stationOf(this.x, this.y).s; const lp = []; for (let s = s0 - w / sc; s < s0 + w / sc; s += 1) { const q = LINE.at(s); lp.push(P(q[0], q[1])); }
    FS.line(c, lp, TH.accent, 1.2, [5, 4]);
    this.trees.forEach(t => { if (Math.hypot(t.x - this.x, t.y - this.y) < 60) { const q = P(t.x, t.y); FS.circle(c, q[0], q[1], 2 * sc * .6, FS.alpha(TH.leaf, .25), TH.leaf, 1); } });
    this.spots.forEach((s, i) => { if (Math.abs(s.s - s0) > w / sc + 5) return; const q = P(s.x, s.y);
      const col = i < this.k ? (s.skip ? TH.bad : TH.leaf) : s.skip ? TH.bad : TH.accent; FS.circle(c, q[0], q[1], i < this.k && !s.skip ? 4 : 3, i < this.k && !s.skip ? col : 'transparent', col, 1.5); });
    this.hist.slice(-80).forEach(hp => { const q = P(hp.x, hp.y); FS.circle(c, q[0], q[1], 2, TH.leaf); });
    // rig outline
    const T = this.trailerPose(), rect = (ox, oy, th2, x0, x1, y0, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([a, b]) => P(ox + Math.cos(th2) * a - Math.sin(th2) * b, oy + Math.sin(th2) * a + Math.cos(th2) * b));
    // reach envelope (sled travel x carriage travel)
    FS.poly(c, rect(T.hx, T.hy, T.th, -.85 + this.railMin, -.85 + this.railMax, 1.5, 3.5), FS.alpha(TH.water, .2), TH.water, 1);
    FS.poly(c, rect(T.hx, T.hy, T.th, -4.7, -.2, -1, 1), FS.alpha(TH.ink, .12), TH.ink, 1.2);
    FS.poly(c, rect(this.x, this.y, this.th, -.6, 2.8, -.9, .9), FS.alpha(TH.mark, .5), TH.ink, 1.2);
    const b = this.bitXY(this.sled, this.carr), bq = P(b[0], b[1]); FS.circle(c, bq[0], bq[1], 4.5, TH.water, TH.plot, 1.5);
    FS.line(c, [P(...this.bitXY(this.sled, 0.5)), bq], TH.water, 2);
    // labels
    FS.text(c, 'road', 12, P(...CL.at(this.sc).slice(0, 2))[1] + 5 * sc * .7, { color: TH.muted, size: 10.5 });
    FS.text(c, 'planting line', 12, lp.length ? lp[0][1] - 6 : 20, { color: TH.accent, size: 10.5 });
    const d = this.dSpot; if (Number.isFinite(d)) FS.text(c, `next spot ${d >= 0 ? 'in' : 'passed'} ${Math.abs(d).toFixed(2)} m · ${this.state === 'planting' ? this.cyc.cur : this.state}`, w - 12, 18, { align: 'right', color: TH.ink, size: 11, halo: true });
    const bar = 5 * sc; FS.line(c, [[12, h - 14], [12 + bar, h - 14]], TH.ink, 2); FS.text(c, '5 m', 16 + bar, h - 10, { color: TH.muted, size: 10.5 });
  }
  drawCycle(c, w, h) {
    const cy = this.cycles.slice(-12), names = ['drive', 'locate', 'lower', 'bore', 'retract', 'place', 'close', 'water'];
    const col = { drive: TH.muted, locate: TH.c5, lower: TH.water, bore: TH.accent, retract: TH.c4, place: TH.leaf, close: TH.ok, water: TH.water };
    const rows = cy.map(k => { const m = { drive: k.drive }; k.phases.forEach(([n, s]) => m[n] = s); return m; });
    const tot = rows.map(m => names.reduce((a, n) => a + (m[n] || 0), 0)), xmax = Math.max(15, ...tot) * 1.08;
    const ly = FS.swatches(c, names.map(n => [n, col[n]]), 46, 16, w - 60);
    const box = { x: 46, y: ly + 10, w: w - 60, h: h - ly - 46 }; const { X } = FS.frame(c, box, [0, xmax], [0, 1], { yn: 1, yfmt: () => '', xlabel: 's' });
    const bh = Math.min(14, box.h / Math.max(rows.length, 1) - 3);
    rows.forEach((m, i) => { let x0 = 0; const y = box.y + i * (bh + 3) + 2; names.forEach(n => { const v = m[n] || 0; if (v > 0) FS.poly(c, [[X(x0), y], [X(x0 + v), y], [X(x0 + v), y + bh], [X(x0), y + bh]], col[n]); x0 += v; });
      FS.text(c, FS.n(tot[i], 1), X(x0) + 4, y + bh - 3, { color: TH.muted, size: 10 }); });
    if (!rows.length) FS.text(c, 'waiting for the first planting…', box.x + 10, box.y + 24, { color: TH.muted });
  }
  drawLoad(c, w, h) {
    const items = this.massItems(), tg = this.tongue(), x0 = 40, x1 = w - 30, gy = h - 40, sc = (x1 - x0) / 4.4, X = v => x0 + v * sc;
    FS.line(c, [[x0 - 20, gy], [x1 + 10, gy]], TH.muted, 1);
    FS.poly(c, [[X(0), gy - 50], [X(4.3), gy - 50], [X(4.3), gy - 38], [X(0), gy - 38]], FS.alpha(TH.ink, .15), TH.ink, 1.2);
    FS.circle(c, X(L2 - .43), gy - 14, 14, TH.plot, TH.ink, 1.5); FS.circle(c, X(L2 + .43), gy - 14, 14, TH.plot, TH.ink, 1.5);
    FS.poly(c, [[X(0) - 6, gy - 44], [X(0) + 6, gy - 44], [X(0), gy - 34]], TH.ink);
    const maxW = Math.max(...items.map(m => m[1]));
    items.slice().sort((a, b) => a[2] - b[2]).forEach(([name, kg, x], i) => { const L = 10 + 50 * kg / maxW; FS.arrow(c, X(x), gy - 52 - L, X(x), gy - 52, TH.accent, 1.8);
      FS.line(c, [[X(x), gy - 52 - L], [X(x), gy - 120 - (i % 3) * 13]], FS.alpha(TH.muted, .5), 1, [2, 2]);
      FS.text(c, `${name} ${FS.n(kg)} kg`, X(x), gy - 124 - (i % 3) * 13, { align: 'center', size: 10, color: TH.ink, halo: true }); });
    const T = tg.T * FS.G / 1000, A = (tg.W - tg.T) * FS.G / 1000;
    FS.arrow(c, X(0), gy + 26, X(0), gy - 30, tg.frac < .05 || tg.frac > .3 ? TH.warn : TH.leaf, 2.2);
    FS.arrow(c, X(L2), gy + 26, X(L2), gy, TH.leaf, 2.2);
    FS.text(c, `hitch ${FS.n(T, 1)} kN (${FS.n(tg.frac * 100, 1)} %)`, X(0) + 8, gy + 22, { size: 11, color: TH.ink });
    FS.text(c, `axle ${FS.n(A, 1)} kN`, X(L2) + 8, gy + 22, { size: 11, color: TH.ink });
    const cgx = items.reduce((a, m) => a + m[1] * m[2], 0) / tg.W; FS.circle(c, X(cgx), gy - 44, 5, TH.bad, TH.plot, 1.5);
    FS.text(c, 'CG', X(cgx) + 8, gy - 40, { size: 10, color: TH.bad });
    FS.text(c, 'hitch', X(0), gy - 60 + 70, { align: 'center', size: 10, color: TH.muted });
  }
  drawSpacing(c, w, h) {
    const v = this.spacings.filter(x => Math.abs(x - this.p.spacing) < 3 * this.p.spacing), sp = this.p.spacing;
    const lo = sp - .6, hi = sp + .6, bins = 24, cnt = new Array(bins).fill(0);
    v.forEach(x => { const b = clamp(Math.floor((x - lo) / (hi - lo) * bins), 0, bins - 1); cnt[b]++; });
    const box = { x: 40, y: 22, w: w - 54, h: h - 58 }, ymax = Math.max(4, ...cnt), { X, Y } = FS.frame(c, box, [lo, hi], [0, ymax], { xlabel: 'm', ylabel: 'gaps' });
    cnt.forEach((n, i) => { const a = lo + i * (hi - lo) / bins, b = a + (hi - lo) / bins; if (n) FS.poly(c, [[X(a) + 1, Y(n)], [X(b) - 1, Y(n)], [X(b) - 1, Y(0)], [X(a) + 1, Y(0)]], TH.leaf); });
    FS.line(c, [[X(sp), box.y], [X(sp), box.y + box.h]], TH.accent, 1.5, [4, 3]);
    FS.text(c, `target ${sp} m`, X(sp) + 4, box.y + 10, { color: TH.accent, size: 10.5 });
    if (!v.length) FS.text(c, 'needs two plantings in a row…', box.x + 10, box.y + 30, { color: TH.muted });
  }
  drawPower(c, w, h) {
    const s = this.series; if (s.t.length < 2) { FS.text(c, 'collecting…', 50, 40, { color: TH.muted }); return; }
    const box = { x: 40, y: 24, w: w - 84, h: h - 56 }, t0 = s.t[0], t1 = s.t[s.t.length - 1];
    const { X, Y } = FS.frame(c, box, [t0 / 60, Math.max(t1 / 60, t0 / 60 + 1)], [0, Math.max(14, ...s.P) * 1.1], { xlabel: 'min', ylabel: 'kW' });
    FS.series(c, X, Y, s.t.map((t, i) => [t / 60, s.P[i]]), TH.accent, { area: true });
    const vmax = 5, Y2 = v => box.y + box.h - v / vmax * box.h;
    FS.series(c, X, Y2, s.t.map((t, i) => [t / 60, s.v[i]]), TH.water, { w: 1.5 });
    [0, 1, 2, 3, 4, 5].forEach(v => FS.text(c, String(v), box.x + box.w + 6, Y2(v) + 3.5, { color: TH.water, size: 10 }));
    FS.text(c, 'km/h', box.x + box.w + 6, box.y - 8, { color: TH.water, size: 10.5 });
    FS.legend(c, [['tractor power', TH.accent], ['speed', TH.water]], box.x + 6, box.y + 12);
  }
}
})();
