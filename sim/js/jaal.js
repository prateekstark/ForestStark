/* B · Jaal — autonomous swerve-drive fleet on a surveyed grid with a tender truck.
   Same logic as fs_autonomy jaal_fleet_manager + jaal_navigator + gnss_degrader. */
(() => {
const { clamp, SOILS, TH } = FS;
FS.SIMS = FS.SIMS || {};
const GANTRY = 1.65, HEADLAND = 2.8;
const STATES = ['drive', 'turn', 'plant', 'detour', 'to_tender', 'queue', 'service', 'return', 'safety_stop', 'wait_fix', 'done'];

function makeField(p, seed) {
  const r = FS.rng(seed), W = p.field === 'full' ? 120 : 60, H = p.field === 'full' ? 84 : 42;
  const obs = [], free = (x, y, rr) => obs.every(o => Math.hypot(o.x - x, o.y - y) > o.r + rr + 1);
  const scale = W * H / (120 * 84);
  for (const [kind, n, r0, r1, h0, h1] of [['termite mound', 7, .8, 1.4, .9, 1.6], ['stump', 16, .25, .35, .3, .5], ['stone', 70, .15, .32, .12, .25]]) {
    let k = 0, guard = 0; while (k < Math.round(n * scale) && guard++ < 4000) { const x = 2 + r() * (W - 4), y = 2 + r() * (H - 4), rr = r0 + r() * (r1 - r0);
      if (free(x, y, rr)) { obs.push({ kind, x, y, r: rr, h: h0 + r() * (h1 - h0) }); k++; } }
  }
  const dy = p.pattern === 'square' ? p.spacing : p.spacing * Math.sqrt(3) / 2, rows = [];
  for (let y = 1.5, j = 0; y <= H - 1.5 + 1e-9; y += dy, j++) { const sh = p.pattern === 'triangular' && j % 2 ? p.spacing / 2 : 0, row = [];
    for (let x = 1.5 + sh; x <= W - 1.5 + 1e-9; x += p.spacing) row.push({ x, y, row: j }); rows.push(row); }
  // survey: mounds and stumps are known (shift or skip); stones are not
  let id = 0;
  rows.forEach(row => row.forEach(s => { s.id = id++; s.state = 'todo';
    const bl = obs.filter(o => o.kind !== 'stone' && Math.hypot(o.x - s.x, o.y - s.y) < o.r + .35);
    if (bl.length) { const dx = [.3, -.3, .6, -.6].find(d => obs.every(o => o.kind === 'stone' || Math.hypot(o.x - s.x - d, o.y - s.y) >= o.r + .35));
      if (dx != null) { s.x += dx; s.note = `shifted ${dx > 0 ? '+' : ''}${dx} m`; } else { s.skip = true; s.state = 'skip'; s.note = 'on a ' + bl[0].kind; } }
    s.stone = obs.some(o => o.kind === 'stone' && Math.hypot(o.x - s.x, o.y - s.y) < o.r + .06); }));
  return { W, H, dy, rows, obs, tender: { x: -14, y: 10 }, treeline: H - 10 };
}

function splitRows(rows, n) {
  const cnt = rows.map(r => r.filter(s => !s.skip).length), cum = [0]; cnt.forEach(c => cum.push(cum[cum.length - 1] + c));
  const tot = cum[cum.length - 1], cuts = [0];
  for (let k = 1; k < n; k++) { const tgt = tot * k / n; let j = 0, best = 1e9; cum.forEach((v, i) => { if (Math.abs(v - tgt) < best) { best = Math.abs(v - tgt); j = i; } });
    cuts.push(clamp(j, cuts[cuts.length - 1] + 1, rows.length - (n - k))); }
  cuts.push(rows.length); return Array.from({ length: n }, (_, k) => Array.from({ length: Math.max(0, cuts[k + 1] - cuts[k]) }, (_, i) => cuts[k] + i));
}

class Fleet {
  constructor(p, headless = false) {
    this.p = p; this.headless = headless; this.t = 0; this.r = FS.rng(11);
    this.f = makeField(p, 11); this.soil = FS.soil.atMoisture(SOILS[p.soil], p.moist);
    this.cyc = FS.cycle('jaal', this.soil, p.moist, .25, .5); this.cycStone = FS.cycle('jaal', SOILS.lateritic_gravel, p.moist, .03, 0);
    this.stock = p.stock; this.restockAt = null; this.bays = new Array(p.bays).fill(null); this.queue = [];
    this.events = []; this.timeline = []; this.series = { t: [], planted: [], soc: [] }; this.lastSample = -99; this.errs = [];
    this.cows = Array.from({ length: p.cows }, (_, i) => ({ x: this.f.W * (.3 + .4 * i), y: this.f.H * .5, yaw: i, walk: false, until: 0 }));
    const blocks = splitRows(this.f.rows, Math.min(p.n, this.f.rows.length));
    this.robots = blocks.map((b, i) => this.makeRobot(i, b));
    this.total = this.f.rows.flat().filter(s => !s.skip).length; this.planted = 0; this.skipped = this.f.rows.flat().filter(s => s.skip).length;
    this.done = false;
    if (!headless) this.ev('ok', `${this.total} spots on ${(this.f.W * this.f.H / 1e4).toFixed(2)} ha (${Math.round(this.total / (this.f.W * this.f.H / 1e4))}/ha), ${p.n} robots, predicted cycle ${this.cyc.total.toFixed(1)} s`);
  }
  ev(level, text) { if (this.headless) return; this.events.unshift({ t: this.t, level, text }); if (this.events.length > 80) this.events.pop(); }
  makeRobot(i, block) {
    const name = `jaal_${i + 1}`, R = { name, i, x: -6, y: 2 + 4.5 * i, yaw: 0, vx: 0, vy: 0, w: 0,
      plugs: this.p.plugs, water: 250, soc: .9, state: 'idle', tasks: [], task: null, planted: 0, est: [0, 0], gnss: 'rtk_fix', dropUntil: -1, drBias: 0, timer: 0, lastState: null };
    let dir = 1; block.forEach(j => { const row = this.f.rows[j].slice(); if (dir < 0) row.reverse(); const d = row.length > 1 ? Math.sign(row[row.length - 1].x - row[0].x) || dir : dir, yaw = d > 0 ? 0 : Math.PI, y = row[0].y;
      R.tasks.push({ k: 'goto', x: row[0].x - d * (HEADLAND - GANTRY), y, yaw, tol: .15, label: 'turn' }); let prev = row[0].x - d * (HEADLAND - GANTRY);
      row.forEach(s => { const stop = s.x + d * GANTRY; this.detours(prev, stop, y, d, yaw).forEach(t => R.tasks.push(t));
        if (s.skip) return; R.tasks.push({ k: 'goto', x: stop, y: s.y, yaw, tol: .03, label: 'drive' }, { k: 'plant', s, d, yaw, retry: 0 }); prev = stop; });
      const ex = row[row.length - 1].x + d * (GANTRY + HEADLAND); this.detours(prev, ex, y, d, yaw).forEach(t => R.tasks.push(t)); R.tasks.push({ k: 'goto', x: ex, y, yaw, tol: .2, label: 'drive' }); dir = -dir; });
    R.tasks.push({ k: 'park' }); return R;
  }
  detours(x0, x1, y, d, yaw) {
    const lo = Math.min(x0, x1), hi = Math.max(x0, x1), out = [];
    this.f.obs.filter(o => o.kind !== 'stone' && o.x > lo - o.r - 1.5 && o.x < hi + o.r + 1.5 && Math.abs(o.y - y) < o.r + 1.2).sort((a, b) => d * (a.x - b.x)).forEach(o => {
      const side = o.y <= y ? 1 : -1, off = o.y + side * (o.r + 1.4), a = o.x - d * (o.r + 2), b = o.x + d * (o.r + 2);
      out.push({ k: 'goto', x: a, y, yaw, tol: .15, label: 'detour' }, { k: 'goto', x: a, y: off, yaw, tol: .15, label: 'detour' }, { k: 'goto', x: b, y: off, yaw, tol: .15, label: 'detour' }, { k: 'goto', x: b, y, yaw, tol: .1, label: 'detour' }); });
    return out;
  }
  setState(R, s) { if (R.state === s) return; R.state = s; const last = this.timeline.filter(e => e.r === R.i && e.t1 == null).pop(); if (last) last.t1 = this.t; this.timeline.push({ r: R.i, s, t0: this.t, t1: null }); if (this.timeline.length > 4000) this.timeline.splice(0, 1000); }

  step(dt) {
    if (this.done) return; this.t += dt;
    if (this.restockAt != null && this.t >= this.restockAt) { this.stock += 1080; this.restockAt = null; this.ev('ok', `tender back from the nursery with 6 drums (1,080 plugs)`); }
    this.cows.forEach(c => { if (this.t > c.until) { c.walk = !c.walk; c.until = this.t + (c.walk ? 15 + 25 * this.r() : 20 + 70 * this.r()); c.yaw += FS.gauss(this.r) * .8; }
      if (c.walk) { const nx = c.x + .4 * Math.cos(c.yaw) * dt, ny = c.y + .4 * Math.sin(c.yaw) * dt; if (nx < -4 || nx > this.f.W + 4 || ny < -4 || ny > this.f.H + 4) c.yaw += Math.PI; else { c.x = nx; c.y = ny; } } });
    this.robots.forEach(R => this.stepRobot(R, dt));
    if (this.robots.every(R => R.state === 'done')) { this.done = true; this.ev('ok', `block complete in ${FS.fmtT(this.t)}: ${this.planted} planted, ${this.skipped} skipped`); }
    if (this.t - this.lastSample > (this.headless ? 60 : 10)) { this.lastSample = this.t; const s = this.series; s.t.push(this.t); s.planted.push(this.planted); s.soc.push(this.robots.map(R => R.soc)); }
  }
  gnssUpdate(R, dt) {
    const p = this.p; if (p.dropouts > 0 && this.t > R.dropUntil && this.r() < p.dropouts * dt / 3600) { R.dropUntil = this.t + 20 + 70 * this.r(); this.ev('warn', `${R.name}: RTK lost for ${(R.dropUntil - this.t).toFixed(0)} s`); }
    let g = 'rtk_fix'; if (this.t < R.dropUntil) g = 'dead_reckoning'; else if (p.treeline && R.y > this.f.treeline) g = 'float';
    if (g !== R.gnss && g !== 'rtk_fix' && R.gnss === 'rtk_fix') this.ev('warn', `${R.name}: GNSS ${g.replace('_', ' ')}`);
    R.gnss = g; const v = Math.hypot(R.vx, R.vy);
    if (g === 'rtk_fix') { R.est = [FS.gauss(this.r) * .02, FS.gauss(this.r) * .02]; R.drBias = FS.gauss(this.r) * .004; }
    else if (g === 'float') { const a = Math.exp(-dt / 20), s = .35 * Math.sqrt(1 - a * a); R.est = [a * R.est[0] + s * FS.gauss(this.r), a * R.est[1] + s * FS.gauss(this.r)]; }
    else { R.drBias += FS.gauss(this.r) * .0003 * Math.sqrt(dt); R.est[0] += (.012 * v * Math.cos(R.yaw) - R.drBias * v * Math.sin(R.yaw)) * dt; R.est[1] += (.012 * v * Math.sin(R.yaw) + R.drBias * v * Math.cos(R.yaw)) * dt; }
  }
  safety(R, ux, uy) {
    if (Math.hypot(ux, uy) < 1e-3) return 1; const L = Math.hypot(ux, uy); ux /= L; uy /= L;
    const cs = Math.cos(R.yaw), sn = Math.sin(R.yaw), front = Math.abs(ux) > .7 ? 1.45 : .95; let near = 1e9;
    const test = (wx, wy, rad) => { const dx = wx - R.x, dy = wy - R.y, bx = cs * dx + sn * dy, by = -sn * dx + cs * dy, al = bx * ux + by * uy, ac = Math.abs(-bx * uy + by * ux);
      if (al > 0 && ac < 1.1 + rad) near = Math.min(near, al - front - rad); };
    this.cows.forEach(c => test(c.x, c.y, .8));
    this.robots.forEach(o => { if (o !== R && o.i < R.i) test(o.x, o.y, 1.4); });
    if (near < 1.5) return 0; if (near < 5) return .2 + .8 * (near - 1.5) / 3.5; return 1;
  }
  stepRobot(R, dt) {
    this.gnssUpdate(R, dt);
    const p = this.p, P = 350 + Math.hypot(R.vx, R.vy) * .07 * 1000 * FS.G / .85 + Math.abs(R.w) * 1500 + (R.boring ? 4000 : 0) - p.solar;
    R.soc = Math.max(0, R.soc - P * dt / (30 * 3.6e6));
    if (!R.task) { if (!R.tasks.length) { this.setState(R, 'done'); R.vx = R.vy = R.w = 0; return; } R.task = R.tasks.shift(); R.timer = 0; }
    const k = R.task; R.timer += dt; R.boring = false;
    if (k.k === 'goto') {
      if (R.gnss !== 'rtk_fix' && p.rtk === 'wait' && (R.waitFix = (R.waitFix || 0) + dt) < 60) { this.setState(R, 'wait_fix'); R.vx = R.vy = R.w = 0; return; }
      if (R.gnss === 'rtk_fix') R.waitFix = 0;
      this.setState(R, k.label);
      // the robot steers its *estimate* onto the target, so the truth lands est-error away
      const ex = R.x + R.est[0], ey = R.y + R.est[1], dx = k.x - ex, dy = k.y - ey, dist = Math.hypot(dx, dy), err = FS.wrap(k.yaw - R.yaw);
      let vx = 0, vy = 0, w = 0;
      if (Math.abs(err) > .3 && dist > .25) w = clamp(1.5 * err, -.6, .6);
      else { let sp = Math.min(p.cruise, Math.sqrt(2 * .5 * dist)); if (dist < .4) sp = Math.min(sp, Math.max(.03, dist)); vx = dist > 1e-3 ? dx / dist * sp : 0; vy = dist > 1e-3 ? dy / dist * sp : 0; w = clamp(2 * err, -.5, .5);
        const s = this.safety(R, vx * Math.cos(R.yaw) + vy * Math.sin(R.yaw), -vx * Math.sin(R.yaw) + vy * Math.cos(R.yaw)); if (s === 0) this.setState(R, 'safety_stop'); vx *= s; vy *= s; }
      const a = .6 * dt; R.vx += clamp(vx - R.vx, -a, a); R.vy += clamp(vy - R.vy, -a, a); R.w += clamp(w - R.w, -dt, dt);
      R.x += R.vx * dt; R.y += R.vy * dt; R.yaw = FS.wrap(R.yaw + R.w * dt);
      if (dist < k.tol && Math.abs(err) < .06) { R.task = null; R.vx = R.vy = R.w = 0; }
      return;
    }
    R.vx = R.vy = R.w = 0;
    if (k.k === 'plant') {
      this.setState(R, 'plant');
      if (R.timer < dt * 1.5 && (R.plugs < 1 || R.water < .5 || R.soc < .2)) { this.tenderTrip(R, k); R.task = null; return; }
      const c = k.s.stone && !k.stoneHit ? this.cycStone : this.cyc;
      R.boring = R.timer > c.phases[0][1] && R.timer < c.phases[0][1] + c.phases[1][1]; R.tool = R.timer < c.total ? c : null;
      if (R.timer >= (k.s.stone && !k.stoneHit ? this.cycStone.phases.slice(0, 3).reduce((a, q) => a + q[1], 0) : c.total)) {
        if (k.s.stone && !k.stoneHit) { k.stoneHit = true; this.ev('warn', `${R.name}: auger refused at 3 cm on an unsurveyed stone, shifting 0.3 m`);
          k.s.x += .3 * k.d; R.tasks.unshift({ k: 'goto', x: k.s.x + k.d * GANTRY, y: k.s.y, yaw: k.yaw, tol: .03, label: 'drive' }, { ...k, retry: 1, stoneHit: true }); k.s.stone = false; R.task = null; return; }
        const tx = R.x - Math.cos(R.yaw) * GANTRY, ty = R.y - Math.sin(R.yaw) * GANTRY, err = [tx - k.s.x, ty - k.s.y];
        k.s.state = 'done'; k.s.ax = tx; k.s.ay = ty; k.s.gnss = R.gnss; this.errs.push({ e: err, g: R.gnss }); if (this.errs.length > 3000) this.errs.shift();
        R.plugs--; R.water -= .5; R.planted++; this.planted++; this.newPlant = (this.newPlant || []).concat([k.s]); R.task = null;
      }
      return;
    }
    if (k.k === 'reserve') { this.setState(R, 'queue');
      if (R.timer < 3 && k.tried) return; k.tried = true; R.timer = 0;
      const free = this.bays.indexOf(null);
      if (this.bays.includes(R.i) || (free >= 0 && (!this.queue.length || this.queue[0] === R.i))) { const b = this.bays.includes(R.i) ? this.bays.indexOf(R.i) : free; this.bays[b] = R.i; this.queue = this.queue.filter(q => q !== R.i); R.slot = null;
        R.tasks.unshift({ k: 'goto', x: this.f.tender.x, y: this.f.tender.y + 4 * b, yaw: 0, tol: .3, label: 'to_tender' }); R.task = null; }
      else { if (!this.queue.includes(R.i)) { this.queue.push(R.i); this.ev('warn', `${R.name} waiting for the tender bay (queue ${this.queue.length})`); }
        // queue slots by arrival, in a column between the field and the bay
        if (R.slot == null) { const taken = new Set(this.robots.filter(o => o.slot != null).map(o => o.slot)); R.slot = 0; while (taken.has(R.slot)) R.slot++; }
        const wx = -9, wy = 4 - 4.5 * R.slot;
        if (Math.hypot(wx - R.x, wy - R.y) > .5) { R.tasks.unshift({ k: 'goto', x: wx, y: wy, yaw: 0, tol: .3, label: 'queue' }, { k: 'reserve' }); R.task = null; } }
      return;
    }
    if (k.k === 'service') { this.setState(R, 'service');
      if (!k.dur) { const need = this.p.plugs - R.plugs;
        if (need > 0 && this.stock < need) { if (this.restockAt == null) { this.restockAt = this.t + 2400; this.ev('bad', `tender out of drums (${this.stock} plugs): 40 min restock trip, ${R.name} waits`); } return; }
        k.need = need; k.dur = Math.max(need > 0 ? 120 : 0, (250 - R.water) / 2, R.soc < .6 ? (.95 - R.soc) * 30 / 11 * 3600 : 0) + 20; R.timer = 0; }
      if (R.timer >= k.dur) { this.stock -= k.need; R.plugs += k.need; R.water = 250; if (R.soc < .6) R.soc = .95; this.bays = this.bays.map(b => b === R.i ? null : b);
        this.ev('ok', `${R.name} serviced in ${k.dur.toFixed(0)} s: ${k.need} plugs, tender stock ${this.stock}`); R.task = null; }
      return;
    }
    if (k.k === 'park') { R.tasks = [{ k: 'goto', x: -6, y: 2 + 4.5 * R.i, yaw: 0, tol: .4, label: 'return' }]; R.task = null; R.parked = true;
      if (!R.parkedLogged) { R.parkedLogged = true; this.ev('ok', `${R.name} finished its block: ${R.planted} planted`); } }
  }
  tenderTrip(R, k) {
    const lane = R.y - this.f.dy / 2, hx = -3, resume = { k: 'goto', x: k.s.x + k.d * GANTRY, y: k.s.y, yaw: k.yaw, tol: .03, label: 'return' };
    this.ev('ok', `${R.name}: ${R.plugs < 1 ? 'drums empty' : R.water < .5 ? 'water out' : 'battery low'}, going to the tender`);
    R.tasks.unshift({ k: 'goto', x: R.x, y: lane, yaw: R.yaw, tol: .2, label: 'to_tender' }, { k: 'goto', x: hx, y: lane, yaw: Math.PI, tol: .3, label: 'to_tender' },
      { k: 'reserve' }, { k: 'service' }, { k: 'goto', x: hx, y: lane, yaw: 0, tol: .3, label: 'return' }, { k: 'goto', x: resume.x, y: lane, yaw: resume.yaw, tol: .2, label: 'return' }, resume, k);
  }
}

FS.SIMS.B = {
  key: 'B', name: 'Jaal', deva: 'जाल', gloss: 'net, grid', mode: 'Autonomous fleet · square grid',
  desc: 'Electric planters with four independently steered wheels plant a surveyed grid on their own and swap sapling drums at a tender truck. One supervisor watches the fleet.',
  speeds: [1, 10, 60, 240], defaultSpeed: 60, dt: .1,
  params: [
    { id: 'n', label: 'Fleet size', type: 'range', min: 1, max: 6, step: 1, value: 3, unit: 'robots' },
    { id: 'field', label: 'Block', type: 'opts', value: 'full', opts: [['full', '1.0 ha'], ['demo', '0.25 ha']] },
    { id: 'pattern', label: 'Pattern', type: 'opts', value: 'square', opts: [['square', 'Square'], ['triangular', 'Triangular']] },
    { id: 'spacing', label: 'Spacing', type: 'range', min: 2.5, max: 4, step: .5, value: 3, unit: 'm' },
    { id: 'plugs', label: 'Plugs per drum load', type: 'range', min: 60, max: 360, step: 30, value: 360, unit: 'plugs' },
    { id: 'stock', label: 'Tender stock', type: 'range', min: 0, max: 1800, step: 180, value: 720, unit: 'plugs', help: 'When it runs out the tender makes a 40-minute trip to the nursery.' },
    { id: 'bays', label: 'Service bays', type: 'range', min: 1, max: 2, step: 1, value: 1, unit: '' },
    { id: 'cruise', label: 'Cruise speed', type: 'range', min: .4, max: 1.5, step: .1, value: 1.0, unit: 'm/s' },
    { id: 'treeline', label: 'Tree line degrades RTK', type: 'opts', value: true, opts: [[true, 'Yes'], [false, 'No']] },
    { id: 'dropouts', label: 'Random RTK dropouts', type: 'range', min: 0, max: 12, step: 1, value: 2, unit: '/h' },
    { id: 'rtk', label: 'When RTK is lost', type: 'opts', value: 'continue', opts: [['continue', 'Continue on odometry'], ['wait', 'Wait for fix']] },
    { id: 'cows', label: 'Cattle in the block', type: 'range', min: 0, max: 3, step: 1, value: 2, unit: '' },
    { id: 'solar', label: 'Solar input', type: 'range', min: 0, max: 700, step: 50, value: 450, unit: 'W' },
    { id: 'soil', label: 'Soil', type: 'opts', value: 'sandy_loam', opts: [['sandy_loam', 'Sandy loam'], ['clayey_soil', 'Clayey'], ['dry_sand', 'Dry sand']] },
    { id: 'moist', label: 'Soil moisture', type: 'range', min: .05, max: .35, step: .01, value: .12, unit: 'm³/m³' },
  ],
  create(p) { return new JaalSim(p); },
  model: `
  <div class="eq"><h5>Coverage plan</h5>
    <div class="f">rows <var>y</var><sub>j</sub> = 1.5 + <var>j</var>·Δ<var>y</var>,  Δ<var>y</var> = 3 m (square) or 3·sin 60° (triangular)</div>
    <div class="f">split: cut where Σ spots ≈ <var>k N</var>/<var>n</var>, boustrophedon inside each block</div>
    <p>1,111 plants/ha square, 1,283/ha triangular. Spots on surveyed mounds and stumps shift ±0.3–0.6 m along the row or are skipped; stones are not in the drone survey and show up as an auger refusal, after which the robot shifts 0.3 m and retries.</p></div>
  <div class="eq"><h5>Holonomic stop control (fs::SwerveDrive)</h5>
    <div class="f"><var>v</var> = min(<var>v</var><sub>cruise</sub>, √(2<var>a d</var>)),  <var>v</var> → <var>k d</var> for <var>d</var> &lt; 0.4 m,  <var>ω</var> = 2 Δψ</div>
    <div class="f">stop where base = spot + 1.65 m · row direction (gantry over the spot, trailing)</div>
    <p>Each wheel module steers to v + ω × r<sub>i</sub>; pivots in place at the headland and crabs sideways onto the next row. Safety: anything in the travel corridor slows the robot from 5 m and stops it at 1.5 m.</p></div>
  <div class="eq"><h5>Localization quality (gnss_degrader + EKF)</h5>
    <div class="f">RTK fix σ = 0.02 m;  float: OU wander σ 0.35 m, τ 20 s;  dead reckoning: 1.2 % scale, gyro bias random walk</div>
    <p>The robot steers its estimate onto the target, so the true hole lands one estimation error away; the bullseye shows this split by GNSS state. Tree lines within 10 m of the north edge put the receiver in float.</p></div>
  <div class="eq"><h5>Energy and logistics</h5>
    <div class="f"><var>P</var> = 350 W + <var>C</var><sub>rr</sub><var>mgv</var>/0.85 + 1.5 kW·|<var>ω</var>| + <var>P</var><sub>auger</sub> − <var>P</var><sub>solar</sub>,  30 kWh LFP</div>
    <div class="f"><var>t</var><sub>service</sub> = max(120 s drum swap, water/2 L s⁻¹, charge at 11 kW) + 20 s docking</div>
    <p>Robots go to the tender when plugs, water (0.5 L per plant) or charge (20 %) run out, via the lane between rows so planted saplings are never driven over. One bay: robots queue in their own slots.</p></div>`,
};

class JaalSim {
  constructor(p) { this.p = p; this.F = new Fleet(p); this.events = this.F.events; this.sweep = null; }
  get done() { return this.F.done; }
  step(dt) { this.F.step(dt); }
  progress() { return (this.F.planted + this.F.skipped) / (this.F.total + this.F.skipped); }
  build3D() {
    const F = this.F, f = F.f, W = FS.T3.world, M = FS.MAT, B = FS.B, C = FS.C;
    FS.groundMesh(f.W + 70, f.H + 60, 60, 50, () => 0, (x, z) => FS.mixHex(0xBBA67B, 0x8E8E58, .45 + .55 * FS.noise2(x * .7 + 3, z * .7)), f.W / 2 - 8, -f.H / 2);
    f.obs.forEach((o, i) => { if (o.kind === 'stump') FS.add(W, C(o.r, o.r * 1.1, o.h, 10), M.trunk, o.x, o.h / 2, -o.y);
      else { const g = new THREE.SphereGeometry(1, 10, 8); const m = FS.add(W, g, o.kind === 'stone' ? M.rock : M.mound, o.x, 0, -o.y); m.scale.set(o.r, o.h, o.r * .85); m.rotation.y = i; } });
    for (let x = -10; x < f.W + 15; x += 7.5) FS.bigTree(x, -(f.H + 9 + (x * 13 % 4)), 1.1);
    const tr = new THREE.Group(); tr.position.set(f.tender.x - 4.5, 0, -(f.tender.y + 3.2)); W.add(tr);
    FS.add(tr, B(6, .3, 2), M.graphite, 0, .8, 0); FS.add(tr, B(1.8, 1.8, 2.1), M.cab, 2.3, 1.85, 0); FS.add(tr, B(.02, .8, 1.8), M.glass, 3.21, 2.25, 0);
    [2.2, -1, -2.2].forEach(x => [-1, 1].forEach(z => FS.wheel(tr, .5, .3, x, .5, z, M.graphite, 0)));
    FS.add(tr, B(3.9, .1, 2.1), M.graphite, -.6, 1.0, 0);
    this.tenderDrums = []; [-2.05, -1.1, -.15].forEach(x => [-.5, .5].forEach(z => { FS.add(tr, C(.44, .44, 1.5, 20), M.tray, x, 1.8, z); this.tenderDrums.push(FS.add(tr, C(.44, .44, .06, 20), M.leaf, x, 2.58, z)); }));
    const lab = FS.makeLabel('Tender truck', 'drum swap · water · charge'); lab.position.set(f.tender.x - 4.5, 4.6, -(f.tender.y + 3.2)); W.add(lab);
    this.spots = f.rows.flat(); this.rings = new FS.RingPool(this.spots, () => 0); this.pool = new FS.PlantPool(this.spots.length + 10);
    this.cowM = F.cows.map(() => { const g = new THREE.Group(); W.add(g); FS.add(g, B(1.6, .7, .6), M.cow, 0, 1.05, 0); FS.add(g, B(.5, .32, .3), M.cow, 1.0, 1.35, 0);
      [[.6, .2], [.6, -.2], [-.6, .2], [-.6, -.2]].forEach(([x, z]) => FS.add(g, B(.12, .7, .12), M.cow, x, .35, z)); return g; });
    this.ms = F.robots.map(() => { const m = FS.buildJaal(); W.add(m.root); return m; });
  }
  sync3D(dt) {
    const F = this.F; if (!this.ms) return;
    F.robots.forEach((R, i) => { const m = this.ms[i]; m.root.position.set(R.x, 0, -R.y); m.root.rotation.y = R.yaw;
      const v = Math.hypot(R.vx, R.vy); m.spins.forEach(w => w.spin.rotation.z -= v * dt / w.r);
      const steer = Math.abs(R.w) > .05 && v < .05 ? [Math.atan2(.95, .83), -Math.atan2(.95, .83), -Math.atan2(.95, .83), Math.atan2(.95, .83)] : [0, 0, 0, 0].map(() => Math.atan2(R.vy * Math.cos(R.yaw) - R.vx * Math.sin(R.yaw), R.vx * Math.cos(R.yaw) + R.vy * Math.sin(R.yaw) + 1e-6) * (v > .02 ? 1 : 0));
      m.steer.forEach((s, j) => s.rotation.y = FS.lerp(s.rotation.y, clamp(steer[j], -1.6, 1.6), .2));
      m.lidar.rotation.y += dt * 20; m.drums.forEach((d, j) => d.userData.fill.scale.y = clamp((R.plugs - j * F.p.plugs / 2) / (F.p.plugs / 2), .02, 1));
      const c = R.state === 'plant' && R.task && R.task.k === 'plant' ? (R.task.s.stone && !R.task.stoneHit ? F.cycStone : F.cyc) : null;
      if (c) { let acc = 0, d = 0; for (const [n, s] of c.phases) { const u = clamp((R.timer - acc) / s, 0, 1); if (n === 'lower') d = u * .58; else if (n === 'bore') d = .58 + u * .25; else if (n === 'retract') d = (.83) * (1 - u); acc += s; if (R.timer < acc) break; } m.setTool(d); if (d > 0) m.spinTool(dt); } else m.setTool(0);
      m.safety.visible = R.state === 'safety_stop'; });
    (F.newPlant || []).forEach(s => { this.pool.add(s.ax, s.ay, 0); this.rings.color(this.spots.indexOf(s), 'done'); }); F.newPlant = null;
    F.cows.forEach((c, i) => { this.cowM[i].position.set(c.x, 0, -c.y); this.cowM[i].rotation.y = c.yaw; });
    this.tenderDrums.forEach((d, i) => d.visible = F.stock > i * 180);
    this.pool.update(dt);
  }
  focus() { const R = this.F.robots[0]; return R; }
  camTarget() { const f = this.F.f; return [f.W * .42, f.H * .45, 0]; }
  camOffset() { const f = this.F.f; return [-f.W * .25, Math.max(f.W, f.H) * .55, f.H * 1.05]; }
  hud() { const F = this.F; return { title: `B · Jaal — ${F.p.n} robots, ${(F.f.W * F.f.H / 1e4).toFixed(2)} ha`, rows: [['Planted', `${F.planted} / ${F.total}`], ['Sim time', FS.fmtT(F.t)],
    ['Fleet rate', F.t > 300 ? `${FS.n(F.planted / F.t * 3600)}/h` : '—'], ['Tender stock', `${F.stock} plugs`], ['Queue', F.queue.length ? F.queue.map(i => 'jaal_' + (i + 1)).join(', ') : 'none']] }; }
  tele() {
    const F = this.F, mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN, e = F.errs.map(q => Math.hypot(...q.e) * 100);
    const p95 = e.length ? e.slice().sort((a, b) => a - b)[Math.floor(e.length * .95)] : NaN;
    const out = [['Fleet'], ['Planted · skipped', `${F.planted} · ${F.skipped}`], ['Fleet rate', F.t > 300 ? `${FS.n(F.planted / F.t * 3600)} /h` : '—'],
      ['Per robot', F.t > 300 ? `${FS.n(F.planted / F.t * 3600 / F.p.n)} /h` : '—'], ['Placement mean · p95', e.length ? `${FS.n(mean(e), 1)} · ${FS.n(p95, 1)} cm` : '—', p95 > 20 ? 'warn' : null],
      ['Tender'], ['Stock', `${F.stock} plugs`, F.stock < F.p.plugs ? 'warn' : 'ok'], ['Restock', F.restockAt != null ? `back in ${FS.fmtT(F.restockAt - F.t)}` : '—', F.restockAt != null ? 'bad' : null],
      ['Robots']];
    F.robots.forEach(R => out.push([R.name, `${R.state.replace('_', ' ')} · ${R.planted} · ${R.plugs} plugs · ${FS.n(R.soc * 100)} %`, R.state === 'safety_stop' || R.state === 'wait_fix' ? 'warn' : R.gnss !== 'rtk_fix' ? 'warn' : null]));
    return out;
  }
  figs() {
    return [
      { id: 'map', title: 'Field map', tag: 'live fleet state', wide: true, draw: (c, w, h) => this.drawMap(c, w, h),
        caption: 'Rings are planned spots (yellow), planted (green) or skipped (red). Brown: termite mounds and stumps from the drone survey; grey dots: stones nobody surveyed. The purple band is where the tree line puts RTK in float.' },
      { id: 'gantt', title: 'Fleet timeline', tag: 'state of every robot over time', draw: (c, w, h) => this.drawGantt(c, w, h),
        caption: 'Green is planting; the grey, purple and red bands are what keeps a robot from planting: travel, turns, the tender queue, service, safety stops.' },
      { id: 'bull', title: 'Placement error', tag: 'hole minus target, cm', draw: (c, w, h) => this.drawBull(c, w, h),
        caption: 'Each dot is one hole, coloured by GNSS state when it was bored. RTK holes cluster within 5 cm; float and dead-reckoning holes spread out.' },
      { id: 'prog', title: 'Progress and charge', tag: 'plants and battery over time', draw: (c, w, h) => this.drawProgress(c, w, h),
        caption: 'Flat steps in the planted line are tender waits. Battery lines step up when a robot is charged during service.' },
      { id: 'sweep', title: 'How many robots?', tag: 'same block, fleet of 1 to 6', draw: (c, w, h) => this.drawSweep(c, w, h), action: ['Run fleet sweep', () => this.runSweep()],
        caption: 'Runs the whole block headless for each fleet size with the current settings. Throughput stops scaling when the tender bay and drum stock become the bottleneck.' },
    ];
  }
  drawMap(c, w, h) {
    const F = this.F, f = F.f, x0 = f.tender.x - 10, x1 = f.W + 6, y0 = -8, y1 = f.H + 14, sc = Math.min((w - 20) / (x1 - x0), (h - 20) / (y1 - y0));
    const P = (x, y) => [10 + (x - x0) * sc, h - 10 - (y - y0) * sc];
    if (F.p.treeline) FS.poly(c, [P(-5, f.treeline), P(f.W + 5, f.treeline), P(f.W + 5, y1), P(-5, y1)], FS.alpha(TH.c4, .1));
    FS.poly(c, [P(0, 0), P(f.W, 0), P(f.W, f.H), P(0, f.H)], FS.alpha(TH.accent, .05), TH.accent, 1.2);
    for (let x = -10; x < f.W + 15; x += 7.5) { const q = P(x, f.H + 10); FS.circle(c, q[0], q[1], 2.2 * sc, FS.alpha(TH.leaf, .3)); }
    f.obs.forEach(o => { const q = P(o.x, o.y); FS.circle(c, q[0], q[1], Math.max(1.2, o.r * sc), o.kind === 'stone' ? FS.alpha(TH.muted, .6) : FS.alpha(TH.accent, .45)); });
    const rs = Math.max(1.4, .28 * sc);
    f.rows.flat().forEach(s => { const q = P(s.x, s.y); if (s.state === 'done') FS.circle(c, q[0], q[1], rs, TH.leaf); else FS.circle(c, q[0], q[1], rs, null, s.skip ? TH.bad : FS.alpha(TH.accent, .9), 1); });
    const tq = P(f.tender.x - 4.5, f.tender.y + 3.2); FS.poly(c, [[tq[0] - 3 * sc, tq[1] - sc], [tq[0] + 3 * sc, tq[1] - sc], [tq[0] + 3 * sc, tq[1] + sc], [tq[0] - 3 * sc, tq[1] + sc]], TH.muted);
    FS.text(c, `tender · ${F.stock}`, tq[0] - 3 * sc, tq[1] - sc - 5, { size: 10, color: TH.muted });
    F.cows.forEach(cw => { const q = P(cw.x, cw.y); FS.circle(c, q[0], q[1], Math.max(3, .8 * sc), TH.warn, TH.plot, 1); });
    const cols = [TH.water, TH.c4, TH.c5, TH.warn, TH.leaf, TH.accent];
    F.robots.forEach((R, i) => { const cs = Math.cos(R.yaw), sn = Math.sin(R.yaw), pts = [[1.4, .65], [1.4, -.65], [-1.4, -.65], [-1.4, .65]].map(([a, b]) => P(R.x + cs * a - sn * b, R.y + sn * a + cs * b));
      FS.poly(c, pts, cols[i % cols.length], TH.ink, 1); const g = P(R.x - cs * GANTRY, R.y - sn * GANTRY); FS.circle(c, g[0], g[1], 2, TH.ink);
      const q = P(R.x, R.y); FS.text(c, `${i + 1}`, q[0], q[1] + 4, { align: 'center', color: TH.plot, size: 10, weight: 500 });
      if (R.state === 'safety_stop') FS.circle(c, q[0], q[1], 4 * sc, null, TH.warn, 1.5); });
    FS.text(c, `${FS.fmtT(F.t)} · ${F.planted}/${F.total} planted`, w - 10, 16, { align: 'right', size: 11, halo: true });
  }
  drawGantt(c, w, h) {
    const F = this.F, n = F.robots.length, t1 = Math.max(F.t, 60);
    const col = { drive: TH.muted, turn: FS.alpha(TH.muted, .5), plant: TH.leaf, detour: TH.c5, to_tender: TH.c4, return: FS.alpha(TH.c4, .55), queue: TH.bad, service: TH.water, safety_stop: TH.warn, wait_fix: TH.accent, done: FS.alpha(TH.line, 1), idle: TH.line };
    const ly = FS.swatches(c, ['plant', 'drive', 'detour', 'to_tender', 'queue', 'service', 'safety_stop', 'wait_fix'].map(k => [k.replace('_', ' '), col[k]]), 56, 14, w - 70);
    const box = { x: 56, y: ly + 10, w: w - 70, h: h - ly - 44 };
    const { X } = FS.frame(c, box, [0, t1 / 60], [0, 1], { yn: 1, yfmt: () => '', xlabel: 'min' });
    const rh = box.h / n;
    F.robots.forEach((R, i) => FS.text(c, R.name, box.x - 6, box.y + rh * (i + .5) + 3.5, { align: 'right', color: TH.muted, size: 10 }));
    F.timeline.forEach(e => { const y = box.y + rh * e.r + 2, a = X(e.t0 / 60), b = X((e.t1 ?? F.t) / 60); if (b - a >= .3) FS.poly(c, [[a, y], [b, y], [b, y + rh - 4], [a, y + rh - 4]], col[e.s] || TH.line); });

  }
  drawBull(c, w, h) {
    const F = this.F, cx = w / 2, cy = h / 2 + 6, R = Math.min(w, h) / 2 - 22, lim = 40, sc = R / lim;
    [5, 10, 20, 40].forEach(r => { FS.circle(c, cx, cy, r * sc, null, TH.line, 1); FS.text(c, `${r} cm`, cx + r * sc * .71 + 3, cy - r * sc * .71, { size: 10, color: TH.muted }); });
    FS.line(c, [[cx - R, cy], [cx + R, cy]], TH.line, 1); FS.line(c, [[cx, cy - R], [cx, cy + R]], TH.line, 1);
    const col = { rtk_fix: TH.leaf, float: TH.c4, dead_reckoning: TH.bad };
    F.errs.forEach(q => { const x = clamp(q.e[0] * 100, -lim, lim), y = clamp(q.e[1] * 100, -lim, lim); FS.circle(c, cx + x * sc, cy - y * sc, 2.2, FS.alpha(col[q.g], .75)); });
    FS.legend(c, [['RTK fix', TH.leaf], ['float', TH.c4], ['dead reckoning', TH.bad]], 10, 14);
  }
  drawProgress(c, w, h) {
    const F = this.F, s = F.series; if (s.t.length < 2) { FS.text(c, 'collecting…', 50, 40, { color: TH.muted }); return; }
    const box = { x: 44, y: 24, w: w - 90, h: h - 56 }, { X, Y } = FS.frame(c, box, [0, Math.max(s.t[s.t.length - 1] / 60, 1)], [0, F.total], { xlabel: 'min', ylabel: 'planted' });
    FS.series(c, X, Y, s.t.map((t, i) => [t / 60, s.planted[i]]), TH.leaf, { area: true });
    const Y2 = v => box.y + box.h - v * box.h, cols = [TH.water, TH.c4, TH.c5, TH.warn, TH.leaf, TH.accent];
    F.robots.forEach((R, i) => FS.series(c, X, Y2, s.t.map((t, k) => [t / 60, s.soc[k][i]]), cols[i % cols.length], { w: 1.2, dot: false, dash: [3, 3] }));
    [0, 50, 100].forEach(v => FS.text(c, `${v}%`, box.x + box.w + 6, Y2(v / 100) + 3.5, { size: 10, color: TH.muted }));
    FS.legend(c, [['planted', TH.leaf], ['battery (each robot)', TH.water, [3, 3]]], box.x + 6, box.y + 12);
  }
  runSweep() {
    if (this.sweep && this.sweep.running) return; const res = []; this.sweep = { running: true, res, n: 1 };
    const next = () => {
      const n = this.sweep.n; if (n > 6) { this.sweep.running = false; return; }
      const F = new Fleet({ ...this.p, n, cows: 0 }, true); let guard = 0; const t0 = performance.now();
      while (!F.done && guard++ < 400000 && performance.now() - t0 < 4000) F.step(.5);
      res.push({ n, t: F.t, rate: F.planted / F.t * 3600, util: F.robots.map(R => F.timeline.filter(e => e.r === R.i && e.s === 'plant').reduce((a, e) => a + ((e.t1 ?? F.t) - e.t0), 0) / F.t) });
      this.sweep.n++; setTimeout(next, 30);
    }; setTimeout(next, 30);
  }
  drawSweep(c, w, h) {
    const sw = this.sweep; if (!sw) { FS.text(c, 'Press “Run fleet sweep” to simulate this block with 1 to 6 robots.', 20, h / 2, { color: TH.muted, size: 12, font: 'body' }); return; }
    const box = { x: 54, y: 26, w: w - 120, h: h - 60 }, res = sw.res, tmax = Math.max(...res.map(r => r.t / 3600), 1);
    const { X, Y } = FS.frame(c, box, [.5, 6.5], [0, tmax * 1.1], { xlabel: 'robots', ylabel: 'hours to finish', xn: 6 });
    res.forEach(r => { const a = X(r.n - .3), b = X(r.n + .3); FS.poly(c, [[a, Y(r.t / 3600)], [b, Y(r.t / 3600)], [b, Y(0)], [a, Y(0)]], FS.alpha(TH.water, .5), TH.water); FS.text(c, FS.n(r.t / 3600, 1) + ' h', X(r.n), Y(r.t / 3600) - 5, { align: 'center', size: 10 }); });
    const rmax = Math.max(...res.map(r => r.rate), 1) * 1.15, Y2 = v => box.y + box.h - v / rmax * box.h;
    FS.series(c, X, Y2, res.map(r => [r.n, r.rate]), TH.leaf, { w: 2 });
    res.forEach(r => FS.circle(c, X(r.n), Y2(r.rate), 3.5, TH.leaf, TH.plot, 1.5));
    FS.niceTicks(0, rmax, 4).t.forEach(v => FS.text(c, FS.n(v), box.x + box.w + 6, Y2(v) + 3.5, { size: 10, color: TH.leaf }));
    FS.text(c, 'plants/h', box.x + box.w + 6, box.y - 8, { size: 10.5, color: TH.leaf });
    if (sw.running) FS.text(c, `simulating ${sw.n} robot${sw.n > 1 ? 's' : ''}…`, box.x + box.w, box.y + 12, { align: 'right', color: TH.accent, size: 11 });
    if (!sw.running && res.length) { const solo = res[0].rate, keep = res.filter(r => r.rate / r.n >= .85 * solo).pop();
      FS.text(c, `each added robot keeps ≥ 85 % of a solo robot's rate up to ${keep.n} robots`, box.x + 6, box.y + 12, { color: TH.muted, size: 10.5 }); }
  }
}
})();
