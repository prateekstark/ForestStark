/* ForestStark Sim Lab — page shell: tabs, scenario controls, simulation loop, 3D bay, diagrams. */
(() => {
const $ = (s, r = document) => r.querySelector(s);
const THREE_SOURCES = ['https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js', 'https://unpkg.com/three@0.128.0/build/three.min.js', 'https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js'];
const loadThree = i => new Promise(res => {
  if (window.THREE || i >= THREE_SOURCES.length) return res();
  const s = document.createElement('script'); let done = false; const next = () => { if (done) return; done = true; s.remove(); loadThree(i + 1).then(res); };
  const timer = setTimeout(next, 9000); s.src = THREE_SOURCES[i];
  s.onload = () => { clearTimeout(timer); if (done) return; done = true; window.THREE ? res() : loadThree(i + 1).then(res); }; s.onerror = () => { clearTimeout(timer); next(); };
  document.head.appendChild(s);
});

const HASH = { katar: 'A', jaal: 'B', pahadi: 'C' }, KEYS = { A: 'katar', B: 'jaal', C: 'pahadi' };
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
const state = { k: 'A', def: null, sim: null, params: {}, playing: !reduceMotion, speed: 12, follow: true, figs: [], flat: null, lastEvents: '', camInit: false };

function boot() {
  FS.readTheme();
  const host = $('#viewport'), msg = $('#vp-msg');
  const has3D = FS.init3D(host);
  if (msg) msg.remove();
  if (!has3D) {
    const cv = document.createElement('canvas'); cv.className = 'flat'; host.insertBefore(cv, host.firstChild); state.flat = cv;
    $('#hint').textContent = window.THREE ? 'WebGL is off here: showing the live plan view' : '3D library unavailable: showing the live plan view';
  }
  const applySky = () => { FS.readTheme(); if (FS.T3.ok) { const c = new THREE.Color(FS.TH.sky || '#D6E2DF'); FS.T3.scene.background = c; FS.T3.scene.fog = new THREE.Fog(c, 70, 240); } drawFigs(true); };
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applySky);
  new MutationObserver(applySky).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  applySky();
  if (FS.T3.ok) new ResizeObserver(resize).observe(host);
  resize();

  document.querySelectorAll('.tab').forEach(t => t.addEventListener('click', () => select(t.dataset.k)));
  const tabs = [...document.querySelectorAll('.tab')];
  tabs.forEach((t, i) => t.addEventListener('keydown', e => { if (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft') return;
    const n = tabs[(i + (e.key === 'ArrowRight' ? 1 : tabs.length - 1)) % tabs.length]; n.focus(); select(n.dataset.k); }));
  $('#play').addEventListener('click', () => { state.playing = !state.playing; syncPlay(); });
  $('#follow').addEventListener('click', e => { state.follow = !state.follow; e.currentTarget.setAttribute('aria-pressed', String(state.follow)); });
  $('#restart').addEventListener('click', () => restart());
  syncPlay();
  addEventListener('hashchange', () => { const k = HASH[(location.hash || '').slice(1)]; if (k && k !== state.k) select(k); });
  select(HASH[(location.hash || '').slice(1)] || 'A');
  requestAnimationFrame(frame);
}

function syncPlay() { $('#play').textContent = state.playing ? 'Pause' : 'Play'; $('#play').setAttribute('aria-pressed', String(!state.playing)); }

function resize() {
  const host = $('#viewport'), w = host.clientWidth, h = host.clientHeight; if (!w || !h) return;
  if (FS.T3.ok) { FS.T3.renderer.setSize(w, h, false); FS.T3.camera.aspect = w / h; FS.T3.camera.updateProjectionMatrix(); }
}

function select(k) {
  state.k = k; state.def = FS.SIMS[k];
  document.querySelectorAll('.tab').forEach(t => t.setAttribute('aria-selected', String(t.dataset.k === k)));
  try { history.replaceState(null, '', '#' + KEYS[k]); } catch (e) { /* hash is optional */ }
  const d = state.def;
  state.params = Object.fromEntries(d.params.map(p => [p.id, p.value]));
  state.speed = d.defaultSpeed;
  $('#robot-card').innerHTML = `<div class="robot-head"><div class="big-k">${k}</div><div><h2>${d.name}<span class="deva" lang="hi">${d.deva}</span></h2>
    <div class="mode">${FS.esc(d.mode)} · “${FS.esc(d.gloss)}”</div></div></div><p class="robot-desc">${FS.esc(d.desc)}</p>`;
  renderParams(); renderSpeeds();
  $('#model').innerHTML = d.model;
  restart();
}

function renderSpeeds() {
  const seg = $('#speeds'); seg.innerHTML = state.def.speeds.map(s => `<button class="btn" data-speed="${s}" aria-pressed="${s === state.speed}">${s}×</button>`).join('');
  seg.querySelectorAll('button').forEach(b => b.addEventListener('click', () => { state.speed = +b.dataset.speed; seg.querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', String(x === b))); }));
}

function renderParams() {
  const box = $('#params'), d = state.def;
  box.innerHTML = d.params.map(p => {
    const id = `p-${d.key}-${p.id}`, help = p.help ? `<div class="help">${FS.esc(p.help)}</div>` : '';
    if (p.type === 'range') return `<div class="prm"><label for="${id}"><span>${FS.esc(p.label)}</span><output id="${id}-o">${fmtVal(p, state.params[p.id])}</output></label>
      <input type="range" id="${id}" min="${p.min}" max="${p.max}" step="${p.step}" value="${state.params[p.id]}">${help}</div>`;
    return `<div class="prm"><div class="lbl"><span>${FS.esc(p.label)}</span></div><div class="opts" role="group" aria-label="${FS.esc(p.label)}">${p.opts.map(([v, l]) =>
      `<button type="button" data-p="${p.id}" data-v="${String(v)}" aria-pressed="${String(v) === String(state.params[p.id])}">${FS.esc(l)}</button>`).join('')}</div>${help}</div>`;
  }).join('');
  d.params.forEach(p => {
    if (p.type !== 'range') return; const el = $(`#p-${d.key}-${p.id}`), out = $(`#p-${d.key}-${p.id}-o`);
    el.addEventListener('input', () => { out.textContent = fmtVal(p, +el.value); });
    el.addEventListener('change', () => { state.params[p.id] = +el.value; restart(); });
  });
  box.querySelectorAll('.opts button').forEach(b => b.addEventListener('click', () => {
    const p = d.params.find(q => q.id === b.dataset.p), raw = b.dataset.v, v = typeof p.value === 'boolean' ? raw === 'true' : typeof p.value === 'number' ? +raw : raw;
    state.params[p.id] = v; b.parentElement.querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', String(x === b))); restart();
  }));
}
function fmtVal(p, v) { const d = p.step < .1 ? 2 : p.step < 1 ? 1 : 0; return `${FS.n(+v, d)}${p.unit ? ' ' + p.unit : ''}`; }

function restart() {
  const d = state.def;
  state.sim = d.create({ ...state.params });
  state.lastEvents = ''; state.camInit = false;
  if (FS.T3.ok) {
    const W = FS.T3.world;
    W.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material && o.material.map) o.material.map.dispose(); });
    FS.T3.scene.remove(W); FS.T3.world = new THREE.Group(); FS.T3.scene.add(FS.T3.world);
    state.sim.build3D();
  }
  buildFigs(); drawFigs(true); updatePanels(true);
}

function buildFigs() {
  const figs = state.sim.figs(), box = $('#figs');
  box.innerHTML = figs.map(f => `<figure class="fig${f.wide ? ' wide' : ''}" id="fig-${f.id}" style="margin:0"><header><h4>${FS.esc(f.title)}</h4><span class="tag">${FS.esc(f.tag)}</span></header>
    <canvas role="img" aria-label="${FS.esc(f.title)}"></canvas>${f.action ? `<div class="row"><button class="btn primary" type="button">${FS.esc(f.action[0])}</button></div>` : ''}<p>${FS.esc(f.caption)}</p></figure>`).join('');
  state.figs = figs.map(f => ({ ...f, cv: $(`#fig-${f.id} canvas`) }));
  figs.forEach(f => { if (f.action) $(`#fig-${f.id} button`).addEventListener('click', f.action[1]); });
}
let lastFigs = 0;
function drawFigs(force) {
  const now = performance.now(); if (!force && now - lastFigs < 180) return; lastFigs = now;
  state.figs.forEach(f => { const r = f.cv.getBoundingClientRect(); if (!force && (r.bottom < 0 || r.top > innerHeight)) return; const { c, w, h } = FS.fit(f.cv); try { f.draw(c, w, h); } catch (e) { console.error(e); } });
  if (state.flat) { const { c, w, h } = FS.fit(state.flat); const f = state.figs[0]; if (f) f.draw(c, w, h); }
}

let lastPanel = 0, lastHud = 0;
function updatePanels(force) {
  const sim = state.sim, now = performance.now();
  if (force || now - lastHud > 120) { lastHud = now;
    const hd = sim.hud(); $('#hud').innerHTML = `<div class="ttl">${FS.esc(hd.title)}</div><dl>${hd.rows.map(([a, b]) => `<dt>${FS.esc(a)}</dt><dd>${FS.esc(b)}</dd>`).join('')}</dl>${state.playing ? '' : '<div style="margin-top:3px;color:var(--muted)">Paused</div>'}`;
    $('#prog').style.width = `${Math.round(Math.min(1, sim.progress()) * 100)}%`;
    const tt = sim.t !== undefined ? sim.t : sim.F.t; $('#simclock').textContent = 'sim ' + FS.fmtT(tt);
  }
  if (force || now - lastPanel > 300) { lastPanel = now;
    $('#tele').innerHTML = sim.tele().map(r => r.length === 1 ? `<div class="grp">${FS.esc(r[0])}</div>` :
      `<dt>${FS.esc(r[0])}</dt><dd>${FS.esc(r[1])}${r[2] ? `<span class="chip ${r[2]}">${r[2] === 'ok' ? 'ok' : r[2] === 'warn' ? 'watch' : 'alert'}</span>` : ''}</dd>`).join('');
    const ev = sim.events, key = ev.length + ':' + (ev[0] ? ev[0].t : 0);
    if (force || key !== state.lastEvents) { state.lastEvents = key;
      $('#log').innerHTML = ev.length ? ev.slice(0, 40).map(e => `<li><time>${FS.fmtT(e.t)}</time><span class="${e.level}">${FS.esc(e.text)}</span></li>`).join('') : '<li class="empty">Nothing yet.</li>';
      $('#evcount').textContent = ev.length ? `${ev.length} events` : ''; }
    const b = $('#banner'), last = ev[0];
    if (last && last.level === 'bad' && (sim.t ?? sim.F.t) - last.t < 25) { b.hidden = false; b.className = 'banner bad'; b.textContent = last.text; } else b.hidden = true;
  }
}

let last = performance.now();
const tmpV = () => new THREE.Vector3();
function frame(now) {
  const dtR = Math.min(.05, (now - last) / 1000); last = now; const sim = state.sim, d = state.def;
  if (sim && state.playing && !sim.done) {
    let rem = dtR * state.speed; const t0 = performance.now();
    while (rem > 1e-9 && performance.now() - t0 < 22) { const h = Math.min(rem, d.dt); sim.step(h); rem -= h; }
  }
  if (sim && FS.T3.ok) {
    sim.sync3D(dtR * (state.playing ? Math.min(state.speed, 4) : 1));
    const T = FS.T3, tg = sim.camTarget(), want = new THREE.Vector3(tg[0], tg[2], -tg[1]);
    if (!state.camInit) { const o = sim.camOffset(); T.controls.target.copy(want); T.camera.position.copy(want).add(new THREE.Vector3(o[0], o[1], o[2])); state.camInit = true; }
    else if (state.follow && performance.now() - T.controls.user > 1500) { const dv = tmpV().copy(want).sub(T.controls.target).multiplyScalar(1 - Math.exp(-dtR * 3)); T.controls.target.add(dv); T.camera.position.add(dv); }
    T.sun.position.copy(T.controls.target).add(new THREE.Vector3(9, 15, 7)); T.sun.target.position.copy(T.controls.target);
    T.controls.update(); T.renderer.render(T.scene, T.camera);
  }
  if (sim) { drawFigs(false); updatePanels(false); }
  requestAnimationFrame(frame);
}

loadThree(0).then(boot);
})();
