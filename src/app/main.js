// 光点法边界追踪 — 网页版主程序.
// 流程: ① 光路 → ② 晶体取向 → ③ 太阳高度 (任一改动自动重算) → 图 (天空 / 取向平面 / 3D 抬升) → 右栏解读.
// 计算在 Worker 里 (每条光路: 天球 + 取向平面两个 Worker 并行), 分段回传: 弧 → 取向平面 → 末端点 → 连线.

import { EnginePool } from './pool.js';
import { Renderer, PLOT_THEMES, lensNames, fovMax, oriArea, css, lensTag, skyColorIndex } from './render.js';
import { CrystalPreview } from './crystal.js';
import { parsePaths, modeParams } from '../engine/formlogic.js';
import { apexDegFromMiller, millerFromApexDeg, setSun, axesMeta, isPyramidFace, LP, SqK } from '../engine/lp.js';
import { apSupports } from '../engine/aperture.js';
import { term, famInfo, skyName, epLabel, EP_KINDS } from './terms.js';
import { PRESETS } from './presets.js';
import { initGlossary } from './glossary.js';
import { View3D, KIND3D } from './view3d.js';
import { SphereView } from './sphere3d.js';
import { t as tr, L, LANGS, setLang, detectLang, applyI18n } from './i18n.js';

/* global WORKER_SRC */

const $ = id => document.getElementById(id);
const el = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text !== undefined) e.textContent = text; return e; };
const STORE_KEY = 'lightpoint-trace/v2';

const DEFAULTS = {
  view: 0, path: '3-1-5', sun: 10, partial: false, mode: 0, fixed: 90,
  apexMode: 0, apexUp: 56.0, apexLo: 56.0, upA: 1, upB: 1, loA: 1, loB: 1,
  real: false, dist: [1, 1, 1, 1, 1, 1], h: 1,
  lens: 2, az: 0, el: 10, fov: 180,
  xw: 360, yw: 360, xc: 180, yc: 180,
  arcs: true, mc: true, lines: true, endpoints: true, sym: false,
  lineW: 2, mcSize: 1, checked: [], introDone: false, tab: 'setup', moreOpen: false,
  kz: 1.6, liftRef: 'auto', drops: true, lang: '', zenHalf: false, hlFull: true, liftBase: 'flat',
};

const S = (() => {
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(STORE_KEY) || '{}') || {}; } catch (e) { saved = {}; }
  const s = { ...DEFAULTS, ...saved };
  if (!Array.isArray(s.dist) || s.dist.length !== 6) s.dist = DEFAULTS.dist.slice();
  s.checked = new Set(Array.isArray(s.checked) ? s.checked : []);
  s.probe = null; s.hoverEp = null;
  return s;
})();

let saveTimer = 0;
function save() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    try {
      const o = { ...S, checked: [...S.checked] };
      delete o.probe; delete o.hoverEp;
      localStorage.setItem(STORE_KEY, JSON.stringify(o));
    } catch (e) { /* 隐私模式等: 不记忆即可 */ }
  }, 250);
}

// 取向模式: 固定角的符号与范围; 名称、说明、标签在语言文件 modes[i] 里
const MODE_META = [{ sym: 'ζ', min: 0, max: 180 }, { sym: '', min: 0, max: 180 }, { sym: 'roll', min: 0, max: 360 }, { sym: 'roll', min: 0, max: 360 }];
const modeInfo = i => ({ ...MODE_META[i], ...L().modes[i] });

// ============================================================ 引擎 / 渲染
const pool = new EnginePool(WORKER_SRC);
const renderer = new Renderer($('cv'));
const view3d = new View3D($('cv3'), $('cv3o'));
const sphere = new SphereView($('cv4'), $('cv4o'));     // 视图 3: 从球外旁观天球
sphere.onHover = h => readout3d(h);
view3d.kz = S.kz;
view3d.base = S.liftBase === 'sphere' ? 'sphere' : 'flat';
view3d.onHover = h => readout3d(h);
S.lang = setLang(detectLang(S.lang));          // 记住的语言, 否则按浏览器语言
const glossary = initGlossary();
const crystal = new CrystalPreview($('cvCrystal'), f => appendFace(f));
let R = null, Rnext = null;
LP.zenHalf = S.mode === 3 && S.zenHalf;
let lastAxes = modeParams(S.mode, S.fixed).om + '|' + LP.zenHalf;   // 取向平面的轴 (模式 + ζ 范围) 变了就回到全图
let dragging = false;

const theme = () => PLOT_THEMES.paper;           // 白底 + matplotlib 配色

// ============================================================ 输入: 光路
const FACE_KIND = f => (f === 1 ? tr('face.top') : f === 2 ? tr('face.bottom') : f >= 3 && f <= 8 ? tr('face.prism')
  : f >= 13 && f <= 18 ? tr('face.upper') : f >= 23 && f <= 28 ? tr('face.lower') : null);

function renderChain() {
  const box = $('chain'), msg = $('pathMsg');
  box.textContent = ''; msg.textContent = '';
  const text = $('inPath').value.trim();
  if (!text) { msg.textContent = tr('path.empty'); msg.style.color = 'var(--ink-3)'; return; }
  msg.style.color = '';
  const groups = text.split(/[,;，；\s]+/).filter(Boolean);
  const multi = groups.length > 1;
  const problems = [];
  groups.forEach((g, gi) => {
    const row = el('div', 'chain-row');
    if (multi) { const d = el('i', 'pathdot'); d.style.background = css(theme().pathcol[gi % 6]); row.append(d); }
    const toks = g.split('-').filter(t => t !== '');
    toks.forEach((t, i) => {
      const f = /^\d+$/.test(t) ? +t : NaN, kind = FACE_KIND(f);
      const role = toks.length === 1 ? 'in' : i === 0 ? 'in' : i === toks.length - 1 ? 'out' : 'mid';
      const chip = el('span', 'face ' + role);
      chip.append(el('b', '', t), el('small', '', kind ? tr('role.' + role) : tr('face.none')));
      if (!kind) { chip.style.setProperty('--c', 'var(--bad)'); problems.push(tr('path.errNoFace', { t })); }
      chip.title = kind ? `${t}: ${kind}` : '';
      if (i > 0) row.append(el('span', 'arrow', '→'));
      row.append(chip);
    });
    if (toks.length === 1) problems.push(tr('path.errShort', { g }));
    box.append(row);
  });
  if (groups.length > 6) problems.push(tr('path.errMax'));
  if (problems.length) msg.textContent = problems[0] + tr('path.errHelp');
}

let editTimer = 0;
function setPathText(t, delay) {
  $('inPath').value = t;
  renderChain();
  clearTimeout(editTimer);
  if (delay !== undefined) editTimer = setTimeout(compute, delay);
}
function appendFace(f) {
  let t = $('inPath').value.trim();
  t = !t || /[,，;；]\s*$/.test(t) ? t + f : t + '-' + f;
  setPathText(t, 550);
}

// ============================================================ 计算
function readApex() {
  const miller = S.apexMode === 1;
  const one = (deg, a, b, info, fallback) => {
    let d = fallback;
    if (miller) {
      if (Number.isInteger(a) && Number.isInteger(b) && a > 0 && b > 0) d = apexDegFromMiller(a, b);
      info.textContent = `= ${d.toFixed(2)}°`;
    } else {
      if (Number.isFinite(deg) && deg > 0 && deg < 180) d = deg;
      const [ma, mb] = millerFromApexDeg(d);
      info.textContent = `≈ (${ma} 0 −${ma} ${mb})`;
    }
    return d;
  };
  S.apexUpEff = one(S.apexUp, S.upA, S.upB, $('lbUpInfo'), S.apexUpEff ?? 56.0);
  S.apexLoEff = one(S.apexLo, S.loA, S.loB, $('lbLoInfo'), S.apexLoEff ?? 56.0);
}

function readReal() {
  const d = S.dist.map(v => (Number.isFinite(v) && v > 0.05 && v <= 5 ? v : 1));
  const h = Number.isFinite(S.h) && S.h > 0.005 && S.h <= 20 ? S.h : 1;
  document.querySelectorAll('[data-d]').forEach((e, i) => e.classList.toggle('invalid', d[i] !== S.dist[i]));
  $('inH').classList.toggle('invalid', h !== S.h);
  return { d, h };
}

function realTag(on, d, h) {
  if (!on) return '';
  const reg = d.every(x => Math.abs(x - 1) <= 1e-9);
  const f = x => String(Math.round(x * 100) / 100);
  return tr('export.real', { h: Math.round(h * 1000) / 1000, d: reg ? 'd=1' : 'd=[' + d.map(f).join(',') + ']' });
}

function syncMore(paths) {
  const flat = (paths || []).flat();
  const pyr = flat.some(isPyramidFace);
  $('optApex').classList.toggle('hot', pyr);
  $('optApex').classList.toggle('idle', !pyr);
  $('apexOff').hidden = pyr;
  $('optReal').classList.toggle('off', !S.real);
  $('apexGrid').classList.toggle('miller', S.apexMode === 1);
  const tags = $('moreTags');
  tags.textContent = '';
  if (pyr) tags.append(el('span', '', tr('tag.apex', { deg: S.apexUpEff.toFixed(1) })));
  if (S.real) tags.append(el('span', '', tr('tag.real')));
  if (S.real && paths && paths.length && !paths.some(apSupports))
    tags.append(el('span', '', tr('tag.realNoEffect')));
}

function compute() {
  clearTimeout(editTimer);
  renderChain();
  const paths = parsePaths($('inPath').value);
  const sun = parseFloat($('inSun').value);
  const sunOk = Number.isFinite(sun) && sun >= 0 && sun < 90;
  $('inPath').classList.toggle('invalid', !paths && $('inPath').value.trim() !== '');
  $('inSun').closest('.numbox').classList.toggle('invalid', !sunOk);
  if (!paths || !sunOk) {
    setStatus('err', !paths ? 'status.badPath' : 'status.badSun');
    if (!sunOk) $('pathMsg').textContent = $('pathMsg').textContent || tr('path.errSun');
    return;
  }
  S.path = $('inPath').value.trim(); S.sun = sun;
  readApex();
  const { d, h } = readReal();
  syncMore(paths);
  const { om, fx } = modeParams(S.mode, S.fixed);
  setSun(sun);
  const zenHalf = S.mode === 3 && S.zenHalf;
  LP.zenHalf = zenHalf;                          // 主线程里也要知道取向平面的 ζ 范围 (画轴、回到全图)
  const cfg = { sun, partial: S.partial, apexUp: S.apexUpEff, apexLo: S.apexLoEff, aperture: { on: S.real, dist: d, h }, zenHalf };
  if (om + '|' + zenHalf !== lastAxes) { lastAxes = om + '|' + zenHalf; resetOriView(); }
  const twoDof = S.mode === 0 || S.mode === 3;
  const tasks = [], prs = [];
  paths.forEach((faces, i) => {
    const base = { faces, cfg, modeIdx: S.mode, fixedDeg: S.fixed, idx: i };
    const pr = { idx: i, name: faces.join('-'), faces, parts: [], doneParts: new Set(), oriSlot: -1, skySlot: tasks.length, base };
    tasks.push({ ...base, part: 'sky' }); pr.parts.push('sky');
    if (twoDof) { pr.oriSlot = tasks.length; tasks.push({ ...base, part: 'ori' }); pr.parts.push('ori'); }
    prs.push(pr);
  });
  const job = pool.compute(tasks);
  Rnext = {
    job, t0: performance.now(),
    params: { sun, partial: S.partial, mode: S.mode, fixed: S.fixed, om, fx, twoDof, real: S.real, h, realTag: realTag(S.real, d, h), zenHalf },
    paths: prs, oriLocal: null, clouds: new Map(), requestCloud,
  };
  S.probe = null; S.hoverEp = null; $('probeCard').hidden = true;
  setStatus('busy', 'status.busy');
  $('progress').classList.add('on');
  crystal.set(d, h, paths[0]);
  renderCrumbs(paths);
  save();
}

function requestCloud(pr, fm) {
  const T = R, key = pr.name + '|' + fm.name;
  T.clouds.set(key, 'pending');
  pool.request(pr.oriSlot, { type: 'cloud', fam: fm.name }).then(data => {
    T.clouds.set(key, data || new Float32Array(0));
    if (T === R) requestRender();
  });
}

pool.onMessage = m => {
  const T = Rnext && Rnext.job === m.job ? Rnext : (R && R.job === m.job ? R : null);
  if (!T) return;
  const pr = T.paths[m.idx];
  if (!pr) return;
  switch (m.type) {
    case 'sky': Object.assign(pr, { sky: m.sky, mc: m.mc, traceMs: m.ms }); break;
    case 'ori': Object.assign(pr, { ori: m.ori, oriMs: m.ms }); break;
    case 'eps': pr.eps = m.eps; pr.epsMs = m.ms; break;
    case 'lines': pr.lines = new Map(m.lines.map(l => [l.name, l])); pr.linesMs = m.ms; break;
    case 'done':
      pr.doneParts.add(m.part);
      pr.done = pr.parts.every(p => pr.doneParts.has(p));
      if (pr.done) pr.totalMs = performance.now() - T.t0;
      break;
    case 'error': pr.error = (pr.error ? pr.error + '\n' : '') + m.message; break;
  }
  if (T === Rnext) {
    if (m.type !== 'sky' && m.type !== 'ori' && !T.paths.every(p => p.done)) return;
    R = Rnext; Rnext = null;                    // 首个结果到达才换下旧图, 免得画面闪白
    if (S.tab === 'setup') $('tabBadge').classList.add('on');
  }
  if (T.paths.every(p => p.done)) {
    const wall = (performance.now() - T.t0) / 1000;
    const bad = T.paths.filter(p => p.error);
    setStatus(bad.length ? 'err' : 'ok', bad.length ? 'status.doneErr' : 'status.done', { n: bad.length, s: wall.toFixed(wall < 10 ? 1 : 0) });
    $('progress').classList.remove('on');
  }
  requestRender();
  renderRead();
};

let lastStatus = ['ok', 'status.ready', null];
/** 状态栏: 存下键和变量, 换语言时照样重写. */
function setStatus(kind, key, vars) {
  lastStatus = [kind, key, vars];
  const text = tr(key, vars);
  const s = $('status');
  s.classList.toggle('busy', kind === 'busy');
  s.classList.toggle('err', kind === 'err');
  $('statusText').textContent = text;
  s.title = text;
  if (kind !== 'busy') $('progress').classList.remove('on');
}

function renderCrumbs(paths) {
  const c = $('crumbs');
  c.textContent = '';
  const mi = modeInfo(S.mode);
  c.append(el('b', '', paths.map(p => p.join('-')).join(', ')), el('span', 'sep', '/'),
           el('span', '', mi.name + (S.mode === 1 ? '' : ` ${mi.sym} ${fmt(S.fixed)}°`) + (S.mode === 3 && S.zenHalf ? ' · ζ 0–180°' : '')),
           el('span', 'sep', '/'), el('span', '', tr('crumb.sun', { v: fmt(S.sun) })));
}
const axName = s => s.split(' ')[0].replace('°', '');
const fmt = v => (Number.isInteger(v) ? String(v) : String(Math.round(v * 100) / 100));

// ============================================================ 渲染
let rafId = 0;
function requestRender() {
  if (rafId) return;
  rafId = requestAnimationFrame(() => { rafId = 0; doRender(false); });
}

function viewState(chrome) {
  const plot = $('plot');
  const P = R ? R.params : null;
  const famTitle = (pr, fm) => { const i = famInfo(fm, pr.faces, P && P.partial); return `${i.title}  ${i.tag}`; };
  const skyTitle = (pr, key) => skyName(key, pr.faces, P && P.partial);
  let title = '';
  if (chrome && R) {
    const lens = S.view === 0 ? ` · ${lensTag(S.lens)} ${S.lens === 5 ? '360×180' : Math.round(S.fov) + '°'}` : tr('export.ori');
    title = tr('export.title', { paths: R.paths.map(p => p.name).join(' + '),
      mode: modeInfo(P.mode).name + (P.mode === 1 ? '' : ' ' + fmt(P.fixed) + '°'), sun: fmt(P.sun),
      refl: tr(P.partial ? 'export.partial' : 'export.pure'), real: P.realTag, view: lens });
  }
  return {
    W: plot.clientWidth, H: plot.clientHeight, dpr: Math.min(window.devicePixelRatio || 1, 3), theme: theme(),
    view: S.view, lens: S.lens, az: S.az, el: S.el, fov: S.fov, xw: S.xw, yw: S.yw, xc: S.xc, yc: S.yc,
    mc: S.mc, arcs: S.arcs, lines: S.lines, endpoints: S.endpoints, sym: S.sym,
    lineW: S.lineW, mcSize: S.mcSize, dragging, checked: S.checked, probe: S.view === 1 ? S.probe : null,
    hoverEp: S.hoverEp, chrome, title, famTitle, skyTitle, drops: S.drops, hlFull: S.hlFull,
    famKey: (pr, name) => (R && R.paths.length > 1 ? pr.name + '·' + name : name),
    axes: axesMeta(R ? R.params.om : modeParams(S.mode, S.fixed).om),
  };
}

function doRender(chrome) {
  if (S.view === 1) ensureMirrors();
  if (S.view === 2) {
    ensureLifts();
    const V = viewState(false);
    view3d.setData(R, V);
    view3d.layout(V);
  } else if (S.view === 3) {
    const V = viewState(false);
    sphere.setData(R, V);
    sphere.layout(V);
  } else renderer.render(viewState(chrome), R);
  // 画面上没有东西可看时说一句为什么
  const note = $('emptyNote');
  let msg = '';
  if (S.view === 2 && R && R.paths.some(p => p.liftPending)) msg = tr('note.lift3dBusy');
  else if (S.view === 2 && R && R.paths.every(p => p.lift)) {
    if (R.paths.every(p => !p.lift.n && !(p.lift.lines || []).some(l => l.seg.length)))
      msg = tr('note.liftNone');
  } else if (R && R.paths.every(p => p.done)) {
    if (S.view === 1 && !R.params.twoDof) msg = tr('note.ori1dof');
    else if ((S.view === 0 || S.view === 3) && R.paths.every(p => (!p.mc || p.mc.length === 0) && (!p.sky || p.sky.every(s => s.pts.length === 0))))
      msg = tr('note.skyNone');
  }
  note.hidden = !msg; note.textContent = msg; note.style.whiteSpace = 'pre-line';
}

// ============================================================ 左右镜像: 取向平面里补上镜像光路 (勾选时才算)
function ensureMirrors() {
  const T = R;
  if (!T || !T.params.twoDof || !S.sym) return;
  for (const pr of T.paths) {
    if (pr.mirror || pr.mirrorPending || !pr.lines || !pr.doneParts.has('ori')) continue;
    pr.mirrorPending = true;
    liftBusy++; $('progress').classList.add('on');
    pool.request(pr.oriSlot, { type: 'mirror' }).then(d => {
      pr.mirrorPending = false;
      if (--liftBusy === 0 && !Rnext) $('progress').classList.remove('on');
      d = d || { status: 'none' };
      if (d.status === 'ok') {
        d.linesMap = new Map(d.lines.map(l => [l.name, l]));
        d.dup = T.paths.some(q => q !== pr && q.name === d.name);   // 镜像光路本来就在输入里
      }
      pr.mirror = d;
      if (T === R) { requestRender(); renderCurves(); }
    });
  }
}

function mirrorNote(pr) {
  const m = pr.mirror;
  if (!m || pr.mirrorPending) return tr('mirror.pending', { name: pr.name });
  if (m.status === 'self') return tr('mirror.self', { name: pr.name });
  if (m.status === 'none') return tr('mirror.none');
  return tr(m.dup ? 'mirror.dup' : 'mirror.ok', { name: m.name });
}

// ============================================================ 3D 抬升: 切到这个视图时才向 Worker 要网格
let liftBusy = 0;
function ensureLifts() {
  const T = R;
  if (!T) return;
  for (const pr of T.paths) {
    if (pr.lift || pr.liftPending) continue;
    let slot, msg;
    if (T.params.twoDof) {
      if (!pr.lines || !pr.doneParts.has('ori')) continue;
      slot = pr.oriSlot; msg = { type: 'lift', ref: S.liftRef };
    } else {
      if (!pr.doneParts.has('sky')) continue;
      slot = pr.skySlot; msg = { type: 'lift1', faces: pr.base.faces, cfg: pr.base.cfg, modeIdx: pr.base.modeIdx, fixedDeg: pr.base.fixedDeg };
    }
    pr.liftPending = true;
    liftBusy++; $('progress').classList.add('on');
    pool.request(slot, msg).then(d => {
      pr.liftPending = false;
      pr.lift = d || { n: 0, lines: [], eps: [] };
      if (--liftBusy === 0 && !Rnext) $('progress').classList.remove('on');
      if (T === R) requestRender();
    });
  }
}

function resetLifts() {
  if (R) for (const pr of R.paths) if (!pr.liftPending) pr.lift = undefined;
  requestRender();
}

function readout3d(h) {
  if (S.view !== 2 && S.view !== 3) return;
  if (!h || !R) { updateReadout(null); return; }
  const ro = $('readout');
  ro.textContent = '';
  const D = h.D, th = R.params.sun * Math.PI / 180, per = v => ((v % 360) + 360) % 360;
  const az = Math.atan2(D[1], D[0]) * 180 / Math.PI, elv = Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI;
  const sep = Math.acos(Math.max(-1, Math.min(1, D[0] * Math.cos(th) + D[2] * Math.sin(th)))) * 180 / Math.PI;
  if (h.onSurf) {
    const ax = axesMeta(R.params.om);
    ro.append(axName(ax.xl) + ' ', el('b', '', per(h.x).toFixed(1) + '°'), '  ' + axName(ax.yl) + ' ', el('b', '', per(h.y).toFixed(1) + '°'), '  →  ');
  }
  ro.append(tr('ro.az') + ' ', el('b', '', az.toFixed(1) + '°'), '  ' + tr('ro.el') + ' ', el('b', '', elv.toFixed(1) + '°'), '  ' + tr('ro.sep') + ' ', el('b', '', sep.toFixed(1) + '°'));
}

// ============================================================ 右栏: 解读
const KIND_ORDER = { end: 0, turn: 1, cusp: 2, cross: 3, aper: 4 };

function renderRead() {
  renderCurves();
  renderEndpoints();
  renderStats();
  $('intro').hidden = !!S.introDone;
}

function famKey(T, pr, fm) { return T.paths.length > 1 ? pr.name + '·' + fm.name : fm.name; }

function renderCurves() {
  const box = $('curves');
  box.textContent = '';
  const T = R;
  if (!T) { box.append(el('div', 'empty', tr('curves.empty'))); return; }
  const th = theme(), multi = T.paths.length > 1, P = T.params;
  let n = 0;
  T.paths.forEach((pr, k) => {
    if (multi) {
      const g = el('div', 'curve-group');
      const dot = el('i'); dot.style.background = css(th.pathcol[k % 6]);
      g.append(dot, document.createTextNode(pr.name));
      box.append(g);
    }
    if (pr.ori) {
      pr.ori.fams.forEach((fm, fi) => {
        n++;
        const info = famInfo(fm, pr.faces, P.partial), key = famKey(T, pr, fm), on = S.checked.has(key);
        const row = el('div', 'curve' + (on ? ' on' : ''));
        row.setAttribute('role', 'button'); row.tabIndex = 0;
        row.setAttribute('aria-pressed', String(on));
        const sw = el('span', 'swt');
        sw.style.background = css(S.view === 2 ? (KIND3D[fm.kind] || [0, 0, 0]) : multi ? th.pathcol[k % 6] : th.palette[fi % th.palette.length]);
        if (multi && S.view !== 2) sw.style.boxShadow = `0 0 0 2px ${css(th.palette[fi % th.palette.length], 0.35)}`;
        const t = el('div', 't');
        const flat = fm.kind === 6 && pr.ori.collapsed;      // 整块压成一条线: 焦散没有意义, 不画
        t.append(el('b', '', info.title), el('code', '', info.tag));
        if (info.note) t.append(el('span', 'note', info.note));
        if (flat) t.append(el('span', 'note', tr('curves.noDraw')));
        const q = el('button', 'qmark', '?'); q.dataset.term = info.term; q.setAttribute('aria-label', tr('curves.explain', { name: info.title }));
        t.append(q);
        const d = el('div', 'd', flat ? tr('curves.collapsed') : term(info.term).short);
        const hl = el('span', 'hl', tr(on ? 'curves.hlOn' : 'curves.hl'));
        row.append(sw, t, hl, d);
        const toggle = () => {
          if (S.checked.has(key)) S.checked.delete(key); else S.checked.add(key);
          save(); requestRender(); renderCurves();
        };
        row.addEventListener('click', e => { if (!e.target.closest('[data-term]')) toggle(); });
        row.addEventListener('keydown', e => { if ((e.key === 'Enter' || e.key === ' ') && e.target === row) { e.preventDefault(); toggle(); } });
        box.append(row);
      });
      if (S.sym && S.view === 1 && P.twoDof) box.append(el('div', 'mirror-note', mirrorNote(pr)));
      if (P.zenHalf && k === T.paths.length - 1) box.append(el('div', 'info-note', tr('curves.zenHalf')));
    } else if (pr.sky) {
      pr.sky.forEach((s, fi) => {
        n++;
        const row = el('div', 'curve static');
        const sw = el('span', 'swt'); sw.style.background = css(multi || S.view === 2 ? th.pathcol[k % 6] : th.palette[skyColorIndex(pr, s.name, fi) % th.palette.length]);
        const t = el('div', 't');
        t.append(el('b', '', skyName(s.name, pr.faces, P.partial)), el('code', '', tr('curves.pts', { n: s.pts.length / 3 })));
        const q = el('button', 'qmark', '?'); q.dataset.term = 'dof'; t.append(q);
        row.append(sw, t, el('span'), el('div', 'd', tr('curves.dof1')));
        box.append(row);
      });
    } else if (!pr.done) box.append(el('div', 'empty', tr('curves.pending', { name: pr.name })));
  });
  $('curveAside').textContent = n ? tr('curves.count', { n }) : '';
  $('curveFoot').hidden = !P.twoDof;
  $('hlFullRow').hidden = !P.twoDof;
}

function renderEndpoints() {
  const sum = $('epSum'), tbl = $('epTable');
  sum.textContent = ''; tbl.textContent = '';
  const T = R;
  $('epAside').textContent = '';
  if (!T) return;
  if (!T.params.twoDof) { sum.append(el('div', 'ep-note', tr('eps.only2dof'))); return; }
  const rows = [];
  let hidden = 0, pending = false;
  T.paths.forEach((pr, k) => {
    if (!pr.eps) { pending = true; return; }
    pr.eps.forEach((r, i) => { if (r.alive) rows.push({ pr, k, i, r }); else hidden++; });
  });
  for (const K of EP_KINDS) {
    const cnt = rows.filter(x => x.r.kind === K.kind).length;
    if (cnt === 0 && ((K.kind === 'cross' && !T.params.partial) || (K.kind === 'aper' && !T.params.real))) continue;
    const b = el('button', 'epk' + (cnt ? '' : ' zero'));
    b.dataset.term = K.term;
    b.append(el('span', 'gl gl-' + K.kind, K.glyph), el('span', 'n', String(cnt)), el('span', 'nm', tr(K.key)));
    sum.append(b);
  }
  if (hidden) {
    const b = el('button', 'epk');
    b.dataset.term = 'ep_hidden';
    b.append(el('span', 'gl', '–'), el('span', 'n', String(hidden)), el('span', 'nm', tr('eps.hidden')));
    sum.append(b);
  }
  if (pending) { tbl.append(el('div', 'empty', tr('eps.pending'))); return; }
  $('epAside').textContent = rows.length ? tr('eps.count', { n: rows.length }) : '';
  if (!rows.length) { tbl.append(el('div', 'empty', tr('eps.none'))); return; }
  rows.sort((a, b) => (KIND_ORDER[a.r.kind] - KIND_ORDER[b.r.kind]) || (a.r.az - b.r.az));
  const ax = axesMeta(T.params.om);
  const hd = el('div', 'eph');
  hd.append(el('span'), el('span', '', axName(ax.xl)), el('span', '', axName(ax.yl)), el('span', '', tr('eps.az')), el('span', '', tr('eps.el')));
  tbl.append(hd);
  const multi = T.paths.length > 1;
  const f3 = v => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(2);
  for (const x of rows) {
    const r = x.r, K = EP_KINDS.find(k => k.kind === r.kind);
    const b = el('button', 'epr');
    b.append(el('span', 'gl gl-' + r.kind, K.glyph), el('span', 'v', r.sc.toFixed(1)), el('span', 'v', r.sw.toFixed(1)),
             el('span', 'v', f3(r.az)), el('span', 'v', f3(r.el)));
    const lbl = el('span', 'lbl');
    if (multi) lbl.append(el('span', 'pn', x.pr.name + '  '));
    lbl.append(document.createTextNode(tr(K.key) + ': ' + epLabel(r.label, T.params.partial)));
    b.append(lbl);
    b.title = `${tr(K.key)} — ${term(K.term).short}`;
    b.addEventListener('mouseenter', () => { S.hoverEp = { k: x.k, i: x.i }; requestRender(); });
    b.addEventListener('mouseleave', () => { S.hoverEp = null; requestRender(); });
    b.addEventListener('focus', () => { S.hoverEp = { k: x.k, i: x.i }; requestRender(); });
    b.addEventListener('blur', () => { S.hoverEp = null; requestRender(); });
    b.addEventListener('click', () => focusEndpoint(r, x.k, x.i));
    tbl.append(b);
  }
}

function focusEndpoint(r, k, i) {
  if (S.view === 2) { view3d.focusEp(k, i); return; }
  if (S.view === 3) { sphere.focusDir(r.D); return; }
  if (S.view === 0) {
    S.az = ((-r.az % 360) + 360) % 360; S.el = r.el;
    S.fov = Math.min(S.fov, 120); clampFov();
  } else {
    S.xc = r.sc; S.yc = r.sw;
    if (S.xw > 120) { S.xw = 90; S.yw = 90; }
  }
  syncCam(); requestRender(); save();
}

function renderStats() {
  const box = $('stats');
  box.textContent = '';
  const T = R;
  $('statsAside').textContent = '';
  if (!T) return;
  const g = el('div', 'grid');
  const ms = v => (Number.isFinite(v) ? (v >= 1000 ? (v / 1000).toFixed(2) + ' s' : v.toFixed(0) + ' ms') : '…');
  const th = theme(), multi = T.paths.length > 1;
  let maxMs = 0;
  T.paths.forEach((pr, k) => {
    const hd = el('div', 'hd');
    if (multi) { const d = el('i'); d.style.background = css(th.pathcol[k % 6]); hd.append(d); }
    hd.append(document.createTextNode(pr.name));
    g.append(hd);
    const add = (a, b) => g.append(el('span', 'k2', a), el('span', '', b));
    const nSky = (pr.sky || []).reduce((s, x) => s + x.pts.length / 3, 0);
    add(tr('stats.pts'), pr.sky ? `${nSky} / ${pr.mc.length / 3}` : '…');
    add(tr('stats.trace'), ms(pr.traceMs));
    if (pr.parts.includes('ori')) {
      add(tr('stats.ori'), ms(pr.oriMs));
      add(tr('stats.eps'), ms(pr.epsMs));
      add(tr('stats.lines'), ms(pr.linesMs));
    }
    add(tr('stats.total'), ms(pr.totalMs));
    if (pr.error) g.append(el('span', 'err', pr.error.split('\n')[0]));
    maxMs = Math.max(maxMs, pr.totalMs || 0);
  });
  box.append(g);
  if (maxMs) $('statsAside').textContent = ms(maxMs);
}

// ============================================================ 位置查看 (取向平面单击)
function probeAt(mx, my) {
  const plot = $('plot');
  const area = oriArea(plot.clientWidth, plot.clientHeight);
  if (mx < area.X || mx > area.X + area.W || my < area.Y || my > area.Y + area.H) return;
  const px = (S.xc - S.xw / 2) + (mx - area.X) / area.W * S.xw;
  const py = (S.yc + S.yw / 2) - (my - area.Y) / area.H * S.yw;
  S.probe = { x: px, y: py };
  requestRender();
  if (!R || !R.params.twoDof) return;
  const T = R, P = S.probe;
  $('probeCard').hidden = false;
  if (S.tab === 'setup') $('tabBadge').classList.add('on');
  const body = $('probeBody');
  body.textContent = '';
  const ax = axesMeta(T.params.om), per = v => ((v % 360) + 360) % 360;
  body.append(el('div', 'probe-pos', `${axName(ax.xl)} = ${per(px).toFixed(2)}°   ${axName(ax.yl)} = ${per(py).toFixed(2)}°`));
  Promise.all(T.paths.map(pr => (pr.ori ? pool.request(pr.oriSlot, { type: 'probe', x: px, y: py }) : Promise.resolve(null))))
    .then(list => {
      if (T !== R || S.probe !== P) return;
      const multi = T.paths.length > 1, th = theme();
      list.forEach((d, k) => { if (d) body.append(probeBlock(d, multi ? th.pathcol[k % 6] : null, T.params)); });
    });
}

function probeBlock(d, col, P) {
  const box = el('div', 'probe-path');
  const h = el('h4');
  if (col) { const i = el('i'); i.style.background = css(col); h.append(i); }
  h.append(document.createTextNode(tr('probe.path', { name: d.name })));
  box.append(h);
  const sg = v => (v < 0 ? '−' : '+') + Math.abs(v).toFixed(3);
  const ck = (state, text, code) => {
    const r = el('div', 'ck ' + state);
    r.append(el('span', 'm', state === 'ok' ? '✓' : state === 'bad' ? '✗' : '–'), el('span', '', text), el('code', '', code));
    box.append(r);
  };
  const tol = 1e-8, faces = d.faces, f0 = faces[0], fE = faces[faces.length - 1];
  ck(d.g0 >= -tol ? 'ok' : 'bad', tr(d.g0 >= -tol ? 'probe.lit' : 'probe.unlit', { f: f0 }), `g₀ = ${sg(d.g0)}`);
  d.qs.forEach((q, i) => {
    const f = faces[i + 1];
    ck(q <= tol ? 'ok' : 'bad', tr(q <= tol ? 'probe.hit' : 'probe.miss', { f }), `q${i + 1} = ${sg(q)}`);
    const tir = q >= -SqK - tol;
    if (tir) ck('ok', tr('probe.tir', { f }), `≥ −0.846`);
    else if (d.partial) ck('na', tr('probe.partial', { f }), `< −0.846`);
    else ck('bad', tr('probe.leak', { f }), `< −0.846`);
  });
  const ex = d.qE <= -SqK + 1e-6;
  ck(ex ? 'ok' : 'bad', tr(ex ? 'probe.out' : 'probe.trapped', { f: fE }), `q_E = ${sg(d.qE)}`);
  if (P.real && Number.isFinite(d.margin)) {
    const okA = d.margin > -1e-6 * Math.max(1, P.h);
    ck(okA ? 'ok' : 'bad', tr(okA ? 'probe.apOk' : 'probe.apBad'), tr('probe.margin', { v: sg(d.margin) }));
  }
  const v = el('div', 'verdict ' + (d.ok ? 'ok' : 'bad'));
  v.textContent = d.ok ? tr('probe.ok', { az: d.az.toFixed(1), el: d.el.toFixed(1) }) : tr('probe.bad');
  box.append(v);
  return box;
}

function clearProbe() { S.probe = null; $('probeCard').hidden = true; requestRender(); }

// ============================================================ 界面语言
function localize() {
  applyI18n();
  document.title = tr('app.title');
  $('metaDesc').setAttribute('content', tr('app.desc'));
  $('selLang').value = S.lang;
  for (const sel of [$('selLens'), $('selLens2')]) lensNames().forEach((n, i) => { if (sel.options[i]) sel.options[i].textContent = n; });
  $('presetsPop').textContent = ''; buildPresets();
  glossary.rebuild();
  syncMode(); syncView(); syncCam(); syncMore(parsePaths(S.path)); renderChain();
  const paths = parsePaths(S.path);
  if (paths) renderCrumbs(paths);
  setStatus(...lastStatus);
  if (lastStatus[0] !== 'busy') $('progress').classList.toggle('on', liftBusy > 0);
  $('btnLift').textContent = tr(view3d.lift > 0.5 ? 'v3.lower' : 'v3.raise');
  if (S.probe) { $('probeCard').hidden = true; S.probe = null; }   // 位置查看的文字是当时生成的, 换语言后收起
  renderRead();
  view3d.requestDraw();
  sphere.requestDraw();
  requestRender();
}

function setLanguage(code) {
  S.lang = setLang(code);
  localize();
  save();
}

// ============================================================ 镜头
function clampFov() { S.fov = Math.max(1, Math.min(fovMax(S.lens), S.fov)); }
function clampOri() { S.xw = Math.max(2, Math.min(360, S.xw)); S.yw = Math.max(2, Math.min(360, S.yw)); }
function resetOriView() {
  const ax = axesMeta(modeParams(S.mode, S.fixed).om);
  S.xw = ax.x1 - ax.x0; S.yw = ax.y1 - ax.y0; S.xc = (ax.x0 + ax.x1) / 2; S.yc = (ax.y0 + ax.y1) / 2;
  if (R) R.oriLocal = null;
  syncCam();
}
function lookAt(az, elv, fov) {
  S.az = ((az % 360) + 360) % 360; S.el = elv;
  if (fov) { S.fov = fov; clampFov(); }
  syncCam(); requestRender(); save();
}
const worldAz = () => { let a = -S.az; a = ((a % 360) + 540) % 360 - 180; return a; };
function syncCam() {
  const set = (id, v) => { if (document.activeElement !== $(id)) $(id).value = String(Math.round(v * 10) / 10); };
  set('inAz', worldAz()); set('inEl', S.el); set('inFov', S.fov);
  $('btnCamNum').textContent = S.lens === 5 ? tr('cam.allSky') : `${Math.round(S.fov)}°`;
  $('selLens').value = S.lens; $('selLens2').value = S.lens;
  $('oriZoomVal').textContent = `${Math.round(S.xw)}°`;
}

function syncView() {
  const v3 = S.view === 2, sph = S.view === 3;
  document.querySelectorAll('#viewSeg button').forEach(b => b.classList.toggle('on', +b.dataset.view === S.view));
  $('cv').hidden = v3 || sph; $('cv3').hidden = !v3; $('cv3o').hidden = !v3; $('cv4').hidden = !sph; $('cv4o').hidden = !sph;
  $('camSky').hidden = S.view !== 0;
  $('camOri').hidden = S.view !== 1;
  $('cam3d').hidden = !v3;
  $('camSph').hidden = !sph;
  $('camPop').hidden = true;
  $('viewQ').dataset.term = ['sky', 'oriplane', 'lift3d', 'sphere'][S.view];
  $('app').classList.toggle('view-ori', S.view === 1);
  $('app').classList.toggle('view-3d', v3);
  document.querySelectorAll('[data-l2]').forEach(e => { e.textContent = tr(v3 ? e.dataset.l3 : e.dataset.l2); });
  view3d.setActive(v3);
  sphere.setActive(sph);
  updateReadout(null);
  renderCurves();
}

function syncMode() {
  document.querySelectorAll('#modes button').forEach(b => {
    const on = +b.dataset.mode === S.mode;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
    b.querySelector('.mode-name').textContent = modeInfo(+b.dataset.mode).name;
  });
  const mi = modeInfo(S.mode);
  $('modeDesc').textContent = mi.desc;
  $('fixedRow').hidden = S.mode === 1;
  $('lbFixed').textContent = mi.label;
  $('slFixed').min = mi.min; $('slFixed').max = mi.max;
  $('slFixed').value = S.fixed;
  if (document.activeElement !== $('inFixed')) $('inFixed').value = S.fixed;
  $('fixedNote').textContent = mi.note;
  const lw = S.mode === 3;
  $('zenRow').hidden = !lw; $('zenNote').hidden = !lw;
  document.querySelectorAll('#zenSeg button').forEach(b => {
    const on = (b.dataset.zen === '1') === S.zenHalf;
    b.classList.toggle('on', on); b.setAttribute('aria-checked', String(on));
  });
  $('zenNote').textContent = tr(S.zenHalf ? 'zen.noteHalf' : 'zen.noteFull');
}

// ============================================================ 读数 (鼠标处)
function updateReadout(e) {
  const ro = $('readout');
  ro.textContent = '';
  const hint = tr(['hint.sky', 'hint.ori', 'hint.3d', 'hint.sph'][S.view]);
  if (!e) { ro.append(el('span', '', hint)); return; }
  const r = $('cv').getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (S.view === 0) {
    const D = renderer.proj.unproject(x, y);
    if (!D) { ro.append(el('span', '', hint)); return; }
    const az = Math.atan2(D[1], D[0]) * 180 / Math.PI, elv = Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI;
    const A = LP.A3, sep = Math.acos(Math.max(-1, Math.min(1, D[0] * A[0] + D[1] * A[1] + D[2] * A[2]))) * 180 / Math.PI;
    ro.append(tr('ro.az') + ' ', el('b', '', az.toFixed(1) + '°'), '  ' + tr('ro.el') + ' ', el('b', '', elv.toFixed(1) + '°'), '  ' + tr('ro.sep') + ' ', el('b', '', sep.toFixed(1) + '°'));
  } else {
    const area = oriArea(r.width, r.height);
    if (x < area.X || x > area.X + area.W || y < area.Y || y > area.Y + area.H) { ro.append(el('span', '', hint)); return; }
    const px = (S.xc - S.xw / 2) + (x - area.X) / area.W * S.xw, py = (S.yc + S.yw / 2) - (y - area.Y) / area.H * S.yw;
    const ax = axesMeta(R ? R.params.om : modeParams(S.mode, S.fixed).om), per = v => ((v % 360) + 360) % 360;
    ro.append(axName(ax.xl) + ' ', el('b', '', per(px).toFixed(1) + '°'), '  ' + axName(ax.yl) + ' ', el('b', '', per(py).toFixed(1) + '°'));
  }
}

// ============================================================ 鼠标 / 触控
function initPointer() {
  const cv = $('cv');
  const pts = new Map();
  let drag = null, pinchD = 0;
  const pinchDist = () => { const a = [...pts.values()]; return Math.hypot(a[0].x - a[1].x, a[0].y - a[1].y); };
  cv.addEventListener('pointerdown', e => {
    cv.setPointerCapture(e.pointerId);
    pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pts.size === 1) drag = { x: e.clientX, y: e.clientY, sx: e.clientX, sy: e.clientY, moved: false };
    else { if (drag) drag.moved = true; pinchD = pinchDist(); }
    closePops();
  });
  cv.addEventListener('pointermove', e => {
    if (!pts.has(e.pointerId)) { if (e.pointerType === 'mouse') updateReadout(e); return; }
    pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pts.size >= 2) {
      const d = pinchDist();
      if (pinchD > 0 && d > 0) zoom(pinchD / d);
      pinchD = d;
      return;
    }
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    if (dx === 0 && dy === 0) return;
    drag.x = e.clientX; drag.y = e.clientY;
    if (!drag.moved && Math.hypot(e.clientX - drag.sx, e.clientY - drag.sy) > 3) drag.moved = true;
    if (!drag.moved) return;
    if (!dragging) { dragging = true; cv.classList.add('dragging'); }
    const W = cv.clientWidth, H = cv.clientHeight;
    if (S.view === 0) {
      const scale = S.fov / Math.max(200, Math.min(W, H));
      S.az = ((S.az - dx * scale) % 360 + 360) % 360;         // 内容跟随光标
      S.el = Math.max(-90, Math.min(90, S.el + dy * scale));
    } else {
      const area = oriArea(W, H);
      if (area.W > 10) { S.xc -= dx * S.xw / area.W; S.yc += dy * S.yw / area.H; }
    }
    syncCam(); requestRender();
    if (e.pointerType === 'mouse') updateReadout(e);
  });
  const up = e => {
    if (!pts.has(e.pointerId)) return;
    pts.delete(e.pointerId);
    if (pts.size > 0) { pinchD = pts.size >= 2 ? pinchDist() : 0; return; }
    const wasDrag = dragging;
    dragging = false; cv.classList.remove('dragging');
    if (drag && !drag.moved && S.view === 1 && e.type === 'pointerup') {
      const r = cv.getBoundingClientRect(); probeAt(e.clientX - r.left, e.clientY - r.top);
    }
    drag = null;
    if (wasDrag) { requestRender(); save(); }
  };
  cv.addEventListener('pointerup', up);
  cv.addEventListener('pointercancel', up);
  cv.addEventListener('pointerleave', () => updateReadout(null));
  cv.addEventListener('wheel', e => {
    e.preventDefault();
    let dy = e.deltaY * (e.deltaMode === 1 ? 33 : e.deltaMode === 2 ? 400 : 1);
    dy = Math.max(-100, Math.min(100, dy));
    zoom(Math.exp(dy * 0.0014));               // 一格滚轮 ≈ ×1.15 / ×0.87 (同原窗体)
  }, { passive: false });
}

function zoom(f) {
  if (S.view === 0) { if (S.lens === 5) return; S.fov *= f; clampFov(); }
  else { S.xw *= f; S.yw *= f; clampOri(); }
  syncCam(); requestRender(); save();
}

// ============================================================ 弹出层
function closePops(except) {
  for (const [pop, btn] of [['layersPop', 'btnLayers'], ['camPop', 'btnCamNum'], ['presetsPop', 'btnPresets']]) {
    if (pop === except) continue;
    $(pop).hidden = true;
    $(btn).setAttribute('aria-expanded', 'false');
  }
}
function togglePop(pop, btn) {
  const show = $(pop).hidden;
  closePops(pop);
  $(pop).hidden = !show;
  $(btn).setAttribute('aria-expanded', String(show));
  return show;
}

function buildPresets() {
  const box = $('presetsPop');
  for (const p of PRESETS) {
    const b = el('button', 'preset');
    b.setAttribute('role', 'menuitem');
    const mi = modeInfo(p.mode);
    const txt = L().presets[p.id];
    b.append(el('b', '', txt.name), el('code', '', p.path),
             el('span', 'meta', `${mi.name}${p.mode === 1 ? '' : ' · ' + mi.sym + ' ' + p.fixed + '°'} · ${tr('preset.sun', { v: p.sun })}`),
             el('span', '', txt.note));
    b.addEventListener('click', () => applyPreset(p));
    box.append(b);
  }
}

function applyPreset(p) {
  closePops();
  S.mode = p.mode; S.fixed = p.fixed; S.sun = p.sun; S.sym = !!p.sym; S.zenHalf = false;
  $('inSun').value = p.sun; $('slSun').value = p.sun; $('ckSym').checked = S.sym;
  syncMode();
  if (p.cam) { S.az = p.cam[0]; S.el = p.cam[1]; S.fov = p.cam[2]; clampFov(); syncCam(); }
  if (S.view === 1) S.view = 0;
  syncView();
  setPathText(p.path);
  compute();
}

// ============================================================ 导出
function exportPng() {
  const now = new Date(), p2 = n => String(n).padStart(2, '0');
  const name = `halo_${(R ? R.paths.map(p => p.name).join('_') : 'empty')}_${['sky', 'orient', '3d', 'sphere'][S.view]}_${p2(now.getHours())}${p2(now.getMinutes())}${p2(now.getSeconds())}.png`;
  const done = blob => {
    if (!blob) return;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = name;
    document.body.append(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 4000);
  };
  if (S.view === 2) { view3d.snapshot(done); return; }
  if (S.view === 3) { sphere.snapshot(done); return; }
  doRender(true);
  $('cv').toBlob(blob => { doRender(false); done(blob); }, 'image/png');
}

// ============================================================ 绑定
function bindNum(inputId, apply) {
  const inp = $(inputId);
  const go = () => { const v = parseFloat(inp.value); if (Number.isFinite(v)) apply(v); };
  inp.addEventListener('change', go);
  inp.addEventListener('keydown', e => { if (e.key === 'Enter') go(); });
}

function initControls() {
  // ---- 语言 ----
  for (const l of LANGS) { const o = el('option', '', l.label); o.value = l.code; $('selLang').append(o); }
  $('selLang').value = S.lang;
  $('selLang').addEventListener('change', () => setLanguage($('selLang').value));

  // ---- ① 光路 ----
  $('inPath').value = S.path;
  $('inPath').addEventListener('input', () => { renderChain(); clearTimeout(editTimer); });
  $('inPath').addEventListener('keydown', e => { if (e.key === 'Enter') compute(); });
  $('inPath').addEventListener('change', compute);
  $('btnCompute').addEventListener('click', compute);
  $('btnBack').addEventListener('click', () => {
    let t = $('inPath').value.replace(/\s+$/, '');
    t = /[,，;；]$/.test(t) ? t.replace(/[,，;；]\s*$/, '') : t.replace(/-?\d*$/, '');
    setPathText(t.replace(/[-\s]+$/, ''), parsePaths(t) ? 550 : undefined);
  });
  $('btnNewPath').addEventListener('click', () => {
    const t = $('inPath').value.trim();
    setPathText(t && !/[,，]$/.test(t) ? t + ', ' : t);
    $('inPath').focus();
  });
  $('btnClear').addEventListener('click', () => { setPathText(''); $('inPath').focus(); });
  $('btnPresets').addEventListener('click', e => {
    e.stopPropagation();
    if (togglePop('presetsPop', 'btnPresets')) {
      const r = $('btnPresets').getBoundingClientRect(), pop = $('presetsPop');
      const w = Math.min(340, window.innerWidth - 24);
      pop.style.left = Math.max(12, Math.min(window.innerWidth - w - 12, r.left)) + 'px';
      pop.style.top = (r.bottom + 6) + 'px';
      pop.style.maxHeight = (window.innerHeight - r.bottom - 18) + 'px';
    }
  });
  buildPresets();

  // ---- ② 取向 ----
  document.querySelectorAll('#zenSeg button').forEach(b => b.addEventListener('click', () => {
    const half = b.dataset.zen === '1';
    if (half === S.zenHalf) return;
    S.zenHalf = half; syncMode(); compute();
  }));
  document.querySelectorAll('#modes button').forEach(b => b.addEventListener('click', () => {
    if (S.mode === +b.dataset.mode) return;
    S.mode = +b.dataset.mode; syncMode(); compute();
  }));
  $('slFixed').addEventListener('input', () => { $('inFixed').value = $('slFixed').value; });
  $('slFixed').addEventListener('change', () => { S.fixed = +$('slFixed').value; compute(); });
  bindNum('inFixed', v => { S.fixed = v; $('slFixed').value = v; compute(); });

  // ---- ③ 太阳 ----
  $('inSun').value = S.sun; $('slSun').value = S.sun;
  $('slSun').addEventListener('input', () => { $('inSun').value = $('slSun').value; });
  $('slSun').addEventListener('change', compute);
  bindNum('inSun', v => { $('slSun').value = v; compute(); });

  // ---- 更多 ----
  $('more').open = !!S.moreOpen;
  $('more').addEventListener('toggle', () => { S.moreOpen = $('more').open; save(); });
  $('ckPartial').checked = S.partial;
  $('ckPartial').addEventListener('change', () => { S.partial = $('ckPartial').checked; compute(); });
  const apexInputs = [['inUpDeg', 'apexUp'], ['inLoDeg', 'apexLo'], ['inUpA', 'upA'], ['inUpB', 'upB'], ['inLoA', 'loA'], ['inLoB', 'loB']];
  for (const [id, key] of apexInputs) {
    $(id).value = S[key];
    bindNum(id, v => { S[key] = key.length === 3 ? Math.round(v) : v; compute(); });
  }
  $('selApex').value = String(S.apexMode);
  $('selApex').addEventListener('change', () => {
    readApex();
    S.apexMode = +$('selApex').value;
    if (S.apexMode === 1) { [S.upA, S.upB] = millerFromApexDeg(S.apexUpEff); [S.loA, S.loB] = millerFromApexDeg(S.apexLoEff); }
    else { S.apexUp = Math.round(S.apexUpEff * 100) / 100; S.apexLo = Math.round(S.apexLoEff * 100) / 100; }
    for (const [id, key] of apexInputs) $(id).value = S[key];
    compute();
  });
  $('ckReal').checked = S.real;
  $('ckReal').addEventListener('change', () => { S.real = $('ckReal').checked; compute(); });
  document.querySelectorAll('[data-d]').forEach(inp => {
    const i = +inp.dataset.d;
    inp.value = S.dist[i];
    bindNum(inp.id || (inp.id = 'inD' + (i + 3)), v => { S.dist[i] = v; compute(); });
  });
  $('inH').value = S.h;
  bindNum('inH', v => { S.h = v; compute(); });
  $('btnRealReset').addEventListener('click', () => {
    S.dist = [1, 1, 1, 1, 1, 1]; S.h = 1;
    document.querySelectorAll('[data-d]').forEach(e => { e.value = 1; });
    $('inH').value = 1;
    compute();
  });

  // ---- 图上浮动工具 ----
  document.querySelectorAll('#viewSeg button').forEach(b => b.addEventListener('click', () => {
    S.view = +b.dataset.view; syncView(); requestRender(); save();
  }));
  const chip = $('chipEps');
  const syncChip = () => chip.setAttribute('aria-pressed', String(S.endpoints));
  syncChip();
  chip.addEventListener('click', () => { S.endpoints = !S.endpoints; syncChip(); requestRender(); save(); });
  $('btnLayers').addEventListener('click', e => { e.stopPropagation(); togglePop('layersPop', 'btnLayers'); });
  for (const [id, key] of [['ckArcs', 'arcs'], ['ckMc', 'mc'], ['ckLines', 'lines'], ['ckSym', 'sym']]) {
    $(id).checked = S[key];
    $(id).addEventListener('change', () => { S[key] = $(id).checked; requestRender(); if (key === 'sym') renderCurves(); save(); });
  }
  for (const [id, out, key] of [['slLineW', 'outLineW', 'lineW'], ['slMcSize', 'outMcSize', 'mcSize']]) {
    $(id).value = S[key]; $(out).textContent = S[key];
    $(id).addEventListener('input', () => { S[key] = +$(id).value; $(out).textContent = S[key]; requestRender(); save(); });
  }
  for (const sel of [$('selLens'), $('selLens2')]) {          // 窄屏时投影选择挪进「图层」菜单
    lensNames().forEach((n, i) => { const o = el('option', '', n); o.value = i; sel.append(o); });
    sel.value = S.lens;
    sel.addEventListener('change', () => { S.lens = +sel.value; clampFov(); syncCam(); requestRender(); save(); });
  }
  document.querySelectorAll('[data-look]').forEach(b => b.addEventListener('click', () => {
    const sun = R ? R.params.sun : S.sun;
    if (b.dataset.look === 'sun') lookAt(0, sun);
    else if (b.dataset.look === 'anti') lookAt(180, sun);
    else lookAt(S.az, 90);                       // 只抬到天顶, 保留当前方位 (不把画面扭回方位 0)
  }));
  document.querySelectorAll('[data-zoom]').forEach(b => b.addEventListener('click', () => zoom(b.dataset.zoom === 'in' ? 0.8 : 1.25)));
  $('btnCamNum').addEventListener('click', e => { e.stopPropagation(); togglePop('camPop', 'btnCamNum'); });
  bindNum('inAz', v => { S.az = ((-v % 360) + 360) % 360; syncCam(); requestRender(); save(); });
  bindNum('inEl', v => { S.el = Math.max(-90, Math.min(90, v)); syncCam(); requestRender(); save(); });
  bindNum('inFov', v => { S.fov = v; clampFov(); syncCam(); requestRender(); save(); });
  $('btnOriReset').addEventListener('click', () => { resetOriView(); requestRender(); save(); });
  document.querySelectorAll('[data-view3]').forEach(b => b.addEventListener('click', () => view3d.goView(b.dataset.view3)));
  document.querySelectorAll('[data-sph]').forEach(b => b.addEventListener('click', () => sphere.goView(b.dataset.sph)));
  $('btnLift').addEventListener('click', () => { $('btnLift').textContent = tr(view3d.toggleLift() > 0.5 ? 'v3.lower' : 'v3.raise'); });
  $('ckHlFull').checked = S.hlFull;
  $('ckHlFull').addEventListener('change', () => { S.hlFull = $('ckHlFull').checked; requestRender(); save(); });
  // 3D 抬升的地面: 平面鱼眼图 / 天球
  const syncBase = () => document.querySelectorAll('#baseSeg button').forEach(b => b.classList.toggle('on', b.dataset.base === view3d.base));
  syncBase();
  document.querySelectorAll('#baseSeg button').forEach(b => b.addEventListener('click', () => {
    S.liftBase = b.dataset.base; view3d.setBase(S.liftBase); syncBase(); save();
  }));
  $('ckDrops').checked = S.drops;
  $('ckDrops').addEventListener('change', () => { S.drops = $('ckDrops').checked; requestRender(); save(); });
  $('slKz').value = S.kz; $('outKz').textContent = S.kz.toFixed(1);
  $('slKz').addEventListener('input', () => {
    S.kz = view3d.kz = +$('slKz').value; $('outKz').textContent = S.kz.toFixed(1);
    view3d.layoutKey = null; requestRender(); save();
  });
  $('selLiftRef').value = S.liftRef;
  $('selLiftRef').addEventListener('change', () => { S.liftRef = $('selLiftRef').value; resetLifts(); save(); });
  $('btnRefine').addEventListener('click', refineWindow);

  // ---- 右栏 / 顶栏 ----
  $('btnProbeClose').addEventListener('click', clearProbe);
  $('btnIntroClose').addEventListener('click', () => { S.introDone = true; $('intro').hidden = true; save(); });
  $('btnGlossary').addEventListener('click', () => glossary.open());
  $('btnExport').addEventListener('click', exportPng);
  document.querySelectorAll('#tabs button').forEach(b => b.addEventListener('click', () => setTab(b.dataset.tab)));
  setTab(S.tab);
  document.addEventListener('click', e => {
    if (!e.target.closest('.pop, #btnLayers, #btnCamNum, #btnPresets')) closePops();
  });
  window.addEventListener('keydown', e => {
    if (e.key === 'Escape') closePops();
    if (e.target.matches('input, select, textarea')) return;
    if (e.key === 'v' || e.key === 'V') { S.view = { 0: 3, 3: 1, 1: 2, 2: 0 }[S.view]; syncView(); requestRender(); save(); }
  });

  syncMode(); syncView(); syncCam(); readApex(); syncMore(parsePaths(S.path));
  renderChain();
  crystal.set(readReal().d, readReal().h, (parsePaths(S.path) || [[]])[0]);
}

function setTab(t) {
  S.tab = t;
  $('app').dataset.tab = t;
  document.querySelectorAll('#tabs button').forEach(b => b.classList.toggle('on', b.dataset.tab === t));
  if (t === 'read') $('tabBadge').classList.remove('on');
  save();
}

function refineWindow() {
  if (!R || !R.paths[0] || !R.paths[0].ori) return;
  const T = R;
  setStatus('busy', 'status.refining'); $('progress').classList.add('on');
  pool.request(R.paths[0].oriSlot, { type: 'refine', x0: S.xc - S.xw / 2, x1: S.xc + S.xw / 2, y0: S.yc - S.yw / 2, y1: S.yc + S.yw / 2 })
    .then(d => {
      if (T !== R) return;
      R.oriLocal = d;
      setStatus(d ? 'ok' : 'err', d ? 'status.refined' : 'status.refineFail', { name: R.paths[0].name });
      requestRender();
    });
}

// ============================================================ 启动
applyI18n();
document.title = tr('app.title');
$('metaDesc').setAttribute('content', tr('app.desc'));
initControls();
initPointer();
new ResizeObserver(requestRender).observe($('plot'));
renderRead();
updateReadout(null);
compute();
