// 计算 Worker. 每条光路分两个 Worker 并行:
//   part='sky' : 天球边界点 (Trace2D / Parry) + MC                       → sky, done
//   part='ori' : 取向平面 601² → 末端点 → 连线 (仅 2DOF)               → ori, eps, lines, done
// ori Worker 保留该光路状态, 供点云 / 细化视窗 / 位置查看 / 3D 抬升 / 左右镜像 / 等光强线按需请求 (cloud / refine / probe / lift / mirror / iso → reply).
// 退化光路 (整块姿态压成天上一条线, J ≡ 0) 不追踪焦散: 那里只有舍入噪声 (见 engine/collapse.js).

import { setConfig, computeSky, computeOri, computeEps, computeLines } from './engine/pipeline.js';
import { modeParams, sampleCloud, epAlive } from './engine/formlogic.js';
import { PathCtx, orientCompute, Chain3, validAt, skyD } from './engine/lp.js';
import { apMargin } from './engine/aperture.js';
import { computeLift, computeLift1 } from './engine/lift.js';
import { computeIso } from './engine/iso.js';
import { isCollapsed, isCausticEp } from './engine/collapse.js';
import { mirrorPath } from './engine/mirror.js';

let S = null;   // { pr: { P, faces, name, ori, eps }, om, fx, cfg }

function f32(arr) { return arr instanceof Float32Array ? arr : Float32Array.from(arr); }

function packRoots(P, ori, om, fx, withD) {
  return ori.fams.map(fm => {
    const r = ori.roots.get(fm.name);
    const n = r.sc.length;
    const out = { sc: f32(r.sc), sw: f32(r.sw), ok: r.ok };
    if (withD) {
      // 完整曲线像 (勾选族检视用): 根点的天球方向
      const D = new Float32Array(3 * n);
      for (let i = 0; i < n; i++) {
        const d = skyD(P, om, fx, r.sc[i] * Math.PI / 180, r.sw[i] * Math.PI / 180);
        const nn = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const s = nn > 1e-12 ? 1 / nn : 1;
        D[3 * i] = d[0] * s; D[3 * i + 1] = d[1] * s; D[3 * i + 2] = d[2] * s;
      }
      out.D = D;
    }
    return out;
  });
}

function transferOf(obj, list = []) {
  if (obj && typeof obj === 'object') {
    if (ArrayBuffer.isView(obj)) { if (!list.includes(obj.buffer)) list.push(obj.buffer); return list; }
    for (const k in obj) transferOf(obj[k], list);
  }
  return list;
}

function post(msg) { postMessage(msg, transferOf(msg)); }

function runSky(m, P, om, fx) {
  const t0 = performance.now();
  const r = computeSky(P, m.modeIdx, om, fx, m.fixedDeg);
  const collapsed = (m.modeIdx === 0 || m.modeIdx === 3) && isCollapsed(P, om, fx);
  const sky = [];
  for (const [name, pts] of r.sky) {
    if (collapsed && /^J=0/.test(name)) continue;
    const a = new Float32Array(pts.length * 3);
    for (let i = 0; i < pts.length; i++) { a[3 * i] = pts[i][0]; a[3 * i + 1] = pts[i][1]; a[3 * i + 2] = pts[i][2]; }
    sky.push({ name, pts: a });
  }
  post({ type: 'sky', job: m.job, idx: m.idx, sky, mc: f32(r.mc), ms: performance.now() - t0 });
}

function runOri(m, P, om, fx) {
  const { job, idx } = m;
  S = { pr: null, om, fx, cfg: m.cfg, mirror: null };
  S.pr = oriStages(P, om, fx, (type, data, ms) => post({ type, job, idx, ms, ...data }));
}

/** 取向平面整套: 网格 → 末端点 → 连线, 每完成一段 emit(类型, 数据, 毫秒) 一次. 返回该光路的状态. */
function oriStages(P, om, fx, emit) {
  let t = performance.now();
  const O = computeOri(P, om, fx);
  const collapsed = isCollapsed(P, om, fx);
  const jf = O.fams.find(f => f.kind === 6);
  if (collapsed && jf) O.roots.set(jf.name, { sc: new Float64Array(0), sw: new Float64Array(0), ok: new Uint8Array(0) });
  const pr = { P, faces: P.faces, name: P.name, ori: O, eps: null, collapsed };
  // 分侧着色: 每 ~3 个网格点取一个 (同原窗体 RenderOri 的步长)
  const stepW = Math.max(1, Math.trunc(O.nW / 200)), stepS = Math.max(1, Math.trunc(O.nS / 200));
  const iws = [], iss = [];
  for (let i = 0; i < O.nW; i += stepW) iws.push(i);
  for (let i = 0; i < O.nS; i += stepS) iss.push(i);
  const side = O.fams.map(fm => {
    if (fm.sense === 0) return null;
    const G = O.F.get(fm.name);
    const s = new Uint8Array(iws.length * iss.length);
    for (let a = 0; a < iws.length; a++)
      for (let b = 0; b < iss.length; b++)
        s[a * iss.length + b] = fm.sense * G[iws[a] * O.nS + iss[b]] >= 0 ? 1 : 0;
    return s;
  });
  emit('ori', { ori: {
    fams: O.fams.map(f => ({ name: f.name, kind: f.kind, j: f.j, sense: f.sense })),
    roots: packRoots(P, O, om, fx, true),
    mcX: f32(O.mcX), mcY: f32(O.mcY), collapsed,
    side, sideSC: Float32Array.from(iss.map(i => O.SCs[i] * 180 / Math.PI)),
    sideSW: Float32Array.from(iws.map(i => O.SWs[i] * 180 / Math.PI)),
  } }, performance.now() - t);

  t = performance.now();
  pr.eps = computeEps(pr, om, fx);
  if (collapsed) pr.eps = pr.eps.filter(r => !isCausticEp(r));
  const eps = pr.eps.map(r => ({ kind: r.kind, label: r.label, sc: r.sc, sw: r.sw, D: r.D, az: r.az, el: r.el,
                                 alive: epAlive(P, om, fx, r) }));
  emit('eps', { eps }, performance.now() - t);

  t = performance.now();
  const out = [];
  pr.lines = computeLines(pr, om, fx, collapsed && jf ? new Set([jf.name]) : null);
  // 焦散补线 (fl.conn) 与原窗体同款连线一起交给渲染
  for (const [name, fl] of pr.lines)
    out.push({ name, par: [...fl.par, ...fl.conn.par].map(f32), sky: [...fl.sky, ...fl.conn.sky].map(f32) });
  emit('lines', { lines: out }, performance.now() - t);
  return pr;
}

function runCompute(m) {
  const t0 = performance.now();
  setConfig(m.cfg);
  const { om, fx } = modeParams(m.modeIdx, m.fixedDeg);
  const P = new PathCtx(m.faces);
  if (m.part === 'ori') runOri(m, P, om, fx); else runSky(m, P, om, fx);
  post({ type: 'done', job: m.job, idx: m.idx, part: m.part, ms: performance.now() - t0 });
}


self.onmessage = e => {
  const m = e.data;
  try {
    if (m.type === 'compute') return runCompute(m);
    if (m.type === 'lift1') {                    // 1DOF 的抬升曲线: 不依赖已算的状态, 天球 Worker 直接算
      setConfig(m.cfg);
      const { om, fx } = modeParams(m.modeIdx, m.fixedDeg);
      const fixedV = m.modeIdx === 2 ? ((m.fixedDeg % 360) + 360) % 360 * Math.PI / 180 : 0;
      return post({ type: 'reply', rid: m.rid, data: computeLift1(new PathCtx(m.faces), om, fx, fixedV) });
    }
    if (!S || !S.pr) return post({ type: 'reply', rid: m.rid, data: null });
    setConfig(S.cfg);
    const { pr, om, fx } = S;
    if (m.type === 'cloud') {
      const fm = pr.ori.fams.find(f => f.name === m.fam);
      post({ type: 'reply', rid: m.rid, data: fm ? sampleCloud(pr.P, fm, om, fx) : null });
    } else if (m.type === 'refine') {
      const d2r = Math.PI / 180;
      const r = orientCompute(pr.P, om, fx, m.x0 * d2r, m.x1 * d2r, m.y0 * d2r, m.y1 * d2r, 601, 601, 80000);
      post({ type: 'reply', rid: m.rid, data: { fams: r.fams.map(f => ({ name: f.name })), roots: packRoots(pr.P, r, om, fx, false) } });
    } else if (m.type === 'mirror') {
      // 左右镜像那一半: 找镜像光路; 它自己就是镜像 / 镜像不在这张图里时不用算
      const mp = mirrorPath(pr.faces, om, fx);
      if (!mp || mp.self) return post({ type: 'reply', rid: m.rid, data: { status: mp ? 'self' : 'none' } });
      const t0 = performance.now(), data = { status: 'ok', faces: mp.faces, name: mp.faces.join('-') };
      S.mirror = oriStages(new PathCtx(mp.faces), om, fx, (type, d) => Object.assign(data, d));
      data.ms = performance.now() - t0;
      post({ type: 'reply', rid: m.rid, data });
    } else if (m.type === 'lift') {
      if (!pr.lines) return post({ type: 'reply', rid: m.rid, data: null });
      post({ type: 'reply', rid: m.rid, data: computeLift(pr.P, om, fx, pr.ori.fams, pr.lines, pr.eps, { ref: m.ref }) });
    } else if (m.type === 'iso') {
      if (!pr.lines) return post({ type: 'reply', rid: m.rid, data: null });
      if (pr.collapsed) return post({ type: 'reply', rid: m.rid, data: { status: 'degenerate', curves: [], levels: [] } });
      post({ type: 'reply', rid: m.rid, data: computeIso(pr.P, om, fx, pr.ori.fams, pr.lines, { weight: m.weight, levels: m.levels }) });
    } else if (m.type === 'probe') {
      const sc = m.x * Math.PI / 180, sw = m.y * Math.PI / 180;
      const c = Chain3(sc, sw, om, fx, pr.P);
      let D = [c.Dx, c.Dy, c.Dz];
      const nn = Math.hypot(D[0], D[1], D[2]);
      if (nn > 1e-12) D = D.map(v => v / nn);
      const ok = validAt(pr.P, om, fx, sc, sw, 1e-8);
      const az = Math.atan2(D[1], D[0]) * 180 / Math.PI;
      const el = Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI;
      // 原始链标量交给界面逐条解释 (照到入射面 / 打到反射面 / 全反射 / 能射出 / 孔径)
      post({ type: 'reply', rid: m.rid, data: {
        name: pr.name, faces: pr.faces, g0: c.g0, qs: Array.from(c.qs), qE: c.qE, partial: S.cfg.partial,
        margin: apMargin(pr.P, om, fx, sc, sw), ok, az, el } });
    }
  } catch (err) {
    if (m.type === 'compute') {
      post({ type: 'error', job: m.job, idx: m.idx, part: m.part, message: String(err && err.message || err) });
      post({ type: 'done', job: m.job, idx: m.idx, part: m.part, ms: 0 });
    } else post({ type: 'reply', rid: m.rid, data: null });
  }
};
