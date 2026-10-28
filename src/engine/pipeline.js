// 计算流水线 — 与原窗体 Recompute → EnsureEndpoints → GetFamLines 的顺序一致.
// Web Worker 与 Node 对账脚本共用这一份.

import { LP, setSun, PathCtx, faceOk, trace2D, traceParry, mc2D, orientCompute, axesMeta } from './lp.js';
import { AP } from './aperture.js';
import { computeEndpoints } from './ep.js';
import { getFamLines, apertureEndpoints } from './formlogic.js';

/**
 * 全局配置 (同窗体 Recompute 开头): 太阳高度°, 部分反射, 锥晶顶角°, 真实晶体孔径层.
 * cfg = { sun, partial, apexUp, apexLo, aperture: { on, dist[6], h } }
 */
export function setConfig(cfg) {
  setSun(cfg.sun);
  LP.partial = !!cfg.partial;
  if (Number.isFinite(cfg.apexUp)) LP.upperApexDeg = cfg.apexUp;
  if (Number.isFinite(cfg.apexLo)) LP.lowerApexDeg = cfg.apexLo;
  const ap = cfg.aperture || {};
  AP.on = !!ap.on;
  AP.H = Number.isFinite(ap.h) ? ap.h : 1;
  AP.dist = (ap.dist && ap.dist.length === 6) ? ap.dist.slice() : [1, 1, 1, 1, 1, 1];
}

export const isTwoDof = modeIdx => modeIdx === 0 || modeIdx === 3;

/** 天球部分: 边界点 (Trace2D / Parry) + MC 填充. modeIdx: 0 柱晶 1 板晶 2 Parry 3 Lowitz */
export function computeSky(P, modeIdx, om, fx, rollDeg) {
  if (modeIdx === 2) return { sky: traceParry(P, (rollDeg % 360) * Math.PI / 180), mc: new Float64Array(0) };
  return { sky: trace2D(P, om, fx), mc: mc2D(P, om, fx, 400000) };
}

/** 取向平面部分 (仅 2DOF): 601×601 网格 + 族根点 + MC. 与天球部分互相独立, 可在另一个 Worker 并行. */
export function computeOri(P, om, fx) {
  const ax = axesMeta(om);
  return orientCompute(P, om, fx,
    ax.x0 * Math.PI / 180, ax.x1 * Math.PI / 180, ax.y0 * Math.PI / 180, ax.y1 * Math.PI / 180,
    601, 601, 120000);
}

/** 主结果 (天球 + 取向平面, 顺序执行; 对账脚本用). */
export function computeMain(faces, modeIdx, om, fx, rollDeg) {
  if (faces.some(f => !faceOk(f))) return null;
  const P = new PathCtx(faces);
  const t0 = performance.now();
  const { sky, mc } = computeSky(P, modeIdx, om, fx, rollDeg);
  const traceMs = performance.now() - t0;
  const ori = isTwoDof(modeIdx) ? computeOri(P, om, fx) : null;
  return { name: P.name, faces, P, sky, mc, ori, traceMs, oriMs: performance.now() - t0 - traceMs };
}

/** 末端点 (2DOF): 闭式引擎 + 真实晶体孔径末端点. */
export function computeEps(pr, om, fx) {
  let eps;
  try { eps = computeEndpoints(pr.faces, om, fx); } catch (e) { eps = []; }
  try { eps.push(...apertureEndpoints(pr.P, pr.ori, om, fx)); } catch (e) { /* 同 C#: 忽略 */ }
  return eps;
}

/** 全部族的连线 (族序 = 绘制顺序; 相接点注册表每光路一份). 返回 Map(族名 → {par, sky}). skip: 不追踪的族名 */
export function computeLines(pr, om, fx, skip = null) {
  const out = new Map();
  if (!pr.ori) return out;
  const reg = [];
  const ax = axesMeta(om);
  for (const fm of pr.ori.fams)
    if (!skip || !skip.has(fm.name)) out.set(fm.name, getFamLines(pr.P, pr.ori, pr.eps, fm, ax.perX, ax.perY, om, fx, reg));
  return out;
}
