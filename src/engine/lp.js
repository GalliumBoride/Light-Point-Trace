// 光点法边界追踪引擎 — HaloLightPointEngine.cs (HaloLP) 的 JS 移植 (窗体用到的全部: 三维链、
// 解析雅可比焦散、族定义、取向平面计算、Trace2D/Parry/MC、等值线追踪+断头延拓).
// 与 C# 逐句对应, 运算次序保持不变 (结果与原窗体逐点对账, 见 tools/compare.mjs).
// 差别只在工程层面: 光路先做成 PathCtx (面法向预先算好), 网格存成 Float64Array.

import { DotNetRandom } from './random.js';
import { AP, apMargin, apOk, apSupports } from './aperture.js';

export const N_ICE = 1.31;
export const K = N_ICE * N_ICE - 1.0;
export const SqK = Math.sqrt(K);
export const KC_ICE = 1.629;                    // c/a, 与后端 Crystal::kC 一致

export const OMode = { Rot1: 0, ZenFree: 1, ZetaFixed: 2, AlphaFixed: 3 };

// ---------- 全局状态 (同 C# 静态字段; 每次计算前由 setConfig 设好) ----------
export const LP = {
  TH: 10.0 * Math.PI / 180.0,
  A3: [Math.cos(10 * Math.PI / 180), 0, Math.sin(10 * Math.PI / 180)],
  partial: false,                               // 允许部分内反射 (ModePartial)
  upperApexDeg: 0,
  lowerApexDeg: 0,
};

export function setSun(deg) {
  LP.TH = deg * Math.PI / 180.0;
  LP.A3 = [Math.cos(LP.TH), 0.0, Math.sin(LP.TH)];
}

// C# Math.Round 默认是银行家舍入 (半数取偶); JS Math.round 是半数向上 — 周期折回/哈希量化处必须一致
export function rint(x) {
  const r = Math.round(x);
  if (Math.abs(x % 1) === 0.5) return 2 * Math.round(x / 2);
  return r;
}

// ---------- 锥晶 (六方锥面 13-18 上 / 23-28 下) ----------
export function apexDegFromMiller(a, b) {
  const H = KC_ICE * a / b;
  return 2.0 * Math.atan((Math.sqrt(3.0) / 2.0) / H) * 180.0 / Math.PI;
}

export function millerFromApexDeg(apexDeg) {
  const half = Math.max(0.01, Math.min(179.99, apexDeg)) / 2.0 * Math.PI / 180.0;
  const H = (Math.sqrt(3.0) / 2.0) / Math.tan(half);
  const r = H / KC_ICE;
  let a = 1, b = 1, best = Number.MAX_VALUE;
  for (let bb = 1; bb <= 12; bb++)
    for (let aa = 1; aa <= 12; aa++) {
      const err = Math.abs(aa / bb - r);
      if (err < best - 1e-12) { best = err; a = aa; b = bb; }
    }
  return [a, b];
}

LP.upperApexDeg = apexDegFromMiller(1, 1);
LP.lowerApexDeg = apexDegFromMiller(1, 1);

export function isPyramidFace(f) {
  return (f >= 13 && f <= 18) || (f >= 23 && f <= 28);
}

export const BETA = {
  3: 0.0, 4: -60 * Math.PI / 180, 5: -120 * Math.PI / 180,
  6: Math.PI, 7: 120 * Math.PI / 180, 8: 60 * Math.PI / 180,
};

export function faceOk(f) {
  if (f === 1 || f === 2 || BETA[f] !== undefined) return true;
  return isPyramidFace(f);
}

// 晶体系下面法向系数 (c1,c2,c3)
export function faceCoef(f) {
  if (f === 1) return [0, 0, 1];
  if (f === 2) return [0, 0, -1];
  if (isPyramidFace(f)) {
    const b0 = BETA[f % 10];
    const apex = f < 20 ? LP.upperApexDeg : LP.lowerApexDeg;
    const delta = apex / 2.0 * Math.PI / 180.0;
    return [Math.cos(b0) * Math.cos(delta), Math.sin(b0) * Math.cos(delta),
            (f < 20 ? 1.0 : -1.0) * Math.sin(delta)];
  }
  const b = BETA[f];
  return [Math.cos(b), Math.sin(b), 0];
}

/** 光路上下文: 面号 + 预先算好的面法向系数 (依赖锥晶顶角, 改顶角后要重建). */
export class PathCtx {
  constructor(faces) {
    this.faces = faces.slice();
    this.name = faces.join('-');
    const n = faces.length;
    this.coef = new Float64Array(3 * n);
    for (let i = 0; i < n; i++) {
      const c = faceCoef(faces[i]);
      this.coef[3 * i] = c[0]; this.coef[3 * i + 1] = c[1]; this.coef[3 * i + 2] = c[2];
    }
    this.nMid = Math.max(0, n - 2);
    this.apSupported = apSupports(faces);
  }
}

// ---------- 取向 ----------
// Euler 取向框架 (_orient_euler): 写入 out[0..8] = e1, e2, e3
export function orientEuler(al, ze, rho, out) {
  const sz = Math.sin(ze), cz = Math.cos(ze);
  const sa = Math.sin(al), ca = Math.cos(al);
  const sr = Math.sin(rho), cr = Math.cos(rho);
  out[6] = sz * ca; out[7] = sz * sa; out[8] = cz;
  const e1b0 = -sa, e1b1 = ca, e1b2 = 0;
  const e2b0 = -cz * ca, e2b1 = -cz * sa, e2b2 = sz;
  out[0] = cr * e1b0 + sr * e2b0; out[1] = cr * e1b1 + sr * e2b1; out[2] = cr * e1b2 + sr * e2b2;
  out[3] = -sr * e1b0 + cr * e2b0; out[4] = -sr * e1b1 + cr * e2b1; out[5] = -sr * e1b2 + cr * e2b2;
  return out;
}

export function orientOf(m, fixedAng, sc, sw, out) {
  if (m === OMode.ZenFree) return orientEuler(sc, sw, fixedAng, out);
  if (m === OMode.ZetaFixed) return orientEuler(sc, fixedAng, sw, out);
  return orientEuler(fixedAng, sw, sc, out);          // AlphaFixed
}

// ---------- 三维链 ----------
export function newC3(P) {
  return { g0: 0, qE: 0, qs: new Float64Array(P.nMid), Dx: 0, Dy: 0, Dz: 0 };
}

const _E = new Float64Array(9);

/** Chain3: 写入 out (newC3 分配), 返回 out. */
export function chain3(sc, sw, m, fixedAng, P, out) {
  const e = orientOf(m, fixedAng, sc, sw, _E);
  const A3 = LP.A3, cf = P.coef, n = P.faces.length;
  let c1 = cf[0], c2 = cf[1], c3 = cf[2];
  let ux = c1 * e[0] + c2 * e[3] + c3 * e[6];
  let uy = c1 * e[1] + c2 * e[4] + c3 * e[7];
  let uz = c1 * e[2] + c2 * e[5] + c3 * e[8];
  const g0 = A3[0] * ux + A3[1] * uy + A3[2] * uz;
  const t0 = -g0 + Math.sqrt(g0 * g0 + K);
  let Px = A3[0] + t0 * ux, Py = A3[1] + t0 * uy, Pz = A3[2] + t0 * uz;
  out.g0 = g0;
  for (let i = 1; i < n - 1; i++) {
    c1 = cf[3 * i]; c2 = cf[3 * i + 1]; c3 = cf[3 * i + 2];
    ux = c1 * e[0] + c2 * e[3] + c3 * e[6];
    uy = c1 * e[1] + c2 * e[4] + c3 * e[7];
    uz = c1 * e[2] + c2 * e[5] + c3 * e[8];
    const q = Px * ux + Py * uy + Pz * uz;
    Px -= 2 * q * ux; Py -= 2 * q * uy; Pz -= 2 * q * uz;
    out.qs[i - 1] = q;
  }
  const j = 3 * (n - 1);
  c1 = cf[j]; c2 = cf[j + 1]; c3 = cf[j + 2];
  ux = c1 * e[0] + c2 * e[3] + c3 * e[6];
  uy = c1 * e[1] + c2 * e[4] + c3 * e[7];
  uz = c1 * e[2] + c2 * e[5] + c3 * e[8];
  const qE = Px * ux + Py * uy + Pz * uz;
  const tE = -qE - Math.sqrt(Math.max(qE * qE - K, 0.0));
  out.qE = qE;
  out.Dx = Px + tE * ux; out.Dy = Py + tE * uy; out.Dz = Pz + tE * uz;
  return out;
}

/** 分配新结果的 Chain3 (非热路径用). */
export function Chain3(sc, sw, m, fixedAng, P) {
  return chain3(sc, sw, m, fixedAng, P, newC3(P));
}

export function valid3(c, tol) {
  if (!(c.g0 >= -tol)) return false;
  if (!(c.qE <= -SqK + Math.max(1e-6, tol))) return false;
  const qs = c.qs, partial = LP.partial;
  for (let i = 0; i < qs.length; i++) {
    if (!(qs[i] <= tol)) return false;
    if (!partial && !(qs[i] >= -SqK - tol)) return false;
  }
  return true;
}

// ---------- 解析雅可比 Jana3 (世界系生成元) ----------
function gens(m, fixedAng, sc, sw, w) {      // w[0..2]=w1, w[3..5]=w2
  if (m === OMode.ZenFree) {
    w[0] = 0.0; w[1] = 0.0; w[2] = 1.0;
    w[3] = -Math.sin(sc); w[4] = Math.cos(sc); w[5] = 0.0;
  } else if (m === OMode.ZetaFixed) {
    w[0] = 0.0; w[1] = 0.0; w[2] = 1.0;
    w[3] = Math.sin(fixedAng) * Math.cos(sc); w[4] = Math.sin(fixedAng) * Math.sin(sc); w[5] = Math.cos(fixedAng);
  } else {
    w[0] = Math.sin(sw) * Math.cos(fixedAng); w[1] = Math.sin(sw) * Math.sin(fixedAng); w[2] = Math.cos(sw);
    w[3] = -Math.sin(fixedAng); w[4] = Math.cos(fixedAng); w[5] = 0.0;
  }
}

function clampMask(qE, v) {
  return qE * qE - K > 1e-12 ? v : NaN;
}

const _JE = new Float64Array(9), _JW = new Float64Array(6);
const _JU = new Float64Array(3 * 64);

export function jana3(sc, sw, m, fixedAng, P) {
  const e = orientOf(m, fixedAng, sc, sw, _JE);
  gens(m, fixedAng, sc, sw, _JW);
  const cf = P.coef, n = P.faces.length, A3 = LP.A3;
  const U = n <= 64 ? _JU : new Float64Array(3 * n);
  for (let i = 0; i < n; i++) {
    const c1 = cf[3 * i], c2 = cf[3 * i + 1], c3 = cf[3 * i + 2];
    U[3 * i] = c1 * e[0] + c2 * e[3] + c3 * e[6];
    U[3 * i + 1] = c1 * e[1] + c2 * e[4] + c3 * e[7];
    U[3 * i + 2] = c1 * e[2] + c2 * e[5] + c3 * e[8];
  }
  const u00 = U[0], u01 = U[1], u02 = U[2];
  const g0 = u00 * A3[0] + u01 * A3[1] + u02 * A3[2];
  const S0 = Math.sqrt(g0 * g0 + K);
  const t0 = -g0 + S0, t0p = g0 / S0 - 1.0;
  let P0 = A3[0] + t0 * u00, P1 = A3[1] + t0 * u01, P2 = A3[2] + t0 * u02;
  // dP[k] (k=0,1)
  let d00, d01, d02, d10, d11, d12;
  {
    // du = w × u0
    let wx = _JW[0], wy = _JW[1], wz = _JW[2];
    let dux = wy * u02 - wz * u01, duy = wz * u00 - wx * u02, duz = wx * u01 - wy * u00;
    let dg = dux * A3[0] + duy * A3[1] + duz * A3[2];
    d00 = t0p * dg * u00 + t0 * dux; d01 = t0p * dg * u01 + t0 * duy; d02 = t0p * dg * u02 + t0 * duz;
    wx = _JW[3]; wy = _JW[4]; wz = _JW[5];
    dux = wy * u02 - wz * u01; duy = wz * u00 - wx * u02; duz = wx * u01 - wy * u00;
    dg = dux * A3[0] + duy * A3[1] + duz * A3[2];
    d10 = t0p * dg * u00 + t0 * dux; d11 = t0p * dg * u01 + t0 * duy; d12 = t0p * dg * u02 + t0 * duz;
  }
  for (let i = 1; i < n - 1; i++) {
    const ux = U[3 * i], uy = U[3 * i + 1], uz = U[3 * i + 2];
    const q = P0 * ux + P1 * uy + P2 * uz;
    // k = 0
    let wx = _JW[0], wy = _JW[1], wz = _JW[2];
    let dux = wy * uz - wz * uy, duy = wz * ux - wx * uz, duz = wx * uy - wy * ux;
    let dq = d00 * ux + d01 * uy + d02 * uz + P0 * dux + P1 * duy + P2 * duz;
    d00 -= 2 * dq * ux + 2 * q * dux; d01 -= 2 * dq * uy + 2 * q * duy; d02 -= 2 * dq * uz + 2 * q * duz;
    // k = 1
    wx = _JW[3]; wy = _JW[4]; wz = _JW[5];
    dux = wy * uz - wz * uy; duy = wz * ux - wx * uz; duz = wx * uy - wy * ux;
    dq = d10 * ux + d11 * uy + d12 * uz + P0 * dux + P1 * duy + P2 * duz;
    d10 -= 2 * dq * ux + 2 * q * dux; d11 -= 2 * dq * uy + 2 * q * duy; d12 -= 2 * dq * uz + 2 * q * duz;
    P0 -= 2 * q * ux; P1 -= 2 * q * uy; P2 -= 2 * q * uz;
  }
  const j = 3 * (n - 1);
  const ex = U[j], ey = U[j + 1], ez = U[j + 2];
  const qE = P0 * ex + P1 * ey + P2 * ez;
  const TE = Math.sqrt(Math.max(qE * qE - K, 1e-30));
  const tE = -qE - TE;
  const D0 = P0 + tE * ex, D1 = P1 + tE * ey, D2 = P2 + tE * ez;
  let a0, a1, a2, b0, b1, b2;
  {
    let wx = _JW[0], wy = _JW[1], wz = _JW[2];
    let dux = wy * ez - wz * ey, duy = wz * ex - wx * ez, duz = wx * ey - wy * ex;
    let dq = d00 * ex + d01 * ey + d02 * ez + P0 * dux + P1 * duy + P2 * duz;
    let dt = -(1.0 + qE / TE) * dq;
    a0 = d00 + dt * ex + tE * dux; a1 = d01 + dt * ey + tE * duy; a2 = d02 + dt * ez + tE * duz;
    wx = _JW[3]; wy = _JW[4]; wz = _JW[5];
    dux = wy * ez - wz * ey; duy = wz * ex - wx * ez; duz = wx * ey - wy * ex;
    dq = d10 * ex + d11 * ey + d12 * ez + P0 * dux + P1 * duy + P2 * duz;
    dt = -(1.0 + qE / TE) * dq;
    b0 = d10 + dt * ex + tE * dux; b1 = d11 + dt * ey + tE * duy; b2 = d12 + dt * ez + tE * duz;
  }
  const cr0 = a1 * b2 - a2 * b1, cr1 = a2 * b0 - a0 * b2, cr2 = a0 * b1 - a1 * b0;
  return clampMask(qE, cr0 * D0 + cr1 * D1 + cr2 * D2);
}

// ---------- 族定义 (alg_fams: 名称/求值/满足侧) ----------
// Kind: 0=g0, 1=q_j(=0退化), 2=q_j+√K(TIR), 3=qE+√K(出射), 6=J3 焦散, 7=孔径篱笆
export function algFams(faces) {
  const fams = [];
  fams.push({ name: 'g0=0 入射掠射', kind: 0, j: 0, sense: 1 });
  for (let j = 0; j < faces.length - 2; j++) {
    const f = faces[j + 1];
    fams.push({ name: `q${j + 1}(面${f})=0 退化`, kind: 1, j, sense: -1 });
    const nm = LP.partial ? `蓝边 q${j + 1}(面${f})=-√K 阶跃` : `q${j + 1}(面${f})=-√K TIR`;
    fams.push({ name: nm, kind: 2, j, sense: 1 });
  }
  fams.push({ name: '出射临界', kind: 3, j: 0, sense: -1 });
  return fams;
}

export function allFams(P, m) {
  const fams = algFams(P.faces);
  fams.push({ name: 'J=0 焦散', kind: 6, j: 0, sense: 0 });
  if (AP.on && P.apSupported)
    fams.push({ name: '孔径篱笆(真实晶体)', kind: 7, j: 0, sense: 1 });
  return fams;
}

export function famEval3(fm, c) {
  switch (fm.kind) {
    case 0: return c.g0;
    case 1: return c.qs[fm.j];
    case 2: return c.qs[fm.j] + SqK;
    default: return c.qE + SqK;
  }
}

function bracketOk(f0, f1) {
  if (!Number.isFinite(f0) || !Number.isFinite(f1)) return false;
  if (f0 * f1 > 0) return false;
  return Math.max(Math.abs(f0), Math.abs(f1)) > 1e-12;
}

const _FC = { g0: 0, qE: 0, qs: new Float64Array(64), Dx: 0, Dy: 0, Dz: 0 };
function scratchC3(P) {
  if (_FC.qs.length !== P.nMid) _FC.qs = new Float64Array(P.nMid);
  return _FC;
}

export function famValue(fm, P, m, fixedAng, sc, sw) {
  if (fm.kind === 7) return apMargin(P, m, fixedAng, sc, sw);
  if (fm.kind === 6) return jana3(sc, sw, m, fixedAng, P);
  return famEval3(fm, chain3(sc, sw, m, fixedAng, P, scratchC3(P)));
}

/** 只用光点判据 (不含孔径层) 的有效性. */
export function validLp(P, m, fixedAng, sc, sw, tol) {
  return valid3(chain3(sc, sw, m, fixedAng, P, scratchC3(P)), tol);
}

export function validAt(P, m, fixedAng, sc, sw, tol) {
  return validLp(P, m, fixedAng, sc, sw, tol) && apOk(P, m, fixedAng, sc, sw);
}

/** 逐族有效性: 孔径篱笆自身是 margin=0, 只测光点判据 (否则被自己切碎). */
export function validForFam(fm, P, m, fixedAng, sc, sw, tol) {
  if (fm && fm.kind === 7) return validLp(P, m, fixedAng, sc, sw, tol);
  return validAt(P, m, fixedAng, sc, sw, tol);
}

export function skyD(P, m, fixedAng, sc, sw) {
  const c = chain3(sc, sw, m, fixedAng, P, scratchC3(P));
  return [c.Dx, c.Dy, c.Dz];
}

// ---------- 取向平面计算 (orientation_viewer_cf.compute_one) ----------
export function axesMeta(m) {
  // ζ 轴取 0-360 周期 (双覆盖): sinζ<0 即 c 轴翻到对侧方位, 链对角度天然周期
  switch (m) {
    case OMode.Rot1: return { xl: 'φ 面内方位°', x0: 0, x1: 360, perX: true, yl: 'α 光点方位°', y0: 0, y1: 360, perY: true };
    case OMode.ZenFree: return { xl: 'α 方位°', x0: 0, x1: 360, perX: true, yl: 'ζ 天顶角°', y0: 0, y1: 360, perY: true };
    case OMode.ZetaFixed: return { xl: 'α 方位°', x0: 0, x1: 360, perX: true, yl: 'roll°', y0: 0, y1: 360, perY: true };
    default: return { xl: 'roll°', x0: 0, x1: 360, perX: true, yl: 'ζ 天顶角°', y0: 0, y1: 360, perY: true };
  }
}

/**
 * 取向平面: 各族场网格 F (Float64Array, [iw*nS+isc]), 有效性, 双轴括根 + 48 次二分的根点 (deg), MC.
 * 返回 { SCs, SWs (rad), F: Map(name→grid), valid: Uint8Array, fams, roots: Map(name→{sc,sw,ok}), mcX, mcY (deg) }
 */
export function orientCompute(P, m, fixedAng, x0, x1, y0, y1, nS = 721, nW = 721, M = 250000) {
  const t0 = Date.now();
  const fams = allFams(P, m);
  const SCs = new Float64Array(nS), SWs = new Float64Array(nW);
  {
    const ax = axesMeta(m);
    const offX = (x1 - x0) / (nS - 1) * 0.1371, offY = (y1 - y0) / (nW - 1) * 0.1371;
    for (let i = 0; i < nS; i++) SCs[i] = ax.perX ? x0 + offX + (x1 - x0) * i / (nS - 1)
                                                  : x0 + 1e-4 + (x1 - x0 - 2e-4) * i / (nS - 1);
    for (let i = 0; i < nW; i++) SWs[i] = ax.perY ? y0 + offY + (y1 - y0) * i / (nW - 1)
                                                  : y0 + 1e-4 + (y1 - y0 - 2e-4) * i / (nW - 1);
  }
  const F = new Map();
  const grids = fams.map(fm => { const g = new Float64Array(nW * nS); F.set(fm.name, g); return g; });
  const valid = new Uint8Array(nW * nS);
  const c = newC3(P);
  for (let iw = 0; iw < nW; iw++)
    for (let isc = 0; isc < nS; isc++) {
      const sc = SCs[isc], swv = SWs[iw], idx = iw * nS + isc;
      chain3(sc, swv, m, fixedAng, P, c);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind <= 3) grids[k][idx] = famEval3(fams[k], c);
      let ok = valid3(c, 1e-8);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind === 6) grids[k][idx] = jana3(sc, swv, m, fixedAng, P);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind === 7) grids[k][idx] = apMargin(P, m, fixedAng, sc, swv);
      if (ok && !apOk(P, m, fixedAng, sc, swv)) ok = false;
      valid[idx] = ok ? 1 : 0;
    }

  const dS = SCs[1] - SCs[0], dW = SWs[1] - SWs[0];
  const roots = new Map();
  for (let k = 0; k < fams.length; k++) {
    const fm = fams[k], G = grids[k];
    const sc = [], swl = [];
    for (let iw = 0; iw < nW; iw++)
      for (let isc = 0; isc < nS - 1; isc++)
        if (bracketOk(G[iw * nS + isc], G[iw * nS + isc + 1]))
          bisect1(fm, P, m, fixedAng, SCs[isc], dS, SWs[iw], false, sc, swl);
    for (let iw = 0; iw < nW - 1; iw++)
      for (let isc = 0; isc < nS; isc++)
        if (bracketOk(G[iw * nS + isc], G[(iw + 1) * nS + isc]))
          bisect1(fm, P, m, fixedAng, SWs[iw], dW, SCs[isc], true, sc, swl);
    const n = sc.length;
    const ok = new Uint8Array(n), scD = new Float64Array(n), swD = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      ok[i] = validForFam(fm, P, m, fixedAng, sc[i], swl[i], 1e-6) ? 1 : 0;
      scD[i] = sc[i] * 180 / Math.PI; swD[i] = swl[i] * 180 / Math.PI;
    }
    roots.set(fm.name, { sc: scD, sw: swD, ok });
  }

  const rng = new DotNetRandom(7);
  const mx = [], my = [];
  for (let i = 0; i < M; i++) {
    const p0 = x0 + rng.nextDouble() * (x1 - x0);
    const p1 = y0 + rng.nextDouble() * (y1 - y0);
    if (validAt(P, m, fixedAng, p0, p1, 1e-8)) { mx.push(p0 * 180 / Math.PI); my.push(p1 * 180 / Math.PI); }
  }
  return { SCs, SWs, nS, nW, F, valid, fams, roots, mcX: Float64Array.from(mx), mcY: Float64Array.from(my),
           elapsedMs: Date.now() - t0 };
}

function bisect1(fm, P, m, fixedAng, lo, wid, fix, varIsSweep, outSc, outSw) {
  let hi = lo + wid;
  const FF = varIsSweep ? (v => famValue(fm, P, m, fixedAng, fix, v)) : (v => famValue(fm, P, m, fixedAng, v, fix));
  let flo = FF(lo);
  const fhi = FF(hi);
  if (!bracketOk(flo, fhi)) return;
  for (let it = 0; it < 48; it++) {
    const mid = 0.5 * (lo + hi);
    const fmv = FF(mid);
    if (flo * fmv <= 0) hi = mid; else { lo = mid; flo = fmv; }
  }
  const vv = 0.5 * (lo + hi);
  if (varIsSweep) { outSc.push(fix); outSw.push(vv); }
  else { outSc.push(vv); outSw.push(fix); }
}

// ---------- 3D 模式追踪 (trace2d 简明版: 帽/双轴求根) ----------
/** 返回 Map(族名 → 天球点 [x,y,z][]) */
export function trace2D(P, m, fixedAng, nSW = 900, nSC = 720) {
  const ax = axesMeta(m);
  const perX = ax.perX, perY = ax.perY;
  const X0 = ax.x0 * Math.PI / 180, X1 = ax.x1 * Math.PI / 180;
  const Y0 = ax.y0 * Math.PI / 180, Y1 = ax.y1 * Math.PI / 180;
  const c = newC3(P);

  // 定ζ 极点退化 -> 1DOF
  if (m === OMode.ZetaFixed && Math.abs(Math.sin(fixedAng)) < 1e-7) {
    const pts1 = [];
    for (let i = 0; i < 24000; i++) {
      const ph = 2 * Math.PI * i / 24000;
      chain3(0, ph, m, fixedAng, P, c);
      if (valid3(c, 1e-7) && apOk(P, m, fixedAng, 0, ph)) pts1.push([c.Dx, c.Dy, c.Dz]);
    }
    return new Map([['固定ζ极点退化1DOF', pts1]]);
  }

  const fams = allFams(P, m);
  const nX = nSC + 1, nY = nSW + 1;
  const SCs = new Float64Array(nX), SWs = new Float64Array(nY);
  const offTx = perX ? (X1 - X0) / nSC * 0.1371 : 0;
  const offTy = perY ? (Y1 - Y0) / nSW * 0.1371 : 0;
  for (let i = 0; i <= nSC; i++) SCs[i] = perX ? X0 + offTx + (X1 - X0) * i / nSC : X0 + 1e-4 + (X1 - X0 - 2e-4) * i / nSC;
  for (let i = 0; i <= nSW; i++) SWs[i] = perY ? Y0 + offTy + (Y1 - Y0) * i / nSW : Y0 + 1e-4 + (Y1 - Y0 - 2e-4) * i / nSW;
  const dS = SCs[1] - SCs[0], dW = SWs[1] - SWs[0];

  const grids = fams.map(() => new Float64Array(nY * nX));
  for (let iw = 0; iw <= nSW; iw++)
    for (let isc = 0; isc <= nSC; isc++) {
      const idx = iw * nX + isc;
      chain3(SCs[isc], SWs[iw], m, fixedAng, P, c);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind <= 3) grids[k][idx] = famEval3(fams[k], c);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind === 6) grids[k][idx] = jana3(SCs[isc], SWs[iw], m, fixedAng, P);
      for (let k = 0; k < fams.length; k++)
        if (fams[k].kind === 7) grids[k][idx] = apMargin(P, m, fixedAng, SCs[isc], SWs[iw]);
    }

  const outp = new Map();
  // scan-无关帽族 (sweep-帽)
  const capNames = new Set();
  const sweepLabel = ax.yl.split(' ')[0].replace('°', '');
  for (let k = 0; k < fams.length; k++) {
    const fm = fams[k];
    if (fm.kind > 3) continue;
    const G = grids[k];
    let mxv = 0;
    for (let iw = 0; iw <= nSW; iw++)
      for (let isc = 0; isc <= nSC; isc++)
        mxv = Math.max(mxv, Math.abs(G[iw * nX + isc] - G[iw * nX]));
    if (mxv > 1e-10) continue;
    capNames.add(fm.name);
    const pts = [];
    for (let iw = 0; iw < nSW; iw++) {
      if (!bracketOk(G[iw * nX], G[(iw + 1) * nX])) continue;
      let lo = SWs[iw], hi = SWs[iw] + dW;
      let flo = famEval3(fm, chain3(SCs[0], lo, m, fixedAng, P, c));
      for (let it = 0; it < 48; it++) {
        const mid = 0.5 * (lo + hi);
        const fv = famEval3(fm, chain3(SCs[0], mid, m, fixedAng, P, c));
        if (flo * fv <= 0) hi = mid; else { lo = mid; flo = fv; }
      }
      const wstar = 0.5 * (lo + hi);
      for (let kk = 0; kk < 12000; kk++) {
        const sc = X0 + (X1 - X0) * kk / 12000.0;
        chain3(sc, wstar, m, fixedAng, P, c);
        if (valid3(c, 1e-6) && apOk(P, m, fixedAng, sc, wstar)) pts.push([c.Dx, c.Dy, c.Dz]);
      }
    }
    outp.set(sweepLabel + '-帽 ' + fm.name, pts);
  }

  // 域边帽 (非周期 sweep; 窗体模式下所有轴都是周期, 保留以备 AlphaFixed 等)
  if (!perY) {
    for (const edge of [{ V: Y0, L: 'ζ=0°' }, { V: Y1, L: 'ζ=180°' }]) {
      const pts = [];
      for (let kk = 0; kk < 12000; kk++) {
        const sc = X0 + (X1 - X0) * kk / 12000.0;
        chain3(sc, edge.V, m, fixedAng, P, c);
        if (valid3(c, 1e-6) && apOk(P, m, fixedAng, sc, edge.V)) pts.push([c.Dx, c.Dy, c.Dz]);
      }
      outp.set('域边帽 ' + edge.L, pts);
    }
  }

  // 其余族: 双轴括根 + 二分
  for (let k = 0; k < fams.length; k++) {
    const fm = fams[k];
    if (capNames.has(fm.name)) continue;
    const G = grids[k];
    const pts = [];
    for (let iw = 0; iw <= nSW; iw++)
      for (let isc = 0; isc < nSC; isc++)
        if (bracketOk(G[iw * nX + isc], G[iw * nX + isc + 1]))
          solve2D(fm, P, m, fixedAng, SCs[isc], dS, SWs[iw], false, pts);
    for (let iw = 0; iw < nSW; iw++)
      for (let isc = 0; isc <= nSC; isc++)
        if (bracketOk(G[iw * nX + isc], G[(iw + 1) * nX + isc]))
          solve2D(fm, P, m, fixedAng, SWs[iw], dW, SCs[isc], true, pts);
    outp.set(fm.name, pts);
  }
  return outp;
}

function solve2D(fm, P, m, fixedAng, lo, wid, fix, varIsSweep, pts) {
  let hi = lo + wid;
  const FF = varIsSweep ? (v => famValue(fm, P, m, fixedAng, fix, v)) : (v => famValue(fm, P, m, fixedAng, v, fix));
  let flo = FF(lo);
  const fhi = FF(hi);
  if (!bracketOk(flo, fhi)) return;
  // 预筛 (tol=0.08, 与 trace2d 一致)
  const sc0 = varIsSweep ? fix : 0.5 * (lo + hi);
  const sw0 = varIsSweep ? 0.5 * (lo + hi) : fix;
  if (!validAt(P, m, fixedAng, sc0, sw0, 0.08)) return;
  for (let it = 0; it < 48; it++) {
    const mid = 0.5 * (lo + hi);
    const fv = FF(mid);
    if (flo * fv <= 0) hi = mid; else { lo = mid; flo = fv; }
  }
  const vv = 0.5 * (lo + hi);
  const scr = varIsSweep ? fix : vv;
  const swr = varIsSweep ? vv : fix;
  if (!validAt(P, m, fixedAng, scr, swr, 1e-8)) return;
  const c = Chain3(scr, swr, m, fixedAng, P);
  pts.push([c.Dx, c.Dy, c.Dz]);
}

// ---------- 1DOF / 随机 ----------
export function traceParry(P, roll0) {
  const pts = [];
  const c = newC3(P);
  for (let i = 0; i < 24000; i++) {
    const al = 2 * Math.PI * i / 24000;
    // Parry: c 轴水平 (ζ=90°), roll 锁定, 仅 α 自由
    chain3(al, roll0, OMode.ZetaFixed, Math.PI / 2, P, c);
    if (valid3(c, 1e-7) && apOk(P, OMode.ZetaFixed, Math.PI / 2, al, roll0)) pts.push([c.Dx, c.Dy, c.Dz]);
  }
  return new Map([['Parry轨迹(1DOF)', pts]]);
}

/** MC 填充: 返回 Float64Array (x,y,z 连排) */
export function mc2D(P, m, fixedAng, M, seed = 5) {
  const ax = axesMeta(m);
  const rng = new DotNetRandom(seed);
  const pts = [];
  const X0 = ax.x0 * Math.PI / 180, X1 = ax.x1 * Math.PI / 180;
  const Y0 = ax.y0 * Math.PI / 180, Y1 = ax.y1 * Math.PI / 180;
  const c = newC3(P);
  for (let i = 0; i < M; i++) {
    const sc = X0 + rng.nextDouble() * (X1 - X0);
    const sw = Y0 + rng.nextDouble() * (Y1 - Y0);
    chain3(sc, sw, m, fixedAng, P, c);
    if (valid3(c, 0) && apOk(P, m, fixedAng, sc, sw)) pts.push(c.Dx, c.Dy, c.Dz);
  }
  return Float64Array.from(pts);
}

// ---------- 等值线追踪 (F = 0): marching squares + 端点链接, 周期轴跨缝连续 ----------
// F[iw*nS+isc] 定义在 SCs (x, nS 点) × SWs (y, nW 点) 上; 周期轴末列/末行与首列/首行重合.
// 返回折线按取向空间顺序, 坐标已"展开" (跨缝时加减周期). refine(xa,ya,xb,yb,fa,fb) 返回边上零点 t∈[0,1].
export function traceContours(F, SCs, SWs, perX, perY, refine = null, evalF = null, noiseFloor = 0.0) {
  const nW = SWs.length, nS = SCs.length;
  const result = [];
  if (nW < 2 || nS < 2) return result;
  const Px = SCs[nS - 1] - SCs[0], Py = SWs[nW - 1] - SWs[0];
  const dxg = Px / (nS - 1), dyg = Py / (nW - 1);
  const segs = [];   // [x0,y0,x1,y1]
  const cross = (xa, ya, xb, yb, fa, fb) => {
    let t = refine !== null ? refine(xa, ya, xb, yb, fa, fb) : fa / (fa - fb);
    if (Number.isNaN(t) || t < 0) t = 0;
    if (t > 1) t = 1;
    return [xa + (xb - xa) * t, ya + (yb - ya) * t];
  };
  for (let iw = 0; iw < nW - 1; iw++)
    for (let isc = 0; isc < nS - 1; isc++) {
      const f00 = F[iw * nS + isc], f01 = F[iw * nS + isc + 1], f11 = F[(iw + 1) * nS + isc + 1], f10 = F[(iw + 1) * nS + isc];
      if (!Number.isFinite(f00) || !Number.isFinite(f01) || !Number.isFinite(f11) || !Number.isFinite(f10)) continue;
      // 噪声地板: 四角 |F| 全在舍入残渣量级 = 恒零平地 (如 3-5-7 的退化 J 场)
      if (noiseFloor > 0 &&
          Math.abs(f00) < noiseFloor && Math.abs(f01) < noiseFloor &&
          Math.abs(f11) < noiseFloor && Math.abs(f10) < noiseFloor) continue;
      const bits = (f00 < 0 ? 1 : 0) | (f01 < 0 ? 2 : 0) | (f11 < 0 ? 4 : 0) | (f10 < 0 ? 8 : 0);
      if (bits === 0 || bits === 15) continue;
      const x0 = SCs[isc], x1 = SCs[isc + 1], y0 = SWs[iw], y1 = SWs[iw + 1];
      const pts = [];
      if ((f00 < 0) !== (f01 < 0)) pts.push(cross(x0, y0, x1, y0, f00, f01));   // 下边
      if ((f01 < 0) !== (f11 < 0)) pts.push(cross(x1, y0, x1, y1, f01, f11));   // 右边
      if ((f10 < 0) !== (f11 < 0)) pts.push(cross(x0, y1, x1, y1, f10, f11));   // 上边
      if ((f00 < 0) !== (f10 < 0)) pts.push(cross(x0, y0, x0, y1, f00, f10));   // 左边
      if (pts.length === 4) {   // 鞍点: 用中心值决定配对
        const cen = (f00 + f01 + f10 + f11) / 4;
        const swap = (bits === 5 && cen >= 0) || (bits === 10 && cen < 0);
        if (swap) { const tmp = pts[1]; pts[1] = pts[3]; pts[3] = tmp; }
      }
      for (let k = 0; k + 1 < pts.length; k += 2)
        segs.push([pts[k][0], pts[k][1], pts[k + 1][0], pts[k + 1][1]]);
    }
  if (segs.length === 0) return result;

  // 端点哈希 (周期轴折回基域后量化) → O(1) 找邻接段
  const q = Math.max(1e-12, Math.min(dxg, dyg) * 0.02);
  const wrapX = x => (perX && Px > 0) ? x - Px * Math.floor((x - SCs[0]) / Px) : x;
  const wrapY = y => (perY && Py > 0) ? y - Py * Math.floor((y - SWs[0]) / Py) : y;
  const mk = (kx, ky) => kx * 1000003 + ky;
  const map = new Map();
  const put = (x, y, v) => {
    const k = mk(rint(wrapX(x) / q), rint(wrapY(y) / q));
    let l = map.get(k);
    if (l === undefined) { l = []; map.set(k, l); }
    l.push(v);
  };
  for (let i = 0; i < segs.length; i++) { put(segs[i][0], segs[i][1], 2 * i); put(segs[i][2], segs[i][3], 2 * i + 1); }
  const used = new Uint8Array(segs.length);
  const findNext = (x, y) => {
    const kx = rint(wrapX(x) / q), ky = rint(wrapY(y) / q);
    for (let ex = -1; ex <= 1; ex++)
      for (let ey = -1; ey <= 1; ey++) {
        const l = map.get(mk(kx + ex, ky + ey));
        if (l === undefined) continue;
        for (const v of l) if (!used[v >> 1]) return v;
      }
    return -1;
  };
  // 把段端点 (sx,sy) 平移到与 (x,y) 同一个展开分支
  const shiftTo = (x, y, sx, sy) => [
    (perX && Px > 0) ? Px * rint((x - sx) / Px) : 0,
    (perY && Py > 0) ? Py * rint((y - sy) / Py) : 0];
  for (let seed = 0; seed < segs.length; seed++) {
    if (used[seed]) continue;
    used[seed] = 1;
    const back = [[segs[seed][0], segs[seed][1]], [segs[seed][2], segs[seed][3]]];
    const front = [];          // 反序存放 (AddFirst)
    for (let dir = 0; dir < 2; dir++) {
      while (true) {
        const endPt = dir === 0 ? back[back.length - 1] : (front.length ? front[front.length - 1] : back[0]);
        const v = findNext(endPt[0], endPt[1]);
        if (v < 0) break;
        const si = v >> 1, end = v & 1;
        used[si] = 1;
        const s = segs[si];
        const ex0 = end === 0 ? s[0] : s[2], ey0 = end === 0 ? s[1] : s[3];
        const ox = end === 0 ? s[2] : s[0], oy = end === 0 ? s[3] : s[1];
        const sh = shiftTo(endPt[0], endPt[1], ex0, ey0);
        const np = [ox + sh[0], oy + sh[1]];
        if (dir === 0) back.push(np); else front.push(np);
      }
    }
    front.reverse();
    const cpts = front.concat(back);
    const ct = { pts: cpts, closed: false };
    if (cpts.length >= 3) {   // 首尾重合 (模周期) → 闭合: 补上首点
      const h = cpts[0], t = cpts[cpts.length - 1];
      const sh = shiftTo(t[0], t[1], h[0], h[1]);
      const hx = h[0] + sh[0], hy = h[1] + sh[1];
      if (Math.abs(hx - t[0]) < 3 * q && Math.abs(hy - t[1]) < 3 * q) { ct.closed = true; cpts.push([hx, hy]); }
    }
    result.push(ct);
  }
  if (evalF !== null) continueOpenEnds(result, SCs, SWs, perX, perY, evalF);
  return result;
}

// 场的噪声地板: 90 分位幅值 × 1e-6
export function noiseFloorOf(F, nW, nS) {
  const v = [];
  const sw = Math.max(1, Math.trunc(nW / 64)), ss = Math.max(1, Math.trunc(nS / 64));
  for (let i = 0; i < nW; i += sw)
    for (let j = 0; j < nS; j += ss) {
      const a = Math.abs(F[i * nS + j]);
      if (Number.isFinite(a)) v.push(a);
    }
  if (v.length === 0) return 0;
  v.sort((a, b) => a - b);
  const scale = v[Math.trunc(v.length * 0.90)];
  return Math.max(1e-12, 1e-6 * scale);
}

function wrap1(d, per, P) {
  if (per && P > 0) d -= P * rint(d / P);
  return d;
}

// 断头延拓: marching squares 会整段丢掉宽度小于一个网格胞元的"薄条". 对每个未闭合链端点做预测-校正续走,
// 用精确函数值 Newton 投影回 F=0; 碰到别的链端点就桥接合并, 回到自己另一端就闭合.
function continueOpenEnds(chains, SCs, SWs, perX, perY, evalF) {
  const nS = SCs.length, nW = SWs.length;
  if (nS < 2 || nW < 2) return;
  const Px = SCs[nS - 1] - SCs[0], Py = SWs[nW - 1] - SWs[0];
  const dxg = Px / (nS - 1), dyg = Py / (nW - 1);
  const step = 0.5 * Math.min(dxg, dyg);
  const eps = step * 0.05;
  const snap = Math.min(dxg, dyg) * 0.9;
  const maxSteps = 3 * Math.max(nS, nW);
  const xLo = SCs[0] - (perX ? Px : 2 * step), xHi = SCs[nS - 1] + (perX ? Px : 2 * step);
  const yLo = SWs[0] - (perY ? Py : 2 * step), yHi = SWs[nW - 1] + (perY ? Py : 2 * step);

  for (const c of chains) {
    if (c === null || c.closed || c.pts.length < 2) continue;
    for (let side = 0; side < 2 && !c.closed; side++) {
      for (let attempt = 0; attempt < 8 && !c.closed; attempt++) {
        let pts = c.pts;
        if (pts.length < 2) break;
        const pE = side === 0 ? pts[pts.length - 1] : pts[0];
        const pP = side === 0 ? pts[pts.length - 2] : pts[1];
        let tx = pE[0] - pP[0], ty = pE[1] - pP[1];
        const tn = Math.sqrt(tx * tx + ty * ty);
        if (!(tn > 1e-15)) break;
        tx /= tn; ty /= tn;
        let cx = pE[0], cy = pE[1];
        const ext = [];
        let closedNow = false, merged = null, mAtStart = false, mShX = 0, mShY = 0;
        for (let st = 0; st < maxSteps; st++) {
          const gx = (evalF(cx + eps, cy) - evalF(cx - eps, cy)) / (2 * eps);
          const gy = (evalF(cx, cy + eps) - evalF(cx, cy - eps)) / (2 * eps);
          const g2 = gx * gx + gy * gy;
          if (Number.isNaN(g2) || !(g2 > 1e-20)) break;
          const gl = Math.sqrt(g2);
          let ux = -gy / gl, uy = gx / gl;
          if (ux * tx + uy * ty < 0) { ux = -ux; uy = -uy; }
          let qx = cx + ux * step, qy = cy + uy * step;
          let ok = true;
          for (let it = 0; it < 4; it++) {
            const f = evalF(qx, qy);
            if (Number.isNaN(f)) { ok = false; break; }
            const hgx = (evalF(qx + eps, qy) - evalF(qx - eps, qy)) / (2 * eps);
            const hgy = (evalF(qx, qy + eps) - evalF(qx, qy - eps)) / (2 * eps);
            const hg2 = hgx * hgx + hgy * hgy;
            if (Number.isNaN(hg2) || !(hg2 > 1e-20)) { ok = false; break; }
            qx -= f * hgx / hg2; qy -= f * hgy / hg2;
            if (Math.abs(f) < 1e-12 * (1 + gl)) break;
          }
          if (!ok) break;
          const fq = evalF(qx, qy);
          if (Number.isNaN(fq) || Math.abs(fq) > 0.5 * step * gl) break;   // 没能投回曲线 (可能是真端点)
          const sx = qx - cx, sy = qy - cy;
          const sl = Math.sqrt(sx * sx + sy * sy);
          if (!(sl > 1e-15) || sl > 3 * step) break;                        // 停滞 / 跳飞
          if (sx * tx + sy * ty < 0) break;                                 // 折返 (尖点)
          cx = qx; cy = qy; tx = sx / sl; ty = sy / sl;
          if (cx < xLo || cx > xHi || cy < yLo || cy > yHi) break;
          ext.push([cx, cy]);
          const oth = side === 0 ? pts[0] : pts[pts.length - 1];
          const odx = wrap1(cx - oth[0], perX, Px), ody = wrap1(cy - oth[1], perY, Py);
          if (ext.length > 3 && Math.abs(odx) < snap && Math.abs(ody) < snap) {
            ext.push([cx - odx, cy - ody]);
            closedNow = true; break;
          }
          for (const o of chains) {
            if (o === null || o === c || o.closed || o.pts.length < 2) continue;
            for (let oe = 0; oe < 2; oe++) {
              const q2 = oe === 0 ? o.pts[0] : o.pts[o.pts.length - 1];
              const ddx = wrap1(cx - q2[0], perX, Px), ddy = wrap1(cy - q2[1], perY, Py);
              if (Math.abs(ddx) < snap && Math.abs(ddy) < snap) {
                merged = o; mAtStart = oe === 0;
                mShX = cx - ddx - q2[0]; mShY = cy - ddy - q2[1];
                break;
              }
            }
            if (merged !== null) break;
          }
          if (merged !== null) break;
        }
        if (side === 0) { for (const p3 of ext) pts.push(p3); }
        else { pts = ext.slice().reverse().concat(pts); c.pts = pts; }      // = 逐个 Insert(0, ext[k])
        if (closedNow) { c.closed = true; break; }
        if (merged === null) break;
        const mp = merged.pts, n2 = mp.length;
        const sh = k3 => [mp[k3][0] + mShX, mp[k3][1] + mShY];
        if (side === 0) {
          if (mAtStart) { for (let k3 = 0; k3 < n2; k3++) pts.push(sh(k3)); }
          else { for (let k3 = n2 - 1; k3 >= 0; k3--) pts.push(sh(k3)); }
        } else {
          const add = [];
          if (mAtStart) { for (let k3 = n2 - 1; k3 >= 0; k3--) add.push(sh(k3)); }   // 逐个 Insert(0, M[k3]) 的结果
          else { for (let k3 = 0; k3 < n2; k3++) add.push(sh(k3)); }
          pts = add.concat(pts); c.pts = pts;
        }
        merged.pts = [];     // 已并入, 置空待移除
      }
    }
  }
  for (let i = chains.length - 1; i >= 0; i--)
    if (chains[i] === null || chains[i].pts.length < 2) chains.splice(i, 1);
}
