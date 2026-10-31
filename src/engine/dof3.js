// 三自由度: 第三个取向角放开到一个区间 [lo, hi] (柱晶 ζ / 板晶摆动 / Lowitz roll). 姿态 3 维, 天空 2 维, 弧铺成一整片.
//
// 理论 (myown/experiments/dof3_range): 记晶体系里的太阳方向 w = Rᵀs, 绕太阳的转角 ψ. 光点链只依赖 w, 所以
//   天空位置 = (偏向角 δ(w), 绕太阳方位 β(w) + ψ),  光路成不成立也只依赖 w.
// 2×3 雅可比秩 < 2 ⇔ ∇δ = 0, 于是区域的边只可能是:
//   ① 两张端面 (第三个角 = lo / hi) 的 2DOF 边界与焦散 — 现成引擎;
//   ② 以太阳为心的圆弧, 半径 = δ 的临界值 (有效区内部驻点 / 沿边界的驻点 / 两条边界的交角), 只由光路决定.
// 这里:
//   (1) 在 w 球面上求临界点 (内部牛顿 + 每条边界预测-校正走一圈), 按局部极值判断它的圆弧能不能当边;
//   (2) 每个临界点绕太阳转 ψ, 落在区间里的那几段就是圆弧 (两头 = 接上端面曲线的切接点);
//   (3) 端面曲线 + 圆弧互相切段, 每段两侧各测一点 "这群晶体照不照得到" → 真正的外轮廓;
//       判断照不照得到: 在 w 球面上沿 δ(w) = δ_x 的等值线追, 看哪里需要的 ψ 落在区间里 (一维追踪, 不铺网格);
//   (4) 轮廓换曲线的地方 = 三自由度的末端点: 切接 (圆弧接端面曲线)、拐角 (两段边相交)、端面自己的 2DOF 末端点.

import { LP, OMode, orientEuler, chain3, newC3, valid3 } from './lp.js';
import * as LPm from './lp.js';
import { AP, apMarginW, apOk } from './aperture.js';
import { computeOri, computeEps, computeLines } from './pipeline.js';
import { zenEdgeSky } from './zenhalf.js';
import { fisheyeOf } from './lift.js';

const TAU = 2 * Math.PI, D2R = Math.PI / 180, R2D = 180 / Math.PI;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const nrm = a => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
const axpy = (a, b, s) => [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]];
const clamp1 = x => (x > 1 ? 1 : x < -1 ? -1 : x);
const wrap2 = x => x - TAU * Math.floor(x / TAU);
const wrapPi = x => x - TAU * Math.floor((x + Math.PI) / TAU);
function tbasis(w) {
  const a = Math.abs(w[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const t1 = nrm(cross(w, a));
  return [t1, cross(w, t1)];
}

// ============================================================ 取向族
/**
 * kind: 'col' = 柱晶族 (α、roll 全开, 第三个角 ζ ∈ [lo, hi]; 板晶摆动 = [0, 摆幅]);
 *       'low' = Lowitz 族 (α、ζ 全开, 第三个角 roll ∈ [lo, hi]; zenHalf 时 ζ 只取 0–180°).
 * 族内坐标 (x, y, z): 柱晶 (α, roll, ζ), Lowitz (α, ζ, roll). 返回角度一律弧度.
 */
export function famSpec(kind, loDeg, hiDeg, zenHalf) {
  let lo = Math.min(loDeg, hiDeg), hi = Math.max(loDeg, hiDeg);
  if (kind === 'col') { lo = Math.max(0, lo); hi = Math.min(180, hi); }
  else { lo = ((lo % 360) + 360) % 360; hi = lo + Math.min(360, Math.max(loDeg, hiDeg) - Math.min(loDeg, hiDeg)); }
  const F = { kind, lo: lo * D2R, hi: hi * D2R, zenHalf: kind === 'low' && !!zenHalf,
              om: kind === 'col' ? OMode.ZetaFixed : OMode.ZenFree };
  F.full = kind === 'low' && F.hi - F.lo >= TAU - 1e-9;
  // 端面: 柱晶 ζ = 0 / 180° 是 c 轴竖直, 在 SO(3) 里只是一个圆, 不是端面
  F.ends = [F.lo, F.hi].map(z => ({ z, face: !F.full && (kind === 'low' || (z > 1e-6 && z < Math.PI - 1e-6)) }));
  F.c = 0.5 * (F.lo + F.hi); F.half = 0.5 * (F.hi - F.lo);
  return F;
}

const _E = new Float64Array(9);
function eulerSol(e1, e3, flip) {
  let ze = Math.acos(clamp1(e3[2])), al = Math.atan2(e3[1], e3[0]);
  if (flip) { al += Math.PI; ze = -ze; }
  orientEuler(al, ze, 0, _E);
  const ro = Math.atan2(e1[0] * _E[3] + e1[1] * _E[4] + e1[2] * _E[5], e1[0] * _E[0] + e1[1] * _E[1] + e1[2] * _E[2]);
  return [al, ze, ro];
}

/** 姿态 (e1, e3 = 晶体 x 轴与 c 轴的世界方向) → 族内坐标 {x, y, z} 与区间裕度 m (≥ 0 在族里, 连续). */
export function boxOf(F, e1, e3) {
  if (F.kind === 'col') {
    const [al, ze, ro] = eulerSol(e1, e3, false);
    return { x: wrap2(al), y: wrap2(ro), z: ze, m: Math.min(ze - F.lo, F.hi - ze) };
  }
  let best = null;
  for (let s = 0; s < (F.zenHalf ? 1 : 2); s++) {      // (α, ζ) 与 (α + π, −ζ) 是同一个姿态 (ζ 整圈双覆盖)
    const [al, ze, ro] = eulerSol(e1, e3, s === 1);
    const m = F.full ? 1 : F.half - Math.abs(wrapPi(ro - F.c));
    if (!best || m > best.m) best = { x: wrap2(al), y: wrap2(ze), z: F.lo + wrap2(ro - F.lo), m };
  }
  return best;
}

/** 同一个姿态的各组族内坐标 (柱晶 1 组; Lowitz 的 ζ 双覆盖 2 组, ζ 半圈 1 组) */
function boxAll(F, e1, e3) {
  if (F.kind === 'col') return [boxOf(F, e1, e3)];
  const out = [];
  for (let s = 0; s < (F.zenHalf ? 1 : 2); s++) {
    const [al, ze, ro] = eulerSol(e1, e3, s === 1);
    out.push({ x: wrap2(al), y: wrap2(ze), z: F.lo + wrap2(ro - F.lo + Math.PI) - Math.PI, m: F.full ? 1 : F.half - Math.abs(wrapPi(ro - F.c)) });
  }
  return out;
}

// ============================================================ 晶体系里的光点链
function chainW(P, w) {
  const cf = P.coef, n = P.faces.length, K = LPm.K;
  let ux = cf[0], uy = cf[1], uz = cf[2];
  const g0 = w[0] * ux + w[1] * uy + w[2] * uz;
  const t0 = -g0 + Math.sqrt(g0 * g0 + K);
  let Px = w[0] + t0 * ux, Py = w[1] + t0 * uy, Pz = w[2] + t0 * uz;
  const qs = [];
  for (let i = 1; i < n - 1; i++) {
    ux = cf[3 * i]; uy = cf[3 * i + 1]; uz = cf[3 * i + 2];
    const q = Px * ux + Py * uy + Pz * uz;
    Px -= 2 * q * ux; Py -= 2 * q * uy; Pz -= 2 * q * uz; qs.push(q);
  }
  const j = 3 * (n - 1);
  ux = cf[j]; uy = cf[j + 1]; uz = cf[j + 2];
  const qE = Px * ux + Py * uy + Pz * uz;
  const tE = -qE - Math.sqrt(Math.max(qE * qE - K, 0));
  return { g0, qs, qE, D: nrm([Px + tE * ux, Py + tE * uy, Pz + tE * uz]) };
}
/** 偏向角 δ(w), 不分配内存 (等值线追踪里每步要算好几次) */
function devW(P, w) {
  const cf = P.coef, n = P.faces.length, K = LPm.K, w0 = w[0], w1 = w[1], w2 = w[2];
  let ux = cf[0], uy = cf[1], uz = cf[2];
  const g0 = w0 * ux + w1 * uy + w2 * uz, t0 = -g0 + Math.sqrt(g0 * g0 + K);
  let Px = w0 + t0 * ux, Py = w1 + t0 * uy, Pz = w2 + t0 * uz;
  for (let i = 1; i < n - 1; i++) {
    ux = cf[3 * i]; uy = cf[3 * i + 1]; uz = cf[3 * i + 2];
    const q = Px * ux + Py * uy + Pz * uz;
    Px -= 2 * q * ux; Py -= 2 * q * uy; Pz -= 2 * q * uz;
  }
  const j = 3 * (n - 1);
  ux = cf[j]; uy = cf[j + 1]; uz = cf[j + 2];
  const qE = Px * ux + Py * uy + Pz * uz, tE = -qE - Math.sqrt(Math.max(qE * qE - K, 0));
  const Dx = Px + tE * ux, Dy = Py + tE * uy, Dz = Pz + tE * uz, dn = Math.sqrt(Dx * Dx + Dy * Dy + Dz * Dz) || 1;
  return Math.acos(clamp1((w0 * Dx + w1 * Dy + w2 * Dz) / dn));
}

/** 各条边界的判据 (≥ 0 成立, 同 lift.js margin) 与它们的名字 / 种类 (与 lp.js 的族一致). */
function fenceMeta(P) {
  const out = [{ name: 'g0=0 入射掠射', kind: 0, j: 0 }];
  for (let j = 0; j < P.faces.length - 2; j++) {
    const f = P.faces[j + 1];
    out.push({ name: `q${j + 1}(面${f})=0 退化`, kind: 1, j });
    if (!LP.partial) out.push({ name: `q${j + 1}(面${f})=-√K TIR`, kind: 2, j });
  }
  out.push({ name: '出射临界', kind: 3, j: 0 });
  if (AP.on && P.apSupported) out.push({ name: '孔径篱笆(真实晶体)', kind: 7, j: 0 });
  return out;
}
function marginsW(P, w, c = chainW(P, w)) {
  const SqK = LPm.SqK, m = [c.g0];
  for (const q of c.qs) { m.push(-q); if (!LP.partial) m.push(q + SqK); }
  m.push(-SqK - c.qE);
  if (AP.on && P.apSupported) { const a = apMarginW(P.faces, w[0], w[1], w[2]); m.push(Number.isFinite(a) ? a : 1); }
  return m;
}
const okMargins = (m, tol) => m.every(v => v >= -tol);

// ============================================================ 太阳系标架与 R(w, ψ)
// x̂ = 太阳处朝天顶, ŷ = S × x̂. 绕太阳方位 φ 从 x̂ 量向 ŷ; 天空点 = cos δ S + sin δ (cos φ x̂ + sin φ ŷ).
function sunFrame() {
  const S = LP.A3.slice(), z = [0, 0, 1];
  let x = axpy(z, S, -dot(z, S));
  if (Math.hypot(...x) < 1e-9) x = [1, 0, 0];
  x = nrm(x);
  return { S, x, y: cross(S, x) };
}
/** w 处的规范: p1, p2 ⊥ w; F = 晶体系出射方向; β = F 在 (p1, p2) 里的方位; δ = 偏向角 */
function frameAt(P, w, F = chainW(P, w).D) {
  const ref = Math.abs(w[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const p1 = nrm(axpy(ref, w, -dot(ref, w))), p2 = cross(w, p1);
  return { w, p1, p2, F, beta: Math.atan2(dot(F, p2), dot(F, p1)), d: Math.acos(clamp1(dot(w, F))) };
}
/** R = [S X(ψ) Y(ψ)]·[w p1 p2]ᵀ, X = cosψ x̂ + sinψ ŷ, Y = S × X. 返回晶体系向量 → 世界的映射. 天空方位 φ = β + ψ. */
function rotOf(SF, fr, psi) {
  const c = Math.cos(psi), s = Math.sin(psi);
  const X = [c * SF.x[0] + s * SF.y[0], c * SF.x[1] + s * SF.y[1], c * SF.x[2] + s * SF.y[2]];
  const Y = [c * SF.y[0] - s * SF.x[0], c * SF.y[1] - s * SF.x[1], c * SF.y[2] - s * SF.x[2]];
  const S = SF.S, w = fr.w, p1 = fr.p1, p2 = fr.p2;
  return v => {
    const a = dot(v, w), b = dot(v, p1), d = dot(v, p2);
    return [a * S[0] + b * X[0] + d * Y[0], a * S[1] + b * X[1] + d * Y[1], a * S[2] + b * X[2] + d * Y[2]];
  };
}
const E1 = [1, 0, 0], E3 = [0, 0, 1];
function famAt(F, SF, fr, psi) { const R = rotOf(SF, fr, psi); return boxOf(F, R(E1), R(E3)); }
/** 天空方向 → 以太阳为中心的平面 (度): (δ cos φ, δ sin φ). 只用来求交与取法向 */
function planeOf(SF, D) {
  const d = Math.acos(clamp1(dot(D, SF.S))) * R2D, ph = Math.atan2(dot(D, SF.y), dot(D, SF.x));
  return [d * Math.cos(ph), d * Math.sin(ph)];
}
function dirOfPlane(SF, u, v) {
  const d = Math.hypot(u, v) * D2R, ph = Math.atan2(v, u), s = Math.sin(d), c = Math.cos(d);
  const cp = Math.cos(ph), sp = Math.sin(ph);
  return [c * SF.S[0] + s * (cp * SF.x[0] + sp * SF.y[0]), c * SF.S[1] + s * (cp * SF.x[1] + sp * SF.y[1]), c * SF.S[2] + s * (cp * SF.x[2] + sp * SF.y[2])];
}

// ============================================================ (1) δ 的临界点
/** 球面上沿切平面的数值梯度 (f 是 w 的标量函数) */
function sgrad(f, w, e = 1e-6) {
  const [t1, t2] = tbasis(w);
  const g1 = (f(nrm(axpy(w, t1, e))) - f(nrm(axpy(w, t1, -e)))) / (2 * e);
  const g2 = (f(nrm(axpy(w, t2, e))) - f(nrm(axpy(w, t2, -e)))) / (2 * e);
  return [g1 * t1[0] + g2 * t2[0], g1 * t1[1] + g2 * t2[1], g1 * t1[2] + g2 * t2[2]];
}

function interiorCrit(P) {
  const found = [], N = 3000;
  for (let i = 0; i < N; i++) {
    const z = 1 - 2 * (i + 0.5) / N, r = Math.sqrt(1 - z * z), ph = i * 2.399963229728653;
    let w = [r * Math.cos(ph), r * Math.sin(ph), z];
    if (!okMargins(marginsW(P, w), 1e-3)) continue;
    let conv = false, H = null;
    for (let it = 0; it < 60; it++) {
      const [t1, t2] = tbasis(w), e = 1e-5;
      const f = (x, y) => devW(P, nrm([w[0] + x * t1[0] + y * t2[0], w[1] + x * t1[1] + y * t2[1], w[2] + x * t1[2] + y * t2[2]]));
      const f0 = f(0, 0), fpx = f(e, 0), fmx = f(-e, 0), fpy = f(0, e), fmy = f(0, -e);
      const fx = (fpx - fmx) / (2 * e), fy = (fpy - fmy) / (2 * e);
      const fxx = (fpx - 2 * f0 + fmx) / (e * e), fyy = (fpy - 2 * f0 + fmy) / (e * e);
      const fxy = (f(e, e) - f(e, -e) - f(-e, e) + f(-e, -e)) / (4 * e * e);
      const det = fxx * fyy - fxy * fxy;
      H = [fxx, fxy, fyy];
      if (!Number.isFinite(det) || Math.abs(det) < 1e-12) break;
      let dx = -(fyy * fx - fxy * fy) / det, dy = -(-fxy * fx + fxx * fy) / det;
      const st = Math.hypot(dx, dy);
      if (st > 0.05) { dx *= 0.05 / st; dy *= 0.05 / st; }
      w = nrm([w[0] + dx * t1[0] + dy * t2[0], w[1] + dx * t1[1] + dy * t2[1], w[2] + dx * t1[2] + dy * t2[2]]);
      if (Math.hypot(fx, fy) < 1e-7 && st < 1e-10) { conv = true; break; }
    }
    if (!conv || !okMargins(marginsW(P, w), 1e-9)) continue;
    if (found.some(q => dot(q.w, w) > 1 - 1e-10) || found.length >= 24) continue;
    const tr = H[0] + H[2], det = H[0] * H[2] - H[1] * H[1];
    const cls = det > 0 ? (tr > 0 ? 'lo' : 'hi') : 'none';         // 极小: 区域在 δ ≥ δ* 一侧; 极大反之; 鞍点不当边
    found.push({ w, d: devW(P, w), src: 'interior', cls });
  }
  return found;
}

/**
 * 每条边界 (判据 fi = 0) 在 w 球面上是闭曲线: 预测-校正走一圈. 返回 { fences: [{fi, name, kind, pts, ok, d}], crit }.
 * crit: 沿边界 δ 的驻点 (只在其余判据都成立的段上) 和有效性翻转处 (两条边界的交角), 带 cls.
 */
function traceFences(P) {
  const meta = fenceMeta(P), nf = meta.length, h = 0.002;
  const mf = (fi, w) => marginsW(P, w)[fi];
  const grad = (fi, w) => sgrad(x => mf(fi, x), w);
  const proj = (fi, w) => {
    for (let it = 0; it < 6; it++) {
      const f = mf(fi, w), g = grad(fi, w), gg = dot(g, g);
      if (!(gg > 0)) break;
      w = nrm(axpy(w, g, -f / gg));
      if (Math.abs(f) < 1e-14) break;
    }
    return w;
  };
  const othersOk = (fi, m, tol) => { for (let i = 0; i < nf; i++) if (i !== fi && !(m[i] >= -tol)) return false; return true; };
  // 种子: 72 条大圆 × 2000 点, 一次算出全部判据
  const seeds = Array.from({ length: nf }, () => []);
  for (let gc = 0; gc < 72; gc++) {
    const th = (gc % 36) / 36 * Math.PI;
    const A = gc < 36 ? [Math.cos(th), Math.sin(th), 0] : [0, Math.cos(th), Math.sin(th)], B = gc < 36 ? [0, 0, 1] : [1, 0, 0];
    const W = t => [Math.cos(t) * A[0] + Math.sin(t) * B[0], Math.cos(t) * A[1] + Math.sin(t) * B[1], Math.cos(t) * A[2] + Math.sin(t) * B[2]];
    let mp = marginsW(P, W(0));
    for (let i = 1; i <= 2000; i++) {
      const t = i / 2000 * TAU, m = marginsW(P, W(t));
      for (let fi = 0; fi < nf; fi++) {
        if (mp[fi] * m[fi] >= 0) continue;
        let ta = (i - 1) / 2000 * TAU, tb = t, fa = mp[fi];
        for (let k = 0; k < 50; k++) { const tm = 0.5 * (ta + tb), fm = mf(fi, W(tm)); if (fa * fm <= 0) tb = tm; else { ta = tm; fa = fm; } }
        seeds[fi].push(W(0.5 * (ta + tb)));
      }
      mp = m;
    }
  }
  const fences = [], crit = [];
  for (let fi = 0; fi < nf; fi++) {
    const traced = [];
    for (const sd of seeds[fi]) {
      if (traced.some(T => T.some(p => dot(p, sd) > Math.cos(4 * h)))) continue;
      const pts = [proj(fi, sd)];
      let w = pts[0];
      let closed = false;
      for (let st = 0; st < 20000; st++) {
        const t = nrm(cross(w, grad(fi, w)));
        w = proj(fi, nrm(axpy(w, t, h)));
        pts.push(w);
        if (st > 20 && dot(w, pts[0]) > Math.cos(0.75 * h)) { closed = true; break; }
      }
      traced.push(pts);
      const n = pts.length, M = pts.map(p => marginsW(P, p));
      const ok = M.map(m => othersOk(fi, m, 1e-12)), d = pts.map(p => devW(P, p));
      fences.push({ fi, name: meta[fi].name, kind: meta[fi].kind, pts, ok, d });
      if (!closed) continue;                          // 走不回起点: 判据在退化处没有好的梯度, 只拿来当隶属测试的种子
      const dv = d.filter((_, i) => ok[i]);
      if (dv.length && Math.max(...dv) - Math.min(...dv) < 1e-7) {
        const okIdx = d.map((_, i) => i).filter(i => ok[i]);
        for (let k = 0; k < 48; k++) {
          const i = okIdx[Math.floor((k + 0.5) / 48 * okIdx.length)], wv = pts[i];
          const g = grad(fi, wv), dIn = devW(P, nrm(axpy(wv, nrm(g), 1e-5))) - d[i];
          crit.push({ w: wv, d: d[i], src: 'fence', fi, name: meta[fi].name, kind: meta[fi].kind, ext: 'flat', cls: dIn < -1e-12 ? 'hi' : dIn > 1e-12 ? 'lo' : 'none' });
        }
        continue;
      }
      for (let i = 0; i < n - 1; i++) {
        const i0 = (i - 1 + n - 1) % (n - 1), i2 = i + 1;
        if (ok[i0] && ok[i] && ok[i2]) {
          const s1 = d[i] - d[i0], s2 = d[i2] - d[i];
          if (s1 * s2 < 0 && Math.min(Math.abs(s1), Math.abs(s2)) > 1e-13) {    // 驻点: 抛物线顶点, 投回边界
            const den = d[i0] - 2 * d[i] + d[i2], tv = den !== 0 ? 0.5 * (d[i0] - d[i2]) / den : 0;
            const nb = tv >= 0 ? pts[i2] : pts[i0], f = Math.abs(tv);
            const wv = proj(fi, nrm([pts[i][0] * (1 - f) + nb[0] * f, pts[i][1] * (1 - f) + nb[1] * f, pts[i][2] * (1 - f) + nb[2] * f]));
            const dv = devW(P, wv);
            // 往有效一侧走一小步, δ 怎么变: 沿边界极大且往里变小 → 局部极大 (外缘); 沿边界极小且往里变大 → 局部极小 (内缘)
            const g = grad(fi, wv), dIn = devW(P, nrm(axpy(wv, nrm(g), 1e-5))) - dv;
            const isMax = s1 > 0;
            const cls = isMax && dIn < 0 ? 'hi' : !isMax && dIn > 0 ? 'lo' : 'none';
            crit.push({ w: wv, d: dv, src: 'fence', fi, name: meta[fi].name, kind: meta[fi].kind, ext: isMax ? 'max' : 'min', cls });
          }
        }
        if (ok[i] !== ok[i2] && Math.min(...(ok[i] ? M[i2] : M[i]).filter((v, k) => k !== fi)) < -1e-9) {   // 交角: 沿弦二分 (每步投回边界)
          let wa = ok[i] ? pts[i] : pts[i2], wb = ok[i] ? pts[i2] : pts[i];
          for (let k = 0; k < 50; k++) {
            const wm = proj(fi, nrm([(wa[0] + wb[0]) / 2, (wa[1] + wb[1]) / 2, (wa[2] + wb[2]) / 2]));
            if (othersOk(fi, marginsW(P, wm), 1e-12)) wa = wm; else wb = wm;
          }
          const mc = marginsW(P, wb);
          let fj = -1, mv = Infinity;
          for (let k = 0; k < nf; k++) if (k !== fi && mc[k] < mv) { mv = mc[k]; fj = k; }
          crit.push({ w: wa, d: devW(P, wa), src: 'corner', fi, fj, name: meta[fi].name + '∩' + (meta[fj] ? meta[fj].name : '?'),
                      kind: meta[fi].kind, tIn: ok[i] ? nrm(axpy(pts[i], pts[i2], -1)) : nrm(axpy(pts[i2], pts[i], -1)) });
        }
      }
    }
  }
  // 交角的 cls: 沿两条边界都往有效一侧走, δ 都变大 → 局部极小; 都变小 → 局部极大
  const corners = [];
  for (const c of crit.filter(q => q.src === 'corner')) {
    if (corners.some(q => dot(q.w, c.w) > 1 - 1e-12)) continue;
    const dv = c.d, e = 1e-4;
    const di = devW(P, proj(c.fi, nrm(axpy(c.w, c.tIn, e)))) - dv;
    let tj = nrm(cross(c.w, grad(c.fj, c.w)));
    if (dot(grad(c.fi, c.w), tj) < 0) tj = tj.map(v => -v);
    const dj = c.fj >= 0 ? devW(P, proj(c.fj, nrm(axpy(c.w, tj, e)))) - dv : 0;
    c.cls = di > 0 && dj > 0 ? 'lo' : di < 0 && dj < 0 ? 'hi' : 'none';
    delete c.tIn;
    corners.push(c);
  }
  return { fences, crit: [...crit.filter(q => q.src === 'fence'), ...corners], proj, meta };
}

// ============================================================ (2) 圆弧: 临界点绕太阳转 ψ, 落在区间里的段
function arcsOf(P, F, SF, crit) {
  const arcs = [], NPS = 1440;
  for (let ci = 0; ci < crit.length; ci++) {
    const c = crit[ci], fr = frameAt(P, c.w);
    const m = psi => famAt(F, SF, fr, psi).m;
    const ms = new Float64Array(NPS);
    for (let i = 0; i < NPS; i++) ms[i] = m(i / NPS * TAU);
    const inside = i => ms[((i % NPS) + NPS) % NPS] >= 0;
    if (![...ms].some(v => v >= 0)) continue;
    const bis = (ia, ib) => {                       // ia 在里, ib 在外 → 边界 ψ
      let a = ia / NPS * TAU, b = ib / NPS * TAU;
      for (let k = 0; k < 60; k++) { const mid = 0.5 * (a + b); if (m(mid) >= 0) a = mid; else b = mid; }
      return a;
    };
    const runs = [];
    if ([...ms].every(v => v >= 0)) runs.push([0, TAU, true]);
    else {
      let s = 0; while (inside(s)) s++;             // 从一个外点开始转一圈
      for (let k = 1; k <= NPS; k++) {
        const i = s + k;
        if (inside(i) && !inside(i - 1)) runs.push([bis(i, i - 1), null, false]);
        if (!inside(i) && inside(i - 1) && runs.length) runs[runs.length - 1][1] = bis(i - 1, i);
      }
    }
    for (const [a0, a1raw, full] of runs) {
      let a1 = a1raw;
      if (a1 === null) continue;
      if (a1 < a0) a1 += TAU;
      const n = Math.max(2, Math.ceil((a1 - a0) / (0.25 * D2R)));
      const D = [], box = [], psi = [];
      for (let k = 0; k <= n; k++) {
        const ps = a0 + (a1 - a0) * k / n, R = rotOf(SF, fr, ps), b = boxOf(F, R(E1), R(E3));
        D.push(nrm(R(fr.F))); box.push([b.x, b.y, b.z]); psi.push(ps);
      }
      arcs.push({ ci, d: c.d, cls: c.cls, src: c.src, name: c.name || '', D, box, psi, full });
    }
  }
  return arcs;
}

// ============================================================ (3) 照不照得到: 沿 δ = δ_x 的等值线追
// 天空点 x 被照到 ⇔ 有效区里有一点 w, δ(w) = δ_x, 且把它转到 x 所需的 ψ = φ_x − β(w) 落在区间里.
// 等值线与有效区的每个交段, 要么两头搭在边界上 (从边界曲线上 δ 穿过 δ_x 的地方出发), 要么是围着内部极值点的闭圈.
function makeMember(P, F, SF, fences, interior) {
  const h = 0.006, e = 1e-6;
  const dev = w => devW(P, w);
  const gradAt = (w, d0) => {
    const [t1, t2] = tbasis(w);
    const g1 = (dev(nrm(axpy(w, t1, e))) - d0) / e, g2 = (dev(nrm(axpy(w, t2, e))) - d0) / e;
    return [g1 * t1[0] + g2 * t2[0], g1 * t1[1] + g2 * t2[1], g1 * t1[2] + g2 * t2[2]];
  };
  const member = function (D, hint) {
    const dx = Math.acos(clamp1(dot(D, SF.S))), phx = Math.atan2(dot(D, SF.y), dot(D, SF.x));
    const winC = (w, c) => { const fr = frameAt(P, w, c.D); return famAt(F, SF, fr, phx - fr.beta).m; };
    const seeds = [];
    if (hint && okMargins(marginsW(P, hint), 1e-9)) seeds.push({ w: hint, hint: true });
    for (const fc of fences) {
      const { pts, ok, d } = fc;
      for (let i = 0; i + 1 < pts.length; i++) {
        if ((!ok[i] && !ok[i + 1]) || (d[i] - dx) * (d[i + 1] - dx) > 0 || d[i] === d[i + 1]) continue;
        const t = (dx - d[i]) / (d[i + 1] - d[i]);
        const w = nrm(axpy(pts[i], axpy(pts[i + 1], pts[i], -1), t));
        // 一头已出有效区 (紧挨两条边界的交角): 交点本身成立才算 — 有效区尖端里的等值线很短, 只能从这里进
        if ((!ok[i] || !ok[i + 1]) && !okMargins(marginsW(P, w).map((v, k) => (k === fc.fi ? Math.abs(v) : v)), 1e-9)) continue;
        seeds.push({ w, fi: fc.fi });
      }
    }
    for (const c of interior) {                       // 有效区内部的闭合等值线一定围着一个极值点
      if (c.cls === 'none' || (c.cls === 'lo' && dx < c.d) || (c.cls === 'hi' && dx > c.d)) continue;
      const [t1, t2] = tbasis(c.w);
      for (let r = 0; r < 12; r++) {
        const th = r / 12 * TAU, t = axpy(t1.map(v => v * Math.cos(th)), t2, Math.sin(th));
        const W = s => axpy(c.w.map(v => v * Math.cos(s)), t, Math.sin(s));
        let sp = 0, fp = c.d - dx;
        for (let s = 0.01; s < Math.PI; s += 0.01) {
          const f = dev(W(s)) - dx;
          if (fp * f <= 0) {
            const w = W(sp + (s - sp) * fp / (fp - f));
            if (okMargins(marginsW(P, w), 1e-9)) seeds.push({ w });
            break;
          }
          sp = s; fp = f;
        }
      }
    }
    const done = new Map(), Q = 0.02;
    const near = w => {
      const k0 = [Math.round(w[0] / Q), Math.round(w[1] / Q), Math.round(w[2] / Q)];
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
        const L = done.get(`${k0[0] + a},${k0[1] + b},${k0[2] + c}`);
        if (L && L.some(p => dot(p, w) > Math.cos(2.5 * h))) return true;
      }
      return false;
    };
    const mark = w => { const k = `${Math.round(w[0] / Q)},${Math.round(w[1] / Q)},${Math.round(w[2] / Q)}`; const L = done.get(k); if (L) L.push(w); else done.set(k, [w]); };
    // 窗口裕度在追线上的局部极大附近: 黄金分割精修 (弦上取点, 投回等值线)
    const refine = (a, c) => {
      const at = t => {
        let w = nrm(axpy(a, axpy(c, a, -1), t));
        for (let it = 0; it < 3; it++) { const d0 = dev(w), g = gradAt(w, d0), gg = dot(g, g); if (gg > 0) w = nrm(axpy(w, g, -(d0 - dx) / gg)); }
        const ch = chainW(P, w);
        if (Math.abs(Math.acos(clamp1(dot(w, ch.D))) - dx) > 1e-7) return -Infinity;
        return okMargins(marginsW(P, w, ch), 1e-9) ? winC(w, ch) : -Infinity;
      };
      const g = (Math.sqrt(5) - 1) / 2;
      let lo = 0, hi = 1, x1 = hi - g, x2 = g, f1 = at(x1), f2 = at(x2);
      for (let k = 0; k < 24; k++) {
        if (f1 >= 0 || f2 >= 0) return 1;
        if (f1 > f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - g * (hi - lo); f1 = at(x1); }
        else { lo = x1; x1 = x2; f1 = f2; x2 = lo + g * (hi - lo); f2 = at(x2); }
      }
      return Math.max(f1, f2);
    };
    let budget = 60000;                                // 退化光路 (δ 沿整条线不变) 的保险
    const step = Math.max(1, Math.ceil(seeds.length / 800));
    for (let si = 0; si < seeds.length; si += step) {
      const sd = seeds[si];
      if (near(sd.w)) continue;
      let w0 = sd.w, d0 = dev(w0);
      if (sd.hint) {                                  // 提示点: 先牛顿落到等值线上, 落不上 (太远) 就不用
        for (let it = 0; it < 4; it++) { const g = gradAt(w0, d0), gg = dot(g, g); if (!(gg > 1e-8)) break; const st = -(d0 - dx) / gg; if (Math.abs(st) * Math.sqrt(gg) > 0.05) break; w0 = nrm(axpy(w0, g, st)); d0 = dev(w0); }
        if (Math.abs(d0 - dx) > 1e-7 || !okMargins(marginsW(P, w0), 1e-9)) continue;
      } else {
        for (let it = 0; it < 3 && Math.abs(d0 - dx) > 1e-12; it++) { const g = gradAt(w0, d0), gg = dot(g, g); if (!(gg > 1e-12)) break; w0 = nrm(axpy(w0, g, -(d0 - dx) / gg)); d0 = dev(w0); }
        if (Math.abs(d0 - dx) > 1e-7) continue;
      }
      let dirs = [1, -1];
      if (sd.fi !== undefined) {                      // 从边界出发: 只往有效一侧走
        const t = nrm(cross(w0, gradAt(w0, d0)));
        dirs = [marginsW(P, nrm(axpy(w0, t, h)))[sd.fi] >= marginsW(P, nrm(axpy(w0, t, -h)))[sd.fi] ? 1 : -1];
      }
      for (const dir of dirs) {
        let w = w0, d = d0, c0 = chainW(P, w);
        let f = winC(w, c0), wP = null, fP = -Infinity, wPP = null, fPP = -Infinity;
        if (f >= 0) { member.last = { w, why: sd.hint ? 'hint' : sd.fi !== undefined ? 'fence' + sd.fi : 'interior', at: 'seed' }; return true; }
        mark(w);
        for (let st = 0; st < 3000; st++) {
          if (--budget < 0) return false;
          const g = gradAt(w, d), gg = dot(g, g);
          if (!(gg > 1e-12)) break;
          const t = nrm(cross(w, g));
          // 预测一步, 牛顿两次落回等值线; 落不回 (出射临界附近 δ ∝ √裕度, 梯度很陡) 就减半步长重来, 还不行就停
          let wn = null, cn = null, dn = 0;
          for (let hh = h, tries = 0; tries < 5 && !wn; tries++, hh /= 2) {
            let q = nrm(axpy(w, t, dir * hh));
            for (let it = 0; it < 3; it++) { const dq = dev(q), gq = it === 0 ? g : gradAt(q, dq), gqq = dot(gq, gq); if (!(gqq > 1e-12)) break; q = nrm(axpy(q, gq, -(dq - dx) / gqq)); }
            const cq = chainW(P, q), dq = Math.acos(clamp1(dot(q, cq.D)));
            if (Math.abs(dq - dx) < 1e-7 && Math.acos(clamp1(dot(q, w))) < 3 * hh) { wn = q; cn = cq; dn = dq; }
          }
          if (!wn) break;
          if (!okMargins(marginsW(P, wn, cn), 1e-9)) break;
          const fn = winC(wn, cn);
          if (fn >= 0) { member.last = { w: wn, why: sd.hint ? 'hint' : sd.fi !== undefined ? 'fence' + sd.fi : 'interior', at: 'step' + st }; return true; }
          if (wP && fP > fPP && fP > fn && fP > -0.05 && refine(wPP || wP, wn) >= 0) return true;
          wPP = wP; fPP = fP; wP = w; fP = f;
          w = wn; d = dn; f = fn;
          mark(w);
          if (st > 10 && dot(w, w0) > Math.cos(0.75 * h)) break;
        }
      }
    }
    return false;
  };
  return member;
}

// ============================================================ 主入口
/**
 * P: PathCtx; F: famSpec(...). 全局配置 (setConfig) 已设好.
 * 返回 { slices, crit, arcs, segs, eps, ms } — 天空里的东西都是单位方向 D (世界系).
 * opts.skipCaustic: 端面不追踪焦散 (整块压成一条线的退化光路)
 */
export function analyzeDof3(P, F, opts = {}) {
  const t0 = performance.now();
  const log = opts.log || (() => {});
  const SF = sunFrame();
  // ---- 端面: 现成 2DOF 引擎
  const slices = opts.slices || F.ends.map(e => {          // 端面可由调用方先算好传进来 (Worker 里要顺便把取向平面发给界面)
    if (!e.face) return { z: e.z, face: false };
    const fx = e.z, ori = computeOri(P, F.om, fx);
    const pr = { P, faces: P.faces, name: P.name, ori };
    pr.eps = computeEps(pr, F.om, fx);
    pr.lines = computeLines(pr, F.om, fx, opts.skipCaustic ? new Set(ori.fams.filter(f => f.kind === 6).map(f => f.name)) : null);
    return { z: e.z, face: true, fx, pr };
  });
  const tSlices = performance.now();
  log('slices', tSlices - t0);
  // ---- 临界点与圆弧
  const inner = interiorCrit(P);
  log('interior', performance.now() - tSlices, inner.length);
  const tf = traceFences(P);
  {
    const byD = new Map(), keep = [];
    for (const c of tf.crit) { const k = Math.round(c.d * 1e6); const n = byD.get(k) || 0; if (n < 4 && keep.length < 300) { keep.push(c); byD.set(k, n + 1); } }
    tf.crit = keep;
  }
  log('fences', performance.now() - tSlices, tf.fences.length, tf.fences.map(f => f.pts.length).join('/'));
  const crit = [...inner, ...tf.crit];
  const arcs = arcsOf(P, F, SF, crit);
  const tCrit = performance.now();
  log('arcs', tCrit - tSlices, arcs.length);

  // ---- 候选曲线: 端面曲线 (成立段) + 能当边的圆弧 (+ Lowitz 半圈时 c 轴竖直的那条板晶弧)
  const raw = [];
  slices.forEach((sl, si) => {
    if (!sl.face) return;
    const famOf = new Map(sl.pr.ori.fams.map(f => [f.name, f]));
    for (const [name, fl] of sl.pr.lines) {
      const fm = famOf.get(name);
      if (!fm || (LP.partial && fm.kind === 2)) continue;       // 蓝边只是亮度台阶, 不是边
      const pars = [...fl.par, ...fl.conn.par], skys = [...fl.sky, ...fl.conn.sky];
      skys.forEach((sky, li) => {
        const par = pars[li], n = sky.length / 4;
        let run = null;
        const flush = () => { if (run && run.D.length >= 2) raw.push(run); run = null; };
        for (let i = 0; i < n; i++) {
          if (!(sky[4 * i + 3] > 0.5)) { flush(); continue; }
          if (!run) run = { src: 'slice', si, fam: name, kind: fm.kind, D: [], box: [] };
          run.D.push(nrm([sky[4 * i], sky[4 * i + 1], sky[4 * i + 2]]));
          run.box.push([wrap2(par[2 * i]), wrap2(par[2 * i + 1]), sl.z]);
        }
        flush();
      });
    }
  });
  // 去重叠: 对称的配置 (如柱晶 ζ = 90° 左右对称) 里同一条天空曲线会被两段取向曲线各画一遍. 重叠 (贴着且平行) 的部分只留先到的一份
  const cands = [];
  {
    const OV = 0.004, OC = 0.05, og = new Map();
    const cellsOf = (x0, y0, x1, y1, f) => {
      for (let gx = Math.floor((Math.min(x0, x1) - OV) / OC); gx <= Math.floor((Math.max(x0, x1) + OV) / OC); gx++)
        for (let gy = Math.floor((Math.min(y0, y1) - OV) / OC); gy <= Math.floor((Math.max(y0, y1) + OV) / OC); gy++) f(gx + ',' + gy);
    };
    const add = (A, B) => {
      if (Math.hypot(B[0] - A[0], B[1] - A[1]) > 3) return;
      cellsOf(A[0], A[1], B[0], B[1], k => { let L = og.get(k); if (!L) og.set(k, L = []); L.push([A, B]); });
    };
    const hit = (p, tx, ty) => {
      const L = og.get(Math.floor(p[0] / OC) + ',' + Math.floor(p[1] / OC));
      if (!L) return false;
      for (const [A, B] of L) {
        const ux = B[0] - A[0], uy = B[1] - A[1], l2 = ux * ux + uy * uy;
        if (!(l2 > 0)) continue;
        const t = Math.max(0, Math.min(1, ((p[0] - A[0]) * ux + (p[1] - A[1]) * uy) / l2));
        if (Math.hypot(A[0] + t * ux - p[0], A[1] + t * uy - p[1]) > OV) continue;
        if (Math.abs(tx * ux + ty * uy) / Math.sqrt(l2) > 0.995) return true;
      }
      return false;
    };
    for (const c0 of raw) {
      // 2DOF 线上的点间隔可达 0.2°: 先加密到 0.02°, 去重叠时最多留 0.02° 的缝
      const c = { ...c0, D: [], box: [] };
      for (let i = 0; i < c0.D.length; i++) {
        if (i > 0) {
          const a = c0.D[i - 1], b = c0.D[i], ang = Math.acos(clamp1(dot(a, b))) * R2D;
          const m = ang < 3 ? Math.ceil(ang / 0.02) : 1, A = c0.box[i - 1], B = c0.box[i];
          for (let k = 1; k < m; k++) {
            const t = k / m;
            c.D.push(nrm([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]));
            c.box.push([wrap2(A[0] + t * wrapPi(B[0] - A[0])), wrap2(A[1] + t * wrapPi(B[1] - A[1])), A[2]]);
          }
        }
        c.D.push(c0.D[i]); c.box.push(c0.box[i]);
      }
      const Q = c.D.map(D => planeOf(SF, D)), n = Q.length, LAG = 20, kept = new Uint8Array(n);
      let run = null;
      const flush = () => { if (run && run.D.length >= 2) cands.push(run); run = null; };
      // 只登记真正留下的段 (闭合环最后那条弦若被当作自重叠去掉, 就不能再挡住别的线)
      const addKept = i => { if (kept[i] && kept[i + 1]) add(Q[i], Q[i + 1]); };
      for (let i = 0; i < n; i++) {
        const A = Q[Math.max(0, i - 1)], B = Q[Math.min(n - 1, i + 1)], tl = Math.hypot(B[0] - A[0], B[1] - A[1]) || 1;
        if (hit(Q[i], (B[0] - A[0]) / tl, (B[1] - A[1]) / tl)) flush();
        else {
          if (!run) run = { src: c.src, si: c.si, fam: c.fam, kind: c.kind, D: [], box: [], Q: [] };
          run.D.push(c.D[i]); run.box.push(c.box[i]); run.Q.push(Q[i]); kept[i] = 1;
        }
        if (i >= LAG) addKept(i - LAG);
      }
      flush();
      for (let i = Math.max(0, n - LAG); i + 1 < n; i++) addKept(i);
    }
  }
  // 圆弧: 同一个半径的 (对称的临界点给出同一个圆) 先在方位上取并集
  const circles = [];
  {
    const groups = [];
    arcs.forEach((a, ai) => {
      if (a.cls === 'none') return;
      let g = groups.find(q => Math.abs(q.d - a.d) < 1e-7);
      if (!g) groups.push(g = { d: a.d, cls: a.cls, iv: [] });
      const ph0 = Math.atan2(dot(a.D[0], SF.y), dot(a.D[0], SF.x)), len = a.psi[a.psi.length - 1] - a.psi[0];
      g.iv.push({ s: wrap2(ph0), e: wrap2(ph0) + len, full: a.full, ai });
    });
    for (const g of groups) {
      const iv = g.iv.slice().sort((x, y) => x.s - y.s);
      let out = [];
      if (iv.some(v => v.full || v.e - v.s >= TAU - 1e-9)) out = [{ s: 0, e: TAU, full: true, a0: iv[0].ai, a1: iv[0].ai }];
      else {
        for (const v of iv) {
          const last = out[out.length - 1];
          if (last && v.s <= last.e + 1e-9) { if (v.e > last.e) { last.e = v.e; last.a1 = v.ai; } }
          else out.push({ s: v.s, e: v.e, a0: v.ai, a1: v.ai });
        }
        if (out.length > 1 && out[out.length - 1].e >= out[0].s + TAU - 1e-9) {
          const L = out.pop();
          out[0].s = L.s - TAU; out[0].a0 = L.a0;
          if (L.e - TAU > out[0].e) { out[0].e = L.e - TAU; out[0].a1 = L.a1; }
        }
        if (out.length === 1 && out[0].e - out[0].s >= TAU - 1e-9) out[0].full = true;
      }
      for (const v of out) {
        const n = Math.max(2, Math.ceil((v.e - v.s) / (0.25 * D2R))), D = [];
        const sd = Math.sin(g.d), cd = Math.cos(g.d);
        for (let k = 0; k <= n; k++) {
          const ph = v.s + (v.e - v.s) * k / n, c = Math.cos(ph), s2 = Math.sin(ph);
          D.push([cd * SF.S[0] + sd * (c * SF.x[0] + s2 * SF.y[0]), cd * SF.S[1] + sd * (c * SF.x[1] + s2 * SF.y[1]), cd * SF.S[2] + sd * (c * SF.x[2] + s2 * SF.y[2])]);
        }
        circles.push({ d: g.d, cls: g.cls, full: !!v.full, a0: v.a0, a1: v.a1 });
        cands.push({ src: 'arc', ci0: circles.length - 1, ai: v.a0, kind: 'arc', D, box: D.map(() => null), Q: D.map(x => planeOf(SF, x)) });
      }
    }
  }
  if (F.zenHalf) {
    const pts = zenEdgeSky(P, F.om, F.ends[0].z, 3600).map(nrm);
    let run = [];
    const flush = () => { if (run.length >= 2) cands.push({ src: 'zen', kind: 'zen', D: run, box: run.map(() => null), Q: run.map(x => planeOf(SF, x)) }); run = []; };
    for (let i = 0; i < pts.length; i++) {
      if (run.length && Math.acos(clamp1(dot(run[run.length - 1], pts[i]))) > 2 * D2R) flush();
      run.push(pts[i]);
    }
    flush();
  }

  // ---- 切点: 圆弧两头 (切接)、端面的 2DOF 末端点、两两相交 (拐角)
  const splits = cands.map(() => []);                 // 每条候选: [{s (段号 + 比例), mi}]
  const marks = [];                                   // 每个切点: {Q, kind, pieces, at}
  const segOf = (ci, i) => { const Q = cands[ci].Q; return [Q[i][0], Q[i][1], Q[i + 1][0], Q[i + 1][1]]; };
  const CELL = 0.5, grid = new Map();
  cands.forEach((c, ci) => {
    for (let i = 0; i + 1 < c.Q.length; i++) {
      const [ax, ay, bx, by] = segOf(ci, i);
      if (Math.hypot(bx - ax, by - ay) > 3) continue;
      for (let gx = Math.floor(Math.min(ax, bx) / CELL); gx <= Math.floor(Math.max(ax, bx) / CELL); gx++)
        for (let gy = Math.floor(Math.min(ay, by) / CELL); gy <= Math.floor(Math.max(ay, by) / CELL); gy++) {
          const k = gx + ',' + gy; let L = grid.get(k); if (!L) grid.set(k, L = []); L.push([ci, i]);
        }
    }
  });
  // 把一个点投到候选曲线上 (距离 < tol 度才算)
  const nearestOn = (ci, Q, tol) => {
    const C = cands[ci].Q;
    let best = null;
    for (let i = 0; i + 1 < C.length; i++) {
      const ax = C[i][0], ay = C[i][1], bx = C[i + 1][0], by = C[i + 1][1];
      if (Math.min(ax, bx) > Q[0] + tol || Math.max(ax, bx) < Q[0] - tol || Math.min(ay, by) > Q[1] + tol || Math.max(ay, by) < Q[1] - tol) continue;
      const L2 = (bx - ax) ** 2 + (by - ay) ** 2; if (L2 > 9) continue;
      const t = L2 > 0 ? Math.max(0, Math.min(1, ((Q[0] - ax) * (bx - ax) + (Q[1] - ay) * (by - ay)) / L2)) : 0;
      const dd = Math.hypot(ax + t * (bx - ax) - Q[0], ay + t * (by - ay) - Q[1]);
      if (dd < tol && (!best || dd < best.dd)) best = { s: i + t, dd };
    }
    return best;
  };
  const addPointMark = (Q, kind, info, own) => {
    const mi = marks.length;
    marks.push({ Q, kind, ...info, pieces: [], at: [] });
    cands.forEach((c, ci) => {
      const hit = own === ci ? { s: info.ownS } : nearestOn(ci, Q, 0.02);
      if (hit) { splits[ci].push({ s: hit.s, mi }); marks[mi].pieces.push(ci); marks[mi].at.push([ci, hit.s]); }
    });
  };
  cands.forEach((c, ci) => {
    if (c.src !== 'arc' || circles[c.ci0].full) return;
    addPointMark(c.Q[0], 'tan', { ai: circles[c.ci0].a0, end: 0, ownS: 0 }, ci);
    addPointMark(c.Q[c.Q.length - 1], 'tan', { ai: circles[c.ci0].a1, end: 1, ownS: c.Q.length - 1 }, ci);
  });
  slices.forEach((sl, si) => {
    if (!sl.face) return;
    for (const r of sl.pr.eps) addPointMark(planeOf(SF, r.D), 'ep2', { si, ep: r });
  });
  // 一条候选的端点落在另一条上 (去重叠留下的接头, 或一条线正好止于另一条线): 也要切开另一条
  cands.forEach((c, ci) => {
    for (const s0 of [0, c.Q.length - 1]) {
      const Q = c.Q[s0];
      if (marks.some(m => Math.hypot(m.Q[0] - Q[0], m.Q[1] - Q[1]) < 0.005)) continue;
      addPointMark(Q, 'touch', { ownS: s0 }, ci);
    }
  });
  const pointMarks = marks.length;
  let nCross = 0;
  for (const [gk, L] of grid) {
    const [gx, gy] = gk.split(',').map(Number);
    if (nCross > 20000) break;                      // 退化光路的保险
    for (let p = 0; p < L.length; p++) for (let q = p + 1; q < L.length; q++) {
      const [ci, i] = L[p], [cj, j] = L[q];
      if (ci === cj && Math.abs(i - j) < 3) continue;
      const [ax, ay, bx, by] = segOf(ci, i), [cx, cy, dx, dy] = segOf(cj, j);
      const den = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx);
      // 几乎平行的两段 (两条边贴着走, 如 V 字两脚重合) 交点在舍入噪声里来回出现, 不当拐角
      if (Math.abs(den) < 0.03 * Math.hypot(bx - ax, by - ay) * Math.hypot(dx - cx, dy - cy)) continue;
      const t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / den, u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / den;
      if (t < 0 || t > 1 || u < 0 || u > 1) continue;
      const Q = [ax + t * (bx - ax), ay + t * (by - ay)];
      if (Math.floor(Q[0] / CELL) !== gx || Math.floor(Q[1] / CELL) !== gy) continue;     // 一对线段可能同在几个格子里: 只在交点所在的格子收
      nCross++;
      // 切接点附近、或已有很近的交点: 不重复切 (相切处数值上会擦出好几个交点)
      if (marks.some((m, mi) => Math.hypot(m.Q[0] - Q[0], m.Q[1] - Q[1]) < 0.02 && (mi >= pointMarks || (m.pieces.includes(ci) && m.pieces.includes(cj))))) continue;
      const mi = marks.length;
      marks.push({ Q, kind: 'corner', pieces: [ci, cj], at: [[ci, i + t], [cj, j + u]] });
      splits[ci].push({ s: i + t, mi }); splits[cj].push({ s: j + u, mi });
    }
  }
  log('splits', performance.now() - tCrit, cands.length, marks.length, cands.reduce((s, c) => s + c.Q.length, 0));
  // 点 M 到别的候选曲线的最近距离 (只看附近格子; 远于 0.5° 时返回 1)
  const clearance = (M, own) => {
    let best = 1;
    const gx0 = Math.floor(M[0] / CELL), gy0 = Math.floor(M[1] / CELL);
    for (let gx = gx0 - 1; gx <= gx0 + 1; gx++) for (let gy = gy0 - 1; gy <= gy0 + 1; gy++) {
      const L = grid.get(gx + ',' + gy); if (!L) continue;
      for (const [ci, i] of L) {
        if (ci === own) continue;
        const [ax, ay, bx, by] = segOf(ci, i), ux = bx - ax, uy = by - ay, l2 = ux * ux + uy * uy;
        const t = l2 > 0 ? Math.max(0, Math.min(1, ((M[0] - ax) * ux + (M[1] - ay) * uy) / l2)) : 0;
        best = Math.min(best, Math.hypot(ax + t * ux - M[0], ay + t * uy - M[1]));
      }
    }
    return best;
  };
  // ---- 切段; 每段在几处两侧各测一点. 段内结论不一致 ⇒ 漏了接头: 二分找到翻转处, 补切后再测
  const member = makeMember(P, F, SF, tf.fences, inner);
  let tests = 0;
  const atS = (c, s) => {
    const i = Math.min(c.Q.length - 2, Math.max(0, Math.floor(s))), t = s - i;
    return [c.Q[i][0] + t * (c.Q[i + 1][0] - c.Q[i][0]), c.Q[i][1] + t * (c.Q[i + 1][1] - c.Q[i][1])];
  };
  const _HE = new Float64Array(9);
  const hintOf = (c, i) => {
    const b = c.src === 'slice' ? c.box[i] : null; if (!b) return null;      // 圆弧的临界点处 ∇δ = 0, 不能当种子
    if (F.kind === 'col') orientEuler(b[0], b[2], b[1], _HE); else orientEuler(b[0], b[1], b[2], _HE);
    const A = SF.S;
    return [A[0] * _HE[0] + A[1] * _HE[1] + A[2] * _HE[2], A[0] * _HE[3] + A[1] * _HE[4] + A[2] * _HE[5], A[0] * _HE[6] + A[1] * _HE[7] + A[2] * _HE[8]];
  };
  // 候选 ci 上参数 s 处是不是轮廓: 0 = 不是 (两侧都有光); ±1 = 是, 有光的一侧沿 ±法向; 2 = 两侧都没光 (孤立细线, 也算边)
  const classify = (ci, s, len) => {
    const c = cands[ci], i = Math.min(c.Q.length - 2, Math.max(0, Math.floor(s))), M = atS(c, s), hw = hintOf(c, i);
    const tx = c.Q[i + 1][0] - c.Q[i][0], ty = c.Q[i + 1][1] - c.Q[i][1], tn = Math.hypot(tx, ty) || 1, nx = -ty / tn, ny = tx / tn;
    const side = (sg, e) => { tests++; return member(dirOfPlane(SF, M[0] + sg * e * nx, M[1] + sg * e * ny), hw); };
    // 步长不超过到旁边别的候选曲线距离的 0.4 倍: 相切处两条线贴得很近, 中间那条细缝也要测到
    let e = Math.max(0.002, Math.min(0.03, len / 4, 0.4 * clearance(M, ci)));
    if (c.src === 'arc') {
      // 圆弧: 局部极值已说明哪一侧有光 (lo: 偏向角大的一侧, hi: 小的一侧); 只测另一侧有没有被别处照到
      const larger = nx * M[0] + ny * M[1] > 0 ? 1 : -1, ins = circles[c.ci0].cls === 'lo' ? larger : -larger;
      return side(-ins, e) ? 0 : ins;
    }
    let a = side(1, e), b = side(-1, e);
    for (let k = 0; k < 2 && !a && !b; k++) { e /= 6; a = side(1, e); b = side(-1, e); }   // 有光的一侧很窄 (尖端附近): 缩小再测
    return a !== b ? (a ? 1 : -1) : a ? 0 : 2;
  };
  const cache = new Map();
  let segs = [];
  for (let iter = 0; iter < 4; iter++) {
    segs = [];
    const flips = [];
    cands.forEach((c, ci) => {
      const sp = [{ s: 0, mi: -1 }, ...splits[ci], { s: c.Q.length - 1, mi: -1 }].sort((a, b) => a.s - b.s);
      for (let k = 0; k + 1 < sp.length; k++) {
        const s0 = sp[k].s, s1 = sp[k + 1].s;
        if (s1 - s0 < 1e-9) continue;
        const pts = [atS(c, s0)];
        for (let i = Math.floor(s0) + 1; i < s1; i++) pts.push(c.Q[i]);
        pts.push(atS(c, s1));
        let len = 0;
        for (let i = 1; i < pts.length; i++) len += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
        const seg = { ci, src: c.src, kind: c.kind, si: c.si, fam: c.fam, ai: c.ai, s0, s1, m0: sp[k].mi, m1: sp[k + 1].mi, len, outline: false, inside: 0, pts };
        segs.push(seg);
        if (len < 1e-6) continue;
        const key = `${ci}|${s0}|${s1}`;
        if (cache.has(key)) { Object.assign(seg, cache.get(key)); continue; }
        const fs = len < 1 ? [0.5] : [0.2, 0.5, 0.8];
        const st = fs.map(f => classify(ci, s0 + f * (s1 - s0), len));
        const isOut = v => v !== 0;
        const j = st.findIndex((v, q) => q > 0 && isOut(v) !== isOut(st[q - 1]));
        if (j > 0 && iter < 3) {
          let fa = fs[j - 1], fb = fs[j], oa = isOut(st[j - 1]);
          for (let q = 0; q < 14; q++) { const fm = 0.5 * (fa + fb); if (isOut(classify(ci, s0 + fm * (s1 - s0), len)) === oa) fa = fm; else fb = fm; }
          flips.push([ci, s0 + 0.5 * (fa + fb) * (s1 - s0)]);
          continue;
        }
        const mid = st[st.length >> 1], res = { outline: isOut(mid), inside: mid === 2 ? 0 : mid };
        cache.set(key, res); Object.assign(seg, res);
      }
    });
    if (!flips.length) break;
    for (const [ci, s] of flips) addPointMark(atS(cands[ci], s), 'flip', { ownS: s }, ci);
  }

  // ---- 末端点: 切点处至少一段轮廓, 且轮廓在这里换了曲线 (或到头)
  const eps = [];
  marks.forEach((mk, mi) => {
    const adj = segs.filter(s => s.outline && (s.m0 === mi || s.m1 === mi));
    if (!adj.length) return;
    const curves = new Set(adj.map(s => s.ci));
    // 轮廓在这里的走向 (从切点指向各段); 全都共线 = 顺着过去 (重叠留下的接头), 不是拐角
    const dirs = adj.map(s => {
      const q = s.pts, n = q.length, at0 = s.m0 === mi;
      const a = at0 ? q[0] : q[n - 1], b = at0 ? q[Math.min(n - 1, 2)] : q[Math.max(0, n - 3)];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      return [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
    });
    const collinear = dirs.every(d => Math.abs(d[0] * dirs[0][1] - d[1] * dirs[0][0]) < 0.05);
    if (mk.kind === 'tan' && curves.size < 2) return;                           // 圆弧不当边, 轮廓顺着端面曲线过去
    if ((mk.kind === 'corner' || mk.kind === 'touch' || mk.kind === 'flip') && (curves.size < 2 || (collinear && adj.length > 1))) return;
    const D = dirOfPlane(SF, mk.Q[0], mk.Q[1]);
    const rec = { kind: mk.kind === 'ep2' ? mk.ep.kind : (mk.kind === 'touch' || mk.kind === 'flip') ? (adj.length === 1 ? 'end' : 'corner') : mk.kind, D, pieces: [...curves].map(ci => pieceName(cands[ci], slices, arcs)), box: [] };
    if (mk.kind === 'ep2') rec.label = mk.ep.label;
    // 原像 (族内坐标): 端面 2DOF 末端点用它自己的取向角; 切接用圆弧端点 (精确); 拐角只取在这里相接的那几条线
    if (mk.kind === 'ep2') rec.box.push([mk.ep.sc * D2R, mk.ep.sw * D2R, slices[mk.si].z]);
    else if (mk.kind === 'tan') { const a = arcs[mk.ai], b = a.box[mk.end ? a.box.length - 1 : 0]; if (b) rec.box.push(b); }
    else for (const [ci, s] of mk.at) {
      if (!curves.has(ci)) continue;
      const c = cands[ci]; if (!c.box[0]) continue;
      const i = Math.min(c.box.length - 2, Math.max(0, Math.floor(s))), t = Math.max(0, Math.min(1, s - i)), A = c.box[i], B = c.box[i + 1] || A;
      if (!A) continue;
      const lerpA = (p, q) => p + t * wrapPi(q - p);
      rec.box.push([wrap2(lerpA(A[0], B[0])), wrap2(lerpA(A[1], B[1])), A[2] + t * (B[2] - A[2])]);
    }
    if (eps.some(e => Math.acos(clamp1(dot(e.D, D))) < (e.kind === rec.kind ? 0.05 : 2e-4) * D2R)) return;   // 同类的挨得很近 (< 0.05°) 只留一个
    rec.az = Math.atan2(D[1], D[0]) * R2D; rec.el = Math.asin(clamp1(D[2])) * R2D;
    rec.d = Math.acos(clamp1(dot(D, SF.S))) * R2D;
    eps.push(rec);
  });

  return {
    slices, crit, arcs, circles, fences: tf.fences, cands, segs, eps, inner, member,
    ms: { slices: tSlices - t0, crit: tCrit - tSlices, outline: performance.now() - tCrit, total: performance.now() - t0 }, tests,
  };
}

function pieceName(c, slices, arcs) {
  if (c.src === 'slice') return { src: 'slice', si: c.si, z: slices[c.si].z * R2D, fam: c.fam, kind: c.kind };
  if (c.src === 'arc') { const a = arcs[c.ai]; return { src: 'arc', d: a.d * R2D, from: a.src, name: a.name, cls: a.cls }; }
  return { src: 'zen' };
}

// ============================================================ 可视化数据 (按需)
/** 这群晶体的 MC: 族内坐标均匀撒点, 返回成立的 { D (xyz), box (xyz, 度) } */
export function mcDof3(P, F, M, seed = 7) {
  let s = seed >>> 0 || 1;
  const rnd = () => { s = (s + 0x6D2B79F5) | 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const c = newC3(P), D = [], box = [];
  const yMax = F.kind === 'low' && F.zenHalf ? Math.PI : TAU;
  for (let i = 0; i < M; i++) {
    const x = rnd() * TAU, y = rnd() * yMax, z = F.lo + rnd() * (F.hi - F.lo);
    // 柱晶: (α, roll) 自由, ζ = z; Lowitz: (α, ζ) 自由, roll = z
    chain3(x, y, F.om, z, P, c);
    if (!valid3(c, 0) || !apOk(P, F.om, z, x, y)) continue;
    const n = Math.hypot(c.Dx, c.Dy, c.Dz) || 1;
    D.push(c.Dx / n, c.Dy / n, c.Dz / n); box.push(x * R2D, y * R2D, z * R2D);
  }
  return { D: Float32Array.from(D), box: Float32Array.from(box) };
}

/**
 * 边界面: 每条边界曲线 (w 球面上) × 绕太阳的 ψ, 落在区间里的部分. 精确参数化, 不是隐式网格.
 * mode 'box' → 顶点 = 族内坐标 (度); 'lift' → 顶点 = (鱼眼 x, y, 第三个角°).
 * 返回 [{ kind, name, tri: Float32Array (9/三角形) }]
 */
export function fenceSurfaces(P, F, A, mode, nT = 150, nPsi = 180) {
  const SF = sunFrame(), out = [], fo = [0, 0];
  for (const fc of A.fences) {
    // 成立段, 每段沿曲线均匀取 ≤ nT 个点
    const runs = [];
    let cur = null;
    fc.pts.forEach((p, i) => { if (fc.ok[i]) { (cur || (cur = [])).push(p); } else if (cur) { runs.push(cur); cur = null; } });
    if (cur) runs.push(cur);
    if (runs.length > 1 && fc.ok[0] && fc.ok[fc.pts.length - 1]) { runs[0] = [...runs.pop(), ...runs[0]]; }
    const tri = [];
    for (const run of runs) {
      if (run.length < 3) continue;
      const step = Math.max(1, Math.floor(run.length / nT));
      const ws = run.filter((_, i) => i % step === 0);
      // 每个顶点存全部各组族内坐标; 一个三角形在 "里面那个顶点" 所用的那一组里插值裁剪 (Lowitz 两组解的 roll 差半圈, 混用会拉出乱面)
      const V = ws.map(w => {
        const fr = frameAt(P, w), row = [];
        for (let j = 0; j <= nPsi; j++) {
          const R = rotOf(SF, fr, j / nPsi * TAU), all = boxAll(F, R(E1), R(E3));
          let sky = null;
          if (mode !== 'box') { const D = R(fr.F); fisheyeOf(D[0], D[1], D[2], fo); sky = [fo[0], fo[1]]; }
          row.push(all.map(b => ({ p: mode === 'box' ? [b.x * R2D, b.y * R2D, b.z * R2D] : [sky[0], sky[1], b.z * R2D], m: b.m })));
        }
        return row;
      });
      for (let i = 0; i + 1 < V.length; i++)
        for (let j = 0; j < nPsi; j++) {
          const a = V[i][j], b = V[i + 1][j], c = V[i + 1][j + 1], d = V[i][j + 1];
          for (const t of [[a, b, c], [a, c, d]]) {
            // 选组: 三个顶点里裕度最大的那组 (同组才连续)
            let best = -1, bm = -Infinity;
            for (let k = 0; k < t[0].length; k++) { const mm = Math.max(t[0][k].m, t[1][k].m, t[2][k].m); if (mm > bm) { bm = mm; best = k; } }
            if (bm < 0) continue;
            clipTri([t[0][best], t[1][best], t[2][best]], tri, mode);
          }
        }
    }
    if (tri.length) out.push({ kind: fc.kind, name: fc.name, tri: Float32Array.from(tri) });
  }
  return out;
}
// 三角形按区间裕度 m ≥ 0 裁剪 (线性插值), 跨周期缝的丢掉
function clipTri(t, out, mode) {
  const ins = t.filter(v => v.m >= 0).length;
  if (ins === 0) return;
  const seam = (u, v) => mode === 'box' && (Math.abs(u.p[0] - v.p[0]) > 180 || Math.abs(u.p[1] - v.p[1]) > 180);
  if (seam(t[0], t[1]) || seam(t[1], t[2]) || seam(t[0], t[2])) return;
  if (mode === 'lift' && Math.max(Math.hypot(t[0].p[0] - t[1].p[0], t[0].p[1] - t[1].p[1]), Math.hypot(t[1].p[0] - t[2].p[0], t[1].p[1] - t[2].p[1])) > 20) return;
  if (ins === 3) { for (const v of t) out.push(...v.p); return; }
  const poly = [];
  for (let k = 0; k < 3; k++) {
    const u = t[k], v = t[(k + 1) % 3];
    if (u.m >= 0) poly.push(u.p);
    if ((u.m >= 0) !== (v.m >= 0)) { const s = u.m / (u.m - v.m); poly.push(u.p.map((x, i) => x + s * (v.p[i] - x))); }
  }
  for (let k = 1; k + 1 < poly.length; k++) out.push(...poly[0], ...poly[k], ...poly[k + 1]);
}

// ============================================================ 交给界面的数据
/**
 * 外轮廓 (同一条候选上相连的轮廓段并成一条折线) + 末端点 + 圆弧一览. 天空方向都是单位向量 (世界系).
 * 折线名: 端面 → `端面|${第三个角°}|${族名}`; 圆弧 → `圆弧|${δ°}|${来源}`; c 轴竖直的截断边 → `ζ边界`.
 */
export function packDof3(A) {
  const lines = [];
  const byC = new Map();
  for (const s of A.segs) if (s.outline && s.len > 1e-6) { if (!byC.has(s.ci)) byC.set(s.ci, []); byC.get(s.ci).push(s); }
  for (const [ci, list] of byC) {
    const c = A.cands[ci];
    list.sort((a, b) => a.s0 - b.s0);
    const at = u => { const i = Math.min(c.D.length - 2, Math.max(0, Math.floor(u))), t = u - i, a = c.D[i], b = c.D[i + 1]; return nrm([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]); };
    let cur = null, last = -1;
    const flush = () => { if (cur && cur.length >= 6) lines.push({ name: cur.name, kind: c.kind, src: c.src, pts: Float32Array.from(cur) }); cur = null; };
    for (const sg of list) {
      if (!cur || Math.abs(sg.s0 - last) > 1e-9) {
        flush();
        cur = [];
        cur.name = c.src === 'slice' ? `端面|${+(A.slices[c.si].z * R2D).toFixed(3)}|${c.fam}` : c.src === 'arc' ? `圆弧|${(A.circles[c.ci0].d * R2D).toFixed(2)}|${A.arcs[c.ai].src}` : 'ζ边界';
        cur.push(...at(sg.s0));
      }
      for (let i = Math.floor(sg.s0) + 1; i < sg.s1; i++) cur.push(...c.D[i]);
      cur.push(...at(sg.s1));
      last = sg.s1;
    }
    flush();
  }
  const eps = A.eps.map(e => ({
    kind: e.kind, label: e.label || '', D: e.D, az: e.az, el: e.el, d: e.d,
    box: e.box.map(b => [b[0] * R2D, b[1] * R2D, b[2] * R2D]).filter((b, i, A) => A.findIndex(q => Math.abs(q[0] - b[0]) < 0.05 && Math.abs(q[1] - b[1]) < 0.05 && Math.abs(q[2] - b[2]) < 0.05) === i),
    pieces: e.pieces, alive: true,
  }));
  const circles = A.circles.map(c => ({ d: c.d * R2D, cls: c.cls, full: c.full }));
  const crit = A.crit.map(c => ({ d: c.d * R2D, cls: c.cls, src: c.src, name: c.name || '' }));
  return { lines, eps, circles, crit, ms: A.ms, tests: A.tests, nOutline: A.segs.filter(s => s.outline).length };
}

/** 取向空间 (族内坐标的方盒, 度): 边界面、临界纤维 (圆弧的原像)、MC 点 */
export function spaceData(P, F, A, nMC = 160000) {
  const surf = fenceSurfaces(P, F, A, 'box');
  const fibers = A.arcs.map(a => ({ d: a.d * R2D, cls: a.cls, src: a.src, pts: Float32Array.from(a.box.flatMap(b => [b[0] * R2D, b[1] * R2D, b[2] * R2D])) }));
  const mc = mcDof3(P, F, nMC, 3).box;
  return { surf, fibers, mc };
}

/** 抬升 (以太阳为中心的鱼眼 x, y (度), 高度 = 第三个角°): 边界面扫出的墙、临界纤维、MC 点、端面曲线 */
export function liftData(P, F, A, nMC = 200000) {
  const fo = [0, 0];
  const fish = D => { fisheyeOf(D[0], D[1], D[2], fo); return [fo[0], fo[1]]; };
  const surf = fenceSurfaces(P, F, A, 'lift');
  const fibers = A.arcs.map(a => ({ d: a.d * R2D, cls: a.cls, src: a.src, pts: Float32Array.from(a.D.flatMap((D, i) => [...fish(D), a.box[i][2] * R2D])) }));
  const m = mcDof3(P, F, nMC, 5), mc = new Float32Array(m.D.length);
  for (let i = 0; i < m.D.length / 3; i++) { fisheyeOf(m.D[3 * i], m.D[3 * i + 1], m.D[3 * i + 2], fo); mc[3 * i] = fo[0]; mc[3 * i + 1] = fo[1]; mc[3 * i + 2] = m.box[3 * i + 2]; }
  const slices = A.slices.map(sl => {
    if (!sl.face) return { z: sl.z * R2D, lines: [] };
    const lines = [];
    for (const [name, fl] of sl.pr.lines)
      for (const sky of [...fl.sky, ...fl.conn.sky]) {
        const pts = [];
        for (let i = 0; i < sky.length / 4; i++) { const n = Math.hypot(sky[4 * i], sky[4 * i + 1], sky[4 * i + 2]) || 1; fisheyeOf(sky[4 * i] / n, sky[4 * i + 1] / n, sky[4 * i + 2] / n, fo); pts.push(fo[0], fo[1], sky[4 * i + 3] > 0.5 ? 1 : 0); }
        lines.push({ name, pts: Float32Array.from(pts) });
      }
    return { z: sl.z * R2D, lines };
  });
  const outline = packDof3(A).lines.map(L => { const p = []; for (let i = 0; i < L.pts.length / 3; i++) p.push(...fish([L.pts[3 * i], L.pts[3 * i + 1], L.pts[3 * i + 2]])); return { name: L.name, src: L.src, pts: Float32Array.from(p) }; });
  return { surf, fibers, mc, slices, outline, lo: F.lo * R2D, hi: F.hi * R2D };
}
