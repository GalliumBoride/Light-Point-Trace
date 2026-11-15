// 直纹曲面 — 同 IceHaloSimulatorPrototype「直纹曲面分析」窗体 (MotherSurfaceCalculator.ComputeUnifiedSurface).
//
// 一个姿态 → 一条直线 (母线) L(t) = P + t·n_out:
//   P     = 晶体里的光点 (半径 n 的球上, 中间的反射都做完), 即准线上的一点;
//   n_out = 出射面的法向.
// 出射折射就是光点沿 n_out 从 n 球走到天球 (单位球), 所以母线和天球的交点就是光射向天空的方向 (弧上的点).
// 姿态沿一条曲线走 — 1DOF 是扫描角转一圈, 2DOF 是取向平面里的某条边界 (连同不成立的延伸部分) — 母线扫出直纹曲面;
// 曲面 ∩ 天球 = 弧 (连同延伸弧). 光路成立的母线算「可达」(窗体里蓝色), 不成立的算「不可达」(红色).
// t ∈ [−3, 3] 同窗体; 母线沿 t 分 8 段, 相邻母线之间的双线性小片用三角形逼近 (窗体同法).

import { LP, K, orientOf } from './lp.js';

const T_MAX = 3.0, NT = 9;
const _E = new Float64Array(9);

/** 姿态 (sc, sw) 的准线点 P 与出射面法向 n_out (世界系), 写进 out[0..5] */
export function ruledFrame(P, m, fixedAng, sc, sw, out) {
  const e = orientOf(m, fixedAng, sc, sw, _E), A3 = LP.A3, cf = P.coef, n = P.faces.length;
  const nrm = i => {
    const c1 = cf[3 * i], c2 = cf[3 * i + 1], c3 = cf[3 * i + 2];
    return [c1 * e[0] + c2 * e[3] + c3 * e[6], c1 * e[1] + c2 * e[4] + c3 * e[7], c1 * e[2] + c2 * e[5] + c3 * e[8]];
  };
  let u = nrm(0);
  const g0 = A3[0] * u[0] + A3[1] * u[1] + A3[2] * u[2], t0 = -g0 + Math.sqrt(g0 * g0 + K);
  let Px = A3[0] + t0 * u[0], Py = A3[1] + t0 * u[1], Pz = A3[2] + t0 * u[2];
  for (let i = 1; i < n - 1; i++) {
    u = nrm(i);
    const q = 2 * (Px * u[0] + Py * u[1] + Pz * u[2]);
    Px -= q * u[0]; Py -= q * u[1]; Pz -= q * u[2];
  }
  u = nrm(n - 1);
  out[0] = Px; out[1] = Py; out[2] = Pz; out[3] = u[0]; out[4] = u[1]; out[5] = u[2];
  return out;
}

/** 母线和天球的交点 (同出射折射 tE = −q_E − √(q_E² − K) 那一支); 不相交 (出射全反射) 返回 null */
function hitSphere(f) {
  const q = f[0] * f[3] + f[1] * f[4] + f[2] * f[5], d = q * q - K;
  if (d < 0) return null;
  const t = -q - Math.sqrt(d);
  return [f[0] + t * f[3], f[1] + t * f[4], f[2] + t * f[5]];
}

/**
 * 沿一串姿态 (uv: [sc0, sw0, sc1, sw1, …] 弧度, ok: 每个姿态光路是否成立) 扫出直纹曲面.
 * 相邻两个姿态都成立 → 可达片, 否则不可达片 (成立/不成立的交界已由调用方放进 uv: 1DOF 二分, 2DOF 边界线自带吸附点).
 * 母线跳变 (准线点相距 > 0.25 或方向夹角 > 25°) 处断开, 不连成一片.
 * 返回 { reach: {pos, idx}, unreach: {pos, idx}, ext: 延伸弧线段 (不成立的母线与天球的交点, 两两一段) }
 */
export function ruledStrips(P, m, fixedAng, uv, ok, periodic = false) {
  const n = uv.length / 2, F = new Float64Array(6 * n), tmp = new Float64Array(6);
  for (let i = 0; i < n; i++) F.set(ruledFrame(P, m, fixedAng, uv[2 * i], uv[2 * i + 1], tmp), 6 * i);
  const grp = () => ({ pos: [], idx: [], lastI: -1, lastBase: -1 });
  const G = [grp(), grp()];                      // [不可达, 可达]
  const row = (g, i) => {
    if (g.lastI === i) return g.lastBase;
    const base = g.pos.length / 3, o = 6 * i;
    for (let k = 0; k < NT; k++) {
      const t = -T_MAX + 2 * T_MAX * k / (NT - 1);
      g.pos.push(F[o] + t * F[o + 3], F[o + 1] + t * F[o + 4], F[o + 2] + t * F[o + 5]);
    }
    g.lastI = i; g.lastBase = base;
    return base;
  };
  const ext = [];
  const pairs = periodic ? n : n - 1;
  for (let a = 0; a < pairs; a++) {
    const b = (a + 1) % n, oa = 6 * a, ob = 6 * b;
    const dP = Math.hypot(F[oa] - F[ob], F[oa + 1] - F[ob + 1], F[oa + 2] - F[ob + 2]);
    const cosE = F[oa + 3] * F[ob + 3] + F[oa + 4] * F[ob + 4] + F[oa + 5] * F[ob + 5];
    if (dP > 0.25 || cosE < 0.906) continue;
    const reach = ok[a] && ok[b];
    const g = G[reach ? 1 : 0];
    const b0 = row(g, a), b1 = row(g, b);
    for (let k = 0; k < NT - 1; k++) g.idx.push(b0 + k, b0 + k + 1, b1 + k, b0 + k + 1, b1 + k + 1, b1 + k);
    if (!reach) {
      const ha = hitSphere(F.subarray(oa, oa + 6)), hb = hitSphere(F.subarray(ob, ob + 6));
      if (ha && hb && Math.hypot(ha[0] - hb[0], ha[1] - hb[1], ha[2] - hb[2]) < 0.2) ext.push(...ha, ...hb);
    }
  }
  const pack = g => ({ pos: Float32Array.from(g.pos), idx: Uint32Array.from(g.idx) });
  return { reach: pack(G[1]), unreach: pack(G[0]), ext: Float32Array.from(ext) };
}

/**
 * 1DOF (板晶 / Parry): 扫描角 u 转一圈 (Parry 的 roll 固定为 fixedV), 每 0.25° 一条母线;
 * 成立 / 不成立交界二分到 1e-9 rad, 在那里插一条母线, 颜色就在那里切换 (同窗体 FindBoundaryParam).
 */
export function ruled1(P, m, fixedAng, fixedV, valid) {
  const N = 1440, du = 2 * Math.PI / N, uv = [], ok = [];
  const okAt = u => valid(u, fixedV);
  let prev = okAt(0);
  for (let i = 0; i < N; i++) {
    const u = i * du;
    uv.push(u, fixedV); ok.push(prev ? 1 : 0);
    const next = okAt(u + du);
    if (next !== prev) {
      let lo = u, hi = u + du;
      for (let k = 0; k < 40; k++) { const mid = 0.5 * (lo + hi); if (okAt(mid) === prev) lo = mid; else hi = mid; }
      uv.push(lo, fixedV); ok.push(prev ? 1 : 0);               // 交界两侧各一条, 中间那一小片按不可达画 (宽 < 1e-9 rad)
      uv.push(hi, fixedV); ok.push(next ? 1 : 0);
    }
    prev = next;
  }
  return ruledStrips(P, m, fixedAng, Float64Array.from(uv), Uint8Array.from(ok), true);
}

/**
 * 2DOF: 某条边界在取向平面里的所有连线 (getFamLines 的 par/sky, 含不成立的延伸部分),
 * 每条连线扫一张直纹曲面. 太密的连线按取向平面里 0.4° 抽稀, 成立/不成立切换处的点都保留.
 */
export function ruledFam(P, m, fixedAng, fl) {
  const parts = [];
  for (let li = 0; li < fl.par.length; li++) {
    const par = fl.par[li], sky = fl.sky[li], n = par.length / 2;
    if (n < 2) continue;
    const uv = [], ok = [];
    let lu = NaN, lv = NaN;
    for (let i = 0; i < n; i++) {
      const o = sky[4 * i + 3] > 0.5 ? 1 : 0;
      const keep = i === 0 || i === n - 1 || o !== ok[ok.length - 1] || (i + 1 < n && (sky[4 * i + 7] > 0.5 ? 1 : 0) !== o) ||
                   Math.hypot(par[2 * i] - lu, par[2 * i + 1] - lv) >= 0.4 * Math.PI / 180;
      if (!keep) continue;
      uv.push(par[2 * i], par[2 * i + 1]); ok.push(o);
      lu = par[2 * i]; lv = par[2 * i + 1];
    }
    parts.push(ruledStrips(P, m, fixedAng, Float64Array.from(uv), Uint8Array.from(ok), false));
  }
  // 合并成一份
  const cat = key => {
    let nv = 0, ni = 0;
    for (const p of parts) { nv += p[key].pos.length; ni += p[key].idx.length; }
    const pos = new Float32Array(nv), idx = new Uint32Array(ni);
    let ov = 0, oi = 0;
    for (const p of parts) {
      pos.set(p[key].pos, ov);
      const base = ov / 3;
      for (let k = 0; k < p[key].idx.length; k++) idx[oi + k] = p[key].idx[k] + base;
      ov += p[key].pos.length; oi += p[key].idx.length;
    }
    return { pos, idx };
  };
  let ne = 0;
  for (const p of parts) ne += p.ext.length;
  const ext = new Float32Array(ne);
  let oe = 0;
  for (const p of parts) { ext.set(p.ext, oe); oe += p.ext.length; }
  return { reach: cat('reach'), unreach: cat('unreach'), ext };
}
