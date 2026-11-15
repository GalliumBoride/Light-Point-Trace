// 直纹曲面 — 同 IceHaloSimulatorPrototype「直纹曲面分析」窗体 (MotherSurfaceCalculator.ComputeUnifiedSurface).
//
// 一个姿态 → 一条直线 (母线) L(t) = P + t·n_out:
//   P     = 晶体里的光点 (半径 n 的球上, 中间的反射都做完), 即准线上的一点;
//   n_out = 出射面的法向.
// 出射折射就是光点沿 n_out 从 n 球走到天球 (单位球), 所以母线和天球的交点就是光射向天空的方向 (弧上的点).
// 姿态沿一条曲线走 — 1DOF 是扫描角转一圈, 2DOF 是取向平面里的某条边界 (连同不成立的延伸部分) — 母线扫出直纹曲面;
// 曲面 ∩ 天球 = 弧 (连同延伸弧). 光路成立的母线算「可达」(窗体里蓝色), 不成立的算「不可达」(红色).
// t ∈ [−3, 3] 同窗体; 母线沿 t 分 8 段, 相邻母线之间的双线性小片用三角形逼近 (窗体同法).
//
// 曲面按「片」给出: 一片 = 一串相邻母线, 可达性相同、中间没有跳变. 取向平面里不相连的几块成立区域, 各自的边界扫出各自的片;
// 可达片标上它挨着的那块区域的编号 (1DOF: 成立的角度区间; 2DOF: 取向平面网格上成立格点的连通块), 界面可以逐片、逐块开关.

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

function framesOf(P, m, fixedAng, uv) {
  const n = uv.length / 2, F = new Float64Array(6 * n), tmp = new Float64Array(6);
  for (let i = 0; i < n; i++) F.set(ruledFrame(P, m, fixedAng, uv[2 * i], uv[2 * i + 1], tmp), 6 * i);
  return F;
}

/** 相邻两条母线之间是否跳变 (准线点相距 > 0.25 或方向夹角 > 25°): 跳变处不连成一片 */
function jump(F, a, b) {
  const oa = 6 * a, ob = 6 * b;
  const dP = Math.hypot(F[oa] - F[ob], F[oa + 1] - F[ob + 1], F[oa + 2] - F[ob + 2]);
  const cosE = F[oa + 3] * F[ob + 3] + F[oa + 4] * F[ob + 4] + F[oa + 5] * F[ob + 5];
  return dP > 0.25 || cosE < 0.906;
}

/**
 * 把一串母线切成片: 每条边 (a, a+1) 的类型 = 跳变 / 可达 (两端都成立) / 不可达, 同类型的连续边连成一片.
 * periodic: 首尾相接 (1DOF 转一圈); 起点挪到一个类型变化处, 跨 0/360 的那片不会被切成两半.
 * 返回 [{ reach, gens: [母线序号…] }]
 */
function runsOf(F, ok, periodic) {
  const n = ok.length, ne = periodic ? n : n - 1;
  if (ne <= 0) return [];
  const type = new Int8Array(ne);                  // −1 跳变, 1 可达, 0 不可达
  for (let a = 0; a < ne; a++) {
    const b = (a + 1) % n;
    type[a] = jump(F, a, b) ? -1 : (ok[a] && ok[b] ? 1 : 0);
  }
  let s0 = 0;
  if (periodic) {
    s0 = -1;
    for (let a = 0; a < ne; a++) if (type[a] !== type[(a - 1 + ne) % ne]) { s0 = a; break; }
    if (s0 < 0) return type[0] < 0 ? [] : [{ reach: type[0] === 1, gens: [...Array(n).keys(), 0] }];   // 整圈同一类: 一片闭合
  }
  const out = [];
  let cur = null;
  for (let k = 0; k < ne; k++) {
    const a = (s0 + k) % ne, ty = type[a];
    if (ty < 0) { cur = null; continue; }
    if (!cur || cur.reach !== (ty === 1)) { cur = { reach: ty === 1, gens: [a] }; out.push(cur); }
    cur.gens.push((a + 1) % n);
  }
  return out;
}

/** 一片的网格: 每条母线 NT 个点, 相邻母线之间连三角形; 不可达片另给延伸弧 (母线与天球的交点连成的线段) */
function pieceMesh(F, gens, reach) {
  const pos = new Float32Array(3 * NT * gens.length), idx = [];
  gens.forEach((g, r) => {
    const o = 6 * g;
    for (let k = 0; k < NT; k++) {
      const t = -T_MAX + 2 * T_MAX * k / (NT - 1), w = 3 * (r * NT + k);
      pos[w] = F[o] + t * F[o + 3]; pos[w + 1] = F[o + 1] + t * F[o + 4]; pos[w + 2] = F[o + 2] + t * F[o + 5];
    }
    if (r > 0) {
      const b0 = (r - 1) * NT, b1 = r * NT;
      for (let k = 0; k < NT - 1; k++) idx.push(b0 + k, b0 + k + 1, b1 + k, b0 + k + 1, b1 + k + 1, b1 + k);
    }
  });
  const ext = [];
  if (!reach)
    for (let r = 1; r < gens.length; r++) {
      const ha = hitSphere(F.subarray(6 * gens[r - 1], 6 * gens[r - 1] + 6)), hb = hitSphere(F.subarray(6 * gens[r], 6 * gens[r] + 6));
      if (ha && hb && Math.hypot(ha[0] - hb[0], ha[1] - hb[1], ha[2] - hb[2]) < 0.2) ext.push(...ha, ...hb);
    }
  return { reach, pos, idx: Uint32Array.from(idx), ext: Float32Array.from(ext) };
}

/**
 * 1DOF (板晶 / Parry): 扫描角 u 转一圈 (Parry 的 roll 固定为 fixedV), 每 0.25° 一条母线;
 * 成立 / 不成立交界二分到 1e-9 rad, 在那里插一条母线, 颜色就在那里切换 (同窗体 FindBoundaryParam).
 * 每段成立的角度区间是一块区域 (区域号按 u 从小到大). 返回 { pieces: [{ reach, pos, idx, ext, region, a0, a1 (°) }] }
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
      uv.push(lo, fixedV); ok.push(prev ? 1 : 0);               // 交界两侧各一条, 中间宽 < 1e-9 rad 的那一小条按不可达画
      uv.push(hi, fixedV); ok.push(next ? 1 : 0);
    }
    prev = next;
  }
  const F = framesOf(P, m, fixedAng, uv);
  const runs = runsOf(F, ok, true);
  const deg = g => ((uv[2 * g] * 180 / Math.PI) % 360 + 360) % 360;
  const reachRuns = runs.filter(r => r.reach).sort((a, b) => deg(a.gens[0]) - deg(b.gens[0]));
  const pieces = runs.map(r => ({ ...pieceMesh(F, r.gens, r.reach), region: r.reach ? reachRuns.indexOf(r) + 1 : 0,
                                  a0: deg(r.gens[0]), a1: deg(r.gens[r.gens.length - 1]) }));
  return { pieces };
}

/**
 * 取向平面里成立格点的连通块 (4 邻接, 按轴的周期性首尾相接), 按大小编号 (1 = 最大).
 * 返回 labelAt(u, v): 该点附近 (3 格内) 成立格点所在块的编号, 没有为 0.
 */
export function regionLabeler(ori, perX, perY) {
  const { nS, nW, valid, SCs, SWs } = ori, lab = new Int32Array(nS * nW), sizes = [0];
  let next = 0;
  const stack = [];
  for (let s = 0; s < nS * nW; s++) {
    if (!valid[s] || lab[s]) continue;
    lab[s] = ++next; sizes.push(0); stack.push(s);
    while (stack.length) {
      const c = stack.pop(), iw = (c / nS) | 0, is = c - iw * nS;
      sizes[next]++;
      for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        let i2 = is + di, j2 = iw + dj;
        if (i2 < 0 || i2 >= nS) { if (!perX) continue; i2 = (i2 + nS) % nS; }
        if (j2 < 0 || j2 >= nW) { if (!perY) continue; j2 = (j2 + nW) % nW; }
        const c2 = j2 * nS + i2;
        if (valid[c2] && !lab[c2]) { lab[c2] = next; stack.push(c2); }
      }
    }
  }
  const order = [...Array(next).keys()].map(i => i + 1).sort((a, b) => sizes[b] - sizes[a]);
  const rank = new Int32Array(next + 1);
  order.forEach((l, i) => { rank[l] = i + 1; });
  const dx = (SCs[nS - 1] - SCs[0]) / (nS - 1), dy = (SWs[nW - 1] - SWs[0]) / (nW - 1);
  const wrapI = i => (perX ? ((i % nS) + nS) % nS : i), wrapJ = j => (perY ? ((j % nW) + nW) % nW : j);
  return (u, v) => {
    const i0 = Math.round((u - SCs[0]) / dx), j0 = Math.round((v - SWs[0]) / dy);
    for (let r = 0; r <= 3; r++)
      for (let dj = -r; dj <= r; dj++)
        for (let di = -r; di <= r; di++) {
          if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;
          const i = wrapI(i0 + di), j = wrapJ(j0 + dj);
          if (i < 0 || i >= nS || j < 0 || j >= nW) continue;
          const l = lab[j * nS + i];
          if (l) return rank[l];
        }
    return 0;
  };
}

/**
 * 2DOF: 某条边界在取向平面里的所有连线 (getFamLines 的 par/sky, 含不成立的延伸部分), 切成片.
 * 太密的连线按取向平面里 0.4° 抽稀, 成立/不成立切换处的点都保留. 可达片标上 labelAt 给的区域号 (取片中间那条母线).
 * 返回 { pieces: [{ reach, pos, idx, ext, region }] }
 */
export function ruledFam(P, m, fixedAng, fl, labelAt) {
  const pieces = [];
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
    const F = framesOf(P, m, fixedAng, uv);
    for (const r of runsOf(F, ok, false)) {
      const g = r.gens[r.gens.length >> 1];
      pieces.push({ ...pieceMesh(F, r.gens, r.reach), region: r.reach && labelAt ? labelAt(uv[2 * g], uv[2 * g + 1]) : 0 });
    }
  }
  return { pieces };
}
