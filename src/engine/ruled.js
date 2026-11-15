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

import { LP, K, orientOf, traceContours } from './lp.js';

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

/**
 * 一片的网格: 每条母线 NT 个点, 相邻母线之间连三角形; 不可达片另给延伸弧 (母线与天球的交点连成的线段).
 * 顶点法向直接按曲面算: L(s, t) = P(s) + t·e(s), N = ∂L/∂s × e = A + t·B, A = P'×e, B = e'×e
 * (P', e' 用相邻母线差分), 不靠三角形法向平均 — 曲面折回自己时相邻三角形绕向相反, 平均会抵消.
 * N 不归一化: 它沿母线是 t 的一次式, 着色器线性插值后正好是像素处的真法向 (腰点附近法向在很短一段里转半圈,
 * 归一化后再插值会出放射状的假褶子). 参数 s 取 (P, e) 这条六维曲线的弧长, 母线疏密不同的地方 N 的大小也一致.
 */
function pieceMesh(F, gens, reach) {
  const m = gens.length, pos = new Float32Array(3 * NT * m), nrm = new Float32Array(3 * NT * m), idx = [];
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
  for (let r = 0; r < m; r++) {
    const oa = 6 * gens[Math.max(0, r - 1)], ob = 6 * gens[Math.min(m - 1, r + 1)], o = 6 * gens[r];
    const ex = F[o + 3], ey = F[o + 4], ez = F[o + 5];
    const d = [0, 1, 2, 3, 4, 5].map(j => F[ob + j] - F[oa + j]);
    const ds = Math.hypot(...d) || 1;                // 六维弧长的增量
    const ax = (d[1] * ez - d[2] * ey) / ds, ay = (d[2] * ex - d[0] * ez) / ds, az = (d[0] * ey - d[1] * ex) / ds;   // A = P'×e
    const bx = (d[4] * ez - d[5] * ey) / ds, by = (d[5] * ex - d[3] * ez) / ds, bz = (d[3] * ey - d[4] * ex) / ds;   // B = e'×e
    for (let k = 0; k < NT; k++) {
      const t = -T_MAX + 2 * T_MAX * k / (NT - 1), w = 3 * (r * NT + k);
      nrm[w] = ax + t * bx; nrm[w + 1] = ay + t * by; nrm[w + 2] = az + t * bz;
    }
  }
  const ext = [];
  if (!reach)
    for (let r = 1; r < gens.length; r++) {
      const ha = hitSphere(F.subarray(6 * gens[r - 1], 6 * gens[r - 1] + 6)), hb = hitSphere(F.subarray(6 * gens[r], 6 * gens[r] + 6));
      if (ha && hb && Math.hypot(ha[0] - hb[0], ha[1] - hb[1], ha[2] - hb[2]) < 0.2) ext.push(...ha, ...hb);
    }
  return { reach, pos, nrm, idx: Uint32Array.from(idx), ext: Float32Array.from(ext) };
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
  const pieces = runs.map(r => ({ ...pieceMesh(F, r.gens, r.reach), region: r.reach ? reachRuns.indexOf(r) + 1 : 0, loop: 1, nLoops: 1, seg: 0,
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
 * 2DOF 的准备 (每条光路一次, 所有边界一起): 每条边界的连线抽稀 (取向平面里 0.4°, 成立/不成立切换处的点都保留) 后切成片,
 * 再给可达片编号:
 *   区域 = 取向平面网格上成立格点的连通块 (regionLabeler, 按大小编号);
 *   环   = 成立区域的一圈边界: 直接在网格上把「成立 / 不成立」的分界描出来 (成立 = +1, 不成立 = −1 的等值线,
 *          两个周期角首尾相接), 每条闭合的分界线是一个环. 环面上一块区域常有好几圈边界 (Lowitz 尤其常见),
 *          每圈扫出一张单独的直纹曲面. 可达片归到离它中点最近的那条分界线.
 *          (不用「片的端点在角点处相接」来接环: 角点处常有好几段吸附到同一点, 会把不同的环连成一个.)
 *   离所有分界线都超过 2° 的片 (成立条比网格还细), 自成一环; 网格里找不到区域号的, 排成网格区域之后的新区域.
 *   焦散不是区域的边, 在区域里面: 它的可达片只标区域号 (loop = 0).
 *   边   = 一条边界在两个角点之间成立的那一段 (一片), 各自扫出一张看得出的曲面; 每块区域里的边按环走的顺序编号 1, 2, …
 *          (先按环号, 环内按离片中点最近的分界线点的序号), 焦散片接在后面编号.
 *   去重: 边界连线里同一段常出现好几次 (平移 360° 的周期副本、反向再走一遍、只覆盖一部分 — 取向平面视图拿它们拼周期图),
 *          曲面若照画会一层叠一层 (半透明越叠越实、共面打架). 按长短从长到短, 大部分点都落在已留片附近的算副本.
 * 返回 Map(边界名 → [{ uv, F, runs }]), runs 里可达片带 region / loop / nLoops / seg; 重复的片标 dup, 不出网格.
 */
export function ruledPrepare(P, m, fixedAng, lines, ori, perX, perY, causticName = 'J=0 焦散') {
  const labelAt = regionLabeler(ori, perX, perY);
  const fams = new Map();
  for (const [name, fl] of lines) {
    const polys = [];
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
      polys.push({ uv, F, runs: runsOf(F, ok, false) });
    }
    fams.set(name, polys);
  }
  // 去重 (见上)
  const TAU0 = 2 * Math.PI, q = 0.05 * Math.PI / 180;
  const ptKey = (u, v) => {
    if (perX) u = ((u % TAU0) + TAU0) % TAU0;
    if (perY) v = ((v % TAU0) + TAU0) % TAU0;
    return Math.round(u / q) + ',' + Math.round(v / q);
  };
  // 覆盖式去重: 每条边界里按长短从长到短, 一片若有 ≥ 80% 的点落在已留下的片 0.5° 以内 (按周期折回), 就是副本
  // (整段平移 360° 的、反向的、只覆盖一部分的、从角点出发绕回角点的 — 都算).
  const tolD = 0.5 * Math.PI / 180, cell = tolD;
  const wrapU = u => (perX ? ((u % TAU0) + TAU0) % TAU0 : u), wrapV = v => (perY ? ((v % TAU0) + TAU0) % TAU0 : v);
  const nCellU = Math.ceil(TAU0 / cell), nCellV = nCellU;
  for (const polys of fams.values()) {
    const items = [];
    for (const pl of polys) for (const r of pl.runs) items.push({ pl, r });
    items.sort((x, y) => y.r.gens.length - x.r.gens.length);
    const hash = [new Map(), new Map()];       // [不可达, 可达]: 格 → [u, v, …]
    const cellOf = (u, v) => [Math.floor(u / cell), Math.floor(v / cell)];
    const ck = (i, j) => (perX ? ((i % nCellU) + nCellU) % nCellU : i) + ',' + (perY ? ((j % nCellV) + nCellV) % nCellV : j);
    for (const { pl, r } of items) {
      const H = hash[r.reach ? 1 : 0];
      let cov = 0;
      for (const g of r.gens) {
        const u = wrapU(pl.uv[2 * g]), v = wrapV(pl.uv[2 * g + 1]), [i, j] = cellOf(u, v);
        let hit = false;
        for (let di = -1; di <= 1 && !hit; di++)
          for (let dj = -1; dj <= 1 && !hit; dj++) {
            const L = H.get(ck(i + di, j + dj));
            if (!L) continue;
            for (let k = 0; k < L.length; k += 2) {
              let du = u - L[k], dv = v - L[k + 1];
              if (perX) du -= TAU0 * Math.round(du / TAU0);
              if (perY) dv -= TAU0 * Math.round(dv / TAU0);
              if (du * du + dv * dv < tolD * tolD) { hit = true; break; }
            }
          }
        if (hit) cov++;
      }
      if (cov >= 0.8 * r.gens.length) { r.dup = true; continue; }
      for (const g of r.gens) {
        const u = wrapU(pl.uv[2 * g]), v = wrapV(pl.uv[2 * g + 1]), key = ck(...cellOf(u, v));
        if (!H.has(key)) H.set(key, []);
        H.get(key).push(u, v);
      }
    }
  }
  // 成立区域的分界线 (网格上 ±1 的等值线), 每条一个环; 区域号取线上各点附近成立格点所在的块 (多数)
  const sgn = new Float64Array(ori.valid.length);
  for (let i = 0; i < sgn.length; i++) sgn[i] = ori.valid[i] ? 1 : -1;
  const TAU = 2 * Math.PI;
  const loops = traceContours(sgn, ori.SCs, ori.SWs, perX, perY).map(c => {
    const votes = new Map();
    for (let i = 0; i < c.pts.length; i += 4) { const l = labelAt(c.pts[i][0], c.pts[i][1]); if (l) votes.set(l, (votes.get(l) || 0) + 1); }
    return { pts: c.pts, reg: votes.size ? [...votes.entries()].sort((x, y) => y[1] - x[1])[0][0] : 0, len: c.pts.length, used: 0 };
  });
  const d2 = (u, v, q) => {
    let du = u - q[0], dv = v - q[1];
    if (perX) du -= TAU * Math.round(du / TAU);
    if (perY) dv -= TAU * Math.round(dv / TAU);
    return du * du + dv * dv;
  };
  const far = (2 * Math.PI / 180) ** 2;
  let nextReg = Math.max(0, ...loops.map(l => l.reg)), solo = [];
  for (const [name, polys] of fams)
    for (const pl of polys)
      for (const r of pl.runs) {
        if (!r.reach || r.dup) continue;
        const g = r.gens[r.gens.length >> 1], u = pl.uv[2 * g], v = pl.uv[2 * g + 1];
        if (name === causticName) { Object.assign(r, { region: labelAt(u, v), loop: 0, nLoops: 0 }); continue; }
        let best = Infinity, bl = null, bi = 0;
        for (const L of loops) L.pts.forEach((q, i) => { const d = d2(u, v, q); if (d < best) { best = d; bl = L; bi = i; } });
        if (!bl || best > far) {                       // 比网格还细的成立条: 自成一环
          bl = { pts: [[u, v]], reg: labelAt(u, v), len: r.gens.length, used: 0 };
          loops.push(bl); solo.push(bl); bi = 0;
        }
        bl.used++;
        r._loop = bl; r._at = bi;
      }
  for (const L of solo) if (!L.reg) L.reg = ++nextReg;
  // 每块区域里用到的环按长短编号
  const byReg = new Map();
  for (const L of loops) {
    if (!L.used) continue;
    if (!L.reg) L.reg = ++nextReg;
    if (!byReg.has(L.reg)) byReg.set(L.reg, []);
    byReg.get(L.reg).push(L);
  }
  for (const ls of byReg.values()) {
    ls.sort((x, y) => y.len - x.len);
    ls.forEach((L, j) => { L.no = j + 1; L.n = ls.length; });
  }
  // 一条边常被切成几片: 连线在取向平面 0/360° 接缝处断开, 或在母线跳变处断开. 这些断点上只有这一条边界的两片相接
  // (不是角点 — 角点上还有别的边界的片), 把它们并成一条边; 跳变处断开的相邻两片也并.
  const all = [];
  for (const [name, polys] of fams)
    for (const pl of polys) {
      let prev = null;
      for (const r of pl.runs) {
        if (r._loop) { Object.assign(r, { region: r._loop.reg, loop: r._loop.no, nLoops: r._loop.n }); delete r._loop; }
        if (!r.reach || r.dup) { prev = null; continue; }
        const ends = [ptKey(pl.uv[2 * r.gens[0]], pl.uv[2 * r.gens[0] + 1]), ptKey(pl.uv[2 * r.gens[r.gens.length - 1]], pl.uv[2 * r.gens[r.gens.length - 1] + 1])];
        const it = { r, name, ends, i: all.length, pl };
        if (prev && prev.r.gens[prev.r.gens.length - 1] + 1 === r.gens[0]) it.joinPrev = prev.i;
        all.push(it); prev = it;
      }
    }
  const atKey = new Map();                       // 端点键 → [片序号]
  for (const it of all) for (const k of it.ends) { if (!atKey.has(k)) atKey.set(k, []); atKey.get(k).push(it.i); }
  const up = all.map((_, i) => i);
  const root = a => { while (up[a] !== a) a = up[a] = up[up[a]]; return a; };
  for (const it of all) if (it.joinPrev !== undefined) up[root(it.i)] = root(it.joinPrev);
  for (const ids of atKey.values()) {
    if (ids.length !== 2) continue;
    const A = all[ids[0]], B = all[ids[1]];
    if (A.name === B.name && A !== B && A.r.region === B.r.region) up[root(A.i)] = root(B.i);
  }
  // 角点处吸附出来的碎片 (不到 6 条母线) 并进同一条边界、同一块区域里离它最近的那条边
  const sizeOf = new Map();
  for (const it of all) sizeOf.set(root(it.i), (sizeOf.get(root(it.i)) || 0) + it.r.gens.length);
  const near2 = (u, v, uu, vv) => {
    let du = u - uu, dv = v - vv;
    if (perX) du -= TAU0 * Math.round(du / TAU0);
    if (perY) dv -= TAU0 * Math.round(dv / TAU0);
    return du * du + dv * dv;
  };
  const endUV = it => { const pl = it.pl, g0 = it.r.gens[0], g1 = it.r.gens[it.r.gens.length - 1]; return [[pl.uv[2 * g0], pl.uv[2 * g0 + 1]], [pl.uv[2 * g1], pl.uv[2 * g1 + 1]]]; };
  for (const it of all) {
    if (sizeOf.get(root(it.i)) >= 6) continue;
    let best = Infinity, to = -1;
    for (const jt of all) {
      if (jt.name !== it.name || jt.r.region !== it.r.region || root(jt.i) === root(it.i) || sizeOf.get(root(jt.i)) < 6) continue;
      for (const p of endUV(it)) for (const q of endUV(jt)) { const d = near2(p[0], p[1], q[0], q[1]); if (d < best) { best = d; to = jt.i; } }
    }
    if (to >= 0) up[root(it.i)] = root(to);
  }
  const groups = new Map();                      // 区域号 → Map(根 → [片])
  for (const it of all) {
    const reg = it.r.region || 0;
    if (!groups.has(reg)) groups.set(reg, new Map());
    const g = groups.get(reg), k = root(it.i);
    if (!g.has(k)) g.set(k, []);
    g.get(k).push(it.r);
  }
  for (const g of groups.values()) {
    const list = [...g.values()].map(rs => ({ rs, loop: Math.min(...rs.map(r => (r.loop === 0 ? 1e9 : r.loop))),
                                               at: Math.min(...rs.map(r => r._at ?? 1e9)), len: rs.reduce((t, r) => t + r.gens.length, 0) }));
    list.sort((x, y) => x.loop - y.loop || x.at - y.at || y.len - x.len);
    list.forEach((e, i) => { for (const r of e.rs) { r.seg = i + 1; delete r._at; } });
  }
  return fams;
}

/** 2DOF: 某条边界 (已在 ruledPrepare 里切好片、编好号) 的网格. 返回 { pieces: [{ reach, pos, nrm, idx, ext, region, loop, nLoops, seg }] } */
export function ruledFamPieces(prep, name) {
  const pieces = [];
  for (const pl of prep.get(name) || [])
    for (const r of pl.runs)
      if (!r.dup) pieces.push({ ...pieceMesh(pl.F, r.gens, r.reach), region: r.region || 0, loop: r.loop ?? 1, nLoops: r.nLoops ?? 1, seg: r.seg || 0 });
  return { pieces };
}
