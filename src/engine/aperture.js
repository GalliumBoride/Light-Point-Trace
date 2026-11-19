// 真实晶体孔径层 (光点法的"第二族篱笆"): margin(w) > 0 ⟺ 该取向下存在入射点走通整条光路.
// 六棱柱 — HaloAperture.cs 的 JS 移植 (纯几何闭式, 无采样): 横截窗口 = 镜像展开的 σ(V) 区间交; 轴向 = slab 展开 + 楼层公式.
//   支持: 入射/出射为柱面 3-8, 中间任意柱面反弹 + 基面 1/2 交替插入; 锥面/基面进出不支持 (返回 NaN).
// 锥晶 (AP.pyr) — 通用凸多面体 (myown/真实晶体与光点法/04 篇 L3, generic_aperture.py): 六棱柱上下各接一段截头六棱锥,
//   锥面过柱面的上/下棱, 底面在 z = ±(H + 锥台高). 没有 slab 展开可用 (轴向不再平移对称), 改用精确线性规划:
//   入射点 s ∈ 入射面 (二维) → 每一腿的命中点对 s 仿射 (方向只依赖光点链); 每个命中点都要在其余各面的半空间里.
//   margin = max_s min_k (该约束到边界的距离, 入射面内量) — 孔径多边形的内切圆半径, 走不通时为负 (最小违背);
//   零集 = 孔径多边形缩成一点, 与 04 篇面积归零同一条线. 任意面序列 (锥面 / 基面进出) 都支持.
// 长度单位 = dist=1 正六棱柱的外接半径; 面距 r_f = (√3/2)·dist[f-3]; 柱身 z ∈ [−H, H].

import { LP, K, BETA, OMode, orientEuler } from './lp.js';

export const AP = {
  on: false,
  dist: [1, 1, 1, 1, 1, 1],       // 面 3..8
  H: 1.0,                         // 半高 (柱身)
  pyr: false,                     // 锥晶: 上下各接一段截头六棱锥
  hU: 0.5, hL: 0.5,               // 上 / 下锥台高 (柱身之外, 同一长度单位)
};

const D2R_ = Math.PI / 180;

/**
 * 晶体的半空间表 (晶体系, 面号同引擎): { faces, n: [[x,y,z]…] 单位外法向, r: 面距 }.
 * 锥面法向同 faceCoef (方位 = 下面那张柱面, 仰角 = 顶角/2); 锥面过柱面的上 (下) 棱 ⟹ r = cosδ·r_柱 + sinδ·H.
 * 锥台高超过锥顶高时底面不再是晶面 (平面不碰晶体), 用到它的光路自然走不通.
 */
export function crystalPlanes(dist, H, pyr, hU, hL, apexUp, apexLo) {
  const faces = [], n = [], r = [];
  const add = (f, v, d) => { faces.push(f); n.push(v); r.push(d); };
  for (let f = 3; f <= 8; f++) add(f, [Math.cos(BETA[f]), Math.sin(BETA[f]), 0], Sq32 * dist[f - 3]);
  add(1, [0, 0, 1], H + (pyr ? hU : 0));
  add(2, [0, 0, -1], H + (pyr ? hL : 0));
  if (pyr)
    for (const [off, apex, sg] of [[10, apexUp, 1], [20, apexLo, -1]]) {
      const d = apex / 2 * D2R_, c = Math.cos(d), s = Math.sin(d);
      for (let f = 3; f <= 8; f++) add(off + f, [Math.cos(BETA[f]) * c, Math.sin(BETA[f]) * c, sg * s], c * Sq32 * dist[f - 3] + s * H);
    }
  return { faces, n, r };
}

const Sq32 = 0.8660254037844386;  // √3/2
const RING = [3, 8, 7, 6, 5, 4];  // 逆时针棱序 (按 Beta 角)

/** margin 封顶 (长度量纲): 只压幅值、不动零集与符号, 让 margin 成为良态场. */
export function apCap() { return 4.0 * (2.0 * AP.H + (AP.pyr ? AP.hU + AP.hL : 0) + 2.0); }

function clampCap(v) {
  const c = apCap();
  return v > c ? c : (v < -c ? -c : v);
}

/** 孔径层是否覆盖该光路. 六棱柱: 柱面进出, 中间柱面/基面, 基面全程 1/2 交替. 锥晶 (通用引擎): 晶体上有的面都行. */
export function apSupports(path, pyr = AP.pyr) {
  if (!path || path.length < 2) return false;
  if (pyr) {
    for (let i = 0; i < path.length; i++) {
      const f = path[i];
      if (!(f === 1 || f === 2 || (f >= 3 && f <= 8) || (f >= 13 && f <= 18) || (f >= 23 && f <= 28))) return false;
      if (i > 0 && path[i - 1] === f) return false;
    }
    return true;
  }
  const f0 = path[0], fE = path[path.length - 1];
  if (f0 < 3 || f0 > 8 || fE < 3 || fE > 8) return false;
  let lastBasal = 0;
  for (let i = 1; i < path.length; i++) {
    const f = path[i];
    if (f === 1 || f === 2) {
      if (i === path.length - 1) return false;
      if (lastBasal === f) return false;
      lastBasal = f;
    } else if (f < 3 || f > 8) return false;
  }
  return true;
}

const _WE = new Float64Array(9);

/** 取向 → 太阳在晶体系的分量 w (单图卡). */
export function apWOf(m, fixedAng, sc, sw) {
  let e;
  if (m === OMode.ZenFree) e = orientEuler(sc, sw, fixedAng, _WE);
  else if (m === OMode.ZetaFixed) e = orientEuler(sc, fixedAng, sw, _WE);
  else e = orientEuler(fixedAng, sw, sc, _WE);
  const A = LP.A3;
  return [A[0] * e[0] + A[1] * e[1] + A[2] * e[2],
          A[0] * e[3] + A[1] * e[4] + A[2] * e[5],
          A[0] * e[6] + A[1] * e[7] + A[2] * e[8]];
}

/** 孔径裕度: >0 活, <0 死, NaN 不支持. 过零即孔径篱笆. P = PathCtx */
export function apMargin(P, m, fixedAng, sc, sw) {
  if (!AP.on || !P.apSupported) return NaN;
  const w = apWOf(m, fixedAng, sc, sw);
  return apMarginW(P.faces, w[0], w[1], w[2]);
}

/** 有效性门 (带容差: 篱笆自身残差 ±1e-16 量级, 严格 >0 会让篱笆自己闪烁成碎段). */
export function apOk(P, m, fixedAng, sc, sw) {
  if (!AP.on || !P.apSupported) return true;
  const v = apMargin(P, m, fixedAng, sc, sw);
  return Number.isNaN(v) || v > -1e-6 * Math.max(1.0, AP.H);
}

// 邻接顶点: 面 f 与其环上邻居 g 的交点 (旋转系), 写入 out[0..1]
function vertex(mx, my, r, fi, gi, out) {
  const det = mx[fi] * my[gi] - my[fi] * mx[gi];
  out[0] = (r[fi] * my[gi] - r[gi] * my[fi]) / det;
  out[1] = (mx[fi] * r[gi] - mx[gi] * r[fi]) / det;
  return out;
}

// 热路径 (每个网格点/采样点都调用): 预分配工作区, 免逐次分配 typed array
const MAXE = 64;
const _ps = new Int32Array(MAXE), _nB = new Int32Array(MAXE);
const _mx = new Float64Array(6), _my = new Float64Array(6), _rr = new Float64Array(6);
const _a0 = new Float64Array(MAXE), _a1 = new Float64Array(MAXE);
const _u0 = new Float64Array(MAXE + 1), _u1 = new Float64Array(MAXE + 1);
const _l0 = new Float64Array(MAXE + 1), _l1 = new Float64Array(MAXE + 1);
const _va = [0, 0], _vb = [0, 0], _v = [0, 0];
let _best = 0;

// ---------------------------------------------------------------- 锥晶: 通用凸多面体 + 线性规划
let _cry = null, _cryKey = '';
function crystal() {
  const key = [AP.dist.join(','), AP.H, AP.hU, AP.hL, LP.upperApexDeg, LP.lowerApexDeg].join(';');
  if (key !== _cryKey) {
    const c = crystalPlanes(AP.dist, AP.H, true, AP.hU, AP.hL, LP.upperApexDeg, LP.lowerApexDeg), F = c.faces.length;
    const idx = new Int32Array(32).fill(-1), N = new Float64Array(3 * F), R = new Float64Array(F);
    c.faces.forEach((f, i) => { idx[f] = i; N.set(c.n[i], 3 * i); R[i] = c.r[i]; });
    _cry = { F, idx, N, R, absent: new Uint8Array(F) };
    // 面积为零的面 (锥台高 0 时锥面缩成一条棱; 锥台高过锥顶时底面碰不到晶体) 不是晶面: 用到它的光路一律走不通.
    // 判法 = 面多边形的内切圆半径 (同一个线性规划, 入射点换成面上的点)
    for (let i = 0; i < F; i++) {
      const nx = N[3 * i], ny = N[3 * i + 1], nz = N[3 * i + 2];
      let ux = -ny, uy = nx, uz = 0, ul = Math.hypot(ux, uy);
      if (ul < 1e-9) { ux = 0; uy = -nz; uz = ny; ul = Math.hypot(uy, uz); }
      ux /= ul; uy /= ul; uz /= ul;
      _m = 0;
      addRows(_cry, [R[i] * nx, R[i] * ny, R[i] * nz], [ux, uy, uz], [ny * uz - nz * uy, nz * ux - nx * uz, nx * uy - ny * ux], i);
      if (!(lpMaxT() > 1e-9)) _cry.absent[i] = 1;
    }
    _cryKey = key;
  }
  return _cry;
}

/** 当前锥晶里哪些面不存在 (面积为零): 面号数组. 诊断 / 界面提示用 */
export function apAbsentFaces() {
  const cr = crystal(), out = [];
  for (let f = 0; f < 32; f++) if (cr.idx[f] >= 0 && cr.absent[cr.idx[f]]) out.push(f);
  return out;
}

const MAXROW = 2048;
const _A1 = new Float64Array(MAXROW), _A2 = new Float64Array(MAXROW), _Bv = new Float64Array(MAXROW);
let _m = 0;

/** 命中点 X(s) = C0 + s1·Cu + s2·Cv 要在除 skip 以外各面的半空间里: 每面一行 (距离化: 除以对 s 的梯度长) */
function addRows(cr, C0, Cu, Cv, skip) {
  const N = cr.N, R = cr.R;
  for (let g = 0; g < cr.F; g++) {
    if (g === skip || _m >= MAXROW) continue;
    const nx = N[3 * g], ny = N[3 * g + 1], nz = N[3 * g + 2];
    const a1 = nx * Cu[0] + ny * Cu[1] + nz * Cu[2], a2 = nx * Cv[0] + ny * Cv[1] + nz * Cv[2];
    const b = R[g] - (nx * C0[0] + ny * C0[1] + nz * C0[2]), L = Math.hypot(a1, a2);
    if (L > 1e-12) { _A1[_m] = a1 / L; _A2[_m] = a2 / L; _Bv[_m] = b / L; }
    else { _A1[_m] = 0; _A2[_m] = 0; _Bv[_m] = b; }               // 与入射点无关: 只看成不成立
    _m++;
  }
}

const _M = [new Float64Array(3), new Float64Array(3), new Float64Array(3)];
const _I = [new Float64Array(3), new Float64Array(3), new Float64Array(3)];
function inv3(M, I) {
  const [a, b, c] = M[0], [d, e, f] = M[1], [g, h, i] = M[2];
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
  if (!(Math.abs(det) > 1e-14)) return false;
  const k = 1 / det;
  I[0][0] = A * k; I[0][1] = -(b * i - c * h) * k; I[0][2] = (b * f - c * e) * k;
  I[1][0] = B * k; I[1][1] = (a * i - c * g) * k; I[1][2] = -(a * f - c * d) * k;
  I[2][0] = C * k; I[2][1] = -(a * h - b * g) * k; I[2][2] = (a * e - b * d) * k;
  return true;
}

/**
 * max t  s.t.  A1·s1 + A2·s2 + t ≤ B (每行), s 自由. 顶点跳跃 (单纯形, 三个变量): 基 = 三条紧约束;
 * 乘子 λ = (A_B⁻¹ 第三行); 全 ≥ 0 即最优, 否则把 λ<0 的那条放松, 沿 d = −A_B⁻¹ e_p 走到下一条挡住的约束. Bland 规则防循环.
 * 起点 s = 0, t = min B; 先用两条假约束 s1 ≤ 0、s2 ≤ 0 凑成顶点, 它们最先被换出 (方向可正可负, 都不降 t).
 */
function lpMaxT() {
  const m = _m, A1 = _A1, A2 = _A2, B = _Bv;
  let z0 = 0, z1 = 0, z2 = Infinity, k0 = -1;
  for (let k = 0; k < m; k++) if (B[k] < z2) { z2 = B[k]; k0 = k; }
  if (k0 < 0) return Infinity;
  const bas = [-1, -2, k0];
  for (let it = 0; it < 200; it++) {
    for (let r = 0; r < 3; r++) {
      const j = bas[r], M = _M[r];
      if (j === -1) { M[0] = 1; M[1] = 0; M[2] = 0; } else if (j === -2) { M[0] = 0; M[1] = 1; M[2] = 0; }
      else { M[0] = A1[j]; M[1] = A2[j]; M[2] = 1; }
    }
    if (!inv3(_M, _I)) return z2;
    const lam = _I[2];
    let p = -1, sigma = 1;
    for (let r = 0; r < 3; r++) if (bas[r] < 0) { p = r; sigma = lam[r] >= 0 ? -1 : 1; break; }
    if (p < 0) {
      let best = Infinity;
      for (let r = 0; r < 3; r++) if (lam[r] < -1e-12 && bas[r] < best) { best = bas[r]; p = r; }
      if (p < 0) return z2;                                        // KKT 成立: 最优
    }
    for (let tries = 0; ; tries++) {
      const d0 = -sigma * _I[0][p], d1 = -sigma * _I[1][p], d2 = -sigma * _I[2][p];
      let best = Infinity, kin = -1;
      for (let k = 0; k < m; k++) {
        if (k === bas[0] || k === bas[1] || k === bas[2]) continue;
        const ad = A1[k] * d0 + A2[k] * d1 + d2;
        if (ad <= 1e-13) continue;
        let r = (B[k] - (A1[k] * z0 + A2[k] * z1 + z2)) / ad;
        if (r < 0) r = 0;
        if (r < best - 1e-15) { best = r; kin = k; }
      }
      if (kin >= 0) { z0 += best * d0; z1 += best * d1; z2 += best * d2; bas[p] = kin; break; }
      if (bas[p] < 0 && tries === 0) { sigma = -sigma; continue; }   // 假约束往这边走不受挡: 换个方向
      return z2;
    }
  }
  return z2;
}

const _C0 = new Float64Array(3), _Cu = new Float64Array(3), _Cv = new Float64Array(3);
/** 锥晶 (通用凸多面体) 的 margin. 入射点 s 在入射面上 (原点 = 晶体中心在入射面上的投影). */
function genericMargin(path, w1, w2, w3) {
  const cr = crystal(), N = cr.N, R = cr.R, cap = apCap();
  const i0 = cr.idx[path[0]];
  if (i0 < 0) return NaN;
  for (const f of path) if (cr.absent[cr.idx[f]]) return -cap;      // 光路用到不存在的面
  const n0x = N[3 * i0], n0y = N[3 * i0 + 1], n0z = N[3 * i0 + 2];
  const g0 = w1 * n0x + w2 * n0y + w3 * n0z, t0 = -g0 + Math.sqrt(g0 * g0 + K);
  let Px = w1 + t0 * n0x, Py = w2 + t0 * n0y, Pz = w3 + t0 * n0z;    // 光点; 腿方向 = −P
  // 入射面内的正交基
  let ux = -n0y, uy = n0x, uz = 0, ul = Math.hypot(ux, uy);
  if (ul < 1e-9) { ux = 0; uy = -n0z; uz = n0y; ul = Math.hypot(uy, uz); }
  ux /= ul; uy /= ul; uz /= ul;
  const vx = n0y * uz - n0z * uy, vy = n0z * ux - n0x * uz, vz = n0x * uy - n0y * ux;
  const C0 = _C0, Cu = _Cu, Cv = _Cv;
  C0[0] = R[i0] * n0x; C0[1] = R[i0] * n0y; C0[2] = R[i0] * n0z;
  Cu[0] = ux; Cu[1] = uy; Cu[2] = uz; Cv[0] = vx; Cv[1] = vy; Cv[2] = vz;
  _m = 0;
  addRows(cr, C0, Cu, Cv, i0);
  const E = path.length - 1;
  for (let k = 1; k <= E; k++) {
    const i = cr.idx[path[k]];
    if (i < 0) return NaN;
    const nx = N[3 * i], ny = N[3 * i + 1], nz = N[3 * i + 2];
    const dd = -(nx * Px + ny * Py + nz * Pz);                      // n·d, d = −P
    if (dd <= 1e-12) return -cap;                                   // 这张面不在下游: 走不通
    const tc = (R[i] - (nx * C0[0] + ny * C0[1] + nz * C0[2])) / dd;
    const tu = -(nx * Cu[0] + ny * Cu[1] + nz * Cu[2]) / dd, tv = -(nx * Cv[0] + ny * Cv[1] + nz * Cv[2]) / dd;
    C0[0] -= tc * Px; C0[1] -= tc * Py; C0[2] -= tc * Pz;
    Cu[0] -= tu * Px; Cu[1] -= tu * Py; Cu[2] -= tu * Pz;
    Cv[0] -= tv * Px; Cv[1] -= tv * Py; Cv[2] -= tv * Pz;
    addRows(cr, C0, Cu, Cv, i);
    if (k < E) { const q = 2 * (Px * nx + Py * ny + Pz * nz); Px -= q * nx; Py -= q * ny; Pz -= q * nz; }
  }
  return clampCap(lpMaxT());
}

/** margin 的 w 版本 (晶体系). path 需已通过 apSupports. */
export function apMarginW(path, w1, w2, w3) {
  if (AP.pyr) return genericMargin(path, w1, w2, w3);
  const H = AP.H, Dist = AP.dist;
  const f0 = path[0];
  // ---- 柱面子序列 + 每棱之前的基面累计数 ----
  let E = 0;
  for (let i = 1; i < path.length; i++) if (path[i] >= 3) E++;
  if (E > MAXE) return NaN;
  const ps = _ps, nB = _nB;
  let firstBasal = 0, cnt = 0, k = 0;
  for (let i = 1; i < path.length; i++) {
    const f = path[i];
    if (f === 1 || f === 2) { if (firstBasal === 0) firstBasal = f; cnt++; }
    else { ps[k] = f; nB[k] = cnt; k++; }
  }

  // ---- 旋转到入射面法向 = (1,0) 的横截系 ----
  const b0 = BETA[f0];
  const cb = Math.cos(b0), sb = Math.sin(b0);
  const X = cb * w1 + sb * w2;
  const Yr = -sb * w1 + cb * w2;
  const Z = w3;
  const S = Math.sqrt(X * X + K);

  const mx = _mx, my = _my, rr = _rr;
  for (let f = 3; f <= 8; f++) {
    const ang = BETA[f] - b0;
    mx[f - 3] = Math.cos(ang); my[f - 3] = Math.sin(ang);
    rr[f - 3] = Sq32 * Dist[f - 3];
  }
  const r0 = rr[f0 - 3];

  // 入射棱 s 范围: f0 的两个环邻顶点的 y 坐标
  const ringPos = RING.indexOf(f0);
  const nbA = RING[(ringPos + 5) % 6], nbB = RING[(ringPos + 1) % 6];
  const va = vertex(mx, my, rr, f0 - 3, nbA - 3, _va);
  const vb = vertex(mx, my, rr, f0 - 3, nbB - 3, _vb);
  let sL = Math.min(va[1], vb[1]), sR = Math.max(va[1], vb[1]);

  // ---- 镜像展开: 窗口 σ 区间交 + 每棱转移时间 t_j(s) = a0 + a1 s ----
  let R11 = 1, R12 = 0, R21 = 0, R22 = 1, bx = 0, by = 0;
  const a0 = _a0, a1 = _a1;
  for (let j = 0; j < E; j++) {
    const fi = ps[j] - 3;
    const mtx = R11 * mx[fi] + R12 * my[fi];
    const mty = R21 * mx[fi] + R22 * my[fi];
    const rt = rr[fi] + mtx * bx + mty * by;
    const speed = -(mtx * S + mty * Yr);
    if (speed <= 1e-12) return -apCap();                 // 该棱不在下游 (接缝彼侧): 走不通
    const rp = RING.indexOf(ps[j]);
    let sMin = Number.MAX_VALUE, sMax = -Number.MAX_VALUE;
    for (let q = 0; q < 2; q++) {
      const g = RING[(rp + (q === 0 ? 5 : 1)) % 6];
      const v = vertex(mx, my, rr, fi, g - 3, _v);
      const tx = R11 * v[0] + R12 * v[1] + bx;
      const ty = R21 * v[0] + R22 * v[1] + by;
      const t = (r0 - tx) / S;
      const sg = ty + t * Yr;
      if (sg < sMin) sMin = sg;
      if (sg > sMax) sMax = sg;
    }
    if (sMin > sL) sL = sMin;
    if (sMax < sR) sR = sMax;
    a0[j] = (rt - mtx * r0) / speed;
    a1[j] = -mty / speed;
    const dxx = 1 - 2 * mtx * mtx, dxy = -2 * mtx * mty, dyy = 1 - 2 * mty * mty;
    const n11 = dxx * R11 + dxy * R21, n12 = dxx * R12 + dxy * R22;
    const n21 = dxy * R11 + dyy * R21, n22 = dxy * R12 + dyy * R22;
    const nbx = dxx * bx + dxy * by + 2 * rt * mtx;
    const nby = dxy * bx + dyy * by + 2 * rt * mty;
    R11 = n11; R12 = n12; R21 = n21; R22 = n22; bx = nbx; by = nby;
  }
  if (sL >= sR) return clampCap(-(sL - sR));

  // ---- 轴向楼层胞元 ----
  const nU = E + 1;
  const u0 = _u0, u1 = _u1, l0 = _l0, l1 = _l1;
  u0[0] = H; u1[0] = 0; l0[0] = -H; l1[0] = 0;
  for (let j = 0; j < E; j++) {
    const n = nB[j];
    let lo, hi;
    if (n === 0) { lo = -H; hi = H; }
    else if (firstBasal === 1) { lo = (2 * n - 1) * H; hi = (2 * n + 1) * H; }
    else { lo = -(2 * n + 1) * H; hi = -(2 * n - 1) * H; }
    u0[j + 1] = hi + Z * a0[j]; u1[j + 1] = Z * a1[j];
    l0[j + 1] = lo + Z * a0[j]; l1[j + 1] = Z * a1[j];
  }

  // ---- margin = max_s min( minU(s) - maxL(s), s - sL, sR - s ) (凹分段线性) ----
  _best = -Number.MAX_VALUE;
  evalCand(sL, sL, sR, nU, u0, u1, l0, l1);
  evalCand(sR, sL, sR, nU, u0, u1, l0, l1);
  evalCand(0.5 * (sL + sR), sL, sR, nU, u0, u1, l0, l1);
  for (let i = 0; i < nU; i++)
    for (let j = i + 1; j < nU; j++) {
      crossCand(u0[i], u1[i], u0[j], u1[j], sL, sR, nU, u0, u1, l0, l1);
      crossCand(l0[i], l1[i], l0[j], l1[j], sL, sR, nU, u0, u1, l0, l1);
    }
  for (let i = 0; i < nU; i++)
    for (let j = 0; j < nU; j++) {
      const g0c = u0[i] - l0[j], g1c = u1[i] - l1[j];
      crossCand(g0c, g1c, -sL, 1.0, sL, sR, nU, u0, u1, l0, l1);
      crossCand(g0c, g1c, sR, -1.0, sL, sR, nU, u0, u1, l0, l1);
    }
  return clampCap(_best);
}

function crossCand(p0, p1, q0, q1, sL, sR, nU, u0, u1, l0, l1) {
  const d = p1 - q1;
  if (Math.abs(d) < 1e-14) return;
  const s = (q0 - p0) / d;
  if (s <= sL || s >= sR) return;
  evalCand(s, sL, sR, nU, u0, u1, l0, l1);
}

function evalCand(s, sL, sR, nU, u0, u1, l0, l1) {
  let mu = Number.MAX_VALUE, ml = -Number.MAX_VALUE;
  for (let i = 0; i < nU; i++) {
    const uv = u0[i] + u1[i] * s;
    const lv = l0[i] + l1[i] * s;
    if (uv < mu) mu = uv;
    if (lv > ml) ml = lv;
  }
  let v = mu - ml;
  const slackA = s - sL, slackB = sR - s;
  if (slackA < v) v = slackA;
  if (slackB < v) v = slackB;
  if (v > _best) _best = v;
}
