// 真实六棱柱孔径层 — HaloAperture.cs 的 JS 移植 (光点法的"第二族篱笆", 纯几何闭式, 无采样).
//   横截窗口 = 镜像展开的 σ(V) 区间交;  轴向 = slab 展开 + 楼层公式;
//   margin(w) > 0 ⟺ 该取向下存在入射点走通整条 (含基面插入次序的) 光路.
// 支持: 入射/出射为柱面 3-8, 中间任意柱面反弹 + 基面 1/2 交替插入; 锥面/基面进出不支持 (返回 NaN).
// 长度单位 = dist=1 正六棱柱的外接半径; 面距 r_f = (√3/2)·dist[f-3]; 底面 z = ±H (总高 2H).

import { LP, K, BETA, OMode, orientEuler } from './lp.js';

export const AP = {
  on: false,
  dist: [1, 1, 1, 1, 1, 1],       // 面 3..8
  H: 1.0,                         // 半高
};

const Sq32 = 0.8660254037844386;  // √3/2
const RING = [3, 8, 7, 6, 5, 4];  // 逆时针棱序 (按 Beta 角)

/** margin 封顶 (长度量纲): 只压幅值、不动零集与符号, 让 margin 成为良态场. */
export function apCap() { return 4.0 * (2.0 * AP.H + 2.0); }

function clampCap(v) {
  const c = apCap();
  return v > c ? c : (v < -c ? -c : v);
}

/** 孔径层是否覆盖该光路 (柱面进出, 中间柱面/基面, 基面全程 1/2 交替). */
export function apSupports(path) {
  if (!path || path.length < 2) return false;
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

/** margin 的 w 版本 (晶体系). path 需已通过 apSupports. */
export function apMarginW(path, w1, w2, w3) {
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
