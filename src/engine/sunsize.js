// 有限大小的太阳 (视半径 r, 真实太阳 ≈ 0.27°).
//
// 解决什么问题: 引擎默认太阳是一个点, 每条边界都是一条无限细的线. 真实太阳是个小圆盘, 盘上每一点都各自画出一套晕,
// 叠起来以后每条边界都变成一条「半影带」: 带外一点光都没有, 带内整个日盘都照得到, 带里亮度从 0 渐变到满.
//
// 用什么工具: 太阳挪一点 δS (天球上的一个小切向量), 边界线沿自己的法向挪 v = g·δS (一阶).
// 日盘上所有 δS (|δS| ≤ r) 叠起来, 法向挪动的最大 / 最小值就是 ±r·|g|, 所以半影带 = 边界线沿法向 ±r·|g|.
// g 是边界线对太阳方向的「法向速度」, 逐点算 (纯几何, 不用重算整条线):
//   边界在取向平面里是 F(u; S) = 0, 天空方向是 D(u; S).
//   太阳挪 δS: 取向平面里的边界跟着挪 δu = −∇F (∂F/∂S·δS) / |∇F|², 天空里的点挪 δD = ∂D/∂S·δS + ∂D/∂u·δu,
//   v = n·δD (n = 天空里边界线的法向). 焦散 (F = J) 处 n·∂D/∂u = 0, 公式自动退化成包络定理 v = n·∂D/∂S.
// 太阳沿地平经度挪, 整片晕绕天顶刚性转 (晶体取向的分布对方位对称) — 这一分量是精确的, 用来对账.
// 一阶近似: 误差 ~ r²·|∂g/∂S|; 真实太阳 r = 0.0047 rad 时 ≈ 0.001°. 末端点附近 g 不光滑, 带子在那里自己合拢.

import { LP, K, famValue, chain3, newC3, orientOf } from './lp.js';

const HU = 1e-6, HS = 1e-6;

function unit(d) { const n = Math.hypot(d[0], d[1], d[2]); return [d[0] / n, d[1] / n, d[2] / n]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/**
 * 天空方向 (单位向量), 但出射那一步的 √(qE² − K) 项不加: D̃ = P − qE·u_E.
 * 出射临界边界 (qE = −√K) 上 D̃ = D, 而 D 本身在那里有平方根折点 (导数无穷大), 数值求导要用 D̃;
 * 别的族用 D̃ 也行 — 只要在边界线上两者相等, 线的像和它随太阳的挪动就一样. 平行光路 (D = L·S) 没有 √ 项.
 */
const _E = new Float64Array(9);
function skyDs(P, om, fx, u, v, c, smooth) {
  chain3(u, v, om, fx, P, c);
  let x = c.Dx, y = c.Dy, z = c.Dz;
  if (smooth && !P.zw) {
    const e = orientOf(om, fx, u, v, _E), cf = P.coef, j = 3 * (P.faces.length - 1);
    const ux = cf[j] * e[0] + cf[j + 1] * e[3] + cf[j + 2] * e[6];
    const uy = cf[j] * e[1] + cf[j + 1] * e[4] + cf[j + 2] * e[7];
    const uz = cf[j] * e[2] + cf[j + 1] * e[5] + cf[j + 2] * e[8];
    const s = Math.sqrt(Math.max(c.qE * c.qE - K, 0));
    x += s * ux; y += s * uy; z += s * uz;
  }
  return unit([x, y, z]);
}

/** 在太阳方向 S 下求值 (临时换 LP.A3) */
function atSun(S, f) { const A = LP.A3; LP.A3 = S; try { return f(); } finally { LP.A3 = A; } }

/** 太阳处的单位切向基: eh 朝太阳高度增加, ea 朝方位增加 (A3 = (cos h, 0, sin h)) */
export function sunBasis() {
  const A = LP.A3;
  return [[-A[2], 0, A[0]], [0, 1, 0]];
}
function sunPM(e) {
  const A = LP.A3;
  return [unit([A[0] + HS * e[0], A[1] + HS * e[1], A[2] + HS * e[2]]),
          unit([A[0] - HS * e[0], A[1] - HS * e[1], A[2] - HS * e[2]])];
}

/**
 * 边界线上一点 (取向 u, v; 族 fm) 的法向速度. 返回 { D, n, g: [gh, ga] } 或 null (退化点).
 * 半影带: D 沿 ±n 转 r·|g|.
 */
export function boundaryGrad(fm, P, om, fx, u, v) {
  const F = (a, b) => famValue(fm, P, om, fx, a, b);
  const c = newC3(P), smooth = fm.kind === 3;                          // 出射临界: 用 D̃ 求导
  const Dd = (a, b) => skyDs(P, om, fx, a, b, c, smooth);
  const D0 = Dd(u, v);
  const Fu = (F(u + HU, v) - F(u - HU, v)) / (2 * HU), Fv = (F(u, v + HU) - F(u, v - HU)) / (2 * HU);
  const g2 = Fu * Fu + Fv * Fv;
  if (!(g2 > 0) || !Number.isFinite(g2)) return null;
  const Dp = Dd(u + HU, v), Dm = Dd(u - HU, v), Dq = Dd(u, v + HU), Dn = Dd(u, v - HU);
  const Du = [0, 1, 2].map(i => (Dp[i] - Dm[i]) / (2 * HU)), Dv = [0, 1, 2].map(i => (Dq[i] - Dn[i]) / (2 * HU));
  const T = [0, 1, 2].map(i => -Fv * Du[i] + Fu * Dv[i]);              // 边界切向 (取向平面的切向 (−Fv, Fu) 映到天空)
  const nr = cross(D0, T), nn = Math.hypot(nr[0], nr[1], nr[2]);
  if (!(nn > 1e-4 * Math.sqrt(g2))) return null;                      // 像几乎不动 (整段线缩成一点, 如汇聚点): 法向没有定义
  const n = [nr[0] / nn, nr[1] / nn, nr[2] / nn];
  const g = sunBasis().map(e => {
    const [Sp, Sm] = sunPM(e);
    const [fp, dp] = atSun(Sp, () => [F(u, v), Dd(u, v)]);
    const [fm_, dm] = atSun(Sm, () => [F(u, v), Dd(u, v)]);
    const dF = (fp - fm_) / (2 * HS);
    const s = -dF / g2, du = s * Fu, dv = s * Fv;
    let w = 0;
    for (let i = 0; i < 3; i++) w += n[i] * ((dp[i] - dm[i]) / (2 * HS) + Du[i] * du + Dv[i] * dv);
    return w;
  });
  if (!g.every(Number.isFinite)) return null;
  return { D: D0, n, g };
}

/**
 * 一自由度的弧 (板晶 / Parry): 弧上一点 = 取向参数 t 的像. 有限太阳把它加粗成一条带, 半宽 r·|g|, g = n·∂D/∂S.
 * pt(t) 返回 (u, v); 返回 { D, n, g } 或 null.
 */
export function arcGrad(P, om, fx, u, v, du, dv) {
  const c = newC3(P);
  const Dd = (a, b) => { chain3(a, b, om, fx, P, c); return unit([c.Dx, c.Dy, c.Dz]); };
  const D0 = Dd(u, v);
  const Dp = Dd(u + HU * du, v + HU * dv), Dm = Dd(u - HU * du, v - HU * dv);
  const T = [0, 1, 2].map(i => (Dp[i] - Dm[i]) / (2 * HU));
  const nr = cross(D0, T), nn = Math.hypot(nr[0], nr[1], nr[2]);
  if (!(nn > 1e-14)) return null;
  const n = [nr[0] / nn, nr[1] / nn, nr[2] / nn];
  const g = sunBasis().map(e => {
    const [Sp, Sm] = sunPM(e);
    const dp = atSun(Sp, () => Dd(u, v)), dm = atSun(Sm, () => Dd(u, v));
    return dot(n, [0, 1, 2].map(i => (dp[i] - dm[i]) / (2 * HS)));
  });
  return { D: D0, n, g };
}

/** D 沿 n 转角 a (弧度) */
export function offsetDir(D, n, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c * D[0] + s * n[0], c * D[1] + s * n[1], c * D[2] + s * n[2]];
}

// ---------------------------------------------------------------- 精确包络 (不做一阶近似)
// 一阶公式在太阳很大 (≳1°) 或边界对太阳弯得厉害的地方会偏. 精确版: 沿基点 x0 的法向大圆,
// 对日盘边缘每个位置 S_θ 求「S_θ 下的同一条边界线」与这条大圆的交点 (二维牛顿: F = 0 且 D·b0 = 0),
// 交点沿法向的角距 s(θ) 对 θ 取最大 / 最小, 就是并集的外沿 / 全日盘区的内沿. θ 从一阶最优方向 atan2(ga, gh) 起找.

function limbSun(r, th) {
  const A = LP.A3, [eh, ea] = sunBasis(), c = Math.cos(r), s = Math.sin(r), ct = Math.cos(th), st = Math.sin(th);
  return [0, 1, 2].map(i => c * A[i] + s * (ct * eh[i] + st * ea[i]));
}

/** 在太阳 S 下, 从 (u, v) 出发解 F = 0 且 D·b = 0; 返回 [u, v, D] 或 null */
function project2(F, Dd, S, u, v, b) {
  return atSun(S, () => {
    for (let it = 0; it < 12; it++) {
      const f = F(u, v), d = dot(Dd(u, v), b);
      if (!Number.isFinite(f) || !Number.isFinite(d)) return null;
      const fu = (F(u + HU, v) - F(u - HU, v)) / (2 * HU), fv = (F(u, v + HU) - F(u, v - HU)) / (2 * HU);
      const du_ = (dot(Dd(u + HU, v), b) - dot(Dd(u - HU, v), b)) / (2 * HU), dv_ = (dot(Dd(u, v + HU), b) - dot(Dd(u, v - HU), b)) / (2 * HU);
      const det = fu * dv_ - fv * du_;
      if (!(Math.abs(det) > 1e-14)) return null;
      const su = (f * dv_ - fv * d) / det, sv = (fu * d - f * du_) / det;
      u -= su; v -= sv;
      if (Math.abs(su) + Math.abs(sv) < 1e-12) break;
      if (Math.abs(su) + Math.abs(sv) > 0.15) return null;
    }
    const D = Dd(u, v);
    return Math.abs(F(u, v)) < 1e-8 && Math.abs(dot(D, b)) < 1e-9 ? [u, v, D] : null;
  });
}

/** 一自由度: 在太阳 S 下, 从参数 t 出发解 D(t)·b = 0; 返回 [t, D] 或 null */
function project1(Dt, S, t, b) {
  return atSun(S, () => {
    for (let it = 0; it < 12; it++) {
      const d = dot(Dt(t), b), dd = (dot(Dt(t + HU), b) - dot(Dt(t - HU), b)) / (2 * HU);
      if (!(Math.abs(dd) > 1e-14)) return null;
      const st = d / dd; t -= st;
      if (Math.abs(st) < 1e-13) break;
      if (Math.abs(st) > 0.15) return null;
    }
    const D = Dt(t);
    return Math.abs(dot(D, b)) < 1e-9 ? [t, D] : null;
  });
}

/** s(θ) 在 θ0 附近取极值 (sign = +1 最大, −1 最小): 抛物线插值几轮. evalS(θ) 返回 s 或 NaN */
function extremum(evalS, th0, sign) {
  let a = th0 - 0.35, m = th0, c = th0 + 0.35;
  let fa = sign * evalS(a), fm = sign * evalS(m), fc = sign * evalS(c);
  if (![fa, fm, fc].every(Number.isFinite)) return NaN;
  for (let it = 0; it < 6; it++) {
    if (fa > fm || fc > fm) {                                           // 极值不在中间: 朝高的一边挪
      const step = fa > fc ? -(c - a) / 2 : (c - a) / 2;
      a += step; m += step; c += step;
      fa = sign * evalS(a); fm = sign * evalS(m); fc = sign * evalS(c);
      if (![fa, fm, fc].every(Number.isFinite)) return NaN;
      continue;
    }
    const den = (fa - 2 * fm + fc);
    if (!(Math.abs(den) > 1e-300)) break;
    const h = (c - a) / 2, x = m + 0.5 * h * (fa - fc) / den;
    const fx = sign * evalS(x);
    if (!Number.isFinite(fx)) break;
    const nh = Math.max(h / 4, 1e-4);
    if (fx >= fm) { m = x; fm = fx; } 
    a = m - nh; c = m + nh; fa = sign * evalS(a); fc = sign * evalS(c);
    if (![fa, fc].every(Number.isFinite)) break;
    if (h < 0.01) break;                                                // θ 准到 0.01: s 的相对误差 ~ (0.01)²/2
  }
  return sign * Math.max(fa, fm, fc);
}

/**
 * 半影带两沿 (精确). base = boundaryGrad / arcGrad 的结果, r = 日盘视半径 (弧度).
 * 返回 [外沿 s+, 内沿 s−] (沿 n 的带符号角距, 弧度); 找不到交点的一侧给一阶值.
 */
export function bandExact(base, r, solveAt) {
  const { D, n, g } = base, b = cross(D, n);
  const th0 = Math.atan2(g[1], g[0]), w = r * Math.hypot(g[0], g[1]);
  const win = Math.cos(Math.min(1, 4 * Math.max(w, r)));               // 交点必须在基点附近 (法向大圆可能和同一条线别处再相交)
  const evalS = th => {
    const hitD = solveAt(limbSun(r, th), b);
    return hitD && dot(hitD, D) > win ? Math.atan2(dot(hitD, n), dot(hitD, D)) : NaN;
  };
  const sp = extremum(evalS, th0, +1), sm = extremum(evalS, th0 + Math.PI, -1);
  return [Number.isFinite(sp) ? sp : w, Number.isFinite(sm) ? sm : -w];
}

/** 二自由度边界线上一点的精确半影带: [D, n, s+, s−] 或 null */
export function boundaryBand(fm, P, om, fx, u, v, r) {
  const base = boundaryGrad(fm, P, om, fx, u, v);
  if (!base) return null;
  const F = (a, b) => famValue(fm, P, om, fx, a, b);
  const c = newC3(P), smooth = fm.kind === 3;
  const Dd = (a, b) => skyDs(P, om, fx, a, b, c, smooth);
  let last = [u, v];
  const solveAt = (S, b) => {
    const h = project2(F, Dd, S, last[0], last[1], b) || project2(F, Dd, S, u, v, b);
    if (!h) return null;
    last = [h[0], h[1]];                                                // 下一个 θ 从这里起步, 牛顿两三步就收敛
    return h[2];
  };
  const [sp, sm] = bandExact(base, r, solveAt);
  return { D: base.D, n: base.n, g: base.g, sp, sm };
}

/** 一自由度弧上一点 (参数方向 (du, dv)) 的精确带 */
export function arcBand(P, om, fx, u, v, du, dv, r) {
  const base = arcGrad(P, om, fx, u, v, du, dv);
  if (!base) return null;
  const c = newC3(P);
  const Dt = t => { chain3(u + t * du, v + t * dv, om, fx, P, c); return unit([c.Dx, c.Dy, c.Dz]); };
  const solveAt = (S, b) => { const h = project1(Dt, S, 0, b); return h ? h[1] : null; };
  const [sp, sm] = bandExact(base, r, solveAt);
  return { D: base.D, n: base.n, g: base.g, sp, sm };
}

// ---------------------------------------------------------------- 整条线的半影带 (Worker 用)
const neg = a => [-a[0], -a[1], -a[2]];

/**
 * 一串连续的点 (每点给出 band(i) → {D, n, g} 一阶结果, exact(i) → 精确 {sp, sm} 或 null) → 带子两沿.
 * 一阶每点都算; 精确值每隔 stride 点算一次, 中间按「精确 − 一阶」的差线性插值 (差是二阶小量, 沿线光滑).
 * 法向 n 沿线保持同一侧 (像在尖点处切向反转时 n 会翻面, 翻回来).
 */
function bandRun(count, first, exact, r, stride) {
  const pts = [];
  let prevN = null;
  for (let i = 0; i < count; i++) {
    const B = first(i);
    if (!B) { pts.push(null); continue; }
    let { n } = B, flip = false;
    if (prevN && dot(prevN, n) < 0) { n = neg(n); flip = true; }
    prevN = n;
    const w = r * Math.hypot(B.g[0], B.g[1]);
    pts.push({ D: B.D, n, w, flip, cp: 0, cm: 0, have: false });
  }
  // 精确修正: 每 stride 点 (以及首尾) 算一次
  const idx = [];
  for (let i = 0; i < count; i++) if (pts[i] && (i % stride === 0 || i === count - 1)) idx.push(i);
  for (const i of idx) {
    const E = exact(i), p = pts[i];
    if (!E) continue;
    const sp = p.flip ? -E.sm : E.sp, sm = p.flip ? -E.sp : E.sm;
    const tol = 0.35 * p.w + 0.02 * r;                                  // 精确修正是二阶小量; 差太多 = 交点落到了别的分支 (末端点附近), 不用
    if (Math.abs(sp - p.w) > tol || Math.abs(sm + p.w) > tol) continue;
    p.cp = sp - p.w; p.cm = sm + p.w; p.have = true;
  }
  const have = idx.filter(i => pts[i].have);
  for (let k = 0; k + 1 < have.length; k++) {
    const a = have[k], b = have[k + 1];
    if (b - a > 4 * stride) continue;                                   // 中间断过 (末端点附近): 不插值
    for (let i = a + 1; i < b; i++) {
      const p = pts[i]; if (!p) continue;
      const t = (i - a) / (b - a);
      p.cp = pts[a].cp + t * (pts[b].cp - pts[a].cp); p.cm = pts[a].cm + t * (pts[b].cm - pts[a].cm);
    }
  }
  // 拆成连续段, 输出两沿 (Float32Array, xyz 连排)
  const out = [];
  let lo = [], hi = [];
  const flush = () => { if (lo.length >= 6) out.push({ lo: Float32Array.from(lo), hi: Float32Array.from(hi) }); lo = []; hi = []; };
  for (const p of pts) {
    if (!p) { flush(); continue; }
    hi.push(...offsetDir(p.D, p.n, p.w + p.cp));
    lo.push(...offsetDir(p.D, p.n, -p.w + p.cm));
  }
  flush();
  return out;
}

/**
 * 二自由度: 每个族的成立段 → 半影带. lines = Map(族名 → {par, sky}), fams = 族定义, r = 视半径 (弧度).
 * 返回 [{ name, runs: [{lo, hi}] }]
 */
export function bands2(P, om, fx, fams, lines, r, stride = 4) {
  const out = [];
  for (const fm of fams) {
    const fl = lines.get(fm.name); if (!fl) continue;
    const runs = [];
    const all = [...fl.par.map((p, k) => [p, fl.sky[k]]), ...(fl.conn ? fl.conn.par.map((p, k) => [p, fl.conn.sky[k]]) : [])];
    for (const [par, sky] of all) {
      const n = par.length / 2;
      let i = 0;
      while (i < n) {
        while (i < n && sky[4 * i + 3] < 0.5) i++;
        const s = i;
        while (i < n && sky[4 * i + 3] >= 0.5) i++;
        if (i - s < 2) continue;
        runs.push(...bandRun(i - s,
          k => boundaryGrad(fm, P, om, fx, par[2 * (s + k)], par[2 * (s + k) + 1]),
          k => boundaryBand(fm, P, om, fx, par[2 * (s + k)], par[2 * (s + k) + 1], r),
          r, stride));
      }
    }
    if (runs.length) out.push({ name: fm.name, runs });
  }
  return out;
}

/**
 * 一自由度 (板晶 / Parry): 沿取向参数走一圈, 成立的连续段 → 带. pick(t) 返回 (u, v); dir = 参数方向 (du, dv).
 */
export function bands1(P, om, fx, pick, dir, validFn, r, N = 4000, stride = 4) {
  const runs = [];
  let seg = [];
  const flush = () => {
    if (seg.length >= 3) {
      const S = seg;
      runs.push(...bandRun(S.length,
        k => arcGrad(P, om, fx, S[k][0], S[k][1], dir[0], dir[1]),
        k => arcBand(P, om, fx, S[k][0], S[k][1], dir[0], dir[1], r),
        r, stride));
    }
    seg = [];
  };
  for (let i = 0; i <= N; i++) {
    const [u, v] = pick(2 * Math.PI * i / N);
    if (validFn(u, v)) seg.push([u, v]); else flush();
  }
  flush();
  return runs;
}
