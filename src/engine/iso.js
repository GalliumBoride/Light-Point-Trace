// 等光强线: 天空点 x 的光强 = 落到 x 的全部姿态之和
//   I(x) = (1/4π²) · Σ_{f(q) = x} w(q)/|J(q)|      (每球面度、每颗晶体; 族的角坐标体积元为常数, 均匀取向 = (u,v) 上均匀)
// w: 'geo' 只算几何密度 (= 1); 'fresnel' = 入射面投影面积 g0 × 入射/内反射/出射的 Fresnel 透过/反射率 (非偏振, 逐界面),
//    全由光点法的法向动量给出: 入射 (g0, S0), 内反射 q_i, 出射 (|qE|, TE); r_s = (a−b)/(a+b), r_p = (n²a−b)/(n²a+b).
// J: 光点法雅可比 (jana3). 等光强线 = { x : I(x) = c }, 在天空里沿曲线走 (预测-校正), 不用像素、不用网格.
//
// 落到 x 的全部姿态 (纤维) — 光路展开 = 虚楔 (入射面 + 镜像后的出射面, 棱 L) + 镜像组合 M:
//   C · 族绕体轴 a 自旋封闭, L ∥ a, M 与绕 a 转动对易 (柱晶 × 棱柱楔, Lowitz × 基面-棱柱楔):
//       沿棱守恒定另一个角 (三角方程), 垂直棱平面是棱镜 (入射角由二次方程给出), 全闭式.
//   A · 虚楔两面平行 (3-5-7 镜面、3-4-1-5 半周、3-5-6-8 转动 …): 出射只依赖 M 的轴的世界方向 m, m 闭式, 再解姿态. 全闭式.
//   B · 其余楔 (Lowitz 3-5、锥晶 …): 沿棱守恒给出一条闭式曲线, 剩一个方程沿它一维求根.
// 追线时纤维随 x 用牛顿跟着走 (隐函数定理), 每一步用上面的完整求解核对一次 (姿态只在篱笆/焦散的像上出生/消失);
// 光强在焦散上跳变时曲线在那里终止. 起点: 从焦散 (没有焦散时用篱笆) 的天空像沿法向走, 二分到各级.

import { LP, K, N_ICE, newC3, chain3, jana3, validAt, skyD, orientOf, faceCoef, axesMeta, OMode } from './lp.js';
import { AP } from './aperture.js';
import { fisheyeOf } from './lift.js';

const TAU = 2 * Math.PI, D2R = Math.PI / 180;
const wrapPi = x => x - TAU * Math.floor((x + Math.PI) / TAU);
const wrap2Pi = x => x - TAU * Math.floor(x / TAU);
const dot = (p, q) => p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
const cross = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
const nrm = v => { const m = Math.hypot(v[0], v[1], v[2]); return [v[0] / m, v[1] / m, v[2] / m]; };
const refl = v => [[1 - 2 * v[0] * v[0], -2 * v[0] * v[1], -2 * v[0] * v[2]], [-2 * v[1] * v[0], 1 - 2 * v[1] * v[1], -2 * v[1] * v[2]], [-2 * v[2] * v[0], -2 * v[2] * v[1], 1 - 2 * v[2] * v[2]]];
const mul = (X, Y) => X.map(r => [0, 1, 2].map(j => r[0] * Y[0][j] + r[1] * Y[1][j] + r[2] * Y[2][j]));
const mv = (X, v) => X.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
const mtv = (X, v) => [0, 1, 2].map(j => X[0][j] * v[0] + X[1][j] * v[1] + X[2][j] * v[2]);
const det3 = X => X[0][0] * (X[1][1] * X[2][2] - X[1][2] * X[2][1]) - X[0][1] * (X[1][0] * X[2][2] - X[1][2] * X[2][0]) + X[0][2] * (X[1][0] * X[2][1] - X[1][1] * X[2][0]);
const rotM = (a, t) => { const c = Math.cos(t), s = Math.sin(t), C = 1 - c, [x, y, z] = a;
  return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]; };

/** Fresnel 透过率 (非偏振): a = 介质 1 侧法向动量 n1·cosθ1, b = 介质 2 侧 n2·cosθ2 */
export function fresnelT(a, b, n1, n2) {
  const rs = (a - b) / (a + b), rp = (n2 * n2 * a - n1 * n1 * b) / (n2 * n2 * a + n1 * n1 * b);
  return 1 - 0.5 * (rs * rs + rp * rp);
}

function axisAngle(Rm) {
  const c = Math.max(-1, Math.min(1, (Rm[0][0] + Rm[1][1] + Rm[2][2] - 1) / 2)), th = Math.acos(c);
  if (th < 1e-9) return { u: [0, 0, 1], th: 0 };
  if (Math.PI - th < 1e-6) {
    const B = [0, 1, 2].map(i => [0, 1, 2].map(j => (Rm[i][j] + (i === j ? 1 : 0)) / 2));
    const k = [0, 1, 2].reduce((a, i) => (B[i][i] > B[a][a] ? i : a), 0);
    return { u: nrm(B[k]), th: Math.PI };
  }
  return { u: nrm([Rm[2][1] - Rm[1][2], Rm[0][2] - Rm[2][0], Rm[1][0] - Rm[0][1]]), th };
}

/** 一条光路 × 当前族的光强模型: 权重、纤维求解、天空映射 */
export function makeIso(P, om, fx, weight = 'fresnel') {
  const n = N_ICE, c3 = newC3(P), A = Array.from(LP.A3), ax = axesMeta(om);
  const y0 = ax.y0 * D2R, y1 = ax.y1 * D2R;
  const vOK = v => ax.perY || (wrap2Pi(v) >= y0 - 1e-9 && wrap2Pi(v) <= y1 + 1e-9);
  const ok = (u, v) => vOK(v) && validAt(P, om, fx, u, v, 1e-12);
  const fo = [0, 0];
  const fish = (u, v, out) => { const D = skyD(P, om, fx, u, v); return fisheyeOf(D[0], D[1], D[2], out); };
  const skyUnit = (u, v) => { const D = skyD(P, om, fx, u, v), m = Math.hypot(D[0], D[1], D[2]); return [D[0] / m, D[1] / m, D[2] / m]; };
  const wOf = (u, v) => {
    if (weight === 'geo') return 1;
    chain3(u, v, om, fx, P, c3);
    const g0 = c3.g0, S0 = Math.sqrt(g0 * g0 + K), qE = c3.qE, TE = Math.sqrt(Math.max(0, qE * qE - K));
    let w = g0 * fresnelT(g0, S0, 1, n) * fresnelT(-qE, TE, n, 1);
    for (let i = 0; i < c3.qs.length; i++) { const q = c3.qs[i]; if (q * q > K) w *= 1 - fresnelT(Math.abs(q), Math.sqrt(q * q - K), n, 1); }
    return w;
  };
  const wJ = (u, v) => wOf(u, v) / Math.abs(jana3(u, v, om, fx, P));
  const g0At = (u, v) => { chain3(u, v, om, fx, P, c3); return c3.g0; };

  // ---------- 光路展开 ----------
  const N = P.faces.map(f => faceCoef(f));
  let M = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let i = 1; i < N.length - 1; i++) M = mul(refl(N[i]), M);
  const nbv = mtv(M, N[N.length - 1]), Lraw = cross(N[0], nbv), Ln = Math.hypot(Lraw[0], Lraw[1], Lraw[2]);
  const e = new Float64Array(9);
  const Rt = (u, v, t) => { orientOf(om, fx, u, v, e); return [t[0] * e[0] + t[1] * e[3] + t[2] * e[6], t[0] * e[1] + t[1] * e[4] + t[2] * e[7], t[0] * e[2] + t[1] * e[5] + t[2] * e[8]]; };
  const accept = (out, x, u, v) => {
    if (!Number.isFinite(u) || !Number.isFinite(v) || !ok(u, v)) return;
    const y = skyUnit(u, v);
    if (!(Math.hypot(x[0] - y[0], x[1] - y[1], x[2] - y[2]) < 1e-7)) return;
    u = wrapPi(u); v = wrapPi(v);
    if (out.some(o => Math.abs(wrapPi(o[0] - u)) < 1e-5 && Math.abs(wrapPi(o[1] - v)) < 1e-5)) return;   // 两条闭式路线解出同一姿态
    out.push([u, v]);
  };
  const spinAxis = om === OMode.ZetaFixed ? [0, 0, 1] : [Math.cos(fx), -Math.sin(fx), 0];
  let kind, fiberRaw;

  if (Ln < 1e-9) {
    // ---------- A: 虚楔平行 ----------
    const dM = det3(M), rot = dM > 0 ? M : M.map(r => r.map(y => -y));
    const { u: ub, th: th0 } = axisAngle(rot);
    const th = dM > 0 ? th0 : th0 + Math.PI, type = dM > 0 ? (th0 < 1e-9 ? 'identity' : 'rotation') : 'rotoreflection';
    kind = 'A';
    const orientFor = (t, m, out, x) => {
      const lz = v => Rt(0, v, t)[2];
      const z0 = lz(0), zpi = lz(Math.PI), p = 0.5 * (z0 + zpi), q = 0.5 * (z0 - zpi), r = lz(Math.PI / 2) - p, C = Math.hypot(q, r);
      if (C < 1e-10) return 'degenerate';
      const cv = (m[2] - p) / C;
      if (Math.abs(cv) > 1 + 1e-12) return;
      const ph = Math.atan2(r, q), ac = Math.acos(Math.max(-1, Math.min(1, cv)));
      for (const v of [ph + ac, ph - ac]) { const l = Rt(0, v, t); accept(out, x, Math.atan2(m[1], m[0]) - Math.atan2(l[1], l[0]), v); }
    };
    fiberRaw = x => {
      const out = [];
      if (type === 'identity') return out;
      const w = dM > 0 ? [x[0] - A[0], x[1] - A[1], x[2] - A[2]] : [x[0] + A[0], x[1] + A[1], x[2] + A[2]];
      if (Math.hypot(w[0], w[1], w[2]) < 1e-10) return out;
      const ct = Math.cos(th), Ax = dot(A, x), h2 = dM > 0 ? (Ax - ct) / (1 - ct) : (ct - Ax) / (1 + ct);
      if (!(h2 >= -1e-12 && h2 <= 1 + 1e-12)) return out;
      const wh = nrm(w), b1 = nrm(Math.abs(wh[0]) < 0.9 ? cross(wh, [1, 0, 0]) : cross(wh, [0, 1, 0])), b2 = cross(wh, b1);
      const a1 = dot(A, b1), a2 = dot(A, b2), rho = Math.hypot(a1, a2), ps = Math.atan2(a2, a1);
      for (const h of [Math.sqrt(Math.max(0, h2)), -Math.sqrt(Math.max(0, h2))]) {
        if (Math.abs(h) > rho + 1e-12) continue;
        const ac = Math.acos(Math.max(-1, Math.min(1, h / rho)));
        for (const phi of [ps + ac, ps - ac]) {
          const m = [0, 1, 2].map(k => Math.cos(phi) * b1[k] + Math.sin(phi) * b2[k]);
          if (orientFor(ub, m, out, x) === 'degenerate') { out.degenerate = true; return out; }
          orientFor(ub, m.map(y => -y), out, x);
        }
      }
      return out;
    };
  } else {
    const L = Lraw.map(y => y / Ln), Lo = mv(M, L);
    const Ra = rotM(spinAxis, 0.7), MR = mul(M, Ra), RM = mul(Ra, M);
    const closed = Math.hypot(...cross(L, spinAxis)) < 1e-9 && MR.every((r, i) => r.every((y, j) => Math.abs(y - RM[i][j]) < 1e-9));
    if (closed) {
      // ---------- C: 族绕棱自旋, 全闭式 ----------
      kind = 'C';
      const eps = dot(Lo, L) > 0 ? 1 : -1;
      const Aw = Math.PI - Math.acos(Math.max(-1, Math.min(1, dot(N[0], nbv))));
      const z0 = fx;
      const axisOf = u => (om === OMode.ZetaFixed ? [Math.sin(z0) * Math.cos(u), Math.sin(z0) * Math.sin(u), Math.cos(z0)] : [-Math.sin(u), Math.cos(u), 0]);
      fiberRaw = x => {
        const vv = [x[0] - eps * A[0], x[1] - eps * A[1], x[2] - eps * A[2]];
        let roots;
        if (om === OMode.ZetaFixed) {
          const rr = Math.sin(z0) * Math.hypot(vv[0], vv[1]), psi = Math.atan2(vv[1], vv[0]), cc = -Math.cos(z0) * vv[2] / rr;
          roots = rr > 1e-14 && Math.abs(cc) <= 1 ? [psi + Math.acos(cc), psi - Math.acos(cc)] : [];
        } else { const u0 = Math.atan2(vv[1], vv[0]); roots = [u0, u0 + Math.PI]; }
        const out = [];
        for (const u of roots) {
          const a = axisOf(u), Z = dot(A, a), Ap = A.map((t, k) => t - Z * a[k]), xa = dot(x, a), xp = x.map((t, k) => t - xa * a[k]);
          const la = Math.hypot(...Ap), lx = Math.hypot(...xp);
          if (la < 1e-12 || lx < 1e-12) continue;
          const d0 = Math.acos(Math.max(-1, Math.min(1, dot(Ap, xp) / (la * lx))));
          const np = Math.sqrt(n * n - Z * Z) / Math.sqrt(1 - Z * Z);
          const m = 0.5 * (d0 + Aw), s2 = Math.sin(Aw / 2) ** 2, c2 = Math.cos(Aw / 2) ** 2;
          const X = (np * np - Math.cos(m) ** 2 / c2) / (Math.sin(m) ** 2 / s2 - Math.cos(m) ** 2 / c2);
          if (!(X >= -1e-12 && X <= 1 + 1e-12)) continue;
          const t = Math.acos(Math.sqrt(Math.min(1, Math.max(0, X))));
          const gA = g0At(u, 0), gB = g0At(u, Math.PI), c0 = 0.5 * (gA + gB), aa = 0.5 * (gA - gB), bb = g0At(u, Math.PI / 2) - c0;
          const C = Math.hypot(aa, bb), phi = Math.atan2(bb, aa);
          for (const i of [m + t, m - t]) {
            const cv = (la * Math.cos(i) - c0) / C;
            if (Math.abs(cv) > 1 + 1e-12) continue;
            for (const sg of [1, -1]) accept(out, x, u, phi + sg * Math.acos(Math.max(-1, Math.min(1, cv))));
          }
        }
        return out;
      };
    } else {
      // ---------- B: 一般楔 — 沿棱守恒的闭式曲线 + 一维求根 ----------
      kind = 'B';
      const sinus = t => { const p0 = Rt(0, 0, t), pp = Rt(0, Math.PI, t), ph = Rt(0, Math.PI / 2, t);
        const c0 = p0.map((y, k) => 0.5 * (y + pp[k])); return [c0, p0.map((y, k) => 0.5 * (y - pp[k])), ph.map((y, k) => y - c0[k])]; };
      const [a0, a1, a2] = sinus(L), [b0, b1, b2] = sinus(Lo);
      const ell = (c, v) => [0, 1, 2].map(k => c[0][k] + c[1][k] * Math.cos(v) + c[2][k] * Math.sin(v));
      const NV = 120;
      fiberRaw = x => {
        const out = [];
        const zr = (w, al) => [w[0] * Math.cos(al) + w[1] * Math.sin(al), -w[0] * Math.sin(al) + w[1] * Math.cos(al), w[2]];
        const branch = (Pc, Qs, Rr, s) => { const H = Math.hypot(Pc, Qs); return H < 1e-14 || Math.abs(Rr) > H ? NaN : Math.atan2(Qs, Pc) + s * Math.acos(Math.max(-1, Math.min(1, -Rr / H))); };
        const pointOf = (par, t, s) => {
          if (par === 0) {
            const l1 = ell([a0, a1, a2], t), l2 = ell([b0, b1, b2], t);
            return [branch(l1[0] * A[0] + l1[1] * A[1] - l2[0] * x[0] - l2[1] * x[1], l1[0] * A[1] - l1[1] * A[0] - l2[0] * x[1] + l2[1] * x[0], l1[2] * A[2] - l2[2] * x[2], s), t];
          }
          const Aa = zr(A, t), xa = zr(x, t);
          return [t, branch(dot(a1, Aa) - dot(b1, xa), dot(a2, Aa) - dot(b2, xa), dot(a0, Aa) - dot(b0, xa), s)];
        };
        const g = (par, t, s) => {
          const [al, v] = pointOf(par, t, s);
          if (!Number.isFinite(al) || !Number.isFinite(v)) return NaN;
          const y = skyUnit(al, v), lw = Rt(al, v, Lo);
          if (!y.every(Number.isFinite)) return NaN;
          const yd = dot(y, lw), xd = dot(x, lw), yp = y.map((q, k) => q - yd * lw[k]), xp = x.map((q, k) => q - xd * lw[k]);
          return Math.atan2(dot(cross(yp, xp), lw), dot(yp, xp));
        };
        const root = (par, s, lo, hi, glo) => {
          for (let it = 0; it < 50; it++) {
            const mid = 0.5 * (lo + hi), gm = g(par, mid, s);
            if (!Number.isFinite(gm)) break;
            if (glo * gm <= 0) hi = mid; else { lo = mid; glo = gm; }
          }
          const [al, v] = pointOf(par, 0.5 * (lo + hi), s);
          accept(out, x, al, v);
        };
        const scan = (par, s, t0, t1, g0, g1, depth) => {
          if (!Number.isFinite(g0) || !Number.isFinite(g1)) return;
          if (g0 * g1 <= 0 && Math.abs(g0) < 1.5 && Math.abs(g1) < 1.5) { root(par, s, t0, t1, g0); return; }
          if (depth > 0 && g0 * g1 > 0 && Math.min(Math.abs(g0), Math.abs(g1)) < 0.05) {
            const tm = 0.5 * (t0 + t1), gm = g(par, tm, s);
            scan(par, s, t0, tm, g0, gm, depth - 1); scan(par, s, tm, t1, gm, g1, depth - 1);
          }
        };
        for (const par of [0, 1])
          for (const s of [1, -1]) {
            let tp = 0, gp = g(par, 0, s);
            for (let k = 1; k <= NV; k++) { const t = TAU * k / NV, gt = g(par, t, s); scan(par, s, tp, t, gp, gt, 5); tp = t; gp = gt; }
          }
        return out;
      };
    }
  }

  // ---------- 牛顿: 姿态 q 跟着鱼眼目标点走 ----------
  const f1 = [0, 0], f2 = [0, 0], f3 = [0, 0];
  const polish = (u, v, gx, gy) => {
    for (let it = 0; it < 12; it++) {
      fish(u, v, f1);
      const ex = f1[0] - gx, ey = f1[1] - gy;
      if (Math.hypot(ex, ey) < 1e-10) return ok(u, v) ? [wrapPi(u), wrapPi(v)] : null;
      const h = 1e-7;
      fish(u + h, v, f2); fish(u, v + h, f3);
      const a11 = (f2[0] - f1[0]) / h, a21 = (f2[1] - f1[1]) / h, a12 = (f3[0] - f1[0]) / h, a22 = (f3[1] - f1[1]) / h, dd = a11 * a22 - a12 * a21;
      if (!Number.isFinite(dd) || Math.abs(dd) < 1e-14) return null;
      let du = (a22 * ex - a12 * ey) / dd, dv = (-a21 * ex + a11 * ey) / dd;
      const st = Math.hypot(du, dv);
      if (st > 0.02) { du *= 0.02 / st; dv *= 0.02 / st; }
      u -= du; v -= dv;
    }
    return null;
  };
  const sh = A[2], ch = Math.hypot(A[0], A[1]);
  const dirOf = (gx, gy) => { const r = Math.hypot(gx, gy) * D2R, ps = Math.atan2(gy, gx), s = Math.sin(r), c = Math.cos(r); return [c * A[0] - s * Math.sin(ps) * sh, -s * Math.cos(ps), c * A[2] + s * Math.sin(ps) * ch]; };
  const uniq = (arr, p) => { if (p && !arr.some(o => Math.abs(wrapPi(o[0] - p[0])) < 1e-5 && Math.abs(wrapPi(o[1] - p[1])) < 1e-5)) arr.push(p); };
  /** 鱼眼点 (°) 的完整纤维 (精修到 1e-10°) */
  const full = (gx, gy) => { const raw = fiberRaw(dirOf(gx, gy)), out = []; for (const q of raw) uniq(out, polish(q[0], q[1], gx, gy) || q); if (raw.degenerate) out.degenerate = true; return out; };
  const track = (fib, gx, gy) => { const out = []; for (const q of fib) uniq(out, polish(q[0], q[1], gx, gy)); return out; };
  const Iof = fib => { let s = 0; for (const q of fib) s += wJ(q[0], q[1]); return s / (TAU * TAU); };
  return { kind, ok, fish, dirOf, full, track, Iof, wJ };
}

const sameFib = (a, b) => a.length === b.length && a.every(p => b.some(q => Math.abs(wrapPi(p[0] - q[0])) < 1e-5 && Math.abs(wrapPi(p[1] - q[1])) < 1e-5));

/**
 * 等光强线. lines: 引擎族线 Map (name → { par, sky, conn }), fams: 族定义; levels: 光强值数组 (空 → 自动取 4 级).
 * 返回 { status, kind, levels, curves: [{ c, D: Float32Array (x,y,z)×n }], ms }
 */
export function computeIso(P, om, fx, fams, lines, opts = {}) {
  const t0 = performance.now();
  if (AP.on && P.apSupported) return { status: 'aperture', curves: [], levels: [] };
  const M = makeIso(P, om, fx, opts.weight === 'geo' ? 'geo' : 'fresnel');
  const { ok, fish, full, track, Iof } = M;
  const fo = [0, 0];
  // ---------- 自动等级: 随机姿态处的光强取分位 (10/35/65/90%), 两位有效数字 ----------
  let levels = (opts.levels || []).filter(v => v > 0);
  if (!levels.length) {
    let seed = 0x1234567 >>> 0;
    const rnd = () => { seed ^= seed << 13; seed >>>= 0; seed ^= seed >>> 17; seed ^= seed << 5; seed >>>= 0; return seed / 4294967296; };
    const ax = axesMeta(om), us = (ax.x1 - ax.x0) * D2R, vs = (ax.y1 - ax.y0) * D2R, smp = [];
    for (let tries = 0; smp.length < 240 && tries < 60000; tries++) {
      const u = ax.x0 * D2R + rnd() * us, v = ax.y0 * D2R + rnd() * vs;
      if (!ok(u, v)) continue;
      fish(u, v, fo);
      const fb = full(fo[0], fo[1]);
      if (fb.degenerate) return { status: 'degenerate', curves: [], levels: [] };
      if (!fb.some(q => Math.abs(wrapPi(q[0] - u)) < 1e-5 && Math.abs(wrapPi(q[1] - v)) < 1e-5)) fb.push([u, v]);
      const I = Iof(fb);
      if (I > 0 && Number.isFinite(I)) smp.push(I);
    }
    if (smp.length < 20) return { status: 'none', curves: [], levels: [] };
    smp.sort((a, b) => a - b);
    levels = [...new Set([0.1, 0.35, 0.65, 0.9].map(f => +smp[Math.floor(f * (smp.length - 1))].toPrecision(2)))];
  }
  levels.sort((a, b) => a - b);

  // ---------- 起点: 焦散 (没有就用篱笆) 天空像的法向行走 ----------
  const runsOf = kinds => {
    const runs = [];
    for (const fm of fams) {
      if (!kinds.includes(fm.kind)) continue;
      const fl = lines.get(fm.name);
      if (!fl) continue;
      for (const sk of [...fl.sky, ...(fl.conn ? fl.conn.sky : [])]) {
        let run = [];
        for (let i = 0; i < sk.length / 4; i++) {
          if (!(sk[4 * i + 3] > 0.5)) { if (run.length > 1) runs.push(run); run = []; continue; }
          fisheyeOf(sk[4 * i], sk[4 * i + 1], sk[4 * i + 2], fo); run.push([fo[0], fo[1]]);
        }
        if (run.length > 1) runs.push(run);
      }
    }
    return runs;
  };
  let runs = runsOf([6]);
  if (!runs.length) runs = runsOf([0, 1, 2, 3]);
  const seeds = levels.map(() => []);
  for (const run of runs) {
    let acc = 1e9;
    for (let i = 1; i < run.length - 1; i++) {
      acc += Math.hypot(run[i][0] - run[i - 1][0], run[i][1] - run[i - 1][1]);
      if (acc < 3) continue;
      acc = 0;
      const tx = run[i + 1][0] - run[i - 1][0], ty = run[i + 1][1] - run[i - 1][1], tl = Math.hypot(tx, ty);
      if (!(tl > 1e-9)) continue;
      for (const sg of [1, -1]) {
        const nx = -sg * ty / tl, ny = sg * tx / tl;
        let gx = run[i][0] + 0.02 * nx, gy = run[i][1] + 0.02 * ny, fib = full(gx, gy), I = Iof(fib);
        if (!(I > 0)) continue;
        for (let j = 1; j <= 120; j++) {
          const hx = run[i][0] + (0.02 + 0.1 * j) * nx, hy = run[i][1] + (0.02 + 0.1 * j) * ny;
          const fb2 = j % 5 === 0 ? full(hx, hy) : track(fib, hx, hy), I2 = Iof(fb2);
          for (let k = 0; k < levels.length; k++)
            if ((I - levels[k]) * (I2 - levels[k]) < 0) {
              let ax_ = gx, ay = gy, bx = hx, by = hy, fa = fib;
              for (let it = 0; it < 40; it++) {
                const mx = 0.5 * (ax_ + bx), my = 0.5 * (ay + by), fm = track(fa, mx, my), Im = Iof(fm);
                if ((I - levels[k]) * (Im - levels[k]) > 0) { ax_ = mx; ay = my; fa = fm; } else { bx = mx; by = my; }
              }
              seeds[k].push([0.5 * (ax_ + bx), 0.5 * (ay + by)]);
            }
          gx = hx; gy = hy; fib = fb2; I = I2;
          if (!(I > 0)) break;
        }
      }
    }
  }

  // ---------- 沿 I(x) = c 走 ----------
  const segDist = (px, py, ax_, ay, bx, by) => { const vx = bx - ax_, vy = by - ay, l2 = vx * vx + vy * vy, t = l2 > 0 ? Math.max(0, Math.min(1, ((px - ax_) * vx + (py - ay) * vy) / l2)) : 0; return Math.hypot(px - ax_ - t * vx, py - ay - t * vy); };
  const curves = [];
  for (let k = 0; k < levels.length; k++) {
    const c = levels[k], done = [];
    const nearDone = (x, y) => done.some(pl => { for (let i = 1; i < pl.length; i++) if (segDist(x, y, pl[i - 1][0], pl[i - 1][1], pl[i][0], pl[i][1]) < 0.1) return true; return false; });
    const F = fb => { const I = Iof(fb); return I > 0 ? Math.log(I / c) : NaN; };
    const corr = (x, y, fb) => {
      for (let it = 0; it < 10; it++) {
        const f0 = F(fb);
        if (!Number.isFinite(f0)) return null;
        if (Math.abs(f0) < 1e-10) return [x, y, fb, it];
        const h = 1e-4, gx = (F(track(fb, x + h, y)) - F(track(fb, x - h, y))) / (2 * h), gy = (F(track(fb, x, y + h)) - F(track(fb, x, y - h))) / (2 * h), g2 = gx * gx + gy * gy;
        if (!(g2 > 1e-24)) return null;
        x -= f0 * gx / g2; y -= f0 * gy / g2;
        fb = track(fb, x, y);
        if (!fb.length) return null;
      }
      return Math.abs(F(fb)) < 1e-9 ? [x, y, fb, 10] : null;
    };
    for (const sd of seeds[k]) {
      let st0 = corr(sd[0], sd[1], full(sd[0], sd[1])), st0ok = false;
      for (let rep = 0; st0 && rep < 4; rep++) {
        const fbF = full(st0[0], st0[1]);
        if (sameFib(fbF, st0[2])) { st0ok = true; break; }
        st0 = corr(st0[0], st0[1], fbF);
      }
      if (!st0ok || nearDone(st0[0], st0[1])) continue;        // 起点落在焦散上 / 已经追过
      const halves = [];
      for (const sgn of [1, -1]) {
        let [x, y, fb] = st0, s = 0.15, closed = false, stall = 0;
        const pts = [];
        for (let step = 0; step < 4000; step++) {
          const h = 1e-4;
          const gx = (F(track(fb, x + h, y)) - F(track(fb, x - h, y))) / (2 * h), gy = (F(track(fb, x, y + h)) - F(track(fb, x, y - h))) / (2 * h), gl = Math.hypot(gx, gy);
          if (!(gl > 0)) break;
          const tx = -sgn * gy / gl, ty = sgn * gx / gl;
          let r = null;
          for (let tries = 0; tries < 10 && !r; tries++) {
            const fbp = track(fb, x + s * tx, y + s * ty);
            if (fbp.length) { r = corr(x + s * tx, y + s * ty, fbp); if (r && Math.hypot(r[0] - x, r[1] - y) > 3 * s) r = null; }
            if (!r) s *= 0.5;
            if (s < 1e-5) break;
          }
          if (!r) break;
          const fbFull = full(r[0], r[1]);                        // 姿态有没有出生/消失
          if (!sameFib(fbFull, r[2])) {
            if (!(Math.abs(F(fbFull)) < 1e-6)) {                    // 光强跳变 (焦散): 二分出位置, 曲线在那里终止
              let ax_ = x, ay = y, bx = r[0], by = r[1];
              for (let it = 0; it < 30; it++) { const mx = 0.5 * (ax_ + bx), my = 0.5 * (ay + by); if (sameFib(full(mx, my), track(fb, mx, my))) { ax_ = mx; ay = my; } else { bx = mx; by = my; } }
              pts.push([0.5 * (ax_ + bx), 0.5 * (ay + by), 1]);
              break;
            }
            r[2] = fbFull;
          }
          stall = Math.hypot(r[0] - x, r[1] - y) < 1e-4 ? stall + 1 : 0;
          if (stall > 5) break;                                   // 卡在篱笆夹角 (弧的尖端)
          [x, y, fb] = r;
          pts.push([x, y]);
          s = r[3] <= 2 ? Math.min(0.3, s * 1.4) : s * 0.7;
          if (step > 8 && Math.hypot(x - st0[0], y - st0[1]) < 1.2 * s) { closed = true; break; }
        }
        halves.push({ pts, closed });
        if (closed) break;
      }
      const pts = halves[0].closed ? [[st0[0], st0[1]], ...halves[0].pts, [st0[0], st0[1]]]
        : [...(halves[1] ? halves[1].pts.slice().reverse() : []), [st0[0], st0[1]], ...halves[0].pts];
      done.push(pts);
      const D = new Float32Array(3 * pts.length);
      pts.forEach((p, i) => { const d = M.dirOf(p[0], p[1]); D[3 * i] = d[0]; D[3 * i + 1] = d[1]; D[3 * i + 2] = d[2]; });
      const cv = { c, D };
      if (opts.debug) { cv.fish = pts.map(p => [p[0], p[1]]); cv.end = pts.map(p => p[2] || 0); }
      curves.push(cv);
    }
  }
  return { status: 'ok', kind: M.kind, levels, curves, ms: performance.now() - t0 };
}
