// 光点法·平行光二自由度【末端点】闭式引擎 — HaloEndpointEngine.cs (HaloEP.Compute 部分) 的 JS 移植.
//   一切曲线活在晶体系太阳球 w = R^T·A 上; 链标量都是 L-型层级线 q = ℓ·w + c·S(X), S=√(X²+K), X=w·u0;
//   焦散因式分解 J = N(w)·Λ;  Ĵ ≡ T_E·J = T_E(Na·Λ) − q_E(Nb·Λ) 在出射临界上仍有限;
//   可达带 = Gram 行列式 G ≥ 0.
// 末端点四类: end (曲线∩约束篱笆) / turn (曲线∩可达带圆) / cross (部分反射蓝边内部穿越) / cusp (焦散 pleat 尖).
// 发散光 h-延拓部分 (ComputeDivergent*) 窗体不用, 未移植.

import { LP, K, SqK, OMode, orientEuler, faceCoef, PathCtx, Chain3 } from './lp.js';

// ================================================== 向量小件
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const nrm = a => Math.sqrt(dot(a, a));
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];

// .NET Math.IEEERemainder
function ieeeRemainder(x, y) {
  if (Number.isNaN(x)) return x;
  if (Number.isNaN(y)) return y;
  const regularMod = x % y;
  if (Number.isNaN(regularMod)) return NaN;
  if (regularMod === 0 && (x < 0 || Object.is(x, -0))) return -0;
  const alternativeResult = regularMod - (Math.abs(y) * Math.sign(x));
  if (Math.abs(alternativeResult) === Math.abs(regularMod)) {
    const divisionResult = x / y;
    const roundedResult = roundEven(divisionResult);
    if (Math.abs(roundedResult) > Math.abs(divisionResult)) return alternativeResult;
    return regularMod;
  }
  if (Math.abs(alternativeResult) < Math.abs(regularMod)) return alternativeResult;
  return regularMod;
}
function roundEven(x) {
  if (Math.abs(x % 1) === 0.5) return 2 * Math.round(x / 2);
  return Math.round(x);
}
const wrapPi = x => ieeeRemainder(x, 2 * Math.PI);

function inv3(A) {
  const a = A[0][0], b = A[0][1], c = A[0][2];
  const d = A[1][0], e = A[1][1], f = A[1][2];
  const g = A[2][0], h = A[2][1], i = A[2][2];
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-13) return null;
  const id = 1.0 / det;
  return [
    [(e * i - f * h) * id, (c * h - b * i) * id, (b * f - c * e) * id],
    [(f * g - d * i) * id, (a * i - c * g) * id, (c * d - a * f) * id],
    [(d * h - e * g) * id, (b * g - a * h) * id, (a * e - b * d) * id]];
}

function solve3(A, r) {
  const Ai = inv3(A);
  if (Ai === null) return null;
  const x = [0, 0, 0];
  for (let i = 0; i < 3; i++) x[i] = Ai[i][0] * r[0] + Ai[i][1] * r[1] + Ai[i][2] * r[2];
  return x;
}

// adj(Φ): 行 = (c2×c3, c3×c1, c1×c2), ci = Φ 的列
function adj3(P) {
  const c1 = [P[0][0], P[1][0], P[2][0]];
  const c2 = [P[0][1], P[1][1], P[2][1]];
  const c3 = [P[0][2], P[1][2], P[2][2]];
  return [cross(c2, c3), cross(c3, c1), cross(c1, c2)];
}

function mul(M, v) {
  return [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]];
}

const faceN = f => faceCoef(f);

// ================================================== 光路常量 (P = L·w + S·v 链)
class PathC {
  constructor(faces, partial) {
    this.faces = faces;
    this.U0 = faceN(faces[0]); this.UE = faceN(faces[faces.length - 1]);
    let Lm = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        Lm[i][j] = (i === j ? 1.0 : 0.0) - this.U0[i] * this.U0[j];
    let Vv = this.U0.slice();
    this.MidL = []; this.MidC = [];
    this.Fences = [{ name: 'entry', L: this.U0.slice(), C: 0, Kk: 0, side: +1 }];
    for (let m = 1; m < faces.length - 1; m++) {
      const u = faceN(faces[m]);
      const l = [
        Lm[0][0] * u[0] + Lm[1][0] * u[1] + Lm[2][0] * u[2],
        Lm[0][1] * u[0] + Lm[1][1] * u[1] + Lm[2][1] * u[2],
        Lm[0][2] * u[0] + Lm[1][2] * u[1] + Lm[2][2] * u[2]];      // Lᵀu
      const cc = dot(Vv, u);
      this.MidL.push(l); this.MidC.push(cc);
      this.Fences.push({ name: `q${m}(面${faces[m]})=0`, L: l.slice(), C: cc, Kk: 0, side: -1 });
      this.Fences.push({ name: `q${m}(面${faces[m]})=-√K`, L: l.slice(), C: cc, Kk: -SqK, side: partial ? 0.0 : +1.0 });
      // 反射: L ← R_f L, v ← R_f v  (R_f = I − 2uuᵀ)
      const Ln = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for (let i = 0; i < 3; i++)
        for (let j = 0; j < 3; j++)
          Ln[i][j] = Lm[i][j] - 2 * u[i] * (u[0] * Lm[0][j] + u[1] * Lm[1][j] + u[2] * Lm[2][j]);
      const qv = dot(u, Vv);
      const vn = [Vv[0] - 2 * qv * u[0], Vv[1] - 2 * qv * u[1], Vv[2] - 2 * qv * u[2]];
      Lm = Ln; Vv = vn;
    }
    const UE = this.UE;
    this.LE = [
      Lm[0][0] * UE[0] + Lm[1][0] * UE[1] + Lm[2][0] * UE[2],
      Lm[0][1] * UE[0] + Lm[1][1] * UE[1] + Lm[2][1] * UE[2],
      Lm[0][2] * UE[0] + Lm[1][2] * UE[1] + Lm[2][2] * UE[2]];
    this.CE = dot(Vv, UE);
    this.Lm = Lm; this.Vv = Vv;
    this.Fences.push({ name: 'exit', L: this.LE.slice(), C: this.CE, Kk: -SqK, side: -1 });
  }

  chainW(w) {
    const U0 = this.U0, Lm = this.Lm, Vv = this.Vv, UE = this.UE;
    const X = dot(U0, w), S = Math.sqrt(X * X + K);
    const qs = new Array(this.MidL.length);
    for (let i = 0; i < this.MidL.length; i++) qs[i] = dot(this.MidL[i], w) + this.MidC[i] * S;
    const qE = dot(this.LE, w) + this.CE * S;
    const t2 = qE * qE - K;
    const TE = t2 >= 0 ? Math.sqrt(t2) : (t2 > -1e-11 ? 0.0 : NaN);
    const P = [
      Lm[0][0] * w[0] + Lm[0][1] * w[1] + Lm[0][2] * w[2] + S * Vv[0],
      Lm[1][0] * w[0] + Lm[1][1] * w[1] + Lm[1][2] * w[2] + S * Vv[1],
      Lm[2][0] * w[0] + Lm[2][1] * w[1] + Lm[2][2] * w[2] + S * Vv[2]];
    const tEv = -qE - TE;
    const Dc = [P[0] + tEv * UE[0], P[1] + tEv * UE[1], P[2] + tEv * UE[2]];
    return { X, S, qs, qE, TE, Dc };
  }

  // 焦散: N_a, N_b  (adj(Φ) 对 λ=−qE/TE 严格仿射; Ĵ = TE·(Na·Λ) − qE·(Nb·Λ))
  naNb(w) {
    const { X, S, qE, TE, Dc } = this.chainW(w);
    const U0 = this.U0, UE = this.UE, LE = this.LE, CE = this.CE, Lm = this.Lm, Vv = this.Vv;
    const xs = X / S;
    const g = [LE[0] + xs * CE * U0[0], LE[1] + xs * CE * U0[1], LE[2] + xs * CE * U0[2]];
    const M1 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        M1[i][j] = Lm[i][j] + Vv[i] * xs * U0[j] - UE[i] * g[j];
    const Wx = [[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]];
    const Dx = [[0, -Dc[2], Dc[1]], [Dc[2], 0, -Dc[0]], [-Dc[1], Dc[0], 0]];
    const Phi1 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let t = 0; t < 3; t++) s += M1[i][t] * Wx[t][j];
        Phi1[i][j] = s - Dx[i][j];
      }
    const gxw = cross(g, w);
    const Phi2 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        Phi2[i][j] = Phi1[i][j] + UE[i] * gxw[j];
    const aj1 = adj3(Phi1), aj2 = adj3(Phi2);
    const Na = mul(aj1, Dc);
    const dj = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) dj[i][j] = aj2[i][j] - aj1[i][j];
    const Nb = mul(dj, Dc);
    return { Na, Nb, qE, TE, Dc };
  }

  fval(f, w) {
    const X = dot(this.U0, w);
    return dot(f.L, w) + f.C * Math.sqrt(X * X + K) - f.Kk;
  }

  fgrad(f, w) {
    const U0 = this.U0;
    const X = dot(U0, w);
    const S = Math.sqrt(X * X + K);
    const xs = f.C * X / S;
    return [f.L[0] + xs * U0[0], f.L[1] + xs * U0[1], f.L[2] + xs * U0[2]];
  }
}

// ================================================== 取向族模板
class FamC {
  static make(m, fixedAng) {
    const f = new FamC();
    f.M = m; f.Fixed = fixedAng;
    if (m === OMode.ZetaFixed) { f.Ec = [0.0, 0.0, 1.0]; f.Gw = [0.0, 0.0, 1.0]; f.C0 = Math.cos(fixedAng); }
    else if (m === OMode.ZenFree) { f.Ec = [Math.cos(fixedAng), -Math.sin(fixedAng), 0.0]; f.Gw = [0.0, 0.0, 1.0]; f.C0 = 0; }
    else if (m === OMode.AlphaFixed) { f.Ec = [0.0, 0.0, 1.0]; f.Gw = [-Math.sin(fixedAng), Math.cos(fixedAng), 0.0]; f.C0 = 0; }
    else return null;
    f.Gamma = dot(f.Gw, LP.A3);
    f.Rad = Math.sqrt(Math.max((1 - f.C0 * f.C0) * (1 - f.Gamma * f.Gamma), 0.0));
    f.KapLo = f.C0 * f.Gamma - f.Rad; f.KapHi = f.C0 * f.Gamma + f.Rad;
    return f;
  }

  gram(w) {
    const k = dot(this.Ec, w);
    return 1 - k * k - this.C0 * this.C0 - this.Gamma * this.Gamma + 2 * k * this.C0 * this.Gamma;
  }

  rho(w, eps) {
    const Ec = this.Ec;
    const k = dot(Ec, w);
    let den = 1 - k * k; if (Math.abs(den) < 1e-14) den = 1e-14;
    const A1 = (this.C0 - k * this.Gamma) / den, A2 = (this.Gamma - k * this.C0) / den;
    const g = this.gram(w);
    const A3v = eps * (g >= 0 ? Math.sqrt(g) : (g > -1e-11 ? 0.0 : NaN)) / den;
    const ew = cross(Ec, w);
    return [A1 * Ec[0] + A2 * w[0] + A3v * ew[0], A1 * Ec[1] + A2 * w[1] + A3v * ew[1], A1 * Ec[2] + A2 * w[2] + A3v * ew[2]];
  }

  reachFences() {
    return [
      { name: '可达带上界', L: this.Ec.slice(), C: 0, Kk: this.KapHi, side: -1 },
      { name: '可达带下界', L: this.Ec.slice(), C: 0, Kk: this.KapLo, side: +1 }];
  }

  // w + 分支 ε → 晶体系→世界系旋转 (双正交架): R^T A = w, R^T g = ρ.  返回 [e1,e2,e3] 或 null
  lift(w, eps) {
    const rho = this.rho(w, eps);
    if (Number.isNaN(rho[0])) return null;
    const A = LP.A3;
    const f1 = scl(A, 1.0 / nrm(A));
    const t = sub(this.Gw, scl(f1, dot(this.Gw, f1)));
    const nt = nrm(t); if (nt < 1e-12) return null;
    const f2 = scl(t, 1.0 / nt);
    const f3 = cross(f1, f2);
    const c1v = scl(w, 1.0 / nrm(w));
    const t2 = sub(rho, scl(c1v, dot(rho, c1v)));
    const nt2 = nrm(t2); if (nt2 < 1e-12) return null;
    const c2v = scl(t2, 1.0 / nt2);
    const c3v = cross(c1v, c2v);
    const e1 = [0, 0, 0], e2 = [0, 0, 0], e3 = [0, 0, 0];
    for (let r = 0; r < 3; r++) {
      e1[r] = f1[r] * c1v[0] + f2[r] * c2v[0] + f3[r] * c3v[0];
      e2[r] = f1[r] * c1v[1] + f2[r] * c2v[1] + f3[r] * c3v[1];
      e3[r] = f1[r] * c1v[2] + f2[r] * c2v[2] + f3[r] * c3v[2];
    }
    return [e1, e2, e3];
  }

  // 提取图卡坐标 (sc, sw) — 与 axesMeta/orientOf 约定一致
  uvOf(e1, e2, e3) {
    let al = Math.atan2(e3[1], e3[0]);
    const szv = Math.sqrt(e3[0] * e3[0] + e3[1] * e3[1]);
    let ze = Math.atan2(szv, e3[2]);                     // [0, π]
    let roll = Math.atan2(e1[2], e2[2]);
    let sc, sw;
    if (this.M === OMode.ZetaFixed) {
      // 图卡 ζ0 可为负: 换等价表示 (α+π, −ζ, roll+π)
      if (Math.abs(wrapPi(-ze - this.Fixed)) < Math.abs(wrapPi(ze - this.Fixed)) - 1e-12) { al += Math.PI; roll += Math.PI; }
      sc = al; sw = roll;
    } else if (this.M === OMode.ZenFree) {
      if (Math.abs(wrapPi(roll - this.Fixed)) > Math.PI / 2) { al += Math.PI; ze = 2 * Math.PI - ze; }
      sc = al; sw = ze;
    } else {
      if (Math.abs(wrapPi(al - this.Fixed)) > Math.PI / 2) { roll += Math.PI; ze = 2 * Math.PI - ze; }
      sc = roll; sw = ze;
    }
    sc = (sc % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    sw = (sw % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
    return [sc, sw];
  }
}

// ================================================== 场景
class Scene {
  constructor(p, f, pctx) {
    this.P = p; this.F = f; this.PC = pctx;
    this.Reach = f.reachFences();
    this.Geo = []; this.Vacuous = []; this.DegenCaustic = [];
    // 恒成立判据剔除: Fibonacci 球面取样, 可达带内不违反 → 非边界
    const n = 4000;
    const W = new Array(n);
    for (let i = 0; i < n; i++) {
      const ph = Math.acos(1 - 2 * (i + 0.5) / n);
      const th = Math.PI * (1 + Math.sqrt(5.0)) * (i + 0.5);
      W[i] = [Math.sin(ph) * Math.cos(th), Math.sin(ph) * Math.sin(th), Math.cos(ph)];
    }
    for (const fen of p.Fences) {
      if (fen.side === 0) { this.Geo.push(fen); continue; }
      let worst = Number.MAX_VALUE, any = false;
      for (const w of W) {
        const kap = dot(f.Ec, w) - f.C0 * f.Gamma;
        if (kap * kap > f.Rad * f.Rad + 1e-12) continue;
        any = true;
        const v = fen.side * p.fval(fen, w);
        if (v < worst) worst = v;
      }
      if (any && worst > -1e-9) this.Vacuous.push(fen.name);
      else this.Geo.push(fen);
    }
  }

  minMargin(w) {
    let m = Number.MAX_VALUE;
    for (const fen of this.Geo)
      if (fen.side !== 0) {
        const v = fen.side * this.P.fval(fen, w);
        if (v < m) m = v;
      }
    return m;
  }

  jhatW(w, eps) {
    const { Na, Nb, qE, TE } = this.P.naNb(w);
    const rho = this.F.rho(w, eps);
    if (Number.isNaN(rho[0])) return NaN;
    const Lam = cross(this.F.Ec, rho);
    return TE * dot(Na, Lam) - qE * dot(Nb, Lam);
  }
}

// ================================================== 多项式实根 (Durand-Kerner, deg ≤ 4; 复数运算同 System.Numerics.Complex)
function cmul(ar, ai, br, bi) { return [(ar * br) - (ai * bi), (ai * br) + (ar * bi)]; }
function cdiv(a, b, c, d) {
  if (Math.abs(d) < Math.abs(c)) {
    const doc = d / c;
    return [(a + b * doc) / (c + d * doc), (b - a * doc) / (c + d * doc)];
  }
  const cod = c / d;
  return [(b + a * cod) / (d + c * cod), (-a + b * cod) / (d + c * cod)];
}
function cabs(re, im) {
  let a = Math.abs(re), b = Math.abs(im);
  let small, large;
  if (a < b) { small = a; large = b; } else { small = b; large = a; }
  if (small === 0.0) return large;
  if (large === Infinity && !Number.isNaN(small)) return Infinity;
  const ratio = small / large;
  return large * Math.sqrt(1.0 + ratio * ratio);
}

function realRoots(c) {
  const res = [];
  let lo = 0;
  while (lo < c.length && Math.abs(c[lo]) < 1e-14) lo++;
  const deg = c.length - 1 - lo;
  if (deg < 1) return res;
  const a = new Array(deg + 1);
  for (let i = 0; i <= deg; i++) a[i] = c[lo + i] / c[lo];
  if (deg === 1) { res.push(-a[1]); return res; }
  const zr = new Array(deg), zi = new Array(deg);
  zr[0] = 0.4; zi[0] = 0.9;
  for (let i = 1; i < deg; i++) { const z = cmul(zr[i - 1], zi[i - 1], 0.4, 0.9); zr[i] = z[0]; zi[i] = z[1]; }
  for (let it = 0; it < 300; it++) {
    let moved = 0;
    for (let i = 0; i < deg; i++) {
      let pr = a[0], pi = 0;
      for (let j = 1; j <= deg; j++) { const t = cmul(pr, pi, zr[i], zi[i]); pr = t[0] + a[j]; pi = t[1]; }
      let dr = a[0], di = 0;
      for (let j = 0; j < deg; j++)
        if (j !== i) { const t = cmul(dr, di, zr[i] - zr[j], zi[i] - zi[j]); dr = t[0]; di = t[1]; }
      if (cabs(dr, di) < 1e-30) continue;
      const dz = cdiv(pr, pi, dr, di);
      zr[i] -= dz[0]; zi[i] -= dz[1];
      moved = Math.max(moved, cabs(dz[0], dz[1]));
    }
    if (moved < 1e-15) break;
  }
  for (let i = 0; i < deg; i++)
    if (Math.abs(zi[i]) < 1e-7) {
      const x = zr[i];
      if (!res.some(r => Math.abs(r - x) < 1e-10)) res.push(x);
    }
  return res;
}

// ================================================== L∩L 闭式交点 (一元四次)
function solveLL(sc, f1, f2) {
  const outw = [];
  const p = sc.P;
  const A = [f1.L.slice(), f2.L.slice(), p.U0.slice()];
  const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(det) > 1e-10) {
    const Ai = inv3(A);
    if (Ai === null) return outw;
    const pv = mul(Ai, [f1.Kk, f2.Kk, 0.0]);
    const qv = mul(Ai, [0.0, 0.0, 1.0]);
    const rv = mul(Ai, [-f1.C, -f2.C, 0.0]);
    // |w|²−1 = P(X) + S·Q(X);  P²−(X²+K)Q² = 0
    const Pc = [dot(qv, qv) + dot(rv, rv), 2 * dot(pv, qv), dot(pv, pv) - 1 + K * dot(rv, rv)];
    const Qc = [2 * dot(qv, rv), 2 * dot(pv, rv)];
    const P2 = [0, 0, 0, 0, 0];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) P2[i + j] += Pc[i] * Pc[j];
    const Q2 = [0, 0, 0];
    for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) Q2[i + j] += Qc[i] * Qc[j];
    const XQ = [0, 0, 0, 0, 0];
    for (let i = 0; i < 3; i++) { XQ[i] += Q2[i]; XQ[i + 2] += K * Q2[i]; }
    const poly = [0, 0, 0, 0, 0];
    for (let i = 0; i < 5; i++) poly[i] = P2[i] - XQ[i];
    for (const X of realRoots(poly)) {
      const S = Math.sqrt(X * X + K);
      const resv = (Pc[0] * X * X + Pc[1] * X + Pc[2]) + S * (Qc[0] * X + Qc[1]);
      if (Math.abs(resv) > 1e-6) continue;                 // 增根 (S 负号支)
      outw.push([pv[0] + X * qv[0] + S * rv[0], pv[1] + X * qv[1] + S * rv[1], pv[2] + X * qv[2] + S * rv[2]]);
    }
  } else {
    // 秩亏: 左零向量 λ 给相容条件 a + bX + c'S = 0; 右零向量 d 给球面上的直线
    const rows = [f1.L, f2.L, p.U0];
    const cols = [0, 1, 2].map(j => [f1.L[j], f2.L[j], p.U0[j]]);
    let lam = null, best = 0;
    const prs = [[0, 1], [0, 2], [1, 2]];
    for (const pr of prs) {
      const cxx = cross(cols[pr[0]], cols[pr[1]]);
      const nn = nrm(cxx);
      if (nn > best) { best = nn; lam = scl(cxx, 1.0 / nn); }
    }
    if (lam === null || best < 1e-10) return outw;
    const av = lam[0] * f1.Kk + lam[1] * f2.Kk;
    const bv = lam[2];
    const cv = -(lam[0] * f1.C + lam[1] * f2.C);
    const Xs = [];
    if (Math.abs(bv) < 1e-13 && Math.abs(cv) < 1e-13) return outw;   // 完全退化
    if (Math.abs(cv) < 1e-13) Xs.push(-av / bv);
    else {
      const pol = [bv * bv - cv * cv, 2 * av * bv, av * av - cv * cv * K];
      for (const X of realRoots(pol))
        if (Math.abs(av + bv * X + cv * Math.sqrt(X * X + K)) < 1e-7) Xs.push(X);
    }
    let dvec = null; best = 0;
    for (const pr of prs) {
      const cxx = cross(rows[pr[0]], rows[pr[1]]);
      const nn = nrm(cxx);
      if (nn > best) { best = nn; dvec = scl(cxx, 1.0 / nn); }
    }
    if (dvec === null || best < 1e-10) return outw;
    let r1i = 0, r2i = 1; best = 0;
    for (const pr of prs) {
      const nn = nrm(cross(rows[pr[0]], rows[pr[1]]));
      if (nn > best) { best = nn; r1i = pr[0]; r2i = pr[1]; }
    }
    for (const X of Xs) {
      const S = Math.sqrt(X * X + K);
      const bfull = [f1.Kk - f1.C * S, f2.Kk - f2.C * S, X];
      const M3 = [rows[r1i].slice(), rows[r2i].slice(), dvec.slice()];
      const rr = [bfull[r1i], bfull[r2i], 0];
      const wp = solve3(M3, rr);
      if (wp === null) continue;
      let consist = true;
      for (let rI = 0; rI < 3; rI++) {
        const lhs = rows[rI][0] * wp[0] + rows[rI][1] * wp[1] + rows[rI][2] * wp[2];
        if (Math.abs(lhs - bfull[rI]) > 1e-8) { consist = false; break; }
      }
      if (!consist) continue;
      const proj = dot(wp, dvec);
      const wpp = [wp[0] - proj * dvec[0], wp[1] - proj * dvec[1], wp[2] - proj * dvec[2]];
      const disc = 1 - dot(wpp, wpp);
      if (disc < -1e-12) continue;
      const tt = Math.sqrt(Math.max(disc, 0.0));
      outw.push(add(wpp, scl(dvec, tt)));
      if (tt > 1e-13) outw.push(add(wpp, scl(dvec, -tt)));
    }
  }
  // Newton 抛光 (解析梯度) + 去重
  const res = [];
  for (const w0 of outw) {
    const w = polishPair(ww => sc.P.fval(f1, ww), ww => sc.P.fgrad(f1, ww),
                         ww => sc.P.fval(f2, ww), ww => sc.P.fgrad(f2, ww), w0);
    if (w === null) continue;
    if (res.some(z => Math.abs(z[0] - w[0]) < 1e-11 && Math.abs(z[1] - w[1]) < 1e-11 && Math.abs(z[2] - w[2]) < 1e-11)) continue;
    res.push(w);
  }
  return res;
}

// 3 方程 Newton: F1=F2=0, |w|²=1.  G 可为 null → 中心差分
function polishPair(F1, G1, F2, G2, w0) {
  let w = w0.slice();
  for (let it = 0; it < 60; it++) {
    const r1 = F1(w), r2 = F2(w), r3 = dot(w, w) - 1;
    if (Number.isNaN(r1) || Number.isNaN(r2)) return null;
    if (Math.abs(r1) < 1e-13 && Math.abs(r2) < 1e-13 && Math.abs(r3) < 1e-14) break;
    const g1 = G1 !== null ? G1(w) : fdGrad(F1, w);
    const g2 = G2 !== null ? G2(w) : fdGrad(F2, w);
    if (g1 === null || g2 === null) return null;
    const J = [g1.slice(), g2.slice(), [2 * w[0], 2 * w[1], 2 * w[2]]];
    let dw = solve3(J, [-r1, -r2, -r3]);
    if (dw === null) return null;
    const nn = nrm(dw);
    if (Number.isNaN(nn)) return null;
    if (nn > 0.3) dw = scl(dw, 0.3 / nn);
    w = add(w, dw);
  }
  if (Number.isNaN(w[0]) || Math.abs(F1(w)) > 1e-9 || Math.abs(F2(w)) > 1e-9) return null;
  return w;
}

function fdGrad(F, w) {
  const h = 1e-6;
  const g = [0, 0, 0];
  for (let j = 0; j < 3; j++) {
    const wp = w.slice(); wp[j] += h;
    const wm = w.slice(); wm[j] -= h;
    const fp = F(wp), fm = F(wm);
    if (Number.isNaN(fp) || Number.isNaN(fm)) return null;
    g[j] = (fp - fm) / (2 * h);
  }
  return g;
}

// ================================================== 篱笆闭式参数化 (取样)
function fenceSamples(sc, fen, n) {
  const comps = [];
  const u0 = sc.P.U0;
  const dt = dot(fen.L, u0);
  const nperp = sub(fen.L, scl(u0, dt));
  const npn = nrm(nperp);
  if (npn > 1e-12) {
    const nh = scl(nperp, 1.0 / npn);
    const ph = cross(u0, nh);
    const bof = X => (fen.Kk - fen.C * Math.sqrt(X * X + K) - dt * X) / npn;
    const NG = 4001;
    const Xg = new Float64Array(NG), ok = new Uint8Array(NG);
    for (let i = 0; i < NG; i++) {
      Xg[i] = -1 + 2.0 * i / (NG - 1);
      const b = bof(Xg[i]);
      ok[i] = 1 - Xg[i] * Xg[i] - b * b >= 0 ? 1 : 0;
    }
    let idx = 0;
    while (idx < NG) {
      if (!ok[idx]) { idx++; continue; }
      let j = idx;
      while (j + 1 < NG && ok[j + 1]) j++;
      let lo = Xg[idx], hi = Xg[j];
      if (idx > 0) {
        let a = Xg[idx - 1], b2 = Xg[idx];
        for (let t = 0; t < 60; t++) {
          const m = 0.5 * (a + b2), bb = bof(m);
          if (1 - m * m - bb * bb >= 0) b2 = m; else a = m;
        }
        lo = b2;
      }
      if (j + 1 < NG) {
        let a = Xg[j], b2 = Xg[j + 1];
        for (let t = 0; t < 60; t++) {
          const m = 0.5 * (a + b2), bb = bof(m);
          if (1 - m * m - bb * bb >= 0) a = m; else b2 = m;
        }
        hi = a;
      }
      const comp = [];
      for (let sgn = 0; sgn < 2; sgn++)
        for (let t = 0; t < n; t++) {
          let s = t / (n - 1);
          if (sgn === 1) s = 1 - s;
          const X = lo + (hi - lo) * 0.5 * (1 - Math.cos(Math.PI * s));   // 端点加密
          const b = bof(X);
          const d2 = Math.max(1 - X * X - b * b, 0.0);
          const d = Math.sqrt(d2) * (sgn === 0 ? 1 : -1);
          comp.push([X * u0[0] + b * nh[0] + d * ph[0], X * u0[1] + b * nh[1] + d * ph[1], X * u0[2] + b * nh[2] + d * ph[2]]);
        }
      comps.push(comp);
      idx = j + 1;
    }
  } else {
    // ℓ ∥ u0: 整圈 w·u0 = X*
    const Xs = [];
    if (Math.abs(fen.C) < 1e-14) { if (Math.abs(dt) > 1e-14) Xs.push(fen.Kk / dt); }
    else {
      const pol = [dt * dt - fen.C * fen.C, -2 * dt * fen.Kk, fen.Kk * fen.Kk - fen.C * fen.C * K];
      for (const X of realRoots(pol))
        if (Math.abs(dt * X + fen.C * Math.sqrt(X * X + K) - fen.Kk) < 1e-9) Xs.push(X);
    }
    for (const X of Xs) {
      if (Math.abs(X) >= 1) continue;
      const r = Math.sqrt(1 - X * X);
      let a1 = cross(u0, [1.0, 0.0, 0.0]);
      if (nrm(a1) < 1e-8) a1 = cross(u0, [0.0, 1.0, 0.0]);
      a1 = scl(a1, 1.0 / nrm(a1));
      const a2 = cross(u0, a1);
      const comp = [];
      for (let t = 0; t < 2 * n; t++) {
        const th = 2 * Math.PI * t / (2 * n);
        comp.push([
          X * u0[0] + r * (Math.cos(th) * a1[0] + Math.sin(th) * a2[0]),
          X * u0[1] + r * (Math.cos(th) * a1[1] + Math.sin(th) * a2[1]),
          X * u0[2] + r * (Math.cos(th) * a1[2] + Math.sin(th) * a2[2])]);
      }
      comps.push(comp);
    }
  }
  return comps;
}

// 投回 {F=0, |w|=1} (在 span(∇F, w) 内 Newton)
function projectFence(sc, fen, w) {
  for (let it = 0; it < 40; it++) {
    const Fv = sc.P.fval(fen, w);
    const g = sc.P.fgrad(fen, w);
    const r2 = dot(w, w) - 1;
    if (Math.abs(Fv) < 1e-15 && Math.abs(r2) < 1e-15) return w;
    const m11 = dot(g, g), m12 = dot(g, w), m21 = 2 * m12, m22 = 2 * dot(w, w);
    const dd = m11 * m22 - m12 * m21;
    if (Math.abs(dd) < 1e-20) return null;
    const aa = (-Fv * m22 + r2 * m12) / dd;
    const bb = (-m11 * r2 + m21 * Fv) / dd;
    w = [w[0] + aa * g[0] + bb * w[0], w[1] + aa * g[1] + bb * w[1], w[2] + aa * g[2] + bb * w[2]];
    if (Number.isNaN(w[0])) return null;
  }
  return w;
}

// ================================================== 焦散 ∩ 篱笆
function solveCausticFence(sc, fen, eps, n) {
  const outw = [];
  const near = (z, w) => Math.abs(z[0] - w[0]) < 1e-9 && Math.abs(z[1] - w[1]) < 1e-9 && Math.abs(z[2] - w[2]) < 1e-9;
  for (const comp of fenceSamples(sc, fen, n)) {
    const m = comp.length;
    const Js = new Float64Array(m);
    for (let i = 0; i < m; i++) {
      Js[i] = NaN;
      if (sc.F.gram(comp[i]) < -1e-11) continue;
      Js[i] = sc.jhatW(comp[i], eps);
    }
    const fin = [];
    for (let i = 0; i < m; i++) if (Number.isFinite(Js[i])) fin.push(Math.abs(Js[i]));
    if (fin.length < 8) continue;
    fin.sort((a, b) => a - b);
    const scale = fin[Math.trunc(fin.length / 2)];
    if (scale < 1e-12) { if (!sc.DegenCaustic.includes(fen.name)) sc.DegenCaustic.push(fen.name); continue; }
    const noise = 1e-9 * scale;
    let nTan = 0; const added0 = outw.length;
    for (let i = 0; i < m; i++) {
      const j = (i + 1) % m;
      const a = Js[i], b = Js[j];
      if (Number.isNaN(a) || Number.isNaN(b) || a * b > 0) continue;
      if (a === b || Math.max(Math.abs(a), Math.abs(b)) < noise) continue;
      let wa = comp[i].slice(), wb = comp[j].slice();
      let fa = a;
      for (let t = 0; t < 70; t++) {
        const wm = projectFence(sc, fen, scl(add(wa, wb), 0.5));
        if (wm === null) break;
        const fm = sc.jhatW(wm, eps);
        if (Number.isNaN(fm)) break;
        if (fa * fm <= 0) wb = wm; else { wa = wm; fa = fm; }
      }
      const w0 = projectFence(sc, fen, scl(add(wa, wb), 0.5));
      if (w0 === null) continue;
      let w = polishPair(ww => sc.P.fval(fen, ww), ww => sc.P.fgrad(fen, ww),
                         ww => sc.jhatW(ww, eps), null, w0);
      // 抛光失败但二分点本身已在篱笆上且 Ĵ≈0 时直接采用: 出射临界篱笆上 Ĵ 含 √T_E, 梯度无界,
      // 差分 Newton 会失败, 而沿篱笆的二分早已收敛到机器精度. (原 C# 在此直接丢弃 → 漏掉出射端,
      // 例 Lowitz roll 90° 3-5 太阳 20°: 4 个焦散∩出射只找到 2 个.) 跳变间断处 |Ĵ| 不小, 不会被误收.
      if (w === null && Math.abs(sc.jhatW(w0, eps)) <= 1e-6 * scale && Math.abs(sc.P.fval(fen, w0)) < 1e-11) w = w0;
      if (w === null) continue;
      if (outw.some(z => near(z, w))) continue;
      outw.push(w);
    }
    // 兜底: |Ĵ| 局部极小 = 相切接触 (偶次零点不号变)
    for (let i = 0; i < m; i++) {
      const a = Js[(i - 1 + m) % m], b = Js[i], c = Js[(i + 1) % m];
      if (Number.isNaN(a) || Number.isNaN(b) || Number.isNaN(c)) continue;
      if (!(Math.abs(b) < Math.abs(a) && Math.abs(b) < Math.abs(c))) continue;
      if (Math.abs(b) > 0.05 * scale) continue;
      const w = polishPair(ww => sc.P.fval(fen, ww), ww => sc.P.fgrad(fen, ww),
                           ww => sc.jhatW(ww, eps), null, comp[i].slice());
      if (w === null) continue;
      if (Math.abs(sc.jhatW(w, eps)) > noise || Math.abs(sc.P.fval(fen, w)) > 1e-11) continue;
      const move = Math.max(Math.abs(w[0] - comp[i][0]), Math.max(Math.abs(w[1] - comp[i][1]), Math.abs(w[2] - comp[i][2])));
      if (move > 0.15) continue;
      if (outw.some(z => near(z, w))) continue;
      nTan++;
      if (nTan > 8) { outw.length = added0; break; }   // 病态分量整体丢弃
      outw.push(w);
    }
  }
  return outw;
}

// ================================================== (sc,sw) 图卡侧 (尖点用)
const _FE = new Float64Array(9);
function frameUV(m, fixedAng, scv, swv) {
  let e;
  if (m === OMode.ZenFree) e = orientEuler(scv, swv, fixedAng, _FE);
  else if (m === OMode.ZetaFixed) e = orientEuler(scv, fixedAng, swv, _FE);
  else e = orientEuler(fixedAng, swv, scv, _FE);
  return [[e[0], e[1], e[2]], [e[3], e[4], e[5]], [e[6], e[7], e[8]]];
}

function wUV(f, scv, swv) {
  const [e1, e2, e3] = frameUV(f.M, f.Fixed, scv, swv);
  const A = LP.A3;
  return [dot(A, e1), dot(A, e2), dot(A, e3)];
}

function jhatUV(sc, scv, swv) {
  const [e1, e2, e3] = frameUV(sc.F.M, sc.F.Fixed, scv, swv);
  const A = LP.A3;
  const w = [dot(A, e1), dot(A, e2), dot(A, e3)];
  const g = sc.F.Gw;
  const rho = [dot(g, e1), dot(g, e2), dot(g, e3)];
  const { Na, Nb, qE, TE } = sc.P.naNb(w);
  const Lam = cross(sc.F.Ec, rho);
  return TE * dot(Na, Lam) - qE * dot(Nb, Lam);
}

function dUV(sc, scv, swv) {
  const c = Chain3(scv, swv, sc.F.M, sc.F.Fixed, sc.PC);
  return [c.Dx, c.Dy, c.Dz];
}

// 尖点状态: Ĵ, ∇Ĵ, V = −Ĵ_v·D_u + Ĵ_u·D_v   (中心差分)
function cuspState(sc, u, v) {
  const h = 2e-6;
  const J0 = jhatUV(sc, u, v);
  const Ju = (jhatUV(sc, u + h, v) - jhatUV(sc, u - h, v)) / (2 * h);
  const Jv = (jhatUV(sc, u, v + h) - jhatUV(sc, u, v - h)) / (2 * h);
  const Dup = dUV(sc, u + h, v), Dum = dUV(sc, u - h, v);
  const Dvp = dUV(sc, u, v + h), Dvm = dUV(sc, u, v - h);
  const V = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    const Du = (Dup[i] - Dum[i]) / (2 * h);
    const Dv = (Dvp[i] - Dvm[i]) / (2 * h);
    V[i] = -Jv * Du + Ju * Dv;
  }
  return { J0, g: [Ju, Jv], V };
}

function minMarginUV(sc, u, v) {
  return sc.minMargin(wUV(sc.F, u, v));
}

function jGradUV(sc, u, v) {
  const h = 2e-6;
  const J0 = jhatUV(sc, u, v);
  return { J0, g: [(jhatUV(sc, u + h, v) - jhatUV(sc, u - h, v)) / (2 * h),
                   (jhatUV(sc, u, v + h) - jhatUV(sc, u, v - h)) / (2 * h)] };
}

// st = {u, v}: 原地校正 (C# 的 ref 参数 — 失败时已改动的值同样保留)
function correctOnCaustic(sc, st) {
  for (let it = 0; it < 3; it++) {
    const { J0, g } = jGradUV(sc, st.u, st.v);
    const n2 = g[0] * g[0] + g[1] * g[1];
    if (Number.isNaN(n2) || n2 < 1e-18) return false;
    st.u -= J0 * g[0] / n2; st.v -= J0 * g[1] / n2;
    if (Math.abs(J0) < 1e-14) break;
  }
  return !Number.isNaN(st.u) && !Number.isNaN(st.v);
}

function closeTor(a0, a1, b0, b1, tol) {
  const da = Math.abs(wrapPi(a0 - b0)), db = Math.abs(wrapPi(a1 - b1));
  return da < tol && db < tol;
}

// 沿 {Ĵ=0} 追踪找尖点: 环起步 + 贴壁起步 + 天球弦反转检测 + V 投影二分 + 对比度过滤
function findCusps(sc, seedsUV, seedFens) {
  const cusps = [];
  const step = 0.6 * Math.PI / 180;
  const r0 = 0.6 * Math.PI / 180;
  const starts = [];
  for (let si = 0; si < seedsUV.length; si++) {
    const u0 = seedsUV[si][0], v0 = seedsUV[si][1];
    let found = 0;
    const NP = 240;
    const vals = new Float64Array(NP), psis = new Float64Array(NP);
    for (let i = 0; i < NP; i++) {
      psis[i] = 2 * Math.PI * i / NP;
      vals[i] = NaN;
      const uu = u0 + r0 * Math.cos(psis[i]), vv = v0 + r0 * Math.sin(psis[i]);
      if (minMarginUV(sc, uu, vv) < 1e-4) continue;
      const jj = jhatUV(sc, uu, vv);
      if (Number.isFinite(jj)) vals[i] = jj;
    }
    for (let i = 0; i < NP; i++) {
      const j = (i + 1) % NP;
      const a = vals[i], b = vals[j];
      if (Number.isNaN(a) || Number.isNaN(b) || a * b > 0) continue;
      let pa = psis[i], pb = psis[i] + 2 * Math.PI / NP, fa = a;
      for (let t = 0; t < 45; t++) {
        const pm = 0.5 * (pa + pb);
        const fm = jhatUV(sc, u0 + r0 * Math.cos(pm), v0 + r0 * Math.sin(pm));
        if (Number.isNaN(fm)) break;
        if (fa * fm <= 0) pb = pm; else { pa = pm; fa = fm; }
      }
      const pmid = 0.5 * (pa + pb);
      starts.push([u0 + r0 * Math.cos(pmid), v0 + r0 * Math.sin(pmid), si]);
      found++;
    }
    // 贴壁起步: 相切接触的出射端 (焦散以 √m 分支贴着篱笆, 环上被余量滤掉)
    if (found === 0 && seedFens[si] !== null) {
      const fen = seedFens[si];
      const hh = 1e-6;
      const Fu = (sc.P.fval(fen, wUV(sc.F, u0 + hh, v0)) - sc.P.fval(fen, wUV(sc.F, u0 - hh, v0))) / (2 * hh);
      const Fv2 = (sc.P.fval(fen, wUV(sc.F, u0, v0 + hh)) - sc.P.fval(fen, wUV(sc.F, u0, v0 - hh))) / (2 * hh);
      const nF = Math.sqrt(Fu * Fu + Fv2 * Fv2);
      if (nF > 1e-13) {
        const nInU = fen.side * Fu / nF, nInV = fen.side * Fv2 / nF;
        const tfU = -nInV, tfV = nInU;
        const ds = 0.5 * Math.PI / 180;
        for (let dl = 0; dl < 2; dl++) {
          const sgn = dl === 0 ? 1 : -1;
          const pu = u0 + ds * sgn * tfU, pv = v0 + ds * sgn * tfV;
          const ms = [1e-7, 3e-7];
          for (let t = 0; t < 60; t++) ms.push(1e-6 * Math.pow(0.12 / 1e-6, t / 59.0));
          let prevM = NaN, prevJ = NaN;
          for (const mv of ms) {
            const qu = pu + mv * nInU, qv = pv + mv * nInV;
            if (minMarginUV(sc, qu, qv) < 0) { prevJ = NaN; continue; }
            const jj = jhatUV(sc, qu, qv);
            if (Number.isNaN(jj)) { prevJ = NaN; continue; }
            if (!Number.isNaN(prevJ) && prevJ * jj < 0) {
              let am = prevM, bm = mv, fa = prevJ;
              for (let t2 = 0; t2 < 50; t2++) {
                const mm = 0.5 * (am + bm);
                const fm = jhatUV(sc, pu + mm * nInU, pv + mm * nInV);
                if (Number.isNaN(fm)) break;
                if (fa * fm <= 0) bm = mm; else { am = mm; fa = fm; }
              }
              const mmid = 0.5 * (am + bm);
              starts.push([pu + mmid * nInU, pv + mmid * nInV, si]);
              break;
            }
            prevM = mv; prevJ = jj;
          }
        }
      }
    }
  }
  // 追踪 (弦反转检测: 天球折线相邻弦向量在尖点处反号; V 只在候选处精算)
  const doneSeeds = new Set();
  const startsD = [];
  for (const st0 of starts)
    if (!startsD.some(z => closeTor(st0[0], st0[1], z[0], z[1], 1.5 * step)))
      startsD.push(st0);
  for (const st of startsD) {
    const mySeed = Math.trunc(st[2]);
    if (doneSeeds.has(mySeed)) continue;
    for (let dir = 0; dir < 2; dir++) {
      const sgn = dir === 0 ? 1 : -1;
      const cur = { u: st[0], v: st[1] };
      if (!correctOnCaustic(sc, cur)) continue;
      let Dprev = null, chordPrev = null;
      let p1u = cur.u, p1v = cur.v, p2u = cur.u, p2v = cur.v;
      let nCand = 0;
      for (let k2 = 0; k2 < 1500; k2++) {
        const u = cur.u, v = cur.v;
        const { g } = jGradUV(sc, u, v);
        const ng = Math.sqrt(g[0] * g[0] + g[1] * g[1]);
        if (Number.isNaN(ng) || ng < 1e-14) break;
        if (k2 > 3 && minMarginUV(sc, u, v) < -1e-6) break;
        if (k2 > 12) {
          let hit = -1;
          for (let si2 = 0; si2 < seedsUV.length; si2++)
            if (closeTor(u, v, seedsUV[si2][0], seedsUV[si2][1], 1.5 * step)) { hit = si2; break; }
          if (hit >= 0) { doneSeeds.add(hit); break; }   // 抵达另一末端: 弧走完
        }
        const Dnow = dUV(sc, u, v);
        if (Dprev !== null) {
          const chord = sub(Dnow, Dprev);
          const nch = nrm(chord);
          if (chordPrev !== null && nch > 1e-9 && nrm(chordPrev) > 1e-9 &&
              dot(chord, chordPrev) < 0 && nCand < 12) {
            nCand++;
            // 候选: 尖点在 (p2, 当前) 之间 — 用 V 投影二分定位
            const Sp = cuspState(sc, p2u, p2v);
            const nVp = nrm(Sp.V);
            if (nVp > 0) {
              const rfix = scl(Sp.V, 1.0 / nVp);
              let au = p2u, av = p2v, bu = u, bv = v;
              let sa = nVp;
              for (let t = 0; t < 52; t++) {
                const mid = { u: 0.5 * (au + bu), v: 0.5 * (av + bv) };
                if (!correctOnCaustic(sc, mid)) break;
                const Sm = cuspState(sc, mid.u, mid.v);
                const sm = dot(Sm.V, rfix);
                if (sa * sm <= 0) { bu = mid.u; bv = mid.v; }
                else { au = mid.u; av = mid.v; sa = sm; }
              }
              const cc = { u: 0.5 * (au + bu), v: 0.5 * (av + bv) };
              correctOnCaustic(sc, cc);
              const uc = cc.u, vc = cc.v;
              const Sc = cuspState(sc, uc, vc);
              const okm = minMarginUV(sc, uc, vc) > -1e-7;
              let okIso = false;
              if (okm && Math.abs(Sc.J0) < 1e-9) {
                // 对比度过滤: 真尖点两侧 ±3 步 |V| 须显著回升
                const gc = Sc.g;
                const ngc = Math.sqrt(gc[0] * gc[0] + gc[1] * gc[1]);
                if (ngc > 1e-300) {
                  const tcU = -gc[1] / ngc, tcV = gc[0] / ngc;
                  const nbs = [];
                  for (let dd = 0; dd < 2; dd++) {
                    const s2 = dd === 0 ? 3.0 : -3.0;
                    const xs = { u: uc + s2 * step * tcU, v: vc + s2 * step * tcV };
                    if (!correctOnCaustic(sc, xs)) continue;
                    nbs.push(nrm(cuspState(sc, xs.u, xs.v).V));
                  }
                  const nVc = nrm(Sc.V);
                  okIso = nbs.length === 2 && Math.min(nbs[0], nbs[1]) > Math.max(1e3 * nVc, 1e-8);
                }
              }
              if (okIso && !cusps.some(cp => closeTor(uc, vc, cp.U, cp.V, 1e-5)))
                cusps.push({ U: (uc % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI),
                             V: (vc % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI), ResV: nrm(Sc.V) });
            }
          }
          chordPrev = chord;
        }
        Dprev = Dnow; p2u = p1u; p2v = p1v; p1u = u; p1v = v;
        const tU = sgn * (-g[1]) / ng, tV = sgn * g[0] / ng;
        cur.u = u + step * tU; cur.v = v + step * tV;
        if (!correctOnCaustic(sc, cur)) break;
      }
    }
  }
  return cusps;
}

// ================================================== 主入口
/**
 * 末端点 (end/turn/cross/cusp). 记录 { kind, label, W, eps, sc, sw (deg), D, az, el }.
 * @param {number[]} faces 光路面号
 */
export function computeEndpoints(faces, m, fixedAng, withCusps = true) {
  const recs = [];
  const fam = FamC.make(m, fixedAng);
  if (fam === null) return recs;
  if (Math.abs(Math.sin(m === OMode.ZetaFixed ? fixedAng : Math.PI / 2)) < 1e-9) return recs;  // 板晶退化 1DOF
  const pctx = new PathCtx(faces);
  const pc = new PathC(faces, LP.partial);
  const sc = new Scene(pc, fam, pctx);

  const addPt = (w, eps, kind, label) => {
    if (fam.gram(w) < -1e-9) return;
    if (sc.minMargin(w) < -1e-9) return;
    for (const r0 of recs) {
      if (r0.eps === eps && Math.abs(r0.W[0] - w[0]) < 1e-8 && Math.abs(r0.W[1] - w[1]) < 1e-8 && Math.abs(r0.W[2] - w[2]) < 1e-8) {
        if (!r0.label.includes(label)) r0.label += ' | ' + label;
        const rank0 = r0.kind === 'end' ? 3 : (r0.kind === 'turn' ? 2 : 1);
        const rank1 = kind === 'end' ? 3 : (kind === 'turn' ? 2 : 1);
        if (rank1 > rank0) r0.kind = kind;
        return;
      }
    }
    const E = fam.lift(w, eps);
    if (E === null) return;
    const [scv, swv] = fam.uvOf(E[0], E[1], E[2]);
    const wc = wUV(fam, scv, swv);
    if (Math.abs(wc[0] - w[0]) > 1e-6 || Math.abs(wc[1] - w[1]) > 1e-6 || Math.abs(wc[2] - w[2]) > 1e-6) return;
    const c3 = Chain3(scv, swv, m, fixedAng, pctx);
    let D = [c3.Dx, c3.Dy, c3.Dz];
    const nn = nrm(D); if (nn > 1e-12) D = scl(D, 1.0 / nn);
    recs.push({
      kind, label, W: w.slice(), eps,
      sc: scv * 180 / Math.PI, sw: swv * 180 / Math.PI, D,
      az: Math.atan2(D[1], D[0]) * 180 / Math.PI,
      el: Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI,
    });
  };

  // (a) 几何曲线两两相交
  for (let i = 0; i < sc.Geo.length; i++)
    for (let j = i + 1; j < sc.Geo.length; j++) {
      const fi = sc.Geo[i], fj = sc.Geo[j];
      const kind = (fi.side !== 0 && fj.side !== 0) ? 'end' : 'cross';
      for (const w of solveLL(sc, fi, fj)) {
        addPt(w, +1, kind, fi.name + '∩' + fj.name); addPt(w, -1, kind, fi.name + '∩' + fj.name);
      }
    }
  // (b) 几何篱笆 ∩ 可达带圆 = 转折
  for (const fen of sc.Geo)
    for (const rf of sc.Reach)
      for (const w of solveLL(sc, fen, rf))
        addPt(w, +1, 'turn', fen.name + '∩' + rf.name);
  // (c) 焦散 ∩ 几何曲线
  for (const fen of sc.Geo)
    for (let e = 0; e < 2; e++) {
      const eps = e === 0 ? 1 : -1;
      for (const w of solveCausticFence(sc, fen, eps, 1600))
        addPt(w, eps, fen.side !== 0 ? 'end' : 'cross', '焦散∩' + fen.name);
    }
  // (d) 焦散 ∩ 可达带圆 = 转折
  for (const rf of sc.Reach)
    for (const w of solveCausticFence(sc, rf, +1, 1600))
      addPt(w, +1, 'turn', '焦散∩' + rf.name);
  // (e) 尖点: 从焦散 end/turn 种子沿曲线追踪
  if (withCusps) {
    const seedsUV = [], seedFens = [];
    for (const r of recs) {
      if (!r.label.includes('焦散')) continue;
      if (r.kind !== 'end' && r.kind !== 'turn') continue;
      seedsUV.push([r.sc * Math.PI / 180, r.sw * Math.PI / 180]);
      let sf = null;
      for (const fen of sc.Geo)
        if (r.label.includes('焦散∩' + fen.name)) { sf = fen; break; }
      seedFens.push(sf);
    }
    for (const cp of findCusps(sc, seedsUV, seedFens)) {
      const w = wUV(fam, cp.U, cp.V);
      const c3 = Chain3(cp.U, cp.V, m, fixedAng, pctx);
      let D = [c3.Dx, c3.Dy, c3.Dz];
      const nn = nrm(D); if (nn > 1e-12) D = scl(D, 1.0 / nn);
      recs.push({
        kind: 'cusp', label: '焦散尖点 W=0 (pleat)', W: w, eps: 0,
        sc: cp.U * 180 / Math.PI, sw: cp.V * 180 / Math.PI, D,
        az: Math.atan2(D[1], D[0]) * 180 / Math.PI,
        el: Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI,
      });
    }
  }
  return recs;
}
