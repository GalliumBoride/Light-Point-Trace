// OrientationManifoldForm 里属于"计算"的私有逻辑 (逐句移植):
// 边界族连线 GetFamLines (等值线 + 有效性翻转角点 + 吸附精确末端点), 孔径末端点 ApertureEndpoints,
// 勾选族满足侧点云 SampleCloud, 以及输入解析 / 取向模式换算.

import {
  LP, OMode, rint, faceOk, isPyramidFace, famValue, validForFam, validLp, skyD,
  traceContours, noiseFloorOf, axesMeta,
} from './lp.js';
import { AP, apMargin, apWOf } from './aperture.js';
import { DotNetRandom } from './random.js';

// ---------- 输入 ----------
/** 光路文本 → 面号数组列表; 无效返回 null (同 ParsePaths: 1-6 条, 每条 ≥2 面). */
export function parsePaths(text) {
  const groups = text.split(/[,;，；\s]+/).filter(s => s.length > 0);
  const res = [];
  for (const g of groups) {
    const parts = g.split('-');
    const p = [];
    for (const t of parts) {
      const s = t.trim();
      if (!/^[+-]?\d+$/.test(s)) return null;
      const f = parseInt(s, 10);
      const ok = f === 1 || f === 2 || (f >= 3 && f <= 8) || isPyramidFace(f);
      if (!ok) return null;
      p.push(f);
    }
    if (p.length < 2) return null;
    res.push(p);
  }
  if (res.length < 1 || res.length > 6) return null;
  return res;
}

/** 取向模式 (0=柱晶 1=板晶 2=Parry 3=Lowitz) + 固定角(°) → {om, fx(rad)} (同 ReadMode). */
export function modeParams(modeIdx, adeg) {
  if (!Number.isFinite(adeg)) adeg = 90;
  switch (modeIdx) {
    case 0: return { om: OMode.ZetaFixed, fx: Math.max(0, Math.min(180, adeg)) * Math.PI / 180 };
    case 1: return { om: OMode.ZetaFixed, fx: 0 };
    case 2: return { om: OMode.ZetaFixed, fx: Math.PI / 2 };
    default: return { om: OMode.ZenFree, fx: (adeg % 360) * Math.PI / 180 };
  }
}

export { faceOk };

// ---------- 边界族连线 ----------
// 把角点估计投影回本族曲线 F=0 (弦上二分出的点会偏离真实曲线)
function projectToCurve(Fv, p) {
  const eps = 2e-4;
  let x = p[0], y = p[1];
  for (let it = 0; it < 2; it++) {
    const f0 = Fv(x, y);
    if (Number.isNaN(f0)) break;
    const gx = (Fv(x + eps, y) - Fv(x - eps, y)) / (2 * eps);
    const gy = (Fv(x, y + eps) - Fv(x, y - eps)) / (2 * eps);
    const g2 = gx * gx + gy * gy;
    if (!(g2 > 1e-20)) break;
    x -= f0 * gx / g2; y -= f0 * gy / g2;
  }
  return [x, y];
}

// 相接点注册表: 第一次算出的角点登记, 后来者在容差内吸附成同一个取向点
function snapJunction(reg, p, tol, perX, Px, perY, Py) {
  for (const r of reg) {
    let dx = p[0] - r[0], dy = p[1] - r[1];
    if (perX && Px > 0) dx -= Px * rint(dx / Px);
    if (perY && Py > 0) dy -= Py * rint(dy / Py);
    if (dx * dx + dy * dy < tol * tol) return [p[0] - dx, p[1] - dy];
  }
  reg.push([p[0], p[1]]);
  return p;
}

/**
 * 2DOF: 取向参数平面上 F=0 的等值线 (marching squares + 跨缝链接 + 断头延拓), 按取向空间顺序;
 * 有效性翻转处二分出角点并吸附到精确末端点 (eps 表) 或注册表, 相邻边界首尾相接.
 * 返回 { par: Float64Array[] (sc,sw rad 展开坐标), sky: Float64Array[] (x,y,z,ok) }
 */
export function getFamLines(P, ori, eps, fm, perX, perY, om, fx, regList) {
  const fl = { par: [], sky: [] };
  const Fg = ori ? ori.F.get(fm.name) : undefined;
  if (!Fg) return fl;
  const FV = (x, y) => famValue(fm, P, om, fx, x, y);
  // 网格边上的零点用族函数二分细化 (线性插值会偏离真实曲线)
  const refine = (xa, ya, xb, yb, fa, fb) => {
    let lo = 0, hi = 1, flo = fa;
    for (let it = 0; it < 20; it++) {
      const mid = 0.5 * (lo + hi);
      const fmv = FV(xa + (xb - xa) * mid, ya + (yb - ya) * mid);
      if (Number.isNaN(fmv)) break;
      if (flo * fmv <= 0) hi = mid; else { lo = mid; flo = fmv; }
    }
    return 0.5 * (lo + hi);
  };
  const SCs = ori.SCs, SWs = ori.SWs;
  const PxJ = SCs[SCs.length - 1] - SCs[0];
  const PyJ = SWs[SWs.length - 1] - SWs[0];
  const tolJ = Math.max(PxJ / (SCs.length - 1), PyJ / (SWs.length - 1)) * 0.35;
  // 精确末端点表: 有效性翻转的角点直接吸附到闭式交点上
  let exactEp = null;
  if (eps && eps.length > 0) {
    exactEp = [];
    for (const er of eps)
      if (er.kind === 'end' || er.kind === 'cross' || er.kind === 'aper')
        exactEp.push([er.sc * Math.PI / 180, er.sw * Math.PI / 180]);
  }
  const valid = (x, y) => validForFam(fm, P, om, fx, x, y, 1e-6);
  for (const c of traceContours(Fg, SCs, SWs, perX, perY, refine, FV, noiseFloorOf(Fg, ori.nW, ori.nS))) {
    const Pl = [];          // [sc, sw, ok]
    let prevPt = null, prevOk2 = false;
    for (const pt of c.pts) {
      const okv = valid(pt[0], pt[1]);
      if (prevPt !== null && okv !== prevOk2) {
        const v = prevOk2 ? prevPt : pt, iv = prevOk2 ? pt : prevPt;
        let lo = 0, hi = 1;
        for (let it = 0; it < 24; it++) {
          const mid = 0.5 * (lo + hi);
          if (valid(v[0] + (iv[0] - v[0]) * mid, v[1] + (iv[1] - v[1]) * mid)) lo = mid; else hi = mid;
        }
        const jraw = projectToCurve(FV, [v[0] + (iv[0] - v[0]) * lo, v[1] + (iv[1] - v[1]) * lo]);
        let jj = null;
        if (exactEp !== null) {
          let bestD2 = Number.MAX_VALUE, bestP = null;
          for (const ep of exactEp) {
            let dx = jraw[0] - ep[0], dy = jraw[1] - ep[1];
            if (perX) dx -= 2 * Math.PI * rint(dx / (2 * Math.PI));
            if (perY) dy -= 2 * Math.PI * rint(dy / (2 * Math.PI));
            const d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; bestP = [jraw[0] - dx, jraw[1] - dy]; }
          }
          const tolE = Math.max(2 * tolJ, 1.2 * Math.PI / 180);
          if (bestP !== null && bestD2 < tolE * tolE) jj = bestP;   // 精确点 (平移到当前展开分支)
        }
        if (jj === null) jj = snapJunction(regList, jraw, tolJ, perX, PxJ, perY, PyJ);
        Pl.push([jj[0], jj[1], 1.0]);
      }
      Pl.push([pt[0], pt[1], okv ? 1.0 : 0.0]);
      prevPt = pt; prevOk2 = okv;
    }
    const n = Pl.length;
    const par = new Float64Array(n * 2), sky = new Float64Array(n * 4);
    for (let i = 0; i < n; i++) {
      const sc = Pl[i][0], sw = Pl[i][1];
      par[2 * i] = sc; par[2 * i + 1] = sw;
      const D = skyD(P, om, fx, sc, sw);
      const nn = Math.sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
      if (nn > 1e-12) { D[0] /= nn; D[1] /= nn; D[2] /= nn; }
      sky[4 * i] = D[0]; sky[4 * i + 1] = D[1]; sky[4 * i + 2] = D[2];
      sky[4 * i + 3] = Pl[i][2];
    }
    fl.par.push(par); fl.sky.push(sky);
  }
  // 焦散补线 (原窗体没有): 焦散在出射临界篱笆外无定义 (J = NaN), 网格追踪总在离末端点一两个胞元处停下,
  // 或在贴着篱笆的细条里折回. 单独存放, par/sky 仍与原窗体逐点一致.
  fl.conn = fm.kind === 6 && eps && eps.length ? causticConnectors(fl, eps, FV, P, om, fx, tolJ) : { par: [], sky: [] };
  return fl;
}

const D2R = Math.PI / 180;

/**
 * 对每个没被焦散线够到的焦散末端点 (焦散∩篱笆), 从最近的线点连一段到它.
 * 中间点逐个解在 J = 0 上 (沿弦的法线找号变), 不是直线弦 — 贴篱笆处天球位置 ∝ √(到篱笆距离), 弦会偏得很明显.
 * 大部分中间点解不出焦散 → 两者之间本无焦散, 不连.
 * 贴篱笆的焦散常是一条比胞元还细的长环: 网格在环变细处提前把它闭合, 尖端只剩一个两点残桩, 缺口可达好几度.
 * 所以找起点时跳过残桩 (整条线不出 tolJ), 缺口超过 3° 时改为沿 J = 0 一步步走过去 (直弦离曲线太远).
 */
function causticConnectors(fl, eps, FV, P, om, fx, tolJ) {
  const out = { par: [], sky: [] };
  const TWO = 2 * Math.PI;
  const wrapD = d => d - TWO * rint(d / TWO);
  const stub = fl.par.map(par => {
    for (let i = 1; i < par.length / 2; i++)
      if (Math.hypot(wrapD(par[2 * i] - par[0]), wrapD(par[2 * i + 1] - par[1])) > tolJ) return false;
    return true;
  });
  for (const er of eps) {
    if ((er.kind !== 'end' && er.kind !== 'cross') || !er.label.includes('焦散∩')) continue;
    const ex = er.sc * D2R, ey = er.sw * D2R;
    let best = Infinity, V = null, Vok = 1;
    for (let li = 0; li < fl.par.length; li++) {
      if (stub[li]) continue;
      const par = fl.par[li], sky = fl.sky[li];
      for (let i = 0; i < par.length / 2; i++) {
        const dx = wrapD(par[2 * i] - ex), dy = wrapD(par[2 * i + 1] - ey), dd = dx * dx + dy * dy;
        if (dd < best) { best = dd; V = [par[2 * i], par[2 * i + 1]]; Vok = sky[4 * i + 3]; }
      }
    }
    if (V === null) continue;
    const dist = Math.sqrt(best);
    if (dist < 0.08 * D2R || dist > 8 * D2R) continue;          // 已经够到 / 太远 (不是这条线的端)
    const E = [V[0] - wrapD(V[0] - ex), V[1] - wrapD(V[1] - ey)];   // 平移到 V 的展开分支
    let pts;
    if (dist > 3 * D2R) {
      pts = walkCaustic(FV, V, E, dist);
      if (pts === null) continue;
    } else {
      const tx = (E[0] - V[0]) / dist, ty = (E[1] - V[1]) / dist;
      const N = Math.max(4, Math.ceil(dist / (0.04 * D2R)));
      const h = Math.max(0.3 * dist, 0.05 * D2R);
      pts = [V];
      let solved = 0;
      for (let k = 1; k < N; k++) {
        const Q = [V[0] + (E[0] - V[0]) * k / N, V[1] + (E[1] - V[1]) * k / N];
        const q = causticOnNormal(FV, Q, -ty, tx, h);
        if (q !== null) { pts.push(q); solved++; }
      }
      if (solved < 0.6 * (N - 1)) continue;
      pts.push(E);
    }
    const n = pts.length, par = new Float64Array(2 * n), sky = new Float64Array(4 * n);
    for (let i = 0; i < n; i++) {
      par[2 * i] = pts[i][0]; par[2 * i + 1] = pts[i][1];
      const D = skyD(P, om, fx, pts[i][0], pts[i][1]);
      const nn = Math.hypot(D[0], D[1], D[2]) || 1;
      sky[4 * i] = D[0] / nn; sky[4 * i + 1] = D[1] / nn; sky[4 * i + 2] = D[2] / nn; sky[4 * i + 3] = Vok;
    }
    out.par.push(par); out.sky.push(sky);
  }
  return out;
}

// 从 V 沿 J = 0 走到 E (预测-校正, 步长 0.04°): 沿切线前进一步, 再在法线上解回曲线.
// 解不出 / 折返 / 越走越远 → 这条线到不了 E, 返回 null.
function walkCaustic(FV, V, E, dist) {
  const s = 0.04 * D2R, pts = [V];
  let C = V, tx = (E[0] - V[0]) / dist, ty = (E[1] - V[1]) / dist;
  for (let k = 0, kMax = Math.ceil(2 * dist / s); k < kMax; k++) {
    const dE = Math.hypot(E[0] - C[0], E[1] - C[1]);
    if (dE < 1.5 * s) { pts.push(E); return pts; }
    if (dE > 1.2 * dist) return null;
    const q = causticOnNormal(FV, [C[0] + tx * s, C[1] + ty * s], -ty, tx, 3 * s);
    if (q === null) return null;
    const sx = q[0] - C[0], sy = q[1] - C[1], sl = Math.hypot(sx, sy);
    if (!(sl > 1e-12) || sl > 4 * s || sx * tx + sy * ty <= 0) return null;
    tx = sx / sl; ty = sy / sl; C = q; pts.push(q);
  }
  return null;
}

// 沿 Q + t·(nx,ny), t∈[−h,h] 找 J = 0, 取离 Q 最近的根. 一端越过出射篱笆 (J = NaN) 时,
// 先二分出篱笆位置, 再贴着篱笆往里找贴壁细条中的号变 (细条宽度随离末端点的距离平方缩小).
function causticOnNormal(FV, Q, nx, ny, h) {
  const at = t => FV(Q[0] + nx * t, Q[1] + ny * t);
  const K = 24, ts = [], fs = [];
  for (let k = 0; k <= K; k++) { const t = -h + 2 * h * k / K; ts.push(t); fs.push(at(t)); }
  const bisect = (a, fa, b) => {
    for (let it = 0; it < 50; it++) {
      const m = 0.5 * (a + b), fm = at(m);
      if (!Number.isFinite(fm) || fa * fm <= 0) b = m; else { a = m; fa = fm; }
    }
    return 0.5 * (a + b);
  };
  const roots = [];
  for (let k = 0; k < K; k++) {
    const a = fs[k], b = fs[k + 1], okA = Number.isFinite(a), okB = Number.isFinite(b);
    if (okA && okB) { if (a * b < 0) roots.push(bisect(ts[k], a, ts[k + 1])); continue; }
    if (okA === okB) continue;
    const t0 = okA ? ts[k] : ts[k + 1], f0 = okA ? a : b;
    let tIn = t0, tOut = okA ? ts[k + 1] : ts[k];
    for (let it = 0; it < 50; it++) { const m = 0.5 * (tIn + tOut); if (Number.isFinite(at(m))) tIn = m; else tOut = m; }
    const span = tIn - t0;
    for (const e of [0.3, 0.1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4, 3e-5, 1e-5, 1e-6]) {
      const t2 = tIn - span * e, f2 = at(t2);
      if (Number.isFinite(f2) && f2 * f0 < 0) { roots.push(bisect(t0, f0, t2)); break; }
    }
  }
  if (roots.length === 0) return null;
  let tb = roots[0];
  for (const t of roots) if (Math.abs(t) < Math.abs(tb)) tb = t;
  return [Q[0] + nx * tb, Q[1] + ny * tb];
}

// ---------- 孔径末端点 (真实晶体) ----------
// 真实尺寸不移动弧, 只把弧剪断: 剪断处 = 孔径篱笆 (margin=0) ∩ 理想族曲线 (F=0).
function zeroSeg(f00, f01, f11, f10, x0, x1, y0, y1, seg) {
  let n = 0;
  if ((f00 < 0) !== (f01 < 0)) { seg[0] = x0 + (x1 - x0) * (f00 / (f00 - f01)); seg[1] = y0; n = 1; }
  if ((f01 < 0) !== (f11 < 0) && n < 2) { seg[2 * n] = x1; seg[2 * n + 1] = y0 + (y1 - y0) * (f01 / (f01 - f11)); n++; }
  if ((f10 < 0) !== (f11 < 0) && n < 2) { seg[2 * n] = x0 + (x1 - x0) * (f10 / (f10 - f11)); seg[2 * n + 1] = y1; n++; }
  if ((f00 < 0) !== (f10 < 0) && n < 2) { seg[2 * n] = x0; seg[2 * n + 1] = y0 + (y1 - y0) * (f00 / (f00 - f10)); n++; }
  return n === 2;
}

function addEp(res, P, om, fx, u, v, tolD, label) {
  for (const r0 of res) {
    let ddx = r0.sc * Math.PI / 180 - u, ddy = r0.sw * Math.PI / 180 - v;
    ddx -= 2 * Math.PI * rint(ddx / (2 * Math.PI));
    ddy -= 2 * Math.PI * rint(ddy / (2 * Math.PI));
    if (ddx * ddx + ddy * ddy < tolD * tolD) return;
  }
  const D = skyD(P, om, fx, u, v);
  const nn = Math.sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
  if (nn > 1e-12) { D[0] /= nn; D[1] /= nn; D[2] /= nn; }
  const W = apWOf(om, fx, u, v);
  res.push({
    kind: 'aper', label, W, eps: 0,
    sc: u * 180 / Math.PI, sw: v * 180 / Math.PI, D,
    az: Math.atan2(D[1], D[0]) * 180 / Math.PI,
    el: Math.asin(Math.max(-1, Math.min(1, D[2]))) * 180 / Math.PI,
  });
}

export function apertureEndpoints(P, ori, om, fx) {
  const res = [];
  if (!AP.on || !ori || !P) return res;
  let apFam = null;
  for (const fm of ori.fams) if (fm.kind === 7) apFam = fm;
  if (apFam === null) return res;
  const Fa = ori.F.get(apFam.name);
  if (!Fa) return res;
  const SCs = ori.SCs, SWs = ori.SWs;
  const nS = SCs.length, nW = SWs.length;
  if (nS < 2 || nW < 2) return res;
  const dS = SCs[1] - SCs[0], dW = SWs[1] - SWs[0];
  const tolD = 1.2 * Math.max(Math.abs(dS), Math.abs(dW));
  const at = (G, iw, isc) => G[iw * nS + isc];
  // ---- 源 A: 沿孔径篱笆等值线走, 有效性翻转处 = 弧真正的断点 (与绘制弧线时的切断逻辑同源) ----
  {
    const apFV = (x, y) => famValue(apFam, P, om, fx, x, y);
    const refA = (xa, ya, xb, yb, fa3) => {
      let lo3 = 0, hi3 = 1, flo3 = fa3;
      for (let it = 0; it < 24; it++) {
        const mid = 0.5 * (lo3 + hi3);
        const fv3 = apFV(xa + (xb - xa) * mid, ya + (yb - ya) * mid);
        if (Number.isNaN(fv3)) break;
        if (flo3 * fv3 <= 0) hi3 = mid; else { lo3 = mid; flo3 = fv3; }
      }
      return 0.5 * (lo3 + hi3);
    };
    for (const c of traceContours(Fa, SCs, SWs, true, true, refA, apFV, noiseFloorOf(Fa, nW, nS))) {
      let prevP = null, prevOk2 = false;
      for (const pt of c.pts) {
        const ok2 = validLp(P, om, fx, pt[0], pt[1], 1e-6);
        if (prevP !== null && ok2 !== prevOk2 && res.length < 400) {
          const vv = prevOk2 ? prevP : pt, iv = prevOk2 ? pt : prevP;
          let lo4 = 0, hi4 = 1;
          for (let it = 0; it < 40; it++) {
            const mid = 0.5 * (lo4 + hi4);
            if (validLp(P, om, fx, vv[0] + (iv[0] - vv[0]) * mid, vv[1] + (iv[1] - vv[1]) * mid, 1e-6)) lo4 = mid; else hi4 = mid;
          }
          const u2 = vv[0] + (iv[0] - vv[0]) * lo4, v2 = vv[1] + (iv[1] - vv[1]) * lo4;
          // 责任族 = 此处最接近零的约束型族 (Kind<=3), 用于标注
          let owner = null, bestA = Number.MAX_VALUE;
          for (const fq of ori.fams) {
            if (fq.kind > 3) continue;
            const av = Math.abs(famValue(fq, P, om, fx, u2, v2));
            if (!Number.isNaN(av) && av < bestA) { bestA = av; owner = fq; }
          }
          addEp(res, P, om, fx, u2, v2, tolD, '孔径∩' + (owner !== null ? owner.name : 'Ω边界'));
        }
        prevP = pt; prevOk2 = ok2;
      }
    }
  }

  const segB = new Float64Array(4);
  for (const fm of ori.fams) {
    if (fm.kind === 7) continue;
    const Fb = ori.F.get(fm.name);
    if (!Fb) continue;
    // 双向检测: pass0 = 沿本族曲线找 margin 过零; pass1 = 沿孔径篱笆找本族 F 过零
    const tMargin = (uu, vv) => apMargin(P, om, fx, uu, vv);
    const tFam = (uu, vv) => famValue(fm, P, om, fx, uu, vv);
    for (let pass = 0; pass < 2; pass++) {
      const Fc = pass === 0 ? Fb : Fa;
      const tf = pass === 0 ? tMargin : tFam;
      for (let iw = 0; iw + 1 < nW && res.length < 400; iw++) {
        const r0 = iw * nS, r1 = r0 + nS;
        for (let isc = 0; isc + 1 < nS && res.length < 400; isc++) {
          const c00 = Fc[r0 + isc], c01 = Fc[r0 + isc + 1], c11 = Fc[r1 + isc + 1], c10 = Fc[r1 + isc];
          // NaN 比较恒假: 任一为 NaN 时 cb 判定前先排除 (同 C# IsNaN 检查)
          if (c00 !== c00 || c01 !== c01 || c11 !== c11 || c10 !== c10) continue;
          const cb = (c00 < 0 ? 1 : 0) + (c01 < 0 ? 1 : 0) + (c11 < 0 ? 1 : 0) + (c10 < 0 ? 1 : 0);
          if (cb === 0 || cb === 4) continue;
          const x0 = SCs[isc], x1 = SCs[isc + 1], y0 = SWs[iw], y1 = SWs[iw + 1];
          if (!zeroSeg(c00, c01, c11, c10, x0, x1, y0, y1, segB)) continue;
          const ma = tf(segB[0], segB[1]);
          const mb = tf(segB[2], segB[3]);
          if (Number.isNaN(ma) || Number.isNaN(mb)) continue;
          if ((ma < 0) === (mb < 0)) continue;
          let t0 = 0, t1 = 1, fa2 = ma;
          for (let it = 0; it < 40; it++) {
            const tm = 0.5 * (t0 + t1);
            const mm = tf(segB[0] + (segB[2] - segB[0]) * tm, segB[1] + (segB[3] - segB[1]) * tm);
            if (Number.isNaN(mm)) break;
            if ((fa2 < 0) === (mm < 0)) { t0 = tm; fa2 = mm; } else t1 = tm;
          }
          const tb = 0.5 * (t0 + t1);
          const ub = segB[0] + (segB[2] - segB[0]) * tb;
          const vb = segB[1] + (segB[3] - segB[1]) * tb;
          let u = ub, v = vb;
          const eps = 1e-5;
          for (let it = 0; it < 14; it++) {
            const f1 = tFam(u, v);
            const f2 = tMargin(u, v);
            if (Number.isNaN(f1) || Number.isNaN(f2)) break;
            if (Math.abs(f2) < 1e-14) break;
            const j11 = (tFam(u + eps, v) - tFam(u - eps, v)) / (2 * eps);
            const j12 = (tFam(u, v + eps) - tFam(u, v - eps)) / (2 * eps);
            const j21 = (tMargin(u + eps, v) - tMargin(u - eps, v)) / (2 * eps);
            const j22 = (tMargin(u, v + eps) - tMargin(u, v - eps)) / (2 * eps);
            const det = j11 * j22 - j12 * j21;
            if (Number.isNaN(det) || Math.abs(det) < 1e-14) break;
            let du = (-f1 * j22 + f2 * j12) / det;
            let dv = (-f2 * j11 + f1 * j21) / det;
            const lim = 2 * Math.max(Math.abs(dS), Math.abs(dW));
            const mag = Math.sqrt(du * du + dv * dv);
            if (!(mag > 0) || Number.isNaN(mag)) break;
            if (mag > lim) { du *= lim / mag; dv *= lim / mag; }
            u += du; v += dv;
          }
          // Newton 点 vs 二分点: 用胞元内变化幅度做尺度, 取归一化残差更小者
          const bmin = Math.min(Math.min(at(Fb, iw, isc), at(Fb, iw, isc + 1)), Math.min(at(Fb, iw + 1, isc + 1), at(Fb, iw + 1, isc)));
          const bmax = Math.max(Math.max(at(Fb, iw, isc), at(Fb, iw, isc + 1)), Math.max(at(Fb, iw + 1, isc + 1), at(Fb, iw + 1, isc)));
          const amin = Math.min(Math.min(at(Fa, iw, isc), at(Fa, iw, isc + 1)), Math.min(at(Fa, iw + 1, isc + 1), at(Fa, iw + 1, isc)));
          const amax = Math.max(Math.max(at(Fa, iw, isc), at(Fa, iw, isc + 1)), Math.max(at(Fa, iw + 1, isc + 1), at(Fa, iw + 1, isc)));
          const rF = Math.max(bmax - bmin, 1e-300), rM = Math.max(amax - amin, 1e-300);
          const resid = (uu, vv) => {
            const f1 = tFam(uu, vv), f2 = tMargin(uu, vv);
            if (Number.isNaN(f1) || Number.isNaN(f2)) return Infinity;
            return Math.abs(f1) / rF + Math.abs(f2) / rM;
          };
          const sN = (u < x0 - 2 * dS || u > x1 + 2 * dS || v < y0 - 2 * dW || v > y1 + 2 * dW) ? Infinity : resid(u, v);
          if (!(sN <= resid(ub, vb))) { u = ub; v = vb; }
          // 其余光点判据必须成立 (否则这段弧本来就不存在)
          if (!validLp(P, om, fx, u, v, 1e-6)) continue;
          addEp(res, P, om, fx, u, v, tolD, '孔径∩' + fm.name);
        }
      }
    }
  }
  return res;
}

/** 理想末端点是否仍然活着 (孔径层开启时, 落在真实晶体死区的要隐藏). */
export function epAlive(P, om, fx, r) {
  if (!AP.on || !P || r.kind === 'aper') return true;
  const m = apMargin(P, om, fx, r.sc * Math.PI / 180, r.sw * Math.PI / 180);
  return Number.isNaN(m) || m > -1e-7;
}

/** 勾选族满足侧点云 (Random(23), 24 万采样). 返回 Float32Array (x,y,z). */
export function sampleCloud(P, fm, om, fx) {
  const ax = axesMeta(om);
  const X0 = ax.x0 * Math.PI / 180, X1 = ax.x1 * Math.PI / 180;
  const Y0 = ax.y0 * Math.PI / 180, Y1 = ax.y1 * Math.PI / 180;
  const rng = new DotNetRandom(23);
  const pts = [];
  for (let i = 0; i < 240000; i++) {
    const sc = X0 + rng.nextDouble() * (X1 - X0);
    const sw = Y0 + rng.nextDouble() * (Y1 - Y0);
    const v = famValue(fm, P, om, fx, sc, sw);
    if (Number.isNaN(v) || fm.sense * v < 0) continue;
    const D = skyD(P, om, fx, sc, sw);
    const nn = Math.sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
    if (nn > 1e-12) { D[0] /= nn; D[1] /= nn; D[2] /= nn; }
    pts.push(D[0], D[1], D[2]);
  }
  return Float32Array.from(pts);
}
