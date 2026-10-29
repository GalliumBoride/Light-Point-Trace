// 「3D 抬升」视图的几何 (在取向平面 Worker 里算): 把取向平面上光路成立的区域做成三角网格,
// 每个顶点放到它把光送去的天空位置 (以太阳为中心的等距鱼眼, 度) 的正上方, 高度 = 第二个取向角.
//
// 网格做法 (参数网格三角化 + 边界二分):
//   360×360 网格上对 F = min(全部判据) 做 marching squares, 格边上的边界点二分钉到 F = 0;
//   投影到鱼眼后边长过长的三角形红绿细分 (出射临界边附近映射被拉得很长); 细分后仍跨过反日点 (鱼眼奇点) 的丢掉.
// 高度 = 第二个取向角 v (柱晶 roll / Lowitz ζ) 相对本连通块参考角的偏移 (弧度):
//   'auto' — 本块在每个方位 u 上恰好有一处焦散 J = 0 时, 参考角取同一 u 的折痕 (折痕抬成水平线;
//            柱晶 3-5 就是甜甜圈影子第 ③ 幅的 ρ − ρ_f);
//   否则 / 'mean' — 本块的圆周平均, 跨度接近整圈时改用正弦;
//   'dist' — 不用取向角: 高度 = ±(取向平面上到本块最近折痕的距离), 紫面 (J < 0) 在上. 折痕永远水平地落在 0,
//            任何光路都像柱晶 3-5 那样上下两片 (Lowitz 的 ζ 沿弧一路从 0 爬到 180°, 取向角当高度时整片是斜坡).
// 天空方向 → 鱼眼: 右 = 面向太阳时的右手边 (与「天空」视图同向), 上 = 太阳的"上".

import { LP, K, SqK, chain3, newC3, jana3, axesMeta } from './lp.js';
import { AP, apMargin } from './aperture.js';

const TAU = 2 * Math.PI;
const OFF = 0.1371;                 // 网格错开 0.1371 格 (同 orientCompute), 避开对称位置上的退化
const R2D = 180 / Math.PI;

const wrapPi = x => x - TAU * Math.floor((x + Math.PI) / TAU);
const wrap2Pi = x => x - TAU * Math.floor(x / TAU);

/** 有效裕度: 全部判据里最小的那个 (≥ 0 即光路成立), 与 valid3 + apOk 同一组判据. */
function margin(P, m, fx, u, v, c) {
  chain3(u, v, m, fx, P, c);
  let F = c.g0;
  const qs = c.qs;
  for (let i = 0; i < qs.length; i++) {
    if (-qs[i] < F) F = -qs[i];
    if (!LP.partial && qs[i] + SqK < F) F = qs[i] + SqK;
  }
  const e = -SqK - c.qE;
  if (e < F) F = e;
  if (AP.on && P.apSupported) {
    const a = apMargin(P, m, fx, u, v);
    if (Number.isFinite(a) && a < F) F = a;
  }
  return F;
}

/** 天空方向 → 以太阳为中心的等距鱼眼 (度). out = [x, y] */
export function fisheyeOf(Dx, Dy, Dz, out) {
  const A = LP.A3, sh = Math.sin(LP.TH), ch = Math.cos(LP.TH);
  const n = Math.hypot(Dx, Dy, Dz) || 1;
  Dx /= n; Dy /= n; Dz /= n;
  const r = Math.acos(Math.max(-1, Math.min(1, Dx * A[0] + Dy * A[1] + Dz * A[2]))) * R2D;
  const px = -Dy, py = -Dx * sh + Dz * ch, pn = Math.hypot(px, py);
  if (pn < 1e-15) { out[0] = 0; out[1] = 0; return out; }
  out[0] = r * px / pn; out[1] = r * py / pn;
  return out;
}

/**
 * opts = { ref: 'auto' | 'mean' | 'dist', n: 网格边长 (默认 360), mcMs: 地面亮度蒙特卡洛的时间上限 }
 * lines: Map(族名 → { par: [rad 对], sky: [X,Y,Z,valid 四元组], conn: { par, sky } })
 * 返回 { n, uv (度), g (鱼眼度), hv (高度偏移, 弧度), jv (压缩后的雅可比, 定折痕两侧), tris, comps,
 *        lines: [{ name, kind, seg: Float32Array (gx,gy,hv)×2 }], eps: [{ gx, gy, hv } | null],
 *        hist: { x0, x1, y0, y1, nx, ny, data, samples }, collapsed, bbox }
 */
export function computeLift(P, m, fx, fams, lines, eps, opts = {}) {
  // 第二个角 v 默认首尾相接 (0–360°); Lowitz 的 ζ 只转半圈时是 0–180°, 不相接 (两头是截断边)
  const ax = axesMeta(m), perV = ax.perY, vMax = (ax.y1 - ax.y0) * Math.PI / 180;
  const nu = opts.n || 360, nv = perV ? nu : Math.round(nu / 2) + 1, du = TAU / nu;
  const dv = perV ? TAU / nv : (vMax - 2e-4) / (nv - 1);
  const U = i => (i + OFF) * du, V = j => (perV ? (j + OFF) * dv : 1e-4 + j * dv);
  const jEnd = perV ? nv : nv - 1;                // 有下一行的格点 (不相接时最后一行没有)
  const c = newC3(P);
  const at = (i, j) => ((i % nu) + nu) % nu * nv + ((j % nv) + nv) % nv;

  // ---------- 网格上的裕度 ----------
  const Fg = new Float64Array(nu * nv), ok = new Uint8Array(nu * nv);
  for (let i = 0; i < nu; i++)
    for (let j = 0; j < nv; j++) {
      const f = margin(P, m, fx, U(i), V(j), c);
      Fg[i * nv + j] = f;
      ok[i * nv + j] = f >= 0 ? 1 : 0;
    }

  // ---------- 顶点: 有效格点 + 格边上的边界点 ----------
  const vu = [], vv = [], vCorner = [];          // vCorner: 边界点所在格边上有效的那一端 (格点下标)
  const gridId = new Int32Array(nu * nv).fill(-1);
  for (let q = 0; q < nu * nv; q++)
    if (ok[q]) { gridId[q] = vu.length; vu.push(U(Math.floor(q / nv))); vv.push(V(q % nv)); vCorner.push(q); }
  const eU = new Int32Array(nu * nv).fill(-1);   // (i,j)–(i+1,j)
  const eV = new Int32Array(nu * nv).fill(-1);   // (i,j)–(i,j+1)
  const bis = (u0, v0, du0, dv0, loPos) => {
    let lo = 0, hi = 1;
    for (let k = 0; k < 34; k++) {
      const mid = 0.5 * (lo + hi);
      if ((margin(P, m, fx, u0 + mid * du0, v0 + mid * dv0, c) >= 0) === loPos) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
  };
  for (let i = 0; i < nu; i++)
    for (let j = 0; j < nv; j++) {
      const q = i * nv + j, a = ok[q];
      const qu = at(i + 1, j), qv = j < jEnd ? at(i, j + 1) : q;
      if (a !== ok[qu]) {
        const t = bis(U(i), V(j), du, 0, a === 1);
        eU[q] = vu.length; vu.push(wrap2Pi(U(i) + t * du)); vv.push(V(j)); vCorner.push(a ? q : qu);
      }
      if (a !== ok[qv]) {
        const t = bis(U(i), V(j), 0, dv, a === 1);
        eV[q] = vu.length; vu.push(U(i)); vv.push(wrap2Pi(V(j) + t * dv)); vCorner.push(a ? q : qv);
      }
    }

  // ---------- marching squares → 三角形 ----------
  const tris = [];
  for (let i = 0; i < nu; i++)
    for (let j = 0; j < jEnd; j++) {
      const k0 = at(i, j), k1 = at(i + 1, j), k2 = at(i + 1, j + 1), k3 = at(i, j + 1);
      const on = [ok[k0], ok[k1], ok[k2], ok[k3]];
      const cs = on[0] | (on[1] << 1) | (on[2] << 2) | (on[3] << 3);
      if (cs === 0) continue;
      const cid = [gridId[k0], gridId[k1], gridId[k2], gridId[k3]];
      const eid = [eU[k0], eV[k1], eU[k3], eV[k0]];      // 边 k 连角 k 与角 k+1
      const polys = [];
      if ((cs === 5 || cs === 10) && margin(P, m, fx, U(i) + du / 2, V(j) + dv / 2, c) < 0) {
        for (let k = 0; k < 4; k++) if (on[k]) polys.push([cid[k], eid[k], eid[(k + 3) % 4]]);
      } else {
        const t = [];
        for (let k = 0; k < 4; k++) {
          if (on[k]) t.push(cid[k]);
          if (on[k] !== on[(k + 1) % 4]) t.push(eid[k]);
        }
        if (t.length >= 3) polys.push(t);
      }
      for (const p of polys)
        for (let k = 1; k + 1 < p.length; k++)
          if (p[0] >= 0 && p[k] >= 0 && p[k + 1] >= 0) tris.push(p[0], p[k], p[k + 1]);
    }

  // ---------- 连通块 ----------
  const parent = new Int32Array(vu.length);
  for (let k = 0; k < parent.length; k++) parent[k] = k;
  const find = x => { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
  const unite = (a, b) => { a = find(a); b = find(b); if (a !== b) parent[a] = b; };
  for (let t = 0; t < tris.length; t += 3) { unite(tris[t], tris[t + 1]); unite(tris[t + 1], tris[t + 2]); }

  // ---------- 雅可比 (定折痕两侧) ----------
  const Jg = new Float64Array(nu * nv).fill(NaN);
  const absJ = [];
  for (let q = 0; q < nu * nv; q++)
    if (ok[q]) {
      const J = jana3(U(Math.floor(q / nv)), V(q % nv), m, fx, P);
      Jg[q] = J;
      if (Number.isFinite(J)) absJ.push(Math.abs(J));
    }
  absJ.sort((a, b) => a - b);
  const jScale = absJ.length ? absJ[absJ.length >> 1] : 0;
  const collapsed = !absJ.length || jScale < 1e-7;     // J ≡ 0: 整片压成一条线, 没有折痕两侧可分

  // ---------- 高度参考角 (每个连通块) ----------
  const nV0 = vu.length;
  const compOf = new Int32Array(nV0);
  for (let k = 0; k < nV0; k++) compOf[k] = find(k);
  const comps = new Map();                       // 根 → { cols: Map(i → [折痕 v]), vs: [] }
  const compInfo = r => { let o = comps.get(r); if (!o) { o = { cols: new Map(), seen: new Set(), sumC: 0, sumS: 0, n: 0 }; comps.set(r, o); } return o; };
  for (let k = 0; k < nV0; k++) {
    const o = compInfo(compOf[k]);
    o.sumC += Math.cos(vv[k]); o.sumS += Math.sin(vv[k]); o.n++;
  }
  for (let i = 0; i < nu; i++)
    for (let j = 0; j < nv; j++) {
      const q = i * nv + j;
      if (!ok[q]) continue;
      const o = compInfo(compOf[gridId[q]]);
      o.seen.add(i);
      if (j >= jEnd) continue;
      const q2 = at(i, j + 1);
      if (collapsed || !ok[q2] || compOf[gridId[q2]] !== compOf[gridId[q]]) continue;
      const J0 = Jg[q], J1 = Jg[q2];
      if (!(Number.isFinite(J0) && Number.isFinite(J1)) || (J0 >= 0) === (J1 >= 0)) continue;
      let lo = 0, hi = 1;
      for (let k = 0; k < 26; k++) {
        const mid = 0.5 * (lo + hi), Jm = jana3(U(i), V(j) + mid * dv, m, fx, P);
        if (Number.isFinite(Jm) && (Jm >= 0) === (J0 >= 0)) lo = mid; else hi = mid;
      }
      const list = o.cols.get(i);
      const fv = wrap2Pi(V(j) + 0.5 * (lo + hi) * dv);
      if (list) list.push(fv); else o.cols.set(i, [fv]);
    }
  const refMode = opts.ref === 'mean' || opts.ref === 'dist' ? opts.ref : 'auto';
  const wantFold = refMode === 'auto';
  for (const o of comps.values()) {
    o.ref = Math.atan2(o.sumS, o.sumC);
    o.fold = null;
    if (wantFold && !collapsed && o.seen.size) {
      const one = [], multi = [];
      for (const [i, l] of o.cols) (l.length === 1 ? one : multi).push(i);
      if (one.length >= Math.max(3, 0.6 * o.seen.size) && multi.length <= 0.05 * o.seen.size)
        o.fold = one.map(i => [U(i), o.cols.get(i)[0]]);           // 先用格列上的折痕, 下面换成引擎的精确焦散线
    }
    o.sinMode = false;
  }
  // 某个取向点属于哪个连通块 (线、末端点用)
  const compAt = (u, v) => {
    const x = u / du - OFF, y = perV ? v / dv - OFF : (v - 1e-4) / dv, i0 = Math.floor(x), j0 = Math.floor(y);
    let best = -1, bd = Infinity;
    for (let r = 1; r <= 3 && best < 0; r++)
      for (let di = 1 - r; di <= r; di++)
        for (let dj = 1 - r; dj <= r; dj++) {
          if (!perV && (j0 + dj < 0 || j0 + dj >= nv)) continue;
          const q = at(i0 + di, j0 + dj);
          if (!ok[q]) continue;
          const d = (i0 + di - x) ** 2 + (j0 + dj - y) ** 2;
          if (d < bd) { bd = d; best = compOf[gridId[q]]; }
        }
    return best;
  };
  // 引擎追踪的 J = 0 焦散线上能当折痕的点, 逐条线按序回调 fn(u, v, 块, 是否与上一个回调点断开)
  const eachCrease = fn => {
    for (const fm of fams) {
      if (fm.kind !== 6) continue;
      const fl = lines.get(fm.name);
      if (!fl) continue;
      const pars = [...fl.par, ...(fl.conn ? fl.conn.par : [])], skys = [...fl.sky, ...(fl.conn ? fl.conn.sky : [])];
      for (let li = 0; li < pars.length; li++) {
        // Ĵ 在区域边界上也会是 0 (含 √T_E 和反射面的 q 因子: 出射临界、接缝 q = 0): 那种"焦散"一路贴着边界走,
        // 不是折痕 (柱晶 3-5-6 的接缝和真折痕混在一起, 参考角在两者之间来回跳, 曲面成了一把扇子).
        // 网格追出的线里连着贴边界 (裕度 < 1e-5) 的点去掉. 补线 (fl.conn) 是真焦散相切地伸到尖端末端点, 裕度按距离平方
        // 降到 0, 全部留着; 网格线本来就在尖端前 1–2° 停下, 不会这样贴边界
        const pl = pars[li], n = pl.length / 2, edge = new Uint8Array(n);
        if (li < fl.par.length)
          for (let i = 0; i < n; i++) edge[i] = margin(P, m, fx, wrap2Pi(pl[2 * i]), wrap2Pi(pl[2 * i + 1]), c) < 1e-5 ? 1 : 0;
        let brk = true;
        for (let i = 0; i < n; i++) {
          if (!(skys[li][4 * i + 3] > 0.5) || (edge[i] && (i === 0 || edge[i - 1]) && (i === n - 1 || edge[i + 1]))) { brk = true; continue; }
          const u = wrap2Pi(pl[2 * i]), v = wrap2Pi(pl[2 * i + 1]), r = compAt(u, v);
          if (r < 0) { brk = true; continue; }
          fn(u, v, r, brk); brk = false;
        }
      }
    }
  };
  // 折痕参考角改用引擎追踪的 J = 0 焦散线 (根点二分到机器精度, 补线一直连到尖端的末端点);
  // 只靠格列插值时, 透镜尖端附近折痕随方位变化极快, 夹取最近一列会差到 0.1 rad
  if (!collapsed && wantFold) {
    const pts = new Map();
    eachCrease((u, v, r) => {
      if (!comps.get(r).fold) return;
      if (!pts.has(r)) pts.set(r, []);
      pts.get(r).push([u, v]);
    });
    for (const [r, list] of pts) if (list.length >= 10) comps.get(r).fold = list;
  }
  for (const o of comps.values())
    if (o.fold) {
      o.fold.sort((a, b) => a[0] - b[0]);
      o.fold = { u: Float64Array.from(o.fold, p => p[0]), v: Float64Array.from(o.fold, p => p[1]) };
    }
  const foldRef = (o, u0) => {
    const Fu = o.fold.u, Fv = o.fold.v, n = Fu.length, u = wrap2Pi(u0);
    if (n === 1) return Fv[0];
    let lo = 0, hi = n;                                   // 第一个 ≥ u 的下标
    while (lo < hi) { const mid = (lo + hi) >> 1; if (Fu[mid] < u) lo = mid + 1; else hi = mid; }
    const b = lo === n ? 0 : lo, a = lo === 0 ? n - 1 : lo - 1;
    const gap = wrap2Pi(Fu[b] - Fu[a]) || (a === b ? TAU : 0), da = wrap2Pi(u - Fu[a]), db = wrap2Pi(Fu[b] - u);
    if (gap > Math.PI) return da <= db ? Fv[a] : Fv[b];  // 块外: 取最近的折痕
    const t = gap > 0 ? da / gap : 0;
    return Fv[a] + t * wrapPi(Fv[b] - Fv[a]);
  };
  // 「距折痕」: 某点到本块折痕的距离 (周围 4×4 格: 登记的折痕线段取精确距离, 格点记住的最近点作远处的候选),
  // 以及该点 J 的号 (边界上 J → 0, 往有效一侧挪一点再算)
  const distAt = (o, u, v) => {
    const D = o.dist, x = wrap2Pi(u), y = perV ? wrap2Pi(v) : v, S = D.segs;
    const i0 = Math.floor(x / du - OFF), j0 = perV ? Math.floor(y / dv - OFF) : Math.floor((y - 1e-4) / dv);
    let b = Infinity;
    for (let i = i0 - 1; i <= i0 + 2; i++)
      for (let j = j0 - 1; j <= j0 + 2; j++) {
        if (!perV && (j < 0 || j >= nv)) continue;
        const q = at(i, j);
        if (D.ok[q] === Infinity) continue;
        const a = wrapPi(x - D.su[q]), e = perV ? wrapPi(y - D.sv[q]) : y - D.sv[q], dd = a * a + e * e;
        if (dd < b) b = dd;
        const cl = D.cells.get(q);
        if (cl)
          for (const s of cl) {
            const px = wrapPi(x - S[4 * s]), py = perV ? wrapPi(y - S[4 * s + 1]) : y - S[4 * s + 1];
            const sx = S[4 * s + 2], sy = S[4 * s + 3], l2 = sx * sx + sy * sy;
            const f = l2 > 0 ? Math.max(0, Math.min(1, (px * sx + py * sy) / l2)) : 0;
            const ex = px - f * sx, ey = py - f * sy, d = ex * ex + ey * ey;
            if (d < b) b = d;
          }
      }
    return Math.sqrt(b);
  };
  const signAt = (u, v) => {
    const tol = 1e-9 * (jScale || 1);
    let J = jana3(u, v, m, fx, P);
    if (Number.isFinite(J) && Math.abs(J) > tol) return J < 0 ? -1 : 1;
    for (const e of [2e-3, 6e-3, 2e-2])
      for (let k = 0; k < 8; k++) {
        const u2 = u + e * Math.cos(k * Math.PI / 4), v2 = v + e * Math.sin(k * Math.PI / 4);
        if (margin(P, m, fx, u2, v2, c) < 0) continue;
        J = jana3(u2, v2, m, fx, P);
        if (Number.isFinite(J) && Math.abs(J) > tol) return J < 0 ? -1 : 1;
      }
    return 1;
  };
  const hvOf = (r, u, v) => {
    const o = comps.get(r);
    if (o.dist) return -signAt(u, v) * distAt(o, u, v);
    if (o.fold) return wrapPi(v - foldRef(o, u));
    return o.sinMode ? Math.sin(v - o.ref) : wrapPi(v - o.ref);
  };
  // 折痕模式下离折痕太远 (快绕满一圈) 的块退回平均模式; 平均模式里跨度接近整圈的块改用正弦
  for (let k = 0; k < nV0; k++) {
    const o = comps.get(compOf[k]);
    if (o.fold && Math.abs(wrapPi(vv[k] - foldRef(o, vu[k]))) > 0.92 * Math.PI) o.fold = null;
  }
  for (let k = 0; k < nV0; k++) {
    const o = comps.get(compOf[k]);
    if (!o.fold && Math.abs(wrapPi(vv[k] - o.ref)) > 0.92 * Math.PI) o.sinMode = true;
  }

  // ---------- 顶点属性 ----------
  const g = [], hv = [], jv = [], cmp = [];
  const fo = [0, 0];
  const jNorm = J => (collapsed ? 1 : J / (Math.abs(J) + jScale));
  const addAttrs = (k, u, v, r, J) => {
    chain3(u, v, m, fx, P, c);
    fisheyeOf(c.Dx, c.Dy, c.Dz, fo);
    g[2 * k] = fo[0]; g[2 * k + 1] = fo[1];
    hv[k] = hvOf(r, u, v);
    jv[k] = J;
    cmp[k] = r;
  };
  for (let k = 0; k < nV0; k++) {
    let J = Jg[vCorner[k]];
    if (k >= 0 && gridId[vCorner[k]] !== k) {               // 边界点: 在点上算, 出射临界边上 J 无定义就往有效端挪一点
      const q = vCorner[k], cu = U(Math.floor(q / nv)), cvv = V(q % nv);
      const Jh = jana3(vu[k], vv[k], m, fx, P);
      const Jn = jana3(vu[k] + 0.05 * wrapPi(cu - vu[k]), vv[k] + 0.05 * wrapPi(cvv - vv[k]), m, fx, P);
      J = Number.isFinite(Jh) ? Jh : Number.isFinite(Jn) ? Jn : J;
    }
    addAttrs(k, vu[k], vv[k], compOf[k], Number.isFinite(J) ? jNorm(J) : 0);
  }

  // ---------- 细分: 鱼眼上边长过长的三角形一分为四, 只有一条边被分的邻格一分为二 ----------
  let x0 = -25, x1 = 25, y0 = -25, y1 = 25;
  for (let k = 0; k < nV0; k++) {
    if (g[2 * k] < x0) x0 = g[2 * k]; if (g[2 * k] > x1) x1 = g[2 * k];
    if (g[2 * k + 1] < y0) y0 = g[2 * k + 1]; if (g[2 * k + 1] > y1) y1 = g[2 * k + 1];
  }
  const maxLen = Math.max(0.9, Math.max(x1 - x0, y1 - y0) / 110), cullLen = 40;
  let T = Uint32Array.from(tris);
  const elen = (a, b) => Math.hypot(g[2 * a] - g[2 * b], g[2 * a + 1] - g[2 * b + 1]);
  for (let depth = 0; depth < 5 && T.length; depth++) {
    const nt = T.length / 3, mark = new Uint8Array(nt);
    let any = false;
    for (let t = 0; t < nt; t++) {
      const a = T[3 * t], b = T[3 * t + 1], d = T[3 * t + 2];
      if (Math.max(elen(a, b), elen(b, d), elen(d, a)) > maxLen) { mark[t] = 1; any = true; }
    }
    if (!any) break;
    const nv1 = vu.length, key = (a, b) => (a < b ? a * nv1 + b : b * nv1 + a);
    const edgeIdx = new Map(), eids = new Int32Array(3 * nt);
    for (let t = 0; t < nt; t++)
      for (let s = 0; s < 3; s++) {
        const kk = key(T[3 * t + s], T[3 * t + (s + 1) % 3]);
        let id = edgeIdx.get(kk);
        if (id === undefined) { id = edgeIdx.size; edgeIdx.set(kk, id); }
        eids[3 * t + s] = id;
      }
    const split = new Uint8Array(edgeIdx.size);
    for (let t = 0; t < nt; t++) if (mark[t]) { split[eids[3 * t]] = split[eids[3 * t + 1]] = split[eids[3 * t + 2]] = 1; }
    for (let grow = true; grow;) {                         // 两条边要分的三角形也整个一分为四
      grow = false;
      for (let t = 0; t < nt; t++)
        if (!mark[t] && split[eids[3 * t]] + split[eids[3 * t + 1]] + split[eids[3 * t + 2]] >= 2) {
          mark[t] = 1; grow = true;
          split[eids[3 * t]] = split[eids[3 * t + 1]] = split[eids[3 * t + 2]] = 1;
        }
    }
    const midId = new Int32Array(edgeIdx.size).fill(-1);
    for (const [kk, id] of edgeIdx) {
      if (!split[id]) continue;
      const a = Math.floor(kk / nv1), b = kk % nv1, k = vu.length;
      const u = wrap2Pi(vu[a] + 0.5 * wrapPi(vu[b] - vu[a])), v = wrap2Pi(vv[a] + 0.5 * wrapPi(vv[b] - vv[a]));
      vu.push(u); vv.push(v);
      // 两端同侧的中点直接继承; 只有跨折痕的边才重算雅可比 (出射临界边附近差分会乱跳)
      let J = 0.5 * (jv[a] + jv[b]);
      if ((jv[a] >= 0) !== (jv[b] >= 0)) { const Jm = jana3(u, v, m, fx, P); if (Number.isFinite(Jm)) J = jNorm(Jm); }
      addAttrs(k, u, v, cmp[a], J);
      midId[id] = k;
    }
    const out = [];
    for (let t = 0; t < nt; t++) {
      const v0 = T[3 * t], v1 = T[3 * t + 1], v2 = T[3 * t + 2];
      const m01 = midId[eids[3 * t]], m12 = midId[eids[3 * t + 1]], m20 = midId[eids[3 * t + 2]];
      if (mark[t]) out.push(v0, m01, m20, m01, v1, m12, m20, m12, v2, m01, m12, m20);
      else if (m01 >= 0) out.push(v0, m01, v2, m01, v1, v2);
      else if (m12 >= 0) out.push(v1, m12, v0, m12, v2, v0);
      else if (m20 >= 0) out.push(v2, m20, v1, m20, v0, v1);
      else out.push(v0, v1, v2);
    }
    T = Uint32Array.from(out);
  }
  {   // 细分到底仍跨过反日点的三角形丢掉
    const keep = [];
    for (let t = 0; t < T.length; t += 3) {
      const a = T[t], b = T[t + 1], d = T[t + 2];
      if (Math.max(elen(a, b), elen(b, d), elen(d, a)) <= cullLen) keep.push(a, b, d);
    }
    T = Uint32Array.from(keep);
  }

  // ---------- 「距折痕」高度: ±(取向平面上到最近折痕的距离), 紫面 (J < 0) 在上 ----------
  // 折痕 = 引擎的 J = 0 线 (精确; 过滤同上) + 最终网格里 J 变号的三角形内的零线段 (变号处距离必为 0, 曲面不会撕开).
  // 距离: 每个格点记住离它最近的折痕点, 正反两遍扫描向邻居传 (周期轴再来一轮), 远处靠它;
  // 近处 (一两格内) 按格子登记的折痕线段算点到线段的精确距离 — 每格只记一个点时, 折痕线上的点会差半格 (≈ 0.009 rad)
  if (refMode === 'dist' && !collapsed) {
    const seeds = new Map(), segOf = new Map(), h0 = 0.25 * Math.min(du, dv);
    const cellI = x => Math.floor(x / du - OFF), cellJ = y => (perV ? Math.floor(y / dv - OFF) : Math.floor((y - 1e-4) / dv));
    const addSeg = (r, ua, va, ub, vb) => {
      const dU = wrapPi(ub - ua), dV = perV ? wrapPi(vb - va) : vb - va, len = Math.hypot(dU, dV);
      if (len > 0.05) return;                                    // 跨缝 / 断开的大跳不连
      let l = seeds.get(r), sg = segOf.get(r);
      if (!l) { seeds.set(r, (l = [])); segOf.set(r, (sg = { segs: [], cells: new Map() })); }
      for (let s = 0, k = Math.max(1, Math.ceil(len / h0)); s <= k; s++) l.push(ua + dU * s / k, va + dV * s / k);
      const id = sg.segs.length / 4;
      sg.segs.push(ua, va, dU, dV);
      const i0 = cellI(Math.min(ua, ua + dU)), i1 = cellI(Math.max(ua, ua + dU));
      const j0 = cellJ(Math.min(va, va + dV)), j1 = cellJ(Math.max(va, va + dV));
      for (let i = i0; i <= i1; i++)
        for (let j = j0; j <= j1; j++) {
          if (!perV && (j < 0 || j >= nv)) continue;
          const q = at(i, j), cl = sg.cells.get(q);
          if (cl) { if (cl[cl.length - 1] !== id) cl.push(id); } else sg.cells.set(q, [id]);
        }
    };
    let pu = 0, pv = 0, pr = -1;
    eachCrease((u, v, r, brk) => {
      if (brk || r !== pr) addSeg(r, u, v, u, v); else addSeg(r, pu, pv, u, v);
      pu = u; pv = v; pr = r;
    });
    const xs = [];
    for (let t = 0; t < T.length; t += 3) {
      xs.length = 0;
      for (let s = 0; s < 3; s++) {
        const a = T[t + s], b = T[t + (s + 1) % 3];
        if ((jv[a] < 0) === (jv[b] < 0)) continue;
        const f = jv[a] / (jv[a] - jv[b]);
        xs.push(vu[a] + f * wrapPi(vu[b] - vu[a]), vv[a] + f * (perV ? wrapPi(vv[b] - vv[a]) : vv[b] - vv[a]));
      }
      if (xs.length === 4) addSeg(cmp[T[t]], xs[0], xs[1], xs[2], xs[3]);
    }
    const NN = nu * nv;
    const d2 = (q, x, y) => {
      const a = wrapPi(x - U(Math.floor(q / nv))), b = perV ? wrapPi(y - V(q % nv)) : y - V(q % nv);
      return a * a + b * b;
    };
    for (const [r, S] of seeds) {
      const best = new Float64Array(NN).fill(Infinity), su = new Float64Array(NN), sv = new Float64Array(NN);
      for (let s = 0; s < S.length; s += 2) {
        const x = wrap2Pi(S[s]), y = perV ? wrap2Pi(S[s + 1]) : S[s + 1];
        const i = Math.round(x / du - OFF), j = perV ? Math.round(y / dv - OFF) : Math.max(0, Math.min(nv - 1, Math.round((y - 1e-4) / dv)));
        const q = at(i, j), dd = d2(q, x, y);
        if (dd < best[q]) { best[q] = dd; su[q] = x; sv[q] = y; }
      }
      const pull = (q, i2, j2) => {
        if (!perV && (j2 < 0 || j2 >= nv)) return;
        const q2 = at(i2, j2);
        if (best[q2] === Infinity) return;
        const dd = d2(q, su[q2], sv[q2]);
        if (dd < best[q]) { best[q] = dd; su[q] = su[q2]; sv[q] = sv[q2]; }
      };
      for (let rep = 0; rep < 2; rep++) {
        for (let i = 0; i < nu; i++)
          for (let j = 0; j < nv; j++) { const q = i * nv + j; pull(q, i - 1, j - 1); pull(q, i - 1, j); pull(q, i - 1, j + 1); pull(q, i, j - 1); }
        for (let i = nu - 1; i >= 0; i--)
          for (let j = nv - 1; j >= 0; j--) { const q = i * nv + j; pull(q, i + 1, j + 1); pull(q, i + 1, j); pull(q, i + 1, j - 1); pull(q, i, j + 1); }
      }
      comps.get(r).dist = { su, sv, ok: best, ...segOf.get(r) };
    }
    for (let k = 0; k < vu.length; k++) if (comps.get(cmp[k]).dist) hv[k] = (jv[k] < 0 ? 1 : -1) * distAt(comps.get(cmp[k]), vu[k], vv[k]);
  }

  // ---------- 折痕参考靠不住的块退回平均模式 ----------
  // 折痕不是方位 u 的单值函数时 (柱晶 3-1-5 太阳 28.5°: 折痕斜着进来, 又顺着 roll 往下拐), 「同一 u 上的折痕」
  // 在几条分支之间跳, 曲面碎成锯齿. 直接查最终网格边上的高度跳变: 正常的块 (连透镜尖端) 相邻顶点差 < 0.05 rad,
  // 碎掉的块有几百到上万条 0.15–1 rad 的跳变 — 超过 20 条就让这一块退回平均模式, 重算它的高度
  const jumpN = new Map();
  for (let t = 0; t < T.length; t += 3)
    for (let s = 0; s < 3; s++) {
      const a = T[t + s], b = T[t + (s + 1) % 3], r = cmp[a];
      if ((comps.get(r).fold || comps.get(r).dist) && Math.abs(hv[a] - hv[b]) > 0.15) jumpN.set(r, (jumpN.get(r) || 0) + 1);
    }
  const redo = new Set([...jumpN].filter(([, n]) => n > 20).map(([r]) => r));
  if (redo.size) {
    for (const r of redo) { comps.get(r).fold = null; comps.get(r).dist = null; }
    for (let k = 0; k < vu.length; k++) {
      const o = comps.get(cmp[k]);
      if (redo.has(cmp[k]) && Math.abs(wrapPi(vv[k] - o.ref)) > 0.92 * Math.PI) o.sinMode = true;
    }
    for (let k = 0; k < vu.length; k++) if (redo.has(cmp[k])) hv[k] = hvOf(cmp[k], vu[k], vv[k]);
  }

  // ---------- 族线: 只留成立的段, 抬到同一高度规则下 ----------
  const outLines = [];
  for (const fm of fams) {
    const fl = lines.get(fm.name);
    if (!fl) continue;
    const pars = [...fl.par, ...(fl.conn ? fl.conn.par : [])], skys = [...fl.sky, ...(fl.conn ? fl.conn.sky : [])];
    const seg = [];
    for (let li = 0; li < pars.length; li++) {
      const par = pars[li], sky = skys[li], n = par.length / 2;
      let prev = null;
      for (let i = 0; i < n; i++) {
        const u = par[2 * i], v = par[2 * i + 1];
        if (!(sky[4 * i + 3] > 0.5)) { prev = null; continue; }
        const r = compAt(u, v);
        if (r < 0) { prev = null; continue; }
        fisheyeOf(sky[4 * i], sky[4 * i + 1], sky[4 * i + 2], fo);
        const cur = [fo[0], fo[1], hvOf(r, u, v), r];
        if (prev && prev[3] === r && Math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < cullLen && Math.abs(cur[2] - prev[2]) < 1)
          seg.push(prev[0], prev[1], prev[2], cur[0], cur[1], cur[2]);
        prev = cur;
      }
    }
    outLines.push({ name: fm.name, kind: fm.kind, seg: Float32Array.from(seg) });
  }

  // ---------- 末端点 ----------
  const outEps = (eps || []).map(r => {
    const u = r.sc / R2D, v = r.sw / R2D, cr = compAt(u, v);
    if (cr < 0 || !r.D) return null;
    fisheyeOf(r.D[0], r.D[1], r.D[2], fo);
    return { gx: fo[0], gy: fo[1], hv: hvOf(cr, u, v) };
  });

  // ---------- 地面亮度: 均匀随机姿态的落点直方图 ----------
  const hist = groundHist(P, m, fx, x0, x1, y0, y1, (rnd, uv) => { uv[0] = rnd() * TAU; uv[1] = rnd() * (perV ? TAU : vMax); }, opts.mcMs || 700);

  return {
    n: vu.length,
    uv: Float32Array.from({ length: 2 * vu.length }, (_, q) => (q & 1 ? vv[q >> 1] : vu[q >> 1]) * R2D),
    g: Float32Array.from(g), hv: Float32Array.from(hv), jv: Float32Array.from(jv), tris: T,
    lines: outLines, eps: outEps, collapsed,
    folds: [...comps.values()].filter(o => o.fold).length, dists: [...comps.values()].filter(o => o.dist).length, comps: comps.size,
    bbox: { x0, x1, y0, y1 },
    hist,
  };
}

/** 地面亮度: 按 draw 抽随机姿态, 光路成立的按天空落点累加 (鱼眼度). 范围 = 曲面范围外扩 6°, 取整到 5°. */
function groundHist(P, m, fx, x0, x1, y0, y1, draw, budget) {
  const c = newC3(P), fo = [0, 0], uv = [0, 0];
  const pad = 6, r5 = (v, up) => (up ? Math.ceil : Math.floor)(v / 5) * 5;
  const hx0 = Math.max(-185, r5(x0 - pad, false)), hx1 = Math.min(185, r5(x1 + pad, true));
  const hy0 = Math.max(-185, r5(y0 - pad, false)), hy1 = Math.min(185, r5(y1 + pad, true));
  const bin = Math.max(0.3, Math.max(hx1 - hx0, hy1 - hy0) / 700);
  const nx = Math.max(1, Math.round((hx1 - hx0) / bin)), ny = Math.max(1, Math.round((hy1 - hy0) / bin));
  const data = new Float32Array(nx * ny);
  let seed = 0x9e3779b9 >>> 0;
  const rnd = () => { seed ^= seed << 13; seed >>>= 0; seed ^= seed >>> 17; seed ^= seed << 5; seed >>>= 0; return seed / 4294967296; };
  const tStart = performance.now(), sx = nx / (hx1 - hx0), sy = ny / (hy1 - hy0);
  let samples = 0;
  while (samples < 4e6) {
    for (let k = 0; k < 20000; k++) {
      draw(rnd, uv);
      if (margin(P, m, fx, uv[0], uv[1], c) < 0) continue;
      fisheyeOf(c.Dx, c.Dy, c.Dz, fo);
      const bx = Math.floor((fo[0] - hx0) * sx), by = Math.floor((fo[1] - hy0) * sy);
      if (bx >= 0 && bx < nx && by >= 0 && by < ny) data[by * nx + bx] += 1;
    }
    samples += 20000;
    if (performance.now() - tStart > budget) break;
  }
  return { x0: hx0, x1: hx1, y0: hy0, y1: hy1, nx, ny, data, samples };
}

/**
 * 1DOF 取向 (板晶: c 轴竖直, 只有方位 α; Parry: c 轴水平、roll 固定, 只有方位 α) 只有一个随机角,
 * 光路成立的姿态在取向空间里是几段弧, 抬起来是空间曲线 (不是曲面): 水平位置 = 光落点, 高度 = α 相对本段平均的偏移.
 * 返回结构同 computeLift (没有网格, n = 0), 曲线放在 lines 里 (kind = −1).
 */
export function computeLift1(P, m, fx, fixedV) {
  const N = 7200, du = TAU / N, c = newC3(P), fo = [0, 0];
  const ok = new Uint8Array(N), F = u => margin(P, m, fx, u, fixedV, c);
  for (let i = 0; i < N; i++) ok[i] = F(i * du) >= 0 ? 1 : 0;
  // 成立的段 (首尾相接的那段并起来), 端点二分钉到 F = 0
  const edge = (i, rising) => {      // i 与 i+1 之间
    let lo = i * du, hi = lo + du;
    for (let k = 0; k < 40; k++) { const mid = 0.5 * (lo + hi); if ((F(mid) >= 0) === rising) hi = mid; else lo = mid; }
    return 0.5 * (lo + hi);
  };
  const runs = [];
  if (ok.every(v => v)) runs.push(Array.from({ length: N + 1 }, (_, i) => i * du));
  else {
    const s0 = ok.indexOf(0);                         // 从一个不成立的格点出发绕一圈, u 连续递增
    let cur = null;
    for (let k = 1; k <= N; k++) {
      const i = (s0 + k) % N, uc = (s0 + k) * du, ip = (i - 1 + N) % N;
      if (ok[i]) {
        if (!cur) cur = [uc - du + (edge(ip, true) - ip * du)];
        cur.push(uc);
      } else if (cur) {
        cur.push(uc - du + (edge(ip, false) - ip * du));
        runs.push(cur);
        cur = null;
      }
    }
  }
  let x0 = -25, x1 = 25, y0 = -25, y1 = 25;
  const seg = [];
  const gOf = u => { chain3(u, fixedV, m, fx, P, c); fisheyeOf(c.Dx, c.Dy, c.Dz, fo); return [fo[0], fo[1]]; };
  for (let run of runs) {
    let sc = 0, ss = 0;
    for (const u of run) { sc += Math.cos(u); ss += Math.sin(u); }
    const ref = Math.atan2(ss, sc), sinMode = run[run.length - 1] - run[0] > 0.92 * TAU / 2;
    // 出射临界端附近天上走得极快: 天上相邻两点超过 0.3° 就在中间插点 (最多 8 轮)
    let gs = run.map(gOf);
    for (let pass = 0; pass < 8; pass++) {
      const u2 = [run[0]], g2 = [gs[0]];
      let added = false;
      for (let i = 1; i < run.length; i++) {
        const d = Math.hypot(gs[i][0] - gs[i - 1][0], gs[i][1] - gs[i - 1][1]);
        if (d > 0.3 && d < 40) { const um = 0.5 * (run[i - 1] + run[i]); u2.push(um); g2.push(gOf(um)); added = true; }
        u2.push(run[i]); g2.push(gs[i]);
      }
      run = u2; gs = g2;
      if (!added) break;
    }
    let prev = null;
    for (let i = 0; i < run.length; i++) {
      const u = run[i];
      const cur = [gs[i][0], gs[i][1], sinMode ? Math.sin(u - ref) : wrapPi(u - ref)];
      x0 = Math.min(x0, cur[0]); x1 = Math.max(x1, cur[0]); y0 = Math.min(y0, cur[1]); y1 = Math.max(y1, cur[1]);
      if (prev && Math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < 40) seg.push(...prev, ...cur);
      prev = cur;
    }
  }
  const hist = groundHist(P, m, fx, x0, x1, y0, y1, (rnd, uv) => { uv[0] = rnd() * TAU; uv[1] = fixedV; }, 300);
  return { n: 0, curve: true, uv: new Float32Array(0), g: new Float32Array(0), hv: new Float32Array(0), jv: new Float32Array(0),
           tris: new Uint32Array(0), lines: [{ name: '弧', kind: -1, seg: Float32Array.from(seg) }], eps: [], collapsed: true,
           folds: 0, comps: runs.length, bbox: { x0, x1, y0, y1 }, hist };
}
