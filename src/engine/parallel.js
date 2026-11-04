// 末端点整理: 平行光路的假焦散端、面积为 0 的孤立交点, 以及「汇聚点」(一整段焦散在天上只落成一点).
// 闭式末端点引擎 (ep.js) 不改, 这里只在它的结果上加减.
//
// (1) 假焦散端: ep.js 用 Ĵ = T_E·J 找焦散; 平行光路 (lp.js isParallelPath) 的 T_E = |g0|, 入射掠射边上每一点都是 Ĵ 的根,
//     于是每个「入射掠射 ∩ 某条边」都会多出一个「焦散 ∩ 某条边」. 真焦散上 J 只剩舍入 (~1e-15), 假的 |J| ~ 0.1–1, 按 J 的量级判.
// (2) 孤立交点: 几条边恰好交于一点, 周围却没有成立的姿态 (柱晶 3-4-6-4: 入射掠射 ∩ 接缝 ∩ 全反射临界三线共点, 区域面积为 0).
//     平行光路的入射掠射与出射临界重合, 这种三线共点很常见. 交点四周取一圈小圆, 一个成立的姿态都没有就去掉.
// (3) 汇聚点: 焦散 J = 0 上有一整段姿态把光送到同一个方向 — 自由度在这里坍缩, 一条线压成天上一点 (例: 柱晶 3-1-5-7 的反日点,
//     Lowitz 3-1-6 的太阳点). 这段「焦散」在天上没有长度, 它和边界的交点也都落在这一点上; 换成一个汇聚点 ◎, 原来那些交点去掉.

import { jana3, validAt, skyD } from './lp.js';

const D2R = Math.PI / 180;
const CAUSTIC = '焦散';
const unit = d => { const n = Math.hypot(d[0], d[1], d[2]) || 1; return [d[0] / n, d[1] / n, d[2] / n]; };
const ang = (a, b) => Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2])));
const azEl = D => ({ az: Math.atan2(D[1], D[0]) / D2R, el: Math.asin(Math.max(-1, Math.min(1, D[2]))) / D2R });

/** 去掉标签里满足 drop(part) 的部分; 什么都不剩返回 null */
function stripLabel(label, drop) {
  const parts = label.split(' | ').filter(p => !drop(p));
  return parts.length ? parts.join(' | ') : null;
}

/** 成立区域里 |J| 的典型量级 (MC 成立姿态上取中位数) */
function jScale(P, ori, om, fx) {
  const v = [];
  const n = ori.mcX.length, step = Math.max(1, Math.floor(n / 200));
  for (let i = 0; i < n; i += step) {
    const j = Math.abs(jana3(ori.mcX[i] * D2R, ori.mcY[i] * D2R, om, fx, P));
    if (Number.isFinite(j)) v.push(j);
  }
  if (!v.length) return 1;
  v.sort((a, b) => a - b);
  return v[v.length >> 1] || 1;
}

/** 交点四周有没有成立的姿态 (三圈小圆, 每圈 24 个方向) */
function hasValidNeighbour(P, om, fx, r) {
  const u = r.sc * D2R, v = r.sw * D2R;
  for (const rad of [0.02, 0.1, 0.5])
    for (let k = 0; k < 24; k++) {
      const a = 2 * Math.PI * (k + 0.5) / 24;
      if (validAt(P, om, fx, u + rad * D2R * Math.cos(a), v + rad * D2R * Math.sin(a), 0)) return true;
    }
  return false;
}

/**
 * 汇聚点: 焦散根点里, 成立且天空方向完全相同 (只差舍入, < 1e-10 rad) 的一大串, 在取向平面上铺开 ≥ 0.5°.
 * 门槛要这么严: 焦散在天上打尖的地方, 像沿焦散走得极慢 (∝ 弧长² 或 ³), 宽一点就会把尖点附近误认成汇聚点.
 * 返回 [{ D, sc, sw, n }], sc/sw 取这串根点中间的一个 (单击定位用).
 */
export function focalPoints(P, ori, om, fx) {
  const jf = ori.fams.find(f => f.kind === 6);
  const R = jf && ori.roots.get(jf.name);
  if (!R || !R.sc.length) return [];
  const buckets = new Map();
  for (let i = 0; i < R.sc.length; i++) {
    if (!R.ok[i]) continue;
    const D = unit(skyD(P, om, fx, R.sc[i] * D2R, R.sw[i] * D2R));
    const key = D.map(x => Math.round(x * 1e7)).join(',');
    let b = buckets.get(key);
    if (!b) buckets.set(key, b = { D, idx: [] });
    b.idx.push(i);
  }
  const wrap = d => d - 360 * Math.round(d / 360);
  const out = [];
  for (const b of buckets.values()) {
    if (b.idx.length < 6) continue;
    const i0 = b.idx[0];
    let spread = 0, dev = 0;
    for (const i of b.idx) {
      spread = Math.max(spread, Math.hypot(wrap(R.sc[i] - R.sc[i0]), wrap(R.sw[i] - R.sw[i0])));
      dev = Math.max(dev, ang(b.D, unit(skyD(P, om, fx, R.sc[i] * D2R, R.sw[i] * D2R))));
    }
    if (spread < 0.5 || dev > 1e-10) continue;
    const near = out.find(f => ang(f.D, b.D) < 2e-4);
    if (near) { near.n += b.idx.length; continue; }
    const im = b.idx[b.idx.length >> 1];
    out.push({ D: b.D, sc: R.sc[im], sw: R.sw[im], n: b.idx.length });
  }
  return out;
}

/** 末端点整理 (见文件头). eps 为 ep.js + 孔径 + ζ 半圈的结果; 返回新数组. */
export function tidyEndpoints(eps, pr, om, fx) {
  const P = pr.P, ori = pr.ori;
  if (!ori) return eps;
  const empty = ori.mcX.length === 0 && !ori.valid.some(x => x);
  let out = eps;
  // (1) 平行光路的假焦散端
  if (P.zw) {
    const tol = 1e-6 * jScale(P, ori, om, fx);
    out = out.flatMap(r => {
      if (!r.label.includes(CAUSTIC)) return [r];
      const j = Math.abs(jana3(r.sc * D2R, r.sw * D2R, om, fx, P));
      if (j < tol) return [r];
      const label = stripLabel(r.label, p => p.startsWith(CAUSTIC));
      return label ? [{ ...r, label }] : [];
    });
  }
  // (2) 孤立交点
  if (P.zw || empty) out = out.filter(r => r.kind === 'aper' || hasValidNeighbour(P, om, fx, r));
  // (3) 汇聚点 (只在平行光路上找: 它来自 D = L·S 的对称结构, 普通光路不会有). 汇聚段很短时 (只擦过区域一角, 网格根点不够)
  //     也能认出来: 两个焦散端落在天上同一点, 取向却不同
  const F = empty || !P.zw ? [] : focalPoints(P, ori, om, fx);
  if (!empty && P.zw) {
    const ce = out.filter(r => r.label.includes(CAUSTIC));
    for (let a = 0; a < ce.length; a++)
      for (let b = a + 1; b < ce.length; b++) {
        const A = ce[a], B = ce[b];
        if (ang(A.D, B.D) > 1e-10 || Math.hypot(A.sc - B.sc, A.sw - B.sw) < 0.05) continue;
        if (!F.some(f => ang(f.D, A.D) < 2e-4)) F.push({ D: A.D, sc: A.sc, sw: A.sw, n: 2 });
      }
  }
  if (F.length) {
    out = out.flatMap(r => {
      if (!r.label.includes(CAUSTIC) || !F.some(f => ang(f.D, r.D) < 1e-4)) return [r];
      const label = stripLabel(r.label, p => p.startsWith(CAUSTIC));
      return label ? [{ ...r, label }] : [];
    });
    for (const f of F) out.push({ kind: 'focus', label: '汇聚点', sc: f.sc, sw: f.sw, D: f.D, ...azEl(f.D), n: f.n });
  }
  return out;
}
