// Lowitz 的 ζ 只转半圈 (0–180°) 时多出来的几何. 默认整圈 (0–360°, 双覆盖, 与 C# 原窗体相同) 时这里都不起作用.
//
// 半圈时成立的区域在 ζ = 0° 与 180° 处被截断 — 那两条线上晶体正好放平 (c 轴竖直), 姿态退化成板晶绕竖轴转.
// 截断边在天上就是板晶的那条弧; 篱笆 / 焦散碰到截断边的地方, 弧在那里结束, 也算末端点.
// 原闭式末端点引擎在双覆盖上求解, ζ > 180° 的那些 (= roll + 180° 的姿态) 不在半圈里, 要去掉.

import { LP, OMode, Chain3, famValue, validAt, validForFam } from './lp.js';

const EDGES = [1e-6, Math.PI - 1e-6];          // 截断边 (稍微往里一点, 免得正好落在极点)

export const zenHalfOn = om => LP.zenHalf && om === OMode.ZenFree;

/** 截断边在天上的像 (只取光路成立的点), 供天空视图当一条边界画出来. */
export function zenEdgeSky(P, om, fx, n = 7200) {
  const pts = [];
  for (const ze of EDGES)
    for (let i = 0; i < n; i++) {
      const al = 2 * Math.PI * i / n;
      if (!validAt(P, om, fx, al, ze, 1e-7)) continue;
      const c = Chain3(al, ze, om, fx, P);
      pts.push([c.Dx, c.Dy, c.Dz]);
    }
  return pts;
}

/** 去掉 ζ > 180° 的末端点, 补上各族与截断边的交点. fams = allFams(P, om). */
export function zenHalfEndpoints(eps, P, om, fx, fams, n = 7200) {
  const out = eps.filter(r => r.sw <= 180 + 1e-7);
  const D2R = Math.PI / 180;
  for (const fm of fams) {
    for (const ze of EDGES) {
      const f = al => famValue(fm, P, om, fx, al, ze);
      let a0 = 0, f0 = f(0);
      for (let i = 1; i <= n; i++) {
        const a1 = 2 * Math.PI * i / n, f1 = f(a1);
        if (Number.isFinite(f0) && Number.isFinite(f1) && (f0 >= 0) !== (f1 >= 0) && Math.max(Math.abs(f0), Math.abs(f1)) > 1e-12) {
          let lo = a0, hi = a1, flo = f0;
          for (let k = 0; k < 48; k++) {
            const mid = 0.5 * (lo + hi), fm2 = f(mid);
            if (!Number.isFinite(fm2)) break;
            if ((fm2 >= 0) === (flo >= 0)) { lo = mid; flo = fm2; } else hi = mid;
          }
          const al = 0.5 * (lo + hi);
          if (validForFam(fm, P, om, fx, al, ze, 1e-6)) {
            const c = Chain3(al, ze, om, fx, P);
            const nn = Math.hypot(c.Dx, c.Dy, c.Dz) || 1, D = [c.Dx / nn, c.Dy / nn, c.Dz / nn];
            out.push({
              kind: 'end', label: `${fm.name}∩ζ边界`, sc: ((al / D2R) % 360 + 360) % 360, sw: ze / D2R, D,
              az: Math.atan2(D[1], D[0]) / D2R, el: Math.asin(Math.max(-1, Math.min(1, D[2]))) / D2R,
            });
          }
        }
        a0 = a1; f0 = f1;
      }
    }
  }
  return out;
}
