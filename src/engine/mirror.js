// 左右镜像: 天上左右对调 (y → −y) 的那一半弧, 来自哪条光路、哪些姿态.
//
// 场景镜像 M: (x, y, z) → (x, −y, z), 太阳在 xz 平面上不动. 六棱晶有 12 个非真对称元素 g (D6h: 6 个竖直镜面,
// 底面镜面与 6 个绕 c 轴转动的复合). 姿态 R 下走光路 P 的光, 在姿态 R' = M·R·g 下走光路 P' = g⁻¹(P)
// (每个面法向都被 M 镜像, 光点链对正交变换协变), 出射方向正好是 M·D — 天上的左右镜像.
// R' 仍在同一张取向平面里 (柱晶: c 轴天顶角 ζ 不变; Lowitz: roll 模 180° 不变) 时, 镜像那一半才画得进这张图.
// 在所有可用的 g 里挑 P' 最接近 P 的: P' = P 时取向平面本身已经左右对称; 其次改动的面最少、优先底面镜像
// (柱晶 ζ = 90° 时 3-1-5 ↔ 3-2-5, 与习惯一致).

import { OMode, faceCoef, faceOk, orientOf } from './lp.js';

const FACES = [1, 2, 3, 4, 5, 6, 7, 8, 13, 14, 15, 16, 17, 18, 23, 24, 25, 26, 27, 28].filter(faceOk);

/** 12 个非真元素 (晶体坐标里的 3×3 矩阵, 行优先) 及类别: 'v' 竖直镜面, 'h' 底面镜面 × 转动. */
function improper() {
  const out = [];
  for (let k = 0; k < 6; k++) {
    const t = k * Math.PI / 3, c = Math.cos(t), s = Math.sin(t);       // 镜面过 c 轴, 与 e1 夹角 t/2
    out.push({ kind: 'v', M: [c, s, 0, s, -c, 0, 0, 0, 1] });
  }
  for (let k = 0; k < 6; k++) {
    const t = k * Math.PI / 3, c = Math.cos(t), s = Math.sin(t);       // 绕 c 轴转 t 再翻底面
    out.push({ kind: 'h', M: [c, -s, 0, s, c, 0, 0, 0, -1] });
  }
  return out;
}

const mulT = (M, v) => [M[0] * v[0] + M[3] * v[1] + M[6] * v[2], M[1] * v[0] + M[4] * v[1] + M[7] * v[2], M[2] * v[0] + M[5] * v[1] + M[8] * v[2]];

/** 面 f 在 g⁻¹ 下变成哪个面 (法向逐分量相等才算; 上下锥顶角不同时底面镜像就不成立). */
function mapFace(g, f) {
  const c = mulT(g.M, faceCoef(f));          // g⁻¹ = gᵀ
  for (const h of FACES) {
    const d = faceCoef(h);
    if (Math.abs(d[0] - c[0]) + Math.abs(d[1] - c[1]) + Math.abs(d[2] - c[2]) < 1e-9) return h;
  }
  return 0;
}

/** R' = M·R·g 还在不在这张取向平面里 (取几个随机姿态检验). */
function keepsSlice(g, m, fx) {
  const e = new Float64Array(9);
  for (const [u, v] of [[0.7, 1.9], [2.3, 4.1], [5.2, 0.4], [3.9, 2.8]]) {
    orientOf(m, fx, u, v, e);
    // R 的列 = e1 (e[0..2]), e2 (e[3..5]), e3 (e[6..8]); R' 的第 k 列 = M · Σ_j g[j][k] e_j
    const col = k => {
      const w = [0, 1, 2].map(i => g.M[k] * e[i] + g.M[3 + k] * e[3 + i] + g.M[6 + k] * e[6 + i]);
      return [w[0], -w[1], w[2]];
    };
    const e1 = col(0), e3 = col(2);
    const ze = Math.acos(Math.max(-1, Math.min(1, e3[2])));
    if (m === OMode.ZetaFixed) {
      if (Math.abs(ze - fx) > 1e-7) return false;
    } else if (m === OMode.ZenFree) {
      const sz = Math.sin(ze);
      if (sz < 1e-6) continue;
      const ca = e3[0] / sz, sa = e3[1] / sz, cz = Math.cos(ze);
      const rho = Math.atan2(e1[0] * -cz * ca + e1[1] * -cz * sa + e1[2] * sz, -e1[0] * sa + e1[1] * ca);
      const d = ((rho - fx) % Math.PI + Math.PI) % Math.PI;          // ζ 走满整圈: roll 与 roll+180° 同在这张图里
      if (Math.min(d, Math.PI - d) > 1e-7) return false;
    } else return false;
  }
  return true;
}

/**
 * 光路 faces 在当前取向平面里的左右镜像光路.
 * 返回 { faces, self } (self = 镜像后还是它自己), 或 null (镜像那一半不在这张取向平面里).
 */
export function mirrorPath(faces, m, fx) {
  let best = null;
  for (const g of improper()) {
    if (!keepsSlice(g, m, fx)) continue;
    const f2 = faces.map(f => mapFace(g, f));
    if (f2.some(f => !f)) continue;
    const diff = f2.reduce((a, f, i) => a + (f !== faces[i] ? 1 : 0), 0);
    const score = diff * 10 + (g.kind === 'h' ? 0 : 1) + (f2[0] !== faces[0] ? 5 : 0);
    if (!best || score < best.score) best = { faces: f2, self: diff === 0, score };
  }
  return best && { faces: best.faces, self: best.self };
}

