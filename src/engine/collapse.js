// 退化光路: 雅可比处处为 0 — 整块成立的姿态被压成天上的一条线 (如柱晶 3-5-6-8、3-1-6、1-4-6-2).
// 这时焦散 J = 0 没有意义: J 只剩舍入噪声 (~1e-16, 出射临界边附近被 √ 放大到 ~1e-7), 原引擎会把噪声的
// 正负翻转当成成百上千个根, 画出一堆假焦散线、假末端点 (3-5-6-8: 3610 个根、282 个"焦散∩入射"末端点).
// 判据: 随机取光路成立的姿态, |J| 的中位数 < 1e-9. 实测正常光路 ≥ 1e-2, 退化光路 ~1e-16, 中间没有灰区.
// 只在界面这一层用 (Worker), 引擎本身仍与 C# 逐点一致.

import { validAt, jana3 } from './lp.js';
import { DotNetRandom } from './random.js';

export function isCollapsed(P, m, fx) {
  const rng = new DotNetRandom(11), J = [];
  for (let i = 0; i < 40000 && J.length < 600; i++) {
    const u = rng.nextDouble() * 2 * Math.PI, v = rng.nextDouble() * 2 * Math.PI;
    if (!validAt(P, m, fx, u, v, 1e-8)) continue;
    const j = jana3(u, v, m, fx, P);
    if (Number.isFinite(j)) J.push(Math.abs(j));
  }
  if (J.length < 20) return false;           // 成立的姿态太少, 判断不了: 按正常光路处理
  J.sort((a, b) => a - b);
  return J[J.length >> 1] < 1e-9;
}

/** 焦散相关的末端点 (标签里带「焦散」), 退化光路上全是噪声. */
export const isCausticEp = r => /焦散/.test(r.label || '');
