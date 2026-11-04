// 弧的名字 — 输入一条光路 (底面 1/2 + 柱面 3–8), 给出它在四种取向下叫什么、哪些光路是同一片晕、名字是怎么来的.
//
// 三步 (都是闭式, 不靠模拟):
//  ① 化简: 光路 = 入射面 e + 中间几面镜子 + 出射面 x. 把镜子都挪到最后 = 直接从 e 进、从 x' 出 (x' = 出射面被镜子照回去),
//     再照一面「合成镜」L. 弧只由 (e, L, x) 决定, 反射的先后、次数不同而 L 相同就是同一条弧 — 这就是 336 弧表的一行
//     (行的签名同 MotherSurfaceKernel: 太阳高度 30° 时光点在出射面上的投影 + 出射面类型 + L).
//  ② 骨架: e 与 x' 的夹角. 0° = 平行光路, 光不偏折, 天空方向就是太阳照镜子 L; 60° = 22° 家族; 90° = 46° 家族.
//  ③ 在这种取向里: 入射面、直接出射面、合成镜各是什么 (水平 / 竖直 / 斜着跟晶体转). 名字由这三样定 (词汇取自 336 表).
//  同一片晕: 晶体对称里不改变这群晶体整体样子的那些 (取向的对称群) 把一行变成另一行; 0° 楔只看 L; Lowitz 还多一条:
//  只用面 1, 2, 3, 6 的光路绕转轴转 90° 互相轮换 (myown/experiments/arc_naming/README.md, 已用模拟逐类核对).
//
// 面号: 界面 (lp.js BETA) 的柱面 3–8 法线按 0°, −60°, −120°, … 排, 336 表按 0°, 60°, 120°, … 排, 两者差一个左右镜像;
// 这里先换成表的面号再算, 输出时换回来. 名字不受影响 (镜像只改名字里的 M, 界面不显示 M).

import { ARC_ROWS } from './arcnames.js';

const S3 = Math.sqrt(3) / 2;
const NORM = { 1: [0, 0, 1], 2: [0, 0, -1], 3: [1, 0, 0], 4: [0.5, S3, 0], 5: [-0.5, S3, 0], 6: [-1, 0, 0], 7: [-0.5, -S3, 0], 8: [0.5, -S3, 0] };
const PTYPE = { 3: 'A', 6: 'A', 4: 'B', 7: 'B', 5: 'C', 8: 'C' };
const WEB2K = { 1: 1, 2: 2, 3: 3, 4: 8, 5: 7, 6: 6, 7: 5, 8: 4 };      // 界面面号 ↔ 表的面号 (自逆)
const N_TABLE = 1.31;                                                     // 签名用的折射率 (同 C# 内核, 与界面的 n 无关)

// ---------- 3×3 (行优先 9 元数组)
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const mul = (A, B) => {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
  return C;
};
const tr = A => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
const mv = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const crs = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const refl = n => [1 - 2 * n[0] * n[0], -2 * n[0] * n[1], -2 * n[0] * n[2], -2 * n[1] * n[0], 1 - 2 * n[1] * n[1], -2 * n[1] * n[2],
                   -2 * n[2] * n[0], -2 * n[2] * n[1], 1 - 2 * n[2] * n[2]];
const close = (A, B) => A.every((a, i) => Math.abs(a - B[i]) < 1e-9);
const det = A => A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);

// ---------- 反射群 G = D3 × Z2 (g 管柱面反射, e 管底面反射的奇偶)
const RA = refl(NORM[3]), RB = refl(NORM[4]), RC = refl(NORM[5]), MZ = refl(NORM[1]);
const D3 = { I: I3, rho: mul(RA, RB), rho2: mul(RA, RC), R_A: RA, R_B: RB, R_C: RC };
const GN = ['I', 'rho', 'rho2', 'R_A', 'R_B', 'R_C'];
const gmat = (g, e) => (e ? mul(D3[g], MZ) : D3[g]);
function gname(M) {
  for (const g of GN) for (const e of [0, 1]) if (close(gmat(g, e), M)) return [g, e];
  return null;
}
const faceOf = v => { for (const f in NORM) if (Math.abs(dot(NORM[f], v) - 1) < 1e-9) return +f; return 0; };
const FREFL = {};
for (const f in NORM) FREFL[f] = refl(NORM[f]);
const accumulate = path => path.slice(1, -1).reduce((M, f) => mul(FREFL[f], M), I3);

/** 行签名: 太阳 (高度 30°, 方位 0) 下光点在出射面上的投影 (出射面与它的对面并在一起) + 出射面类型 */
function signature(path) {
  const S = [Math.cos(Math.PI / 6), 0, Math.sin(Math.PI / 6)];
  const nIn = NORM[path[0]], mu = dot(S, nIn), lam = -mu + Math.sqrt(mu * mu + N_TABLE * N_TABLE - 1);
  let P = [S[0] + lam * nIn[0], S[1] + lam * nIn[1], S[2] + lam * nIn[2]];
  for (const f of path.slice(1, -1)) {
    const n = NORM[f], q = 2 * dot(P, n);
    P = [P[0] - q * n[0], P[1] - q * n[1], P[2] - q * n[2]];
  }
  const x = path[path.length - 1], nO = NORM[x], q = dot(P, nO);
  return [P[0] - q * nO[0], P[1] - q * nO[1], P[2] - q * nO[2]]
    .map(v => { const r = Math.round(v * 1e4) / 1e4; return (r === 0 ? 0 : r).toFixed(4); }).join(',') + '|' + (x > 2 ? PTYPE[x] : 'B');
}

/** (I,0) → (g,e) 的最短反射序列 (面序 1..8 的广度优先, 同 C# FindMinimalSeq) */
const MINSEQ = (() => {
  const out = new Map([['I0', []]]), q = [['I', 0]];
  while (q.length) {
    const [g, e] = q.shift(), seq = out.get(g + e);
    for (let f = 1; f <= 8; f++) {
      const [g2, e2] = gname(mul(FREFL[f], gmat(g, e)));
      if (!out.has(g2 + e2)) { out.set(g2 + e2, [...seq, f]); q.push([g2, e2]); }
    }
  }
  return out;
})();

// ---------- 表
const ROWS = ARC_ROWS.map(([ms, raypath, parry, column, plate, lowitz], i) => {
  const path = raypath.split('-').map(Number), L = accumulate(path), [g, e] = gname(L);
  const ex = path[0], x = path[path.length - 1], xd = faceOf(mv(tr(L), NORM[x]));
  const c = -dot(NORM[ex], NORM[xd]);
  return { i, ms, raypath, path, L, g, e, ex, x, xd, zw: Math.abs(Math.abs(c) - 1) < 1e-9,
           names: { parry, column, plate, lowitz } };
});
const KEY = new Map(ROWS.map(r => [signature(r.path) + '|' + r.g + r.e, r.i]));
const MS_FIRST = new Map();
for (const r of ROWS) if (!MS_FIRST.has(r.ms)) MS_FIRST.set(r.ms, r.i);

function rowOf(e, L, x) {
  const gn = gname(L);
  if (!gn || !e || !x) return null;
  const k = KEY.get(signature([e, ...MINSEQ.get(gn[0] + gn[1]), x]) + '|' + gn[0] + gn[1]);
  return k === undefined ? null : k;
}

// ---------- 晶体对称 D6h 与各取向的对称群
const D6H = (() => {
  const gens = [[0.5, -S3, 0, S3, 0.5, 0, 0, 0, 1], refl([0, 1, 0]), MZ], G = [I3];
  for (let changed = true; changed;) {
    changed = false;
    for (const a of [...G]) for (const b of gens) { const c = mul(a, b); if (!G.some(d => close(c, d))) { G.push(c); changed = true; } }
  }
  return G;
})();
const stab = a => D6H.filter(h => Math.abs(Math.abs(dot(a, mv(h, a))) - 1) < 1e-9);

/**
 * 四种取向 (界面 modeIdx 0 柱晶 / 1 板晶 / 2 Parry / 3 Lowitz). 晶体系 (表的面号): c 轴 = z, 面 3 法线 = x.
 * G = 不改变这群晶体整体样子的晶体对称; V = 始终竖直的晶体方向, H = 始终水平的晶体方向 (用来说面和镜子在天上是什么样).
 * Parry 按 3 朝上与 6 朝上都算 (真实大气里两种一样多); Lowitz 按经典轴 (长对角线, 界面 roll 270°).
 */
export const ORIENTS = [
  { key: 'column', G: D6H, V: null, H: [0, 0, 1] },
  { key: 'plate', G: D6H, V: [0, 0, 1], H: null },
  { key: 'parry', G: stab([1, 0, 0]), V: [1, 0, 0], H: null },
  { key: 'lowitz', G: stab([0, 1, 0]), V: null, H: [0, 1, 0] },
];

function act(h, r) { return rowOf(faceOf(mv(h, NORM[r.ex])), mul(mul(h, r.L), tr(h)), faceOf(mv(h, NORM[r.x]))); }

const CLASS = new Map();
/** 各行在取向 o 下的晕类号 (同号 = 同一片晕或它的左右镜像) */
function classesOf(o) {
  if (CLASS.has(o)) return CLASS.get(o);
  const O = ORIENTS[o], n = ROWS.length, par = [...Array(n).keys()];
  const find = a => { while (par[a] !== a) a = par[a] = par[par[a]]; return a; };
  const union = (a, b) => { a = find(a); b = find(b); if (a !== b) par[a] = b; };
  for (const r of ROWS) for (const h of O.G) { const j = act(h, r); if (j !== null) union(r.i, j); }
  // 0° 楔: 天空方向 = 太阳照合成镜 L, 只看 L 在世界里是哪种变换 (L 被取向的对称群共轭到一起就是同一种)
  const first = new Map();
  for (const r of ROWS) {
    if (!r.zw) continue;
    let k = null;
    for (const h of O.G) { const [g, e] = gname(mul(mul(h, r.L), tr(h))); if (k === null || g + e < k) k = g + e; }
    if (O.key === 'lowitz' && k === 'I1') k = 'R_A0';          // 底面镜与柱面 3/6 的镜面都含转轴, 随自转一样
    if (first.has(k)) union(r.i, first.get(k)); else first.set(k, r.i);
  }
  // Lowitz: 只用面 1, 2, 3, 6 的光路, 绕转轴转 90° 把这四个面轮换 (1→3→2→6), 自转又是任意的
  if (O.key === 'lowitz') {
    const Q = [0, 0, 1, 0, 1, 0, -1, 0, 0];
    for (const r of ROWS) {
      if (r.zw || !r.path.every(f => f === 1 || f === 2 || f === 3 || f === 6)) continue;
      const j = rowOf(faceOf(mv(Q, NORM[r.ex])), mul(mul(Q, r.L), tr(Q)), faceOf(mv(Q, NORM[r.x])));
      if (j !== null) union(r.i, j);
    }
  }
  const ids = ROWS.map(r => find(r.i));
  CLASS.set(o, ids);
  return ids;
}

// ---------- 面与镜子在这种取向里是什么样
function vkind(v, O) {
  if (O.V && Math.abs(Math.abs(dot(v, O.V)) - 1) < 1e-9) return 'vert';
  if ((O.V && Math.abs(dot(v, O.V)) < 1e-9) || (O.H && Math.abs(Math.abs(dot(v, O.H)) - 1) < 1e-9)) return 'horiz';
  return 'tilt';
}
/** 面: 法线竖直 = 水平面 */
const faceKind = (f, O) => ({ vert: 'flat', horiz: 'upright', tilt: 'slant' })[vkind(NORM[f], O)];
/** 合成镜 L 是哪一种 + 它在取向 O 里的样子 */
function mirrorKind(g, e, O) {
  if (g === 'I' && !e) return { type: 'none' };
  if (g === 'I') return { type: 'mirror', world: { vert: 'flat', horiz: 'upright', tilt: 'slant' }[vkind(NORM[1], O)], faces: [1, 2] };
  if (g[0] === 'R') {
    const f = { R_A: 3, R_B: 4, R_C: 5 }[g], faces = [WEB2K[f], WEB2K[f + 3]].sort((p, q) => p - q);   // 镜面所在的那一对柱面 (界面面号)
    if (!e) return { type: 'mirror', world: { vert: 'flat', horiz: 'upright', tilt: 'slant' }[vkind(NORM[f], O)], faces };
    const a = crs(NORM[f], NORM[1]);
    return { type: 'half', world: vkind(a, O), faces };                  // 柱面镜 + 底面镜 = 绕两镜交线转半圈
  }
  return { type: e ? 'turn120m' : 'turn120', world: vkind(NORM[1], O) };  // 两面柱面镜 = 绕 c 轴转 120°
}

const stripM = s => s.replace(/ M$/, '');
const toWeb = f => WEB2K[f];

/**
 * 光路 (界面面号) → 名字卡片的数据; 锥面光路或少于两个面返回 null.
 * { names: {column, plate, parry, lowitz}, steps: { e, mids, x, xd, wedge, L, roles: [{o, e, xd, L}] },
 *   same: [{ o, paths: [界面面号光路], conflict: [名字…] }] }
 */
export function classifyPath(faces) {
  if (!faces || faces.length < 2 || faces.some(f => !(f >= 1 && f <= 8))) return null;
  const k = faces.map(f => WEB2K[f]);
  const L = accumulate(k), gn = gname(L);
  const i = KEY.get(signature(k) + '|' + gn[0] + gn[1]);
  if (i === undefined) return null;
  const r = ROWS[i];
  const e = k[0], x = k[k.length - 1], xd = faceOf(mv(tr(L), NORM[x]));
  // 楔角: 出射面与它的对面在表里并在一起, 取光出得去的那个
  const c = -dot(NORM[e], NORM[xd]);
  const wedge = Math.abs(Math.abs(c) - 1) < 1e-9 ? 0 : Math.abs(c) < 1e-9 ? 90 : 60;
  const names = {}, roles = [], same = [];
  ORIENTS.forEach((O, o) => {
    names[O.key] = stripM(r.names[O.key]);
    roles.push({ o: O.key, e: faceKind(e, O), xd: faceKind(xd, O), L: mirrorKind(gn[0], gn[1], O) });
    const ids = classesOf(o), mine = ids[i];
    const members = ROWS.filter(q => ids[q.i] === mine && q.i !== i);
    const nm = new Set([names[O.key], ...members.map(q => stripM(q.names[O.key]))]);
    same.push({ o: O.key, paths: members.map(q => q.path.map(toWeb).join('-')), conflict: nm.size > 1 ? [...nm] : null });
  });
  return {
    row: i + 1, ms: r.ms, names, same,
    steps: { e: toWeb(e), mids: faces.slice(1, -1), x: toWeb(x), xd: toWeb(xd), wedge, L: mirrorKind(gn[0], gn[1], ORIENTS[1]), roles },
  };
}

/** 自检用: 表里每一行的光路都要分回自己那一行 */
export function selfTest() {
  let bad = 0;
  for (const r of ROWS) { const c = classifyPath(r.path.map(toWeb)); if (!c || c.row !== r.i + 1) bad++; }
  return { rows: ROWS.length, bad, classes: ORIENTS.map((O, o) => [O.key, new Set(classesOf(o)).size]) };
}

export { det, D6H };
