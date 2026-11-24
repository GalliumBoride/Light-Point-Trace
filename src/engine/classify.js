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
import { ARC_PHYS } from './arcphys.js';

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

// ---------- 一行的真实光路 (同 C# ArcClassifierForm 的反向生成: MotherSurfaceKernel.ReverseGenerate + 签名 + CheckReachability)
// 表里每行的 Raypath 只是代表 (入射面 + 最短反射序列 + 出射面, 按群的代数算出来), 常常没有任何取向能走通 — 不能拿给人用.
// 真光路 = 签名与这一行相同的 (入射面, 出射面) 对 × 合成镜相同的反射序列, 再要求某个光线方向能依次走通 (可达).
// 取能走通的最短长度上的全部序列 (同一行常有两条互补的, 如 4-6-3 与 4-3-6, 只取一条会丢光).
const STATES = (() => {                                                    // 群的 12 个元素 (g, e) 与最短距离表
  const els = [];
  for (const g of GN) for (const e of [0, 1]) els.push(gmat(g, e));
  const idx = M => els.findIndex(A => close(A, M));
  const next = els.map(A => { const row = [0]; for (let f = 1; f <= 8; f++) row.push(idx(mul(FREFL[f], A))); return row; });
  const dist = els.map((_, s) => {
    const d = new Array(12).fill(99); d[s] = 0;
    const q = [s];
    while (q.length) { const c = q.shift(); for (let f = 1; f <= 8; f++) { const n = next[c][f]; if (d[n] > d[c] + 1) { d[n] = d[c] + 1; q.push(n); } } }
    return d;
  });
  return { els, idx, next, dist };
})();
const SIG_PAIRS = (() => {                                                 // 签名|群元 → [(入射面, 出射面)] (同 C# BuildSigGepsPairsTable)
  const map = new Map();
  for (const g of GN) for (const e of [0, 1]) {
    const seq = MINSEQ.get(g + e);
    for (let en = 1; en <= 8; en++) for (let ex = 1; ex <= 8; ex++) {
      const k = signature([en, ...seq, ex]) + '|' + g + e;
      if (!map.has(k)) map.set(k, []);
      map.get(k).push([en, ex]);
    }
  }
  return map;
})();
const COS_TC = Math.cos(Math.asin(1 / N_TABLE));
/**
 * 同 C# CheckReachability: 球冠约束 (入射: 折射后方向在 −n_e 的临界锥里; 每次反射: 朝着那面走; 出射: 在 n_x 的临界锥里) 有没有公共点.
 * C# 只要求打得到那面 (= 允许部分反射); tir = true 时每次反射再加一个球冠「入射角 ≥ 临界角」(d·n ≤ cosθc ⟺ d·(−n) ≥ −cosθc),
 * 就是界面默认的全反射规则. 同一个找点办法 (40 个起点的坐标下降), 也同样把约束都表示成最后那段光的方向.
 */
function reachable(path, tir = false) {
  if (path.length < 2) return false;
  const caps = [[NORM[path[0]].map(v => -v), COS_TC]];
  for (let i = 1; i < path.length - 1; i++) {
    const n = NORM[path[i]];
    caps.push([n.slice(), 0]);
    if (tir) caps.push([n.map(v => -v), -COS_TC]);
    for (const c of caps) { const d = dot(c[0], n); c[0] = [c[0][0] - 2 * d * n[0], c[0][1] - 2 * d * n[1], c[0][2] - 2 * d * n[2]]; }
  }
  caps.push([NORM[path[path.length - 1]].slice(), COS_TC]);
  const acos = x => Math.acos(Math.max(-1, Math.min(1, x)));
  for (let i = 0; i < caps.length; i++)
    for (let j = i + 1; j < caps.length; j++)
      if (acos(caps[i][1]) + acos(caps[j][1]) < acos(dot(caps[i][0], caps[j][0])) - 1e-9) return false;
  const worst = (t, p) => {
    const d = [Math.sin(t) * Math.cos(p), Math.sin(t) * Math.sin(p), Math.cos(t)];
    let m = -Infinity;
    for (const c of caps) m = Math.max(m, c[1] - dot(d, c[0]));
    return m;
  };
  for (let it = 0; it < 5; it++)
    for (let ip = 0; ip < 8; ip++) {
      let t = (it + 0.5) * Math.PI / 5, p = ip * Math.PI / 4, step = 0.1, v = 0;
      for (let iter = 0; iter < 100; iter++) {
        v = worst(t, p);
        if (v < -1e-9) break;
        let bv = v, bt = t, bp = p;
        for (const [dt, dp] of [[step, 0], [-step, 0], [0, step], [0, -step]]) {
          const nt = Math.max(0.01, Math.min(Math.PI - 0.01, t + dt)), np = p + dp, nv = worst(nt, np);
          if (nv < bv) { bv = nv; bt = nt; bp = np; }
        }
        if (Math.abs(bv - v) < 1e-12) step *= 0.5;
        if (step < 1e-10) break;
        t = bt; p = bp;
      }
      if (worst(t, p) < -1e-9) return true;
    }
  return false;
}
const PHYS = new Map();
/**
 * 表的第 r 行在真实光路里怎么走 (表的面号): 签名与合成镜都对的序列里, 在当前反射规则下有光线方向能走通的 (C# 可达判据,
 * 不允许部分反射时加全反射约束 — 否则很多光路默认设置下一个点也画不出), 取最短的那个长度上的全部 (最多比代数最短长 2).
 * partial = 允许部分反射 (界面的「蓝边」).
 */
function physPaths(r, partial) {
  const ck = r.i + (partial ? 'p' : 't');
  if (PHYS.has(ck)) return PHYS.get(ck);
  const pre = ARC_PHYS[r.i];                                              // 预先生成的 (arcphys.js), 长光路现算要好几秒
  if (pre) { const out = pre[partial ? 1 : 0].map(s => s.split('-').map(Number)); PHYS.set(ck, out); return out; }
  return livePhysPaths(r, partial);
}
/** 现算 (arcphys.js 的生成器也用它): 见 physPaths */
export function livePhysPaths(r, partial) {
  if (typeof r === 'number') r = ROWS[r];
  const ck = r.i + (partial ? 'p' : 't');
  const sig = signature(r.path), ge = r.g + r.e, target = STATES.idx(r.L);
  const pairs = SIG_PAIRS.get(sig + '|' + ge) || [[r.ex, r.x]];
  const minLen = STATES.dist[STATES.idx(I3)][target] + 2;
  let out = [];
  for (let len = minLen; len <= minLen + 2 && !out.length; len++) {
    const nRefl = len - 2, seq = new Array(nRefl);
    for (const [en, ex] of pairs) {
      const dfs = (depth, s) => {
        if (STATES.dist[s][target] > nRefl - depth) return;
        if (depth === nRefl) {
          if (s !== target) return;
          const p = [en, ...seq, ex];
          if (signature(p) === sig && reachable(p, !partial)) out.push(p);
          return;
        }
        for (let f = 1; f <= 8; f++) { seq[depth] = f; dfs(depth + 1, STATES.next[s][f]); }
      };
      dfs(0, STATES.idx(I3));
    }
  }
  PHYS.set(ck, out);
  return out;
}

/**
 * 光路 (界面面号) → 名字卡片的数据; 锥面光路或少于两个面返回 null.
 * { names: {column, plate, parry, lowitz}, steps: { e, mids, x, xd, wedge, L, roles: [{o, e, xd, L}] },
 *   same: [{ o, paths: [界面面号光路, 真能走通的], needPartial: 只在允许部分反射时有光, conflict: [名字…] }] }
 * opts.partial: 界面的「蓝边」(允许部分反射) — 决定同一片晕里列哪些光路; opts.orient: 只列这种取向 (0–3) 的, 其余留空.
 */
export function classifyPath(faces, opts = {}) {
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
    // 列真光路 (同 C# 窗体的反向生成, 再按当前反射规则筛), 连同自己这一行的其它走法; 不列表里那条只是代数代表的 Raypath.
    // 全反射下一条都走不通、允许部分反射才有光的晕: 列部分反射下的光路, 标 needPartial
    const typed = faces.join('-');
    const collect = part => {
      const seen = new Set([typed]), out = [];
      for (const q of [r, ...members])
        for (const p of physPaths(q, part)) { const s = p.map(toWeb).join('-'); if (!seen.has(s)) { seen.add(s); out.push(s); } }
      return out;
    };
    let paths = [], needPartial = false;
    if (opts.orient === undefined || opts.orient === o) {             // 只算要显示的那种取向 (长光路的生成不便宜)
      paths = collect(!!opts.partial);
      if (!paths.length && !opts.partial) { paths = collect(true); needPartial = paths.length > 0; }
    }
    same.push({ o: O.key, paths, needPartial, conflict: nm.size > 1 ? [...nm] : null });
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
