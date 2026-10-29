// 术语表的结构 (每条属于哪一组, 按什么顺序) 与几个把引擎内部名换成大白话的函数.
// 文字本身在 locales/*.js 的 terms / groups 里 (先说它是什么、管什么事, 符号放最后当注脚).

import { t, L, term } from './i18n.js';

export const GROUPS = ['basic', 'sky', 'ori', 'fence', 'ep', 'other'];

/** 术语键 → 所属组 (顺序即术语表里的顺序). */
export const TERM_GROUP = {
  lightpoint: 'basic', path: 'basic', faces: 'basic', orientation: 'basic', dof: 'basic',
  sky: 'sky', sphere: 'sky', mc: 'sky', projection: 'sky',
  oriplane: 'ori', refine: 'ori', lift3d: 'ori',
  valid: 'fence', fence: 'fence', entry: 'fence', seam: 'fence', tir: 'fence', exit: 'fence', aperture: 'fence',
  side: 'fence', formal: 'fence', caustic: 'fence', reach: 'fence',
  endpoints: 'ep', ep_end: 'ep', ep_turn: 'ep', ep_cusp: 'ep', ep_cross: 'ep', ep_aper: 'ep', ep_hidden: 'ep',
  partial: 'other', realcrystal: 'other', apex: 'other', chain: 'other', n: 'other',
};

export const groupName = g => L().groups[g];
export { term };

/** 图例 / 列表里的族: 友好名 + 公式标签 + 对应术语 */
export function famInfo(fm, faces, partial) {
  const f = faces && fm.j !== undefined ? faces[fm.j + 1] : '';
  switch (fm.kind) {
    case 0: return { title: t('fam.entry'), tag: 'g₀ = 0', term: 'entry' };
    case 1: return { title: t('fam.seam', { f }), tag: `q${sub(fm.j + 1)} = 0`, term: 'seam' };
    case 2: return partial ? { title: t('fam.blue', { f }), tag: `q${sub(fm.j + 1)} = −√K`, term: 'tir', note: t('fam.blueNote') }
                           : { title: t('fam.tir', { f }), tag: `q${sub(fm.j + 1)} = −√K`, term: 'tir' };
    case 3: return { title: t('fam.exit'), tag: 'q_E = −√K', term: 'exit' };
    case 6: return { title: t('fam.caustic'), tag: 'J = 0', term: 'caustic' };
    case 7: return { title: t('fam.aperture'), tag: t('fam.apertureTag'), term: 'aperture' };
    default: return { title: fm.name, tag: '', term: 'fence' };
  }
}

const SUBS = '₀₁₂₃₄₅₆₇₈₉';
function sub(n) { return String(n).split('').map(c => SUBS[+c]).join(''); }

/** 天空图族名 (Trace2D 的键) → 友好名 */
export function skyName(key, faces, partial) {
  if (key === 'Parry轨迹(1DOF)' || key === '固定ζ极点退化1DOF') return t('sky.dof1');
  if (key === 'ζ边界') return t('sky.zenEdge');
  const m = key.match(/^(\S+)-帽 (.*)$/);
  const base = m ? m[2] : key;
  const fm = famFromName(base);
  const nm = fm ? famInfo(fm, faces, partial).title : base;
  return m ? t('sky.cap', { name: nm }) : nm;
}

/** 引擎族名 → {kind, j} (引擎内部名保持中文, 与 C# 逐字一致) */
export function famFromName(name) {
  if (/入射掠射/.test(name)) return { kind: 0 };
  let m = name.match(/q(\d+)\(面\d+\)=0/);
  if (m) return { kind: 1, j: +m[1] - 1 };
  m = name.match(/q(\d+)\(面\d+\)=-√K/);
  if (m) return { kind: 2, j: +m[1] - 1 };
  if (/出射临界/.test(name)) return { kind: 3 };
  if (/焦散/.test(name)) return { kind: 6 };
  if (/孔径篱笆/.test(name)) return { kind: 7 };
  return null;
}

/** 末端点标签 (引擎内部名) → 大白话 */
export function epLabel(label, partial) {
  const one = s => {
    s = s.trim();
    if (s === 'entry') return t('ep.entry');
    if (s === 'exit') return t('ep.exit');
    if (s === '焦散') return t('ep.caustic');
    if (s === 'Ω边界') return t('ep.omega');
    if (s === 'ζ边界') return t('ep.zenEdge');
    if (s.startsWith('可达带上界')) return t('ep.reachUp');
    if (s.startsWith('可达带下界')) return t('ep.reachLo');
    let m = s.match(/^q\d+\(面(\d+)\)=0/);
    if (m) return t('ep.seam', { f: m[1] });
    m = s.match(/q\d+\(面(\d+)\)=-√K/);
    if (m) return t(partial ? 'ep.blue' : 'ep.tir', { f: m[1] });
    if (/入射掠射/.test(s)) return t('ep.entry');
    if (/出射临界/.test(s)) return t('ep.exit');
    if (/焦散/.test(s)) return t('ep.caustic');
    if (/孔径/.test(s)) return t('ep.aperture');
    return s;
  };
  return label.split(' | ').map(part => {
    if (part.startsWith('焦散尖点')) return t('ep.cuspSky');
    if (part.startsWith('孔径∩')) return t('ep.aperture') + t('ep.and') + one(part.slice(3));
    return part.split('∩').map(one).join(t('ep.and'));
  }).join(t('ep.sep'));
}

/** 末端点类型 (名称随语言变) */
export const EP_KINDS = [
  { kind: 'end', glyph: '●', key: 'epk.end', term: 'ep_end' },
  { kind: 'turn', glyph: '▲', key: 'epk.turn', term: 'ep_turn' },
  { kind: 'cusp', glyph: '✱', key: 'epk.cusp', term: 'ep_cusp' },
  { kind: 'cross', glyph: '×', key: 'epk.cross', term: 'ep_cross' },
  { kind: 'aper', glyph: '◆', key: 'epk.aper', term: 'ep_aper' },
];

/** 取向平面的轴名 (引擎里 axesMeta 的中文标签 → 当前语言) */
export function axisLabel(s) {
  if (/^α/.test(s)) return t('axis.alpha');
  if (/^ζ/.test(s)) return t('axis.zeta');
  if (/^roll/.test(s)) return t('axis.roll');
  return s;
}
