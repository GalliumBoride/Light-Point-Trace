// 界面语言: 简体中文与英文. 文字都在 locales/*.js: zh-CN 是原文, en 的键与结构完全相同 (tools/check-locales.mjs 检查).
// t('键', {变量}) 取当前语言, 缺了退回简体中文. HTML 里带 data-i18n / data-i18n-title / -aria / -ph 的元素由 applyI18n 换字.

import zhCN from './locales/zh-CN.js';
import en from './locales/en.js';

export const LANGS = [
  { code: 'zh-CN', label: '中文', dict: zhCN },
  { code: 'en', label: 'English', dict: en },
];
let cur = LANGS[0];

/** 记住的语言, 否则浏览器首选中文就用中文, 其余用英文. */
export function detectLang(saved) {
  if (saved && LANGS.some(l => l.code === saved)) return saved;
  const first = String((navigator.languages && navigator.languages[0]) || navigator.language || '').toLowerCase();
  return first.startsWith('zh') ? 'zh-CN' : 'en';
}

export function setLang(code) {
  cur = LANGS.find(l => l.code === code) || LANGS[0];
  document.documentElement.lang = cur.code;
  return cur.code;
}

export function t(key, vars) {
  let s = cur.dict.ui[key];
  if (s === undefined) s = zhCN.ui[key];
  if (s === undefined) return key;
  if (vars) s = s.replace(/\{(\w+)\}/g, (m, k) => (vars[k] !== undefined ? vars[k] : m));
  return s;
}

/** 当前语言的整份文字 (modes / lenses / groups / terms / presets). */
export const L = () => cur.dict;
export const term = key => cur.dict.terms[key] || zhCN.terms[key];

/** 把带 data-i18n* 的元素换成当前语言 (值里有标记的按 HTML 插入, 只用于语言文件自己的标记). */
export function applyI18n(root = document) {
  for (const el of root.querySelectorAll('[data-i18n]')) {
    const s = t(el.dataset.i18n);
    if (s.includes('<')) el.innerHTML = s; else el.textContent = s;
  }
  for (const [attr, name] of [['data-i18n-title', 'title'], ['data-i18n-aria', 'aria-label'], ['data-i18n-ph', 'placeholder']])
    for (const el of root.querySelectorAll(`[${attr}]`)) el.setAttribute(name, t(el.getAttribute(attr)));
}
