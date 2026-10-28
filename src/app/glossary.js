// 术语表 (右侧抽屉) + 悬停提示. 任何带 data-term="key" 的元素: 鼠标悬停显示一句话解释, 点击打开术语表并定位.
// 换语言时调 rebuild() 重写正文.

import { GROUPS, TERM_GROUP, groupName, term } from './terms.js';
import { t } from './i18n.js';

const $ = id => document.getElementById(id);

export function initGlossary() {
  const sheet = $('sheet'), scrim = $('scrim'), body = $('sheetBody'), nav = $('sheetNav'), tip = $('tip');

  // ---- 术语表正文 ----
  function rebuild() {
    body.textContent = ''; nav.textContent = '';
    for (const g of GROUPS) {
      const keys = Object.keys(TERM_GROUP).filter(k => TERM_GROUP[k] === g);
      if (keys.length === 0) continue;
      const navBtn = document.createElement('button');
      navBtn.textContent = groupName(g);
      nav.append(navBtn);
      const h = document.createElement('h3');
      h.textContent = groupName(g); h.id = 'g-' + g;
      navBtn.addEventListener('click', () => h.scrollIntoView({ behavior: 'smooth', block: 'start' }));
      body.append(h);
      for (const k of keys) {
        const tm = term(k);
        const art = document.createElement('article');
        art.className = 'term'; art.id = 'term-' + k;
        const h4 = document.createElement('h4'); h4.textContent = tm.name;
        art.append(h4);
        for (const para of [tm.short, ...(tm.body || [])]) {
          const p = document.createElement('p'); p.textContent = para; art.append(p);
        }
        if (tm.tech) { const d = document.createElement('div'); d.className = 'tech'; d.textContent = tm.tech; art.append(d); }
        body.append(art);
      }
    }
  }
  rebuild();

  const close = () => { sheet.hidden = true; scrim.hidden = true; };
  const open = key => {
    hideTip();
    sheet.hidden = false; scrim.hidden = false;
    const el = key && document.getElementById('term-' + key);
    if (el) {
      el.scrollIntoView({ block: 'start' });
      el.classList.remove('flash'); void el.offsetWidth; el.classList.add('flash');
    } else body.scrollTop = 0;
  };
  $('btnSheetClose').addEventListener('click', close);
  scrim.addEventListener('click', close);
  window.addEventListener('keydown', e => { if (e.key === 'Escape' && !sheet.hidden) close(); });

  // ---- 悬停提示 (仅鼠标; 触屏直接点开术语表) ----
  let tipFor = null, tipTimer = 0;
  function hideTip() { clearTimeout(tipTimer); tip.hidden = true; tipFor = null; }
  function showTip(el) {
    const tm = term(el.dataset.term);
    if (!tm) return;
    tip.textContent = '';
    const b = document.createElement('b'); b.textContent = tm.name;
    const s = document.createElement('span'); s.textContent = tm.short;
    const sm = document.createElement('small'); sm.textContent = t('tip.more');
    tip.append(b, s, sm);
    tip.hidden = false;
    const r = el.getBoundingClientRect(), tr = tip.getBoundingClientRect();
    let x = r.left + r.width / 2 - tr.width / 2, y = r.bottom + 8;
    if (y + tr.height > window.innerHeight - 8) y = r.top - tr.height - 8;
    x = Math.max(8, Math.min(window.innerWidth - tr.width - 8, x));
    tip.style.left = x + 'px'; tip.style.top = Math.max(8, y) + 'px';
  }
  document.addEventListener('pointerover', e => {
    if (e.pointerType !== 'mouse') return;
    const el = e.target.closest('[data-term]');
    if (el === tipFor) return;
    hideTip();
    if (!el) return;
    tipFor = el;
    tipTimer = setTimeout(() => showTip(el), 220);
  });
  document.addEventListener('focusin', e => {
    const el = e.target.closest && e.target.closest('[data-term]');
    if (el && el.matches(':focus-visible')) { tipFor = el; showTip(el); }
  });
  document.addEventListener('focusout', hideTip);
  document.addEventListener('scroll', hideTip, true);
  document.addEventListener('click', e => {
    const el = e.target.closest('[data-term]');
    if (!el || el.dataset.termClick === 'off') return;
    e.preventDefault(); e.stopPropagation();
    open(el.dataset.term);
  });

  return { open, close, hideTip, rebuild };
}
