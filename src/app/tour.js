// 新手教程: 一步一步带着访客把工具用一遍. 每一步把界面上的一样东西圈出来 (其余变暗), 旁边一张卡片说
// 「它是干什么的 → 现在请你做什么」; 要动手的步骤等访客自己做完 (或点「替我做」) 就自动进下一步.
// 文字在 locales 的 tour.* 里 (中英两份). 第一次来的访客自动弹出欢迎卡; 顶栏「教程」随时重来.
//
// api (main.js 提供): { S, t, setTab, demo({path, mode, fixed, sun, view}), clickView(v), probeAt(x, y), getR(), openMore(), save() }

const $ = id => document.getElementById(id);
const q = sel => document.querySelector(sel);
const sectionOf = el => (el ? el.closest('section, .step, details, .opt') || el : null);

/** 步骤表. target: 要圈出的元素 (返回 null = 居中的卡片); tab: 窄屏时它在哪一栏; enter: 进入这一步时先把界面摆好;
 *  wait: 访客做到了没有; doIt: 「替我做」 */
function steps(api) {
  const { S } = api;
  const modeBtn = m => q(`#modes button[data-mode="${m}"]`), viewBtn = v => q(`#viewSeg button[data-view="${v}"]`);
  return [
    { id: 'welcome', target: () => null, welcome: true },
    { id: 'sky', target: () => $('plot'),
      enter: () => api.demo({ path: '3-5', mode: 0, fixed: 90, sun: 20, view: 0 }) },
    { id: 'sun', tab: 'setup', target: () => sectionOf($('slSun')),
      wait: () => S.sun >= 35,
      doIt: () => { $('inSun').value = 40; $('slSun').value = 40; $('slSun').dispatchEvent(new Event('change')); } },
    { id: 'modes', tab: 'setup', target: () => $('modes'),
      wait: () => S.mode === 1, doIt: () => modeBtn(1).click() },
    { id: 'sundogs', target: () => $('plot'),
      enter: () => { if (S.mode !== 1) modeBtn(1).click(); } },
    { id: 'path', tab: 'setup', target: () => sectionOf($('inPath')),
      enter: () => api.demo({ path: '3-5', mode: 1, fixed: 90, sun: 20, view: 0 }),
      wait: () => S.path.replace(/\s/g, '') === '1-3',
      doIt: () => { $('inPath').value = '1-3'; $('inPath').dispatchEvent(new Event('input')); $('btnCompute').click(); } },
    { id: 'crystal', tab: 'setup', target: () => $('cvCrystal').closest('.crystal-box') },
    { id: 'presets', tab: 'setup', target: () => $('btnPresets') },
    { id: 'lines', tab: 'read', target: () => sectionOf($('curves')),
      enter: () => api.demo({ path: '3-5', mode: 0, fixed: 90, sun: 20, view: 0 }) },   // 换回切弧: 有边界、焦散和末端点可讲
    { id: 'eps', tab: 'read', target: () => sectionOf($('epTable')) },
    { id: 'name', tab: 'read', target: () => $('nameCard') },
    { id: 'ori', target: () => $('viewSeg'),
      enter: () => api.demo({ path: '3-5', mode: 0, fixed: 90, sun: 20, view: 0 }),
      wait: () => S.view === 1 && !S.oriTorus, doIt: () => { if (S.oriTorus) q('#oriSeg button[data-torus="0"]').click(); viewBtn(1).click(); } },
    { id: 'probe', target: () => $('plot'),
      enter: () => { if (S.view !== 1) viewBtn(1).click(); if (S.oriTorus) q('#oriSeg button[data-torus="0"]').click(); },
      wait: () => !!S.probe,
      doIt: () => {
        const R = api.getR(), o = R && R.paths[0] && R.paths[0].ori;
        if (o && o.mcX.length) { const i = o.mcX.length >> 1; api.probeAt(o.mcX[i], o.mcY[i]); }
      } },
    { id: 'torus', target: () => $('oriSeg'),
      enter: () => { if (S.view !== 1) viewBtn(1).click(); },
      wait: () => S.view === 1 && S.oriTorus, doIt: () => q('#oriSeg button[data-torus="1"]').click() },
    { id: 'lift', target: () => viewBtn(2), wait: () => S.view === 2, doIt: () => viewBtn(2).click() },
    { id: 'sphere', target: () => viewBtn(3), wait: () => S.view === 3, doIt: () => viewBtn(3).click() },
    { id: 'layers', target: () => $('btnLayers'),
      enter: () => { if (S.view !== 0) viewBtn(0).click(); } },
    { id: 'partial', tab: 'setup', target: () => $('optPartial') },
    { id: 'more', tab: 'setup', target: () => $('more'), enter: () => api.openMore() },
    { id: 'camera', tab: 'setup', target: () => $('stepCam') },
    { id: 'glossary', target: () => $('btnGlossary') },
    { id: 'export', target: () => $('btnExport') },
    { id: 'done', target: () => null, last: true },
  ];
}

export function createTour(api) {
  const { t, S } = api;
  let list = null, idx = -1, root = null, spot = null, card = null, raf = 0, doneAt = 0, armed = true, lastRect = '';

  function build() {
    root = document.createElement('div');
    root.className = 'tour';
    root.innerHTML = '<div class="tour-spot"></div><div class="tour-card" role="dialog" aria-modal="false"></div>';
    document.body.append(root);
    spot = root.querySelector('.tour-spot');
    card = root.querySelector('.tour-card');
    document.addEventListener('keydown', e => {
      if (idx < 0) return;
      if (e.key === 'Escape') close();
    });
  }

  const btn = (cls, label, fn) => {
    const b = document.createElement('button');
    b.className = 'btn ' + cls; b.textContent = label;
    b.addEventListener('click', fn);
    return b;
  };

  function render() {
    const st = list[idx], n = list.length;
    card.textContent = '';
    card.classList.toggle('center', !st.target());
    if (!st.welcome && !st.last) {
      const meta = document.createElement('div');
      meta.className = 'tour-meta';
      meta.innerHTML = `<span>${t('tour.step', { i: idx, n: n - 2 })}</span>`;
      const bar = document.createElement('i'); bar.style.width = (100 * idx / (n - 2)).toFixed(1) + '%';
      const track = document.createElement('b'); track.append(bar);
      meta.append(track);
      card.append(meta);
    }
    const h = document.createElement('h4'); h.textContent = t(`tour.${st.id}.t`);
    const p = document.createElement('div'); p.className = 'tour-body'; p.innerHTML = t(`tour.${st.id}.b`);
    card.append(h, p);
    if (st.wait) {
      const ok = document.createElement('div');
      ok.className = 'tour-ok'; ok.textContent = t('tour.ok'); ok.hidden = !st.wait();
      card.append(ok);
    }
    const row = document.createElement('div'); row.className = 'tour-btns';
    if (st.welcome) {
      row.append(btn('ghost', t('tour.later'), () => close()), btn('primary', t('tour.start'), () => go(1)));
    } else if (st.last) {
      row.append(btn('ghost', t('tour.back'), () => go(idx - 1)), btn('primary', t('tour.finish'), () => close()));
    } else {
      const skip = btn('link', t('tour.skip'), () => close());
      skip.classList.add('tour-skip');
      row.append(skip, btn('ghost', t('tour.back'), () => go(idx - 1)));
      if (st.doIt && !st.wait()) row.append(btn('ghost', t('tour.doIt'), () => { st.doIt(); }));
      row.append(btn('primary', t('tour.next'), () => go(idx + 1)));
    }
    card.append(row);
    place(true);
  }

  /** 圈出目标, 卡片放在它旁边空得最多的一侧 (放不下就叠在目标里靠角) */
  function place(force) {
    const st = list[idx], el = st.target();
    const vw = window.innerWidth, vh = window.innerHeight, pad = 6, gap = 12;
    const cw = Math.min(360, vw - 24);
    const r = el && el.getClientRects().length ? el.getBoundingClientRect() : null;
    const key = [r ? r.left : -1, r ? r.top : -1, r ? r.width : 0, r ? r.height : 0, vw, vh].map(Math.round).join(',');
    if (!force && key === lastRect) return;                             // 目标没动: 不重排 (卡片高度只在 render 时变)
    lastRect = key;
    card.style.width = cw + 'px';
    const ch = card.offsetHeight;
    if (!r) {
      spot.style.cssText = `left:${vw / 2}px;top:${vh / 2}px;width:0;height:0`;
      card.style.left = Math.round((vw - cw) / 2) + 'px';
      card.style.top = Math.round(Math.max(12, (vh - ch) / 2)) + 'px';
      return;
    }
    const L = Math.max(4, r.left - pad), T = Math.max(4, r.top - pad);
    const W = Math.min(vw - 4, r.right + pad) - L, H = Math.min(vh - 4, r.bottom + pad) - T;
    spot.style.cssText = `left:${L}px;top:${T}px;width:${W}px;height:${H}px`;
    const room = { right: vw - (L + W) - gap - 4, left: L - gap - 4, bottom: vh - (T + H) - gap, top: T - gap };
    let x, y, w = cw, h = ch;
    const narrow = side => {                                            // 收窄到旁边那一栏的宽度, 重新量高度
      w = Math.floor(room[side]); card.style.width = w + 'px'; h = card.offsetHeight;
      return h <= vh - 24;
    };
    if (room.right >= cw) { x = L + W + gap; y = T; }
    else if (room.left >= cw) { x = L - gap - cw; y = T; }
    else if (room.bottom >= ch) { x = L; y = T + H + gap; }
    else if (room.top >= ch) { x = L; y = T - gap - ch; }
    else if (room.right >= 260 && room.right >= room.left && narrow('right')) { x = L + W + gap; y = T; }
    else if (room.left >= 260 && narrow('left')) { x = L - gap - w; y = T; }
    else {                                                              // 目标太大 (窄屏上的整张图): 叠在它右下角
      w = cw; card.style.width = cw + 'px'; h = card.offsetHeight;
      x = L + W - cw - 16; y = T + H - ch - 16;
    }
    x = Math.max(12, Math.min(vw - w - 12, x));
    y = Math.max(12, Math.min(vh - h - 12, y));
    card.style.left = Math.round(x) + 'px';
    card.style.top = Math.round(y) + 'px';
  }

  function tick() {
    raf = requestAnimationFrame(tick);
    if (idx < 0) return;
    const st = list[idx];
    place(false);
    const ok = !!(st.wait && st.wait());
    if (ok && !doneAt) {                                                // 访客做到了: 打勾, 稍停一下自动下一步
      doneAt = performance.now();
      render();
    } else if (!ok && doneAt) {                                         // 又改回去了: 重新等
      doneAt = 0; armed = true;
      render();
    }
    if (doneAt && armed && performance.now() - doneAt > 1100) { doneAt = 0; go(idx + 1); }
  }

  function go(i) {
    if (!list || i < 0 || i >= list.length) return;
    idx = i; doneAt = 0; lastRect = '';
    const st = list[i];
    if (st.tab) api.setTab(st.tab);
    if (st.enter) st.enter();
    // 进来时就已经做到 (访客退回来再看一遍): 只打勾, 不自动往下跳
    armed = !(st.wait && st.wait());
    if (!armed) doneAt = performance.now();
    const el = st.target();
    if (el && el.scrollIntoView) {
      const panel = el.closest('.panel');
      if (panel) el.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    }
    render();
  }

  function open(at = 0) {
    if (!root) build();
    list = steps(api);
    root.hidden = false;
    document.body.classList.add('touring');
    if (!raf) tick();
    go(at);
  }
  function close() {
    idx = -1;
    if (root) root.hidden = true;
    document.body.classList.remove('touring');
    cancelAnimationFrame(raf); raf = 0;
    S.tourSeen = true; api.save();
  }
  return { open, close, get active() { return idx >= 0; }, relocalize() { if (idx >= 0) render(); } };
}
