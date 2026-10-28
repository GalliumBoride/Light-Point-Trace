// 六棱柱预览: 面距 d3..d8 + 半高 h (与引擎同一套面号/法向约定: 面 f 法向方位 = BETA[f], 面距 (√3/2)·d).
// 单击晶面 → 回调 (加入光路); 拖动旋转. 当前首条光路的面着色: 入射绿 / 中间蓝 / 出射红.

import { t } from './i18n.js';
import { BETA } from '../engine/lp.js';

const RING = [3, 8, 7, 6, 5, 4];          // 逆时针棱序
const SQ32 = Math.sqrt(3) / 2;

export class CrystalPreview {
  constructor(canvas, onFace) {
    this.cv = canvas; this.onFace = onFace;
    this.yaw = -0.62; this.pitch = 0.40;
    this.dist = [1, 1, 1, 1, 1, 1]; this.h = 1; this.faces = [];
    this.hit = []; this.hover = 0;
    let drag = null;
    canvas.addEventListener('pointerdown', e => {
      drag = { x: e.clientX, y: e.clientY, sx: e.clientX, sy: e.clientY, moved: false };
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', e => {
      if (drag) {
        if (Math.hypot(e.clientX - drag.sx, e.clientY - drag.sy) > 4) drag.moved = true;
        if (drag.moved) {
          this.yaw -= (e.clientX - drag.x) * 0.012;
          this.pitch = Math.max(-1.45, Math.min(1.45, this.pitch + (e.clientY - drag.y) * 0.012));
          drag.x = e.clientX; drag.y = e.clientY;
          this.draw();
        }
        return;
      }
      const f = this.faceAt(e);
      if (f !== this.hover) { this.hover = f; this.draw(); }
      canvas.title = f ? t('crystal.tipFace', { f }) : t('crystal.tipDrag');
    });
    canvas.addEventListener('pointerleave', () => { if (this.hover) { this.hover = 0; this.draw(); } });
    const end = e => {
      if (drag && !drag.moved && e.type === 'pointerup') {
        const f = this.faceAt(e);
        if (f && this.onFace) this.onFace(f);
      }
      drag = null;
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
    new ResizeObserver(() => this.draw()).observe(canvas);
  }

  set(dist, h, faces) {
    faces = faces || [];
    // 光路换了: 把视角转到所用柱面的平均方向, 入射/出射面都朝向观者
    if (faces.join('-') !== this.faces.join('-')) {
      let x = 0, y = 0;
      for (const f of faces) if (BETA[f] !== undefined) { x += Math.cos(BETA[f]); y += Math.sin(BETA[f]); }
      if (Math.hypot(x, y) > 1e-6) this.yaw = Math.atan2(y, x);
    }
    this.dist = dist.slice(); this.h = h; this.faces = faces;
    this.draw();
  }

  faceAt(e) {
    const r = this.cv.getBoundingClientRect();
    const x = e.clientX - r.left, y = e.clientY - r.top;
    for (let i = this.hit.length - 1; i >= 0; i--) {          // 最前面的面先判
      const { f, pts } = this.hit[i];
      let inside = false;
      for (let a = 0, b = pts.length - 1; a < pts.length; b = a++) {
        const [xa, ya] = pts[a], [xb, yb] = pts[b];
        if ((ya > y) !== (yb > y) && x < (xb - xa) * (y - ya) / (yb - ya) + xa) inside = !inside;
      }
      if (inside) return f;
    }
    return 0;
  }

  geometry() {
    const n = {}, r = {};
    for (const f of RING) { n[f] = [Math.cos(BETA[f]), Math.sin(BETA[f])]; r[f] = SQ32 * this.dist[f - 3]; }
    const poly = [];
    for (let i = 0; i < 6; i++) {
      const a = RING[i], b = RING[(i + 1) % 6];
      const det = n[a][0] * n[b][1] - n[a][1] * n[b][0];
      poly.push([(r[a] * n[b][1] - r[b] * n[a][1]) / det, (n[a][0] * r[b] - n[b][0] * r[a]) / det]);
    }
    const H = this.h, faces = [];
    for (let i = 0; i < 6; i++) {
      const p = poly[(i + 5) % 6], q = poly[i], f = RING[i];
      faces.push({ f, n: [n[f][0], n[f][1], 0], pts: [[p[0], p[1], -H], [q[0], q[1], -H], [q[0], q[1], H], [p[0], p[1], H]] });
    }
    faces.push({ f: 1, n: [0, 0, 1], pts: poly.map(p => [p[0], p[1], H]) });
    faces.push({ f: 2, n: [0, 0, -1], pts: poly.slice().reverse().map(p => [p[0], p[1], -H]) });
    return faces;
  }

  draw() {
    const cv = this.cv;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth, Hc = cv.clientHeight;
    if (W < 10 || Hc < 10) return;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(Hc * dpr)) {
      cv.width = Math.round(W * dpr); cv.height = Math.round(Hc * dpr);
    }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, Hc);
    const st = getComputedStyle(cv);
    const C = k => st.getPropertyValue(k).trim();
    const ink = C('--ink') || '#222', muted = C('--ink-3') || '#999';
    const roleCol = { in: C('--in'), mid: C('--mid'), out: C('--out') };
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    const v = [cp * cy, cp * sy, sp];
    const right = [-sy, cy, 0], up = [-sp * cy, -sp * sy, cp];
    const faces = this.geometry();
    let ext = 0;
    for (const fc of faces) for (const p of fc.pts) ext = Math.max(ext, Math.hypot(p[0], p[1], p[2]));
    const sc = 0.40 * Math.min(W, Hc) / Math.max(ext, 1e-6);
    const P = p => [W / 2 + sc * (p[0] * right[0] + p[1] * right[1] + p[2] * right[2]),
                    Hc / 2 - sc * (p[0] * up[0] + p[1] * up[1] + p[2] * up[2])];
    const depth = fc => fc.pts.reduce((s, p) => s + p[0] * v[0] + p[1] * v[1] + p[2] * v[2], 0) / fc.pts.length;
    faces.sort((a, b) => depth(a) - depth(b));
    const used = this.faces, f0 = used[0], fE = used[used.length - 1];
    const roleOf = f => (!used.includes(f) ? null : f === f0 ? 'in' : f === fE ? 'out' : 'mid');
    this.hit = [];
    ctx.lineJoin = 'round';
    for (const fc of faces) {
      const front = fc.n[0] * v[0] + fc.n[1] * v[1] + fc.n[2] * v[2] > 1e-9;
      const pts = fc.pts.map(P);
      if (front) this.hit.push({ f: fc.f, pts });
      ctx.beginPath();
      pts.forEach((q, i) => (i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1])));
      ctx.closePath();
      const role = roleOf(fc.f);
      ctx.globalAlpha = 1;
      if (role) { ctx.fillStyle = roleCol[role]; ctx.globalAlpha = front ? 0.34 : 0.14; ctx.fill(); }
      else if (front) { ctx.fillStyle = ink; ctx.globalAlpha = this.hover === fc.f ? 0.13 : 0.04; ctx.fill(); }
      ctx.globalAlpha = 1;
      ctx.strokeStyle = front ? ink : muted; ctx.lineWidth = front ? 1.3 : 0.8;
      ctx.setLineDash(front ? [] : [3, 3]);
      if (!front) ctx.globalAlpha = 0.6;
      ctx.stroke();
      ctx.globalAlpha = 1;
      if (front) {
        const c = P(fc.pts.reduce((s, p) => [s[0] + p[0] / fc.pts.length, s[1] + p[1] / fc.pts.length, s[2] + p[2] / fc.pts.length], [0, 0, 0]));
        ctx.setLineDash([]);
        ctx.font = `${role || this.hover === fc.f ? '700 ' : ''}12px ui-monospace, Consolas, Menlo, monospace`;
        ctx.fillStyle = role ? roleCol[role] : ink; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(String(fc.f), c[0], c[1]);
      }
    }
    ctx.setLineDash([]);
  }
}
