// 六棱柱预览: 面距 d3..d8 + 半高 h (与引擎同一套面号/法向约定: 面 f 法向方位 = BETA[f], 面距 (√3/2)·d).
// 可切换成锥晶: 上下各接一段截头六棱锥 (锥面 13–18 / 23–28 在柱面 3–8 之上/下, 法向仰角 = 顶角/2, 同 faceCoef);
// 锥台高 hU / hL (真实晶体里的 h↑ / h↓), 形状直接取孔径引擎的半空间表 (crystalPlanes).
// 单击晶面 → 回调 (加入光路); 拖动旋转. 当前首条光路的面着色: 入射绿 / 中间蓝 / 出射红.

import { t } from './i18n.js';
import { BETA } from '../engine/lp.js';
import { crystalPlanes } from '../engine/aperture.js';

const RING = [3, 8, 7, 6, 5, 4];          // 逆时针棱序
const SQ32 = Math.sqrt(3) / 2;

/** 半空间表 → 各面多边形 (三面求交得顶点, 留在全部半空间里的; 每面的顶点绕外法向逆时针排), 竖直方向居中 */
function polyFaces(pl) {
  const F = pl.faces.length, V = [];
  for (let i = 0; i < F; i++)
    for (let j = i + 1; j < F; j++)
      for (let k = j + 1; k < F; k++) {
        const [a, b, c] = [pl.n[i], pl.n[j], pl.n[k]];
        const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
        if (Math.abs(det) < 1e-9) continue;
        const r = [pl.r[i], pl.r[j], pl.r[k]], col = (u, v, w) => u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0]);
        const cx = [r[0], a[1], a[2]], cy = [a[0], r[0], a[2]], cz = [a[0], a[1], r[0]];
        const bx = [r[1], b[1], b[2]], by = [b[0], r[1], b[2]], bz = [b[0], b[1], r[1]];
        const kx = [r[2], c[1], c[2]], ky = [c[0], r[2], c[2]], kz = [c[0], c[1], r[2]];
        const p = [col(cx, bx, kx) / det, col(cy, by, ky) / det, col(cz, bz, kz) / det];
        if (pl.n.some((m, f) => m[0] * p[0] + m[1] * p[1] + m[2] * p[2] > pl.r[f] + 1e-9)) continue;
        if (!V.some(q => Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]) < 1e-7)) V.push(p);
      }
  let zlo = Infinity, zhi = -Infinity;
  for (const p of V) { zlo = Math.min(zlo, p[2]); zhi = Math.max(zhi, p[2]); }
  const zc = (zlo + zhi) / 2, out = [];
  for (let i = 0; i < F; i++) {
    const nf = pl.n[i], vs = V.filter(p => Math.abs(nf[0] * p[0] + nf[1] * p[1] + nf[2] * p[2] - pl.r[i]) < 1e-7);
    if (vs.length < 3) continue;
    const cen = [0, 1, 2].map(a => vs.reduce((s, p) => s + p[a], 0) / vs.length);
    let u = [-nf[1], nf[0], 0];
    if (Math.hypot(...u) < 1e-6) u = [0, -nf[2], nf[1]];
    const ul = Math.hypot(...u); u = u.map(x => x / ul);
    const v = [nf[1] * u[2] - nf[2] * u[1], nf[2] * u[0] - nf[0] * u[2], nf[0] * u[1] - nf[1] * u[0]];
    const ang = p => Math.atan2((p[0] - cen[0]) * v[0] + (p[1] - cen[1]) * v[1] + (p[2] - cen[2]) * v[2],
                                (p[0] - cen[0]) * u[0] + (p[1] - cen[1]) * u[1] + (p[2] - cen[2]) * u[2]);
    vs.sort((p, q) => ang(p) - ang(q));
    out.push({ f: pl.faces[i], n: nf, pts: vs.map(p => [p[0], p[1], p[2] - zc]) });
  }
  return out;
}

export class CrystalPreview {
  constructor(canvas, onFace) {
    this.cv = canvas; this.onFace = onFace;
    this.yaw = -0.62; this.pitch = 0.40;
    this.dist = [1, 1, 1, 1, 1, 1]; this.h = 1; this.faces = [];
    this.pyr = false; this.apexUp = 56; this.apexLo = 56;
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

  set(dist, h, faces, hU = 0.5, hL = 0.5) {
    faces = faces || [];
    // 光路换了: 把视角转到所用柱面的平均方向, 入射/出射面都朝向观者
    if (faces.join('-') !== this.faces.join('-')) {
      let x = 0, y = 0;
      for (const f of faces) { const b = BETA[f] ?? BETA[f % 10]; if (b !== undefined) { x += Math.cos(b); y += Math.sin(b); } }
      if (Math.hypot(x, y) > 1e-6) this.yaw = Math.atan2(y, x);
    }
    this.dist = dist.slice(); this.h = h; this.faces = faces; this.hU = hU; this.hL = hL;
    this.draw();
  }

  /** 外形: 棱柱 / 锥晶 (上下锥顶角, 度). */
  setShape(pyr, apexUp, apexLo) {
    if (pyr === this.pyr && apexUp === this.apexUp && apexLo === this.apexLo) return;
    this.pyr = pyr; this.apexUp = apexUp; this.apexLo = apexLo;
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
    // 锥晶: 与孔径引擎同一张半空间表 (锥面过柱面上/下棱, 顶面/底面在锥台顶上), 顶点枚举出各面多边形 —
    // 图上的晶体就是算孔径用的那块; 锥台高 0 的一侧没有锥面, 超过锥顶高时顶面消失 (尖顶)
    if (this.pyr) return polyFaces(crystalPlanes(this.dist, this.h, true, this.hU ?? 0.5, this.hL ?? 0.5, this.apexUp, this.apexLo));
    const n = {}, r = {};
    for (const f of RING) { n[f] = [Math.cos(BETA[f]), Math.sin(BETA[f])]; r[f] = SQ32 * this.dist[f - 3]; }
    const hexAt = s => {                         // 六个面距都缩进 s 后的截面 (s = 0 即柱身)
      const poly = [];
      for (let i = 0; i < 6; i++) {
        const a = RING[i], b = RING[(i + 1) % 6], ra = r[a] - s, rb = r[b] - s;
        const det = n[a][0] * n[b][1] - n[a][1] * n[b][0];
        poly.push([(ra * n[b][1] - rb * n[a][1]) / det, (n[a][0] * rb - n[b][0] * ra) / det]);
      }
      return poly;
    };
    const poly = hexAt(0);
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
