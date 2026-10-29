// 绘图: 天球视图 / 取向平面. 点云走软件点栅 (同原窗体 Raster 的整数混色), 线与标记走 Canvas 2D.
// 坐标一律用 CSS 像素; 点栅按 devicePixelRatio 放大. 白底 + 原窗体的 matplotlib 配色.
// 标题与图例只画进导出的 PNG (屏幕上由界面承担).

import { LP, haloDeg } from '../engine/lp.js';
import { t, L } from './i18n.js';
import { axisLabel } from './terms.js';

// ---------- 配色 ----------
const PALETTE_PAPER = [
  [220, 20, 60], [65, 105, 225], [255, 140, 0], [139, 69, 19], [0, 100, 0], [255, 0, 255],
  [31, 119, 180], [148, 103, 189], [227, 119, 194], [23, 190, 207], [188, 189, 34], [127, 127, 127], [0, 0, 0]];
const PATHCOL_PAPER = [[214, 39, 40], [31, 119, 180], [44, 160, 44], [148, 103, 189], [227, 119, 194], [140, 86, 75]];

export const PLOT_THEMES = {
  paper: {
    bg: [255, 255, 255], palette: PALETTE_PAPER, pathcol: PATHCOL_PAPER,
    mc: [158, 158, 158], mcAlpha: 0.16, sideGray: [102, 102, 102], mirrorMc: [70, 130, 200],
    frame: '#000', horizon: 'rgb(46,139,87)', circle: 'rgb(115,115,115)', mark: '#000', ink: '#000',
    grid: 'rgba(0,0,0,0.20)', seam: 'rgba(0,0,0,0.55)', tick: '#333', rim: '#000',
    probe: '#e01b24', hover: '#000', legendBg: 'rgba(255,255,255,0.92)', legendLine: 'rgba(0,0,0,0.35)',
  },
};

export const css = (c, a = 1) => a >= 1 ? `rgb(${c[0]},${c[1]},${c[2]})` : `rgba(${c[0]},${c[1]},${c[2]},${a})`;

export const lensNames = () => L().lenses;     // 六种投影的名字 (随语言)
const LENS_TAGS = ['linear', 'fisheye_equalarea', 'fisheye_equidistant', 'fisheye_stereographic', 'fisheye_orthographic', 'equirectangular'];
export function fovMax(lens) {
  switch (lens) { case 0: return 170; case 3: return 355; case 4: return 180; default: return 360; }
}

const FONT = '"PingFang SC", "Microsoft YaHei UI", "Microsoft YaHei", "Noto Sans SC", system-ui, sans-serif';

// ============================================================ 软件点栅
class Raster {
  constructor() { this.W = 0; this.H = 0; this.s = 1; this.img = null; this.buf = null; }
  resize(ctx, W, H, s) {
    if (W !== this.W || H !== this.H || !this.img) {
      this.img = ctx.createImageData(W, H);
      this.buf = new Uint32Array(this.img.data.buffer);
      this.W = W; this.H = H;
    }
    this.s = s;
  }
  clear(c) { this.buf.fill((0xFF000000 | (c[2] << 16) | (c[1] << 8) | c[0]) >>> 0); }
  // 定点整数混色 (sa: 0..256); x,y,size 为 CSS 像素
  dot(x, y, size, c, a) {
    let sa = (a * 256 + 0.5) | 0;
    if (sa <= 0) return; if (sa > 256) sa = 256;
    const s = this.s, W = this.W, H = this.H;
    const sz = Math.max(1, Math.round(size * s));
    let x0 = Math.round(x * s) - ((sz - 1) >> 1), y0 = Math.round(y * s) - ((sz - 1) >> 1);
    let x1 = x0 + sz - 1, y1 = y0 + sz - 1;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x1 >= W) x1 = W - 1; if (y1 >= H) y1 = H - 1;
    if (x1 < x0 || y1 < y0) return;
    const sr = c[0] * sa, sg = c[1] * sa, sb = c[2] * sa, ia = 256 - sa, buf = this.buf;
    for (let yy = y0; yy <= y1; yy++) {
      const base = yy * W;
      for (let xx = x0; xx <= x1; xx++) {
        const d = buf[base + xx];
        const r = (sr + (d & 255) * ia + 128) >> 8;
        const g = (sg + ((d >> 8) & 255) * ia + 128) >> 8;
        const b = (sb + ((d >> 16) & 255) * ia + 128) >> 8;
        buf[base + xx] = 0xFF000000 | (b << 16) | (g << 8) | r;
      }
    }
  }
  box(x, y, half, c, a) { this.dot(x, y, 2 * half + 1, c, a); }
}

// ============================================================ 天球镜头 (与原窗体 / PreviewGLRenderer shader 一致)
export class SkyProj {
  set(az, el, fov, lens, W, H) {
    this.lens = lens;
    // az 为配置方位角 (原窗体约定, 增角=顺时针俯视); 世界系 atan2(y,x) 逆时针, 故取负号换算
    const a = -az * Math.PI / 180, e = el * Math.PI / 180;
    const ce = Math.cos(e), se = Math.sin(e), ca = Math.cos(a), sa = Math.sin(a);
    const ax = [ce * ca, ce * sa, se];
    // 画面"上"方向 = 视线对仰角的导数. 在天顶/天底也随方位连续变化;
    // 原窗体在极点退回一个固定方向, 视角一到 ±90° 就被扭回方位 0, 且拖动方位不再转动画面
    const up = [-se * ca, -se * sa, ce];
    this.ax = ax; this.up = up;
    this.rt = [ax[1] * up[2] - ax[2] * up[1], ax[2] * up[0] - ax[0] * up[2], ax[0] * up[1] - ax[1] * up[0]];
    this.hf = Math.max(0.5, fov) * Math.PI / 360.0;
    this.cosHf = Math.cos(Math.min(this.hf, Math.PI));
    this.azRad = -az * Math.PI / 180.0;
    const hfLin = Math.min(this.hf, 85.0 * Math.PI / 180);
    this.linFocal = 0.5 * Math.sqrt(W * W + H * H) / Math.tan(hfLin);
    this.eqScale = Math.min(W / 2.0, H) / Math.PI;
    this.cx = W / 2; this.cy = H / 2;
    this.rpx = Math.min(W, H) / 2 - 36;
  }

  /** 天球方向 → 屏幕 (this.x, this.y); 不可见返回 false */
  project(Dx, Dy, Dz) {
    const cx = this.cx, cy = this.cy, rpx = this.rpx, lens = this.lens;
    if (lens === 5) {
      let lon = Math.atan2(Dy, Dx) - this.azRad;
      lon -= 2 * Math.PI * Math.floor((lon + Math.PI) / (2 * Math.PI));
      const lat = Math.asin(Dz > 1 ? 1 : (Dz < -1 ? -1 : Dz));
      this.x = cx + lon * this.eqScale; this.y = cy - lat * this.eqScale;
      return true;
    }
    const ax = this.ax;
    let cc = Dx * ax[0] + Dy * ax[1] + Dz * ax[2];
    if (cc > 1) cc = 1; else if (cc < -1) cc = -1;
    if (lens === 0) { if (cc < 0.0087) return false; }
    else if (cc < this.cosHf) return false;
    const px = Dx - cc * ax[0], py = Dy - cc * ax[1], pz = Dz - cc * ax[2];
    let pn = Math.sqrt(px * px + py * py + pz * pz);
    if (pn < 1e-12) pn = 1;
    const rt = this.rt, up = this.up;
    const tr = (px * rt[0] + py * rt[1] + pz * rt[2]) / pn;
    const tu = (px * up[0] + py * up[1] + pz * up[2]) / pn;
    const th = Math.acos(cc), hf = this.hf;
    let r;
    switch (lens) {
      case 0: r = this.linFocal * Math.tan(th); break;
      case 1: r = rpx * Math.sin(th / 2) / Math.max(1e-9, Math.sin(hf / 2)); break;
      case 3: r = rpx * Math.tan(th / 2) / Math.max(1e-9, Math.tan(hf / 2)); break;
      case 4:
        if (th > Math.PI / 2) return false;
        r = rpx * Math.sin(th) / Math.max(1e-9, Math.sin(Math.min(hf, Math.PI / 2)));
        break;
      default: r = rpx * th / hf;
    }
    this.x = cx + r * tr; this.y = cy - r * tu;
    return true;
  }

  /** 屏幕 → 天球方向 (单位向量); 画面外返回 null. 用于鼠标处读数 */
  unproject(x, y) {
    const lens = this.lens;
    if (lens === 5) {
      const lon = (x - this.cx) / this.eqScale, lat = (this.cy - y) / this.eqScale;
      if (Math.abs(lon) > Math.PI || Math.abs(lat) > Math.PI / 2) return null;
      const wl = lon + this.azRad;
      return [Math.cos(lat) * Math.cos(wl), Math.cos(lat) * Math.sin(wl), Math.sin(lat)];
    }
    const dx = x - this.cx, dy = this.cy - y, r = Math.hypot(dx, dy), hf = this.hf, rpx = this.rpx;
    let th;
    switch (lens) {
      case 0: th = Math.atan(r / this.linFocal); break;
      case 1: { const s = r * Math.sin(hf / 2) / rpx; if (s > 1) return null; th = 2 * Math.asin(s); break; }
      case 3: th = 2 * Math.atan(r * Math.tan(hf / 2) / rpx); break;
      case 4: { const s = r * Math.sin(Math.min(hf, Math.PI / 2)) / rpx; if (s > 1) return null; th = Math.asin(s); break; }
      default: th = r * hf / rpx;
    }
    if (lens !== 0 && th > hf + 1e-9) return null;
    if (th > Math.PI) return null;
    const tr = r > 0 ? dx / r : 0, tu = r > 0 ? dy / r : 0;
    const s = Math.sin(th), c = Math.cos(th), ax = this.ax, rt = this.rt, up = this.up;
    return [c * ax[0] + s * (tr * rt[0] + tu * up[0]), c * ax[1] + s * (tr * rt[1] + tu * up[1]), c * ax[2] + s * (tr * rt[2] + tu * up[2])];
  }

  // 相邻两点是否连线: ≤90° 且等距圆柱下不跨 ±180° 接缝
  connect(ax_, ay, az, bx, by, bz) {
    if (ax_ * bx + ay * by + az * bz < 0) return false;
    if (this.lens === 5) {
      let la = Math.atan2(ay, ax_) - this.azRad, lb = Math.atan2(by, bx) - this.azRad;
      la -= 2 * Math.PI * Math.floor((la + Math.PI) / (2 * Math.PI));
      lb -= 2 * Math.PI * Math.floor((lb + Math.PI) / (2 * Math.PI));
      if (Math.abs(la - lb) > Math.PI) return false;
    }
    return true;
  }
}

// 周期环绕: 求 v + 360k 落入 [lo, lo+w] 的全部副本 (≤3)
function wrapC(v, lo, w, out) {
  let n = 0;
  const s = v + 360.0 * Math.ceil((lo - v) / 360.0);
  for (let x = s; x <= lo + w + 1e-9 && n < 3; x += 360.0) out[n++] = x;
  return n;
}

export function oriArea(W, H) {
  // 上留浮动视图切换, 下留刻度 + 轴名 + 浮动镜头条
  return { X: 58, Y: 58, W: Math.max(10, W - 82), H: Math.max(10, H - 136) };
}

/** 天空图族 (Trace2D 的键) 用哪个颜色: 与取向平面同名族对齐, 让点、线、列表三处颜色一致 */
export function skyColorIndex(pr, key, fi) {
  if (!pr.ori) return fi;
  const base = key.replace(/^\S+-帽 /, '');
  const i = pr.ori.fams.findIndex(f => f.name === base);
  return i >= 0 ? i : fi;
}

// ============================================================ 渲染器
export class Renderer {
  constructor(canvas) {
    this.cv = canvas;
    this.ctx = canvas.getContext('2d');
    this.ras = new Raster();
    this.proj = new SkyProj();
    this._wx = [0, 0, 0]; this._wy = [0, 0, 0];
  }

  /**
   * V: { W, H, dpr, theme, view, lens, az, el, fov, xw, yw, xc, yc, mc, arcs, lines, endpoints, sym,
   *      lineW, mcSize, dragging, checked:Set, probe, hoverEp, chrome, title, famTitle(pr,fm), skyTitle(pr,key) }
   */
  render(V, R) {
    const { W, H, dpr } = V;
    const dw = Math.max(1, Math.round(W * dpr)), dh = Math.max(1, Math.round(H * dpr));
    if (this.cv.width !== dw || this.cv.height !== dh) { this.cv.width = dw; this.cv.height = dh; }
    const ctx = this.ctx;
    this.T = V.theme;
    this.ras.resize(ctx, dw, dh, dpr);
    this.ras.clear(this.T.bg);
    this.V = V; this.R = R;
    if (W < 60 || H < 60) return;
    if (V.view === 0) this.rasterSky(); else this.rasterOri();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.putImageData(this.ras.img, 0, 0);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.lineJoin = 'round';
    if (V.view === 0) {
      if (V.lines) this.drawFamilyLines();
      this.drawSkyDecor();
      this.drawEndpointsSky();
    } else {
      if (V.lines) this.drawOriLines();
      this.drawOriDecor();
      this.drawEndpointsOri();
    }
    if (V.chrome) {
      this.drawTitle(V.title || '');
      this.drawLegend(V.view === 0);
    }
  }

  get multi() { return this.R && this.R.paths.length > 1; }
  /** 「左右镜像补全」时这条光路的镜像光路 (取向平面里另画一份, 点云用蓝色区分); 没有就是 null. */
  mirrorOf(pr) {
    const m = pr.mirror;
    return this.V.sym && m && m.status === 'ok' && !m.dup && m.ori ? m : null;
  }
  famLabel(pr, fm) { return this.multi ? pr.name + '·' + fm.name : fm.name; }
  pal(i) { return this.T.palette[i % this.T.palette.length]; }
  pcol(k) { return this.T.pathcol[k % this.T.pathcol.length]; }

  // ---------------------------------------------------------- 天球
  plotSky(Dx, Dy, Dz, size, col, alpha) {
    const p = this.proj, ras = this.ras;
    if (p.project(Dx, Dy, Dz)) ras.dot(p.x, p.y, size, col, alpha);
    if (this.V.sym && p.project(Dx, -Dy, Dz)) ras.dot(p.x, p.y, size, col, alpha);
  }

  rasterSky() {
    const V = this.V, R = this.R, T = this.T;
    this.proj.set(V.az, V.el, V.fov, V.lens, V.W, V.H);
    if (!R) return;
    const multi = this.multi;
    const mcCap = V.dragging ? 40000 : 150000;
    if (V.mc && V.arcs)
      R.paths.forEach((pr, k) => {
        const mc = pr.mc; if (!mc) return;
        const n = mc.length / 3, step = Math.max(1, Math.floor(n / mcCap));
        const col = multi ? this.pcol(k) : T.mc, alpha = multi ? T.mcAlpha * 0.8 : T.mcAlpha;
        for (let i = 0; i < n; i += step) this.plotSky(mc[3 * i], mc[3 * i + 1], mc[3 * i + 2], V.mcSize, col, alpha);
      });
    this.drawInspectorSky();
    if (V.arcs)
      R.paths.forEach((pr, k) => {
        if (!pr.sky) return;
        pr.sky.forEach((s, fi) => {
          const col = multi ? this.pcol(k) : this.pal(skyColorIndex(pr, s.name, fi));
          const a = s.pts;
          for (let i = 0; i < a.length; i += 3) this.plotSky(a[i], a[i + 1], a[i + 2], 2, col, 0.95);
        });
      });
  }

  drawInspectorSky() {
    const V = this.V, R = this.R;
    if (V.checked.size === 0) return;
    for (const pr of R.paths) {
      if (!pr.ori) continue;
      pr.ori.fams.forEach((fm, fi) => {
        if (!V.checked.has(this.famLabel(pr, fm))) return;
        const col = this.pal(fi);
        if (fm.sense !== 0 && V.mc) {
          const cloud = R.clouds.get(pr.name + '|' + fm.name);
          if (cloud instanceof Float32Array) {
            const n = cloud.length / 3, stp = Math.max(1, Math.floor(n / (V.dragging ? 20000 : 60000)));
            for (let i = 0; i < n; i += stp) this.plotSky(cloud[3 * i], cloud[3 * i + 1], cloud[3 * i + 2], V.mcSize, col, 0.16);
          } else if (cloud === undefined && R.requestCloud) R.requestCloud(pr, fm);
        }
        const rt = pr.ori.roots[fi], D = rt.D, ok = rt.ok;
        for (let i = 0; i < ok.length; i++)
          if (ok[i] || V.hlFull) this.plotSky(D[3 * i], D[3 * i + 1], D[3 * i + 2], ok[i] ? 3 : 2, col, ok[i] ? 1.0 : 0.30);
      });
    }
  }

  drawFamilyLines() {
    const V = this.V, R = this.R;
    if (!R) return;
    const multi = this.multi;
    R.paths.forEach((pr, k) => {
      if (pr.ori) {
        if (!pr.lines) return;
        pr.ori.fams.forEach((fm, fi) => {
          const full = V.checked.has(this.famLabel(pr, fm));
          if (!full && !V.arcs) return;
          const fl = pr.lines.get(fm.name); if (!fl) return;
          this.drawLineSet(fl.sky, multi ? this.pcol(k) : this.pal(fi), full && V.hlFull);
        });
      } else if (pr.sky && V.arcs) {
        pr.sky.forEach((s, fi) => {
          const col = multi ? this.pcol(k) : this.pal(fi);
          const a = s.pts, n = a.length / 3, flat = new Float32Array(n * 4);
          for (let i = 0; i < n; i++) { flat[4 * i] = a[3 * i]; flat[4 * i + 1] = a[3 * i + 1]; flat[4 * i + 2] = a[3 * i + 2]; flat[4 * i + 3] = 1; }
          this.drawLineSet([flat], col, true);
        });
      }
    });
  }

  drawLineSet(lines, col, full) {
    const V = this.V, p = this.proj, ctx = this.ctx;
    const okPath = new Path2D(), noPath = new Path2D();
    const stepL = 1;                 // 拖动时也逐点连 (抽稀会让短线段、尖端的补线一闪一闪地消失)
    for (let mirror = 0; mirror < (V.sym ? 2 : 1); mirror++) {
      const ySign = mirror === 0 ? 1 : -1;
      for (const flat of lines) {
        const n = flat.length / 4;
        let hasPrev = false, prevOk = false, px = 0, py = 0, pX = 0, pY = 0, pZ = 0;
        let lastOk = null, lastNo = null;
        for (let i = 0; i < n; i += stepL) {
          const X = flat[4 * i], Y = ySign * flat[4 * i + 1], Z = flat[4 * i + 2];
          const ok = p.project(X, Y, Z);
          const sx = p.x, sy = p.y;
          const valid = flat[4 * i + 3] > 0.5;
          if (ok && hasPrev && p.connect(pX, pY, pZ, X, Y, Z)) {
            if (valid && prevOk) {
              if (lastOk !== i - stepL) okPath.moveTo(px, py);
              okPath.lineTo(sx, sy); lastOk = i;
            } else if (full) {
              if (lastNo !== i - stepL) noPath.moveTo(px, py);
              noPath.lineTo(sx, sy); lastNo = i;
            }
          }
          hasPrev = ok; px = sx; py = sy; prevOk = valid; pX = X; pY = Y; pZ = Z;
        }
      }
    }
    ctx.lineCap = 'round';
    if (full) { ctx.strokeStyle = css(col, 0.32); ctx.lineWidth = Math.max(1, V.lineW * 0.6); ctx.stroke(noPath); }
    ctx.strokeStyle = css(col); ctx.lineWidth = V.lineW; ctx.stroke(okPath);
  }

  drawCircle3D(gen, stroke, width, dash) {
    const ctx = this.ctx, p = this.proj;
    ctx.save();
    ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.lineCap = 'butt';
    ctx.beginPath();
    let run = 0, lastX = 0, lastY = 0;
    for (let i = 0; i <= 360; i++) {
      const t = 2 * Math.PI * i / 360;
      const d = gen(t);
      const ok = p.project(d[0], d[1], d[2]);
      if (!ok || (run > 0 && (Math.abs(p.x - lastX) > 90 || Math.abs(p.y - lastY) > 90))) {
        run = 0;
        if (!ok) continue;
      }
      if (run === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      run++; lastX = p.x; lastY = p.y;
    }
    ctx.stroke();
    ctx.restore();
  }

  drawMark(D, kind) {
    const ctx = this.ctx, p = this.proj, T = this.T;
    if (!p.project(D[0], D[1], D[2])) return;
    const sx = p.x, sy = p.y;
    ctx.save();
    if (kind === 'sun') {
      ctx.beginPath(); ctx.arc(sx, sy, 10, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(255,190,60,0.22)'; ctx.fill();
      ctx.beginPath(); ctx.arc(sx, sy, 5.5, 0, 2 * Math.PI);
      ctx.fillStyle = '#ffb21e'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = T.rim; ctx.stroke();
    } else if (kind === 'star') {
      ctx.fillStyle = T.mark; ctx.font = `14px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('★', sx, sy + 1);
    } else {
      ctx.strokeStyle = T.mark; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(sx - 6, sy); ctx.lineTo(sx + 6, sy); ctx.moveTo(sx, sy - 6); ctx.lineTo(sx, sy + 6); ctx.stroke();
    }
    ctx.restore();
  }

  drawSkyDecor() {
    const V = this.V, ctx = this.ctx, p = this.proj, T = this.T;
    const W = V.W, H = V.H, cx = p.cx, cy = p.cy, rpx = p.rpx;
    ctx.save();
    ctx.strokeStyle = T.frame; ctx.lineWidth = 1.3; ctx.setLineDash([]);
    ctx.beginPath();
    if (V.lens === 0) ctx.rect(0.5, 0.5, W - 1, H - 1);
    else if (V.lens === 5) { const mw = Math.PI * p.eqScale, mh = Math.PI / 2 * p.eqScale; ctx.rect(cx - mw, cy - mh, 2 * mw, 2 * mh); }
    else ctx.arc(cx, cy, Math.max(1, rpx), 0, 2 * Math.PI);
    ctx.stroke();
    ctx.restore();
    const TH = LP.TH, A3 = LP.A3;
    this.drawCircle3D(t => [Math.cos(t), Math.sin(t), 0.0], T.horizon, 1.5);
    this.drawCircle3D(t => [Math.cos(TH) * Math.cos(t), Math.cos(TH) * Math.sin(t), Math.sin(TH)], T.circle, 1.1, [1.2, 2.6]);
    const r22 = haloDeg() * Math.PI / 180, c22 = Math.cos(r22), s22 = Math.sin(r22);
    const b1 = [0, 1, 0], b2 = [-A3[2], 0, A3[0]];
    if (Number.isFinite(r22)) this.drawCircle3D(t => [
      c22 * A3[0] + s22 * (Math.cos(t) * b1[0] + Math.sin(t) * b2[0]),
      c22 * A3[1] + s22 * (Math.cos(t) * b1[1] + Math.sin(t) * b2[1]),
      c22 * A3[2] + s22 * (Math.cos(t) * b1[2] + Math.sin(t) * b2[2])], T.circle, 1.0, [4.4, 2.4]);
    this.drawMark(A3, 'sun');
    this.drawMark([-A3[0], 0, A3[2]], 'star');
    this.drawMark([0, 0, 1], 'plus');
  }

  drawTitle(s) {
    const ctx = this.ctx;
    ctx.save();
    ctx.font = `600 13px ${FONT}`; ctx.fillStyle = this.T.ink; ctx.textBaseline = 'top';
    ctx.fillText(s, 12, 12);
    ctx.restore();
  }

  // ---------- 末端点标记 ----------
  drawEpGlyph(sx, sy, kind, rim) { drawEpGlyphOn(this.ctx, sx, sy, kind, rim); }

  drawHoverRing(sx, sy) {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = this.T.hover; ctx.lineWidth = 1.5; ctx.setLineDash([3, 2.5]);
    ctx.beginPath(); ctx.arc(sx, sy, 12, 0, 2 * Math.PI); ctx.stroke();
    ctx.restore();
  }

  drawEndpointsSky() {
    const V = this.V, R = this.R, p = this.proj;
    if (!R) return;
    const multi = this.multi;
    R.paths.forEach((pr, k) => {
      if (!pr.eps) return;
      const rim = multi ? css(this.pcol(k)) : this.T.rim;
      pr.eps.forEach((r, i) => {
        if (!r.alive) return;
        const hov = V.hoverEp && V.hoverEp.k === k && V.hoverEp.i === i;
        if (!V.endpoints && !hov) return;
        for (let mir = 0; mir < (V.sym ? 2 : 1); mir++) {
          if (!p.project(r.D[0], (mir === 0 ? 1 : -1) * r.D[1], r.D[2])) continue;
          this.drawEpGlyph(p.x, p.y, r.kind, rim);
          if (hov && mir === 0) this.drawHoverRing(p.x, p.y);
        }
      });
    });
  }

  // ---------------------------------------------------------- 取向平面
  rasterOri() {
    const V = this.V, R = this.R, ras = this.ras, T = this.T;
    if (!R) return;
    const area = oriArea(V.W, V.H);
    const xlo = V.xc - V.xw / 2, ylo = V.yc - V.yw / 2;
    const PX = xd => area.X + (xd - xlo) / V.xw * area.W;
    const PY = yd => area.Y + area.H - (yd - ylo) / V.yw * area.H;
    const multi = this.multi, wx = this._wx, wy = this._wy;

    if (V.checked.size > 0 && V.mc)
      for (const pr of R.paths) {
        if (!pr.ori) continue;
        pr.ori.fams.forEach((fm, fi) => {
          if (!V.checked.has(this.famLabel(pr, fm)) || fm.sense === 0) return;
          const col = this.pal(fi);
          const side = pr.ori.side[fi], SC = pr.ori.sideSC, SW = pr.ori.sideSW;
          const nS = SC.length;
          for (let a = 0; a < SW.length; a++) {
            const ny = wrapC(SW[a], ylo, V.yw, wy);
            if (ny === 0) continue;
            for (let b = 0; b < nS; b++) {
              const nx = wrapC(SC[b], xlo, V.xw, wx);
              if (nx === 0) continue;
              const sat = side[a * nS + b] === 1;
              const cc = sat ? col : T.sideGray, aa = sat ? 0.10 : 0.08;
              for (let a1 = 0; a1 < nx; a1++)
                for (let b1 = 0; b1 < ny; b1++) ras.box(PX(wx[a1]), PY(wy[b1]), 3, cc, aa);
            }
          }
        });
      }

    if (V.mc && V.arcs)
      R.paths.forEach((pr, k) => {
        if (!pr.ori) return;
        const col = multi ? this.pcol(k) : T.mc, alpha = multi ? T.mcAlpha * 0.8 : T.mcAlpha;
        const X = pr.ori.mcX, Y = pr.ori.mcY;
        const step = V.dragging ? Math.max(1, Math.floor(X.length / 40000)) : 1;
        for (let i = 0; i < X.length; i += step) {
          const nx = wrapC(X[i], xlo, V.xw, wx); if (nx === 0) continue;
          const ny = wrapC(Y[i], ylo, V.yw, wy); if (ny === 0) continue;
          for (let a1 = 0; a1 < nx; a1++)
            for (let b1 = 0; b1 < ny; b1++) ras.dot(PX(wx[a1]), PY(wy[b1]), V.mcSize, col, alpha);
        }
      });

    if (V.mc && V.arcs)
      for (const pr of R.paths) {
        const m = this.mirrorOf(pr);
        if (!m) continue;
        const X = m.ori.mcX, Y = m.ori.mcY;
        const step = V.dragging ? Math.max(1, Math.floor(X.length / 40000)) : 1;
        for (let i = 0; i < X.length; i += step) {
          const nx = wrapC(X[i], xlo, V.xw, wx); if (nx === 0) continue;
          const ny = wrapC(Y[i], ylo, V.yw, wy); if (ny === 0) continue;
          for (let a1 = 0; a1 < nx; a1++)
            for (let b1 = 0; b1 < ny; b1++) ras.dot(PX(wx[a1]), PY(wy[b1]), V.mcSize, T.mirrorMc, T.mcAlpha);
        }
      }

    for (const pr of R.paths) {                  // 镜像光路的根点: 只画成立的那些
      const m = this.mirrorOf(pr);
      if (!m || !V.arcs) continue;
      m.ori.fams.forEach((fm, fi) => {
        const col = this.multi ? this.pcol(R.paths.indexOf(pr)) : this.pal(fi);
        const rt = m.ori.roots[fi], sc = rt.sc, sw = rt.sw, ok = rt.ok;
        for (let i = 0; i < sc.length; i++) {
          if (!ok[i]) continue;
          const nx = wrapC(sc[i], xlo, V.xw, wx); if (nx === 0) continue;
          const ny = wrapC(sw[i], ylo, V.yw, wy); if (ny === 0) continue;
          for (let a1 = 0; a1 < nx; a1++)
            for (let b1 = 0; b1 < ny; b1++) ras.dot(PX(wx[a1]), PY(wy[b1]), 2, col, 0.45);
        }
      });
    }

    R.paths.forEach((pr, k) => {
      if (!pr.ori) return;
      const srcs = [pr.ori];
      if (k === 0 && R.oriLocal) srcs.push(R.oriLocal);
      for (const src of srcs)
        src.fams.forEach((fm, fi) => {
          const hot = V.checked.has(this.famLabel(pr, fm));
          if (!V.arcs && !hot) return;
          const col = multi ? this.pcol(k) : this.pal(fi);
          const rt = src.roots[fi], sc = rt.sc, sw = rt.sw, ok = rt.ok;
          for (let i = 0; i < sc.length; i++) {
            const nx = wrapC(sc[i], xlo, V.xw, wx); if (nx === 0) continue;
            const ny = wrapC(sw[i], ylo, V.yw, wy); if (ny === 0) continue;
            for (let a1 = 0; a1 < nx; a1++)
              for (let b1 = 0; b1 < ny; b1++) {
                if (ok[i]) ras.dot(PX(wx[a1]), PY(wy[b1]), hot ? 3 : 2, col, 0.95);
                else ras.dot(PX(wx[a1]), PY(wy[b1]), hot && V.hlFull ? 2 : 1, col, hot && V.hlFull ? 0.40 : 0.12);
              }
          }
        });
    });
  }

  drawOriLines() {
    const V = this.V, R = this.R, ctx = this.ctx;
    if (!R) return;
    const area = oriArea(V.W, V.H);
    const xlo = V.xc - V.xw / 2, xhi = V.xc + V.xw / 2, ylo = V.yc - V.yw / 2, yhi = V.yc + V.yw / 2;
    const PX = xd => area.X + (xd - xlo) / V.xw * area.W;
    const PY = yd => area.Y + area.H - (yd - ylo) / V.yw * area.H;
    const multi = this.multi;
    ctx.save();
    ctx.beginPath(); ctx.rect(area.X, area.Y, area.W, area.H); ctx.clip();
    ctx.lineCap = 'round';
    const srcs = [];
    R.paths.forEach((pr, k) => {
      if (pr.ori && pr.lines) srcs.push({ pr, k, fams: pr.ori.fams, lines: pr.lines, mirror: false });
      const m = this.mirrorOf(pr);
      if (m && m.linesMap) srcs.push({ pr, k, fams: m.ori.fams, lines: m.linesMap, mirror: true });
    });
    for (const { pr, k, fams, lines, mirror } of srcs)
      fams.forEach((fm, fi) => {
        const hot = !mirror && V.checked.has(this.famLabel(pr, fm)), full = hot && V.hlFull;
        if (!hot && !V.arcs) return;
        const fl = lines.get(fm.name); if (!fl) return;
        const col = multi ? this.pcol(k) : this.pal(fi);
        const okPath = new Path2D(), noPath = new Path2D();
        for (let li = 0; li < fl.par.length; li++) {
          const par = fl.par[li], sky = fl.sky[li], n = par.length / 2;
          if (n < 2) continue;
          const xs = new Float64Array(n), ys = new Float64Array(n);
          let mnx = Infinity, mxx = -Infinity, mny = Infinity, mxy = -Infinity;
          for (let i = 0; i < n; i++) {
            xs[i] = par[2 * i] * 180 / Math.PI; ys[i] = par[2 * i + 1] * 180 / Math.PI;
            if (xs[i] < mnx) mnx = xs[i]; if (xs[i] > mxx) mxx = xs[i];
            if (ys[i] < mny) mny = ys[i]; if (ys[i] > mxy) mxy = ys[i];
          }
          const kx0 = Math.floor((xlo - mxx) / 360), kx1 = Math.ceil((xhi - mnx) / 360);
          const ky0 = Math.floor((ylo - mxy) / 360), ky1 = Math.ceil((yhi - mny) / 360);
          for (let kx = kx0; kx <= kx1; kx++)
            for (let ky = ky0; ky <= ky1; ky++) {
              const ox = kx * 360.0, oy = ky * 360.0;
              if (mxx + ox < xlo || mnx + ox > xhi || mxy + oy < ylo || mny + oy > yhi) continue;
              let lastOk = -2, lastNo = -2;
              for (let i = 0; i + 1 < n; i++) {
                const ok = sky[4 * i + 3] > 0.5 && sky[4 * (i + 1) + 3] > 0.5;
                if (!ok && !full) continue;
                const path = ok ? okPath : noPath;
                if ((ok ? lastOk : lastNo) !== i - 1) path.moveTo(PX(xs[i] + ox), PY(ys[i] + oy));
                path.lineTo(PX(xs[i + 1] + ox), PY(ys[i + 1] + oy));
                if (ok) lastOk = i; else lastNo = i;
              }
            }
        }
        if (full) { ctx.strokeStyle = css(col, 0.32); ctx.lineWidth = Math.max(1, V.lineW * 0.6); ctx.stroke(noPath); }
        ctx.strokeStyle = css(col); ctx.lineWidth = V.lineW; ctx.stroke(okPath);
      });
    ctx.restore();
  }

  drawEndpointsOri() {
    const V = this.V, R = this.R, ctx = this.ctx;
    if (!R) return;
    const area = oriArea(V.W, V.H);
    const xlo = V.xc - V.xw / 2, ylo = V.yc - V.yw / 2;
    const PX = xd => area.X + (xd - xlo) / V.xw * area.W;
    const PY = yd => area.Y + area.H - (yd - ylo) / V.yw * area.H;
    const multi = this.multi, wx = this._wx, wy = this._wy;
    ctx.save();
    ctx.beginPath(); ctx.rect(area.X - 8, area.Y - 8, area.W + 16, area.H + 16); ctx.clip();
    R.paths.forEach((pr, k) => {
      const m = this.mirrorOf(pr);
      if (!m || !m.eps || !V.endpoints) return;
      ctx.save();
      ctx.globalAlpha = 0.5;
      for (const r of m.eps) {
        if (!r.alive) continue;
        const nx = wrapC(r.sc, xlo, V.xw, wx); if (nx === 0) continue;
        const ny = wrapC(r.sw, ylo, V.yw, wy); if (ny === 0) continue;
        for (let a1 = 0; a1 < nx; a1++)
          for (let b1 = 0; b1 < ny; b1++) this.drawEpGlyph(PX(wx[a1]), PY(wy[b1]), r.kind, multi ? css(this.pcol(k)) : this.T.rim);
      }
      ctx.restore();
    });
    R.paths.forEach((pr, k) => {
      if (!pr.eps) return;
      const rim = multi ? css(this.pcol(k)) : this.T.rim;
      pr.eps.forEach((r, i) => {
        if (!r.alive) return;
        const hov = V.hoverEp && V.hoverEp.k === k && V.hoverEp.i === i;
        if (!V.endpoints && !hov) return;
        const nx = wrapC(r.sc, xlo, V.xw, wx); if (nx === 0) return;
        const ny = wrapC(r.sw, ylo, V.yw, wy); if (ny === 0) return;
        for (let a1 = 0; a1 < nx; a1++)
          for (let b1 = 0; b1 < ny; b1++) {
            this.drawEpGlyph(PX(wx[a1]), PY(wy[b1]), r.kind, rim);
            if (hov) this.drawHoverRing(PX(wx[a1]), PY(wy[b1]));
          }
      });
    });
    ctx.restore();
  }

  drawOriDecor() {
    const V = this.V, ctx = this.ctx, T = this.T;
    const area = oriArea(V.W, V.H);
    const xlo = V.xc - V.xw / 2, xhi = V.xc + V.xw / 2, ylo = V.yc - V.yw / 2, yhi = V.yc + V.yw / 2;
    const PX = xd => area.X + (xd - xlo) / V.xw * area.W;
    const PY = yd => area.Y + area.H - (yd - ylo) / V.yw * area.H;
    ctx.save();
    ctx.font = `11px ${FONT}`; ctx.fillStyle = T.tick;
    const tickOf = w => (w <= 12 ? 2 : w <= 40 ? 5 : w <= 100 ? 15 : 30);
    const tx = tickOf(V.xw), ty = tickOf(V.yw);
    for (let t = Math.ceil(xlo / tx) * tx; t <= xhi + 1e-9; t += tx) {
      const lt = ((t % 360) + 360) % 360, seam = Math.abs(lt) < 1e-9;
      ctx.strokeStyle = seam ? T.seam : T.grid; ctx.lineWidth = seam ? 1.0 : 0.6;
      ctx.beginPath(); ctx.moveTo(PX(t), area.Y); ctx.lineTo(PX(t), area.Y + area.H); ctx.stroke();
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText(fmtNum(lt), PX(t), area.Y + area.H + 6);
    }
    for (let t = Math.ceil(ylo / ty) * ty; t <= yhi + 1e-9; t += ty) {
      const lt = ((t % 360) + 360) % 360, seam = Math.abs(lt) < 1e-9;
      ctx.strokeStyle = seam ? T.seam : T.grid; ctx.lineWidth = seam ? 1.0 : 0.6;
      ctx.beginPath(); ctx.moveTo(area.X, PY(t)); ctx.lineTo(area.X + area.W, PY(t)); ctx.stroke();
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
      ctx.fillText(fmtNum(lt), area.X - 7, PY(t));
    }
    ctx.strokeStyle = T.frame; ctx.lineWidth = 1;
    ctx.strokeRect(area.X + 0.5, area.Y + 0.5, area.W - 1, area.H - 1);
    if (V.probe) {
      const wx = this._wx, wy = this._wy;
      const nx = wrapC(V.probe.x, xlo, V.xw, wx), ny = wrapC(V.probe.y, ylo, V.yw, wy);
      ctx.strokeStyle = T.probe; ctx.lineWidth = 2; ctx.beginPath();
      for (let a1 = 0; a1 < nx; a1++)
        for (let b1 = 0; b1 < ny; b1++) {
          const sx = PX(wx[a1]), sy = PY(wy[b1]);
          ctx.moveTo(sx - 8, sy); ctx.lineTo(sx - 3, sy); ctx.moveTo(sx + 3, sy); ctx.lineTo(sx + 8, sy);
          ctx.moveTo(sx, sy - 8); ctx.lineTo(sx, sy - 3); ctx.moveTo(sx, sy + 3); ctx.lineTo(sx, sy + 8);
        }
      ctx.stroke();
    }
    const ax = V.axes;
    ctx.font = `600 12px ${FONT}`; ctx.fillStyle = T.ink; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillText(axisLabel(ax.xl), area.X + area.W / 2, area.Y + area.H + 24);
    ctx.save();
    ctx.translate(15, area.Y + area.H / 2); ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(axisLabel(ax.yl), 0, 0);
    ctx.restore();
    ctx.restore();
  }

  drawLegend(sky) {
    const R = this.R, V = this.V, ctx = this.ctx, T = this.T;
    if (!R) return;
    const entries = [];
    const multi = this.multi;
    R.paths.forEach((pr, k) => {
      if (multi) { entries.push([t('legend.path', { name: pr.name }), this.pcol(k)]); return; }
      if (pr.ori) pr.ori.fams.forEach((fm, fi) => entries.push([V.famTitle(pr, fm), this.pal(fi)]));
      else if (pr.sky) pr.sky.forEach((s, fi) => entries.push([V.skyTitle(pr, s.name), this.pal(fi)]));
    });
    if (entries.length === 0) return;
    ctx.save();
    ctx.font = `11.5px ${FONT}`;
    let wMax = 0;
    for (const e of entries) wMax = Math.max(wMax, ctx.measureText(e[0]).width);
    const bw = wMax + 34, bh = entries.length * 17 + 10;
    const bx = V.W - bw - 12, by = V.H - bh - 12;
    ctx.fillStyle = T.legendBg; ctx.fillRect(bx, by, bw, bh);
    ctx.strokeStyle = T.legendLine; ctx.lineWidth = 1; ctx.strokeRect(bx + 0.5, by + 0.5, bw, bh);
    ctx.textBaseline = 'middle';
    entries.forEach((e, i) => {
      const y = by + 13 + i * 17;
      ctx.strokeStyle = css(e[1]); ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(bx + 8, y); ctx.lineTo(bx + 22, y); ctx.stroke();
      ctx.fillStyle = T.ink; ctx.fillText(e[0], bx + 27, y);
    });
    ctx.restore();
  }
}

/** 末端点符号 (天空 / 取向平面 / 3D 三个视图共用). */
export function drawEpGlyphOn(ctx, sx, sy, kind, rim) {
  ctx.setLineDash([]);
  if (kind === 'end') {
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, 2 * Math.PI);
    ctx.fillStyle = '#ffd21a'; ctx.fill(); ctx.lineWidth = 1.6; ctx.strokeStyle = rim; ctx.stroke();
  } else if (kind === 'turn') {
    ctx.beginPath(); ctx.moveTo(sx, sy - 6); ctx.lineTo(sx - 5.5, sy + 4.5); ctx.lineTo(sx + 5.5, sy + 4.5); ctx.closePath();
    ctx.fillStyle = '#7cc8f6'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = rim; ctx.stroke();
  } else if (kind === 'cusp') {
    ctx.strokeStyle = '#e040e0'; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i < 3; i++) {
      const a = Math.PI * i / 3, dx = 7 * Math.cos(a), dy = 7 * Math.sin(a);
      ctx.moveTo(sx - dx, sy - dy); ctx.lineTo(sx + dx, sy + dy);
    }
    ctx.stroke();
    ctx.beginPath(); ctx.arc(sx, sy, 2.2, 0, 2 * Math.PI);
    ctx.fillStyle = '#fff'; ctx.fill(); ctx.lineWidth = 1.1; ctx.strokeStyle = rim; ctx.stroke();
  } else if (kind === 'aper') {
    ctx.beginPath(); ctx.moveTo(sx, sy - 6.2); ctx.lineTo(sx + 6.2, sy); ctx.lineTo(sx, sy + 6.2); ctx.lineTo(sx - 6.2, sy); ctx.closePath();
    ctx.fillStyle = '#ff6a33'; ctx.fill(); ctx.lineWidth = 1.5; ctx.strokeStyle = rim; ctx.stroke();
  } else {
    ctx.strokeStyle = '#8a8f98'; ctx.lineWidth = 1.8; ctx.beginPath();
    ctx.moveTo(sx - 5, sy - 5); ctx.lineTo(sx + 5, sy + 5); ctx.moveTo(sx - 5, sy + 5); ctx.lineTo(sx + 5, sy - 5);
    ctx.stroke();
  }
}

export function lensTag(lens) { return LENS_TAGS[lens]; }

export function fmtNum(v) {
  return Number.isInteger(v) ? String(v) : (Math.round(v * 100) / 100).toString();
}
