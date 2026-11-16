// 取向平面卷成甜甜圈 (环面): 两个取向角都是转一圈回到原处, 平面的左右边、上下边本来就是同一处, 粘起来就是环面.
// 大圈方向 = 横轴 (α 方位等), 绕管子一圈 = 纵轴 (柱晶 roll / Lowitz ζ; ζ 只取 0–180° 时是半根管子).
// 面上的东西都按精确坐标直接放上去 (不是把平面图当贴图贴上): 一点 (横轴°, 纵轴°) ↦ 环面上
//   ((R + r cos w) cos u, (R + r cos w) sin u, r sin w),  u = 横轴折成一圈, w = 纵轴.
// 边界 / 焦散连线、网格是三维折线 (略高出曲面一点, 背面被管子挡住), 点云和线上的采样点是三维点,
// 末端点和位置查看的十字按投影位置画在叠加层上 (被管子挡住的不画); 高亮族的两侧着色直接用网格上的符号做成数据贴图.
// 悬停给读数, 单击 = 取向平面的位置查看.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { drawEpGlyphOn } from './render.js';

const D2R = Math.PI / 180;
const RING = 1, TUBE = 0.44;              // 中心圈半径, 管半径
const LIFT_LINE = 0.006, LIFT_DOT = 0.004, LIFT_MARK = 0.012;   // 线 / 点 / 符号高出曲面多少 (免得和曲面抢深度)
const LIGHT = new THREE.Vector3(-0.35, -0.55, 0.76).normalize();
const hex = c => (c[0] << 16) | (c[1] << 8) | c[2];
const css = (c, a = 1) => `rgba(${c[0]},${c[1]},${c[2]},${a})`;

export class TorusView {
  constructor(canvas, overlay) {
    this.canvas = canvas; this.overlay = overlay;
    this.gl = null; this.active = false; this.dirty = true;
    this.dataKey = null; this.geoKey = null;
    this.onHover = null; this.onPick = null;
    this.axes = null;
    this.marks = [];                             // 叠加层上的符号: { x, y, kind, rim, alpha } / { probe: true, x, y }
    this.lineMats = [];
  }

  _init() {
    if (this.gl) return;
    const r = this.gl = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, preserveDrawingBuffer: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xffffff);
    const cam = this.camera = new THREE.PerspectiveCamera(30, 1, 0.05, 50);
    cam.up.set(0, 0, 1);
    const ctl = this.controls = new OrbitControls(cam, this.canvas);
    ctl.enableDamping = true; ctl.dampingFactor = 0.09;
    ctl.minDistance = 1.2; ctl.maxDistance = 12;
    ctl.addEventListener('change', () => { this.dirty = true; });
    ctl.addEventListener('start', () => { this.tween = null; this.userMoved = true; });
    // 明暗直接烘进顶点色 (固定的斜上方光); 贴图只用来给高亮族的两侧着色 (没有时为空)
    this.mat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    this.mesh = null;
    this.dyn = new THREE.Group();
    this.scene.add(this.dyn);
    this.ray = new THREE.Raycaster();
    let down = null;
    this.canvas.addEventListener('pointermove', e => { this.hoverReq = e; });
    this.canvas.addEventListener('pointerleave', () => { this.hoverReq = 'off'; });
    this.canvas.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY }; });
    this.canvas.addEventListener('pointerup', e => {
      const d = down; down = null;
      if (!d || Math.hypot(e.clientX - d.x, e.clientY - d.y) > 4) return;
      const h = this._pick(e);
      if (h && this.onPick) this.onPick(h.x, h.y);
    });
    this.goView('oblique', true);
    const loop = () => {
      requestAnimationFrame(loop);
      if (!this.active) return;
      if (this.tween) this._stepTween();
      this.controls.update();
      if (this.hoverReq) this._doHover();
      if (this.canvas.clientWidth !== this.W || this.canvas.clientHeight !== this.H) this.dirty = true;   // 画布刚显示 / 窗口变了
      if (this.dirty) { this.dirty = false; this._draw(); }
    };
    requestAnimationFrame(loop);
  }

  setActive(on) {
    this.active = on;
    if (on) { this._init(); this.dirty = true; }
  }

  /** 纵轴范围变了 (ζ 整圈 / 半圈) 才重建曲面网格 */
  _geometry(y0, y1) {
    const key = y0 + ',' + y1;
    if (key === this.geoKey) return;
    this.geoKey = key;
    if (this.mesh) { this.scene.remove(this.mesh); this.mesh.geometry.dispose(); }
    const nu = 288, nv = Math.max(36, Math.round(144 * (y1 - y0) / 360)), nn = (nu + 1) * (nv + 1);
    const pos = new Float32Array(nn * 3), nor = new Float32Array(nn * 3), col = new Float32Array(nn * 3), uv = new Float32Array(nn * 2), idx = [];
    for (let j = 0; j <= nv; j++)
      for (let i = 0; i <= nu; i++) {
        const u = 2 * Math.PI * i / nu, w = (y0 + (y1 - y0) * j / nv) * D2R, k = j * (nu + 1) + i;
        const cu = Math.cos(u), su = Math.sin(u), cw = Math.cos(w), sw = Math.sin(w);
        pos.set([(RING + TUBE * cw) * cu, (RING + TUBE * cw) * su, TUBE * sw], 3 * k);
        nor.set([cw * cu, cw * su, sw], 3 * k);
        const sh = 0.74 + 0.26 * Math.max(0, cw * cu * LIGHT.x + cw * su * LIGHT.y + sw * LIGHT.z);
        col.set([sh, sh, sh], 3 * k);
        uv.set([i / nu, j / nv], 2 * k);
      }
    for (let j = 0; j < nv; j++)
      for (let i = 0; i < nu; i++) {
        const a = j * (nu + 1) + i, b = a + 1, c = a + nu + 1, d = c + 1;
        idx.push(a, b, d, a, d, c);
      }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    g.setIndex(idx);
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.renderOrder = 0;
    this.scene.add(this.mesh);
  }

  /** (横轴°, 纵轴°) → 环面上的点, 沿外法向抬高 lift */
  _xyz(x, y, lift, out, o) {
    const ax = this.axes, u = (x - ax.x0) / (ax.x1 - ax.x0) * 2 * Math.PI, w = y * D2R, r = TUBE + lift;
    const cw = Math.cos(w);
    out[o] = (RING + r * cw) * Math.cos(u); out[o + 1] = (RING + r * cw) * Math.sin(u); out[o + 2] = r * Math.sin(w);
  }
  /** 纵轴只取半圈 (ζ 0–180°) 时, 范围外的点不在管子上 */
  _inY(y) { const ax = this.axes; return ax.y1 - ax.y0 >= 360 - 1e-9 || (y >= ax.y0 - 1e-6 && y <= ax.y1 + 1e-6); }

  _clearDyn() {
    for (const o of [...this.dyn.children]) {
      this.dyn.remove(o);
      if (o.geometry) o.geometry.dispose();
      if (o.material) o.material.dispose();
    }
    this.lineMats = [];
    if (this.mat.map) { this.mat.map.dispose(); this.mat.map = null; this.mat.needsUpdate = true; }
  }
  _lines(pos, col, width, opacity = 1) {
    if (!pos.length) return;
    const m = new LineMaterial({ color: hex(col), linewidth: width, worldUnits: false, transparent: true, opacity, depthTest: true, depthWrite: false });
    if (this.W) m.resolution.set(this.W, this.H);
    this.lineMats.push(m);
    const g = new LineSegmentsGeometry();
    g.setPositions(pos);
    const l = new LineSegments2(g, m);
    l.frustumCulled = false;
    l.renderOrder = 3;
    this.dyn.add(l);
  }
  _points(pos, col, size, opacity, order = 2) {
    if (!pos.length) return;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos instanceof Float32Array ? pos : Float32Array.from(pos), 3));
    const p = new THREE.Points(g, new THREE.PointsMaterial({ color: hex(col), size, sizeAttenuation: false, transparent: true, opacity, depthWrite: false }));
    p.frustumCulled = false;
    p.renderOrder = order;
    this.dyn.add(p);
  }

  /** 新结果 / 图层 / 高亮 / 位置查看变化 → 重建面上的东西. V 来自 viewState. */
  setData(R, V) {
    this._init();
    const ax = this.axes = V.axes;
    this._geometry(ax.y0, ax.y1);
    const flags = R ? R.paths.map(p => [p.ori ? 1 : 0, p.lines ? 1 : 0, p.eps ? 1 : 0, p.mirror ? p.mirror.status : '-'].join('')).join('|') + (R.oriLocal ? 'L' : '') : '';
    const key = [R ? R.job : '', ax.x0, ax.x1, ax.y0, ax.y1, V.mc, V.arcs, V.lines, V.dots, V.endpoints, V.sym, V.lineW, V.mcSize, V.hlFull,
                 [...V.checked].join('|'), V.probe ? V.probe.x + ',' + V.probe.y : '', V.theme.bg.join(','), flags].join(';');
    if (key === this.dataKey) return;
    this.dataKey = key;
    this.scene.background = new THREE.Color(`rgb(${V.theme.bg.join(',')})`);
    this._clearDyn();
    this.marks = [];
    this._grid(V);
    if (R) this._build(R, V);
    this.dirty = true;
  }

  /** 网格: 每 45° 一圈; 横轴、纵轴的 0° (平面图的边) 加粗 */
  _grid(V) {
    const ax = this.axes, thin = [], bold = [], full = ax.y1 - ax.y0 >= 360 - 1e-9;
    const seg = (out, x0, y0, x1, y1, n) => {
      for (let k = 0; k < n; k++) {
        const o = out.length;
        out.length += 6;
        this._xyz(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, LIFT_LINE * 0.5, out, o);
        this._xyz(x0 + (x1 - x0) * (k + 1) / n, y0 + (y1 - y0) * (k + 1) / n, LIFT_LINE * 0.5, out, o + 3);
      }
    };
    for (let x = ax.x0; x < ax.x1 - 1e-9; x += 45) seg(x === ax.x0 ? bold : thin, x, ax.y0, x, ax.y1, Math.max(24, Math.round((ax.y1 - ax.y0) / 3)));
    for (let y = ax.y0; full ? y < ax.y1 - 1e-9 : y <= ax.y1 + 1e-9; y += 45) seg(y === ax.y0 || (!full && y >= ax.y1 - 1e-9) ? bold : thin, ax.x0, y, ax.x1, y, 180);
    this._lines(thin, [0, 0, 0], 1, 0.16);
    this._lines(bold, [0, 0, 0], 1.8, 0.7);
  }

  _build(R, V) {
    const T = V.theme, multi = R.paths.length > 1;
    const famLabel = (pr, fm) => (multi ? pr.name + '·' + fm.name : fm.name);
    const pal = i => T.palette[i % T.palette.length], pcol = k => T.pathcol[k % T.pathcol.length];
    const mirrorOf = pr => { const m = pr.mirror; return V.sym && m && m.status === 'ok' && !m.dup && m.ori ? m : null; };
    const dots = !!V.dots || !V.lines;

    // 高亮族的两侧着色 (同平面图: 那一侧淡淡染成族的颜色, 另一侧淡灰) — 网格上的符号直接做成贴图
    if (V.checked.size > 0 && V.mc) {
      let tex = null, nS = 0, nW = 0;
      for (const pr of R.paths) {
        if (!pr.ori) continue;
        pr.ori.fams.forEach((fm, fi) => {
          if (!V.checked.has(famLabel(pr, fm)) || fm.sense === 0) return;
          const side = pr.ori.side[fi], SC = pr.ori.sideSC, SW = pr.ori.sideSW;
          if (!tex) { nS = SC.length; nW = SW.length; tex = new Float32Array(nS * nW * 3).fill(1); }
          if (SC.length !== nS || SW.length !== nW) return;
          const col = pal(fi);
          for (let a = 0; a < nW; a++)
            for (let b = 0; b < nS; b++) {
              const sat = side[a * nS + b] === 1, c = sat ? col : T.sideGray, al = sat ? 0.10 : 0.08, o = 3 * (a * nS + b);
              for (let ch = 0; ch < 3; ch++) tex[o + ch] = tex[o + ch] * (1 - al) + (c[ch] / 255) * al;
            }
        });
      }
      if (tex) {
        const data = new Uint8Array(nS * nW * 4);
        for (let i = 0; i < nS * nW; i++) {
          data[4 * i] = Math.round(255 * tex[3 * i]); data[4 * i + 1] = Math.round(255 * tex[3 * i + 1]);
          data[4 * i + 2] = Math.round(255 * tex[3 * i + 2]); data[4 * i + 3] = 255;
        }
        const t = new THREE.DataTexture(data, nS, nW, THREE.RGBAFormat);
        t.colorSpace = THREE.SRGBColorSpace;
        t.wrapS = THREE.RepeatWrapping; t.magFilter = THREE.LinearFilter; t.minFilter = THREE.LinearFilter;
        t.needsUpdate = true;
        this.mat.map = t; this.mat.needsUpdate = true;
      }
    }

    // MC 点云 (成立的姿态) 与镜像光路的点云
    const cloud = (X, Y, col, alpha) => {
      const pos = new Float32Array(3 * X.length);
      let n = 0;
      for (let i = 0; i < X.length; i++) { if (!this._inY(Y[i])) continue; this._xyz(X[i], Y[i], LIFT_DOT * 0.5, pos, 3 * n); n++; }
      this._points(pos.subarray(0, 3 * n), col, V.mcSize, alpha, 1);
    };
    if (V.mc && V.arcs)
      R.paths.forEach((pr, k) => {
        if (pr.ori) cloud(pr.ori.mcX, pr.ori.mcY, multi ? pcol(k) : T.mc, multi ? T.mcAlpha * 0.8 : T.mcAlpha);
        const m = mirrorOf(pr);
        if (m) cloud(m.ori.mcX, m.ori.mcY, T.mirrorMc, T.mcAlpha);
      });

    // 边界 / 焦散连线: 成立的段常显; 不成立的段只在高亮这一族 (且勾了"整条") 时淡淡地画 — 同平面图
    if (V.lines) {
      const srcs = [];
      R.paths.forEach((pr, k) => {
        if (pr.ori && pr.lines) srcs.push({ pr, k, fams: pr.ori.fams, lines: pr.lines, mirror: false });
        const m = mirrorOf(pr);
        if (m && m.linesMap) srcs.push({ pr, k, fams: m.ori.fams, lines: m.linesMap, mirror: true });
      });
      for (const { pr, k, fams, lines, mirror } of srcs)
        fams.forEach((fm, fi) => {
          const hot = !mirror && V.checked.has(famLabel(pr, fm)), full = hot && V.hlFull;
          if (!hot && !V.arcs) return;
          const fl = lines.get(fm.name); if (!fl) return;
          const okPos = [], noPos = [];
          for (let li = 0; li < fl.par.length; li++) {
            const par = fl.par[li], sky = fl.sky[li], n = par.length / 2;
            for (let i = 0; i + 1 < n; i++) {
              const ok = sky[4 * i + 3] > 0.5 && sky[4 * i + 7] > 0.5;
              if (!ok && !full) continue;
              const xa = par[2 * i] / D2R, ya = par[2 * i + 1] / D2R, xb = par[2 * i + 2] / D2R, yb = par[2 * i + 3] / D2R;
              if (Math.abs(xb - xa) > 20 || Math.abs(yb - ya) > 20 || !this._inY(ya) || !this._inY(yb)) continue;   // 跨缝的跳变
              const out = ok ? okPos : noPos, o = out.length;
              out.length += 6;
              this._xyz(xa, ya, LIFT_LINE, out, o); this._xyz(xb, yb, LIFT_LINE, out, o + 3);
            }
          }
          const col = multi ? pcol(k) : pal(fi);
          if (full) this._lines(noPos, col, Math.max(1, V.lineW * 0.6), 0.32);
          this._lines(okPos, col, V.lineW, mirror ? 0.6 : 1);
        });
    }

    // 线上的采样点 (根点): 图层开了, 或者没画连线
    if (dots) {
      R.paths.forEach((pr, k) => {
        const m = mirrorOf(pr);
        if (m && V.arcs)
          m.ori.fams.forEach((fm, fi) => {
            const rt = m.ori.roots[fi], pos = [];
            for (let i = 0; i < rt.sc.length; i++) {
              if (!rt.ok[i] || !this._inY(rt.sw[i])) continue;
              const o = pos.length; pos.length += 3; this._xyz(rt.sc[i], rt.sw[i], LIFT_DOT, pos, o);
            }
            this._points(pos, multi ? pcol(k) : pal(fi), 2, 0.45);
          });
        if (!pr.ori) return;
        const srcs = [pr.ori];
        if (k === 0 && R.oriLocal) srcs.push(R.oriLocal);
        for (const src of srcs)
          src.fams.forEach((fm, fi) => {
            const hot = V.checked.has(famLabel(pr, fm));
            if (!V.arcs && !hot) return;
            const col = multi ? pcol(k) : pal(fi), rt = src.roots[fi], okPos = [], noPos = [];
            for (let i = 0; i < rt.sc.length; i++) {
              if (!this._inY(rt.sw[i])) continue;
              const out = rt.ok[i] ? okPos : noPos, o = out.length;
              out.length += 3; this._xyz(rt.sc[i], rt.sw[i], LIFT_DOT, out, o);
            }
            this._points(okPos, col, hot ? 3 : 2, 0.95, 3);
            this._points(noPos, col, hot && V.hlFull ? 2 : 1, hot && V.hlFull ? 0.40 : 0.12, 3);
          });
      });
    }

    // 末端点、位置查看: 叠加层上画 (每帧按镜头投影, 被管子挡住的不画)
    R.paths.forEach((pr, k) => {
      const rim = multi ? css(pcol(k)) : T.rim;
      const m = mirrorOf(pr);
      if (m && m.eps && V.endpoints)
        for (const r of m.eps) if (r.alive && this._inY(r.sw)) this.marks.push({ x: r.sc, y: r.sw, kind: r.kind, rim, alpha: 0.5 });
      if (pr.eps && V.endpoints)
        for (const r of pr.eps) if (r.alive && this._inY(r.sw)) this.marks.push({ x: r.sc, y: r.sw, kind: r.kind, rim, alpha: 1 });
    });
    if (V.probe) this.marks.push({ probe: true, x: V.probe.x, y: V.probe.y, col: T.probe });
  }

  /** 预设视角; 距离按画布宽高比算, 让整个环面 (连同外圈刻度) 正好装进画面 */
  goView(name, instant) {
    const spec = { oblique: [34, -64, 1.0], top: [89.5, -90, 0.97], side: [4, -90, 1.0] }[name];
    this.lastView = name; this.userMoved = false;
    const cam = this.camera, half = Math.min(cam.fov / 2, Math.atan(Math.tan(cam.fov / 2 * D2R) * cam.aspect) / D2R);
    this._tweenTo(spec[0], spec[1], spec[2] * (RING + TUBE + 0.14) / Math.sin(half * D2R), instant);
  }
  _tweenTo(el, az, dist, instant) {
    const off = this.camera.position.clone(), len = off.length() || dist;
    const from = { el: Math.asin(Math.max(-1, Math.min(1, off.z / len))) / D2R, az: Math.atan2(off.y, off.x) / D2R, dist: len };
    const daz = ((az - from.az + 540) % 360) - 180;
    this.tween = { t0: performance.now(), dur: instant ? 0 : 700, from, to: { el, az: from.az + daz, dist } };
    this._stepTween();
  }
  _stepTween() {
    const w = this.tween, tt = w.dur ? Math.min((performance.now() - w.t0) / w.dur, 1) : 1;
    const s = tt * tt * (3 - 2 * tt), lerp = (a, b) => a + (b - a) * s;
    const el = lerp(w.from.el, w.to.el) * D2R, az = lerp(w.from.az, w.to.az) * D2R, d = lerp(w.from.dist, w.to.dist);
    this.camera.position.set(d * Math.cos(el) * Math.cos(az), d * Math.cos(el) * Math.sin(az), d * Math.sin(el));
    this.controls.target.set(0, 0, 0);
    this.camera.lookAt(0, 0, 0);
    this.dirty = true;
    if (tt >= 1) this.tween = null;
  }

  _draw() {
    const W = this.canvas.clientWidth, H = this.canvas.clientHeight;
    if (W !== this.W || H !== this.H) {
      this.gl.setSize(W, H, false);
      this.camera.aspect = W / Math.max(H, 1);
      this.camera.updateProjectionMatrix();
      const dpr = Math.min(window.devicePixelRatio || 1, 3);
      this.overlay.width = Math.round(W * dpr); this.overlay.height = Math.round(H * dpr);
      this.W = W; this.H = H; this.dpr = dpr;
      for (const m of this.lineMats) m.resolution.set(W, H);
      if (!this.userMoved && !this.tween) this.goView(this.lastView || 'oblique', true);
    }
    this.gl.render(this.scene, this.camera);
    this._drawOverlay();
  }

  /** 环面上 (横轴°, 纵轴°) 处的点与外法向 */
  _at(a, v) {
    const ax = this.axes, u = (a - ax.x0) / (ax.x1 - ax.x0) * 2 * Math.PI, w = v * D2R;
    return { p: new THREE.Vector3((RING + TUBE * Math.cos(w)) * Math.cos(u), (RING + TUBE * Math.cos(w)) * Math.sin(u), TUBE * Math.sin(w)),
             n: new THREE.Vector3(Math.cos(w) * Math.cos(u), Math.cos(w) * Math.sin(u), Math.sin(w)) };
  }
  /** 镜头到 p (环面外的点) 的线段有没有穿过环面实体: 环面不是凸的, 只看法向朝不朝镜头不够 */
  _hidden(p) {
    const c = this.camera.position, half = (this.axes.y1 - this.axes.y0) < 360;
    const d = p.clone().sub(c), L = d.length(), q = new THREE.Vector3();
    for (let i = 1; i < 96; i++) {
      const s = i / 96;
      if (L * (1 - s) < 0.03) break;
      q.copy(c).addScaledVector(d, s);
      const rr = Math.hypot(q.x, q.y) - RING;
      if (rr * rr + q.z * q.z < TUBE * TUBE * 0.995 && !(half && q.z < 0)) return true;
    }
    return false;
  }
  /** 叠加层: 末端点、位置查看; 刻度 (横轴沿纵轴起点那一圈标, 纵轴沿横轴 0° 那一圈管子标; 只标看得见、不和已标的重叠的) */
  _drawOverlay() {
    const o = this.overlay, ctx = o.getContext('2d'), dpr = this.dpr || 1;
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, o.width, o.height);
    if (!this.axes) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = this.W, H = this.H, ax = this.axes;
    const screen = (a, v, out) => {
      const { p, n } = this._at(a, v), q = p.addScaledVector(n, out);
      if (this._hidden(q)) return null;
      const s = q.project(this.camera);
      return s.z > 1 ? null : [(s.x + 1) / 2 * W, (1 - s.y) / 2 * H];
    };
    for (const m of this.marks) {
      const s = screen(m.x, m.y, LIFT_MARK);
      if (!s) continue;
      if (m.probe) {
        const [sx, sy] = s;
        ctx.strokeStyle = m.col; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(sx - 8, sy); ctx.lineTo(sx - 3, sy); ctx.moveTo(sx + 3, sy); ctx.lineTo(sx + 8, sy);
        ctx.moveTo(sx, sy - 8); ctx.lineTo(sx, sy - 3); ctx.moveTo(sx, sy + 3); ctx.lineTo(sx, sy + 8);
        ctx.stroke();
        continue;
      }
      ctx.save(); ctx.globalAlpha = m.alpha;
      drawEpGlyphOn(ctx, s[0], s[1], m.kind, m.rim);
      ctx.restore();
    }
    const placed = [];
    const label = (a, v, txt, out, strong) => {
      const s = screen(a, v, out);
      if (!s) return;
      ctx.font = `${strong ? '600 ' : ''}12px system-ui, sans-serif`;
      const hw = ctx.measureText(txt).width / 2 + 4;                 // 轮廓边上的刻度别被画布边裁掉
      const x = Math.max(hw, Math.min(W - hw, s[0])), y = Math.max(10, Math.min(H - 10, s[1]));
      if (placed.some(b => Math.abs(b[0] - x) < b[2] + hw && Math.abs(b[1] - y) < 15)) return;
      placed.push([x, y, hw]);
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.92)'; ctx.strokeText(txt, x, y);
      ctx.fillStyle = strong ? '#1f2937' : '#4b5563'; ctx.fillText(txt, x, y);
    };
    const nm = s => s.split(' ')[0].replace('°', '');
    const xn = nm(ax.xl), yn = nm(ax.yl), full = ax.y1 - ax.y0 >= 360 - 1e-9;
    for (let a = ax.x0; a < ax.x1 - 1e-9; a += 45) label(a, ax.y0, `${xn} ${a}°`, 0.12, a === ax.x0);
    for (let v = ax.y0 + 45; full ? v < ax.y1 - 1e-9 : v <= ax.y1 + 1e-9; v += 45) label(ax.x0, v, `${yn} ${v}°`, 0.1, false);
  }

  _pick(e) {
    if (!this.mesh) return null;
    const r = this.canvas.getBoundingClientRect();
    this.ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), this.camera);
    const hit = this.ray.intersectObject(this.mesh, false)[0];
    if (!hit || !hit.uv) return null;
    const ax = this.axes;
    return { x: ax.x0 + hit.uv.x * (ax.x1 - ax.x0), y: ax.y0 + hit.uv.y * (ax.y1 - ax.y0) };
  }
  _doHover() {
    const e = this.hoverReq;
    this.hoverReq = null;
    if (this.onHover) this.onHover(e === 'off' ? null : this._pick(e));
  }

  snapshot(cb) {
    if (!this.gl) return cb(null);
    this._draw();
    const out = document.createElement('canvas');
    out.width = this.overlay.width; out.height = this.overlay.height;
    const ctx = out.getContext('2d');
    ctx.drawImage(this.canvas, 0, 0, out.width, out.height);
    ctx.drawImage(this.overlay, 0, 0);
    out.toBlob(cb, 'image/png');
  }
}
