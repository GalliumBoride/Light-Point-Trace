// 「天球」视图: 站在球外旁观整个天球. 观察者在球心, 地平面把球切成上下两半, 太阳、点云、边界和焦散线、
// 末端点都画在球面上, 可以拖动绕着球转. 天空视图是站在球心往外看 (要投影到平面上); 这里直接看球本身.
// 球面写深度, 所以球背面的点和线被挡住; 背面的线另画一份不做深度测试的淡色, 透过球隐约可见.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { drawEpGlyphOn, skyColorIndex, ISO_COL } from './render.js';
import { t } from './i18n.js';
import { haloDeg } from '../engine/lp.js';

const RS = 1.004;                 // 球面上的点 / 线略浮在球面外, 不和球面打架
const D2R = Math.PI / 180;
const rgb = c => `rgb(${c[0]},${c[1]},${c[2]})`;
const hex = c => (c[0] << 16) | (c[1] << 8) | c[2];

export class SphereView {
  constructor(canvas, overlay) {
    this.canvas = canvas;
    this.overlay = overlay;
    this.gl = null;               // 第一次切到这个视图才建 WebGL
    this.key = null;
    this.items = [];              // 随结果重建的物体: { obj, role, fi?, k? }
    this.lineMats = [];
    this.dirty = true;
    this.active = false;
    this.onHover = null;
  }

  _init() {
    if (this.gl) return;
    const r = this.gl = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff);
    const cam = this.camera = new THREE.PerspectiveCamera(32, 1, 0.05, 50);
    cam.up.set(0, 0, 1);
    const ctl = this.controls = new OrbitControls(cam, this.canvas);
    ctl.enableDamping = true; ctl.dampingFactor = 0.09;
    ctl.minDistance = 1.25; ctl.maxDistance = 9;
    ctl.addEventListener('change', () => { this.dirty = true; });
    ctl.addEventListener('start', () => { this.tween = null; this.userMoved = true; });
    this.dyn = new THREE.Group();
    scene.add(this.dyn);
    this._buildStatic();
    this.ray = new THREE.Raycaster();
    this.canvas.addEventListener('pointermove', e => { this.hoverReq = e; });
    this.canvas.addEventListener('pointerleave', () => { this.hoverReq = 'off'; });
    this.goView('oblique', true);
    const loop = () => {
      requestAnimationFrame(loop);
      if (!this.active) return;
      if (this.tween) this._stepTween();
      this.controls.update();
      if (this.hoverReq) this._doHover();
      if (this.dirty) { this.dirty = false; this._draw(); }
    };
    requestAnimationFrame(loop);
  }

  setActive(on) {
    this.active = on;
    if (on) { this._init(); this.dirty = true; }
  }

  _mat(col, width, o = {}) {
    const m = new LineMaterial({ color: hex(col), linewidth: width, worldUnits: false, dashed: !!o.dashed,
                                 dashSize: o.dash ?? 0.025, gapSize: o.gap ?? 0.02,
                                 transparent: true, opacity: o.opacity ?? 1,     // 都走透明队列: 排在球面之后画, 才会被球面挡住背面
                                 depthTest: !o.ghost, depthWrite: !o.ghost });
    this.lineMats.push(m);
    return m;
  }

  _seg(pos, col, width, o = {}) {
    const g = new LineSegmentsGeometry();
    g.setPositions(pos);
    const l = new LineSegments2(g, this._mat(col, width, o));
    if (o.dashed) l.computeLineDistances();
    l.frustumCulled = false;
    l.renderOrder = o.ghost ? 4 : 3;
    return l;
  }

  /** 不随结果变的: 球面 (地平线以下略深)、地平面、经纬网、观察者. */
  _buildStatic() {
    const S = this.scene;
    const sg = new THREE.SphereGeometry(1, 96, 64);
    sg.rotateX(Math.PI / 2);                              // 让球的两极落在 z 轴上 (z 朝上)
    const cols = new Float32Array(sg.attributes.position.count * 3);
    for (let i = 0; i < sg.attributes.position.count; i++) {
      const z = sg.attributes.position.getZ(i), c = z >= 0 ? [0.90, 0.93, 0.97] : [0.80, 0.83, 0.87];
      cols.set(c, 3 * i);
    }
    sg.setAttribute('color', new THREE.BufferAttribute(cols, 3));
    const shell = new THREE.Mesh(sg, new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.32,
                                                                 depthWrite: true, side: THREE.FrontSide }));
    shell.renderOrder = 1;
    this.shell = shell;
    const disc = new THREE.Mesh(new THREE.CircleGeometry(1, 160),
      new THREE.MeshBasicMaterial({ color: 0xdfe8d8, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide }));
    disc.renderOrder = 0;
    const obs = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 12), new THREE.MeshBasicMaterial({ color: 0x334155 }));
    obs.renderOrder = 0;
    // 经纬网: 每 30° 一条经线, 仰角 30° / 60° 两圈 (淡), 地平线 (绿)
    const grid = [];
    const circle = (fn, n = 180) => { for (let i = 0; i < n; i++) { const a = fn(i / n), b = fn((i + 1) / n); grid.push(...a, ...b); } };
    for (let k = 0; k < 12; k++) {
      const az = k * 30 * D2R;
      circle(u => { const el = (u - 0.5) * Math.PI; return [RS * Math.cos(el) * Math.cos(az), RS * Math.cos(el) * Math.sin(az), RS * Math.sin(el)]; }, 90);
    }
    for (const el of [-60, -30, 30, 60]) {
      const ce = Math.cos(el * D2R), se = Math.sin(el * D2R);
      circle(u => [RS * ce * Math.cos(2 * Math.PI * u), RS * ce * Math.sin(2 * Math.PI * u), RS * se]);
    }
    const gridObj = this._seg(grid, [150, 160, 178], 0.8, { opacity: 0.45 });
    const hz = [];
    for (let i = 0; i < 360; i++) {
      const a = 2 * Math.PI * i / 360, b = 2 * Math.PI * (i + 1) / 360;
      hz.push(RS * Math.cos(a), RS * Math.sin(a), 0, RS * Math.cos(b), RS * Math.sin(b), 0);
    }
    const horizon = this._seg(hz, [46, 139, 87], 1.8);
    S.add(disc, obs, shell, gridObj, horizon);
  }

  /** 新结果 (或镜像开关) → 重建球面上的东西. */
  setData(R, V) {
    this._init();
    this.V = V;
    const key = R ? [R.job, V.sym, ...R.paths.map(p => [!!p.mc, !!p.sky, !!p.lines, !!p.eps, p.iso ? p.iso.curves.length + ':' + (p.iso.levels || []).join(',') : ''].join(''))].join('|') : '';
    if (key === this.key) return;
    this.key = key;
    for (const o of [...this.dyn.children]) { this.dyn.remove(o); o.geometry && o.geometry.dispose(); }
    this.items = [];
    if (!R) { this.dirty = true; return; }
    const th = V.theme, multi = R.paths.length > 1, h = R.params.sun * D2R;
    const A = [Math.cos(h), 0, Math.sin(h)];
    this.A = A; this.R = R;
    const mirrors = V.sym ? [1, -1] : [1];
    const add = (obj, role, extra = {}) => { this.dyn.add(obj); this.items.push({ obj, role, ...extra }); };

    // 太阳、22° 晕、过太阳的水平圆、反日
    const sun = new THREE.Mesh(new THREE.SphereGeometry(0.03, 20, 14), new THREE.MeshBasicMaterial({ color: 0xf29a1a, transparent: true }));
    sun.position.set(...A.map(v => v * RS)); sun.renderOrder = 3;
    add(sun, 'deco');
    const b1 = [-Math.sin(h), 0, Math.cos(h)], b2 = [0, 1, 0], r22 = haloDeg() * D2R, ring = [];
    const onSph = (u, rr) => { const c = Math.cos(rr), s = Math.sin(rr), cu = Math.cos(u), su = Math.sin(u);
      return [0, 1, 2].map(i => RS * (c * A[i] + s * (cu * b1[i] + su * b2[i]))); };
    for (let i = 0; i < 240; i++) ring.push(...onSph(2 * Math.PI * i / 240, r22), ...onSph(2 * Math.PI * (i + 1) / 240, r22));
    add(this._seg(ring, [115, 115, 115], 1.2, { dashed: true }), 'deco');
    const pc = [], ce = Math.cos(h), se = Math.sin(h);
    for (let i = 0; i < 360; i++) {
      const a = 2 * Math.PI * i / 360, b = 2 * Math.PI * (i + 1) / 360;
      pc.push(RS * ce * Math.cos(a), RS * ce * Math.sin(a), RS * se, RS * ce * Math.cos(b), RS * ce * Math.sin(b), RS * se);
    }
    add(this._seg(pc, [120, 120, 120], 1, { dashed: true, dash: 0.008, gap: 0.016 }), 'deco');

    R.paths.forEach((pr, k) => {
      const pcol = th.pathcol[k % th.pathcol.length];
      // MC 点云
      if (pr.mc && pr.mc.length)
        for (const sy of mirrors) {
          const n = pr.mc.length / 3, pos = new Float32Array(3 * n);
          for (let i = 0; i < n; i++) {
            const x = pr.mc[3 * i], y = pr.mc[3 * i + 1], z = pr.mc[3 * i + 2], nn = Math.hypot(x, y, z) || 1;
            pos[3 * i] = RS * x / nn; pos[3 * i + 1] = sy * RS * y / nn; pos[3 * i + 2] = RS * z / nn;
          }
          const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
          const pts = new THREE.Points(g, new THREE.PointsMaterial({ color: hex(multi ? pcol : th.mc), size: 1.6, sizeAttenuation: false,
                                                                    transparent: true, opacity: multi ? 0.3 : 0.4, depthWrite: false }));
          pts.renderOrder = 2;
          add(pts, 'mc');
        }
      // 天空追踪的边界点 (1DOF 的弧就是它们)
      if (pr.sky)
        pr.sky.forEach((s, fi) => {
          const col = multi ? pcol : th.palette[skyColorIndex(pr, s.name, fi) % th.palette.length];
          for (const sy of mirrors) {
            const n = s.pts.length / 3, pos = new Float32Array(3 * n);
            for (let i = 0; i < n; i++) {
              const x = s.pts[3 * i], y = s.pts[3 * i + 1], z = s.pts[3 * i + 2], nn = Math.hypot(x, y, z) || 1;
              pos[3 * i] = RS * x / nn; pos[3 * i + 1] = sy * RS * y / nn; pos[3 * i + 2] = RS * z / nn;
            }
            const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
            const pts = new THREE.Points(g, new THREE.PointsMaterial({ color: hex(col), size: 2.2, sizeAttenuation: false,
                                                                      transparent: true, opacity: 0.95 }));
            pts.renderOrder = 3;
            add(pts, 'arc');
          }
        });
      // 边界 / 焦散连线: 成立的段常显, 再画一份不做深度测试的淡色透出球背面;
      // 不成立的段 (形式延拓) 和根点只在右栏高亮这一族时画 — 同天空视图
      if (pr.ori)
        pr.ori.fams.forEach((fm, fi) => {
          const col = multi ? pcol : th.palette[fi % th.palette.length];
          const fl = pr.lines && pr.lines.get(fm.name);
          for (const sy of mirrors) {
            if (fl) {
              const seg = [], segNo = [];
              for (const sky of fl.sky) {
                for (let i = 0; i + 1 < sky.length / 4; i++) {
                  const a = [sky[4 * i], sy * sky[4 * i + 1], sky[4 * i + 2]], b = [sky[4 * i + 4], sy * sky[4 * i + 5], sky[4 * i + 6]];
                  const na = Math.hypot(...a) || 1, nb = Math.hypot(...b) || 1;
                  if (Math.hypot(a[0] / na - b[0] / nb, a[1] / na - b[1] / nb, a[2] / na - b[2] / nb) > 0.2) continue;
                  (sky[4 * i + 3] > 0.5 && sky[4 * i + 7] > 0.5 ? seg : segNo).push(...a.map(v => RS * v / na), ...b.map(v => RS * v / nb));
                }
              }
              if (seg.length) {
                add(this._seg(seg, col, 2), 'line', { pr, fm });
                add(this._seg(seg, col, 1.2, { ghost: true, opacity: 0.22 }), 'ghost', { pr, fm });
              }
              if (segNo.length) {
                add(this._seg(segNo, col, 1.2, { opacity: 0.35 }), 'formal', { pr, fm });
                add(this._seg(segNo, col, 0.8, { ghost: true, opacity: 0.12 }), 'formal', { pr, fm });
              }
            }
            const rt = pr.ori.roots && pr.ori.roots[fi];
            if (rt && rt.D && rt.ok.length) {
              for (const want of [1, 0]) {
                const pos = [];
                for (let i = 0; i < rt.ok.length; i++) {
                  if ((rt.ok[i] ? 1 : 0) !== want) continue;
                  const x = rt.D[3 * i], y = sy * rt.D[3 * i + 1], z = rt.D[3 * i + 2], nn = Math.hypot(x, y, z) || 1;
                  pos.push(RS * x / nn, RS * y / nn, RS * z / nn);
                }
                if (!pos.length) continue;
                const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
                const pts = new THREE.Points(g, new THREE.PointsMaterial({ color: hex(col), size: want ? 3 : 2, sizeAttenuation: false,
                                                                          transparent: true, opacity: want ? 1 : 0.3, depthWrite: false }));
                pts.renderOrder = 3;
                add(pts, want ? 'roots' : 'rootsNo', { pr, fm });
              }
            }
          }
        });
    });
    // 等光强线 (勾了图层才显示)
    R.paths.forEach(pr => {
      const d = pr.iso;
      if (!d || d.status !== 'ok') return;
      for (const sy of mirrors) for (const cv of d.curves) {
        const k = d.levels.indexOf(cv.c), col = ISO_COL[(k < 0 ? 0 : k) % ISO_COL.length], D = cv.D, seg = [];
        for (let i = 0; i + 1 < D.length / 3; i++) {
          const a = [D[3 * i], sy * D[3 * i + 1], D[3 * i + 2]], b = [D[3 * i + 3], sy * D[3 * i + 4], D[3 * i + 5]];
          if (Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > 0.2) continue;
          seg.push(...a.map(v => RS * v), ...b.map(v => RS * v));
        }
        if (seg.length) add(this._seg(seg, col, 1.4), 'iso');
      }
    });
    this.clouds = new Map();
    this.layoutKey = null;
    this.layout(V);
  }

  /** 高亮的族要画「满足侧」点云: 没有就请主程序向 Worker 要, 到了建一份 (同天空视图). */
  _syncClouds(V) {
    const R = this.R;
    if (!R || !V.mc) return '';
    const multi = R.paths.length > 1, th = V.theme;
    let state = '';
    for (const pr of R.paths) {
      if (!pr.ori) continue;
      pr.ori.fams.forEach((fm, fi) => {
        if (fm.sense === 0 || !V.checked.has(V.famKey(pr, fm.name))) return;
        const key = pr.name + '|' + fm.name, cloud = R.clouds.get(key);
        if (cloud === undefined && R.requestCloud) { R.requestCloud(pr, fm); return; }
        if (!(cloud instanceof Float32Array)) return;
        state += key + ';';
        if (this.clouds.has(key)) return;
        const objs = [];
        for (const sy of V.sym ? [1, -1] : [1]) {
          const n = cloud.length / 3, pos = new Float32Array(3 * n);
          for (let i = 0; i < n; i++) {
            const x = cloud[3 * i], y = sy * cloud[3 * i + 1], z = cloud[3 * i + 2], nn = Math.hypot(x, y, z) || 1;
            pos[3 * i] = RS * x / nn; pos[3 * i + 1] = RS * y / nn; pos[3 * i + 2] = RS * z / nn;
          }
          const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
          const pts = new THREE.Points(g, new THREE.PointsMaterial({ color: hex(multi ? th.pathcol[R.paths.indexOf(pr) % th.pathcol.length] : th.palette[fi % th.palette.length]),
                                                                   size: 1.6, sizeAttenuation: false, transparent: true, opacity: 0.22, depthWrite: false }));
          pts.renderOrder = 2;
          this.dyn.add(pts); objs.push(pts);
        }
        this.clouds.set(key, { objs, pr, fm });
      });
    }
    return state;
  }

  /** 只改显示 (图层开关、线宽、高亮的族). */
  layout(V) {
    if (!this.gl) return;
    this.V = V;
    const cloudState = this._syncClouds(V);
    const lk = [V.lineW, V.lines, V.arcs, V.mc, V.hlFull, V.iso, [...V.checked].join('|'), cloudState].join(',');
    if (lk !== this.layoutKey) {
      this.layoutKey = lk;
      for (const it of this.items) {
        const full = it.fm && V.checked.has(V.famKey(it.pr, it.fm.name));
        if (it.role === 'mc') it.obj.visible = V.mc && V.arcs;
        else if (it.role === 'arc') it.obj.visible = V.arcs;
        else if (it.role === 'line' || it.role === 'ghost') {
          it.obj.visible = V.lines && (V.arcs || full);
          it.obj.material.linewidth = it.role === 'line' ? (full ? 2 * V.lineW : V.lineW) : 0.6 * V.lineW;
        } else if (it.role === 'formal') {
          it.obj.visible = V.lines && full && V.hlFull;
          it.obj.material.linewidth = 0.6 * V.lineW;
        } else if (it.role === 'roots') it.obj.visible = full;
        else if (it.role === 'rootsNo') it.obj.visible = full && V.hlFull;
        else if (it.role === 'iso') { it.obj.visible = !!V.iso; it.obj.material.linewidth = 0.7 * V.lineW; }
      }
      for (const c of this.clouds.values())
        for (const o of c.objs) o.visible = V.mc && V.checked.has(V.famKey(c.pr, c.fm.name));
    }
    this.dirty = true;
  }

  // ---------- 视角 ----------
  /** 整个球 (加一圈余量) 正好装进画面的镜头距离. */
  _fitDist() {
    const cam = this.camera, vf = cam.fov * D2R, hf = 2 * Math.atan(Math.tan(vf / 2) * cam.aspect);
    return 1.18 / Math.sin(Math.min(vf, hf) / 2);
  }
  goView(name, instant) {
    const h = this.A ? Math.asin(this.A[2]) / D2R : 10;
    const spec = { oblique: [26, -48], sun: [h, 0], top: [89.5, -90], side: [4, -90] }[name];
    this.lastView = name;
    this._tweenTo(spec[0], spec[1], this._fitDist(), instant);
  }
  /** 把视角转到某个天空方向 D 的正前方 (从球外看过去). */
  focusDir(D) {
    const el = Math.asin(Math.max(-1, Math.min(1, D[2]))) / D2R, az = Math.atan2(D[1], D[0]) / D2R;
    this._tweenTo(el, az, this._fitDist() * 0.75, false);
  }
  _tweenTo(el, az, dist, instant) {
    const off = this.camera.position.clone();
    const len = off.length() || dist;
    const from = { el: Math.asin(Math.max(-1, Math.min(1, off.z / len))) / D2R, az: Math.atan2(off.y, off.x) / D2R, dist: len,
                   target: this.controls.target.clone() };
    const daz = ((az - from.az + 540) % 360) - 180;
    this.tween = { t0: performance.now(), dur: instant ? 0 : 700, from, to: { el, az: from.az + daz, dist } };
    this._stepTween();
  }
  _stepTween() {
    const w = this.tween, tt = w.dur ? Math.min((performance.now() - w.t0) / w.dur, 1) : 1;
    const e = tt < 0.5 ? 4 * tt * tt * tt : 1 - Math.pow(-2 * tt + 2, 3) / 2, L = (a, b) => a + (b - a) * e;
    const el = L(w.from.el, w.to.el) * D2R, az = L(w.from.az, w.to.az) * D2R, dist = L(w.from.dist, w.to.dist);
    this.controls.target.lerpVectors(w.from.target, new THREE.Vector3(0, 0, 0), e);
    this.camera.position.set(dist * Math.cos(el) * Math.cos(az), dist * Math.cos(el) * Math.sin(az), dist * Math.sin(el));
    this.controls.update();
    this.dirty = true;
    if (tt >= 1) this.tween = null;
  }

  // ---------- 画 ----------
  requestDraw() { this.dirty = true; }
  _draw() {
    const W = this.canvas.clientWidth, H = this.canvas.clientHeight;
    if (W !== this.W || H !== this.H) {
      this.gl.setSize(W, H, false);
      this.camera.aspect = W / Math.max(H, 1);
      this.camera.updateProjectionMatrix();
      const dpr = Math.min(window.devicePixelRatio || 1, 3);
      this.overlay.width = Math.round(W * dpr); this.overlay.height = Math.round(H * dpr);
      this.W = W; this.H = H; this.dpr = dpr;
      if (!this.userMoved) this.goView(this.lastView || 'oblique', true);   // 画布尺寸定了再取景 (没动过镜头时)
    }
    for (const m of this.lineMats) m.resolution.set(W, H);
    this.gl.render(this.scene, this.camera);
    this._drawOverlay();
  }

  /** 球面上方向 D 是否朝向镜头 (球外看过去的那半个球). */
  _front(D) {
    const c = this.camera.position;
    return D[0] * c.x + D[1] * c.y + D[2] * c.z > 1;
  }
  _proj(p, out) {
    const v = this._v || (this._v = new THREE.Vector3());
    v.set(p[0], p[1], p[2]).project(this.camera);
    if (v.z > 1 || v.z < -1) return false;
    out[0] = (v.x + 1) / 2 * this.W; out[1] = (1 - v.y) / 2 * this.H;
    return true;
  }

  _drawOverlay() {
    const ctx = this.overlay.getContext('2d'), V = this.V;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.overlay.width, this.overlay.height);
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    if (!V) return;
    const s = [0, 0];
    ctx.font = '12px "PingFang SC", "Microsoft YaHei UI", "Microsoft YaHei", system-ui, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    const lab = (D, text, col) => {
      if (!this._proj(D.map(v => v * RS), s)) return;
      ctx.globalAlpha = this._front(D) ? 1 : 0.35;
      ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.9)'; ctx.strokeText(text, s[0], s[1] - 8);
      ctx.fillStyle = col; ctx.fillText(text, s[0], s[1] - 8);
      ctx.globalAlpha = 1;
    };
    const A = this.A || [Math.cos(10 * D2R), 0, Math.sin(10 * D2R)];
    lab(A, t('v3.sunLabel'), '#c46f00');
    lab([0, 0, 1], t('cam.zen'), '#475569');
    lab([-A[0], 0, A[2]], t('cam.anti'), '#475569');
    if (this._proj([0, 0, 0], s)) {
      ctx.fillStyle = '#334155'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.9)';
      ctx.strokeText(t('sph.observer'), s[0], s[1] - 8); ctx.fillText(t('sph.observer'), s[0], s[1] - 8);
    }
    // 末端点: 朝向镜头的正常画, 背面的很淡
    const R = this.R;
    if (R) {
      const multi = R.paths.length > 1;
      R.paths.forEach((pr, k) => {
        if (!pr.eps) return;
        const rim = multi ? rgb(V.theme.pathcol[k % V.theme.pathcol.length]) : '#000';
        pr.eps.forEach((r, i) => {
          if (!r.alive) return;
          const hov = V.hoverEp && V.hoverEp.k === k && V.hoverEp.i === i;
          if (!V.endpoints && !hov) return;
          for (const sy of V.sym ? [1, -1] : [1]) {
            const D = [r.D[0], sy * r.D[1], r.D[2]];
            if (!this._proj(D.map(v => v * RS), s)) continue;
            ctx.globalAlpha = this._front(D) ? 1 : 0.22;
            drawEpGlyphOn(ctx, s[0], s[1], r.kind, rim);
            ctx.globalAlpha = 1;
            if (hov && sy === 1) {
              ctx.save(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; ctx.setLineDash([3, 2.5]);
              ctx.beginPath(); ctx.arc(s[0], s[1], 12, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
            }
          }
        });
      });
    }
    // 鼠标指着的方向
    if (this.hover && this._proj(this.hover.map(v => v * RS), s)) {
      ctx.save(); ctx.strokeStyle = '#111'; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.arc(s[0], s[1], 6, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
    }
  }

  _doHover() {
    const e = this.hoverReq;
    this.hoverReq = null;
    const had = !!this.hover;
    this.hover = null;
    if (e !== 'off') {
      const r = this.canvas.getBoundingClientRect();
      this.ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), this.camera);
      const hit = this.ray.ray.intersectSphere(new THREE.Sphere(new THREE.Vector3(), 1), new THREE.Vector3());
      if (hit) this.hover = [hit.x, hit.y, hit.z];
    }
    if (had || this.hover) this.dirty = true;
    if (this.onHover) this.onHover(this.hover ? { D: this.hover, onSurf: false } : null);
  }

  /** 导出: WebGL 画面 + 叠加层合成一张 PNG. */
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
