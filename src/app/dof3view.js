// 三自由度 (第三个取向角放开到区间 [lo, hi]) 的两个 3D 视图:
//   'space' 取向空间 — 族内坐标的方盒 (α, 第二个角, 第三个角). 顶面 / 底面 = 区间两端的 2DOF 取向平面 (贴图);
//           盒子里: 边界面 (光路成立区的壁, 精确参数化: w 球面上的边界曲线 × 绕太阳的 ψ)、临界纤维 (天上圆弧的原像)、MC 点.
//   'lift'  抬升 — 地面 = 以太阳为中心的天空鱼眼, 高度 = 第三个角. 每一层是那个角度下的 2DOF 区域; 从上往下看, 整块的
//           外轮廓就是三自由度的区域 (地面上的粗线). 边界面扫出的墙在圆弧处竖直 — 圆弧就是墙的 "侧影".
// 数据由 Worker 的 space3 / lift3 请求给出 (engine/dof3.js 的 spaceData / liftData).

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Renderer, oriArea } from './render.js';
import { KIND3D } from './view3d.js';
import { axesMeta } from '../engine/lp.js';

const D2R = Math.PI / 180;
const ZH = 0.45;                       // 方盒 / 抬升的半高
const SRC_COL = { interior: [44, 160, 44], fence: [148, 103, 189], corner: [140, 86, 75] };
const SLICE_COL = [[31, 119, 180], [214, 39, 40]];
const EP_COL = { tan: [44, 160, 44], corner: [224, 123, 0], end: [255, 210, 26], turn: [124, 200, 246], cusp: [224, 64, 224], focus: [148, 103, 189], aper: [255, 106, 51], cross: [138, 143, 152] };
const col3 = c => new THREE.Color(c[0] / 255, c[1] / 255, c[2] / 255);

export class Dof3View {
  constructor(canvas, overlay, mode) {
    this.canvas = canvas; this.overlay = overlay; this.mode = mode;
    this.gl = null; this.active = false; this.dirty = true; this.key = null;
    this.info = null;
  }

  _init() {
    if (this.gl) return;
    const r = this.gl = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, preserveDrawingBuffer: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xffffff);
    const cam = this.camera = new THREE.PerspectiveCamera(30, 1, 0.02, 60);
    cam.up.set(0, 0, 1);
    const ctl = this.controls = new OrbitControls(cam, this.canvas);
    ctl.enableDamping = true; ctl.dampingFactor = 0.09;
    ctl.minDistance = 0.6; ctl.maxDistance = 14;
    ctl.addEventListener('change', () => { this.dirty = true; });
    ctl.addEventListener('start', () => { this.tween = null; this.userMoved = true; });
    this.group = new THREE.Group();
    this.scene.add(this.group);
    this.goView('oblique', true);
    const loop = () => {
      requestAnimationFrame(loop);
      if (!this.active) return;
      if (this.tween) this._stepTween();
      this.controls.update();
      if (this.canvas.clientWidth !== this.W || this.canvas.clientHeight !== this.H) this.dirty = true;
      if (this.dirty) { this.dirty = false; this._draw(); }
    };
    requestAnimationFrame(loop);
  }

  setActive(on) { this.active = on; if (on) { this._init(); this.dirty = true; } }

  _clear() {
    for (const o of [...this.group.children]) {
      this.group.remove(o);
      o.traverse(x => { if (x.geometry) x.geometry.dispose(); if (x.material) { if (x.material.map) x.material.map.dispose(); x.material.dispose(); } });
    }
  }

  /** R: 当前结果 (R.params.dof3 = { kind, lo, hi, zenHalf }); V: viewState. 每条光路的 pr.space3 / pr.lift3 是 Worker 给的数据 */
  setData(R, V) {
    this._init();
    const P = R && R.params.dof3;
    const ready = R ? R.paths.map(p => (this.mode === 'space' ? (p.space3 ? 1 : 0) + (p.slices ? p.slices.map(s => (s && s.lines ? 1 : 0)).join('') : '') : p.lift3 ? 1 : 0)).join('|') : '';
    const key = [R ? R.job : '', this.mode, ready, V.mc, V.lines, V.endpoints, V.lineW, V.mcSize, V.sym, V.theme.bg.join(',')].join(';');
    if (key === this.key) return;
    this.key = key;
    this._clear();
    this.info = null;
    if (!R || !P) { this.dirty = true; return; }
    this.scene.background = new THREE.Color(`rgb(${V.theme.bg.join(',')})`);
    if (this.mode === 'space') this._buildSpace(R, V, P); else this._buildLift(R, V, P);
    if (!this.userMoved && !this.tween) this.goView(this.lastView || 'oblique', true);
    this.dirty = true;
  }

  // ---------------------------------------------------------------- 取向空间
  _buildSpace(R, V, P) {
    const yMax = P.kind === 'low' && P.zenHalf ? 180 : 360, yd = yMax / 360;
    const lo = P.lo, hi = P.hi;
    const map = (x, y, z) => [x / 360 * 2 - 1, (y / yMax * 2 - 1) * yd, ((z - lo) / Math.max(1e-9, hi - lo) * 2 - 1) * ZH];
    this.info = { kind: 'space', yMax, yd, lo, hi, P, map };
    const g = this.group, multi = R.paths.length > 1;
    // 盒框
    const E = [];
    const cs = [[-1, -yd], [1, -yd], [1, yd], [-1, yd]];
    for (let k = 0; k < 4; k++) { const a = cs[k], b = cs[(k + 1) % 4]; E.push(a[0], a[1], -ZH, b[0], b[1], -ZH, a[0], a[1], ZH, b[0], b[1], ZH, a[0], a[1], -ZH, a[0], a[1], ZH); }
    const fg = new THREE.BufferGeometry(); fg.setAttribute('position', new THREE.Float32BufferAttribute(E, 3));
    g.add(new THREE.LineSegments(fg, new THREE.LineBasicMaterial({ color: 0x9aa3b2 })));
    // 顶/底面: 两端的 2DOF 取向平面 (第一条光路; 多条时只画第一条的面)
    const pr0 = R.paths[0];
    [0, 1].forEach(si => {
      const z = si ? ZH : -ZH, sl = pr0.slices && pr0.slices[si];
      const geo = new THREE.PlaneGeometry(2, 2 * yd);
      let mat;
      if (sl && sl.face && sl.ori && sl.lines) {
        const tex = this._faceTexture(R, V, P, sl);
        mat = new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide, transparent: true, opacity: si ? 0.55 : 0.92, depthWrite: false });   // 顶面半透明, 看得见盒子里
      } else mat = new THREE.MeshBasicMaterial({ color: 0xeef1f6, side: THREE.DoubleSide, transparent: true, opacity: 0.5, depthWrite: false });
      const m = new THREE.Mesh(geo, mat); m.position.z = z; m.renderOrder = 1;
      g.add(m);
    });
    R.paths.forEach((pr, k) => {
      const d = pr.space3; if (!d) return;
      if (V.mc && d.mc && d.mc.length) this._points(d.mc, (x, y, z) => map(x, y, z), multi ? V.theme.pathcol[k % 6] : [120, 126, 138], 60000, V.mcSize);
      if (V.lines) for (const s of d.surf) this._surface(s.tri, (x, y, z) => map(x, y, z), KIND3D[s.kind] || [90, 90, 90]);
      for (const f of d.fibers) this._polyline(f.pts, (x, y, z) => map(x, y, z), SRC_COL[f.src] || [44, 160, 44], f.cls === 'none' ? 0.35 : 1, true);
      if (V.endpoints && pr.d3) for (const e of pr.d3.eps) for (const b of e.box) this._marker(map(b[0], b[1], b[2]), EP_COL[e.kind] || [0, 0, 0]);
    });
  }

  /** 一端的 2DOF 取向平面画成贴图 (同 torus3d: 离屏 2D 渲染器画整张 360° 视窗, 裁出绘图区) */
  _faceTexture(R, V, P, sl) {
    const om = P.kind === 'col' ? 2 : 1;      // OMode.ZetaFixed / ZenFree
    const ax = axesMeta(om);
    const xw = ax.x1 - ax.x0, yw = ax.y1 - ax.y0, aw = 900, ah = Math.round(aw / 2 * yw / 360), dpr = 2;
    const paper = document.createElement('canvas'), rr = new Renderer(paper);
    const Rs = { job: R.job, params: R.params, clouds: new Map(), paths: [{ name: R.paths[0].name, faces: R.paths[0].faces, ori: sl.ori, eps: sl.eps, lines: sl.lines, done: true }] };
    const Vt = { ...V, W: aw + 82, H: ah + 136, dpr, view: 1, chrome: false, dragging: false, hoverEp: null, iso: false, sym: false, probe: null,
                 checked: new Set(), xw, yw, xc: (ax.x0 + ax.x1) / 2, yc: (ax.y0 + ax.y1) / 2, axes: ax };
    rr.render(Vt, Rs);
    const area = oriArea(Vt.W, Vt.H), tc = document.createElement('canvas');
    tc.width = aw * dpr; tc.height = ah * dpr;
    tc.getContext('2d').drawImage(paper, area.X * dpr, area.Y * dpr, tc.width, tc.height, 0, 0, tc.width, tc.height);
    const tex = new THREE.CanvasTexture(tc);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = this.gl.capabilities.getMaxAnisotropy();
    return tex;
  }

  // ---------------------------------------------------------------- 抬升
  _buildLift(R, V, P) {
    const lo = P.lo, hi = P.hi;
    let rmax = 10;
    for (const pr of R.paths) {
      const d = pr.lift3; if (!d) continue;
      for (const L of d.outline) for (let i = 0; i < L.pts.length; i += 2) rmax = Math.max(rmax, Math.hypot(L.pts[i], L.pts[i + 1]));
    }
    rmax = Math.min(180, rmax * 1.06);
    const s = 1 / rmax, zOf = z => ((z - lo) / Math.max(1e-9, hi - lo) * 2 - 1) * ZH, map = (x, y, z) => [x * s, y * s, zOf(z)];
    const ground = -ZH - 0.08;
    this.info = { kind: 'lift', s, lo, hi, rmax, ground, map, zOf };
    const g = this.group, multi = R.paths.length > 1;
    // 地面: 太阳 + 22° / 46° 等距圆 (虚线) + 外框圆
    const ring = (r, c, dash) => {
      const pts = [];
      for (let i = 0; i <= 180; i++) { const t = i / 180 * 2 * Math.PI; pts.push(r * s * Math.cos(t), r * s * Math.sin(t), ground); }
      const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
      const line = new THREE.Line(geo, dash ? new THREE.LineDashedMaterial({ color: c, dashSize: 0.02, gapSize: 0.015 }) : new THREE.LineBasicMaterial({ color: c }));
      if (dash) line.computeLineDistances();
      g.add(line);
    };
    for (const r of [22, 46]) if (r < rmax) ring(r, 0xb8bfcc, true);
    ring(rmax, 0xd5dae3, false);
    this._marker([0, 0, ground], [255, 178, 30], 0.022);
    R.paths.forEach((pr, k) => {
      const d = pr.lift3; if (!d) return;
      if (V.mc && d.mc.length) this._points(d.mc, map, multi ? V.theme.pathcol[k % 6] : [120, 126, 138], 80000, V.mcSize);
      if (V.lines) for (const sf of d.surf) this._surface(sf.tri, map, KIND3D[sf.kind] || [90, 90, 90]);
      for (const f of d.fibers) this._polyline(f.pts, map, SRC_COL[f.src] || [44, 160, 44], f.cls === 'none' ? 0.35 : 1, false);
      // 两端的 2DOF 曲线放在各自的高度
      d.slices.forEach((sl, si) => {
        for (const L of sl.lines) {
          const p = [];
          for (let i = 0; i < L.pts.length / 3; i++) p.push(L.pts[3 * i], L.pts[3 * i + 1], sl.z, L.pts[3 * i + 2]);
          this._polyline4(p, map, SLICE_COL[si]);
        }
      });
      // 地面上的外轮廓 (= 从上往下看整块的侧影)
      for (const L of d.outline) {
        const p = [];
        for (let i = 0; i < L.pts.length / 2; i++) p.push(L.pts[2 * i] * s, L.pts[2 * i + 1] * s, ground);
        this._rawLine(p, L.src === 'arc' ? [44, 160, 44] : [40, 44, 52], 1);
      }
      if (V.endpoints && pr.d3) {
        const fo = [0, 0];
        for (const e of pr.d3.eps) {
          const q = fishOf(e.D, R.params.sun, fo);
          this._marker([q[0] * s, q[1] * s, ground], EP_COL[e.kind] || [0, 0, 0]);
          for (const b of e.box) this._marker([q[0] * s, q[1] * s, zOf(b[2])], EP_COL[e.kind] || [0, 0, 0], 0.011);
        }
      }
    });
  }

  // ---------------------------------------------------------------- 图元
  _points(arr, map, c, cap, size) {
    const n = arr.length / 3, step = Math.max(1, Math.floor(n / cap)), pos = [];
    for (let i = 0; i < n; i += step) pos.push(...map(arr[3 * i], arr[3 * i + 1], arr[3 * i + 2]));
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    const pts = new THREE.Points(geo, new THREE.PointsMaterial({ color: col3(c), size: 1.6 * (size || 1), sizeAttenuation: false, transparent: true, opacity: 0.35, depthWrite: false }));
    this.group.add(pts);
  }
  _surface(tri, map, c) {
    const n = tri.length / 3, pos = new Float32Array(tri.length);
    for (let i = 0; i < n; i++) pos.set(map(tri[3 * i], tri[3 * i + 1], tri[3 * i + 2]), 3 * i);
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.computeVertexNormals();
    const mat = new THREE.MeshLambertMaterial({ color: col3(c), side: THREE.DoubleSide, transparent: true, opacity: 0.17, depthWrite: false });
    const m = new THREE.Mesh(geo, mat); m.renderOrder = 2;
    this.group.add(m);
    if (!this.light) {
      this.light = true;
      this.scene.add(new THREE.AmbientLight(0xffffff, 1.6));
      const dl = new THREE.DirectionalLight(0xffffff, 1.4); dl.position.set(-0.4, -0.6, 1); this.scene.add(dl);
    }
  }
  /** 折线 (x, y, z 三元组); box = 族内坐标 (跨 0/360° 缝处断开) */
  _polyline(arr, map, c, alpha, box) {
    const out = [];
    let prev = null;
    for (let i = 0; i < arr.length / 3; i++) {
      const x = arr[3 * i], y = arr[3 * i + 1], z = arr[3 * i + 2];
      if (prev && (!box || (Math.abs(x - prev[0]) < 180 && Math.abs(y - prev[1]) < 180))) out.push(...map(...prev), ...map(x, y, z));
      prev = [x, y, z];
    }
    this._segs(out, c, alpha);
  }
  /** 折线 (x, y, z, 成立) 四元组: 只连成立段 */
  _polyline4(arr, map, c) {
    const out = [];
    for (let i = 4; i < arr.length; i += 4)
      if (arr[i + 3] > 0.5 && arr[i - 1] > 0.5 && Math.hypot(arr[i] - arr[i - 4], arr[i + 1] - arr[i - 3]) < 15) out.push(...map(arr[i - 4], arr[i - 3], arr[i - 2]), ...map(arr[i], arr[i + 1], arr[i + 2]));
    this._segs(out, c, 1);
  }
  _rawLine(p, c, alpha) {
    const out = [];
    for (let i = 3; i < p.length; i += 3) if (Math.hypot(p[i] - p[i - 3], p[i + 1] - p[i - 2]) < 0.2) out.push(p[i - 3], p[i - 2], p[i - 1], p[i], p[i + 1], p[i + 2]);
    this._segs(out, c, alpha);
  }
  _segs(out, c, alpha) {
    if (!out.length) return;
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
    const m = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: col3(c), transparent: alpha < 1, opacity: alpha }));
    m.renderOrder = 3;
    this.group.add(m);
  }
  _marker(p, c, r = 0.014) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 12, 8), new THREE.MeshBasicMaterial({ color: col3(c) }));
    m.position.set(p[0], p[1], p[2]); m.renderOrder = 4;
    this.group.add(m);
    const rim = new THREE.Mesh(new THREE.SphereGeometry(r * 1.25, 12, 8), new THREE.MeshBasicMaterial({ color: 0x222222, side: THREE.BackSide }));
    rim.position.copy(m.position);
    this.group.add(rim);
  }

  // ---------------------------------------------------------------- 镜头
  goView(name, instant) {
    const spec = { oblique: [30, -62, 1.0], top: [89.5, -90, 0.92], side: [3, -90, 1.0] }[name] || [30, -62, 1];
    this.lastView = name; this.userMoved = false;
    const cam = this.camera, half = Math.min(cam.fov / 2, Math.atan(Math.tan(cam.fov / 2 * D2R) * cam.aspect) / D2R);
    const rad = this.mode === 'space' ? Math.hypot(1, (this.info ? this.info.yd : 1), ZH) : Math.hypot(1, ZH + 0.1);
    this._tweenTo(spec[0], spec[1], spec[2] * (rad + 0.1) / Math.sin(half * D2R), instant);
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
    this.controls.target.set(0, 0, 0); this.camera.lookAt(0, 0, 0);
    this.dirty = true;
    if (tt >= 1) this.tween = null;
  }

  _draw() {
    const W = this.canvas.clientWidth, H = this.canvas.clientHeight;
    if (W !== this.W || H !== this.H) {
      this.gl.setSize(W, H, false);
      this.camera.aspect = W / Math.max(H, 1); this.camera.updateProjectionMatrix();
      const dpr = Math.min(window.devicePixelRatio || 1, 3);
      this.overlay.width = Math.round(W * dpr); this.overlay.height = Math.round(H * dpr);
      this.W = W; this.H = H; this.dpr = dpr;
      if (!this.userMoved && !this.tween) this.goView(this.lastView || 'oblique', true);
    }
    this.gl.render(this.scene, this.camera);
    this._drawOverlay();
  }

  /** 刻度与轴名 (覆盖层 2D 画布) */
  _drawOverlay() {
    const o = this.overlay, ctx = o.getContext('2d'), dpr = this.dpr || 1;
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, o.width, o.height);
    const I = this.info; if (!I) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = this.W, H = this.H, placed = [];
    const label = (p, txt, strong) => {
      const v = new THREE.Vector3(...p).project(this.camera);
      if (v.z > 1) return;
      ctx.font = `${strong ? '600 ' : ''}12px system-ui, sans-serif`;
      const hw = ctx.measureText(txt).width / 2 + 4;
      const x = Math.max(hw, Math.min(W - hw, (v.x + 1) / 2 * W)), y = Math.max(10, Math.min(H - 10, (1 - v.y) / 2 * H));
      if (placed.some(b => Math.abs(b[0] - x) < b[2] + hw && Math.abs(b[1] - y) < 15)) return;
      placed.push([x, y, hw]);
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.92)'; ctx.strokeText(txt, x, y);
      ctx.fillStyle = strong ? '#1f2937' : '#4b5563'; ctx.fillText(txt, x, y);
    };
    const t = this.labels || {};
    if (I.kind === 'space') {
      for (let a = 0; a <= 360; a += 90) label(I.map(a, 0, I.lo).map((v, i) => v + [0, -0.08, -0.05][i]), `${t.x || 'α'} ${a}°`, a === 0);
      for (let b = 90; b <= I.yMax; b += 90) label(I.map(0, b, I.lo).map((v, i) => v + [-0.12, 0, -0.05][i]), `${t.y || ''} ${b}°`, false);
      // 第三个角标在后右那条竖棱上 (前面的棱已经挂着 α 的刻度)
      label(I.map(360, I.yMax, I.lo).map((v, i) => v + [0.13, 0.05, 0][i]), `${t.z || ''} ${fmtA(I.lo)}°`, true);
      label(I.map(360, I.yMax, I.hi).map((v, i) => v + [0.13, 0.05, 0][i]), `${t.z || ''} ${fmtA(I.hi)}°`, true);
    } else {
      label([0, 0, I.ground - 0.05], t.sun || '☀', true);
      for (const r of [22, 46]) if (r < I.rmax) label([r * I.s * 0.71, -r * I.s * 0.71, I.ground], `${r}°`, false);
      label([1.04, -1.04 * 0, I.zOf(I.lo)], `${t.z || ''} ${fmtA(I.lo)}°`, true);
      label([1.04, 0, I.zOf(I.hi)], `${t.z || ''} ${fmtA(I.hi)}°`, true);
    }
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

const fmtA = v => String(Math.round(v * 10) / 10);
/** 天空方向 → 以太阳为中心的等距鱼眼 (度), 同 engine/lift.js fisheyeOf */
function fishOf(D, sunDeg, out) {
  const th = sunDeg * D2R, sh = Math.sin(th), ch = Math.cos(th);
  const r = Math.acos(Math.max(-1, Math.min(1, D[0] * ch + D[2] * sh))) / D2R;
  const px = -D[1], py = -D[0] * sh + D[2] * ch, pn = Math.hypot(px, py);
  if (pn < 1e-15) { out[0] = 0; out[1] = 0; return out; }
  out[0] = r * px / pn; out[1] = r * py / pn;
  return out;
}
