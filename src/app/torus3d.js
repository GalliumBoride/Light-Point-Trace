// 取向平面卷成甜甜圈 (环面): 两个取向角都是转一圈回到原处, 平面的左右边、上下边本来就是同一处, 粘起来就是环面.
// 大圈方向 = 横轴 (α 方位等), 绕管子一圈 = 纵轴 (柱晶 roll / Lowitz ζ; ζ 只取 0–180° 时是半根管子).
// 面上的图案就是取向平面本身: 用 2D 渲染器在离屏画布上画一张完整的 360° 视窗 (点云、篱笆、焦散、末端点、高亮、网格), 裁出绘图区贴上去.
// 悬停给读数, 单击 = 取向平面的位置查看.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Renderer, oriArea } from './render.js';

const D2R = Math.PI / 180;
const RING = 1, TUBE = 0.44;              // 中心圈半径, 管半径
const TEX_W = 1000, TEX_DPR = 2;          // 贴图: 横轴 360° 画成 1000 css px (与平时的取向平面差不多大, 线宽、点大小一致), 2 倍像素
const LIGHT = new THREE.Vector3(-0.35, -0.55, 0.76).normalize();

export class TorusView {
  constructor(canvas, overlay) {
    this.canvas = canvas; this.overlay = overlay;
    this.gl = null; this.active = false; this.dirty = true;
    this.texKey = null; this.geoKey = null;
    this.onHover = null; this.onPick = null;
    this.axes = null;
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
    // 贴图: 离屏 2D 渲染器画整张取向平面, 再裁出绘图区
    this.paper = document.createElement('canvas');
    this.paperR = new Renderer(this.paper);
    this.texCanvas = document.createElement('canvas');
    this.tex = new THREE.CanvasTexture(this.texCanvas);
    this.tex.colorSpace = THREE.SRGBColorSpace;
    this.tex.anisotropy = r.capabilities.getMaxAnisotropy();
    // 明暗直接烘进顶点色 (固定的斜上方光), 白底贴图在正面保持接近白色
    this.mat = new THREE.MeshBasicMaterial({ map: this.tex, vertexColors: true, side: THREE.DoubleSide });
    this.mesh = null;
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

  /** 纵轴范围变了 (ζ 整圈 / 半圈) 才重建网格 */
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
    this.scene.add(this.mesh);
  }

  /** 新结果 / 图层 / 高亮 / 位置查看变化 → 重画贴图. V 来自 viewState. */
  setData(R, V) {
    this._init();
    const ax = this.axes = V.axes;
    this._geometry(ax.y0, ax.y1);
    const flags = R ? R.paths.map(p => [p.ori ? 1 : 0, p.lines ? 1 : 0, p.eps ? 1 : 0, p.mirror ? p.mirror.status : '-'].join('')).join('|') + (R.oriLocal ? 'L' : '') : '';
    const key = [R ? R.job : '', ax.x0, ax.x1, ax.y0, ax.y1, V.mc, V.arcs, V.lines, V.endpoints, V.sym, V.lineW, V.mcSize, V.hlFull,
                 [...V.checked].join('|'), V.probe ? V.probe.x + ',' + V.probe.y : '', V.theme.bg.join(','), flags].join(';');
    if (key === this.texKey) return;
    this.texKey = key;
    const xw = ax.x1 - ax.x0, yw = ax.y1 - ax.y0, aw = TEX_W, ah = Math.round(TEX_W / 2 * yw / 360);
    const Vt = { ...V, W: aw + 82, H: ah + 136, dpr: TEX_DPR, view: 1, chrome: false, dragging: false, hoverEp: null, iso: false,
                 xw, yw, xc: (ax.x0 + ax.x1) / 2, yc: (ax.y0 + ax.y1) / 2 };
    this.paperR.render(Vt, R);
    const area = oriArea(Vt.W, Vt.H), tc = this.texCanvas;
    const tw = aw * TEX_DPR, th = ah * TEX_DPR;
    if (tc.width !== tw || tc.height !== th) {
      tc.width = tw; tc.height = th;
      this.tex.dispose();                        // 尺寸变了 (ζ 整圈 ↔ 半圈) 要重新分配 GPU 贴图
    }
    tc.getContext('2d').drawImage(this.paper, area.X * TEX_DPR, area.Y * TEX_DPR, tw, th, 0, 0, tw, th);
    this.tex.needsUpdate = true;
    this.scene.background = new THREE.Color(`rgb(${V.theme.bg.join(',')})`);
    this.dirty = true;
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
      if (!this.userMoved && !this.tween) this.goView(this.lastView || 'oblique', true);
    }
    this.gl.render(this.scene, this.camera);
    this._drawOverlay();
  }

  /** 环面上 (横轴°, 纵轴°) 处的点与外法向 */
  _at(a, v) {
    const u = a * D2R, w = v * D2R;
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
  /** 刻度: 横轴沿纵轴起点那一圈 (外赤道) 标, 纵轴沿横轴 0° 那一圈管子标; 只标看得见、不和已标的重叠的 */
  _drawOverlay() {
    const o = this.overlay, ctx = o.getContext('2d'), dpr = this.dpr || 1;
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, o.width, o.height);
    if (!this.axes) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = this.W, H = this.H, ax = this.axes;
    const placed = [];
    const label = (a, v, txt, out, strong) => {
      const { p, n } = this._at(a, v), q = p.addScaledVector(n, out);
      if (this._hidden(q)) return;
      const s = q.project(this.camera);
      if (s.z > 1) return;
      ctx.font = `${strong ? '600 ' : ''}12px system-ui, sans-serif`;
      const hw = ctx.measureText(txt).width / 2 + 4;                 // 轮廓边上的刻度别被画布边裁掉
      const x = Math.max(hw, Math.min(W - hw, (s.x + 1) / 2 * W)), y = Math.max(10, Math.min(H - 10, (1 - s.y) / 2 * H));
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
