// 「3D 抬升」视图: 取向平面上光路成立的区域抬到天空上方 (水平位置 = 光落在天上哪, 高度 = 第二个取向角),
// 地面 = 以太阳为中心的等距鱼眼天空, 上面是曲面的影子 (均匀随机姿态的落点密度).
// 几何在 Worker 里算 (engine/lift.js), 这里只按「抬升」「高度比例」摆位置并画出来 (three.js, 打包进单文件).
// 地面可选「天球」: 地面换成天球本身, 高度沿径向 (每一点仍在它把光送去的天空方向上), 没有鱼眼变形.
// 柱晶 ζ = 90°、光路 3-5 时, 曲面就是"甜甜圈影子"里抬起来的那片透镜.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { drawEpGlyphOn } from './render.js';
import { t } from './i18n.js';
import { haloDeg } from '../engine/lp.js';

const SKY = 10;                  // 天空 10° = 场景 1 个单位
const Z0 = 2.0;                  // 高度零点 (折痕) 抬到的高度; 曲面最低处不够高时整体再抬
const ZS = 0.008, ZG = 0.004;    // 曲面、地面线的微小抬高, 免得和地面打架
const RSPH = 180 / (Math.PI * SKY);   // 天球地面的半径: 球面上 10° 弧长 = 1 个单位, 与平面地面同尺度
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const SUN = { th: 10 * D2R };    // 当前显示结果的太阳高度 (不跟着正在算的新结果变)
let LIFT_ID = 0;

/** 3D 视图里各类边界线的颜色 (同甜甜圈影子视频: 折痕金、出射临界青). 右栏图例在 3D 视图时也用它. */
export const KIND3D = {
  0: [63, 70, 84],      // 入射掠射
  1: [214, 51, 138],    // 反射面退化 q = 0
  2: [47, 158, 68],     // 全反射临界 / 蓝边
  3: [11, 143, 196],    // 出射临界
  6: [211, 155, 0],     // 焦散 J = 0 (折痕)
  7: [140, 86, 75],     // 孔径边界
};
const SURF = [[255, 138, 61], [107, 140, 255]];       // 折痕两侧: 橙 (J ≥ 0) / 紫
const SLATE = [[233, 236, 244], [192, 199, 218], [116, 127, 157], [44, 52, 74]];

const surfVert = /* glsl */`
attribute float jv;
varying float vJ;
varying vec3 vN;
void main() {
  vJ = jv;
  vN = normalize(normalMatrix * normal);
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;
const surfFrag = /* glsl */`
uniform vec3 colA;
uniform vec3 colB;
uniform float opacity;
varying float vJ;
varying vec3 vN;
void main() {
  vec3 n = normalize(vN);
  float d = abs(dot(n, normalize(vec3(-0.35, 0.55, 0.75))));   // 双面一样亮
  vec3 base = vJ >= 0.0 ? colA : colB;
  gl_FragColor = vec4(min(base * (0.6 + 0.48 * d), vec3(1.0)), opacity);
}`;

// 天球地面: 逐像素算鱼眼坐标去查影子贴图 (贴图范围外用底色), 地平线以下略深
const globeVert = /* glsl */`
varying vec3 vP;
varying vec3 vN;
void main() {
  vP = position;
  vN = normalize(normalMatrix * normal);
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;
const globeFrag = /* glsl */`
uniform sampler2D map;
uniform vec4 ext;
uniform float sunTh;
varying vec3 vP;
varying vec3 vN;
void main() {
  vec3 d = normalize(vP);
  float sh = sin(sunTh), ch = cos(sunTh);
  float r = degrees(acos(clamp(d.x * ch + d.z * sh, -1.0, 1.0)));
  vec2 p = vec2(-d.y, -d.x * sh + d.z * ch);
  float pn = length(p);
  vec2 g = pn > 1e-9 ? r * p / pn : vec2(0.0);
  vec2 uv = vec2((g.x - ext.x) / (ext.y - ext.x), (g.y - ext.z) / (ext.w - ext.z));
  vec3 col = vec3(0.953, 0.957, 0.973);
  if (uv.x > 0.0 && uv.x < 1.0 && uv.y > 0.0 && uv.y < 1.0) col = texture2D(map, uv).rgb;
  if (d.z < 0.0) col *= 0.92;
  float lit = 0.88 + 0.12 * max(dot(normalize(vN), normalize(vec3(-0.35, 0.55, 0.75))), 0.0);
  gl_FragColor = vec4(col * lit, 1.0);
}`;

const mix = (a, b, t) => a.map((v, i) => Math.round(v + (b[i] - v) * t));
const rgb = c => `rgb(${c[0]},${c[1]},${c[2]})`;

export class View3D {
  constructor(canvas, overlay) {
    this.canvas = canvas;
    this.overlay = overlay;
    this.gl = null;              // 第一次切到 3D 才建 WebGL
    this.lift = 1; this.kz = 1.6; this.anim = null;
    this.base = 'flat';          // 地面: 'flat' 以太阳为中心的鱼眼天空图 / 'sphere' 天球
    this.paths = [];             // 每条光路: { data, mesh, lines, ground, drops, eps }
    this.key = null;
    this.onHover = null;
    this.hover = null;
    this.dirty = true;
    this.active = false;
  }

  _init() {
    if (this.gl) return;
    const r = this.gl = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff);
    const cam = this.camera = new THREE.PerspectiveCamera(30, 1, 0.05, 600);
    cam.up.set(0, 0, 1);
    cam.position.set(4, -14, 5);
    const ctl = this.controls = new OrbitControls(cam, this.canvas);
    ctl.enableDamping = true; ctl.dampingFactor = 0.09; ctl.screenSpacePanning = true;
    ctl.addEventListener('change', () => { this.dirty = true; });
    ctl.addEventListener('start', () => { this.userMoved = true; this.tween = null; });
    this.root = new THREE.Group();
    scene.add(this.root);
    this.lineMats = [];
    // 地面: 影子 + 太阳、22° 晕、地平线 (画在一张贴图上)
    this.gcv = document.createElement('canvas');
    this.ground = new THREE.Mesh(new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, toneMapped: false, polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 2 }));
    scene.add(this.ground);
    this.globe = new THREE.Mesh(new THREE.SphereGeometry(RSPH, 192, 96).rotateX(Math.PI / 2),
      new THREE.ShaderMaterial({ vertexShader: globeVert, fragmentShader: globeFrag,
        uniforms: { map: { value: null }, ext: { value: new THREE.Vector4(-30, 30, -30, 30) }, sunTh: { value: SUN.th } },
        polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 2 }));
    this.globe.visible = false;
    scene.add(this.globe);
    this.ray = new THREE.Raycaster();
    this.canvas.addEventListener('pointermove', e => { this.hoverReq = e; });
    this.canvas.addEventListener('pointerleave', () => { this.hoverReq = 'off'; });
    if (this.base === 'sphere') this.goView('proto', true);        // 曲面还没算好时也先看到整个天球
    const loop = () => {
      requestAnimationFrame(loop);
      if (!this.active) return;
      if (this.anim) this._stepAnim();
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
    const m = new LineMaterial({ color: new THREE.Color(rgb(col)).getHex(), linewidth: width, worldUnits: false,
                                 dashed: !!o.dashed, dashSize: 0.07, gapSize: 0.06,
                                 transparent: o.opacity !== undefined, opacity: o.opacity ?? 1 });
    this.lineMats.push(m);
    return m;
  }

  /** 新结果 (或新的抬升数据) 到达: 重建场景. V = 当前视图状态. */
  setData(R, V) {
    this._init();
    const lifts = R ? R.paths.map(p => p.lift || null) : [];
    for (const l of lifts) if (l && !l._id) l._id = ++LIFT_ID;          // 每份网格一个编号: 换了网格才重建场景
    const key = R ? R.job + '|' + lifts.map(l => (l ? l._id : 0)).join(',') : '';
    if (key === this.key) return;
    this.key = key;
    if (R) SUN.th = R.params.sun * D2R;
    for (const o of [...this.root.children]) { this.root.remove(o); o.geometry && o.geometry.dispose(); }
    for (const m of this.lineMats) m.dispose();
    this.lineMats = [];
    this.paths = [];
    if (!R) return;
    const multi = R.paths.length > 1;
    R.paths.forEach((pr, k) => {
      const d = pr.lift;
      if (!d || (!d.n && !(d.lines || []).some(l => l.seg.length))) { this.paths.push(null); return; }
      const pc = V.theme.pathcol[k % V.theme.pathcol.length];
      const cols = multi ? [mix(pc, [255, 255, 255], 0.35), mix(pc, [0, 0, 0], 0.18)] : SURF;
      let mesh = null;
      if (d.n) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * d.n), 3));
        geo.setAttribute('jv', new THREE.BufferAttribute(d.jv, 1));
        geo.setIndex(new THREE.BufferAttribute(d.tris, 1));
        const mat = new THREE.ShaderMaterial({
          vertexShader: surfVert, fragmentShader: surfFrag, side: THREE.DoubleSide,
          uniforms: { colA: { value: new THREE.Vector3(...cols[0].map(v => v / 255)) },
                      colB: { value: new THREE.Vector3(...cols[1].map(v => v / 255)) }, opacity: { value: 1 } },
          polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
        });
        mesh = new THREE.Mesh(geo, mat);
        mesh.userData.k = k;
        this.root.add(mesh);
      }
      // 1DOF 没有曲面, 只有一条抬起来的空间曲线 (kind = −1, 用光路颜色)
      const lines = d.lines.map(l => {
        const col = l.kind < 0 ? pc : KIND3D[l.kind] || [0, 0, 0];
        const up = new LineSegments2(new LineSegmentsGeometry(), this._mat(col, 2));
        const dn = new LineSegments2(new LineSegmentsGeometry(), this._mat(col, 1.4, { opacity: 0.75 }));
        up.frustumCulled = dn.frustumCulled = false;
        this.root.add(up, dn);
        return { l, up, dn };
      });
      const drops = new LineSegments2(new LineSegmentsGeometry(), this._mat(KIND3D[6], 1.1, { dashed: true }));
      drops.frustumCulled = false;
      this.root.add(drops);
      this.paths.push({ pr, d, mesh, lines, drops, cols });
    });
    let hvMin = 0;
    for (const p of this.paths) {
      if (!p) continue;
      for (const h of p.d.hv) if (h < hvMin) hvMin = h;
      for (const l of p.d.lines) for (let i = 2; i < l.seg.length; i += 3) if (l.seg[i] < hvMin) hvMin = l.seg[i];
    }
    this.hvMin = hvMin;
    this._buildGlobeDeco();
    this._groundTexture(V);
    this.layoutKey = null;
    this.layout(V);
    // 新结果: 用户没动过镜头就重新取景 (换光路后范围常常差很多); 动过就保留他的视角
    if (this.paths.some(Boolean) && (!this.fitted || !this.userMoved)) { this.goView('proto', !this.fitted); this.fitted = true; }
  }

  /** 高度零点: 至少 2, 并保证曲面最低处在地面上方 0.3. */
  z0() { return Math.max(Z0, 0.3 - this.kz * this.hvMin); }
  zOf(hv) { return ZS + this.lift * (this.z0() + this.kz * hv); }

  /** 只改位置 (抬升 / 高度比例 / 图层开关 / 高亮的族). */
  layout(V) {
    if (!this.gl) return;
    this.V = V;
    const lk = [this.base, this.lift, this.kz, V.lineW, V.lines, V.arcs, V.drops, V.mc, [...V.checked].join('|')].join(',');
    if (lk === this.layoutKey) { this.dirty = true; return; }
    this.layoutKey = lk;
    const sph = this.base === 'sphere', q = [0, 0, 0];
    for (const p of this.paths) {
      if (!p) continue;
      const { d, mesh } = p;
      // 放平时两半叠在同一块地上, 紫面 (J < 0) 略垫高一点, 免得两层面互相打架闪烁
      const flat = 0.012 * (1 - this.lift);
      if (mesh) {
        const pos = mesh.geometry.attributes.position.array;
        if (sph && !p.dirs) {                    // 天球: 每个顶点的天空方向只算一次
          p.dirs = new Float32Array(3 * d.n);
          for (let k = 0; k < d.n; k++) p.dirs.set(unfisheye(d.g[2 * k], d.g[2 * k + 1]), 3 * k);
        }
        for (let k = 0; k < d.n; k++) {
          const z = this.zOf(d.hv[k]) + (d.jv[k] < 0 ? flat : 0);
          if (sph) {
            const r = RSPH + z;
            pos[3 * k] = r * p.dirs[3 * k]; pos[3 * k + 1] = r * p.dirs[3 * k + 1]; pos[3 * k + 2] = r * p.dirs[3 * k + 2];
          } else {
            pos[3 * k] = d.g[2 * k] / SKY; pos[3 * k + 1] = d.g[2 * k + 1] / SKY; pos[3 * k + 2] = z;
          }
        }
        mesh.geometry.attributes.position.needsUpdate = true;
        mesh.geometry.computeVertexNormals();
        mesh.geometry.computeBoundingSphere();
        mesh.visible = V.arcs;
      }
      for (const { l, up, dn } of p.lines) {
        const s = l.seg, a = new Float32Array(s.length), b = new Float32Array(s.length);
        for (let i = 0; i < s.length; i += 3) {
          a.set(this._at(s[i], s[i + 1], this.zOf(s[i + 2]) + 0.004 + 1.2 * flat, q), i);
          b.set(this._at(s[i], s[i + 1], ZG, q), i);
        }
        const curve = l.kind < 0, full = curve || V.checked.has(V.famKey(p.pr, l.name));
        for (const [obj, arr, w] of [[up, a, full ? 2 * V.lineW : V.lineW], [dn, b, full ? 1.4 * V.lineW : 0.7 * V.lineW]]) {
          obj.geometry.dispose();
          obj.geometry = new LineSegmentsGeometry();
          if (arr.length) obj.geometry.setPositions(arr);
          obj.material.linewidth = w;
          obj.visible = (curve ? V.arcs : V.lines) && arr.length > 0;
        }
        up.visible = up.visible && V.arcs;
      }
      // 折痕到地面的竖直投影线: 整条折痕 (地面上的长度) 大约分成 16 份, 每份一根, 至少隔 4°
      const dr = [];
      let foldLen = 0;
      for (const { l } of p.lines)
        if (l.kind === 6) for (let i = 0; i < l.seg.length; i += 6) foldLen += Math.hypot(l.seg[i + 3] - l.seg[i], l.seg[i + 4] - l.seg[i + 1]);
      const gap = Math.max(4, foldLen / 16);
      for (const { l } of p.lines) {
        if (l.kind !== 6) continue;
        const s = l.seg;
        let acc = gap;
        for (let i = 0; i < s.length; i += 6) {
          acc += Math.hypot(s[i + 3] - s[i], s[i + 4] - s[i + 1]);
          if (acc < gap) continue;
          acc = 0;
          dr.push(...this._at(s[i], s[i + 1], this.zOf(s[i + 2]), q), ...this._at(s[i], s[i + 1], ZG, q));
        }
      }
      p.drops.geometry.dispose();
      p.drops.geometry = new LineSegmentsGeometry();
      if (dr.length) { p.drops.geometry.setPositions(dr); p.drops.computeLineDistances(); }
      p.drops.visible = V.drops && V.arcs && dr.length > 0 && this.lift > 0.05;
    }
    this.ground.visible = !sph;
    this.globe.visible = sph;
    for (const o of this.root.children) if (o.userData.globe) o.visible = sph;
    if (this._groundMc !== V.mc) this._groundTexture(V);
    this.dirty = true;
  }

  /** 地面换成平面 / 天球: 重摆位置并重新取景. */
  setBase(b) {
    if (b === this.base) return;
    this.base = b;
    this.layoutKey = null;
    if (!this.gl) return;
    if (this.V) this.layout(this.V);
    this.goView('proto');
  }

  /** 鱼眼坐标 (度) + 离地高度 → 场景坐标. 平面: 地面 = 鱼眼天空图, 高度朝上; 天球: 方向不变, 高度沿径向. */
  _at(gx, gy, z, out) {
    if (this.base === 'sphere') {
      const D = unfisheye(gx, gy), r = RSPH + z;
      out[0] = r * D[0]; out[1] = r * D[1]; out[2] = r * D[2];
    } else { out[0] = gx / SKY; out[1] = gy / SKY; out[2] = z; }
    return out;
  }

  /** 天球地面上的经纬网、地平线、22° 晕、太阳 (随太阳高度变, 每份新结果重建; 只在天球地面时显示). */
  _buildGlobeDeco() {
    const r = RSPH + ZG, h = SUN.th, A = [Math.cos(h), 0, Math.sin(h)], b1 = [-Math.sin(h), 0, Math.cos(h)], b2 = [0, 1, 0];
    const add = o => { o.userData.globe = true; o.visible = this.base === 'sphere'; o.frustumCulled = false; this.root.add(o); return o; };
    const line = (pts, col, w, o) => {
      const g = new LineSegmentsGeometry();
      g.setPositions(pts);
      const l = new LineSegments2(g, this._mat(col, w, o));
      if (o && o.dashed) l.computeLineDistances();
      return add(l);
    };
    const circ = (fn, n) => { const out = []; for (let i = 0; i < n; i++) out.push(...fn(i / n), ...fn((i + 1) / n)); return out; };
    const grid = [];
    for (let k = 0; k < 12; k++) {
      const az = k * 30 * D2R;
      grid.push(...circ(u => { const el = (u - 0.5) * Math.PI; return [r * Math.cos(el) * Math.cos(az), r * Math.cos(el) * Math.sin(az), r * Math.sin(el)]; }, 90));
    }
    for (const el of [-60, -30, 30, 60]) {
      const ce = Math.cos(el * D2R), se = Math.sin(el * D2R);
      grid.push(...circ(u => [r * ce * Math.cos(2 * Math.PI * u), r * ce * Math.sin(2 * Math.PI * u), r * se], 180));
    }
    line(grid, [150, 160, 178], 0.8, { opacity: 0.45 });
    line(circ(u => [r * Math.cos(2 * Math.PI * u), r * Math.sin(2 * Math.PI * u), 0], 360), [46, 139, 87], 1.8);
    const c22 = Math.cos(haloDeg() * D2R), s22 = Math.sin(haloDeg() * D2R);
    if (Number.isFinite(c22)) line(circ(u => [0, 1, 2].map(i => r * (c22 * A[i] + s22 * (Math.cos(2 * Math.PI * u) * b1[i] + Math.sin(2 * Math.PI * u) * b2[i]))), 240),
         [127, 136, 157], 1.3, { dashed: true });
    const sun = add(new THREE.Mesh(new THREE.SphereGeometry(0.12, 20, 14), new THREE.MeshBasicMaterial({ color: 0xf29a1a })));
    sun.position.set(...A.map(v => v * RSPH));
  }

  // ---------- 地面 ----------
  _groundTexture(V) {
    this._groundMc = V.mc;
    let x0 = -30, x1 = 30, y0 = -30, y1 = 30;
    for (const p of this.paths) {
      if (!p) continue;
      const h = p.d.hist;
      x0 = Math.min(x0, h.x0); x1 = Math.max(x1, h.x1); y0 = Math.min(y0, h.y0); y1 = Math.max(y1, h.y1);
    }
    const ppd = Math.min(14, 2048 / Math.max(x1 - x0, y1 - y0));
    const cv = this.gcv, W = Math.round((x1 - x0) * ppd), H = Math.round((y1 - y0) * ppd);
    cv.width = W; cv.height = H;
    const ctx = cv.getContext('2d');
    const X = x => (x - x0) * ppd, Y = y => (y1 - y) * ppd;
    ctx.fillStyle = '#f3f4f8';
    ctx.fillRect(0, 0, W, H);
    if (V.mc)
      for (const p of this.paths) {
        if (!p) continue;
        const h = p.d.hist, n = h.nx * h.ny, nz = [];
        for (let q = 0; q < n; q++) if (h.data[q] > 0) nz.push(h.data[q]);
        if (!nz.length) continue;
        nz.sort((a, b) => a - b);
        const sc = Math.log1p(nz[Math.floor(0.995 * (nz.length - 1))]);
        const tmp = document.createElement('canvas');
        tmp.width = h.nx; tmp.height = h.ny;
        const tctx = tmp.getContext('2d'), img = tctx.createImageData(h.nx, h.ny), px = img.data;
        for (let by = 0; by < h.ny; by++)
          for (let bx = 0; bx < h.nx; bx++) {
            const c = h.data[by * h.nx + bx], o = 4 * ((h.ny - 1 - by) * h.nx + bx);
            if (c <= 0) continue;
            const t = Math.min(Math.log1p(c) / sc, 1) * (SLATE.length - 1), i = Math.min(SLATE.length - 2, Math.floor(t));
            const col = mix(SLATE[i], SLATE[i + 1], t - i);
            px[o] = col[0]; px[o + 1] = col[1]; px[o + 2] = col[2]; px[o + 3] = 255;
          }
        tctx.putImageData(img, 0, 0);
        ctx.save();
        ctx.globalCompositeOperation = 'multiply';
        ctx.imageSmoothingEnabled = true; ctx.imageSmoothingQuality = 'high';
        ctx.drawImage(tmp, X(h.x0), Y(h.y1), (h.x1 - h.x0) * ppd, (h.y1 - h.y0) * ppd);
        ctx.restore();
      }
    // 天球地面用不带标注的这一份 (地平线、22° 晕、太阳在球上另画成 3D 线)
    const gc = this.gcv2 || (this.gcv2 = document.createElement('canvas'));
    gc.width = W; gc.height = H;
    gc.getContext('2d').drawImage(cv, 0, 0);
    // 22° 晕 (最小偏向角, n = 1.31 时 21.84°)、地平线、太阳
    ctx.save();
    ctx.strokeStyle = '#7f889d'; ctx.lineWidth = Math.max(1.5, ppd * 0.12); ctx.setLineDash([ppd * 0.9, ppd * 0.7]);
    if (Number.isFinite(haloDeg())) { ctx.beginPath(); ctx.arc(X(0), Y(0), haloDeg() * ppd, 0, 2 * Math.PI); ctx.stroke(); }
    ctx.strokeStyle = 'rgb(46,139,87)'; ctx.setLineDash([ppd * 0.3, ppd * 0.6]); ctx.lineWidth = Math.max(1.5, ppd * 0.14);
    ctx.beginPath();
    let pen = false;
    const f = [0, 0];
    for (let t = -180; t <= 180; t += 0.5) {
      const Dx = Math.cos(t * D2R), Dy = Math.sin(t * D2R);
      if (Math.acos(Dx * Math.cos(SUN.th)) * R2D > 150) { pen = false; continue; }
      fisheye(Dx, Dy, 0, f);
      if (pen) ctx.lineTo(X(f[0]), Y(f[1])); else ctx.moveTo(X(f[0]), Y(f[1]));
      pen = true;
    }
    ctx.stroke();
    ctx.restore();
    ctx.fillStyle = '#f29a1a';
    ctx.beginPath(); ctx.arc(X(0), Y(0), Math.max(5, 1.1 * ppd), 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = '#cfd4df'; ctx.lineWidth = 3; ctx.strokeRect(1.5, 1.5, W - 3, H - 3);
    const tex = new THREE.CanvasTexture(cv);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = this.gl.capabilities.getMaxAnisotropy();
    const m = this.ground.material;
    if (m.map) m.map.dispose();
    m.map = tex; m.needsUpdate = true;
    this.ground.geometry.dispose();
    this.ground.geometry = new THREE.PlaneGeometry((x1 - x0) / SKY, (y1 - y0) / SKY);
    this.ground.position.set((x0 + x1) / 2 / SKY, (y0 + y1) / 2 / SKY, 0);
    this.gext = { x0, x1, y0, y1 };
    const gu = this.globe.material.uniforms;
    if (gu.map.value) gu.map.value.dispose();
    gu.map.value = new THREE.CanvasTexture(gc);        // 不标 sRGB: 着色器直接输出贴图里的颜色
    gu.map.value.anisotropy = tex.anisotropy;
    gu.ext.value.set(x0, x1, y0, y1);
    gu.sunTh.value = SUN.th;
    this.dirty = true;
  }

  // ---------- 抬升动画 / 视角 ----------
  toggleLift() {
    this.anim = { from: this.lift, to: this.lift > 0.5 ? 0 : 1, t0: performance.now(), dur: 1400 };
    this.dirty = true;
    return this.anim.to;
  }
  _stepAnim() {
    const a = this.anim, t = Math.min((performance.now() - a.t0) / a.dur, 1);
    const e = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    this.lift = a.from + (a.to - a.from) * e;
    if (t >= 1) this.anim = null;
    this.layout(this.V);
  }

  _box() {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, hv1 = 0;
    for (const p of this.paths) {
      if (!p) continue;
      const b = p.d.bbox;
      x0 = Math.min(x0, b.x0); x1 = Math.max(x1, b.x1); y0 = Math.min(y0, b.y0); y1 = Math.max(y1, b.y1);
      for (const h of p.d.hv) if (h > hv1) hv1 = h;
      for (const l of p.d.lines) for (let i = 2; i < l.seg.length; i += 3) if (l.seg[i] > hv1) hv1 = l.seg[i];
    }
    if (!Number.isFinite(x0)) { x0 = -30; x1 = 30; y0 = -30; y1 = 30; }
    return { x0: x0 / SKY, x1: x1 / SKY, y0: y0 / SKY, y1: y1 / SKY, z1: this.zOf(hv1) + 0.3 };
  }
  /** 天球地面: 曲面 (按当前抬升) 连同它在球上的脚点的包围盒 → 中心、半径. */
  _sphereBox() {
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity], q = [0, 0, 0];
    const addP = (gx, gy, z) => {
      this._at(gx, gy, z, q);
      for (let i = 0; i < 3; i++) { if (q[i] < lo[i]) lo[i] = q[i]; if (q[i] > hi[i]) hi[i] = q[i]; }
    };
    for (const p of this.paths) {
      if (!p) continue;
      const d = p.d, st = Math.max(1, Math.floor(d.n / 3000));
      for (let k = 0; k < d.n; k += st) { addP(d.g[2 * k], d.g[2 * k + 1], this.zOf(d.hv[k])); addP(d.g[2 * k], d.g[2 * k + 1], 0); }
      for (const l of d.lines)
        for (let i = 0; i < l.seg.length; i += 3) { addP(l.seg[i], l.seg[i + 1], this.zOf(l.seg[i + 2])); addP(l.seg[i], l.seg[i + 1], 0); }
    }
    if (!Number.isFinite(lo[0])) addP(0, 0, 0);
    return { c: new THREE.Vector3((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2),
             r: 0.5 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) };
  }
  goView(name, instant) {
    if (!this.gl) return;
    const cam = this.camera, vf = cam.fov * D2R, hf = 2 * Math.atan(Math.tan(vf / 2) * cam.aspect);
    let spec, dist, target;
    if (this.base === 'sphere') {
      // 斜看 = 从曲面所在方向偏开一些; 侧面 = 与它垂直的水平方向 (高度看得最清楚); 俯视 = 正对它
      const b = this._sphereBox();
      const u = b.c.length() > 0.5 ? b.c.clone().normalize() : new THREE.Vector3(Math.cos(SUN.th), 0, Math.sin(SUN.th));
      const elu = Math.asin(u.z) * R2D, azu = Math.atan2(u.y, u.x) * R2D;
      spec = { proto: [Math.min(elu + 22, 80), azu - 50], side: [4, azu - 90], top: [Math.max(-89.5, Math.min(89.5, elu)), azu] }[name];
      dist = Math.max(b.r, 0.75 * RSPH) / Math.sin(Math.min(vf, hf) / 2) * 1.04;     // 小弧也留出一截球面作参照
      target = b.c;
    } else {
      const b = this._box(), cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2;
      const R = 0.5 * Math.hypot(b.x1 - b.x0, b.y1 - b.y0, b.z1);
      dist = R / Math.sin(Math.min(vf, hf) / 2) * 1.02;
      spec = { proto: [14, -70, b.z1 * 0.45], side: [6, 0, b.z1 * 0.5], top: [89.5, -90, 0] }[name];
      target = new THREE.Vector3(cx, cy, spec[2]);
    }
    const off = cam.position.clone().sub(this.controls.target);
    const from = { dist: off.length(), el: Math.asin(off.z / Math.max(off.length(), 1e-9)) * R2D, az: Math.atan2(off.y, off.x) * R2D,
                   target: this.controls.target.clone() };
    const daz = ((spec[1] - from.az + 540) % 360) - 180;
    this.tween = { t0: performance.now(), dur: instant ? 0 : 750, from,
                   to: { el: spec[0], az: from.az + daz, dist, target } };
    this.userMoved = false;
    this._stepTween();
    this.dirty = true;
  }
  _stepTween() {
    const w = this.tween, t = w.dur ? Math.min((performance.now() - w.t0) / w.dur, 1) : 1;
    const e = t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2, L = (a, b) => a + (b - a) * e;
    const el = L(w.from.el, w.to.el) * D2R, az = L(w.from.az, w.to.az) * D2R, dist = L(w.from.dist, w.to.dist);
    this.controls.target.lerpVectors(w.from.target, w.to.target, e);
    this.camera.position.set(this.controls.target.x + dist * Math.cos(el) * Math.cos(az),
                             this.controls.target.y + dist * Math.cos(el) * Math.sin(az), this.controls.target.z + dist * Math.sin(el));
    this.controls.update();
    this.dirty = true;
    if (t >= 1) this.tween = null;
  }

  // ---------- 画 ----------
  resize(W, H) {
    if (!this.gl) return;
    this.gl.setSize(W, H, false);
    this.camera.aspect = W / Math.max(H, 1);
    this.camera.updateProjectionMatrix();
    for (const m of this.lineMats) m.resolution.set(W, H);
    const dpr = Math.min(window.devicePixelRatio || 1, 3);
    this.overlay.width = Math.round(W * dpr); this.overlay.height = Math.round(H * dpr);
    this.W = W; this.H = H; this.dpr = dpr;
    this.dirty = true;
  }
  requestDraw() { this.dirty = true; }

  _draw() {
    const W = this.canvas.clientWidth, H = this.canvas.clientHeight;
    if (W !== this.W || H !== this.H) this.resize(W, H);
    for (const m of this.lineMats) m.resolution.set(W, H);
    this.gl.render(this.scene, this.camera);
    this._drawOverlay();
  }

  _proj(x, y, z, out) {
    const v = this._v || (this._v = new THREE.Vector3());
    v.set(x, y, z).project(this.camera);
    if (v.z > 1 || v.z < -1) return false;
    out[0] = (v.x + 1) / 2 * this.W; out[1] = (1 - v.y) / 2 * this.H;
    return true;
  }
  /** 鱼眼坐标 + 高度 → 屏幕; 天球地面时被球挡在背后的点不画. */
  _projAt(gx, gy, z, out) {
    const p = this._at(gx, gy, z, this._p3 || (this._p3 = [0, 0, 0]));
    if (this.base === 'sphere') {
      // 从镜头到这一点的线段先碰到球 (略缩小一点, 球面上的点自己不算) → 在背面
      const c = this.camera.position, dx = p[0] - c.x, dy = p[1] - c.y, dz = p[2] - c.z;
      const a = dx * dx + dy * dy + dz * dz, b = 2 * (c.x * dx + c.y * dy + c.z * dz), cc = c.lengthSq() - 0.999 * RSPH * RSPH;
      const disc = b * b - 4 * a * cc;
      if (disc > 0) { const t = (-b - Math.sqrt(disc)) / (2 * a); if (t > 0 && t < 1) return false; }
    }
    return this._proj(p[0], p[1], p[2], out);
  }

  _drawOverlay() {
    const ctx = this.overlay.getContext('2d'), V = this.V;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.overlay.width, this.overlay.height);
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    if (!V) return;
    const s = [0, 0];
    // 地面标注
    ctx.font = '12px "PingFang SC", "Microsoft YaHei UI", "Microsoft YaHei", system-ui, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    const lab = (x, y, text, col) => {
      if (!this._projAt(x, y, 0, s)) return;
      ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.9)'; ctx.strokeText(text, s[0], s[1] - 7);
      ctx.fillStyle = col; ctx.fillText(text, s[0], s[1] - 7);
    };
    lab(0, 0, t('v3.sunLabel'), '#c46f00');
    const g = this.gext;
    if (g) {
      const a = -38 * D2R;
      if (Number.isFinite(haloDeg())) lab(haloDeg() * Math.cos(a), haloDeg() * Math.sin(a), t('v3.haloLabel'), '#5d6679');
      const f = [0, 0];
      for (const deg of [62, 50, 40, 30, 20]) {
        fisheye(Math.cos(deg * D2R), -Math.sin(deg * D2R), 0, f);   // 太阳右侧
        if (this.base === 'sphere' || (f[0] > g.x0 + 3 && f[0] < g.x1 - 3 && f[1] > g.y0 + 3 && f[1] < g.y1 - 3)) { lab(f[0], f[1], t('v3.horizonLabel'), 'rgb(46,139,87)'); break; }
      }
    }
    // 末端点 (与天空 / 取向平面视图同一套符号)
    const multi = this.paths.length > 1;
    this.paths.forEach((p, k) => {
      if (!p || !p.pr.eps || !p.d.eps) return;
      const rim = multi ? rgb(V.theme.pathcol[k % V.theme.pathcol.length]) : '#000';
      p.pr.eps.forEach((r, i) => {
        const e = p.d.eps[i];
        if (!e || !r.alive) return;
        const hov = V.hoverEp && V.hoverEp.k === k && V.hoverEp.i === i;
        if (!V.endpoints && !hov) return;
        if (!this._projAt(e.gx, e.gy, this.zOf(e.hv), s)) return;
        drawEpGlyphOn(ctx, s[0], s[1], r.kind, rim);
        if (hov) {
          ctx.save(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; ctx.setLineDash([3, 2.5]);
          ctx.beginPath(); ctx.arc(s[0], s[1], 12, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
        }
      });
    });
    // 鼠标指着的点: 曲面上的点 + 竖直落到地面的位置
    const h = this.hover;
    if (h) {
      const top = [0, 0], bot = [0, 0];
      const okB = this._projAt(h.gx, h.gy, ZG, bot);
      const okT = h.onSurf && this._projAt(h.gx, h.gy, h.z, top);
      ctx.save();
      ctx.strokeStyle = '#111'; ctx.fillStyle = '#111'; ctx.lineWidth = 1.2;
      if (okT && okB) { ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(top[0], top[1]); ctx.lineTo(bot[0], bot[1]); ctx.stroke(); ctx.setLineDash([]); }
      if (okT) { ctx.beginPath(); ctx.arc(top[0], top[1], 3.5, 0, 2 * Math.PI); ctx.fill(); }
      if (okB) { ctx.beginPath(); ctx.ellipse(bot[0], bot[1], 6, 3.5, 0, 0, 2 * Math.PI); ctx.stroke(); }
      ctx.restore();
    }
  }

  // ---------- 鼠标读数 ----------
  _doHover() {
    const e = this.hoverReq;
    this.hoverReq = null;
    const prev = this.hover;
    this.hover = null;
    if (e !== 'off') {
      const r = this.canvas.getBoundingClientRect();
      const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      this.ray.setFromCamera(ndc, this.camera);
      const meshes = this.paths.filter(p => p && p.mesh && p.mesh.visible).map(p => p.mesh);
      let hit = meshes.length ? this.ray.intersectObjects(meshes, false)[0] : null;
      const sph = this.base === 'sphere';
      const gp = sph ? this.ray.ray.intersectSphere(new THREE.Sphere(new THREE.Vector3(), RSPH), new THREE.Vector3()) : null;
      if (hit && gp && gp.distanceTo(this.ray.ray.origin) < hit.distance) hit = null;      // 曲面在球背后
      if (hit && hit.face) {
        const p = this.paths[hit.object.userData.k], d = p.d, f = hit.face;
        const pos = hit.object.geometry.attributes.position;
        const A = new THREE.Vector3().fromBufferAttribute(pos, f.a), B = new THREE.Vector3().fromBufferAttribute(pos, f.b),
              C = new THREE.Vector3().fromBufferAttribute(pos, f.c);
        const w = new THREE.Triangle(A, B, C).getBarycoord(hit.point, new THREE.Vector3());
        const I = (arr, st, o) => w.x * arr[st * f.a + o] + w.y * arr[st * f.b + o] + w.z * arr[st * f.c + o];
        // 取向角在网格顶点间可能跨 0/360 接缝: 以 a 点为基准展开后再插值
        const ang = o => {
          const a0 = d.uv[2 * f.a + o], un = v => a0 + ((v - a0 + 540) % 360 - 180);
          return w.x * a0 + w.y * un(d.uv[2 * f.b + o]) + w.z * un(d.uv[2 * f.c + o]);
        };
        this.hover = { onSurf: true, k: hit.object.userData.k, x: ang(0), y: ang(1), gx: I(d.g, 2, 0), gy: I(d.g, 2, 1),
                       z: sph ? hit.point.length() - RSPH : hit.point.z, side: I(d.jv, 1, 0) >= 0 ? 0 : 1, cols: p.cols };
      } else if (sph) {
        if (gp) { const f = fisheye(gp.x / RSPH, gp.y / RSPH, gp.z / RSPH, [0, 0]); this.hover = { onSurf: false, gx: f[0], gy: f[1] }; }
      } else {
        const gh = this.ray.intersectObject(this.ground, false)[0];
        if (gh) this.hover = { onSurf: false, gx: gh.point.x * SKY, gy: gh.point.y * SKY };
      }
    }
    if (this.hover || prev) this.dirty = true;
    if (this.onHover) this.onHover(this.hover ? { ...this.hover, D: unfisheye(this.hover.gx, this.hover.gy) } : null);
  }

  /** 把视角中心挪到某个末端点 (右栏末端点表单击). */
  focusEp(k, i) {
    const p = this.paths[k], e = p && p.d.eps[i];
    if (!e) return;
    const off = this.camera.position.clone().sub(this.controls.target);
    const dist = Math.min(off.length(), 8), el = Math.asin(off.z / off.length()) * R2D, az = Math.atan2(off.y, off.x) * R2D;
    this.tween = { t0: performance.now(), dur: 650, from: { dist: off.length(), el, az, target: this.controls.target.clone() },
                   to: { dist, el, az, target: new THREE.Vector3(...this._at(e.gx, e.gy, this.zOf(e.hv), [0, 0, 0])) } };
    this.dirty = true;
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

/** 天空方向 → 以太阳为中心的等距鱼眼 (度); 右 = 面向太阳时的右手边. 与 engine/lift.js 的 fisheyeOf 相同. */
function fisheye(Dx, Dy, Dz, out) {
  const sh = Math.sin(SUN.th), ch = Math.cos(SUN.th);
  const r = Math.acos(Math.max(-1, Math.min(1, Dx * ch + Dz * sh))) * R2D;
  const px = -Dy, py = -Dx * sh + Dz * ch, pn = Math.hypot(px, py);
  if (pn < 1e-15) { out[0] = 0; out[1] = 0; return out; }
  out[0] = r * px / pn; out[1] = r * py / pn;
  return out;
}

/** 鱼眼 (度) → 天空方向. */
export function unfisheye(x, y) {
  const r = Math.hypot(x, y) * D2R, sh = Math.sin(SUN.th), ch = Math.cos(SUN.th);
  if (r < 1e-12) return [ch, 0, sh];
  const ux = x / Math.hypot(x, y), uy = y / Math.hypot(x, y), c = Math.cos(r), s = Math.sin(r);
  // 太阳 = (cos h, 0, sin h); 右 = (0, −1, 0); 上 = (−sin h, 0, cos h)
  return [c * ch - s * uy * sh, -s * ux, c * sh + s * uy * ch];
}
