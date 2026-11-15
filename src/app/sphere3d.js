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

// 直纹曲面 (同「直纹曲面分析」窗体的默认值): 可达 = 蓝 55%, 不可达 = 红 35%, 双面.
// 所有显示中的片合成一个缓冲区, 镜头一动就把三角形按远近重排 (由远到近画): 曲面之间、曲面自己前后几层之间、
// 方位 0/360 接缝两侧的遮挡都对, 也不会因为几个物体的先后次序来回换而闪. 再按半径拆两遍画:
// 球内的部分排在球壳之前 (透过球壳看, 被染色); 球外的部分排在球面上的点和线之后、做深度测试 —
// 挡在线前面的那层会盖在线上, 线前面的曲面不会盖住线. 不透明度拉到 100% 的部分改成真正不透明 (写深度, 挡住后面的一切).
export const RULED_REACH = [50, 120, 200], RULED_UNREACH = [150, 80, 80];
/** 直纹曲面一组片的键: 光路|边界#R区域号.边号 (可达; 1DOF 边号 0) 或 #E (延伸); 界面按它开关、加亮 */
export const ruledGroupKey = (prName, famName, pc) => `${prName}|${famName}#${pc.reach ? 'R' + pc.region + '.' + (pc.seg || 0) : 'E'}`;
// 明暗: 顶点法向由 Worker 按直纹曲面本身算 (∂L/∂s × e), 不靠三角形平均; 正负号在顶点着色器里翻成朝向镜头,
// 所以曲面折回自己的地方相邻法向不会抵消, 放大后也看不到一条条的母线条纹.
const RULED_VS = `attribute vec4 rgba;
varying vec4 vC; varying vec3 vW; varying vec3 vN;
void main() {
  vC = rgba;
  vec4 wp = modelMatrix * vec4(position, 1.0);
  vW = wp.xyz;
  vec3 n = normalize(mat3(modelMatrix) * normal);
  vN = dot(n, cameraPosition - wp.xyz) < 0.0 ? -n : n;
  gl_Position = projectionMatrix * viewMatrix * wp;
}`;
const RULED_FS = `uniform float uOut;
varying vec4 vC; varying vec3 vW; varying vec3 vN;
void main() {
  if ((length(vW) - 1.0) * uOut < 0.0) discard;
  float d = abs(dot(normalize(vN), normalize(cameraPosition - vW)));   // 头灯: 正对镜头最亮, 侧着变暗
  gl_FragColor = vec4(vC.rgb * (0.62 + 0.45 * d), vC.a);
}`;

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
    this.ruledObjs = [];          // 直纹曲面的物体 (合成后的网格 + 延伸弧), setData 时随 dyn 一起清掉
    this.ruledSig = '';
    this.occ = new Map(); this.occPrev = null; this.occStable = true;   // 叠加层文字 / 末端点被曲面挡住多少
    this.clouds = new Map();
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
    ctl.minDistance = 1.25; ctl.maxDistance = 18;          // 直纹曲面伸到球外, 要能拉远看全
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
    this.ruledObjs = []; this.ruledSig = ''; this.ruledT = null; this.occTris = null;
    this.layoutKey = null;
    this.layout(V);
  }

  /**
   * 直纹曲面: 1DOF 一张 (扫描角转一圈), 2DOF 每条勾选的边界一张 (连同不成立的延伸部分), 每张切成片
   * (一块成立区域那一段 = 可达片, 其余 = 延伸片). 数据由主程序向 Worker 要 (pr.ruled1 / pr.ruledMap).
   * 显示哪些片 / 不透明度 / 加亮变了才重建合成网格; 返回签名给 layout 判断要不要重排.
   */
  _syncRuled(V) {
    const R = this.R, ents = [];
    if (R && V.ruled) {
      const multi = R.paths.length > 1, th = V.theme, show = V.ruledShow || [true, true];
      R.paths.forEach((pr, k) => {
        const list = R.params.twoDof
          ? (pr.ori ? pr.ori.fams.map((fm, fi) => ({ name: fm.name, fi })).filter(x => V.ruledSel.has(V.famKey(pr, x.name))) : [])
          : [{ name: '', fi: 0 }];
        for (const { name, fi } of list) {
          const data = R.params.twoDof ? pr.ruledMap && pr.ruledMap.get(name) : pr.ruled1;
          if (!data || !data.pieces) continue;
          const col = multi ? th.pathcol[k % th.pathcol.length] : th.palette[fi % th.palette.length];
          data.pieces.forEach((pc, i) => {
            const g = ruledGroupKey(pr.name, name, pc);
            if (!show[pc.reach ? 0 : 1] || (V.ruledHide && V.ruledHide.has(g))) return;
            ents.push({ pc, g, col, id: pr.name + '|' + name + '#' + i });
          });
        }
      });
    }
    const A = V.ruledA || [0.55, 0.35], hov = V.ruledHover || new Set();
    const sig = [ents.map(e => e.id).join(','), A.join(','), V.sym ? 1 : 0, [...hov].join(',')].join(';');
    if (sig !== this.ruledSig) { this.ruledSig = sig; this._buildRuled(ents, A, hov, V.sym); }
    return sig;
  }

  _ruledMat(out, transparent) {
    return new THREE.ShaderMaterial({
      vertexShader: RULED_VS, fragmentShader: RULED_FS, side: THREE.DoubleSide, transparent, depthWrite: !transparent, depthTest: true,
      // 不透明的那部分深度往后让一点: 与它共面的半透明层 (板晶整张曲面在一个平面里; 镜像补全时两半重合) 稳定地叠在上面, 不打架出条纹
      polygonOffset: !transparent, polygonOffsetFactor: 2, polygonOffsetUnits: 2,
      uniforms: { uOut: { value: out } },
    });
  }

  _buildRuled(ents, A, hov, sym) {
    for (const o of this.ruledObjs) {
      this.dyn.remove(o);
      if (o.geometry && !o.userData.shared) o.geometry.dispose();
      if (o.material) { o.material.dispose(); this.lineMats = this.lineMats.filter(m => m !== o.material); }
    }
    this.ruledObjs = [];
    const T = { pos: [], nrm: [], col: [], idx: [] }, O = { pos: [], nrm: [], col: [], idx: [] }, ext = new Map();
    for (const e of ents) {
      const { pc } = e, h = hov.has(e.g), a0 = A[pc.reach ? 0 : 1];
      const a = h ? Math.min(0.92, Math.max(a0, 0.3) + 0.3) : a0;
      const c = (pc.reach ? RULED_REACH : RULED_UNREACH).map(v => (h ? Math.min(255, v + 55) : v) / 255);
      const tg = a >= 0.999 ? O : T, n = pc.pos.length / 3;
      for (const sy of sym ? [1, -1] : [1]) {
        const base = tg.pos.length / 3;
        for (let i = 0; i < n; i++) {
          tg.pos.push(pc.pos[3 * i], sy * pc.pos[3 * i + 1], pc.pos[3 * i + 2]); tg.col.push(c[0], c[1], c[2], a);
          tg.nrm.push(pc.nrm[3 * i], sy * pc.nrm[3 * i + 1], pc.nrm[3 * i + 2]);
        }
        for (let i = 0; i < pc.idx.length; i++) tg.idx.push(base + pc.idx[i]);
        if (!pc.reach && pc.ext.length) {
          const key = e.col.join(','), L = ext.get(key) || ext.set(key, { col: e.col, pos: [] }).get(key);
          for (let i = 0; i < pc.ext.length; i += 3) L.pos.push(RS * pc.ext[i], sy * RS * pc.ext[i + 1], RS * pc.ext[i + 2]);
        }
      }
    }
    const occ = { pos: [], idx: [], a: [] };
    const mk = (S, transparent) => {
      if (!S.idx.length) return null;
      const g = new THREE.BufferGeometry(), idx = Uint32Array.from(S.idx);
      g.setAttribute('position', new THREE.Float32BufferAttribute(S.pos, 3));
      g.setAttribute('rgba', new THREE.Float32BufferAttribute(S.col, 4));
      g.setAttribute('normal', new THREE.Float32BufferAttribute(S.nrm, 3));
      g.setIndex(new THREE.BufferAttribute(idx, 1));
      if (transparent) {                                  // 排序用: 每个三角形的重心与原始顶点序号
        const nt = idx.length / 3, cent = new Float32Array(3 * nt), p = S.pos;
        for (let t = 0; t < nt; t++)
          for (let k = 0; k < 3; k++) cent[3 * t + k] = (p[3 * idx[3 * t] + k] + p[3 * idx[3 * t + 1] + k] + p[3 * idx[3 * t + 2] + k]) / 3;
        g.userData = { cent, base: idx.slice() };
      }
      const off = occ.pos.length / 3;
      for (const v of S.pos) occ.pos.push(v);
      for (let i = 0; i < idx.length; i += 3) { occ.idx.push(off + idx[i], off + idx[i + 1], off + idx[i + 2]); occ.a.push(S.col[4 * idx[i] + 3]); }
      return g;
    };
    const gT = mk(T, true), gO = mk(O, false);
    const add = o => { o.frustumCulled = false; this.dyn.add(o); this.ruledObjs.push(o); };
    if (gO) { const m = new THREE.Mesh(gO, this._ruledMat(0, false)); add(m); }
    if (gT)
      for (const [out, ro] of [[-1, 0.5], [1, 5]]) {
        const m = new THREE.Mesh(gT, this._ruledMat(out, true));
        m.renderOrder = ro; if (out > 0) m.userData.shared = true;     // 两遍共用一个几何体, 只释放一次
        add(m);
      }
    this.ruledT = gT; this.sortKey = '';
    for (const L of ext.values()) {
      add(this._seg(L.pos, L.col, 1.3, { opacity: 0.55 }));
      add(this._seg(L.pos, L.col, 0.8, { ghost: true, opacity: 0.14 }));
    }
    // 遮挡: 每个三角形的包围球 + 不透明度 (叠加层的文字、末端点按它变淡)
    const nt = occ.idx.length / 3;
    if (nt) {
      const P = Float32Array.from(occ.pos), I = Uint32Array.from(occ.idx), C = new Float32Array(4 * nt);
      for (let t = 0; t < nt; t++) {
        const i0 = 3 * I[3 * t], i1 = 3 * I[3 * t + 1], i2 = 3 * I[3 * t + 2];
        const cx = (P[i0] + P[i1] + P[i2]) / 3, cy = (P[i0 + 1] + P[i1 + 1] + P[i2 + 1]) / 3, cz = (P[i0 + 2] + P[i1 + 2] + P[i2 + 2]) / 3;
        let r = 0;
        for (const j of [i0, i1, i2]) r = Math.max(r, Math.hypot(P[j] - cx, P[j + 1] - cy, P[j + 2] - cz));
        C[4 * t] = cx; C[4 * t + 1] = cy; C[4 * t + 2] = cz; C[4 * t + 3] = r;
      }
      this.occTris = { P, I, C, a: Float32Array.from(occ.a), n: nt };
    } else this.occTris = null;
    this.occ = new Map(); this.occPrev = null; this.occStable = true;
  }

  /** 半透明那部分: 镜头动了就按三角形重心离镜头的远近重排 (桶排序, 远的先画) */
  _sortRuled() {
    const g = this.ruledT;
    if (!g) return;
    const c = this.camera.position, tg = this.controls.target;
    const key = [c.x, c.y, c.z, tg.x, tg.y, tg.z].map(v => v.toFixed(4)).join(',');
    if (key === this.sortKey) return;
    this.sortKey = key;
    const { cent, base } = g.userData, nt = cent.length / 3, NB = 4096;
    let dx = tg.x - c.x, dy = tg.y - c.y, dz = tg.z - c.z;
    const dl = Math.hypot(dx, dy, dz) || 1; dx /= dl; dy /= dl; dz /= dl;
    const dep = new Float32Array(nt);
    let lo = Infinity, hi = -Infinity;
    for (let t = 0; t < nt; t++) {
      const d = (cent[3 * t] - c.x) * dx + (cent[3 * t + 1] - c.y) * dy + (cent[3 * t + 2] - c.z) * dz;
      dep[t] = d; if (d < lo) lo = d; if (d > hi) hi = d;
    }
    const sc = (NB - 1) / Math.max(hi - lo, 1e-9), bin = new Uint16Array(nt), cnt = new Uint32Array(NB + 1);
    for (let t = 0; t < nt; t++) { const b = Math.min(NB - 1, ((hi - dep[t]) * sc) | 0); bin[t] = b; cnt[b + 1]++; }
    for (let b = 0; b < NB; b++) cnt[b + 1] += cnt[b];
    const out = g.index.array;
    for (let t = 0; t < nt; t++) {
      const w = 3 * cnt[bin[t]]++;
      out[w] = base[3 * t]; out[w + 1] = base[3 * t + 1]; out[w + 2] = base[3 * t + 2];
    }
    g.index.needsUpdate = true;
  }

  /** 从镜头看点 p (世界坐标) 时, 中间隔着的曲面让它还剩几成可见 (∏(1 − 不透明度)); 镜头在动时用上一次停下时的结果 */
  _occlusion(p) {
    const T = this.occTris;
    if (!T) return 1;
    const key = p.map(v => v.toFixed(4)).join(',');
    const hit = this.occ.get(key);
    if (hit !== undefined) return hit;
    if (!this.occStable) { const v = this.occPrev && this.occPrev.get(key); return v === undefined ? 1 : v; }
    const o = this.camera.position;
    let dx = p[0] - o.x, dy = p[1] - o.y, dz = p[2] - o.z;
    const L = Math.hypot(dx, dy, dz) || 1; dx /= L; dy /= L; dz /= L;
    const { P, I, C, a, n } = T;
    let tr = 1;
    for (let t = 0; t < n && tr > 0.03; t++) {
      if (a[t] < 0.02) continue;
      const vx = C[4 * t] - o.x, vy = C[4 * t + 1] - o.y, vz = C[4 * t + 2] - o.z, r = C[4 * t + 3];
      const tp = vx * dx + vy * dy + vz * dz;
      if (tp < -r || tp > L + r || vx * vx + vy * vy + vz * vz - tp * tp > r * r) continue;
      const i0 = 3 * I[3 * t], i1 = 3 * I[3 * t + 1], i2 = 3 * I[3 * t + 2];
      const e1x = P[i1] - P[i0], e1y = P[i1 + 1] - P[i0 + 1], e1z = P[i1 + 2] - P[i0 + 2];
      const e2x = P[i2] - P[i0], e2y = P[i2 + 1] - P[i0 + 1], e2z = P[i2 + 2] - P[i0 + 2];
      const hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
      const det = e1x * hx + e1y * hy + e1z * hz;
      if (Math.abs(det) < 1e-12) continue;
      const f = 1 / det, sx = o.x - P[i0], sy = o.y - P[i0 + 1], sz = o.z - P[i0 + 2];
      const u = f * (sx * hx + sy * hy + sz * hz);
      if (u < 0 || u > 1) continue;
      const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
      const v = f * (dx * qx + dy * qy + dz * qz);
      if (v < 0 || u + v > 1) continue;
      const tt = f * (e2x * qx + e2y * qy + e2z * qz);
      if (tt > 1e-4 && tt < L - 2e-3) tr *= 1 - a[t];
    }
    this.occ.set(key, tr);
    return tr;
  }

  /** 打开直纹曲面时把镜头拉远一些 (母线伸到天球外 3 个单位), 关掉时回到只装天球 (用户没动过镜头时). */
  setRuledFit(on) {
    if (this.ruledFit === on) return;
    this.ruledFit = on;
    if (this.gl) this.goView(this.lastView || 'oblique', !this.R);
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
    const ruledState = this._syncRuled(V);
    const lk = [V.lineW, V.lines, V.arcs, V.mc, V.hlFull, V.iso, [...V.checked].join('|'), cloudState, ruledState].join(',');
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
    return (this.ruledFit ? 3.2 : 1.18) / Math.sin(Math.min(vf, hf) / 2);
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
    this._sortRuled();
    if (this.occTris) {
      const c = this.camera.position, ck = [c.x, c.y, c.z].map(v => v.toFixed(4)).join(',');
      if (ck !== this.occCamKey) {
        if (this.occCamKey !== undefined) {
          if (this.occ.size) this.occPrev = this.occ;
          this.occ = new Map(); this.occStable = false;
          clearTimeout(this.occTimer);
          this.occTimer = setTimeout(() => { this.occStable = true; this.dirty = true; }, 160);
        }
        this.occCamKey = ck;
      }
    }
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
      ctx.globalAlpha = (this._front(D) ? 1 : 0.35) * this._occlusion(D.map(v => v * RS));
      ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.9)'; ctx.strokeText(text, s[0], s[1] - 8);
      ctx.fillStyle = col; ctx.fillText(text, s[0], s[1] - 8);
      ctx.globalAlpha = 1;
    };
    const A = this.A || [Math.cos(10 * D2R), 0, Math.sin(10 * D2R)];
    lab(A, t('v3.sunLabel'), '#c46f00');
    lab([0, 0, 1], t('cam.zen'), '#475569');
    lab([-A[0], 0, A[2]], t('cam.anti'), '#475569');
    if (this._proj([0, 0, 0], s)) {
      ctx.globalAlpha = this._occlusion([0, 0, 0.0001]);
      ctx.fillStyle = '#334155'; ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,255,255,0.9)';
      ctx.strokeText(t('sph.observer'), s[0], s[1] - 8); ctx.fillText(t('sph.observer'), s[0], s[1] - 8);
      ctx.globalAlpha = 1;
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
            ctx.globalAlpha = (this._front(D) ? 1 : 0.22) * this._occlusion(D.map(v => v * RS));
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
