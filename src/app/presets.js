// 示例: 常见晕弧的光路 + 取向 + 太阳高度. 每条都用引擎核对过天上的位置 (距太阳角距、仰角).
// 名字和说明在 locales/*.js 的 presets[id] 里.
// mode: 0 柱晶 1 板晶 2 Parry 3 Lowitz;  fixed: 柱晶 = ζ, Parry/Lowitz = roll;  cam: [视线方位(原窗体约定), 仰角, 视场].

export const PRESETS = [
  { id: 'sundog', path: '3-5', mode: 1, fixed: 90, sun: 10, sym: true, cam: [0, 10, 140] },
  { id: 'cza', path: '1-3', mode: 1, fixed: 90, sun: 25, cam: [0, 50, 160] },
  { id: 'cha', path: '3-2', mode: 1, fixed: 90, sun: 65, cam: [0, 40, 160] },
  { id: 'tangent', path: '3-5', mode: 0, fixed: 90, sun: 15, cam: [0, 15, 140] },
  { id: 'circumscribed', path: '3-5', mode: 0, fixed: 90, sun: 40, cam: [0, 40, 140] },
  { id: 'supralateral', path: '3-1', mode: 0, fixed: 90, sun: 20, cam: [0, 45, 180] },
  { id: 'infralateral', path: '1-3', mode: 0, fixed: 90, sun: 20, cam: [0, 15, 180] },
  { id: 'wegener', path: '3-1-5', mode: 0, fixed: 90, sun: 10, cam: [180, 15, 180] },
  { id: 'parhelic', path: '5-1-8-5', mode: 0, fixed: 90, sun: 28, cam: [0, 90, 180] },
  { id: 'parry', path: '3-5', mode: 2, fixed: 90, sun: 15, cam: [0, 25, 120] },
  { id: 'lowitz', path: '3-5', mode: 3, fixed: 270, sun: 20, cam: [0, 20, 140] },
];
