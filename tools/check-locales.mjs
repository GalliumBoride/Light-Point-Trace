// 检查各语言文件与 zh-CN 结构一致: 键相同, {变量} 相同, HTML 标签相同, 数组长度相同.
//   node tools/check-locales.mjs            检查全部
//   node tools/check-locales.mjs en fr      只检查这几种
import { pathToFileURL, fileURLToPath } from 'node:url';
import path from 'node:path';
import fs from 'node:fs';

const dir = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'src', 'app', 'locales');
const load = async code => (await import(pathToFileURL(path.join(dir, code + '.js')).href)).default;
const base = await load('zh-CN');
const codes = process.argv.slice(2).length ? process.argv.slice(2)
  : fs.readdirSync(dir).filter(f => f.endsWith('.js') && f !== 'zh-CN.js').map(f => f.slice(0, -3));

const vars = s => [...String(s).matchAll(/\{(\w+)\}/g)].map(m => m[1]).sort().join(',');
const tags = s => [...String(s).matchAll(/<\/?([a-z]+)[^>]*>/g)].map(m => m[0].replace(/\s+/g, ' ')).sort().join('');

let bad = 0;
const err = (code, msg) => { bad++; console.log(`  ✗ ${code}: ${msg}`); };
function same(code, where, a, b) {
  if (typeof a === 'string') {
    if (a === '') { if (b !== '') err(code, `${where}: 原文为空, 译文也应为空`); return; }
    if (typeof b !== 'string' || !b.trim()) return err(code, `${where}: 缺少或为空`);
    if (vars(a) !== vars(b)) err(code, `${where}: 变量 {${vars(a)}} ≠ {${vars(b)}}`);
    if (tags(a) !== tags(b)) err(code, `${where}: HTML 标签不同`);
    if (/[一-鿿]/.test(b) && !code.startsWith('zh') && code !== 'ja') err(code, `${where}: 还有中文: ${b.slice(0, 40)}`);
    return;
  }
  if (Array.isArray(a)) {
    if (!Array.isArray(b) || a.length !== b.length) return err(code, `${where}: 数组长度 ${a.length} ≠ ${b && b.length}`);
    a.forEach((x, i) => same(code, `${where}[${i}]`, x, b[i]));
    return;
  }
  if (a && typeof a === 'object') {
    if (!b || typeof b !== 'object') return err(code, `${where}: 缺少`);
    for (const k of Object.keys(a)) same(code, `${where}.${k}`, a[k], b[k]);
    for (const k of Object.keys(b)) if (!(k in a)) err(code, `${where}.${k}: 多出来的键`);
  }
}

for (const code of codes) {
  let loc;
  try { loc = await load(code); } catch (e) { err(code, '加载失败: ' + e.message); continue; }
  const before = bad;
  same(code, code, base, loc);
  console.log(`${bad === before ? '✓' : '✗'} ${code}  (${Object.keys(loc.ui || {}).length} 条界面文字, ${Object.keys(loc.terms || {}).length} 个术语)`);
}
process.exit(bad ? 1 : 0);
