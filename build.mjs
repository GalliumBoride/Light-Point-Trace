// 构建: src/ → dist/LightPointTrace.html (单文件, 内联 CSS + JS + Worker; 双击即可离线运行, 任何系统),
//       另拷一份到 docs/index.html 供 GitHub Pages 发布.
//   node build.mjs           一次构建
//   node build.mjs --watch   改动 src/ 自动重建
import * as esbuild from 'esbuild';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.dirname(fileURLToPath(import.meta.url));
const src = p => path.join(root, 'src', p);
const outFile = path.join(root, 'dist', 'LightPointTrace.html');

async function bundle(entry, extra = {}) {
  const r = await esbuild.build({
    entryPoints: [entry], bundle: true, write: false, minify: true,
    format: 'iife', target: ['es2020'], legalComments: 'none', charset: 'utf8', ...extra,
  });
  return r.outputFiles[0].text;
}

async function build() {
  const t0 = Date.now();
  const worker = await bundle(src('worker.js'));
  const main = await bundle(src('app/main.js'), { define: { WORKER_SRC: JSON.stringify(worker) } });
  const css = (await esbuild.build({
    entryPoints: [src('app/style.css')], bundle: true, write: false, minify: true, charset: 'utf8',
  })).outputFiles[0].text;
  let html = fs.readFileSync(src('app/index.html'), 'utf8');
  html = html.replace('/*__CSS__*/', () => css)
             .replace('/*__JS__*/', () => main.replace(/<\/script/gi, '<\\/script'));
  fs.mkdirSync(path.dirname(outFile), { recursive: true });
  fs.writeFileSync(outFile, html);
  // GitHub Pages 从 docs/ 发布: 同一个文件再放一份当首页
  fs.mkdirSync(path.join(root, 'docs'), { recursive: true });
  fs.writeFileSync(path.join(root, 'docs', 'index.html'), html);
  console.log(`built ${path.relative(root, outFile)}  ${(html.length / 1024).toFixed(0)} KB  ${Date.now() - t0} ms`);
}

await build();
if (process.argv.includes('--watch')) {
  let timer = 0;
  fs.watch(path.join(root, 'src'), { recursive: true }, () => {
    clearTimeout(timer);
    timer = setTimeout(() => build().catch(e => console.error(e.message)), 80);
  });
  console.log('watching src/ …');
}
