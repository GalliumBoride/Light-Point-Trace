// Worker 池: 每个任务占一个槽 (每条光路 = 天球任务 + 取向平面任务, 各一个 Worker, 全部并行).
// 新计算到来时, 仍在忙的 Worker 直接终止重建 (即时取消); 空闲的复用 (JIT 已热).
// Worker 源码由构建脚本内联成字符串 (WORKER_SRC), 用 Blob URL 启动 — 单文件 HTML 双击即可离线运行.

export class EnginePool {
  constructor(src) {
    this.url = URL.createObjectURL(new Blob([src], { type: 'text/javascript' }));
    this.slots = [];          // { w, busy, task, job, pending: Map(rid → resolve) }
    this.job = 0;
    this.rid = 0;
    this.onMessage = null;    // (msg) => void, 只收当前 job 的计算消息
  }

  _spawn(i) {
    const w = new Worker(this.url);
    const slot = { w, busy: false, task: null, job: 0, pending: new Map() };
    w.onmessage = e => this._handle(slot, e.data);
    w.onerror = e => {
      e.preventDefault();
      if (!slot.busy) return;
      slot.busy = false;
      const t = slot.task || {};
      if (this.onMessage && slot.job === this.job) {
        this.onMessage({ type: 'error', job: slot.job, idx: t.idx, part: t.part, message: e.message || 'Worker error' });
        this.onMessage({ type: 'done', job: slot.job, idx: t.idx, part: t.part, ms: 0 });
      }
    };
    this.slots[i] = slot;
    return slot;
  }

  _handle(slot, m) {
    if (m.type === 'reply') {
      const p = slot.pending.get(m.rid);
      if (p) { slot.pending.delete(m.rid); p(m.data); }
      return;
    }
    if (m.type === 'done') slot.busy = false;
    if (m.job !== this.job) return;          // 过期结果
    if (this.onMessage) this.onMessage(m);
  }

  _kill(i) {
    const s = this.slots[i];
    if (!s) return;
    s.w.terminate();
    for (const p of s.pending.values()) p(null);
    this.slots[i] = null;
  }

  /** 启动一次计算: tasks[i] 发给槽 i. 返回 job id. */
  compute(tasks) {
    const job = ++this.job;
    for (let i = 0; i < Math.max(tasks.length, this.slots.length); i++) {
      const s = this.slots[i];
      if (s && s.busy) this._kill(i);
      if (i >= tasks.length) continue;
      const slot = this.slots[i] || this._spawn(i);
      slot.busy = true; slot.job = job; slot.task = tasks[i];
      slot.w.postMessage({ type: 'compute', job, ...tasks[i] });
    }
    return job;
  }

  /** 按需请求 (cloud / refine / probe) 给槽 i 的 Worker. 解析为数据或 null. */
  request(i, msg) {
    const s = this.slots[i];
    if (!s) return Promise.resolve(null);
    const rid = ++this.rid;
    return new Promise(resolve => {
      s.pending.set(rid, resolve);
      s.w.postMessage({ ...msg, rid });
    });
  }
}
