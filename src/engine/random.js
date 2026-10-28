// .NET System.Random(seed) 的逐位移植 (Knuth 减法发生器, .NET Framework 与 .NET 5+ 带种子构造同一算法).
// 用它而不用 Math.random: MC 填充 / 点云与原 C# 窗体逐点一致, 可以直接对账.

const MBIG = 2147483647;          // int.MaxValue
const MSEED = 161803398;

export class DotNetRandom {
  constructor(seed) {
    const sa = new Int32Array(56);
    const subtraction = seed === -2147483648 ? MBIG : Math.abs(seed);
    let mj = MSEED - subtraction;
    sa[55] = mj;
    let mk = 1;
    let ii = 0;
    for (let i = 1; i < 55; i++) {
      if ((ii += 21) >= 55) ii -= 55;
      sa[ii] = mk;
      mk = mj - mk;
      if (mk < 0) mk += MBIG;
      mj = sa[ii];
    }
    for (let k = 1; k < 5; k++) {
      for (let i = 1; i < 56; i++) {
        let n = i + 30;
        if (n >= 55) n -= 55;
        let v = sa[i] - sa[1 + n];
        if (v < 0) v += MBIG;
        sa[i] = v;
      }
    }
    this.sa = sa;
    this.inext = 0;
    this.inextp = 21;
  }

  internalSample() {
    let a = this.inext + 1; if (a >= 56) a = 1;
    let b = this.inextp + 1; if (b >= 56) b = 1;
    const sa = this.sa;
    let r = sa[a] - sa[b];
    if (r === MBIG) r--;
    if (r < 0) r += MBIG;
    sa[a] = r;
    this.inext = a;
    this.inextp = b;
    return r;
  }

  nextDouble() {
    return this.internalSample() * (1.0 / MBIG);
  }
}
