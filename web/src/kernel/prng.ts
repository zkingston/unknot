// Seeded PRNG: splitmix32 seeds sfc32. Use exactly these; test goldens come from them.
export function splitmix32(seed: number) {
  let s = seed >>> 0;
  return () => { s = (s + 0x9E3779B9) | 0; let z = s;
    z = Math.imul(z ^ (z >>> 16), 0x85EBCA6B); z = Math.imul(z ^ (z >>> 13), 0xC2B2AE35);
    return (z ^ (z >>> 16)) >>> 0; };
}
export function sfc32(seed: number) {
  const sm = splitmix32(seed); let a = sm(), b = sm(), c = sm(), d = sm();
  const next = () => { const t = (((a + b) | 0) + d) | 0; d = (d + 1) | 0;
    a = b ^ (b >>> 9); b = (c + (c << 3)) | 0; c = (c << 21) | (c >>> 11); c = (c + t) | 0;
    return t >>> 0; };
  for (let i = 0; i < 12; i++) next();   // discard the first 12 outputs
  return next;                           // uniform in [0,1) = next() / 2**32; index = Math.floor(uniform * n)
}

export type Rng = () => number;

export const uniform = (r: Rng) => r() / 4294967296;
export const index = (r: Rng, n: number) => Math.floor(uniform(r) * n);

/** Point uniform on the sphere of radius `radius` about the origin (Marsaglia 1972; only sqrt). */
export function onSphere(r: Rng, radius: number, out: Float64Array | number[] = new Float64Array(3)) {
  let x1 = 0, x2 = 0, s = 1;
  while (s >= 1 || s === 0) { x1 = 2 * uniform(r) - 1; x2 = 2 * uniform(r) - 1; s = x1 * x1 + x2 * x2; }
  const f = 2 * Math.sqrt(1 - s);
  out[0] = radius * x1 * f; out[1] = radius * x2 * f; out[2] = radius * (1 - 2 * s);
  return out;
}
