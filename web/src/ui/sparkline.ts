// 1 px sparklines over the timeline: x in [0,1], y scaled to 24 px (optionally log).
export function sparkline(svg: SVGSVGElement, xs: number[], ys: number[], log = false) {
  svg.setAttribute('viewBox', '0 0 1000 24');
  let path = svg.querySelector('path');
  if (!path) { path = document.createElementNS('http://www.w3.org/2000/svg', 'path'); svg.append(path); }
  // log scale: non-positive values (a triangle has energy 0) hold the previous sample
  const v: number[] = [];
  for (const y of ys) v.push(log ? (y > 0 ? Math.log10(y) : v.length ? v[v.length - 1] : 0) : y);
  if (v.length < 1) { path.setAttribute('d', ''); return; }
  let lo = Math.min(...v), hi = Math.max(...v);
  if (hi - lo < 1e-12) { lo -= 1; hi += 1; }
  const Y = (y: number) => (22 - (20 * (y - lo)) / (hi - lo)).toFixed(1);
  // step plot: hold each value until the next sample
  let d = `M${(xs[0] * 1000).toFixed(1)} ${Y(v[0])}`;
  for (let i = 1; i < v.length; i++) d += `H${(xs[i] * 1000).toFixed(1)}V${Y(v[i])}`;
  d += 'H1000';
  path.setAttribute('d', d);
}
