// Import review: the source at 25% under the traced result; pair/unpair endpoints, flip crossings, threshold.
import type { Diagram, Crossing } from '../diagram/types';
import { events } from '../diagram/types';
import type { TraceResult, TraceOptions } from '../import/image/assemble';


export interface OverlayHost {
  retrace(opts: TraceOptions): Promise<TraceResult>;
  accept(d: Diagram): void;
}

/** Remembered crossing choice: position and the over strand's direction. */
interface Choice { x: number; y: number; dx: number; dy: number }

export class ImportOverlay {
  private r: TraceResult | null = null;
  private opts: TraceOptions = {};
  private choices: Choice[] = [];
  private sel = -1;
  private timer = 0;
  private svg: SVGSVGElement;
  private info: HTMLElement;
  private slider: HTMLInputElement;
  private auto: HTMLButtonElement;
  private bg: SVGImageElement;
  private g: SVGGElement;

  constructor(private root: HTMLElement, private sheet: HTMLElement, private host: OverlayHost) {
    sheet.innerHTML = `<h2>Import</h2>
      <svg class="trace" role="img" aria-label="traced diagram"><image opacity="0.25" preserveAspectRatio="none"/><g></g></svg>
      <p class="hints">Click two endpoints to pair or unpair them; click a crossing to flip it.</p>
      <div class="row"><label for="thr">threshold</label><input type="range" id="thr" min="0.05" max="0.95" step="0.01" value="0.5" aria-label="threshold">
      <button class="btn" data-a="auto" aria-pressed="true"><span>auto</span></button></div>
      <p class="trace-info" aria-live="polite"></p>
      <div class="row"><button class="btn" data-a="alt"><span>alternate rest</span></button><button class="btn" data-a="accept"><span>accept</span></button><button class="btn" data-a="cancel"><span>cancel</span></button></div>`;
    this.svg = sheet.querySelector('svg.trace')!;
    this.bg = this.svg.querySelector('image')!;
    this.g = this.svg.querySelector('g')!;
    this.info = sheet.querySelector('.trace-info')!;
    this.slider = sheet.querySelector('#thr')!;
    this.auto = sheet.querySelector('[data-a=auto]')!;
    this.slider.addEventListener('input', () => {
      this.opts.threshold = Number(this.slider.value); this.auto.setAttribute('aria-pressed', 'false');
      clearTimeout(this.timer); this.timer = window.setTimeout(() => this.rerun(), 150);
    });
    sheet.addEventListener('click', (e) => {
      const a = (e.target as HTMLElement).closest<HTMLElement>('[data-a]')?.dataset.a;
      if (a === 'auto') { delete this.opts.threshold; this.auto.setAttribute('aria-pressed', 'true'); this.rerun(); }
      else if (a === 'alt') this.alternateRest();
      else if (a === 'accept') { const d = this.diagram(); if (d && this.ready()) { this.close(); this.host.accept(d); } }
      else if (a === 'cancel') this.close();
    });
    this.svg.addEventListener('click', (e) => this.click(e));
  }

  get isOpen() { return this.root.classList.contains('open'); }
  open(r: TraceResult, imageURL: string) {
    this.r = r; this.opts = {}; this.choices = []; this.sel = -1;
    this.auto.setAttribute('aria-pressed', 'true');
    this.bg.setAttribute('href', imageURL);
    this.root.classList.add('open');
    this.render();
    (this.sheet.querySelector('[data-a=accept]') as HTMLElement).focus();
  }
  close() { this.root.classList.remove('open'); this.r = null; }

  private async rerun() {
    if (!this.r) return;
    const r = await this.host.retrace({ ...this.opts });
    if (!this.isOpen) return;
    this.r = r; this.sel = -1; this.render();
  }

  /** Diagram with remembered choices applied (nearest crossing within 3 stroke widths). */
  diagram(): Diagram | null {
    const r = this.r;
    if (!r) return null;
    const d = r.diagram;
    const crossings: Crossing[] = d.crossings.map((X) => {
      const p = this.pos(d, X), c = this.choices.find((q) => Math.hypot(q.x - p.x, q.y - p.y) < 3 * r.sw);
      if (!c) return X;
      const du = this.dir(d, X.under), aligned = Math.abs(du[0] * c.dx + du[1] * c.dy);
      const dov = this.dir(d, X.over), alignedO = Math.abs(dov[0] * c.dx + dov[1] * c.dy);
      // the remembered over strand is the one more parallel to the stored direction
      return alignedO >= aligned ? { under: X.under, over: X.over } : { under: X.over, over: X.under };
    });
    return { ...d, crossings };
  }
  private ready() { const d = this.diagram(); return !!d && !!this.r && this.r.diagnostics.unpaired === 0 && d.comps.length > 0 && !d.crossings.some((X) => X.flat && !this.chosen(d, X)); }
  private chosen(d: Diagram, X: Crossing) { const p = this.pos(d, X); return this.choices.some((q) => Math.hypot(q.x - p.x, q.y - p.y) < 3 * this.r!.sw); }
  private pos(d: Diagram, X: Crossing) { const P = d.comps[X.over.comp], n = P.length / 2, i = X.over.seg, j = (i + 1) % n; return { x: P[2 * i] + X.over.t * (P[2 * j] - P[2 * i]), y: P[2 * i + 1] + X.over.t * (P[2 * j + 1] - P[2 * i + 1]) }; }
  private dir(d: Diagram, s: { comp: number; seg: number }) { const P = d.comps[s.comp], n = P.length / 2, j = (s.seg + 1) % n, dx = P[2 * j] - P[2 * s.seg], dy = P[2 * j + 1] - P[2 * s.seg + 1], l = Math.hypot(dx, dy) || 1; return [dx / l, dy / l]; }

  private flip(k: number) {
    const d = this.diagram()!, X = d.crossings[k], p = this.pos(d, X);
    this.choices = this.choices.filter((q) => Math.hypot(q.x - p.x, q.y - p.y) >= 3 * this.r!.sw);
    // a flat crossing gets its first strand over; an assigned one swaps
    const newOver = X.flat ? X.over : X.under, dv = this.dir(d, newOver);
    this.choices.push({ x: p.x, y: p.y, dx: dv[0], dy: dv[1] });
    this.render();
  }

  /** Assign remaining flat crossings so each traversal alternates. */
  private alternateRest() {
    const d = this.diagram(); if (!d) return;
    const set = new Map<number, boolean>(); // crossing -> is the `over` field the over strand
    d.crossings.forEach((X, k) => { if (!X.flat || this.chosen(d, X)) set.set(k, true); });
    for (let pass = 0; pass < 2; pass++) d.comps.forEach((_, c) => {
      const ev = events(d, c);
      for (let i = 0; i < ev.length; i++) {
        const e = ev[i], prev = ev[(i + ev.length - 1) % ev.length];
        if (set.has(e.x) || !set.has(prev.x)) continue;
        const prevOver = prev.over === set.get(prev.x);
        set.set(e.x, e.over === prevOver ? false : true);
      }
    });
    d.crossings.forEach((X, k) => {
      if (!X.flat || this.chosen(d, X)) return;
      const p = this.pos(d, X), o = set.get(k) === false ? X.under : X.over, dv = this.dir(d, o);
      this.choices.push({ x: p.x, y: p.y, dx: dv[0], dy: dv[1] });
    });
    this.render();
  }

  private click(e: MouseEvent) {
    const r = this.r; if (!r) return;
    const pt = this.svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
    const m = this.svg.getScreenCTM(); if (!m) return;
    const q = pt.matrixTransform(m.inverse()), x = q.x, y = -q.y, hit = 2.5 * r.sw;
    const d = this.diagram()!;
    const k = d.crossings.findIndex((X) => { const p = this.pos(d, X); return Math.hypot(p.x - x, p.y - y) < hit; });
    if (k >= 0) { this.flip(k); return; }
    const ei = r.endpoints.findIndex((p) => Math.hypot(p.x - x, p.y - y) < hit);
    if (ei < 0) { this.sel = -1; this.render(); return; }
    if (this.sel < 0 || this.sel === ei) { this.sel = this.sel === ei ? -1 : ei; this.render(); return; }
    const a = this.sel, b = ei, key = (p: number[]) => `${Math.min(p[0], p[1])}:${Math.max(p[0], p[1])}`;
    this.sel = -1;
    const pair: [number, number] = [a, b];
    if (r.endpoints[a].partner === b) {
      this.opts.force = (this.opts.force ?? []).filter((p) => key(p) !== key(pair));
      this.opts.forbid = [...(this.opts.forbid ?? []), pair];
    } else {
      this.opts.forbid = (this.opts.forbid ?? []).filter((p) => key(p) !== key(pair));
      this.opts.force = [...(this.opts.force ?? []).filter((p) => !p.includes(a) && !p.includes(b)), pair];
    }
    this.rerun();
  }

  private render() {
    const r = this.r; if (!r) return;
    const W = r.image?.width ?? 1000, H = r.image?.height ?? 1000;
    let x0 = 0, y0 = 0, w = W, h = H;
    if (!r.image) { // vector source: frame the arcs
      let a = Infinity, b = Infinity, c = -Infinity, dd = -Infinity;
      for (const A of r.arcs) for (let i = 0; i < A.length; i += 2) { a = Math.min(a, A[i]); c = Math.max(c, A[i]); b = Math.min(b, -A[i + 1]); dd = Math.max(dd, -A[i + 1]); }
      const m = 0.05 * Math.max(c - a, dd - b); x0 = a - m; y0 = b - m; w = c - a + 2 * m; h = dd - b + 2 * m;
    }

    this.svg.setAttribute('viewBox', `${x0} ${y0} ${w} ${h}`);
    if (r.image) { this.bg.setAttribute('x', '0'); this.bg.setAttribute('y', '0'); this.bg.setAttribute('width', String(W)); this.bg.setAttribute('height', String(H)); }
    const sw = r.sw, pen = Math.max(1, 0.35 * sw), P = (xy: Float64Array) => { let s = ''; for (let i = 0; i < xy.length; i += 2) s += `${i ? 'L' : 'M'}${xy[i].toFixed(1)} ${(-xy[i + 1]).toFixed(1)}`; return s; };
    let html = '';
    for (const A of r.arcs) html += `<path class="arc" stroke-width="${pen}" d="${P(A)}"/>`;
    for (const G of r.gaps) html += `<path class="gap" stroke-width="${pen}" stroke-dasharray="${2 * pen} ${2 * pen}" d="${P(G)}"/>`;
    r.endpoints.forEach((e, i) => {
      const cls = i === this.sel ? 'ep sel' : e.partner >= 0 ? 'ep' : 'ep open';
      html += e.partner >= 0 && i !== this.sel ? `<circle class="${cls}" cx="${e.x}" cy="${-e.y}" r="${0.6 * sw}"/>` : `<circle class="${cls}" cx="${e.x}" cy="${-e.y}" r="${1.2 * sw}" stroke-width="${pen}"/>${e.partner < 0 ? `<text x="${e.x + 1.6 * sw}" y="${-e.y}" font-size="${2.5 * sw}">?</text>` : ''}`;
    });
    const d = this.diagram()!;
    d.crossings.forEach((X) => {
      const p = this.pos(d, X), s = 0.9 * sw, chosen = this.chosen(d, X);
      html += `<rect class="xing${X.flat && !chosen ? ' flat' : chosen ? ' set' : ''}" x="${p.x - s}" y="${-p.y - s}" width="${2 * s}" height="${2 * s}" stroke-width="${pen}"/>`;
      if (X.flat && !chosen) html += `<text x="${p.x + 1.4 * sw}" y="${-p.y - 1.4 * sw}" font-size="${2.5 * sw}">?</text>`;
    });
    this.g.innerHTML = html;
    const flat = d.crossings.filter((X) => X.flat && !this.chosen(d, X)).length, g = r.diagnostics;
    this.info.textContent = `${g.arcs} arcs · ${d.crossings.length} crossings · ${g.components} component${g.components === 1 ? '' : 's'} · ${g.unpaired} unpaired · ${flat} flat${r.image ? ` · ${r.image.method}` : ''}`;
    (this.sheet.querySelector('[data-a=accept]') as HTMLButtonElement).disabled = !this.ready();
    (this.sheet.querySelector('[data-a=alt]') as HTMLButtonElement).disabled = !flat;
  }
}
