// Draw editor: freehand closed strokes kept as splines through control points (drag a point to reshape), live
// crossings, click to flip, undo/redo.
import type { Diagram, StrandRef } from '../diagram/types';
import { findCrossings, validateDiagram, type RawCrossing } from '../diagram/crossings';
import { cutPaths, arcLength, arcAt, GAP } from '../view2d/render';

const NS = 'http://www.w3.org/2000/svg';
const HIT = 12, TOUCH = 1.8; // hit radius (crossings, control points); grows by TOUCH for fingers
const CTRL_EPS = 8;          // px: a stroke's control points simplify it to within this

interface Stroke { ctrl: number[]; pts: number[] }  // screen px, y down; pts = spline(ctrl)
/** A remembered over/under choice: matched by component pair and nearest position. */
interface Rec { ca: number; cb: number; x: number; y: number; overComp: number; overFrac: number }
interface Shown { X: RawCrossing; aOver: boolean; dot: boolean }

/** Ramer–Douglas–Peucker simplification of an open polyline. */
function rdp(p: number[], eps: number): number[] {
  const n = p.length / 2;
  if (n < 3) return p.slice();
  const keep = new Uint8Array(n); keep[0] = keep[n - 1] = 1;
  const stack: [number, number][] = [[0, n - 1]];
  while (stack.length) {
    const [a, b] = stack.pop()!;
    const ax = p[2 * a], ay = p[2 * a + 1], dx = p[2 * b] - ax, dy = p[2 * b + 1] - ay, l2 = dx * dx + dy * dy;
    let best = -1, bd = eps;
    for (let i = a + 1; i < b; i++) {
      let t = l2 > 0 ? ((p[2 * i] - ax) * dx + (p[2 * i + 1] - ay) * dy) / l2 : 0; t = Math.max(0, Math.min(1, t));
      const d = Math.hypot(ax + t * dx - p[2 * i], ay + t * dy - p[2 * i + 1]);
      if (d > bd) { bd = d; best = i; }
    }
    if (best >= 0) { keep[best] = 1; stack.push([a, best], [best, b]); }
  }
  const out: number[] = [];
  for (let i = 0; i < n; i++) if (keep[i]) out.push(p[2 * i], p[2 * i + 1]);
  return out;
}

/** Closed Catmull–Rom spline through the points, resampled by arc length at `step` px. */
function catmullRom(p: number[], step: number): number[] {
  const n = p.length / 2;
  if (n < 2) return p.slice();
  const P = (i: number) => { const k = ((i % n) + n) % n; return [p[2 * k], p[2 * k + 1]]; };
  const dense: number[] = [];
  for (let i = 0; i < n; i++) {
    const [x0, y0] = P(i - 1), [x1, y1] = P(i), [x2, y2] = P(i + 1), [x3, y3] = P(i + 2);
    const m = Math.max(2, Math.ceil(Math.hypot(x2 - x1, y2 - y1) / 1.5));
    for (let k = 0; k < m; k++) {
      const t = k / m, t2 = t * t, t3 = t2 * t;
      dense.push(0.5 * (2 * x1 + (-x0 + x2) * t + (2 * x0 - 5 * x1 + 4 * x2 - x3) * t2 + (-x0 + 3 * x1 - 3 * x2 + x3) * t3),
        0.5 * (2 * y1 + (-y0 + y2) * t + (2 * y0 - 5 * y1 + 4 * y2 - y3) * t2 + (-y0 + 3 * y1 - 3 * y2 + y3) * t3));
    }
  }
  const out: number[] = [dense[0], dense[1]];
  let acc = 0;
  for (let i = 2; i < dense.length; i += 2) {
    const d = Math.hypot(dense[i] - dense[i - 2], dense[i + 1] - dense[i - 1]);
    acc += d;
    if (acc >= step) { out.push(dense[i], dense[i + 1]); acc = 0; }
  }
  return out;
}

/** Control points of a freehand stroke: within CTRL_EPS of it, without a last point that repeats the first. */
function controls(raw: number[]): number[] {
  const c = rdp(raw, CTRL_EPS);
  if (c.length >= 8 && Math.hypot(c[0] - c[c.length - 2], c[1] - c[c.length - 1]) < 2 * CTRL_EPS) c.splice(-2);
  return c;
}
const spline = (ctrl: number[]) => catmullRom(ctrl, 4);

/** Position of a strand point as a fraction of its curve's length. */
function fracOf(curves: Float64Array[]) {
  const arcs = curves.map(arcLength);
  return (r: StrandRef) => { const A = arcs[r.comp]; return arcAt(A, r) / A[A.length - 1]; };
}

export interface EditorHost {
  status(html: string): void;
  analyze(d: Diagram): Promise<string>;   // status HTML for a closed diagram
  done(d: Diagram): void;
  cancel(): void;
}

export class Editor {
  strokes: Stroke[] = [];
  recs: Rec[] = [];
  private shown: Shown[] = [];
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private svg: SVGSVGElement;
  private g: SVGGElement;
  private bar: HTMLDivElement;
  private note: HTMLSpanElement;
  private active = false;
  private hover: { x: number; y: number } | null = null;
  private drag: null | { kind: 'stroke'; raw: number[] } | { kind: 'handle'; s: number; i: number; moved: boolean } | { kind: 'flip'; k: number; x: number; y: number; tol: number } = null;
  /** The stroke being drawn (screen px), shown as an open line until release closes it. */
  private live: number[] | null = null;
  private analyzeTimer = 0;

  constructor(private stage: HTMLElement, private host: EditorHost) {
    this.svg = document.createElementNS(NS, 'svg') as SVGSVGElement;
    this.svg.setAttribute('class', 'editor');
    this.svg.setAttribute('aria-label', 'drawing area');
    this.g = document.createElementNS(NS, 'g');
    this.svg.append(this.g);
    this.bar = document.createElement('div');
    this.bar.className = 'editor-bar';
    this.bar.innerHTML = `<button class="btn" data-a="undo"><span>undo</span></button><button class="btn" data-a="redo"><span>redo</span></button>
      <button class="btn" data-a="clear"><span>clear</span></button><button class="btn" data-a="done"><span>done</span></button><button class="btn" data-a="cancel"><span>cancel</span></button>
      <span class="editor-note" aria-live="polite"></span>`;
    this.note = this.bar.querySelector('.editor-note')!;
    this.bar.addEventListener('click', (e) => {
      const a = (e.target as HTMLElement).closest<HTMLElement>('[data-a]')?.dataset.a;
      if (a === 'undo') this.undo();
      else if (a === 'redo') this.redo();
      else if (a === 'clear') { this.push(); this.strokes = []; this.recs = []; this.update(); }
      else if (a === 'done') this.finish();
      else if (a === 'cancel') this.host.cancel();
    });
    this.bindPointer();
  }

  get isOpen() { return this.active; }

  open() {
    this.active = true;
    this.stage.append(this.svg, this.bar);
    const b = this.stage.getBoundingClientRect();
    this.svg.setAttribute('viewBox', `0 0 ${b.width} ${b.height}`);
    this.update();
  }
  /** Leaving draw mode discards the drawing: the next visit starts fresh. */
  close() {
    this.active = false; this.svg.remove(); this.bar.remove();
    this.strokes = []; this.recs = []; this.shown = []; this.undoStack = []; this.redoStack = []; this.carried = false; this.drag = null; this.hover = null; this.live = null;
    clearTimeout(this.analyzeTimer);
  }

  keydown(e: KeyboardEvent): boolean {
    if (!this.active) return false;
    const mod = e.ctrlKey || e.metaKey;
    if (mod && (e.key === 'z' || e.key === 'Z')) { e.preventDefault(); if (e.shiftKey) this.redo(); else this.undo(); return true; }
    if (mod && (e.key === 'y' || e.key === 'Y')) { e.preventDefault(); this.redo(); return true; }
    return false;
  }

  // ------------------------------------------------------------ state

  private snapshot() { return JSON.stringify({ s: this.strokes, r: this.recs }); }
  private restore(j: string) { const o = JSON.parse(j); this.strokes = o.s; this.recs = o.r; this.update(); }
  private push() { this.undoStack.push(this.snapshot()); if (this.undoStack.length > 200) this.undoStack.shift(); this.redoStack = []; }
  undo() { const j = this.undoStack.pop(); if (j) { this.redoStack.push(this.snapshot()); this.restore(j); } }
  redo() { const j = this.redoStack.pop(); if (j) { this.undoStack.push(this.snapshot()); this.restore(j); } }

  private curves(): Float64Array[] { return this.strokes.filter((s) => s.pts.length >= 6).map((s) => Float64Array.from(s.pts)); }

  /** Over/under for every crossing: remembered choice if one matches, else the default rule. */
  private assign(curves: Float64Array[], X: RawCrossing[]): Shown[] {
    const frac = fracOf(curves);
    // default: alternate along each traversal (first visit decides)
    const def = new Map<number, boolean>();
    curves.forEach((_, c) => {
      const ev: { k: number; isA: boolean; at: number }[] = [];
      X.forEach((x, k) => { if (x.a.comp === c) ev.push({ k, isA: true, at: x.a.seg + x.a.t }); if (x.b.comp === c) ev.push({ k, isA: false, at: x.b.seg + x.b.t }); });
      ev.sort((p, q) => p.at - q.at).forEach((e, i) => { if (!def.has(e.k)) def.set(e.k, (i % 2 === 0) === e.isA); });
    });
    const used = new Set<number>();
    return X.map((x, k) => {
      const ca = Math.min(x.a.comp, x.b.comp), cb = Math.max(x.a.comp, x.b.comp);
      let best = -1, bd = 16;
      this.recs.forEach((r, i) => { if (used.has(i) || r.ca !== ca || r.cb !== cb) return; const d = Math.hypot(r.x - x.x, r.y - x.y); if (d < bd) { bd = d; best = i; } });
      if (best < 0) return { X: x, aOver: def.get(k)!, dot: this.recs.length > 0 && this.carried };
      used.add(best);
      const r = this.recs[best];
      const aOver = x.a.comp !== x.b.comp ? x.a.comp === r.overComp : Math.abs(frac(x.a) - r.overFrac) < Math.abs(frac(x.b) - r.overFrac);
      return { X: x, aOver, dot: false };
    });
  }
  /** True once a reshape has frozen the existing choices (new crossings are then marked until confirmed). */
  private carried = false;

  /** Remember every current over/under choice (with crossing `flipped` reversed). */
  private remember(curves: Float64Array[], flipped = -1) {
    const frac = fracOf(curves);
    this.recs = this.shown.map(({ X, aOver }, i) => { const o = (i === flipped ? !aOver : aOver) ? X.a : X.b; return { ca: Math.min(X.a.comp, X.b.comp), cb: Math.max(X.a.comp, X.b.comp), x: X.x, y: X.y, overComp: o.comp, overFrac: frac(o) }; });
  }
  private freeze(curves: Float64Array[]) { this.remember(curves); this.carried = true; }

  private flip(k: number) {
    this.push();
    this.remember(this.curves(), k);
    this.update();
  }

  diagram(): Diagram | null {
    const curves = this.curves();
    if (!curves.length) return null;
    const comps = curves.map((Q) => { const P = Q.slice(); for (let i = 1; i < P.length; i += 2) P[i] = -P[i]; return P; });
    return { comps, crossings: this.shown.map(({ X, aOver }) => (aOver ? { under: X.b, over: X.a } : { under: X.a, over: X.b })), source: 'draw' };
  }

  /** Nudge any vertex within 0.2 px of a crossing; report crossings closer than 6 px. */
  private degeneracies(X: RawCrossing[]): string {
    let nudged = false;
    for (const x of X) for (const r of [x.a, x.b]) {
      const s = this.strokes.filter((q) => q.pts.length >= 6)[r.comp], n = s.pts.length / 2;
      for (const v of [r.seg, (r.seg + 1) % n]) {
        if (Math.hypot(s.pts[2 * v] - x.x, s.pts[2 * v + 1] - x.y) < 0.2) {
          const j = (r.seg + 1) % n, dx = s.pts[2 * j] - s.pts[2 * r.seg], dy = s.pts[2 * j + 1] - s.pts[2 * r.seg + 1], l = Math.hypot(dx, dy) || 1;
          s.pts[2 * v] += (-dy / l) * 0.5; s.pts[2 * v + 1] += (dx / l) * 0.5; nudged = true;
        }
      }
    }
    if (nudged) return 'Nudged the curve off a crossing.';
    for (let i = 0; i < X.length; i++) for (let j = i + 1; j < X.length; j++)
      if (Math.hypot(X[i].x - X[j].x, X[i].y - X[j].y) < 6) return 'Two crossings are closer than 6 px; reshape to separate them.';
    return '';
  }

  update() {
    if (!this.active) return;
    // nudge until no vertex sits on a crossing, recomputing the crossings after every nudge so they match the curves
    let curves = this.curves(), X = findCrossings(curves), note = '';
    for (let it = 0; it < 8; it++) {
      const n = this.degeneracies(X);
      if (!n.startsWith('Nudged')) { if (n) note = n; break; }
      note = n; curves = this.curves(); X = findCrossings(curves);
    }
    this.shown = this.assign(curves, X);
    this.render(curves);
    const d = this.diagram();
    let ok = !!d;
    if (d) { const v = validateDiagram(d); if (!v.ok) { ok = false; note = `Not a regular diagram (${v.error}); reshape here.`; } }
    if (note.startsWith('Two crossings')) ok = false;
    (this.bar.querySelector('[data-a=done]') as HTMLButtonElement).disabled = !ok;
    this.note.textContent = note;
    clearTimeout(this.analyzeTimer);
    const head = `<span class="name">Drawing · ${X.length} crossing${X.length === 1 ? '' : 's'}</span>`;
    if (!curves.length) this.say('Draw a closed curve, then tap a crossing to flip it or drag a dot to reshape.');
    else if (d && ok) this.analyzeTimer = window.setTimeout(() => { this.host.analyze(d).then((h) => { if (this.active) this.say(`${head} ${h}`); }); }, 150);
  }

  private render(curves: Float64Array[]) {
    const arcs = curves.map(arcLength), cuts = curves.map(() => [] as number[]);
    for (const { X, aOver } of this.shown) { const u = aOver ? X.b : X.a; cuts[u.comp].push(arcAt(arcs[u.comp], u)); }
    const d = cutPaths(curves, cuts, GAP);
    let html = d.map((p) => `<path class="k" d="${p}"/>`).join('');
    if (this.live && this.live.length >= 4) html += `<path class="k" d="M${this.live.join(' ')}"/>`;
    for (const { X, dot } of this.shown) if (dot) html += `<circle class="dot" cx="${X.x}" cy="${X.y}" r="2"/>`;
    if (!this.live) for (const st of this.strokes) for (let i = 0; i < st.ctrl.length; i += 2) html += `<circle class="handle" cx="${st.ctrl[i]}" cy="${st.ctrl[i + 1]}" r="3.5"/>`;
    if (this.hover) html += `<circle class="ann" cx="${this.hover.x}" cy="${this.hover.y}" r="6"/>`;
    this.g.innerHTML = html;
  }

  // ------------------------------------------------------------ input

  private local(e: PointerEvent) { const b = this.svg.getBoundingClientRect(); return [e.clientX - b.left, e.clientY - b.top]; }
  private say(html: string) { this.host.status(html); }
  private crossingAt(x: number, y: number, r = HIT) { let k = -1, bd = r; this.shown.forEach((s, i) => { const d = Math.hypot(s.X.x - x, s.X.y - y); if (d < bd) { bd = d; k = i; } }); return k; }
  /** Nearest control point within r: stroke, index (point number), distance. */
  private handleAt(x: number, y: number, r = HIT) {
    let best: { s: number; i: number; d: number } | null = null;
    this.strokes.forEach((st, si) => { for (let i = 0; i < st.ctrl.length / 2; i++) { const d = Math.hypot(st.ctrl[2 * i] - x, st.ctrl[2 * i + 1] - y); if (d < r && (!best || d < best.d)) best = { s: si, i, d }; } });
    return best as { s: number; i: number; d: number } | null;
  }

  private bindPointer() {
    const sv = this.svg;
    sv.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      sv.setPointerCapture(e.pointerId);
      const [x, y] = this.local(e), f = e.pointerType === 'touch' ? TOUCH : 1;
      // the nearer of a crossing (tap flips it) and a control point (drag reshapes); elsewhere a new stroke
      const k = this.crossingAt(x, y, HIT * f), h = this.handleAt(x, y, HIT * f);
      const dk = k >= 0 ? Math.hypot(this.shown[k].X.x - x, this.shown[k].X.y - y) : Infinity;
      this.push();
      if (h && h.d <= dk) { this.freeze(this.curves()); this.drag = { kind: 'handle', s: h.s, i: h.i, moved: false }; return; }
      if (k >= 0) { this.undoStack.pop(); this.drag = { kind: 'flip', k, x, y, tol: 6 * f }; return; }
      this.drag = { kind: 'stroke', raw: [x, y] };
      this.live = [x, y];
    });
    sv.addEventListener('pointermove', (e) => {
      const [x, y] = this.local(e);
      if (!this.drag) { const k = this.crossingAt(x, y); const h = k >= 0 ? { x: this.shown[k].X.x, y: this.shown[k].X.y } : null; if (JSON.stringify(h) !== JSON.stringify(this.hover)) { this.hover = h; this.render(this.curves()); } return; }
      const evs = (e.getCoalescedEvents?.() ?? []).length ? e.getCoalescedEvents() : [e];
      const d = this.drag;
      if (d.kind === 'stroke') {
        for (const ev of evs) d.raw.push(...this.local(ev));
        this.live = rdp(d.raw, 1.5);
        this.render(this.curves());
      } else if (d.kind === 'handle') {
        const st = this.strokes[d.s];
        st.ctrl[2 * d.i] = x; st.ctrl[2 * d.i + 1] = y; d.moved = true;
        st.pts = spline(st.ctrl);
        this.update();
      }
    });
    const end = (e: PointerEvent) => {
      const d = this.drag; this.drag = null;
      if (!d) return;
      const [x, y] = this.local(e);
      if (d.kind === 'flip') { if (Math.hypot(x - d.x, y - d.y) < d.tol) this.flip(d.k); return; }
      if (d.kind === 'handle') { if (!d.moved) this.undoStack.pop(); this.update(); return; }
      this.live = null;
      // a stroke always closes on release: control points simplify it, the closed spline joins its end to its start
      const ctrl = d.raw.length >= 12 ? controls(d.raw) : [];
      if (ctrl.length < 6) { this.undoStack.pop(); this.update(); return; }
      this.strokes.push({ ctrl, pts: spline(ctrl) });
      this.update();
    };
    sv.addEventListener('pointerup', end);
    sv.addEventListener('pointercancel', end);
  }
  private finish() {
    const d = this.diagram();
    if (!d || !validateDiagram(d).ok) return;
    this.host.done(d);
  }
}
