// The figure: 2D SVG and 3D WebGL2 canvas stacked; shared camera (view basis + frame), flip, auto-framing.
import type { Scene } from '../kernel/scene';
import { type View, TOP, projectCurves } from '../diagram/project';
import { smoothShown } from '../diagram/smooth';
import type { Ghost } from '../planner/trajectory';
import { findCrossings } from '../diagram/crossings';
import { Render2D, PEN, type Annot, type Drawn, type Frame } from '../view2d/render';
import { View3D, TUBE_RADIUS } from '../view3d/view3d';
import { orbit, slerpView, easeInOut } from '../view3d/camera';

/** 3D outline width, px: nearest and farthest (depth cue). */
const OUTLINE: [number, number] = [3.2, 2.2];
const reducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

function anim(ms: number, step: (t: number) => void): Promise<void> {
  if (ms <= 0 || reducedMotion()) { step(1); return Promise.resolve(); }
  return new Promise((res) => {
    const t0 = performance.now();
    const f = (now: number) => { const t = Math.min(1, (now - t0) / ms); step(t); if (t < 1) requestAnimationFrame(f); else res(); };
    requestAnimationFrame(f);
  });
}

export class Stage {
  r2d: Render2D;
  v3d: View3D | null = null;
  view: View = TOP;
  mode: '2d' | '3d' = '2d';
  sticks = false;                  // ball and stick of the unsmoothed polygon, in 2D and 3D
  W = 0; H = 0;
  shown: Scene | null = null;
  persp = false;
  private smoothed: Scene | null = null;   // smoothShown(shown, ghost), shared by both views
  private ghost: Ghost | null = null;
  private drawn2d: Drawn | null = null;
  /** The 2D drawing lags while only 3D paints. */
  private stale2d = false;
  /** Current look of the 3D render (animated during a flip). */
  private look = { flatten: 0, persp: 0, radius: TUBE_RADIUS, outline: OUTLINE };
  private flipping = false;
  /** Auto-framing: the frame eases toward a centred fit of the current curve, unless the user zoomed or panned. */
  private snapNext = true;
  private manual = false;
  private lastPaint = 0;
  private framePending = false;
  onChange: () => void = () => {};
  /** The user moved the camera (orbit, pan, zoom). */
  onManual: () => void = () => {};
  /** View the camera eases toward as frames paint (follow view). */
  private target: View | null = null;

  constructor(private el: HTMLElement, svg: SVGSVGElement, private canvas: HTMLCanvasElement, private notice: HTMLElement,
    private opts: { annot: () => Annot; onBestView: () => void }) {
    this.r2d = new Render2D(svg);
    if (View3D.supported()) {
      this.v3d = new View3D(canvas, () => { this.v3d!.readColors(); this.redraw(); });
      this.v3d.readColors();
    }
    new ResizeObserver(() => this.resize()).observe(el);
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { this.v3d?.readColors(); this.redraw(); });
    this.bindInput();
  }

  get frame() { return this.r2d.frame; }
  set frame(f: Frame) { this.r2d.frame = f; }
  get has3d() { return !!this.v3d?.gl; }

  private resize() {
    const b = this.el.getBoundingClientRect();
    if (b.width === this.W && b.height === this.H) return;
    this.W = b.width; this.H = b.height;
    this.r2d.svg.setAttribute('viewBox', `0 0 ${this.W} ${this.H}`);
    this.v3d?.resize(this.W, this.H);
    this.snapNext = true;
    if (this.shown) this.redraw();
  }

  /** Jump straight to the centred fit on the next paint (new knot, reset, resize) and resume auto-framing. */
  snapFrame() { this.snapNext = true; this.manual = false; this.r2d.resetLabels(); }

  /** Centred fit of the curve's projection with a 10% border. */
  fitted(S: Scene, view: View): Frame {
    const C = projectCurves(S, view);
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const Q of C) for (let i = 0; i < Q.length; i += 2) { x0 = Math.min(x0, Q[i]); x1 = Math.max(x1, Q[i]); y0 = Math.min(y0, Q[i + 1]); y1 = Math.max(y1, Q[i + 1]); }
    const w = Math.max(1, this.W), h = Math.max(1, this.H), m = Math.min(w, h) * 0.1;
    const s = Math.min((w - 2 * m) / Math.max(1e-9, x1 - x0), (h - 2 * m) / Math.max(1e-9, y1 - y0));
    return { s, cx: w / 2 - (s * (x0 + x1)) / 2, cy: h / 2 + (s * (y0 + y1)) / 2 };
  }

  /** Ease toward v from now on (time constant 500 ms, frame by frame); null stops. */
  follow(v: View | null) { this.target = v; }

  /** Ease the frame toward the centred fit (time constant 350 ms); keep animating until it settles. */
  private autoFrame() {
    const now = performance.now(), dt = Math.min(100, now - this.lastPaint);
    this.lastPaint = now;
    if (this.target) {
      const T = this.target, k = reducedMotion() ? 1 : 1 - Math.exp(-dt / 500);
      this.view = slerpView(this.view, T, k);
      const d = (a: ArrayLike<number>, b: ArrayLike<number>) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      if (d(this.view.F, T.F) > 1 - 1e-7 && d(this.view.U, T.U) > 1 - 1e-7) { this.view = T; this.target = null; }
    }
    if (!this.shown || this.manual || !this.W) return;
    const t = this.fitted(this.shown, this.view);
    if (this.snapNext || reducedMotion()) { this.frame = t; this.snapNext = false; return; }
    const f = this.frame, k = 1 - Math.exp(-dt / 350);
    this.frame = { cx: f.cx + (t.cx - f.cx) * k, cy: f.cy + (t.cy - f.cy) * k, s: f.s * Math.pow(t.s / f.s, k) };
    const settled = Math.abs(t.cx - this.frame.cx) < 0.2 && Math.abs(t.cy - this.frame.cy) < 0.2 && Math.abs(t.s / this.frame.s - 1) < 5e-4;
    if (settled) this.frame = t;
    else if (!this.framePending) { this.framePending = true; requestAnimationFrame(() => { this.framePending = false; this.redraw(); }); }
  }

  /**
   * Draw a Scene in whichever views are visible. With a ghost (a deletion still fading), the smoothing blends from
   * the curve with its vertices to without.
   */
  show(S: Scene, ghost: Ghost | null = null) {
    this.shown = S; this.ghost = ghost;
    this.smoothed = null;
    this.paint();
  }
  redraw() { if (this.shown) this.paint(); }

  /** The 2D drawing of the current state and view, brought up to date if only 3D has painted since. */
  get drawn(): Drawn | null {
    if (this.stale2d && this.shown) this.draw2d();
    return this.drawn2d;
  }

  private curve() { return this.sticks ? this.shown! : (this.smoothed ??= smoothShown(this.shown!, this.ghost)); }
  private draw2d() { this.drawn2d = this.r2d.draw(this.curve(), this.view, this.opts.annot(), this.sticks); this.stale2d = false; }

  private paint() {
    this.autoFrame();
    if (this.mode === '2d' || this.flipping) this.draw2d(); else this.stale2d = true;
    if (this.v3d && (this.mode === '3d' || this.flipping)) {
      this.v3d.setCurves(this.curve(), this.sticks);
      const L = this.look;
      this.v3d.render(this.view, this.frame, { flatten: L.flatten, persp: L.persp, radius: L.radius, outlinePx: L.outline });
    }
    this.onChange();
  }

  crossingsAlong(S: Scene, v: View) { return findCrossings(projectCurves(S, v)).length; }

  /** Slerp the camera to `to` over `ms`; the frame follows by auto-framing. */
  async animateView(to: View, ms = 600) {
    this.target = null;
    const from = this.view;
    this.manual = false;
    await anim(ms, (t) => { this.view = slerpView(from, to, easeInOut(t)); this.redraw(); });
    this.view = to;
    this.redraw();
  }

  // ------------------------------------------------------------ flip

  private setLayer(svgOpacity: number) {
    this.r2d.svg.style.opacity = String(svgOpacity);
    this.canvas.style.opacity = String(1 - svgOpacity);
    this.el.classList.toggle('mode3d', svgOpacity < 0.5);
  }

  async flip(to: '2d' | '3d') {
    if (to === this.mode || this.flipping) return;
    if (to === '3d' && !this.has3d) return;
    this.flipping = true;
    const L = this.look;
    const flat = { radius: 0, outline: [PEN / 2, PEN / 2] as [number, number] }, full = { radius: TUBE_RADIUS, outline: OUTLINE };
    const mix = (t: number) => {
      L.flatten = t; L.radius = full.radius + (flat.radius - full.radius) * t;
      L.outline = [full.outline[0] + (flat.outline[0] - full.outline[0]) * t, full.outline[1] + (flat.outline[1] - full.outline[1]) * t];
    };
    if (to === '2d') {
      if (L.persp > 0) { const p0 = L.persp; await anim(200, (t) => { L.persp = p0 * (1 - easeInOut(t)); this.redraw(); }); }
      await anim(400, (t) => { mix(easeInOut(t)); this.redraw(); });
      await anim(150, (t) => this.setLayer(t));
      this.mode = '2d';
    } else {
      mix(1); this.mode = '3d'; this.redraw();
      await anim(150, (t) => this.setLayer(1 - t));
      await anim(400, (t) => { mix(1 - easeInOut(t)); this.redraw(); });
      if (this.persp) await anim(200, (t) => { L.persp = easeInOut(t); this.redraw(); });
    }
    this.flipping = false;
    this.setLayer(this.mode === '2d' ? 1 : 0);
    this.canvas.tabIndex = this.mode === '3d' ? 0 : -1;
    this.redraw();
  }

  async setPersp(on: boolean) {
    this.persp = on;
    if (this.mode !== '3d') { this.look.persp = on ? 1 : 0; return; }
    const p0 = this.look.persp, p1 = on ? 1 : 0;
    await anim(200, (t) => { this.look.persp = p0 + (p1 - p0) * easeInOut(t); this.redraw(); });
  }

  // ------------------------------------------------------------ camera input (3D only)

  private bindInput() {
    const c = this.canvas;
    let drag: { x: number; y: number; pan: boolean } | null = null;
    const pts = new Map<number, { x: number; y: number }>();
    let pinch0 = 0;
    c.addEventListener('pointerdown', (e) => {
      if (this.mode !== '3d') return;
      c.setPointerCapture(e.pointerId); c.focus();
      pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
      drag = { x: e.clientX, y: e.clientY, pan: e.shiftKey || pts.size > 1 };
      if (pts.size === 2) { const [a, b] = [...pts.values()]; pinch0 = Math.hypot(a.x - b.x, a.y - b.y); }
    });
    c.addEventListener('pointermove', (e) => {
      if (!drag || !pts.has(e.pointerId)) return;
      const prev = pts.get(e.pointerId)!;
      pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
      const dx = e.clientX - prev.x, dy = e.clientY - prev.y;
      if (pts.size === 2) {
        const [a, b] = [...pts.values()], d = Math.hypot(a.x - b.x, a.y - b.y);
        if (pinch0 > 0) this.zoomAt(d / pinch0, (a.x + b.x) / 2, (a.y + b.y) / 2);
        pinch0 = d;
        this.pan(dx / 2, dy / 2);
      } else if (drag.pan) this.pan(dx, dy);
      else { this.manualView(); this.view = orbit(this.view, dx * 0.01, dy * 0.01); this.redraw(); }
    });
    const up = (e: PointerEvent) => { pts.delete(e.pointerId); if (!pts.size) drag = null; };
    c.addEventListener('pointerup', up); c.addEventListener('pointercancel', up);
    c.addEventListener('wheel', (e) => { if (this.mode !== '3d') return; e.preventDefault(); this.zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX, e.clientY); }, { passive: false });
    c.addEventListener('dblclick', () => this.opts.onBestView());
    c.addEventListener('keydown', (e) => {
      if (this.mode !== '3d') return;
      const a = (5 * Math.PI) / 180, k: Record<string, [number, number]> = { ArrowLeft: [-a, 0], ArrowRight: [a, 0], ArrowUp: [0, -a], ArrowDown: [0, a] };
      if (k[e.key]) { e.preventDefault(); e.stopPropagation(); this.manualView(); this.view = orbit(this.view, ...k[e.key]); this.redraw(); }
    });
  }
  /** Camera input overrides following. */
  private manualView() { this.target = null; this.onManual(); }
  private pan(dx: number, dy: number) { this.manual = true; const f = this.frame; this.frame = { ...f, cx: f.cx + dx, cy: f.cy + dy }; this.redraw(); }
  private zoomAt(k: number, clientX: number, clientY: number) {
    this.manual = true;
    const b = this.el.getBoundingClientRect(), x = clientX - b.left, y = clientY - b.top, f = this.frame;
    this.frame = { s: f.s * k, cx: x + (f.cx - x) * k, cy: y + (f.cy - y) * k };
    this.redraw();
  }

  disable3d(msg: string) { this.notice.textContent = msg; }
}
