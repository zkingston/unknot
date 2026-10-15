// 3D ink view: white tube outlined in black ink (inverted hull), hand-rolled WebGL2.
import type { Scene } from '../kernel/scene';
import type { View } from '../diagram/project';
import type { Frame } from '../view2d/render';
import { compile, cssRGB } from './gl';
import { tubeMesh, ballStickMesh } from './tube';
import VS from './shaders/tube.vert.glsl?raw';
import FS from './shaders/tube.frag.glsl?raw';

export const TUBE_RADIUS = 0.03;

export interface Render3DOpts {
  flatten: number;       // 0..1
  persp: number;         // 0..1
  radius: number;        // world units
  outlinePx: [number, number]; // near, far
}

export class View3D {
  gl: WebGL2RenderingContext | null = null;
  private prog: WebGLProgram | null = null;
  private vao: WebGLVertexArrayObject | null = null;
  private bufC: WebGLBuffer | null = null; private bufN: WebGLBuffer | null = null; private bufI: WebGLBuffer | null = null;
  private count = 0;
  private lastIndex: Uint32Array | null = null; // the index buffer's contents (new buffers start empty)
  private loc: Record<string, WebGLUniformLocation | null> = {};
  private centre: [number, number, number] = [0, 0, 0];
  private curves: Scene = [];
  private sticks = false;
  ink: [number, number, number] = [0, 0, 0];
  paper: [number, number, number] = [1, 1, 1];
  /** Shadows and shading darken: toward the ink on light paper, toward black on dark paper (else they glow). */
  private get dark(): [number, number, number] { const l = (c: number[]) => 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]; return l(this.paper) > l(this.ink) ? this.ink : [0, 0, 0]; }
  lost = false;
  W = 1; H = 1; dpr = 1;

  static supported(): boolean {
    try { return !!document.createElement('canvas').getContext('webgl2'); } catch { return false; }
  }

  constructor(public canvas: HTMLCanvasElement, private onRestored: () => void) {
    canvas.addEventListener('webglcontextlost', (e) => { e.preventDefault(); this.lost = true; });
    canvas.addEventListener('webglcontextrestored', () => { this.lost = false; this.init(); this.onRestored(); });
    this.init();
  }

  private init() {
    const gl = this.canvas.getContext('webgl2', { antialias: true, alpha: false, depth: true, premultipliedAlpha: false });
    if (!gl) { this.gl = null; return; }
    this.gl = gl;
    this.prog = compile(gl, VS, FS);
    for (const u of ['uR', 'uU', 'uF', 'uViewport', 'uFrame', 'uKnotCentre', 'uDepthRange', 'uRadius', 'uExtrude', 'uOutlinePx', 'uFlatten', 'uPersp', 'uEyeDist', 'uColor', 'uDark', 'uShade', 'uLight', 'uShadow', 'uShadowPx'])
      this.loc[u] = gl.getUniformLocation(this.prog, u);
    this.vao = gl.createVertexArray();
    gl.bindVertexArray(this.vao);
    this.bufC = gl.createBuffer(); this.bufN = gl.createBuffer(); this.bufI = gl.createBuffer(); this.lastIndex = null;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bufC); gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bufN); gl.enableVertexAttribArray(1); gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.bufI);
    gl.bindVertexArray(null);
    if (this.curves.length) this.setCurves(this.curves, this.sticks);
  }

  readColors() {
    const cs = getComputedStyle(document.documentElement);
    this.ink = cssRGB(cs.getPropertyValue('--ink').trim() || '#000');
    this.paper = cssRGB(cs.getPropertyValue('--paper').trim() || '#fff');
  }

  resize(w: number, h: number) {
    this.W = Math.max(1, w); this.H = Math.max(1, h);
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    const bw = Math.round(this.W * this.dpr), bh = Math.round(this.H * this.dpr);
    if (this.canvas.width !== bw || this.canvas.height !== bh) { this.canvas.width = bw; this.canvas.height = bh; }
  }

  /** Upload the curves as a tube, or as ball and stick; rebuilt whole each frame. */
  setCurves(S: Scene, sticks = false) {
    this.curves = S; this.sticks = sticks;
    const gl = this.gl;
    if (!gl || this.lost) return;
    const m = sticks ? ballStickMesh(S) : tubeMesh(S);
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bufC); gl.bufferData(gl.ARRAY_BUFFER, m.centers, gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bufN); gl.bufferData(gl.ARRAY_BUFFER, m.normals, gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.bufI);
    if (m.index !== this.lastIndex) { gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, m.index, gl.DYNAMIC_DRAW); this.lastIndex = m.index; } // unchanged layout: keep it
    gl.bindVertexArray(null);
    this.count = m.index.length;
    let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
    for (const P of S) for (let i = 0; i < P.length; i += 3) {
      x0 = Math.min(x0, P[i]); x1 = Math.max(x1, P[i]); y0 = Math.min(y0, P[i + 1]); y1 = Math.max(y1, P[i + 1]); z0 = Math.min(z0, P[i + 2]); z1 = Math.max(z1, P[i + 2]);
    }
    this.centre = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2];
  }

  render(v: View, f: Frame, o: Render3DOpts) {
    const gl = this.gl;
    if (!gl || this.lost || !this.prog) return;
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(this.paper[0], this.paper[1], this.paper[2], 1);
    gl.clearDepth(1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    if (!this.count) return;
    gl.useProgram(this.prog);
    const L = this.loc;
    gl.uniform3fv(L.uR, v.R as number[]); gl.uniform3fv(L.uU, v.U as number[]); gl.uniform3fv(L.uF, v.F as number[]);
    gl.uniform2f(L.uViewport, this.W, this.H);
    gl.uniform3f(L.uFrame, f.cx, f.cy, f.s);
    gl.uniform3fv(L.uKnotCentre, this.centre);
    let dmin = Infinity, dmax = -Infinity;
    for (const P of this.curves) for (let i = 0; i < P.length; i += 3) { const d = P[i] * v.F[0] + P[i + 1] * v.F[1] + P[i + 2] * v.F[2]; dmin = Math.min(dmin, d); dmax = Math.max(dmax, d); }
    gl.uniform2f(L.uDepthRange, dmin, dmax);
    gl.uniform1f(L.uRadius, o.radius);
    gl.uniform2f(L.uOutlinePx, o.outlinePx[0], o.outlinePx[1]);
    gl.uniform1f(L.uFlatten, o.flatten);
    gl.uniform1f(L.uPersp, o.persp);
    gl.uniform1f(L.uEyeDist, (this.H / 2) / (f.s * Math.tan(Math.PI / 12)));
    // light from the upper left, toward the viewer
    const lx = -0.45 * v.R[0] + 0.6 * v.U[0] + 0.66 * v.F[0], ly = -0.45 * v.R[1] + 0.6 * v.U[1] + 0.66 * v.F[1], lz = -0.45 * v.R[2] + 0.6 * v.U[2] + 0.66 * v.F[2];
    const ll = Math.hypot(lx, ly, lz);
    gl.uniform3f(L.uLight, lx / ll, ly / ll, lz / ll);
    const dark = this.dark;
    gl.uniform3fv(L.uDark, dark);
    gl.enable(gl.CULL_FACE);
    gl.bindVertexArray(this.vao);
    const shadow = 1 - o.flatten;
    // 0. drop shadow: the flattened silhouette, offset down-right, a faint dark wash behind everything
    if (shadow > 0.01) {
      gl.disable(gl.DEPTH_TEST); gl.depthMask(false); gl.cullFace(gl.BACK);
      gl.uniform1f(L.uShadow, 1); gl.uniform2f(L.uShadowPx, 6 * shadow, 8 * shadow);
      gl.uniform1f(L.uExtrude, 1); gl.uniform1f(L.uShade, 0);
      const k = (dark === this.ink ? 0.13 : 0.45) * shadow, P = this.paper;
      gl.uniform3f(L.uColor, P[0] + (dark[0] - P[0]) * k, P[1] + (dark[1] - P[1]) * k, P[2] + (dark[2] - P[2]) * k);
      gl.drawElements(gl.TRIANGLES, this.count, gl.UNSIGNED_INT, 0);
      gl.depthMask(true); gl.uniform1f(L.uShadow, 0);
    }
    gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LESS);
    // 1. outline: back faces pushed out along the normals, in ink
    gl.cullFace(gl.FRONT);
    gl.uniform1f(L.uExtrude, 1); gl.uniform3fv(L.uColor, this.ink); gl.uniform1f(L.uShade, 0);
    gl.drawElements(gl.TRIANGLES, this.count, gl.UNSIGNED_INT, 0);
    // 2. fill: front faces in paper colour, softly shaded away from the light
    gl.cullFace(gl.BACK);
    gl.uniform1f(L.uExtrude, 0); gl.uniform3fv(L.uColor, this.paper); gl.uniform1f(L.uShade, 0.3 * shadow);
    gl.drawElements(gl.TRIANGLES, this.count, gl.UNSIGNED_INT, 0);
    gl.bindVertexArray(null);
  }

  /** RGB at CSS pixel (x, y) of the current frame (call right after render). */
  readPixel(x: number, y: number): [number, number, number] {
    const gl = this.gl!, px = new Uint8Array(4);
    gl.readPixels(Math.round(x * this.dpr), Math.round(this.canvas.height - 1 - y * this.dpr), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
    return [px[0], px[1], px[2]];
  }
}
