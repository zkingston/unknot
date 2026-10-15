import type { Diagram } from '../diagram/types';
import type { Scene } from '../kernel/scene';
import type { View } from '../diagram/project';
import { lift } from '../diagram/lift';
import { simplify } from '../diagram/simplify';
import { analyze } from '../invariants';
import { bestView } from '../diagram/bestView';
import { validateDiagram } from '../diagram/crossings';
import { parseBraid, braidToDiagram } from './braid';
import { traceImage, assembleArcs, type TraceOptions, type TraceResult } from './image/assemble';
import type { Arc } from './image/trace';
import { polylineInk } from './svg';
import { parsePD } from './pdcode';
import { parseGauss } from './gauss';
import { pdToDiagram } from './embed';

const ctx = self as unknown as { postMessage(m: unknown, t?: Transferable[]): void; onmessage: ((e: MessageEvent) => void) | null };

export type ImportRequest =
  | { kind: 'ping' }
  | { kind: 'lift'; diagram: Diagram }
  | { kind: 'invariants'; diagram: Diagram }
  | { kind: 'bestView'; scene: Scene; view: View }
  | { kind: 'braid' | 'pd' | 'gauss'; text: string }
  | { kind: 'image'; bitmap: ImageBitmap; opts?: TraceOptions }
  | { kind: 'arcs'; arcs: Arc[]; sw: number; opts?: TraceOptions }
  | { kind: 'retrace'; opts: TraceOptions };

export interface CodeResult { diagram: Diagram }

// the last traced source, kept for re-runs from the import overlay
let last: { kind: 'image'; img: { width: number; height: number; data: Uint8ClampedArray } } | { kind: 'arcs'; arcs: Arc[]; sw: number } | null = null;
const trace = (opts: TraceOptions = {}): TraceResult => {
  if (!last) throw new Error('nothing to retrace');
  return last.kind === 'image' ? traceImage(last.img, opts) : assembleArcs(last.arcs, last.sw, polylineInk(last.arcs), opts);
};

const handlers = {
  ping: () => 'pong',
  lift: (m: { diagram: Diagram }) => {
    const v = validateDiagram(m.diagram);
    if (!v.ok) throw new Error(`diagram is not regular: ${v.error}`);
    return { scene: simplify(lift(m.diagram), { epsilon: 0.01 }) };
  },
  invariants: (m: { diagram: Diagram }, id: number) => analyze(m.diagram, (f) => ctx.postMessage({ id, progress: f })),
  bestView: (m: { scene: Scene; view: View }) => bestView(m.scene, m.view),
  braid: (m: { text: string }): CodeResult => ({ diagram: braidToDiagram(parseBraid(m.text), m.text) }),
  pd: (m: { text: string }): CodeResult => ({ diagram: { ...pdToDiagram(parsePD(m.text)), source: 'pd', meta: { sourceText: m.text } } }),
  gauss: (m: { text: string }): CodeResult => ({ diagram: { ...pdToDiagram(parseGauss(m.text).pd), source: 'gauss', meta: { sourceText: m.text } } }),
  image: (m: { bitmap: ImageBitmap; opts?: TraceOptions }) => {
    const c = new OffscreenCanvas(m.bitmap.width, m.bitmap.height), g = c.getContext('2d')!;
    g.drawImage(m.bitmap, 0, 0);
    m.bitmap.close();
    last = { kind: 'image', img: g.getImageData(0, 0, c.width, c.height) };
    return trace(m.opts);
  },
  arcs: (m: { arcs: Arc[]; sw: number; opts?: TraceOptions }) => { last = { kind: 'arcs', arcs: m.arcs, sw: m.sw }; return trace(m.opts); },
  retrace: (m: { opts: TraceOptions }) => trace(m.opts),
};

ctx.onmessage = (e) => {
  const m = e.data as ImportRequest & { id: number };
  const t0 = performance.now();
  try {
    const h = (handlers as Record<string, (m: unknown, id: number) => unknown>)[m.kind];
    if (!h) throw new Error(`unknown request ${m.kind}`);
    const result = h(m, m.id);
    ctx.postMessage({ id: m.id, result, ms: performance.now() - t0 });
  } catch (err) {
    ctx.postMessage({ id: m.id, error: err instanceof Error ? err.message : String(err), ms: performance.now() - t0 });
  }
};
