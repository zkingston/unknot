// Main-thread clients for the two workers.
import PlannerWorker from '../planner/planner.worker.ts?worker&inline';
import ImportWorker from '../import/import.worker.ts?worker&inline';
import type { ToPlanner, FromPlanner } from '../planner/planner.worker';
import type { ImportRequest } from '../import/import.worker';

export const planner = new PlannerWorker();
const importer = new ImportWorker();

let nextId = 1;
const pending = new Map<number, { res: (v: any) => void; rej: (e: Error) => void; progress?: (f: number) => void }>();
importer.onmessage = (e) => {
  const m = e.data as { id: number; result?: unknown; error?: string; progress?: number };
  const p = pending.get(m.id);
  if (!p) return;
  if (m.progress !== undefined) { p.progress?.(m.progress); return; }
  pending.delete(m.id);
  if (m.error !== undefined) p.rej(new Error(m.error)); else p.res(m.result);
};

/** Request/response to the import worker. */
export function request<T>(req: ImportRequest, transfer: Transferable[] = [], progress?: (f: number) => void): Promise<T> {
  const id = nextId++;
  return new Promise<T>((res, rej) => { pending.set(id, { res, rej, progress }); importer.postMessage({ ...req, id }, transfer); });
}

export function toPlanner(m: ToPlanner) { planner.postMessage(m); }
export function onPlanner(f: (m: FromPlanner) => void) { planner.addEventListener('message', (e) => f(e.data as FromPlanner)); }

export function pingBoth(): Promise<void> {
  const p1 = new Promise<void>((res, rej) => {
    const t = setTimeout(() => rej(new Error('planner timeout')), 5000);
    const h = (e: MessageEvent) => { if (e.data.type === 'pong') { clearTimeout(t); planner.removeEventListener('message', h); res(); } };
    planner.addEventListener('message', h);
    planner.postMessage({ type: 'ping' });
  });
  return Promise.all([p1, request<string>({ kind: 'ping' })]).then(() => undefined);
}
