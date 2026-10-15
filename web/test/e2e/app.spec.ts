import { test, expect, type Page } from '@playwright/test';
import { fileURLToPath } from 'node:url';
import { resolve, dirname } from 'node:path';

const distUrl = 'file://' + resolve(dirname(fileURLToPath(import.meta.url)), '../../dist/index.html');
const ready = (p: Page) => expect(p.locator('body')).toHaveAttribute('data-ready', '1', { timeout: 15_000 });
/** Open the editor and draw a figure-eight shadow (4 crossings) with the mouse. */
async function drawFigureEight(page: Page) {
  await page.click('#b-draw');
  const box = (await page.locator('#stage').boundingBox())!;
  const cx = box.x + box.width / 2, cy = box.y + box.height / 2, R = Math.min(box.width, box.height) * 0.14;
  const pt = (t: number): [number, number] => [cx + R * (2 + Math.cos(2 * t)) * Math.cos(3 * t), cy - R * (2 + Math.cos(2 * t)) * Math.sin(3 * t)];
  await page.mouse.move(...pt(0)); await page.mouse.down();
  for (let k = 1; k <= 240; k++) await page.mouse.move(...pt((2 * Math.PI * k) / 240 - 0.02));
  await page.mouse.up();
}
/** Some headless browsers on GPU-less hosts have no WebGL2; 3D checks skip there (E2E 6 covers that path). */
async function need3d(p: Page) {
  const ok = await p.evaluate(() => (window as any).__unknot.stage.has3d);
  test.skip(!ok, 'no WebGL2 in this headless browser');
}

for (const [name, url] of [['preview', '/'], ['file', distUrl]] as const) {
  test(`E2E 1: default load, untangle, solved within 3 s, scrub (${name})`, async ({ page }) => {
    await page.goto(url);
    await ready(page);
    await expect(page.locator('body')).toHaveAttribute('data-knot', /Culprit/);
    await expect(page.locator('#stats')).toContainText('Jones V(t)1');
    await expect(page.locator('#b-run')).toHaveText(/untangle/);
    await page.click('#b-run');
    await expect(page.locator('#status')).toContainText('Solved in', { timeout: 3000 });
    // the shortcut playback replaces the planner's log once the worker has it
    await page.waitForFunction(() => (window as any).__unknot.app.raw !== null, null, { timeout: 15_000 });
    // pause and scrub to the middle
    await page.keyboard.press('Space');
    const box = (await page.locator('#scrub').boundingBox())!;
    await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
    const [r, total] = (await page.locator('#counter').textContent())!.split(' / ').map(Number);
    expect(total).toBeGreaterThan(10);
    expect(r).toBeGreaterThan(0);
    expect(r).toBeLessThan(total);
    // keyboard step on the scrub moves one record
    await page.locator('#scrub').focus();
    await page.keyboard.press('ArrowRight');
    const r2 = Number((await page.locator('#counter').textContent())!.split(' / ')[0]);
    expect(r2).toBeGreaterThanOrEqual(r);
    expect(await page.locator('#view2d path.k').count()).toBeGreaterThan(0);
    if (name === 'file') await expect(page).toHaveScreenshot('e2e1-scrubbed.png', { maxDiffPixelRatio: 0.005 });
    // flip to 3D and back: the view direction never changes (where WebGL2 exists)
    if (!(await page.evaluate(() => (window as any).__unknot.stage.has3d))) { test.info().annotations.push({ type: 'note', description: 'flip not checked: no WebGL2' }); return; }
    const before = await page.evaluate(() => JSON.stringify((window as any).__unknot.stage.view));
    await page.click('#b-3d');
    await expect(page.locator('body')).toHaveAttribute('data-view', '3d');
    await page.click('#b-sticks');
    await expect(page.locator('body')).toHaveAttribute('data-style', 'sticks');
    await page.click('#b-2d');
    await expect(page.locator('#view2d circle.v').first()).toBeVisible();
    await page.keyboard.press('s');
    await expect(page.locator('body')).toHaveAttribute('data-style', 'curve');
    await page.click('#b-3d');
    await page.click('#b-2d');
    await expect(page.locator('body')).toHaveAttribute('data-view', '2d');
    expect(await page.evaluate(() => JSON.stringify((window as any).__unknot.stage.view))).toBe(before);
  });
}

test('presets all load: knots say Knotted, unknots show Jones 1', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  for (const [id, name] of [['monster', 'Monster'], ['d28', 'D28'], ['deepmind-42', 'DeepMind 42'], ['goeritz', 'Goeritz'], ['thistlethwaite', 'Thistlethwaite'], ['ochiai', 'Ochiai I'], ['trefoil', 'Trefoil'], ['figure-eight', 'Figure-eight'], ['cinquefoil', 'Cinquefoil'], ['three-twist', 'Three-twist']]) {
    await page.click('#b-open');
    await page.click(`[data-preset="${id}"]`);
    await expect(page.locator('body')).toHaveAttribute('data-knot', new RegExp(name), { timeout: 10_000 });
    if (['trefoil', 'figure-eight', 'cinquefoil', 'three-twist'].includes(id)) await expect(page.locator('#status')).toContainText('Knotted.', { timeout: 10_000 });
    else await expect(page.locator('#stats')).toContainText('Jones V(t)1', { timeout: 10_000 });
  }
});

test('E2E 6: without WebGL2 the 3D toggle is disabled with its message and 2D works', async ({ page }) => {
  await page.addInitScript(() => {
    const orig = HTMLCanvasElement.prototype.getContext;
    HTMLCanvasElement.prototype.getContext = function (this: HTMLCanvasElement, kind: string, ...rest: unknown[]) {
      return kind === 'webgl2' ? null : (orig as any).call(this, kind, ...rest);
    } as typeof orig;
  });
  await page.goto('/');
  await ready(page);
  await expect(page.locator('#b-3d')).toBeDisabled();
  await expect(page.locator('#notice')).toHaveText('3d needs WebGL2');
  await page.keyboard.press('v');
  await expect(page.locator('body')).not.toHaveAttribute('data-view', '3d');
  await page.click('#b-run');
  await expect(page.locator('#status')).toContainText('Solved in', { timeout: 3000 });
  expect(await page.locator('#view2d path.k').count()).toBeGreaterThan(0);
});

test('prefers-reduced-motion makes the flip instant', async ({ page }) => {
  await page.emulateMedia({ reducedMotion: 'reduce' });
  await page.goto('/');
  await ready(page);
  await need3d(page);
  const ms = await page.evaluate(async () => { const t0 = performance.now(); await (window as any).__unknot.flip('3d'); return performance.now() - t0; });
  expect(ms).toBeLessThan(100);
  await expect(page.locator('body')).toHaveAttribute('data-view', '3d');
});

for (const preset of ['trefoil', 'culprit']) test(`3D straight on (${preset}): at every crossing the rear tube outline is interrupted by the front tube`, async ({ page }) => {
  await page.goto('/');
  await ready(page);
  await need3d(page);
  await page.evaluate((id) => (window as any).__unknot.loadPreset(id), preset);
  await page.evaluate(() => (window as any).__unknot.flip('3d'));
  await expect(page.locator('body')).toHaveAttribute('data-view', '3d');
  const res = await page.evaluate(() => {
    const st = (window as any).__unknot.stage;
    st.redraw();
    const d = st.drawn, v = st.v3d, out: { k: number; ink: number; samples: number }[] = [];
    const rpx = 0.03 * st.frame.s;
    const arcs = d.curves.map((Q: Float64Array) => { const n = Q.length / 2, A = new Float64Array(n + 1); for (let i = 0; i < n; i++) { const j = (i + 1) % n; A[i + 1] = A[i] + Math.hypot(Q[2 * j] - Q[2 * i], Q[2 * j + 1] - Q[2 * i + 1]); } return A; });
    d.crossings.forEach((X: any, k: number) => {
      const o = X.aOver ? X.a : X.b, u = X.aOver ? X.b : X.a;
      const dir = (s: any) => { const Q = d.curves[s.comp], n = Q.length / 2, j = (s.seg + 1) % n; const dx = Q[2 * j] - Q[2 * s.seg], dy = Q[2 * j + 1] - Q[2 * s.seg + 1], l = Math.hypot(dx, dy); return [dx / l, dy / l]; };
      const a = dir(o), b = dir(u), sin = Math.abs(a[0] * b[1] - a[1] * b[0]);
      if (sin < 0.35) return;
      const L = Math.min(20, (rpx + 3) / sin + 2);
      // only crossings with no third strand inside the sampled window (tight layouts can crowd a crossing)
      // same strand: within twice the crowding radius of arc length from the crossing
      const reach = 2 * (L + 2 * rpx + 4);
      const near = (s: any, c: number, i: number) => {
        if (c !== s.comp) return false;
        const Q = d.curves[c], n = Q.length / 2, A = arcs[c], tot = A[n], da = Math.abs(A[i] - A[s.seg]);
        return Math.min(da, tot - da) <= reach;
      };
      let crowded = false;
      d.curves.forEach((Q: Float64Array, c: number) => { const n = Q.length / 2; for (let i = 0; i < n && !crowded; i++) {
        if (near(o, c, i) || near(u, c, i)) continue;
        const j = (i + 1) % n, px = Q[2 * i], py = Q[2 * i + 1], dx = Q[2 * j] - px, dy = Q[2 * j + 1] - py, l2 = dx * dx + dy * dy;
        let t = l2 > 0 ? ((X.x - px) * dx + (X.y - py) * dy) / l2 : 0; t = Math.max(0, Math.min(1, t));
        if (Math.hypot(px + t * dx - X.x, py + t * dy - X.y) < L + 2 * rpx + 4) crowded = true;
      } });
      if (crowded) return;
      let ink = 0, samples = 0;
      for (let t = -L; t <= L; t += 0.5) {
        const p = v.readPixel(X.x + a[0] * t, X.y + a[1] * t);
        samples++;
        if (p[0] < 128) ink++;
      }
      out.push({ k, ink, samples });
    });
    return out;
  });
  expect(res.length).toBeGreaterThanOrEqual(3);
  for (const r of res) expect(r.ink, `crossing ${r.k}`).toBe(0);
});

test('E2E 2: draw a figure-eight, flip one crossing, the status changes', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  await drawFigureEight(page);
  await expect(page.locator('#status')).toContainText('Knotted.', { timeout: 5000 });
  const c = await page.evaluate(() => { const s = (window as any).__unknot.editor.shown; return s.map((x: any) => [x.X.x, x.X.y]); });
  expect(c.length).toBe(4);
  const sb = (await page.locator('svg.editor').boundingBox())!;
  await page.mouse.click(sb.x + c[0][0], sb.y + c[0][1]);
  await expect(page.locator('#status')).not.toContainText('Knotted', { timeout: 5000 });
  // reshape: drag a control point; the curve follows and undo puts it back
  expect(await page.locator('svg.editor circle.handle').count()).toBeGreaterThan(5);
  const h0 = await page.evaluate(() => (window as any).__unknot.editor.strokes[0].ctrl.slice(0, 2));
  await page.mouse.move(sb.x + h0[0], sb.y + h0[1]); await page.mouse.down();
  for (let k = 1; k <= 5; k++) await page.mouse.move(sb.x + h0[0] + 3 * k, sb.y + h0[1]);
  await page.mouse.up();
  expect(await page.evaluate(() => (window as any).__unknot.editor.strokes[0].ctrl[0])).toBeCloseTo(h0[0] + 15, 0);
  await page.keyboard.press('Control+z');
  expect(await page.evaluate(() => (window as any).__unknot.editor.strokes[0].ctrl[0])).toBe(h0[0]);
  await page.click('.editor-bar [data-a=done]');
  await expect(page.locator('body')).toHaveAttribute('data-knot', /Drawn knot/, { timeout: 10_000 });
  await expect(page.locator('#stats')).toContainText('Jones V(t)1');
  await page.click('#b-run');
  await expect(page.locator('#status')).toContainText('Solved in', { timeout: 5000 });
});

const fixture = (f: string) => resolve(dirname(fileURLToPath(import.meta.url)), '../fixtures', f);

test('E2E 3: import complex.png, accept, untangle, solved', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  await page.setInputFiles('#file', fixture('complex.png'));
  await expect(page.locator('#o-import')).toHaveClass(/open/, { timeout: 10_000 });
  await expect(page.locator('.trace-info')).toContainText('6 crossings · 1 component · 0 unpaired · 0 flat');
  await page.click('#importsheet [data-a=accept]');
  await expect(page.locator('body')).toHaveAttribute('data-knot', /complex/, { timeout: 10_000 });
  await expect(page.locator('#stats')).toContainText('crossings6');
  await expect(page.locator('#stats')).toContainText('Jones V(t)1');
  await page.click('#b-run');
  await expect(page.locator('#status')).toContainText('Solved in', { timeout: 5000 });
});

test('E2E 4: paste the figure-eight PD: Knotted, and the run stops at >= 6 sticks', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  await page.fill('#i-budget', '2'); await page.locator('#i-budget').dispatchEvent('change');
  await page.click('#b-open');
  await page.click('[data-act=code]');
  await page.fill('#code', 'X[4,2,5,1], X[8,6,1,5], X[6,3,7,4], X[2,7,3,8]');
  await page.click('#code-load');
  await expect(page.locator('#status')).toContainText('Knotted.', { timeout: 10_000 });
  await expect(page.locator('#stats')).toContainText('Alexander Δ(t)−t + 3 − t⁻¹');
  await expect(page.locator('#b-run')).toHaveText(/simplify/);
  await page.click('#b-run');
  await expect(page.locator('body')).toHaveAttribute('data-run', 'stopped', { timeout: 15_000 });
  await expect(page.locator('#status')).toContainText('Knotted: simplified to');
  const n = await page.evaluate(() => (window as any).__unknot.app.traj.final.reduce((s: number, P: Float64Array) => s + P.length / 3, 0));
  expect(n).toBeGreaterThanOrEqual(6);
});

import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
const shaOf = (xs: number[]) => (createHash('sha256').update(new Uint8Array(Float64Array.from(xs).buffer)) as unknown as { digest(e: 'hex'): string }).digest('hex');
const GOLD = JSON.parse(readFileSync(fixture('determinism.json'), 'utf8')) as Record<string, { scene: string; log: string; records: number }>;

/** Run the planner in the page on the loaded knot (seed and planner as set) and return canonical bytes. */
async function runAndRead(page: Page, maxRecords?: number) {
  await page.evaluate(() => { document.body.removeAttribute('data-run'); (window as any).__unknot.startRun(); });
  await expect(page.locator('body')).toHaveAttribute('data-run', /solved|stopped/, { timeout: 20_000 });
  return page.evaluate((max) => {
    const app = (window as any).__unknot.app, R: Float64Array = (app.raw ?? app.traj).records, c = Array.from(R);
    for (let o = 0; o < c.length; o += 6) if (c[o] === 2) c[o + 5] = 0;
    const scene = app.knot.scene.flatMap((P: Float64Array) => Array.from(P));
    return { scene, log: max ? c.slice(0, 6 * max) : c, records: c.length / 6 };
  }, maxRecords);
}

test('determinism: each preset at seed 7 has the Node SHA-256 for its input Scene and MoveLog', async ({ page }) => {
  test.setTimeout(120_000);
  await page.goto('/');
  await ready(page);
  for (const [id, g] of Object.entries(GOLD)) {
    await page.evaluate((i) => (window as any).__unknot.loadPreset(i), id);
    const knotted = id === 'trefoil' || id === 'figure-eight';
    // knotted runs end on the budget (only a prefix is compared); unknots must finish, so give them room
    await page.fill('#i-budget', knotted ? '2' : '30'); await page.locator('#i-budget').dispatchEvent('change');
    const r = await runAndRead(page, knotted ? g.records : undefined);
    expect(shaOf(r.scene), `${id} scene`).toBe(g.scene);
    if (knotted) expect(r.records).toBeGreaterThanOrEqual(g.records);
    expect(shaOf(r.log), `${id} log`).toBe(g.log);
  }
});

test('E2E 5: copy a share link, reload it: same knot, view, Scene bytes and MoveLog hash', async ({ page, context }) => {
  await page.goto('/');
  await ready(page);
  // a drawn knot exercises quantisation on commit: the figure-eight shadow with one crossing flipped (an unknot)
  await drawFigureEight(page);
  await expect(page.locator('#status')).toContainText('Knotted.', { timeout: 5000 });
  const c = await page.evaluate(() => (window as any).__unknot.editor.shown.map((x: any) => [x.X.x, x.X.y]));
  const sb = (await page.locator('svg.editor').boundingBox())!;
  await page.mouse.click(sb.x + c[0][0], sb.y + c[0][1]);
  await expect(page.locator('#status')).not.toContainText('Knotted', { timeout: 5000 });
  await page.click('.editor-bar [data-a=done]');
  await expect(page.locator('body')).toHaveAttribute('data-knot', /Drawn knot/, { timeout: 10_000 });
  const has3d = await page.evaluate(() => (window as any).__unknot.stage.has3d);
  if (has3d) { await page.click('#b-3d'); await expect(page.locator('body')).toHaveAttribute('data-view', '3d'); }
  const a = await runAndRead(page);
  await expect(page.locator('#e-link')).toBeEnabled();
  const link = page.url();
  expect(link).toContain('&k=');
  const p2 = await context.newPage();
  await p2.goto(link);
  await ready(p2);
  await expect(p2.locator('body')).toHaveAttribute('data-knot', /Drawn knot/);
  await expect(p2.locator('body')).toHaveAttribute('data-view', has3d ? '3d' : '2d');
  const b = await runAndRead(p2);
  expect(shaOf(b.scene)).toBe(shaOf(a.scene));
  expect(shaOf(b.log)).toBe(shaOf(a.log));
});

test('E2E 7: keyboard-only pass through open, run, scrub, flip and export', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  const tabTo = async (sel: string, back = false) => {
    for (let k = 0; k < 80; k++) {
      if (await page.evaluate((s) => document.activeElement === document.querySelector(s), sel)) return;
      await page.keyboard.press(back ? 'Shift+Tab' : 'Tab');
    }
    throw new Error(`cannot reach ${sel} with Tab`);
  };
  // open: menu by keyboard, arrow to an example, Enter
  await tabTo('#b-open'); await page.keyboard.press('Enter');
  await expect(page.locator('#m-open')).toHaveClass(/open/);
  for (let k = 0; k < 40 && (await page.locator('#m-open button:focus').getAttribute('data-preset')) !== 'goeritz'; k++) await page.keyboard.press('ArrowDown');
  await expect(page.locator('#m-open button:focus')).toHaveAttribute('data-preset', 'goeritz');
  await page.keyboard.press('Enter');
  await expect(page.locator('body')).toHaveAttribute('data-knot', /Goeritz/, { timeout: 10_000 });
  // run
  await tabTo('#b-run'); await page.keyboard.press('Enter');
  await expect(page.locator('#status')).toContainText('Solved in', { timeout: 5000 });
  await page.keyboard.press('Enter'); // pause
  // scrub with the slider
  await tabTo('#scrub', true); // the scrub sits above the run controls
  await page.keyboard.press('Home');
  for (let k = 0; k < 5; k++) await page.keyboard.press('ArrowRight');
  expect(Number((await page.locator('#counter').textContent())!.split(' / ')[0])).toBeGreaterThan(0);
  await page.keyboard.press('Shift+ArrowRight');
  // flip (V) where 3D exists, and back
  if (await page.evaluate(() => (window as any).__unknot.stage.has3d)) {
    await page.keyboard.press('v'); await expect(page.locator('body')).toHaveAttribute('data-view', '3d');
    await page.keyboard.press('v'); await expect(page.locator('body')).toHaveAttribute('data-view', '2d');
  }
  // export: svg by keyboard (the panel precedes the timeline, so tab backwards)
  await tabTo('#e-svg', true);
  const dl = page.waitForEvent('download');
  await page.keyboard.press('Enter');
  expect((await dl).suggestedFilename()).toBe('goeritz.svg');
});

test('budgets in the browser: 2D frame <= 4 ms and 3D frame <= 4 ms CPU at n ~ 100; image import <= 1.5 s', async ({ page }) => {
  await page.goto('/');
  await ready(page);
  await page.evaluate(() => (window as any).__unknot.loadPreset('thistlethwaite'));
  // best of three: the suite runs three browsers in parallel, so one sample can catch another's load
  const best = async (f: () => Promise<number>) => Math.min(await f(), await f(), await f());
  const ms2 = await best(() => page.evaluate(() => (window as any).__unknot.bench2d(40)));
  console.log(`${test.info().project.name}: 2D frame ${ms2.toFixed(2)} ms`);
  expect(ms2).toBeLessThanOrEqual(4);
  if (await page.evaluate(() => (window as any).__unknot.stage.has3d)) {
    await page.evaluate(() => (window as any).__unknot.flip('3d'));
    const ms3 = await best(() => page.evaluate(() => (window as any).__unknot.bench3d(40)));
    console.log(`${test.info().project.name}: 3D frame ${ms3.toFixed(2)} ms CPU`);
    expect(ms3).toBeLessThanOrEqual(4);
  }
  const t0 = Date.now();
  await page.setInputFiles('#file', fixture('complex.png'));
  await expect(page.locator('#o-import')).toHaveClass(/open/, { timeout: 10_000 });
  const ms = Date.now() - t0;
  console.log(`${test.info().project.name}: image import ${ms} ms`);
  expect(ms).toBeLessThanOrEqual(1500);
});

test.describe('phone', () => {
  test.use({ viewport: { width: 412, height: 915 }, hasTouch: true });
  test('draw is offered; the more sheet closes from its button and from a tap outside', async ({ page }) => {
    await page.goto('/');
    await ready(page);
    await expect(page.locator('#b-draw')).toBeVisible();
    const open = () => page.evaluate(() => document.getElementById('panel')!.classList.contains('sheet-open'));
    await page.tap('#b-more'); expect(await open()).toBe(true);
    await page.tap('#b-sheet-close'); expect(await open()).toBe(false);
    await page.tap('#b-more'); expect(await open()).toBe(true);
    const box = (await page.locator('#stage').boundingBox())!;
    await page.touchscreen.tap(box.x + box.width / 2, box.y + 40);
    expect(await open()).toBe(false);
    await expect(page.locator('body')).toHaveAttribute('data-view', '2d');
  });
});
