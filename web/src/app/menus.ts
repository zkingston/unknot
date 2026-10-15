// Input: the open menu, files and drag-and-drop, image/SVG import review, the code sheet; panel, about, mobile sheet.
import { ImportOverlay } from '../ui/importOverlay';
import { sampleSVG } from '../import/svg';
import type { TraceResult } from '../import/image/assemble';
import { parseKnotJSON } from '../import/json';
import { normalizeScene } from '../kernel/normalize';
import { PRESETS, VERY_HARD } from '../presets';
import { request } from '../state/workers';
import { $, esc, NAME, type CodeFormat } from './state';
import { loadKnot, loadPreset, loadText, showError } from './load';

// ---------------------------------------------------------------- open menu

const menu = $('m-open');
const groups = [...new Set(PRESETS.map((p) => p.group))];
menu.innerHTML = '<h3>input</h3><button class="btn" role="menuitem" data-act="file"><span>image, svg or json…</span></button>'
  + '<button class="btn" role="menuitem" data-act="code"><span>pd, gauss or braid code…</span></button>'
  + groups.map((g) => `<h3>${esc(g)}</h3>` + PRESETS.filter((p) => p.group === g).map((p) => `<button class="btn" role="menuitem" data-preset="${p.id}"><span>${esc(p.label)}</span></button>`).join('')
    + (g === 'hard unknots' ? `<button class="btn" role="menuitem" data-act="deepmind"><span>random from DeepMind’s ${VERY_HARD} very hard</span></button>` : '')).join('');
menu.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-preset],[data-act]');
  if (!b) return;
  closeMenu();
  if (b.dataset.preset) loadPreset(b.dataset.preset);
  else if (b.dataset.act === 'file') $<HTMLInputElement>('file').click();
  else if (b.dataset.act === 'code') openCodeSheet();
  else if (b.dataset.act === 'deepmind') loadPreset(`deepmind:${Math.floor(Math.random() * VERY_HARD)}`);
});
export function openMenu() {
  const b = $('b-open').getBoundingClientRect();
  menu.style.top = `${b.bottom + 4}px`; menu.style.right = `${Math.max(8, innerWidth - b.right)}px`;
  menu.style.maxHeight = `${Math.max(120, innerHeight - b.bottom - 16)}px`; // long lists scroll inside
  menu.classList.add('open'); $('b-open').setAttribute('aria-expanded', 'true');
  menu.querySelector<HTMLElement>('button')?.focus();
}
function closeMenu() {
  const had = menu.contains(document.activeElement);
  menu.classList.remove('open'); $('b-open').setAttribute('aria-expanded', 'false');
  if (had) $('b-open').focus();
}
$('b-open').addEventListener('click', (e) => { e.stopPropagation(); if (menu.classList.contains('open')) closeMenu(); else openMenu(); });
document.addEventListener('click', (e) => { if (!menu.contains(e.target as Node)) closeMenu(); });
menu.addEventListener('keydown', (e) => {
  const items = [...menu.querySelectorAll<HTMLElement>('button')], i = items.indexOf(document.activeElement as HTMLElement);
  if (e.key === 'ArrowDown') { e.preventDefault(); items[(i + 1) % items.length].focus(); }
  if (e.key === 'ArrowUp') { e.preventDefault(); items[(i + items.length - 1) % items.length].focus(); }
});

// ---------------------------------------------------------------- files and drag-and-drop

$<HTMLInputElement>('file').addEventListener('change', (e) => {
  const f = (e.target as HTMLInputElement).files?.[0];
  if (f) openFile(f);
  (e.target as HTMLInputElement).value = '';
});
async function openFile(f: File) {
  const name = f.name.replace(/\.(knot\.json|json|svg|png|jpe?g|gif|webp|bmp|txt)$/i, '');
  try {
    if (/\.json$/i.test(f.name) || f.type === 'application/json') {
      const l = parseKnotJSON(JSON.parse(await f.text()));
      if (l.legacy && l.scene) l.scene = normalizeScene(l.scene);
      await loadKnot(l, l.name ?? name, { kind: 'json' });
    } else if (/\.svg$/i.test(f.name) || f.type === 'image/svg+xml') await importSVG(await f.text(), name);
    else if (f.type.startsWith('image/')) await importImage(f, name);
    else if (!(await loadCode(await f.text()))) openCodeSheet();
  } catch (e) { showError(e); }
}
const drop = $('drop');
let dragDepth = 0;
addEventListener('dragenter', (e) => { if (e.dataTransfer?.types.includes('Files') || e.dataTransfer?.types.includes('text/plain')) { dragDepth++; drop.classList.add('on'); e.preventDefault(); } });
addEventListener('dragleave', () => { if (--dragDepth <= 0) { dragDepth = 0; drop.classList.remove('on'); } });
addEventListener('dragover', (e) => e.preventDefault());
addEventListener('drop', (e) => {
  e.preventDefault(); dragDepth = 0; drop.classList.remove('on');
  const f = e.dataTransfer?.files?.[0];
  if (f) openFile(f);
  else { const t = e.dataTransfer?.getData('text/plain'); if (t) loadCode(t).then((ok) => { if (!ok) { openCodeSheet(); $<HTMLTextAreaElement>('code').value = t; } }); }
});

// ---------------------------------------------------------------- image and SVG import: trace in the worker, review in the overlay

let importName = '', importKind: 'image' | 'svg' = 'image', importURL = '';
const overlay = new ImportOverlay($('o-import'), $('importsheet'), {
  retrace: (opts) => request<TraceResult>({ kind: 'retrace', opts }),
  accept: (d) => { URL.revokeObjectURL(importURL); loadKnot({ diagram: { ...d, source: importKind } }, importName, { kind: importKind }).catch(showError); },
});
async function importImage(f: Blob, name: string) {
  const bitmap = await createImageBitmap(f);
  const r = await request<TraceResult>({ kind: 'image', bitmap }, [bitmap as unknown as Transferable]);
  importName = name || NAME.image; importKind = 'image';
  URL.revokeObjectURL(importURL);   // a cancelled earlier import's
  importURL = URL.createObjectURL(f);
  overlay.open(r, importURL);
}
async function importSVG(text: string, name: string) {
  const s = sampleSVG(text);
  if (s.filled || !s.arcs.length) {
    // filled outlines (common on Commons): rasterise to 1000 px and use the image pipeline
    const img = new Image(), url = URL.createObjectURL(new Blob([text], { type: 'image/svg+xml' }));
    await new Promise((res, rej) => { img.onload = res; img.onerror = () => rej(new Error('cannot rasterise the SVG')); img.src = url; });
    const k = 1000 / Math.max(img.naturalWidth || 1000, img.naturalHeight || 1000), c = document.createElement('canvas');
    c.width = Math.round((img.naturalWidth || 1000) * k); c.height = Math.round((img.naturalHeight || 1000) * k);
    const g = c.getContext('2d')!; g.fillStyle = '#fff'; g.fillRect(0, 0, c.width, c.height); g.drawImage(img, 0, 0, c.width, c.height);
    const blob = await new Promise<Blob>((res) => c.toBlob((b) => res(b!), 'image/png'));
    URL.revokeObjectURL(url);
    await importImage(blob, name);
    importKind = 'svg';
    return;
  }
  const r = await request<TraceResult>({ kind: 'arcs', arcs: s.arcs, sw: s.sw });
  importName = name || NAME.svg; importKind = 'svg';
  overlay.open(r, '');
}

// ---------------------------------------------------------------- paste a PD, Gauss or braid code

/** PD has X[..] or [[..]]; Gauss lists every label twice with both signs; anything else is a braid word. */
function detectFormat(t: string): CodeFormat {
  if (/X\s*\[|\[\s*\[/.test(t)) return 'pd';
  const nums = (t.split(';')[0].match(/[+\-−]?\d+/g) ?? []).map((x) => Number(x.replace('−', '-')));
  if (!/[a-zA-Zσ{]/.test(t.split(';')[0]) && nums.length >= 2) {
    const cnt = new Map<number, number[]>();
    for (const x of nums) { const l = cnt.get(Math.abs(x)) ?? []; l.push(Math.sign(x)); cnt.set(Math.abs(x), l); }
    if ([...cnt.values()].every((l) => l.length === 2 && l[0] !== l[1])) return 'gauss';
  }
  return 'braid';
}
$('textsheet').innerHTML = `<h2>Code</h2>
  <p class="hints">PD <code>X[1,5,2,4], X[3,1,4,6], X[5,3,6,2]</code> or <code>[[1,5,2,4],…]</code> · Gauss <code>1, −2, 3, −1, 2, −3; +,+,+</code> · braid <code>s1 s2^-1 s1</code>, <code>{1,-2,1}</code>, <code>aBa</code></p>
  <textarea id="code" rows="4" aria-label="code" spellcheck="false"></textarea>
  <div class="row"><span class="tog" role="group" aria-label="format" id="fmt"><button data-f="auto" aria-pressed="true"><span>auto</span></button><span class="sep">|</span><button data-f="pd"><span>pd</span></button><span class="sep">|</span><button data-f="gauss"><span>gauss</span></button><span class="sep">|</span><button data-f="braid"><span>braid</span></button></span>
  <button class="btn" id="code-load"><span>load</span></button><button class="btn" id="code-cancel"><span>cancel</span></button></div>
  <p class="editor-note" id="code-err" aria-live="polite"></p>`;
let fmt: 'auto' | CodeFormat = 'auto';
$('fmt').addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-f]'); if (!b) return;
  fmt = b.dataset.f as typeof fmt;
  $('fmt').querySelectorAll('button').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
});
function openCodeSheet() { $('o-text').classList.add('open'); $('code-err').textContent = ''; $<HTMLTextAreaElement>('code').focus(); }
function closeCodeSheet() { const had = $('o-text').contains(document.activeElement); $('o-text').classList.remove('open'); if (had) $('b-open').focus(); }
$('code-cancel').addEventListener('click', closeCodeSheet);
$('code-load').addEventListener('click', () => loadCode($<HTMLTextAreaElement>('code').value, fmt).then((ok) => { if (ok) closeCodeSheet(); }));
$<HTMLTextAreaElement>('code').addEventListener('keydown', (e) => { if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) $('code-load').click(); });

async function loadCode(text: string, f: 'auto' | CodeFormat = 'auto'): Promise<boolean> {
  const format = f === 'auto' ? detectFormat(text) : f;
  try { await loadText(format, text); return true; }
  catch (e) { $('code-err').textContent = `${NAME[format]}: ${e instanceof Error ? e.message : String(e)}`; return false; }
}

// ---------------------------------------------------------------- panel, about, mobile sheet

$('b-collapse').addEventListener('click', () => {
  const c = $('panel').classList.toggle('collapsed');
  $('b-collapse').setAttribute('aria-expanded', String(!c));
  $('b-collapse').firstElementChild!.textContent = c ? '‹' : '›';
});
function openAbout() { $('o-about').classList.add('open'); $('about').scrollTop = 0; $('about-close').focus({ preventScroll: true }); }
function closeAbout() { if ($('o-about').classList.contains('open')) { $('o-about').classList.remove('open'); $('b-about').focus(); } }
$('b-about').addEventListener('click', openAbout);
$('b-about-m').addEventListener('click', openAbout);
$('about-close').addEventListener('click', closeAbout);
$('o-about').addEventListener('click', (e) => { if (e.target === $('o-about')) closeAbout(); });
// mobile: [ more ] shows the panel as a bottom sheet; about lives there too
const sheetOpen = () => $('panel').classList.contains('sheet-open');
function setSheet(on: boolean) { $('panel').classList.toggle('sheet-open', on); $('b-more').setAttribute('aria-expanded', String(on)); }
$('b-more').addEventListener('click', (e) => { e.stopPropagation(); setSheet(!sheetOpen()); });
$('b-sheet-close').addEventListener('click', () => { setSheet(false); $('b-more').focus(); });
// a tap anywhere outside the sheet (and its overlays) closes it and does nothing else
let swallowUntil = 0;
document.addEventListener('pointerdown', (e) => {
  const t = e.target as Element;
  if (!sheetOpen() || $('panel').contains(t) || $('b-more').contains(t) || t.closest?.('.overlay')) return;
  e.preventDefault(); e.stopPropagation(); setSheet(false);
  swallowUntil = performance.now() + 600;
}, true);
document.addEventListener('click', (e) => { if (performance.now() < swallowUntil) { swallowUntil = 0; e.preventDefault(); e.stopPropagation(); } }, true);

/** Escape: close every menu, sheet and overlay. */
export function closeAll() {
  closeMenu(); closeCodeSheet(); closeAbout();
  if (overlay.isOpen) overlay.close();
  if (sheetOpen()) setSheet(false);
}
