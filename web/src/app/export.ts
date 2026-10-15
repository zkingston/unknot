// Export: share link, PD and Gauss codes, SVG, PNG and .knot.json.
import type { Diagram } from '../diagram/types';
import { projectScene } from '../diagram/project';
import { pdCode, formatPD, gaussCode } from '../invariants/pd';
import { standaloneSVG, pngFromSVG, download } from '../view2d/export';
import { diagramToJSON } from '../import/json';
import { $, app } from './state';
import { stage } from './view';

/** Copy from a click: the Clipboard API where the page may use it (secure origins), else select-and-copy, which also
 *  works over plain http (a LAN or Tailscale address). Called before any await, so the click still counts. */
async function copy(text: string, ok: string, fail: string) {
  const note = (s: string) => { $('export-note').textContent = s; };
  if (window.isSecureContext && navigator.clipboard) {
    try { await navigator.clipboard.writeText(text); note(ok); return; } catch { /* refused: try the fallback */ }
  }
  const back = document.activeElement as HTMLElement | null, ta = document.createElement('textarea');
  ta.value = text; ta.readOnly = true; ta.style.cssText = 'position:fixed;top:0;left:0;width:1px;height:1px;opacity:0';
  document.body.append(ta); ta.focus(); ta.select(); ta.setSelectionRange(0, text.length);
  let done = false;
  try { done = document.execCommand('copy'); } catch { /* unsupported */ }
  ta.remove(); back?.focus();
  note(done ? ok : fail);
}

// the address bar already holds the link (updateHash runs on every change; the button is disabled when it cannot)
$('e-link').addEventListener('click', () => copy(location.href, 'Link copied.', 'Copy failed; the link is in the address bar.'));
// PD and Gauss codes of the diagram on screen (shown state, current view)
for (const [id, name, code] of [['e-pd', 'PD code', (d: Diagram) => formatPD(pdCode(d).pd)], ['e-gauss', 'Gauss code', gaussCode]] as const) $(id).addEventListener('click', async () => {
  if (stage.shown) await copy(code(projectScene(stage.shown, stage.view)), `${name} copied.`, 'Copy failed.');
});

const fileBase = () => (app.knot?.name ?? 'knot').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'knot';
function currentSVG() {
  const cs = getComputedStyle(document.documentElement);
  return standaloneSVG(stage.drawn?.paths ?? [], Math.round(stage.W), Math.round(stage.H), cs.getPropertyValue('--ink').trim(), cs.getPropertyValue('--paper').trim());
}
$('e-svg').addEventListener('click', () => download(`${fileBase()}.svg`, currentSVG(), 'image/svg+xml'));
$('e-png').addEventListener('click', async () => download(`${fileBase()}.png`, await pngFromSVG(currentSVG(), Math.round(stage.W), Math.round(stage.H), 2)));
$('e-knot').addEventListener('click', () => {
  const k = app.knot; if (!k) return;
  const f = { version: 1, name: k.name, ...(k.credit ? { credit: k.credit } : {}), diagram: diagramToJSON(k.diagram), scene: k.scene.map((P) => Array.from(P)) };
  download(`${fileBase()}.knot.json`, JSON.stringify(f));
});
