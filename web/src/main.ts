// Entry: the app modules wire themselves to the DOM on import; this adds the keyboard, boot and debug hooks.
import './ui/styles.css';
import { app } from './app/state';
import { stage, draw, editor, flip, setSticks, openEditor, closeEditor, runBestView } from './app/view';
import { player, runButton, resetRun, startRun, takeOver } from './app/run';
import { loadPreset, loadFromHash, showError } from './app/load';
import './app/settings';
import './app/export';
import { openMenu, closeAll } from './app/menus';
import { pingBoth } from './state/workers';

document.addEventListener('keydown', (e) => {
  if (editor.keydown(e)) return;
  if (e.key === 'Escape' && editor.isOpen) { closeEditor(); return; }
  const t = e.target as HTMLElement;
  if (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || e.ctrlKey || e.metaKey || e.altKey) return;
  switch (e.key.toLowerCase()) {
    case ' ': if (t.tagName !== 'BUTTON') { e.preventDefault(); runButton(); } break;
    case 'arrowleft': case 'arrowright': if (t.id !== 'scrub') { e.preventDefault(); takeOver(); player.step(e.key === 'ArrowRight' ? 1 : -1, e.shiftKey); } break;
    case 'b': runBestView(); break;
    case 'v': if (!editor.isOpen) flip(); break;
    case 's': if (!editor.isOpen) setSticks(!stage.sticks); break;
    case 'd': openEditor(); break;
    case 'r': resetRun(); break;
    case 'o': e.preventDefault(); openMenu(); break;
    case 'escape': closeAll(); break;
  }
});

pingBoth().then(() => { document.body.dataset.workers = 'ok'; }, (e) => { document.body.dataset.workers = 'error: ' + e; });
document.body.dataset.view = '2d'; document.body.dataset.style = 'curve';
loadFromHash().catch((e) => { showError(e); return false; })
  .then((ok) => (ok ? undefined : loadPreset('culprit')))
  .then(() => { document.body.dataset.ready = '1'; });

// debug/test hooks
(window as unknown as { __unknot: unknown }).__unknot = {
  app, player, stage, editor, flip, loadPreset, startRun,
  /** ms per 2D frame (smooth + project + gaps + SVG) over `k` frames of the current knot */
  bench2d(k = 50) { const S = stage.shown!; const t0 = performance.now(); for (let i = 0; i < k; i++) draw(S); return (performance.now() - t0) / k; },
  /** CPU ms per 3D frame (smooth + tube mesh + upload + draw calls) over `k` frames */
  bench3d(k = 50) { const S = stage.shown!; const t0 = performance.now(); for (let i = 0; i < k; i++) stage.show(S); return (performance.now() - t0) / k; },
};
