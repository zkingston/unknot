// Timeline player: playback time τ (ms at 1x) drives the displayed Scene; scrub rule, speed, live mode.
import type { Scene } from '../kernel/scene';
import { type Trajectory, type Ghost } from '../planner/trajectory';
import { PhaseKind } from '../planner/movelog';
import { sparkline } from './sparkline';

const SPEEDS = [0.5, 1, 2, 4];

export class Player {
  traj: Trajectory | null = null;
  tau = 0;
  speed = 1;
  playing = false;
  live = false;
  private cursor?: { S: Scene; r: number };
  private last = 0;
  private raf = 0;
  /** Projected crossing counts sampled at some records (filled by the owner). */
  crossings: { r: number; c: number }[] = [];

  /** `area` covers the sparklines and the scrub (same x range): dragging anywhere on it seeks; `line` marks τ. */
  constructor(private el: { scrub: HTMLElement; head: HTMLElement; area: HTMLElement; line: HTMLElement; counter: HTMLElement; speed: HTMLElement; live: HTMLButtonElement; sticks: SVGSVGElement; energy: SVGSVGElement; cross: SVGSVGElement },
    private onFrame: (S: Scene, r: number, ghost: Ghost | null) => void, private onState: () => void) {
    el.speed.addEventListener('click', () => this.cycleSpeed());
    el.live.addEventListener('click', () => this.goLive());
    const seekAt = (e: PointerEvent) => {
      if (!this.traj) return;
      const b = el.area.getBoundingClientRect();
      this.seek(Math.min(1, Math.max(0, (e.clientX - b.left) / b.width)) * this.duration());
    };
    el.area.addEventListener('pointerdown', (e) => { if (!this.traj) return; el.area.setPointerCapture(e.pointerId); el.scrub.focus({ preventScroll: true }); this.detach(); seekAt(e); });
    el.area.addEventListener('pointermove', (e) => { if (el.area.hasPointerCapture(e.pointerId)) seekAt(e); });
    el.scrub.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') { e.preventDefault(); e.stopPropagation(); this.step(e.key === 'ArrowRight' ? 1 : -1, e.shiftKey); }
      else if (e.key === 'Home') { e.preventDefault(); this.detach(); this.seek(0); }
      else if (e.key === 'End') { e.preventDefault(); this.detach(); this.seek(this.duration()); }
    });
  }

  setTrajectory(t: Trajectory | null) {
    this.traj = t; this.tau = 0; this.cursor = undefined; this.crossings = []; this.playing = false; this.live = !!t && !t.done;
    this.update();
  }

  /** Total time at 1x; during live planning, a record-index scale. */
  duration() { const t = this.traj; return !t ? 0 : t.done ? t.T : Math.max(1, t.len); }

  play() {
    if (!this.traj) return;
    if (this.traj.done && this.tau >= this.duration()) this.tau = 0;
    this.playing = true;
    this.last = performance.now();
    cancelAnimationFrame(this.raf);
    this.raf = requestAnimationFrame((t) => this.tick(t));
    this.onState();
  }
  pause() { this.playing = false; cancelAnimationFrame(this.raf); this.onState(); }
  /** Done planning and the playhead is at the end. */
  ended() { return !!this.traj && this.traj.done && !this.live && this.tau >= this.duration(); }

  private tick(now: number) {
    if (!this.playing || !this.traj) return;
    const dt = Math.max(0, Math.min(100, now - this.last)); this.last = now; // a frame can be stamped before play() ran
    if (this.traj.done && !this.live) {
      this.tau += dt * this.speed;
      if (this.tau >= this.duration()) { this.tau = this.duration(); this.render(); this.pause(); return; }
    }
    this.render();
    this.raf = requestAnimationFrame((t) => this.tick(t));
  }

  /** Planning finished: switch from live to the timing rule, from the record currently shown. */
  finished() {
    const t = this.traj; if (!t) return;
    const r = this.live ? t.liveIndex() : this.recordAtTau();
    this.live = false;
    this.tau = t.endTime(r - 1);
    this.cursor = undefined;
    this.update();
  }

  private recordAtTau() { const t = this.traj!; return t.done ? t.indexAt(this.tau) : Math.min(t.len, Math.round(this.tau)); }

  seek(tau: number) { this.tau = Math.max(0, Math.min(tau, this.duration())); this.render(); }
  detach() { if (this.live) { this.live = false; this.tau = this.traj?.done ? this.tau : this.traj?.liveIndex() ?? 0; this.onState(); } }
  goLive() { if (this.traj && !this.traj.done) { this.live = true; this.play(); } }

  step(dir: number, big = false) {
    const t = this.traj; if (!t) return;
    this.detach();
    if (this.playing) this.pause();
    if (big) { this.seek(this.tau + dir * 0.1 * this.duration()); return; }
    if (!t.done) { this.seek(Math.round(this.tau) + dir); return; }
    const r = t.indexAt(this.tau); // records [0, r) complete
    this.seek(dir > 0 ? t.endTime(r) : t.endTime(r - 2));
  }

  cycleSpeed() { this.speed = SPEEDS[(SPEEDS.indexOf(this.speed) + 1) % SPEEDS.length]; this.el.speed.firstElementChild!.textContent = `${this.speed}x`; }

  /** Draw the current state and refresh the scrub UI. */
  render() {
    const t = this.traj;
    if (!t) return;
    let S: Scene, r: number, ghost: Ghost | null = null;
    if (!t.done) {
      r = this.live ? t.liveIndex() : Math.min(t.len, Math.round(this.tau));
      if (this.live) this.tau = r;
      this.cursor = t.stateAt(r, this.cursor); S = this.cursor.S.map((P) => P.slice());
    } else {
      const s = t.sceneAt(this.tau, this.cursor); S = s.S; r = s.r; this.cursor = s.cursor; ghost = s.ghost;
    }
    this.onFrame(S, r, ghost);
    this.update(r);
  }

  update(r = this.traj ? this.recordAtTau() : 0) {
    const t = this.traj, D = this.duration(), f = D ? this.tau / D : 0;
    this.el.head.style.left = this.el.line.style.left = `${(100 * f).toFixed(3)}%`;
    this.el.area.parentElement?.classList.toggle('has-run', !!t);
    this.el.counter.textContent = `${t ? r : 0} / ${t ? t.len : 0}`;
    this.el.scrub.setAttribute('aria-valuemax', String(t ? t.len : 0));
    this.el.scrub.setAttribute('aria-valuenow', String(t ? r : 0));
    this.el.scrub.setAttribute('aria-valuetext', `record ${t ? r : 0} of ${t ? t.len : 0}`);
    this.el.live.disabled = !t || t.done || this.live;
  }

  /** Sparklines and phase ticks, x by playback time once done, by record before. */
  drawSparks() {
    const t = this.traj;
    const all = t ? t.phases : [], step = Math.max(1, Math.floor(all.length / 600));
    const ph = all.filter((_, k) => k % step === 0 || k === all.length - 1); // at most ~600 points per line
    const x = (r: number) => (!t ? 0 : t.done ? t.endTime(r - 1) / t.T : r / Math.max(1, t.len));
    // every line starts at x = 0 from the input: its stick count (energy has no sample before the first phase)
    const xs = t ? [0, ...ph.map((p) => x(p.r))] : [], n0 = t ? t.initial.reduce((s, P) => s + P.length / 3, 0) : 0;
    sparkline(this.el.sticks, xs, t ? [n0, ...ph.map((p) => p.n)] : []);
    sparkline(this.el.energy, ph.length ? xs : [], ph.length ? [ph[0].energy, ...ph.map((p) => p.energy)] : [], true);
    const cs = this.crossings.filter((_, k) => k % Math.max(1, Math.floor(this.crossings.length / 600)) === 0 || k === this.crossings.length - 1);
    sparkline(this.el.cross, cs.map((q) => x(q.r)), cs.map((q) => q.c));
    this.el.scrub.querySelectorAll('.tick').forEach((e) => e.remove());
    const outer = all.filter((p) => p.kind === PhaseKind.elm || p.kind === PhaseKind.tree || p.kind === PhaseKind.pickup || p.kind === PhaseKind.finish);
    if (outer.length <= 60) for (const p of outer) {
      const d = document.createElement('div'); d.className = 'tick'; d.style.left = `${(100 * x(p.r)).toFixed(3)}%`;
      this.el.scrub.insertBefore(d, this.el.head);
    }
  }
}
