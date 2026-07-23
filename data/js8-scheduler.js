// Single time source for every JS8 periodic task: TX pacing, auto replies,
// heartbeat, restrictions and file-transfer timeouts.
//
// The point of this module is the seam, not the algorithm. In L1 the browser
// tab must stay visible, so `tick()` is driven by a plain setInterval and the
// clock is Date.now(). In L3 (background operation) the same scheduler is fed
// by arriving RX audio frames and the clock becomes media time derived from the
// sample counter -- swapping `setClock()` and the tick source, with no change to
// any registered task.
//
// Rule that keeps that upgrade cheap: never call setTimeout/setInterval outside
// this module. Anything time-driven registers here instead.

(function (root, factory) {
  const value = factory();
  if (typeof module === "object" && module.exports) module.exports = value;
  else root.Js8Scheduler = value;
})(typeof globalThis !== "undefined" ? globalThis : self, function () {
  // A repeating task whose due time is far in the past (hidden tab, throttled
  // timer, clock jump) must not fire once per missed period. Coalesce to a
  // single run and reschedule from the current time.
  const MAX_BACKLOG_RUNS = 1;

  class Js8Scheduler {
    constructor({wallNow = () => Date.now(), onEvent = null} = {}) {
      this.wallNow = wallNow;
      this.onEvent = onEvent;
      this.tasks = new Map();
      this.lastTickUtcMs = null;
      this.stats = {ticks: 0, runs: 0, coalesced: 0, errors: 0};
    }

    // Swap the clock without touching registered tasks (L1 -> L3).
    setClock(wallNow) {
      this.wallNow = wallNow;
      this.lastTickUtcMs = null; // a new clock is a new epoch; do not compare across it
      return this;
    }

    now() { return this.wallNow(); }

    every(id, intervalMs, run, {startDelayMs = null} = {}) {
      if (!(intervalMs > 0)) throw new Error("scheduler interval must be positive");
      const now = this.now();
      this.tasks.set(id, {id, run, intervalMs, repeat: true,
        dueUtcMs: now + (startDelayMs === null ? intervalMs : startDelayMs)});
      return this;
    }

    after(id, delayMs, run) {
      this.tasks.set(id, {id, run, intervalMs: 0, repeat: false,
        dueUtcMs: this.now() + Math.max(0, delayMs)});
      return this;
    }

    at(id, utcMs, run) {
      this.tasks.set(id, {id, run, intervalMs: 0, repeat: false, dueUtcMs: utcMs});
      return this;
    }

    cancel(id) { return this.tasks.delete(id); }
    has(id) { return this.tasks.has(id); }
    clear() { this.tasks.clear(); return this; }

    // Milliseconds until the given task fires, or null when it is not scheduled.
    dueIn(id) {
      const task = this.tasks.get(id);
      return task ? task.dueUtcMs - this.now() : null;
    }

    // Run everything due at nowUtcMs. Fed by setInterval in L1, by arriving RX
    // audio frames in L3. Safe to call at any cadence.
    tick(nowUtcMs = this.now()) {
      this.stats.ticks += 1;
      // A backwards clock (correction applied, epoch change) must not strand
      // tasks in a far future; pull them back to the new now.
      if (this.lastTickUtcMs !== null && nowUtcMs < this.lastTickUtcMs) {
        for (const task of this.tasks.values())
          if (task.dueUtcMs > nowUtcMs + task.intervalMs)
            task.dueUtcMs = nowUtcMs + task.intervalMs;
        this._emit({type: "clock-backwards", fromUtcMs: this.lastTickUtcMs,
          toUtcMs: nowUtcMs});
      }
      this.lastTickUtcMs = nowUtcMs;

      for (const task of [...this.tasks.values()]) {
        if (nowUtcMs < task.dueUtcMs) continue;
        if (task.repeat) {
          const missed = Math.floor((nowUtcMs - task.dueUtcMs) / task.intervalMs);
          if (missed > MAX_BACKLOG_RUNS) {
            this.stats.coalesced += missed;
            this._emit({type: "coalesced", id: task.id, missed,
              lateMs: nowUtcMs - task.dueUtcMs});
          }
          task.dueUtcMs = nowUtcMs + task.intervalMs;
        } else {
          this.tasks.delete(task.id);
        }
        this.stats.runs += 1;
        try {
          task.run(nowUtcMs);
        } catch (error) {
          this.stats.errors += 1;
          this._emit({type: "error", id: task.id,
            message: String(error && error.message || error)});
        }
      }
      return this.stats.runs;
    }

    snapshot() {
      const now = this.now();
      return {...this.stats, taskCount: this.tasks.size,
        tasks: [...this.tasks.values()].map(task => ({id: task.id,
          repeat: task.repeat, inMs: task.dueUtcMs - now}))};
    }

    _emit(event) { if (this.onEvent) this.onEvent(event); }
  }

  return {Js8Scheduler, MAX_BACKLOG_RUNS};
});
