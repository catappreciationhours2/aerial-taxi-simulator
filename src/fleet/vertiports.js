// Vertiports as capacity-constrained scheduling resources: landing/take-off pads (time-slot booking),
// charging stations, and a hangar. Pads are booked for a time window and released by owner id.

export class VertiportRuntime {
  constructor(def, fatos = 1) {
    this.def = def;
    this.id = def.id; this.name = def.name;
    this.ix = def.ix; this.iy = def.iy; this.x = def.x; this.y = def.y; this.padElev = def.padElev;
    // `def.pads` are parking stands; flight operations run through shared FATOs (final approach & take-off areas),
    // because every take-off climb and landing descent uses the same airspace column above the vertiport.
    this.fatos = fatos; // default 1: one operation at a time in the shared airspace column (stands: def.pads). >1 is a stress-test relaxation
    this.padBookings = Array.from({ length: this.fatos }, () => []); // one booking list per FATO: [{s,e,id}]
    this.chargersTotal = def.chargers;
    this.charging = new Set(); // aircraft ids on chargers
    this.hangar = new Set(); // aircraft ids parked here
    this.waitingForCharger = [];
  }
  get closed() { return this.def.closed; }
  set closed(v) { this.def.closed = v; }
  get pads() { return this.padBookings.length; }

  _free(pad, s, e, ignore) {
    for (const b of this.padBookings[pad]) if (b.id !== ignore && b.s < e && b.e > s) return b.e;
    return -1;
  }

  /** Earliest slot >= t of length dur on any pad. Returns {pad, start} or null if closed. */
  findSlot(t, dur, ignoreId = null) {
    if (this.closed) return null;
    let best = null;
    for (let p = 0; p < this.padBookings.length; p++) {
      let s = t;
      for (let guard = 0; guard < 200; guard++) {
        const end = this._free(p, s, s + dur, ignoreId);
        if (end < 0) break;
        s = end;
      }
      if (!best || s < best.start) best = { pad: p, start: s };
    }
    return best;
  }

  book(id, pad, s, e, kind = 'any') { this.padBookings[pad].push({ s, e, id, kind }); }
  /** Release an owner's bookings; with `kind` ('takeoff' | 'land') only that kind is released. */
  release(id, kind = null) { for (const l of this.padBookings) for (let i = l.length - 1; i >= 0; i--) if (l[i].id === id && (!kind || l[i].kind === kind)) l.splice(i, 1); }
  gc(t) { for (const l of this.padBookings) for (let i = l.length - 1; i >= 0; i--) if (l[i].e < t - 10) l.splice(i, 1); }
  padsBusyAt(t) { let n = 0; for (const l of this.padBookings) if (l.some((b) => b.s <= t && b.e > t)) n++; return n; }

  // ---- charging
  get chargersFree() { return this.chargersTotal - this.charging.size; }
  tryStartCharging(id) {
    if (this.charging.has(id)) return true;
    if (this.chargersFree > 0) { this.charging.add(id); return true; }
    return false;
  }
  stopCharging(id) { this.charging.delete(id); }
}
