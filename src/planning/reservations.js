// Space-time reservation table: for every grid node, a sorted list of busy time intervals owned by an id.
// Safe intervals (the complement of the busy intervals) are what SIPP searches over.

export class ReservationTable {
  constructor() {
    this.nodes = new Map(); // node -> [{s,e,id}] sorted by s
    this.byId = new Map(); // id -> [[node, entry]]
    this.count = 0;
  }

  add(id, node, s, e) {
    if (!(e > s)) return null;
    const entry = { s, e, id };
    let arr = this.nodes.get(node);
    if (!arr) { arr = []; this.nodes.set(node, arr); }
    let lo = 0, hi = arr.length;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (arr[mid].s <= s) lo = mid + 1; else hi = mid; }
    arr.splice(lo, 0, entry);
    let l = this.byId.get(id);
    if (!l) { l = []; this.byId.set(id, l); }
    l.push([node, entry]);
    this.count++;
    return entry;
  }

  removeId(id) {
    const l = this.byId.get(id);
    if (!l) return 0;
    for (const [node, entry] of l) {
      const arr = this.nodes.get(node);
      if (!arr) continue;
      const i = arr.indexOf(entry);
      if (i >= 0) arr.splice(i, 1);
      if (!arr.length) this.nodes.delete(node);
    }
    this.count -= l.length;
    this.byId.delete(id);
    return l.length;
  }

  /** Remove every id starting with a prefix (e.g. 'EXT:' tracks). */
  removePrefix(prefix) {
    for (const id of [...this.byId.keys()]) if (typeof id === 'string' && id.startsWith(prefix)) this.removeId(id);
  }

  entriesOf(id) { return (this.byId.get(id) || []).map(([node, e]) => ({ node, s: e.s, e: e.e })); }

  /** Safe (free) intervals of a node, ignoring entries of `ignoreId`, starting at tMin. */
  safeIntervals(node, ignoreId = null, tMin = 0) {
    const arr = this.nodes.get(node);
    if (!arr) return [[tMin, Infinity]];
    const out = [];
    let cur = tMin;
    for (let i = 0; i < arr.length; i++) {
      const en = arr[i];
      if (en.id === ignoreId || en.e <= cur) continue;
      if (en.s > cur) out.push([cur, en.s]);
      if (en.e > cur) cur = en.e;
    }
    out.push([cur, Infinity]);
    return out;
  }

  /** If [s,e] overlaps another id's entry, return the latest end among overlapping entries, else -1. */
  conflictEnd(node, s, e, ignoreId = null) {
    const arr = this.nodes.get(node);
    if (!arr) return -1;
    let end = -1;
    for (let i = 0; i < arr.length; i++) {
      const en = arr[i];
      if (en.s >= e) break;
      if (en.id !== ignoreId && en.e > s && en.s < e) end = Math.max(end, en.e);
    }
    return end;
  }

  isFree(node, s, e, ignoreId = null) { return this.conflictEnd(node, s, e, ignoreId) < 0; }

  /** Ids (other than `id`) whose reservations overlap any reservation of `id`. */
  conflictsOf(id, tFrom = -Infinity) {
    const out = new Set();
    for (const [node, entry] of this.byId.get(id) || []) {
      if (entry.e < tFrom) continue;
      const arr = this.nodes.get(node);
      for (const en of arr) {
        if (en.s >= entry.e) break;
        if (en.id !== id && en.e > entry.s && en.s < entry.e) out.add(en.id);
      }
    }
    return out;
  }

  /** Drop entries that ended before time t (bounded memory in long runs). */
  gc(t) {
    for (const [id, l] of this.byId) {
      const keep = [];
      for (const pair of l) {
        if (pair[1].e < t - 5) {
          const arr = this.nodes.get(pair[0]);
          const i = arr ? arr.indexOf(pair[1]) : -1;
          if (i >= 0) arr.splice(i, 1);
          if (arr && !arr.length) this.nodes.delete(pair[0]);
          this.count--;
        } else keep.push(pair);
      }
      if (keep.length) this.byId.set(id, keep); else this.byId.delete(id);
    }
  }
}
