// Pure pose sequence logic, shared by the UI and Node tests.
export function prepareFrames(frames) {
  let segment = -1, previous = null, origin = null, travelled = 0;
  return frames.map((raw) => {
    const f = {...raw};
    f.valid = f.valid && f.p?.length === 3 && f.q?.length === 4 &&
      [...f.p, ...f.q, f.t].every(Number.isFinite);
    if (!f.valid) { previous = null; return {...f, segment: -1}; }
    const step = previous ? Math.hypot(...f.p.map((v, i) => v - previous.p[i])) : 0;
    // Conservative visual discontinuity guard, not a physical-motion classifier.
    const cut = !previous || f.map_id !== previous.map_id || f.source_segment !== previous.source_segment ||
      f.t <= previous.t || f.t - previous.t > 1 || step > 2;
    if (cut) { segment++; origin = f; travelled = 0; }
    else travelled += step;
    f.segment = segment;
    f.distance = travelled;
    f.displacement = Math.hypot(...f.p.map((v, i) => v - origin.p[i]));
    const dot = Math.min(1, Math.abs(f.q.reduce((v, q, i) => v + q * origin.q[i], 0)));
    f.angle = 2 * Math.acos(dot) * 180 / Math.PI;
    previous = f;
    return f;
  });
}

export function timeLabel(seconds) {
  seconds = Math.max(0, seconds || 0);
  return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${(seconds % 60).toFixed(2).padStart(5, '0')}`;
}
