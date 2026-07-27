# Terrain renderer optimizations

Summary of the performance work on the ND terrain pipeline (July 2026).
Goal: the module must run **inside the sim on a single core**, so every
optimization here is single-threaded. All numbers below are from the in-repo
benches (`cargo bench --no-default-features`) on an arm64 dev machine; the
relative gains are what matter, absolute times will differ per machine.

## Result at a glance

One render cycle (extract the elevation map → histogram → render the ND
frame), A380X arc mode, 756x592:

| stage | before | after | change |
|---|---|---|---|
| `extract_local_elevation_map` | ~19.5 ms | ~1.7 ms | ~11x |
| `elevation_histogram` | ~0.6 ms | ~0.6 ms | untouched |
| `render_navigation_display` | ~0.9 ms | ~0.2 ms | ~5x |
| **cycle total** | **~21 ms** | **~2.5 ms** | **~8x** |

## 1. `render_navigation_display` — same output, restructured work

The original port decided the colour of every pixel independently: per pixel
it re-derived which elevation band (red / yellow / green / water / …) the
pixel's 8x8 block falls into, then applied the density-pattern test. But the
band only depends on the block's maximum elevation — it cannot change within
a block — so the f64 threshold comparisons ran 64x more often than they can
change.

Three changes, none of which alter a single output byte:

- **Block-band hoisting** — classify each 8x8 block into its band once
  (7k decisions instead of 447k).
- **Colour lookup tables** — the density test (`pattern % prime == 0`) only
  depends on the pattern byte, so each band gets a 256-entry table mapping
  pattern byte → RGBA. The per-pixel work collapses to one table lookup.
- **Vectorised block maxima** — the 8x8 maximum is restructured as a
  row-wise elementwise `i16` max, which LLVM auto-vectorises to NEON
  (`smax.8h`, 8 lanes per instruction). The `ELEV_INVALID` sentinel folds
  into the `-1000` seed value, which is exact: the seed is also the floor,
  so a value mapped onto it could never have won the maximum anyway.

**Accuracy: bit-exact.** `benches/nd_render_ab.rs` keeps a verbatim copy of
the original implementation and proves old == new byte-for-byte over 192
frame pairs before timing anything.

## 2. `extract_local_elevation_map` — the warp grid

### What the function does

For every display pixel it answers: *which cell of the world elevation map
is under this pixel?* The original (a port of the GPU kernel) computed that
from scratch per pixel: screen offset → distance and bearing → great-circle
projection on the spherical earth (`sin`, `cos`, `asin`, `atan2`, `acos`) →
world-map pixel. That is ~8 transcendental calls x 447k pixels, and it was
~95% of the whole cycle.

### Step 1: closed-form bearing (kept as the "exact path")

The original built the bearing through `acos → degrees → normalize →
radians → sin/cos`. Trigonometric identities collapse all of that into two
multiply-adds against the (per-frame constant) sin/cos of the aircraft
heading:

```
cos(bearing) = (dy·cos_h − dx·sin_h) / r
sin(bearing) = (dx·cos_h + dy·sin_h) / r
```

One formula covers both sign branches of the original. This alone was ~1.8x
and produced identical output on every tested input.

### Step 2: the warp grid (the big one)

"Warp" in the image-processing sense: the function *is* an image warp — a
smooth geometric mapping from display coordinates to world-map coordinates,
exactly like the perspective/rotation warps in graphics pipelines. Over any
small patch of the display the mapping is almost perfectly linear (earth
curvature bends it by ~1 part in 10⁴ across an 8-pixel tile).

So instead of evaluating the expensive spherical math for every pixel:

1. Split the display into **8x8 tiles**.
2. Evaluate the exact mapping only at the **tile corners** (~1/13th of the
   original calls).
3. **Bilinearly interpolate** the world coordinates for the pixels in
   between — two multiply-adds per pixel instead of five transcendentals.
   (Bilinear interpolation = linear blend along x, then along y, of the four
   corner values.)

Two guards make the approximation defensive, each falling back to the exact
closed-form path for the affected tile:

- **Centre probe.** For every tile, the exact mapping is also evaluated at
  the tile centre and compared to the interpolated value. Disagreement
  beyond 0.05 world pixels → the whole tile is computed exactly. This
  catches the antimeridian (the ±180° longitude wrap is a discontinuity —
  interpolating across it would be nonsense).
- **Polar guard.** Above 80° projected latitude, interpolation is disabled
  outright. The world map is an equirectangular grid; near the poles its
  longitude axis degenerates (cos lat → 0) and no small threshold can be
  trusted. Polar frames are therefore exact — correct, just not faster.

### Accuracy policy: budgets instead of bit-exactness

The warp grid is a real approximation: occasionally an interpolated
coordinate rounds to the world cell *adjacent* to the exact one. Agreed
2026-07-26: this is not a scientific application — close enough is fine.
For scale, the TS original ran in **f32 on the GPU**, so its own arithmetic
was far noisier than anything here.

Measured over 240 cases (5 world regions including both poles and the
antimeridian x 4 display geometries x 3 ND ranges x 4 headings):

- 0.02% of samples land one world cell off, almost only at the 160 nm range
- max elevation difference on such a sample: **10 ft**
- rendered ND frames: threshold metadata (the min/max elevation numbers
  shown to the pilot) identical in every case; worst frame differs in 0.05%
  of bytes
- polar regions: zero differences (the guard forces exact evaluation)

`benches/extract_ab.rs` turns those measurements (plus headroom) into hard
**budgets** — polar identical, ≤0.5% per case, ≤0.1% global, ≤100 ft
elevation delta, ND thresholds identical, ≤0.2% frame bytes — and exits
non-zero if any budget is exceeded, so a regression in the approximation
fails CI just as loudly as a bit-flip would have.

### Tried and rejected

- **Loop-invariant hoisting alone**: 1.01x — LLVM already does it.
- **`sin(asin(S)) → S`**: passed all tests, zero measurable gain.
- **Mirror-pair sharing** (pixels at ±dx share sqrt/acos exactly): 1.19x,
  superseded by the warp grid.
- **Row multithreading**: 4.3x on 4 performance cores, removed — the sim
  target is single-threaded, and at 1.7 ms there is nothing left for
  threads to buy. (If a desktop build ever wants it back: the knee is at
  the P-core count; 6 threads measured *worse* than 4 on a 4P+6E part
  because static chunks land on efficiency cores.)

## Benchmarks

```sh
cargo bench --no-default-features                # all gates + timings
cargo bench --no-default-features -- --quick     # fast smoke run
```

- `nd_render_ab` — render A/B, strict bit-exact gate (192 pairs).
- `extract_ab` — extraction A/B, budgeted gate (240 pairs + 216 downstream
  ND frames).
- `nd_cycle` — where cycle time goes; run before optimising anything else.

Each A/B bench keeps a **verbatim copy of the pre-optimization
implementation** inside the bench file as the baseline, so the gates fail if
either the library or the baseline copy drifts. See `benches/README.md` for
methodology.

## Tuning knobs (`src/elevation_map.rs`)

| constant | value | effect |
|---|---|---|
| `WARP_TILE` | 8 | 16 is ~2x faster but ~4x the divergence |
| `PROBE_TOLERANCE_PX` | 0.05 | centre-probe fallback threshold |
| `POLAR_EXACT_LIMIT_DEG` | 80.0 | latitude beyond which everything is exact |
