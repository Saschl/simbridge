# Benches

See `../OPTIMIZATIONS.md` for what the benchmarked optimizations are and why
they are shaped the way they are.

Self-contained benchmarks (`harness = false`, no dev-dependencies). The bench
profile inherits `release` (including `lto = true`), so the timed code is the
shipped codegen.

```sh
cargo bench --no-default-features                       # everything
cargo bench --no-default-features --bench nd_render_ab  # just the A/B
cargo bench --no-default-features -- --quick            # fast smoke run
```

`--no-default-features` skips SimConnect, which needs the MSFS SDK at build
time — same as `cargo build`/`cargo test` on non-Windows machines.

## nd_render_ab — A/B for the render optimisation

Compares the original per-pixel port of the TS/GPU kernel (kept verbatim as a
`baseline` module inside the bench) against the shipped
`render_navigation_display` (per-block band hoisting + colour tables +
vectorised block maxima).

Before timing anything it runs a **bit-exact gate**: 192 frame pairs across
4 geometries x 2 patterns x 6 elevation maps (including degenerate ones:
all-invalid, all-water, all-unknown, flat, single-spike) x 4 flight states
covering both display modes. Any differing byte aborts the bench with a
non-zero exit code — the timings are only meaningful while A == B, and the
gate doubles as a regression tripwire: it fails if either the library or the
baseline copy (which duplicates the private threshold helpers) drifts.

## extract_ab — A/B for the extraction rewrite

Compares the straight per-pixel port of `createLocalElevationMap` (verbatim
`baseline` module, calling the public geodesy helpers per pixel) against the
shipped warp-grid `extract_local_elevation_map`: the display->world mapping
is evaluated exactly only at 8x8 tile corners and bilinearly interpolated in
between, with a centre probe (catches the antimeridian wrap) and a polar
guard (above 80° projected latitude everything is exact) falling back to the
exact closed-form path. Single-threaded — the production target is a sim
module without threads.

The warp grid is an approximation by design, so the **gate** enforces
budgets instead of strict equality, over 240 pairs (5 world regions x 4
geometries x 3 ND ranges x 4 headings): polar regions identical, per-case
divergence <= 0.5% of samples, global <= 0.1%, divergent-sample elevation
delta <= 100 ft, and — on the rendered ND downstream — threshold metadata
identical with frame divergence <= 0.2% of bytes. Any exceeded budget aborts
with a non-zero exit code. Measured at ship time: 0.02% global divergence,
10 ft max delta, thresholds identical everywhere, worst frame 0.05%.

## nd_cycle — where a cycle's time goes

Stage profile of one ND cycle (extraction → histogram → render) on a
synthetic 4000x3200 world map. Not an A/B; it exists so optimisation effort
stays pointed at the actual bottleneck (`extract_local_elevation_map`).

## Methodology

- All inputs are deterministic (fixed xorshift64 seeds, synthetic relief) —
  runs are comparable across machines and sessions.
- Each measurement is the **median** of N timing samples of K calls each,
  after warmup; the minimum is shown in parentheses as the uncontended cost.
  Frame allocation is included, as in the real caller.
- For stable numbers: run on AC power with background load quiesced, and
  compare medians across at least two runs.
