# Aravis decoder ownership benchmark

This microbenchmark measures the performance effect of publishing independently
owned decoder output directly instead of cloning a decoder-owned reusable image.
It uses the real `SwScaleDecoder` and `MonoUnpackedDecoder` implementations.

The two measured modes are:

- `legacy-clone`: decode into one reusable matrix, then call `clone()` before
  retaining the frame. This isolates the behavior of the previous acquisition
  path.
- `direct-owning`: pass an empty matrix to `decodeInto()` and retain that
  independently owned result without a pixel copy. This is the new acquisition
  path.

Both modes perform one output-buffer allocation request per frame: `legacy-clone`
allocates it in `cv::Mat::clone()`, while `direct-owning` allocates it in
`cv::Mat::create()` before decoding into it. The direct path therefore removes
one full-frame memcpy without adding another per-frame allocation request. The
decoder's reusable matrix in the legacy mode is allocated once outside the
measured steady-state loop.

Both modes retain a configurable ring of output matrices to approximate frames
that remain in downstream queues. By default the benchmark runs twice:

- `free=producer` replaces matrices from the benchmark thread, so their final
  release happens there.
- `free=consumer` passes matrices through a bounded single-producer,
  single-consumer queue, so allocation happens on the decoder thread and final
  release happens on the consumer thread.

The queue retains headers, not reusable pixel buffers. It therefore exercises
mimalloc's cross-thread free path without adding buffer pooling. The benchmark
deliberately excludes camera I/O, transformations, stream notification, and
recording so that it measures the decoder ownership change itself.

The preview path is intentionally not represented by `direct-owning`: it passes
the same persistent `cv::Mat` to `decodeInto()` on each call. OpenCV's
`Mat::create()` reuses that allocation while dimensions and type are unchanged,
so preview retains the previous single-buffer behavior.

## Reproducible release build

Configure a dedicated release build with the Aravis module enabled:

```sh
meson setup build-bench \
    --buildtype=release \
    -Db_lto=true \
    -Doptimize-native=true \
    -Dgui-tests=true \
    -Dmodules=camera-arv
meson compile -C build-bench benchmark-camera-arv-decode
```

Run the registered default benchmark:

```sh
meson test -C build-bench \
    --benchmark sy-benchmark-camera-arv-decode \
    --verbose
```

Run the executable directly to change its parameters:

```sh
./build-bench/tests/benchmarks/benchmark-camera-arv-decode \
    --format all \
    --width 1920 \
    --height 1080 \
    --iterations 500 \
    --repetitions 7 \
    --retained 32 \
    --free-thread both
```

Use `--help` for all options. Supported formats are `rgb8`, `mono8`, and
`mono12`. The reported time is the median of all repetitions, and measurement
order alternates between modes to reduce cache and thermal bias.

For comparable results:

1. Use the same release/native build configuration on both machines.
2. Close CPU- and memory-intensive applications.
3. Keep the machine connected to power and use a stable performance governor.
4. Optionally pin the direct executable to one CPU with `taskset`.
5. Record the CPU model, compiler version, complete benchmark output, and
   Syntalos commit hash.

Example:

```sh
git rev-parse HEAD
c++ --version | head -n 1
lscpu | grep 'Model name'
taskset -c 4 ./build-bench/tests/benchmarks/benchmark-camera-arv-decode \
    --format all --width 1920 --height 1080 \
    --iterations 500 --repetitions 7 --retained 32
```

`speedup` is `legacy-clone / direct-owning`; values above 1 favor direct
ownership. `avoided-copy` is the output image payload that no longer needs to
be copied for every acquired frame. Use `--free-thread consumer` to focus on
the allocation-on-acquisition/free-downstream case raised in review.
