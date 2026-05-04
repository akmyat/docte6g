# Three-way benchmark: progress & resume notes

**Goal:** wall-clock head-to-head of three ns-3 channel back-ends running an identical scenario:

1. **Pure ns-3** — analytical `FriisPropagationLossModel` (no ray tracing)
2. **ns3sionna** — Sionna RT via ZMQ to an external Python server (lives at [../../contrib/sionna/](../../contrib/sionna/))
3. **sionnart** — Sionna RT via *embedded* Python (pybind11) (lives at [../../contrib/sionnart/](../../contrib/sionnart/))

Plan file (full design rationale): `/home/aung/.claude/plans/yes-i-want-three-witty-castle.md`

## Aligned scenario (identical across all three binaries)

| Knob | Value |
|---|---|
| WiFi standard | 802.11ax, 20 MHz, channel 40 (5 GHz, fc≈5.2 GHz) |
| Scene (RT only) | `assets/scenes/free_space/free_space.xml` (sionnart) / `free_space/free_space.xml` resolved by Python server (ns3sionna). Both ship `free_space` already. |
| Pure-ns3 propagation | `FriisPropagationLossModel` (5.2 GHz) + `ConstantSpeedPropagationDelayModel`. No `Building`. |
| AP placement | Fixed at (1.0, 2.0, 1.0) |
| STA placement | Uniform random in 6 × 4 m floor area at z=1.0 |
| STA count sweep | N ∈ {1, 2, 4, 8, 16, 32} — driven by shell, **one N per process** |
| Mobility | `RandomWalk2dMobilityModel` (pure-ns3) / `SionnaMobilityModel` with `MODEL_RANDOM_WALK` (ns3sionna) / `SionnaMobilityModel` with `RANDOM_WALK` (sionnart) |
| Traffic | UDP echo broadcast from AP, 100 B packets, 10 s sim |
| Timing | `std::chrono::steady_clock` around `Simulator::Run()` only |
| Caching | ns3sionna: `SetCaching(true)`. sionnart: cache always on (no toggle in API). |
| Antenna | SISO (1×1 / 1×1) |

> **Module renamed.** The upstream `ns3sionna` repo was cloned into `contrib/ns3sionna/` (not `contrib/sionna/`), and all its public headers were renamed `sionna-*.h` → `ns3sionna-*.h` (and `cfr-tag.h` → `ns3sionna-cfr-tag.h`) to avoid collisions with `contrib/sionnart/` which exports the same filenames. `LIBNAME` is `ns3sionna`, so the benchmark links `${libns3sionna}`. C++ class names (`ns3::SionnaPropagationCache`, etc.) are unchanged because each binary links exactly one of the two libs.

### Three workload regimes
1. **stationary_high_load**: `mobile_scenario=false`, `udp_pkt_interval=20`
2. **low_mob_low_load**: `mobile_speed=1.0`, `udp_pkt_interval=1000`
3. **high_mob_high_load**: `mobile_speed=7.0`, `udp_pkt_interval=20`

### Output contract
Each binary prints exactly one machine-readable line at end:
```
RESULT backend=<pure_ns3|ns3sionna|sionnart> num_stas=<N> regime=<...> wall_clock_s=<f>
```

### Common CLI flags (all three binaries)
```
--num_stas=<int>
--mobile_scenario=<bool>
--mobile_speed=<double>
--udp_pkt_interval=<int>     # ms
--sim_seconds=<double>
--regime=<string>            # passthrough label for output
--caching=<bool>             # ignored by pure_ns3, ignored by sionnart
--verbose=<bool>
```
ns3sionna additionally accepts `--zmqUrl` and `--environment`. sionnart additionally accepts `--assetsRoot`.

## Conda env mapping (verified 2026-05-02)

- **6Gold** → ns3sionna server. Python 3.10.20, **sionna 1.2.2 + sionna.rt 1.2.2** (1.x API). Plus mitsuba 3.8, numpy 2.2.6, tensorflow 2.21, zmq 27.1, google.protobuf 7.34, GPUtil, millify. Matches `contrib/sionna/model/ns3sionna/requirements.txt`. Start FIRST in a separate terminal:
  ```
  conda activate 6Gold
  cd ns3_server/contrib/sionna/model/ns3sionna && ./run_python_proto.sh
  ```
- **6G** → sionnart (embedded Python). Python 3.12.13, **sionna.rt 2.0.1** (2.x API; `PathSolver` is 2.x-only and sionnart imports it). Plus mitsuba 3.8, numpy 2.4.3, tensorflow 2.21.
- pure_ns3 → no Python needed.

The two envs are **mutually incompatible** by design: sionna 1.x (ns3sionna) and 2.x (sionnart) cannot coexist in one env.

**Runtime caveat:** `conda activate` from a non-interactive shell (e.g. inside `run_three_way.sh`) may not take effect. Start the ns3sionna server from an already-activated 6Gold shell. The sionnart binary uses *embedded* Python via pybind11 — it locks to whichever Python was linked at **build time**, not the shell env at run time. So always `./ns3 build` from inside 6G to lock sionnart's interpreter to 6G.

(See memory file `~/.claude/projects/-home-aung-code-new-docte6g/memory/conda_env_mapping.md`.)

## Build / link constraints

- Both `contrib/sionna` and `contrib/sionnart` declare `ns3::SionnaPropagationCache`, `ns3::SionnaPropagationLossModel`, etc. in the same namespace.
- Each binary must link **exactly one** of `${libsionna}` / `${libsionnart}` to avoid symbol collisions.
- `pure_ns3_benchmark` links neither.
- `sionnart_benchmark` links `${libsionnart}`.
- `ns3sionna_benchmark` links `${libsionna}` (was `${libsionnart}` in original CMakeLists — fixed).

## Files in this directory

| File | Status | Purpose |
|---|---|---|
| [pure_ns3_main.cc](pure_ns3_main.cc) | ✅ written | Pure ns-3 baseline (Friis + 802.11ax) |
| [ns3sionna_main.cc](ns3sionna_main.cc) | ✅ written | ns3sionna binary; mirrors upstream `performance-sionna.cc` with single-N CLI |
| [sionnart_main.cc](sionnart_main.cc) | ✅ written | sionnart binary; mirrors ns3sionna structure but uses `SionnaPyEmbed` + `SionnaInitSettings` |
| [CMakeLists.txt](CMakeLists.txt) | ✅ updated | Three targets, correct lib linkage |
| [server.sh](server.sh) | ✅ written | ns3sionna ZMQ server lifecycle: `start \| stop \| status \| restart \| log`. Activates 6Gold internally. |
| [bench.sh](bench.sh) | ✅ written | Single-shot wrapper: `./bench.sh {pure_ns3\|sionnart\|ns3sionna\|all} [args]`. Auto-starts server for ns3sionna. |
| [run_three_way.sh](run_three_way.sh) | ✅ written | Full sweep driver: N × regime × backend → `results.csv`. Auto-starts server unless `--no-auto-server`. |
| [plot_results.py](plot_results.py) | ✅ written | One figure per regime, wall-clock vs N, three lines |
| `results.csv` | — | Output of `run_three_way.sh` (created on first run) |

## Implementation references / cribs

- Upstream baseline patterns: [../../contrib/sionna/examples/performance-ns3.cc](../../contrib/sionna/examples/performance-ns3.cc), [../../contrib/sionna/examples/performance-sionna.cc](../../contrib/sionna/examples/performance-sionna.cc)
- Existing sionnart benchmark (propagation-only, simpler): [/home/aung/code/new_docte6g/benchmark-sionnart/benchmark-sionnart.cc](/home/aung/code/new_docte6g/benchmark-sionnart/benchmark-sionnart.cc) — shows the `SionnaPyEmbed::GetInstance().SionnaInitialize(settings)` pattern, `cache->SetAttribute("EnableWeakLinkFastPath", ...)`, and `std::_Exit(0)` finalizer trick.
- sionnart API: `contrib/sionnart/model/sionna-py-embed.h` (lines 35–68 = `SionnaInitSettings`), `sionna-mobility-model.h` (attrs: "Mode" / "Speed" / "ObjectName" / "Bounds"), `sionna-propagation-cache.h` (no `SetCaching` / `SetSionnaHelper` toggles — always cached).
- ns3sionna API: `contrib/sionna/helper/sionna-helper.h` (`Configure`, `Start`, `Destroy`, `SetMode`, `SetSubMode`), `sionna-propagation-cache.h` (has `SetCaching(bool)`, `SetSionnaHelper(...)`).

## Build verified ✅ (2026-05-02)

All three binaries build, link to the correct Python, and produce a `RESULT` line on N=1, sim_seconds=2:

| Binary | Wall-clock (smoke) | Notes |
|---|---|---|
| `pure_ns3_benchmark` | 0.005 s | analytic only, no Python |
| `sionnart_benchmark` | 0.106 s | linked to `libpython3.12.so.1.0` from 6G; cache hit ratio 0.995 |
| `ns3sionna_benchmark` | 0.504 s | ZMQ server (6Gold) on `tcp://localhost:5555`; cache hit ratio 0.998 |

### Build issues we hit and fixed (for future agents)

The freshly-cloned upstream ns3sionna had several incompatibilities with the user's existing build:

1. **Module dirname must match `NS3_ENABLED_MODULES`.** We renamed `contrib/sionna` → `contrib/ns3sionna` so cmake's contrib filter would actually pick it up.
2. **Header filename collisions with sionnart.** Both modules export classes named `ns3::Sionna*` from headers like `sionna-mobility-model.h`. We renamed the `contrib/ns3sionna` files (and `cfr-tag.{h,cc}`) to `ns3sionna-*` and `ns3sionna-cfr-tag` so each lib's headers don't clobber the other's in `build/include/ns3/`.
3. **Internal `#include` paths.** Files used `#include "../model/sionna-..."` style relative paths; we sed'd these to the new `ns3sionna-` prefix everywhere.
4. **`LIBNAME sionna` → `LIBNAME ns3sionna`** in `contrib/ns3sionna/CMakeLists.txt`, plus `${libsionna-obj}` → `${libns3sionna-obj}`.
5. **`CMAKE_CXX_STANDARD 17` override removed.** ns-3.46's `NS_FATAL_ERROR` macro uses `std::stacktrace`, which is C++23-only.
6. **Pre-generated protobuf sources.** `protobuf_generate(TARGET ...)` couldn't find `.proto` files via the SOURCES property; we regenerated `message.pb.{h,cc}` and `message_pb2.py` with `/usr/bin/protoc` (system 3.21, not anaconda's 29.x — the latter emits `runtime_version.h` includes that aren't available against system protobuf headers) and committed them to the source tree. The two `protobuf_generate(...)` calls were removed from CMakeLists.
7. **`MobilityModel::Copy()` is pure virtual** in ns-3.46. The upstream `SionnaMobilityModel` didn't override it. Added a manual member-by-member copy.
8. **`SpectrumPropagationLossModel::DoAssignStreams` is pure virtual.** Added a `return 0` override on `SionnaSpectrumPropagationLossModel`.
9. **`SionnaPhasedArraySpectrumPropagationLossModel` excluded from build.** Upstream marks it "TODO: not yet implemented" and its `DoCalcRxPowerSpectralDensity` signature doesn't match ns-3.46's base. SISO benchmark doesn't need it; we removed it from `SOURCE_FILES`/`HEADER_FILES`.
10. **Python linkage forced to 6G.** Initial configure picked anaconda base's Python 3.13 (`libpython3.13.so` linked into `sionnart_benchmark`), causing `ImportError: numpy._core._multiarray_umath` at runtime. Fix: `rm -rf cmake-cache && ./ns3 configure --enable-examples -- -DPython3_EXECUTABLE=/home/aung/anaconda3/envs/6G/bin/python3`. After this, `ldd sionnart_benchmark | grep python` shows `libpython3.12.so.1.0 from 6G/lib/`.

### How to rebuild from scratch

```bash
conda activate 6G
cd /home/aung/code/new_docte6g/ns3_server
rm -rf cmake-cache build
./ns3 configure --enable-examples -- -DPython3_EXECUTABLE=/home/aung/anaconda3/envs/6G/bin/python3
./ns3 build pure_ns3_benchmark sionnart_benchmark ns3sionna_benchmark
# Verify Python linkage:
ldd build/scratch/benchmark_sionnart_ns3sionna/ns3.46-sionnart_benchmark-default | grep python
# Expect: libpython3.12.so.1.0 => /home/aung/anaconda3/envs/6G/lib/...
```

### How to run (use the helper scripts)

All commands run from `ns3_server/scratch/benchmark_sionnart_ns3sionna/`. The helpers handle conda env activation and ZMQ server lifecycle automatically.

**Single backend, single shot:**
```bash
./bench.sh pure_ns3                                  # defaults: N=1, sim=2s, regime=smoke
./bench.sh sionnart  --num_stas=4 --sim_seconds=5
./bench.sh ns3sionna --num_stas=2 --sim_seconds=5    # auto-starts ZMQ server in 6Gold if needed
./bench.sh all       --num_stas=2 --sim_seconds=3    # all three sequentially with same args
```
Defaults applied when caller omits them: `--num_stas=1 --sim_seconds=2 --regime=smoke`. Any other `./ns3 run "..."` arg pass-through works (`--mobile_scenario`, `--mobile_speed`, `--udp_pkt_interval`, etc.).

**ns3sionna ZMQ server lifecycle (manual):**
```bash
./server.sh status      # check if running on tcp:5555
./server.sh start       # start in background (logs to /tmp/ns3sionna_server.log)
./server.sh log         # tail -f the log
./server.sh stop        # SIGTERM whatever owns tcp:5555
./server.sh restart
```
The server runs Python from conda env **6Gold** (`/home/aung/anaconda3/envs/6Gold/bin/python`) regardless of the shell's active env. You don't need to activate 6Gold yourself.

**Full sweep (all three, all regimes, N=1..32):**
```bash
./run_three_way.sh                                   # auto-starts server, writes results.csv
./run_three_way.sh --backends pure_ns3,sionnart      # skip ns3sionna (no server needed)
./run_three_way.sh --max-stas 16 --sim-seconds 5
./run_three_way.sh --no-auto-server                  # don't auto-start (you start manually)
python plot_results.py                               # one PNG per regime
```

## Resume from here

Three binaries now verified. Remaining steps:

1. **Build:** `cd ns3_server && ./ns3 configure --enable-examples && ./ns3 build`. Three executables expected under `build/scratch/benchmark_sionnart_ns3sionna/`. Expect first compile errors here — fix in the `_main.cc` files.

2. **Smoke runs (verify wiring before full sweep):**
   - `./ns3 run "pure_ns3_benchmark --num_stas=1 --sim_seconds=2"` → finishes <1 s, prints RESULT line.
   - In one terminal: start ns3sionna server (6Gold). In another: `./ns3 run "ns3sionna_benchmark --num_stas=1 --sim_seconds=2"` → cache stats printed, RESULT line printed.
   - `conda activate 6G && ./ns3 run "sionnart_benchmark --num_stas=1 --sim_seconds=2"` → embedded Python loads `assets/scenes/free_space/free_space.xml`, RESULT line printed.

3. **Full sweep + plot:**
   ```
   # Terminal 1 (only if including ns3sionna):
   conda activate 6Gold
   cd ns3_server/contrib/sionna/model/ns3sionna && ./run_python_proto.sh

   # Terminal 2:
   conda activate 6G
   cd ns3_server/scratch/benchmark_sionnart_ns3sionna
   ./run_three_way.sh
   python plot_results.py
   ```
   Or run a subset, e.g. `./run_three_way.sh --backends pure_ns3,sionnart --max-stas 16`.

## Known gotchas / decisions made

- **Pure-ns3 dropped the `Building`.** Upstream `performance-ns3.cc` uses `HybridBuildingsPropagationLossModel` + `Building(0,6,0,4,0,2.5, StoneBlocks)`. Since the user picked `free_space` for both RT back-ends (no walls), keeping a building only on the pure-ns3 side would be a physics mismatch. Replaced with `FriisPropagationLossModel` so all three back-ends model free-space-equivalent geometry.
- **Single-N per process.** Cleaner per-N wall-clock attribution. The upstream `performance-sionna.cc` doubles N inside the binary to keep the Python interpreter / ZMQ session warm — that's faster end-to-end but conflates startup amortization. The shell driver pays the warm-up cost per run; if this matters later, swap to internal-loop variants (the `RunSimulation` function is already factored).
- **sionnart caching has no on/off toggle.** The `--caching` CLI flag is accepted but ignored on the sionnart side. `cache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false))` is the closest analog if needed; not used here.
- **sionnart `RANDOM_WALK` mode uses `Bounds` Box attribute** (not `Wall`/`Time`/`Direction` like ns3sionna). Set to `Box(0,6, 0,4, 0,3)` matching the floor area. ns3sionna's mobility model has its own enum with `MODEL_RANDOM_WALK` plus separate `Wall` boolean.
- **ObjectName must be set before `SionnaPyEmbed::SionnaInitialize`** so `tx_names` / `rx_names` in `SionnaInitSettings` match the names the mobility models will report to the backend. AP is named `"AP"`, STAs are `"STA_0"`, `"STA_1"`, …
- **`std::_Exit(0)` at end of `sionnart_main`** to skip pybind11's Python finalizer (avoids shutdown noise / segfault pattern seen in `benchmark-sionnart.cc`).
- **ns3sionna's `min_coherence_time_ms = 1000`** is kept (matches upstream `performance-sionna.cc`). It throttles channel re-computations; relevant in the high-mobility regime.
- **Mode/sub_mode for ns3sionna** kept at upstream defaults (`SetMode(2)`, `SetSubMode(16)`). These don't have sionnart analogs.
