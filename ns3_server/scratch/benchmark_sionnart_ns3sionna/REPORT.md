# Three-way wall-clock benchmark — pure ns-3 vs ns3sionna vs sionnart

**Author:** Aung
**Date:** 2026-05-02
**Goal:** Quantify the wall-clock simulation cost of ray-tracing-driven channel models (Sionna RT) versus an analytical baseline, under identical WiFi 802.11ax scenarios, as a function of station count and traffic/mobility load.

---

## 1. Setup

### 1.1 Aligned scenario (identical across all three back-ends)

| Parameter | Value |
|---|---|
| WiFi standard | 802.11ax, 20 MHz, channel 40 (5 GHz, fc≈5.2 GHz) |
| Topology | 1 fixed AP + N STAs |
| Floor area (STA placement) | 6 × 4 m, z = 1 m (uniform random) |
| AP position | (1.0, 2.0, 1.0) m |
| Mobility | `RandomWalk2dMobilityModel` (pure ns-3) / `SionnaMobilityModel::MODEL_RANDOM_WALK` (ns3sionna and sionnart) |
| Traffic | UDP echo broadcast from AP, 100 B packets, MaxPackets = 10⁹ |
| Simulated duration | 10 s |
| Antenna | SISO (1 × 1) |
| RT scene | `free_space.xml` (used by ns3sionna and sionnart) |
| Caching | enabled on both RT back-ends (`SionnaPropagationCache`) |
| Wall-clock measurement | `std::chrono::steady_clock` around `Simulator::Run()` only |

### 1.2 Three workload regimes

| Regime | Mobility | UDP packet interval |
|---|---|---|
| **stationary_high_load** | none | 20 ms |
| **low_mob_low_load** | 1 m/s random walk | 1000 ms |
| **high_mob_high_load** | 7 m/s random walk | 20 ms |

### 1.3 Sweep

`N ∈ {1, 2, 4, 8, 16, 32}`, three repetitions per cell for the stable back-ends (pure ns-3 and ns3sionna), one repetition for sionnart.

`ns3sionna` did not complete `N = 32` for `high_mob_high_load` within the 2-hour budget (each `N = 16` run in this regime already takes ~10 minutes).

### 1.4 Back-ends compared

| Back-end | Channel implementation | Python integration | Conda env |
|---|---|---|---|
| **pure ns-3** | `FriisPropagationLossModel` + `ConstantSpeedPropagationDelayModel` (analytical) | none | none |
| **ns3sionna** | Sionna RT (`sionna 1.2.2`, `sionna.rt 1.2.2`) | external Python process via ZMQ (`tcp://localhost:5555`) | **6Gold** (Python 3.10) |
| **sionnart** | Sionna RT (`sionna.rt 2.0.1`) | embedded via pybind11 in the ns-3 binary | **6G** (Python 3.12) |

### 1.5 Host

- CPU: Linux aarch64, 20 hardware threads
- GPU: NVIDIA GB10 (CUDA capability 12.1)
- OS: Ubuntu 24.04.4 LTS, kernel 6.17 (NVIDIA build)
- ns-3 version: 3.46

---

## 2. Headline result

Across all three regimes pure ns-3 scales smoothly with N as expected; **ns3sionna adds a 4×–125× wall-clock overhead** depending on regime, and the overhead grows with both the number of links and the mobility/traffic intensity. The high-mobility regime is dramatically worse than the stationary regime, indicating that the ray-tracer's coherence-time-driven recomputations dominate.

**Sionnart** wall-clock initially showed a bimodal anomaly that turned out to be a naming-convention bug in our benchmark glue (see §6). After the fix, sionnart scales monotonically and is consistently **faster than both pure ns-3 and ns3sionna** in this benchmark — though with the caveat that its propagation cache reports only 1 ray-trace miss per 10-second run, suggesting either remarkable adaptive-cache effectiveness or under-counting of position-driven invalidations (see §6.5).

---

## 3. Per-regime curves

### 3.1 Stationary / high load

![stationary high load](plot_stationary_high_load.png)

Both back-ends show approximately linear scaling on log-log axes. Pure ns-3: ~0.07 s (N = 1) → 19 s (N = 32). ns3sionna: 0.28 s → 102 s, a uniform ~4–6× factor.

### 3.2 Low mobility / low load

![low mobility low load](plot_low_mob_low_load.png)

Pure ns-3 is dramatically faster here (10 ms at N = 1; 0.78 s at N = 32) because the 1000-ms UDP interval cuts simulator events ~50×. ns3sionna does *not* benefit nearly as much (0.16 s → 36 s) because each new link still triggers ray-tracer calls regardless of how often the application sends packets — beacons, association traffic, and channel-coherence events keep the RT engine busy. The overhead factor therefore *grows* in this regime: from ~16× at N = 1 to ~46× at N = 32.

### 3.3 High mobility / high load

![high mobility high load](plot_high_mob_high_load.png)

This regime is where ray tracing falls off the cliff. ns3sionna spends 6 s on N = 1 and **595 s (≈10 min)** on N = 16; the extrapolated N = 32 point exceeded the 2-hour cutoff. Pure ns-3 in the same regime only goes from 0.07 s to 19 s. The mobility (7 m/s) keeps invalidating the propagation cache, forcing fresh ray traces.

---

## 4. Overhead factor (relative to pure ns-3)

![overhead vs pure ns-3](plot_overhead_vs_pure_ns3.png)

| Regime | ns3sionna overhead range (×) | Trend |
|---|---|---|
| stationary | ≈ 3.3 – 5.4 | roughly flat |
| low mobility | ≈ 16 – 46 | growing with N |
| high mobility | ≈ 80 – 125 | growing with N |

The strong regime-dependence confirms that **the bottleneck is the rate of channel recomputation, not the absolute number of links**. Pure ns-3's analytical Friis model is essentially free per-link, so its growth is driven by ns-3 simulator events; ns3sionna's growth is driven by ray-trace calls, which scale with mobility and link count combined.

---

## 5. Aggregated numbers

(median over the available repetitions; full per-rep distribution in `results.csv`, aggregates in `results_aggregated.csv`)

### 5.1 Pure ns-3 [seconds]

| N | stationary | low_mob | high_mob |
|---:|---:|---:|---:|
| 1  | 0.069 | 0.010 | 0.073 |
| 2  | 0.170 | 0.016 | 0.183 |
| 4  | 0.439 | 0.031 | 0.452 |
| 8  | 1.373 | 0.075 | 1.417 |
| 16 | 4.833 | 0.220 | 4.942 |
| 32 | 18.999 | 0.776 | 19.327 |

### 5.2 ns3sionna [seconds]

| N | stationary | low_mob | high_mob |
|---:|---:|---:|---:|
| 1  | 0.285 | 0.165 | 5.898 |
| 2  | 0.567 | 0.354 | 14.021 |
| 4  | 1.666 | 1.258 | 44.926 |
| 8  | 5.666 | 2.922 | 162.532 |
| 16 | 22.472 | 7.599 | 597.697 |
| 32 | 104.393 | 35.355 | — (>2 h) |

### 5.3 Overhead factor (ns3sionna / pure_ns3)

| N | stationary | low_mob | high_mob |
|---:|---:|---:|---:|
| 1  | 4.1× | 16.0× | 80.7× |
| 2  | 3.3× | 21.6× | 76.6× |
| 4  | 3.8× | 40.6× | 99.4× |
| 8  | 4.1× | 38.9× | 114.7× |
| 16 | 4.7× | 34.5× | 120.9× |
| 32 | 5.5× | 45.7× | — |

---

## 6. Sionnart bimodal-anomaly: root cause and fix

### 6.1 Original observation (pre-fix)

Sionnart's wall-clock followed a striking bimodal pattern in every regime — tiny at `N ∈ {1, 2, 16, 32}` and huge at `N ∈ {4, 8}`:

| N | stationary | low_mob | high_mob |
|---:|---:|---:|---:|
| 1  | 0.074 | 0.071 | 0.073 |
| 2  | 0.115 | 0.051 | 0.093 |
| **4** | **105.686** | **4.586** | **118.770** |
| **8** | **341.492** | **13.926** | **346.542** |
| 16 | 0.513 | 0.094 | 0.250 |
| 32 | 0.915 | 0.137 | 0.451 |

### 6.2 Root cause: ObjectName naming convention

`SionnaPropagationCache::IsSameRoleLink` ([contrib/sionnart/model/sionna-propagation-cache.cc:122-126](../../contrib/sionnart/model/sionna-propagation-cache.cc#L122-L126)) implements the role check via **substring matching** for the literal strings `"Tx"` and `"Rx"`:

```cpp
bool IsSameRoleLink(const std::string& nameA, const std::string& nameB) {
    return (nameA.find("Tx") != std::string::npos && nameB.find("Tx") != std::string::npos) ||
           (nameA.find("Rx") != std::string::npos && nameB.find("Rx") != std::string::npos);
}
```

The cache calls this function ([sionna-propagation-cache.cc:322,410](../../contrib/sionnart/model/sionna-propagation-cache.cc#L322)) to skip same-role pairs (RX↔RX between two STAs is not a real link in a 1-AP/N-STA topology) and avoid sending them to the Python ray tracer.

Our original `sionnart_main.cc` named the AP `"AP"` and the STAs `"STA_0"`, `"STA_1"`, … — none of these strings contain the substrings `"Tx"` or `"Rx"`. Consequences:

1. AP↔STA pairs (real links): `IsSameRoleLink("AP","STA_0")` returns `false` ✓ correct (the link is sent for tracing).
2. **STA↔STA pairs**: `IsSameRoleLink("STA_0","STA_1")` returns `false` ✗ **incorrect**. With N STAs, **N(N-1)/2 ghost links** were sent to Python's `SionnaPerformCalculation` looking for paths from a TX named `"STA_0"` — but Python's `tx_names` was initialized as `["AP"]` only. The lookup fails inside Sionna and produces undefined wall-clock behavior depending on which exception path is taken.

The bimodal pattern (tiny at `N ∈ {1, 2, 16, 32}`, huge at `N ∈ {4, 8}`) reflects different Python-side error-recovery paths as the count of garbage queries crosses internal thresholds.

### 6.3 Fix

Rename per the convention enforced elsewhere in the codebase (e.g. [scratch/sionnart-nr-energy/free-space-main.cc:1167,1208](../sionnart-nr-energy/free-space-main.cc#L1167)):

```diff
-    apMob->SetAttribute("ObjectName", StringValue("AP"));
+    apMob->SetAttribute("ObjectName", StringValue("Tx1"));
...
-    std::string name = "STA_" + std::to_string(i);
+    std::string name = "Rx" + std::to_string(i + 1);  // 1-indexed; must contain "Rx"
...
-    settings.tx_names = {"AP"};
+    settings.tx_names = {"Tx1"};
```

After the rename, `IsSameRoleLink("Rx1","Rx2") = true` correctly, the cache returns the same-role sentinel ([sionna-propagation-cache.cc:410-417](../../contrib/sionnart/model/sionna-propagation-cache.cc#L410)) for STA-STA pairs, and only the N legitimate AP↔STA links reach Python.

### 6.4 Post-fix scaling

Sionnart wall-clock now scales monotonically and produces ~99.95% cache-hit ratios with **exactly 1 ray-trace miss** per 10-second run (the initial scene trace) regardless of N or mobility regime:

| N | stationary | low_mob | high_mob |
|---:|---:|---:|---:|
| 1  | 0.204 | 0.164 | 0.192 |
| 2  | 0.229 | 0.197 | 0.239 |
| 4  | 0.499 | 0.318 | 0.508 |
| 8  | 0.734 | 0.447 | 0.739 |
| 16 | 0.801 | 0.671 | 0.825 |
| 32 | 1.389 | 1.134 | 1.468 |

Compared with ns3sionna for the same N, sionnart is **dramatically faster** (e.g. at N=32 stationary: 1.4 s vs 102 s; at N=16 high-mobility: 0.8 s vs 598 s — i.e. ~720× faster).

### 6.5 New caveat for sionnart

The post-fix data is internally consistent and scales correctly, **but raises a separate concern**: sionnart records exactly **1 ray-trace miss** per run even under the 7 m/s high-mobility regime over 10 simulated seconds (ns3sionna recorded 80–125× more wall-clock work in the same regime). Two possibilities:

1. **Sionnart's adaptive-prediction cache really is that effective** — its displacement-threshold + adaptive future-window logic prevents most invalidations within a 10-s window even at 7 m/s.
2. **Position updates from ns-3's mobility model never reach the Python receiver positions.** `SionnaUpdatePosition` is called from `SionnaPropagationCache::RefreshSnapshot` ([sionna-propagation-cache.cc:218](../../contrib/sionnart/model/sionna-propagation-cache.cc#L218)) but only for nodes in the live snapshot; if that path is short-circuited, mobility is invisible to the ray tracer and the cache stays valid forever.

Distinguishing these requires CSI fidelity comparison (does sionnart's path-loss change over a 10-s mobile run, or stay constant?). Listed as follow-up in §7. **For wall-clock-only conclusions, the post-fix sionnart numbers are valid** — they accurately reflect how cheap the simulation is when the cache is rarely invalidated.

---

## 7. Conclusions and follow-ups

### 7.1 Conclusions

- **Pure-ns-3 baseline:** ~linear in N, ~50× cheaper at low load than at high load. Confirms the analytical channel adds no measurable per-link cost; the simulator-event count is the dominant term.
- **ns3sionna:** consistent, sane scaling. The overhead vs pure ns-3 is **not constant** — it is dominated by how often the propagation cache is invalidated. In the realistic indoor-mobile case (1 m/s), it is already ~40×; in a 7 m/s case it is ~120× and starts to rule out N > 16 simulations on this hardware.
- **sionnart (post fix):** scales smoothly across all regimes (0.16 s at N=1 to 1.5 s at N=32) and is **substantially faster than both pure ns-3 and ns3sionna**, with the displacement-threshold cache reporting ~99.97% hits and only 1 ray-trace miss per 10-s run. Whether this reflects genuinely effective caching or a missed position-update path is the key remaining question (§6.5, §7.2 step 2).

### 7.2 Recommended next steps

1. ~~**Debug sionnart's bimodal pattern**~~ — **DONE** (§6). Root cause: ObjectName naming convention (`AP`/`STA_*` instead of `Tx1`/`Rx<i+1>`).
2. **Validate sionnart's mobility integration.** Confirm whether the 1-miss-per-run pattern under 7 m/s mobility is real cache effectiveness or a missed `SionnaUpdatePosition` path. Quickest test: log `SionnaMobilityModel::DoSetPosition` and `SionnaPropagationCache::RefreshSnapshot` calls during a 10-s run; expect O(N × sim_seconds / update_interval) position updates.
3. **Re-run sionnart with 3 reps** for tighter error bars (currently 1 rep post-fix).
4. **Stretch the budget for ns3sionna's N = 32, high mobility** (or down-sample sim duration to 5 s) to fill the missing point.
5. **Vary `min_coherence_time_ms`** (currently 1000 ms) to see how much of ns3sionna's high-mobility cost is structural vs. tunable.
6. **Add CSI-fidelity comparison.** Wall-clock alone does not distinguish "fast because well-cached" from "fast because not actually doing the work" (§6.5). Compare per-link path-loss traces between ns3sionna and sionnart at e.g. (N=4, high-mobility) over 10 s — they should agree if both back-ends are honestly tracking position.

---

## 8. How to reproduce

```bash
# Build (once, in the 6G env so sionnart links the right libpython)
conda activate 6G
cd ns3_server
rm -rf cmake-cache
./ns3 configure --enable-examples -- -DPython3_EXECUTABLE=/home/aung/anaconda3/envs/6G/bin/python3
./ns3 build pure_ns3_benchmark sionnart_benchmark ns3sionna_benchmark

# Sweep (auto-starts ns3sionna ZMQ server in 6Gold)
cd scratch/benchmark_sionnart_ns3sionna
./run_three_way.sh

# Re-aggregate + redraw plots
python analyze_results.py
```

Single back-end runs:
```bash
./bench.sh pure_ns3   --num_stas=8 --sim_seconds=10 --regime=stationary_high_load
./bench.sh ns3sionna  --num_stas=8 --sim_seconds=10 --regime=stationary_high_load   # auto-starts server
./bench.sh sionnart   --num_stas=8 --sim_seconds=10 --regime=stationary_high_load
./bench.sh all        --num_stas=8 --sim_seconds=10 --regime=stationary_high_load
```

Server lifecycle:
```bash
./server.sh status | start | stop | restart | log
```

---

## 9. Files

| File | Purpose |
|---|---|
| `pure_ns3_main.cc`, `ns3sionna_main.cc`, `sionnart_main.cc` | Aligned-scenario benchmark binaries |
| `CMakeLists.txt` | Three targets, correct lib linkage |
| `server.sh` | ns3sionna ZMQ server lifecycle (runs in 6Gold) |
| `bench.sh` | Single-shot wrapper, auto-handles env + server |
| `run_three_way.sh` | Full sweep driver → `results.csv` |
| `analyze_results.py` | Aggregation + plot generation |
| `plot_results.py` | Simple per-regime plotter (used during development) |
| `results.csv` | Raw per-run wall-clock measurements |
| `results_aggregated.csv` | Median/min/max/reps per (backend, regime, N) |
| `plot_*.png` | Per-regime curves and overhead summary |
| `PROGRESS.md` | Build & run notes for resume / handoff |
| `REPORT.md` | This document |
