# Clean warehouse ISAC benchmark

This directory is an intentionally small replacement for the application-heavy
`scratch/warehouse-isac` experiment. It tests only two hypotheses:

1. Detection-triggered ISAC beam refresh should outperform the same system with
   periodic communication-only beam refresh.
2. Increasing the gNB array from 2x2 to 4x4 to 8x8 should improve effective link
   loss and should not reduce throughput or reliability under saturated load.

The benchmark also requires the local correction in
`contrib/nr/model/beamforming-vector.cc`: direct-path beam weights must be the
complex conjugate of the steering vector. The previous sign doubled element
phase in the phased propagation dot product, producing deeper nulls for larger
arrays.

## Metric definitions

- `Goodput`: received application bytes divided by the configured traffic
  measurement interval.
- `EndToEndLostPackets`: `TxPackets - RxPackets` after an 11 second drain. This
  is the benchmark's packet-loss metric.
- `RawFlowMonitorLostPackets`: FlowMonitor's timeout counter, retained only as a
  diagnostic. It can exceed final end-to-end loss when a packet arrives after
  FlowMonitor has already classified it as overdue.
- `PropagationPathLoss_dB`: physical scene/path attenuation. It should not
  improve when the antenna array grows.
- `EffectiveLinkLoss_dB`: negative total channel gain after antenna/beam gain.
  This is the link-loss metric expected to improve with array size and beam
  tracking.

The benchmark uses deterministic waypoint mobility, equal saturated downlink
UDP load for three UEs, 1200 byte packets, direct-path analog beamforming, and
the scalar Sionna channel path. The no-ISAC mode uses a 2 second periodic
communication-only beam update. ISAC uses the sensing-frame cadence for beam
tracking and separately verifies that sensing produced matched UE detections.
This models the control benefit of sensing without calling the beamforming
helper asynchronously from inside a receive/poll callback.

## Build and run

```bash
cd /home/aung/code/docte6g/ns3_server
./ns3 configure
./ns3 build warehouse-benchmark-isac warehouse-benchmark-no-isac
./scratch/warehouse/run_sweep.sh
```

Results are written to `results/warehouse-clean` by default. The analyzer exits
nonzero when a configured monotonicity or ISAC comparison check fails; it does
not alter or normalize results to make the hypothesis pass.
