# Sionna 8x8 Runtime Notes

The Sionna propagation integration now uses one adaptive prediction path.
There is no runtime selector for alternate propagation-record modes.

## Active controls

```text
--sionnaAdaptiveFutureHorizonSeconds=3
--sionnaAdaptiveFutureMinBenefitSeconds=1
--sionnaAdaptiveFutureMaxSteps=3
--sionnaAdaptiveFutureDirectionDotThreshold=0.7
```

## Runtime interpretation

For 8x8 arrays with 20 UEs, the dominant cost can still be the required
Sionna ray-tracing and CFR export itself. Adaptive prediction limits extra
virtual receiver work, but it does not reduce antenna count, exported MIMO
channel fidelity, or physical channel calculation detail.

The expected optimization target is fewer unnecessary speculative receiver
positions. If a short run is still slow before ns-3 simulation time advances,
instrument Python-side initialization, `PathSolver`, and `paths.cfr()` timing.
