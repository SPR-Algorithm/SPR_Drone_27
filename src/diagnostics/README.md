# diagnostics
Owner: SilverWolf. Run manifest, events.jsonl, metrics.csv, offline HTML report.
The bringup supervisor samples process/disk metrics at 1 Hz; sensor preflight
summarizes rates and gaps. The px4_bridge publishes /spr/diagnostics
(best_effort depth 5) with host-time synchronization health.
No hardware access or TF. Logging remains local; HTML has no remote dependencies.
Test: `colcon test --packages-select diagnostics`.
Continuous cross-package diagnostics aggregation/flight telemetry is A1 work.
