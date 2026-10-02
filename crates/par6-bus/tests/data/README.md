`periodic-current-v2.csv` contains the outputs of the real firmware's portable
periodic sequencer at the capture sample instants, for both profiles at 90 mA.
It is a waveform/protocol fixture, not a hardware measurement or a simulated
plant response. Regenerate it by compiling and running
`STEPFOC firmware/tests/periodic_capture.cpp` from the firmware source. That
program also checks settling, sample count, duration, cancellation, faults,
mode changes and clipped/invalid experiments.

The Rust protocol test compares every recorded value with the host's waveform
reconstruction and checks malformed, incomplete and mismatched metadata.

`periodic-current-v1.csv` is retained as the historical equal-weight waveform;
its protocol version must not be accepted by the current host.
