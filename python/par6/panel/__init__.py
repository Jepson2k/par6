"""The control box front panel: two buttons, two LEDs, a 128x64 OLED and
the UART link to the mainboard PCB, run as a service beside ``par6d``.

Everything about the hardware — device paths, I²C address, pins, baud —
comes from ``panel.toml`` (:mod:`par6.panel.config`); the UI is a small
state machine (:mod:`par6.panel.ui`) driven by polled buttons
(:mod:`par6.panel.buttons`); the heartbeat, LEDs and PCB frames live in
:mod:`par6.panel.link`; :mod:`par6.panel.service` is the entry point and
:mod:`par6.panel.preflight` the diagnostic check that brings nothing up.
"""
