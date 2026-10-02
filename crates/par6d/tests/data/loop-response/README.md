# Physical J1 captures

These CSVs are unchanged captures from the real arm on 2026-09-27, recorded
by the drive at 6250 Hz with the divisor in each file's header. No simulator
generated them. Original run IDs and filenames:

| Fixture | Run | Original filename |
|---|---|---|
| j1-fast.csv | selfcal-1790535526375136968 | injected-J1-pose0-fast-1.csv |
| j1-slow.csv | selfcal-1790535526375136968 | injected-J1-pose0-slow-1.csv |
| j1-baseline.csv | selfcal-1790488502258729638 | capture-J1-31.csv |
| j1-worse.csv | selfcal-1790488502258729638 | capture-J1-32.csv |
| j1-better.csv | selfcal-1790488502258729638 | capture-J1-33.csv |

The aligned pair measures the configured PI consistently, but does not resolve
the plant throughout the crossover region. It must not authorize new gains.
The replay also shifts the fast record's injection to check rejection of
misaligned channels.

The three validation steps share Kpv 0.02 and command -5825 motor ticks/s.
Kiv 0.003, 0.0018 and 0.0045 give normalized tracking errors approximately
0.1465, 0.1595 and 0.1012, respectively. A finite score alone would accept
the worse trial. These captures exercise the step decision; they do not
validate new gains in both directions, at other poses, or while holding.
