"""Frontend panels par6 contributes to a waldoctl host.

Importing this package pulls in NiceGUI, so nothing in the runtime path
imports it: the host reaches it through the ``waldoctl.panels``
entry point and skips it with a log line when the ``gui`` extra is absent.
"""
