#pragma once

// macOS App Nap opt-out. App Nap throttles a process (here: OpenEphys's
// plugin-container) a few seconds after it looks idle/background, cutting its CPU
// and timer allocation. That starves the UDP recv thread and produces broadband
// packet loss that begins ~10 s into acquisition (and never with net.py, which is
// a foreground terminal process). beginActivityWithOptions:NSActivityLatencyCritical
// tells macOS "this process is doing latency-critical work -- do not nap it", the
// same mechanism real-time audio/capture apps use. No-op on non-Apple platforms.
#ifdef __APPLE__
void disableAppNap();
#endif
