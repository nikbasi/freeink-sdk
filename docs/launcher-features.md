# T5 S3 Pro features Basilauncher depends on

Basilauncher (the LilyGo T5 S3 Pro firmware hub) builds against this SDK as a
submodule. Some of what keeps its screen fast and free of ghosts came from the
`aurora` line and from T5-specific fixes that upstream FreeInk does not carry.
An upstream merge that resolves a conflict the wrong way can drop one without
breaking the build, so each one has a marker, and Basilauncher refuses to
build without them (`src/sdk_requirements.h` checks the macros,
`scripts/check_sdk.py` checks the waveform tables and power sequence).

When you merge upstream, keep these. If you change one on purpose, update the
marker and the launcher's checks in the same change.

| Marker | Where | What it keeps |
|---|---|---|
| `FREEINK_HAS_DEFERRED_WINDOW` | `FreeInkDisplay.h` | `displayWindowDeferred()` and `windowRefreshBusy()`: a window refresh is queued and the call returns while the waveform runs, so drags and key flashes never block touch. |
| `FREEINK_HAS_DEFERRED_FRAME` | `FreeInkDisplay.h` | `displayFrameDeferred()`: the same for a whole frame, used for the settle clean refresh. |
| `FREEINK_HAS_QUEUED_LONG_PRESS` | `InputManager.h` | The async input task queues long presses for `popTouchLongPress()`. |
| `FREEINK_ED047TC2_CLEAN_FRAMES_PER_DRIVE` | `ED047TC2Waveform.h` | aurora's clean bank: 2*L[15] frames instead of 3*L[15] (about 3.4 s down to 2.5 s at 24 C). |
| `FREEINK_T5S3_STAGED_RAIL_PARK` | `BoardT5S3.h` | STV, VCOM, discharge wait, boost rails, then WAKEUP, so a settled page does not fade to grey. |
| `FREEINK_T5S3_WAVEFORM_FLOOR_C` | `BoardT5S3.h` | Warm panels keep the 21-frame drive; shorter impulses do not latch. |

Two driver details have no marker but are checked by the launcher script:

- `normalizeForCleanBank()` runs before every clean refresh, not only after a
  fast one. Window refreshes leave pixels the clean bank would otherwise skip.
- The panel waits through the ingest window (`lgfxPanelReady()`) before any
  push, so a deferred refresh is never torn by the next one.
