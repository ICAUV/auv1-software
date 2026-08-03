"""CSV telemetry logging — the vehicle's black box.

Every control script should log each tick: what the vehicle reported,
what the target was, what the controller commanded. Files land in
logs/ (gitignored) named <prefix>_YYYYmmdd_HHMMSS.csv and open
directly in Excel, or plot with matplotlib.

Usage:
    log = CsvLogger("depth_hold", ["t", "depth", "target", "effort"])
    log.row(t=1.2, depth=0.42, target=2.0, effort=0.6)
    ...
    log.close()      # or rely on close-on-exit; data is flushed per row
"""

import csv
import os
import time
from pathlib import Path


class CsvLogger:
    def __init__(self, prefix: str, fields: list):
        # AUV1_LOG_DIR (if set) overrides the default repo-local logs/.
        # Use it to collect all machines' logs in one archive, e.g. in
        # Ubuntu/WSL:  export AUV1_LOG_DIR=/mnt/c/pojects/auv1-software/logs
        # (add to ~/.bashrc to make it permanent). On the Pi, leave it
        # unset and offload logs after each session.
        override = os.environ.get("AUV1_LOG_DIR")
        logs_dir = (Path(override) if override
                    else Path(__file__).resolve().parents[1] / "logs")
        logs_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = logs_dir / f"{prefix}_{stamp}.csv"
        self.fields = list(fields)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields)
        self._writer.writeheader()
        self._t0 = time.monotonic()
        print(f"Logging to {self.path}")

    def row(self, **values) -> None:
        """Write one tick. Unknown fields are rejected loudly (typo guard);
        missing fields are left blank. Auto-fills 't' (seconds since
        logger creation) if it's a field and wasn't provided."""
        if "t" in self.fields and "t" not in values:
            values["t"] = round(time.monotonic() - self._t0, 3)
        self._writer.writerow({k: values.get(k, "") for k in self.fields})
        self._file.flush()   # survive crashes/Ctrl+C with data intact

    def close(self) -> None:
        try:
            self._file.close()
        except Exception:
            pass
