"""Unit tests for the CSV logger."""

import csv

from auv1.telemetry_log import CsvLogger


def test_writes_header_and_rows_and_autofills_t():
    log = CsvLogger("unittest", ["t", "depth", "target"])
    try:
        log.row(depth=1.5, target=2.0)
        log.row(depth=1.8)                    # missing field -> blank
        log.close()

        with open(log.path) as f:
            rows = list(csv.reader(f))
        assert rows[0] == ["t", "depth", "target"]
        assert len(rows) == 3
        assert rows[1][1] == "1.5" and rows[1][2] == "2.0"
        assert rows[2][2] == ""               # blank, not crash
        assert rows[1][0] != ""               # t auto-filled
    finally:
        log.close()
        log.path.unlink(missing_ok=True)      # keep logs/ tidy
