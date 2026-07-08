"""
logging_utils/data_logger.py
-----------------------------
Logs every simulation/control tick to an in-memory buffer and periodically
flushes to a CSV file via pandas, so a full time-series of the run is
available for offline analysis.
"""

import os
import time
import pandas as pd
import config


class DataLogger:
    def __init__(self, log_dir: str = None):
        self.log_dir = log_dir or config.LOG_DIR
        os.makedirs(self.log_dir, exist_ok=True)
        filename = time.strftime(config.LOG_FILENAME_FMT)
        self.filepath = os.path.join(self.log_dir, filename)
        self._rows = []
        self._header_written = False

    def log(self, row: dict):
        self._rows.append(row)
        if len(self._rows) >= config.LOG_FLUSH_EVERY_N_ROWS:
            self.flush()

    def flush(self):
        if not self._rows:
            return
        df = pd.DataFrame(self._rows)
        mode = "a" if self._header_written else "w"
        header = not self._header_written
        df.to_csv(self.filepath, mode=mode, header=header, index=False)
        self._header_written = True
        self._rows = []

    def close(self):
        self.flush()

    def load_dataframe(self) -> pd.DataFrame:
        """Convenience method to read back the full log (e.g. for post-run analysis)."""
        self.flush()
        if os.path.exists(self.filepath):
            return pd.read_csv(self.filepath)
        return pd.DataFrame()
