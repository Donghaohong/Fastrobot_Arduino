import os
import pathlib
import sys
import time

import numpy as np

from cmd_types import CMD

PARENT_DIR = pathlib.Path(__file__).resolve().parent.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.append(str(PARENT_DIR))

from utils import load_config_params


class RealRobot:
    """Adapter class that lets the Localization module query the real robot."""

    def __init__(self, commander, ble):
        # The notebook runs from FastRobots-sim-release-main/notebooks.
        self.world_config = os.path.join(
            str(pathlib.Path(os.getcwd()).parent),
            "config",
            "world.yaml",
        )
        self.config_params = load_config_params(self.world_config)

        # Commander is only used by the localization code for plotting.
        self.cmdr = commander
        self.ble = ble

        self.observation_count = int(
            self.config_params["mapper"]["observations_count"]
        )
        self.step_deg = 360.0 / self.observation_count
        self.max_range_m = float(self.config_params["sensor_range"])

        # Reuse the scan settings that were already tuned in Lab 9.
        self.scan_kp = 0.8
        self.scan_ki = 0.001
        self.scan_kd = 0.2
        self.scan_settle_ms = 550
        self.scan_timeout_s = 20.0
        self.log_timeout_s = 10.0
        self.poll_s = 0.05
        self.verbose = True

        self._reset_scan_state()

    def _reset_scan_state(self):
        self._scan_messages = []
        self._scan_status = []
        self._live_samples = []
        self._log_samples = []
        self._scan_done = False
        self._log_active = False
        self._log_done = False

    def _emit_status(self, message):
        if self.verbose:
            print(message)

    def _scan_notification_handler(self, uuid, byte_array):
        message = self.ble.bytearray_to_string(byte_array).strip()
        if not message:
            return

        self._scan_messages.append(message)

        if message.startswith("MAP_SCAN_STARTED"):
            self._scan_status.append(message)
            self._emit_status(message)
            return

        if message.startswith("MAP_STEP_TARGET"):
            return

        if message.startswith("MAP_SAMPLE"):
            parts = message.split(",")
            if len(parts) == 5:
                try:
                    sample = {
                        "step_idx": int(parts[1]),
                        "yaw_deg": float(parts[2]),
                        "dist_mm": float(parts[3]),
                        "setpoint_deg": float(parts[4]),
                    }
                except ValueError:
                    return

                self._live_samples.append(sample)
                self._emit_status(
                    "sample "
                    f"{sample['step_idx']:02d}: "
                    f"{sample['dist_mm']:.1f} mm @ {sample['yaw_deg']:.2f} deg"
                )
            return

        if (
            message.startswith("MAP_SCAN_DONE")
            or message == "MAP_SCAN_STOPPED"
            or message == "MAP_SCAN_YAW_INVALID"
            or message == "MAP_SCAN_BAD_ARGS"
        ):
            self._scan_status.append(message)
            self._scan_done = True
            self._emit_status(message)
            return

        if message == "MAP_BEGIN":
            self._log_samples = []
            self._log_active = True
            self._log_done = False
            return

        if message.startswith("MAP,") and self._log_active:
            parts = message.split(",")
            if len(parts) == 6:
                try:
                    sample = {
                        "step_idx": int(parts[1]),
                        "t_ms": float(parts[2]),
                        "yaw_deg": float(parts[3]),
                        "dist_mm": float(parts[4]),
                        "setpoint_deg": float(parts[5]),
                    }
                except ValueError:
                    return

                self._log_samples.append(sample)
            return

        if message.startswith("MAP_END"):
            self._log_active = False
            self._log_done = True
            return

    def _wait_for(self, attr_name, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if getattr(self, attr_name):
                return True
            time.sleep(self.poll_s)
        return False

    def _request_map_log(self):
        self._log_samples = []
        self._log_active = False
        self._log_done = False
        self.ble.send_command(CMD.SEND_MAP_SCAN, "")
        return self._wait_for("_log_done", self.log_timeout_s)

    def _ordered_samples(self, samples):
        if not samples:
            return None

        by_idx = {}
        for sample in samples:
            by_idx[int(sample["step_idx"])] = sample

        if len(by_idx) < self.observation_count:
            return None

        ordered = []
        for idx in range(self.observation_count):
            if idx not in by_idx:
                return None
            ordered.append(by_idx[idx])

        return ordered

    def get_pose(self):
        raise NotImplementedError(
            "RealRobot.get_pose() is not available in this notebook. "
            "Lab 11 on the real robot uses only the update step."
        )

    def perform_observation_loop(self, rot_vel=120):
        """Run one 360 degree scan and return localization-ready observations."""
        del rot_vel

        self._reset_scan_state()

        try:
            try:
                self.ble.stop_notify(self.ble.uuid["RX_STRING"])
            except Exception:
                pass

            self.ble.start_notify(
                self.ble.uuid["RX_STRING"],
                self._scan_notification_handler,
            )

            command = (
                f"{self.step_deg}|{self.observation_count}|"
                f"{self.scan_kp}|{self.scan_ki}|{self.scan_kd}|{self.scan_settle_ms}"
            )
            self.ble.send_command(CMD.START_MAP_SCAN, command)

            if not self._wait_for("_scan_done", self.scan_timeout_s):
                raise TimeoutError("Timed out waiting for MAP_SCAN_DONE")
            time.sleep(0.2)

            ordered_samples = self._ordered_samples(self._live_samples)
            if ordered_samples is None:
                self._emit_status(
                    "live scan samples incomplete, requesting stored MAP log"
                )
                if not self._request_map_log():
                    raise TimeoutError("Timed out waiting for MAP_END")
                time.sleep(0.2)
                ordered_samples = self._ordered_samples(self._log_samples)

            if ordered_samples is None:
                raise RuntimeError(
                    "Could not recover a full observation loop. "
                    f"Expected {self.observation_count} samples, "
                    f"got {len(self._live_samples)} live and "
                    f"{len(self._log_samples)} log samples."
                )

            ranges_m = np.array(
                [sample["dist_mm"] / 1000.0 for sample in ordered_samples],
                dtype=float,
            )
            invalid = (~np.isfinite(ranges_m)) | (ranges_m <= 0.0)
            ranges_m[invalid] = self.max_range_m
            ranges_m = np.clip(ranges_m, 0.0, self.max_range_m)

            bearings_deg = np.arange(
                self.observation_count,
                dtype=float,
            ) * self.step_deg

            sensor_ranges = ranges_m[:, np.newaxis]
            sensor_bearings = bearings_deg[:, np.newaxis]
            return sensor_ranges, sensor_bearings

        finally:
            try:
                self.ble.stop_notify(self.ble.uuid["RX_STRING"])
            except Exception:
                pass
