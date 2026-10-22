#!/usr/bin/env python3
"""
DomyosBridge for Linux (BlueZ D-Bus BLE Central + FTMS 0x1826 BLE Peripheral)
=============================================================================
Acts as an intermediary "Virtual Smart Trainer" on Linux for Decathlon
Domyos Bike 900 / e-Bike 900:
  1. Connects to Decathlon Domyos 900 bike hardware as a BLE Central via
     BlueZ D-Bus (supporting both standard FTMS 0x1826 and Decathlon
     proprietary 49535343-... / 0xFFF0 UART protocols).
  2. Advertises a standard FTMS (0x1826) + Cycling Power (0x1818) Smart Trainer
     GATT Server as a BLE Peripheral so MyWhoosh on your Pixel 9 Pro XL can
     connect to this laptop.
  3. Automatically adjusts physical Domyos 1-32 resistance as the route grade
     changes in MyWhoosh (OpCode 0x11) or during ERG workouts (OpCode 0x05),
     while allowing instantaneous Mouse Wheel UP/DOWN Virtual Gear shifting
     (Gears 1-32, Base Level 18) on top of the route gradient.
  4. Uses a QZ-calibrated Domyos Bike 900 32-level resistance-to-watts lookup
     curve whenever the physical console does not transmit native watts.
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import math
import os
import re
import signal
import struct
import sys
import time
import termios
import tty
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# ---------------------------------------------------------------------------
# D-Bus & BlueZ Constants
# ---------------------------------------------------------------------------
BLUEZ_SERVICE_NAME = "org.bluez"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"

# Standard Bluetooth SIG FTMS & Cycling Power UUIDs
FTMS_SERVICE_UUID = "00001826-0000-1000-8000-00805f9b34fb"
FTMS_FEATURE_UUID = "00002acc-0000-1000-8000-00805f9b34fb"
INDOOR_BIKE_DATA_UUID = "00002ad2-0000-1000-8000-00805f9b34fb"
SUPPORTED_RES_RANGE_UUID = "00002ad6-0000-1000-8000-00805f9b34fb"
SUPPORTED_POWER_RANGE_UUID = "00002ad8-0000-1000-8000-00805f9b34fb"
FTMS_CONTROL_POINT_UUID = "00002ad9-0000-1000-8000-00805f9b34fb"
FTMS_STATUS_UUID = "00002ada-0000-1000-8000-00805f9b34fb"

CPS_SERVICE_UUID = "00001818-0000-1000-8000-00805f9b34fb"
CPS_MEASUREMENT_UUID = "00002a63-0000-1000-8000-00805f9b34fb"
CPS_FEATURE_UUID = "00002a65-0000-1000-8000-00805f9b34fb"
CPS_SENSOR_LOCATION_UUID = "00002a5d-0000-1000-8000-00805f9b34fb"

# Decathlon / Domyos Proprietary Console UUIDs
DOMYOS_ISSC_SERVICE_UUID = "49535343-fe7d-4ae5-8fa9-9fafd205e455"
DOMYOS_ISSC_NOTIFY_UUID = "49535343-1e4d-4bd9-ba61-23c647249616"
DOMYOS_ISSC_WRITE_UUID = "49535343-8841-43f4-a8d4-ecbe34729bb3"

DOMYOS_FFF0_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
DOMYOS_FFF1_NOTIFY_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
DOMYOS_FFF2_WRITE_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"

TARGET_NAME_PREFIXES = (
    "DOMYOS",
    "TRAINING BIKE",
    "EB900",
    "EB 900",
    "BIKING",
    "CHALLENGE",
    "DECATHLON",
    "FS-",
    "EW-BK",
)

# Domyos proprietary UART initialization sequence, copied from QZ domyosbike.cpp
# (btinit_changyow, without the optional "start tape" frames). Frames longer than
# 20 bytes are sent in 20-byte chunks (ATT MTU 23), exactly like QZ does.
DOMYOS_INIT_SEQUENCE = (
    bytes([0xF0, 0xC8, 0x01, 0xB9]),
    bytes([0xF0, 0xC9, 0xB9]),
    bytes([0xF0, 0xA3, 0x93]),
    bytes([0xF0, 0xA4, 0x94]),
    bytes([0xF0, 0xA5, 0x95]),
    bytes([0xF0, 0xAB, 0x9B]),
    bytes([0xF0, 0xC4, 0x03, 0xB7]),
    # power-on BT icon
    bytes([0xF0, 0xAD] + [0xFF] * 16 + [0x01, 0xFF] + [0xFF, 0xFF, 0x8B]),
    # power-on BT word
    bytes(
        [0xF0, 0xCB, 0x02, 0x00, 0x08] + [0xFF] * 13 + [0x01, 0x00]
        + [0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xB6]
    ),
    bytes(
        [0xF0, 0xAD, 0xFF, 0xFF, 0x00, 0x05, 0xFF, 0xFF, 0xFF, 0xFF,
         0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x01, 0xFF]
        + [0xFF, 0xFF, 0x94]
    ),
)

# QZ keep-alive poll (sent every refresh cycle) and ATT chunk size
DOMYOS_NOOP_FRAME = bytes([0xF0, 0xAC, 0x9C])
DOMYOS_ATT_CHUNK = 20

# QZ ftmsbike (DOMYOS): minimum spacing between FTMS resistance commands
FTMS_MIN_WRITE_INTERVAL_S = 1.5

# QZ-Calibrated Domyos Training Bike 900 Watts @ 80 RPM for Resistance Levels 1..32
DOMYOS_900_WATTS_AT_80RPM = (
    38,  44,  51,  58,  66,  74,  83,  92,   # Levels 1..8   (Recovery / Flat Spin)
    102, 112, 123, 134, 146, 159, 172, 186,  # Levels 9..16  (Endurance / Tempo)
    201, 217, 234, 252, 271, 291, 312, 334,  # Levels 17..24 (Threshold / Vo2Max)
    358, 383, 410, 438, 468, 500, 534, 570,  # Levels 25..32 (Steep Climb / Sprint)
)

SGR_MOUSE_RE = re.compile(r"\x1b\[<(\d+);(\d+);(\d+)([Mm])")
EVDEV_EVENT_STRUCT = struct.Struct("llHHi")
EV_REL = 0x02
REL_WHEEL = 0x08


# ---------------------------------------------------------------------------
# Shared Bridge State & Calibration Helpers
# ---------------------------------------------------------------------------
@dataclass
class BridgeState:
    # Central (Laptop -> Domyos Bike 900)
    central_status: str = "Scanning for Domyos Bike 900..."
    domyos_name: str = "Not Connected"
    domyos_protocol: str = "-"
    domyos_rssi: Optional[int] = None

    # Peripheral (MyWhoosh -> Laptop)
    peripheral_status: str = "Initializing..."
    mywhoosh_subscribed: bool = False

    # Live Telemetry
    speed_kmh: float = 0.0
    cadence_rpm: int = 0
    power_watts: int = 0
    current_level: int = 18
    target_level: int = 18
    slope_percent: float = 0.0
    wind_speed_mps: float = 0.0

    # Virtual Shifting & Slope Mapping
    # Virtual Gear 1..32 represents your flat-road (0% slope) resistance level (Default: Level 18)
    virtual_gear: int = 18
    max_virtual_gear: int = 32
    slope_sensitivity: float = 1.25
    control_mode: str = "SIM (Auto Route Slope + Mouse Gear)"
    last_erg_watts: int = 0
    simulation_mode: bool = False
    protocol_mode: str = "auto"
    max_level: int = 32
    base_gear: int = 18
    global_mouse_attached: bool = False
    last_event: str = "Ready (Base Lvl 18) — Scroll Mouse Wheel Up/Down to shift"


def estimate_domyos_watts(cadence_rpm: int, level: int, max_level: int = 32) -> int:
    """
    QZ-calibrated power lookup for the Decathlon Domyos Bike 900 magnetic brake.
    """
    if cadence_rpm <= 5:
        return 0
    if max_level == 15:
        lvl = max(1, min(15, int(level)))
        idx = int(round((lvl - 1) * 31.0 / 14.0))
    else:
        idx = max(1, min(32, int(level))) - 1
    watts_at_80 = DOMYOS_900_WATTS_AT_80RPM[idx]
    rpm_ratio = cadence_rpm / 80.0
    watts = watts_at_80 * (0.72 * (rpm_ratio ** 1.15) + 0.28 * (rpm_ratio ** 1.85))
    return max(0, min(1500, int(round(watts))))


def normalize_ftms_resistance(raw_val: int, max_level: int = 32) -> int:
    """Normalizes FTMS resistance parameter into discrete Domyos 1..max_level steps."""
    if raw_val <= 0:
        return 1
    if 1 <= raw_val <= max_level:
        return raw_val
    if max_level < raw_val <= max_level * 10:
        return max(1, min(max_level, int(round(raw_val / 10.0))))
    return max(1, min(max_level, int(round((raw_val / 1000.0) * max_level))))


def build_domyos_checksum_frame(prefix: bytes) -> bytes:
    """Appends modulo-256 sum checksum byte for Domyos proprietary UART commands."""
    checksum = sum(prefix) & 0xFF
    return prefix + bytes([checksum])


def build_domyos_display_frame(elapsed_s: int) -> bytes:
    """
    27-byte 0xF0 0xCB console display update (QZ domyosbike::updateDisplay template).
    The bridge only fills the elapsed time; speed/HR/calories stay zero.
    """
    frame = bytearray(
        [0xF0, 0xCB, 0x03, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00,
         0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00]
    )
    frame[3] = (elapsed_s // 60) & 0xFF
    frame[4] = (elapsed_s % 60) & 0xFF
    frame[26] = sum(frame[:26]) & 0xFF
    return bytes(frame)


def build_domyos_23byte_ad_frame(level_or_ff: int = 0xFF) -> bytes:
    """
    Builds the 23-byte 0xF0 0xAD control/keep-alive packet used by Domyos Bike 900
    (matches QDomyos-Zwift domyosbike.cpp forceResistance).
    Byte 10 is the requested resistance level (QZ forceResistance). Keep-alive uses 0xF0 0xAC 0x9C.
    """
    prefix = bytearray(
        [
            0xF0, 0xAD,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            level_or_ff & 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x01,
            0xFF, 0xFF, 0xFF,
        ]
    )
    return build_domyos_checksum_frame(bytes(prefix))


# ---------------------------------------------------------------------------
# BlueZ BLE Peripheral: Advertisement & FTMS 0x1826 + CPS 0x1818 GATT Server
# ---------------------------------------------------------------------------
class FtmsAdvertisement(dbus.service.Object):
    PATH = "/com/domyos/bridge/advertisement0"

    def __init__(self, bus: dbus.Bus):
        self.path = self.PATH
        self.bus = bus
        super().__init__(bus, self.path)

    def get_properties(self) -> Dict[str, Dict[str, object]]:
        # 30-byte advertisement payload: Flags (3B) + UUID16 list 0x1826/0x1818 (6B) +
        # ServiceData [0x01, 0x20, 0x00] Indoor Bike (7B) + LocalName (14B) <= 31B
        service_data = dbus.Dictionary(
            {
                FTMS_SERVICE_UUID: dbus.Array(
                    [dbus.Byte(0x01), dbus.Byte(0x20), dbus.Byte(0x00)],
                    signature="y",
                )
            },
            signature="sv",
        )
        return {
            LE_ADVERTISEMENT_IFACE: {
                "Type": "peripheral",
                "ServiceUUIDs": dbus.Array(
                    [FTMS_SERVICE_UUID, CPS_SERVICE_UUID], signature="s"
                ),
                "ServiceData": service_data,
                "LocalName": dbus.String("DomyosBridge"),
            }
        }

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface: str):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                f"Unknown interface {interface}",
            )
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        pass


class GattCharacteristic(dbus.service.Object):
    """
    Base BlueZ GattCharacteristic1 object.
    Note: dbus-python binds @dbus.service.method to the base class function in
    _dbus_class_table, so subclasses MUST override the underscore hook methods
    (_on_write_value, _on_start_notify, _on_stop_notify) rather than the
    D-Bus-decorated methods directly.
    """

    def __init__(
        self,
        bus: dbus.Bus,
        index: int,
        uuid: str,
        flags: List[str],
        service_path: str,
    ):
        self.path = f"{service_path}/char{index}"
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.service_path = service_path
        self.notifying = False
        self.value: List[dbus.Byte] = []
        super().__init__(bus, self.path)

    def get_properties(self) -> Dict[str, Dict[str, object]]:
        return {
            GATT_CHRC_IFACE: {
                "Service": dbus.ObjectPath(self.service_path),
                "UUID": self.uuid,
                "Flags": dbus.Array(self.flags, signature="s"),
                "Value": dbus.Array(self.value, signature="y"),
            }
        }

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface: str):
        if interface != GATT_CHRC_IFACE:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                f"Unknown interface {interface}",
            )
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return self._on_read_value(options)

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        self._on_write_value(value, options)

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="", out_signature="")
    def StartNotify(self):
        self.notifying = True
        self._on_start_notify()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="", out_signature="")
    def StopNotify(self):
        self.notifying = False
        self._on_stop_notify()

    # Subclass hooks
    def _on_read_value(self, options):
        return dbus.Array(self.value, signature="y")

    def _on_write_value(self, value, options):
        pass

    def _on_start_notify(self):
        pass

    def _on_stop_notify(self):
        pass

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def send_notification(self, payload: bytes):
        self.value = [dbus.Byte(b) for b in payload]
        # Always emit PropertiesChanged to bluetoothd so clients that stayed
        # connected across a script restart still receive notifications immediately.
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {"Value": dbus.Array(self.value, signature="y")},
            [],
        )


class FtmsControlPointCharacteristic(GattCharacteristic):
    """Handles MyWhoosh writes to FTMS Control Point (0x2AD9)."""

    def __init__(
        self,
        bus: dbus.Bus,
        index: int,
        service_path: str,
        on_resistance_cmd: Callable[[int], None],
        on_simulation_cmd: Callable[[float, float], None],
        on_target_power_cmd: Callable[[int], None],
        on_status_notify: Callable[[bytes], None],
        on_subscription_change: Callable[[bool], None],
    ):
        super().__init__(
            bus,
            index,
            FTMS_CONTROL_POINT_UUID,
            ["write", "indicate"],
            service_path,
        )
        self.on_resistance_cmd = on_resistance_cmd
        self.on_simulation_cmd = on_simulation_cmd
        self.on_target_power_cmd = on_target_power_cmd
        self.on_status_notify = on_status_notify
        self.on_subscription_change = on_subscription_change
        self._ind_queue: List[bytes] = []
        self._ind_busy = False

    def _on_start_notify(self):
        self.on_subscription_change(True)

    def _on_stop_notify(self):
        self.on_subscription_change(False)

    def _on_write_value(self, value, options):
        raw = bytes(int(b) for b in value)
        if not raw:
            return

        opcode = raw[0]
        if opcode == 0x00:
            # Request Control
            self.schedule_indication(opcode, 0x01)
        elif opcode == 0x01:
            # Reset: resistance is intentionally left at Base Level 18
            self.schedule_indication(opcode, 0x01)
        elif opcode in (0x07, 0x08):
            # Start / Stop
            self.schedule_indication(opcode, 0x01)
        elif opcode == 0x04:
            # Set Target Resistance Level
            if len(raw) < 2:
                self.schedule_indication(opcode, 0x03)
                return
            if len(raw) >= 3:
                (raw_level,) = struct.unpack_from("<h", raw, 1)
            else:
                raw_level = raw[1]
            level = normalize_ftms_resistance(raw_level)
            self.on_resistance_cmd(level)
            self.schedule_indication(opcode, 0x01)
            self.on_status_notify(bytes([0x07, level & 0xFF]))
        elif opcode == 0x05:
            # Set Target Power (ERG mode)
            if len(raw) < 3:
                self.schedule_indication(opcode, 0x03)
                return
            (target_watts,) = struct.unpack_from("<h", raw, 1)
            self.on_target_power_cmd(max(0, min(2000, target_watts)))
            self.schedule_indication(opcode, 0x01)
        elif opcode == 0x11:
            # Set Indoor Bike Simulation Parameters (Wind Speed, Grade/Slope, Crr, Cw)
            if len(raw) < 7:
                self.schedule_indication(opcode, 0x03)
                return
            wind_raw, grade_raw = struct.unpack_from("<hh", raw, 1)
            wind_mps = wind_raw / 1000.0
            grade_pct = grade_raw / 100.0
            self.on_simulation_cmd(grade_pct, wind_mps)
            self.schedule_indication(opcode, 0x01)
            self.on_status_notify(bytes([0x12]) + raw[1:])
        else:
            self.schedule_indication(opcode, 0x02)

    def schedule_indication(self, request_opcode: int, result_code: int):
        """
        Schedules the 0x80 FTMS Control Point response indication 15ms after
        WriteValue returns so the ATT Write Response completes first on Android.
        """
        response = bytes([0x80, request_opcode & 0xFF, result_code & 0xFF])
        self._ind_queue.append(response)
        if not self._ind_busy:
            self._ind_busy = True
            GLib.timeout_add(15, self._drain_indications)

    def _drain_indications(self) -> bool:
        if not self._ind_queue:
            self._ind_busy = False
            return False
        resp = self._ind_queue.pop(0)
        self.value = [dbus.Byte(b) for b in resp]
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {"Value": dbus.Array(self.value, signature="y")},
            [],
        )
        return True


class WatchedNotifyCharacteristic(GattCharacteristic):
    def __init__(
        self,
        bus: dbus.Bus,
        index: int,
        uuid: str,
        service_path: str,
        on_subscription_change: Callable[[bool], None],
    ):
        super().__init__(bus, index, uuid, ["notify"], service_path)
        self.on_subscription_change = on_subscription_change

    def _on_start_notify(self):
        self.on_subscription_change(True)

    def _on_stop_notify(self):
        self.on_subscription_change(False)


class FtmsGattApplication(dbus.service.Object):
    """BlueZ ObjectManager root exposing FTMS 0x1826 and Cycling Power 0x1818 services."""

    PATH = "/com/domyos/bridge"
    SERVICE_PATH = "/com/domyos/bridge/service0"
    CPS_SERVICE_PATH = "/com/domyos/bridge/service1"

    def __init__(
        self,
        bus: dbus.Bus,
        on_resistance_cmd: Callable[[int], None],
        on_simulation_cmd: Callable[[float, float], None],
        on_target_power_cmd: Callable[[int], None],
        on_subscription_change: Callable[[bool], None],
    ):
        self.path = self.PATH
        self.bus = bus
        super().__init__(bus, self.path)

        self.service_obj = FtmsServiceObject(bus, self.SERVICE_PATH, FTMS_SERVICE_UUID)
        self.cps_service_obj = FtmsServiceObject(bus, self.CPS_SERVICE_PATH, CPS_SERVICE_UUID)
        self.characteristics: List[GattCharacteristic] = []
        self._cum_crank_revs: int = 0
        self._last_crank_event_1024: int = 0
        self._crank_frac: float = 0.0

        # FTMS 0x2ACC: Bit 1 Cadence, Bit 7 Resistance Level, Bit 14 Power Measurement
        machine_features = (1 << 1) | (1 << 7) | (1 << 14)
        target_features = (1 << 2) | (1 << 3) | (1 << 13)
        feature_char = GattCharacteristic(
            bus, 0, FTMS_FEATURE_UUID, ["read"], self.SERVICE_PATH
        )
        feature_char.value = [
            dbus.Byte(b)
            for b in struct.pack("<II", machine_features, target_features)
        ]

        res_range_char = GattCharacteristic(
            bus, 1, SUPPORTED_RES_RANGE_UUID, ["read"], self.SERVICE_PATH
        )
        res_range_char.value = [
            dbus.Byte(b) for b in struct.pack("<hhh", 10, 320, 10)
        ]

        pwr_range_char = GattCharacteristic(
            bus, 2, SUPPORTED_POWER_RANGE_UUID, ["read"], self.SERVICE_PATH
        )
        pwr_range_char.value = [
            dbus.Byte(b) for b in struct.pack("<hhh", 0, 1500, 1)
        ]

        self.bike_data_char = WatchedNotifyCharacteristic(
            bus,
            3,
            INDOOR_BIKE_DATA_UUID,
            self.SERVICE_PATH,
            on_subscription_change,
        )

        self.status_char = GattCharacteristic(
            bus, 4, FTMS_STATUS_UUID, ["notify"], self.SERVICE_PATH
        )

        self.control_point_char = FtmsControlPointCharacteristic(
            bus=bus,
            index=5,
            service_path=self.SERVICE_PATH,
            on_resistance_cmd=on_resistance_cmd,
            on_simulation_cmd=on_simulation_cmd,
            on_target_power_cmd=on_target_power_cmd,
            on_status_notify=self.status_char.send_notification,
            on_subscription_change=on_subscription_change,
        )

        # Cycling Power Service (0x1818) characteristics for MyWhoosh Power/Cadence slots
        cps_feature_char = GattCharacteristic(
            bus, 0, CPS_FEATURE_UUID, ["read"], self.CPS_SERVICE_PATH
        )
        # Bit 3 (0x08): Crank Revolution Data Supported
        cps_feature_char.value = [dbus.Byte(b) for b in struct.pack("<I", 0x00000008)]

        cps_loc_char = GattCharacteristic(
            bus, 1, CPS_SENSOR_LOCATION_UUID, ["read"], self.CPS_SERVICE_PATH
        )
        cps_loc_char.value = [dbus.Byte(13)]  # 13 = Rear Hub

        self.cps_meas_char = WatchedNotifyCharacteristic(
            bus,
            2,
            CPS_MEASUREMENT_UUID,
            self.CPS_SERVICE_PATH,
            on_subscription_change,
        )

        self.characteristics.extend(
            [
                feature_char,
                res_range_char,
                pwr_range_char,
                self.bike_data_char,
                self.status_char,
                self.control_point_char,
                cps_feature_char,
                cps_loc_char,
                self.cps_meas_char,
            ]
        )

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {
            self.service_obj.get_path(): self.service_obj.get_properties(),
            self.cps_service_obj.get_path(): self.cps_service_obj.get_properties(),
        }
        for chrc in self.characteristics:
            response[chrc.get_path()] = chrc.get_properties()
        return response

    def broadcast_telemetry(
        self,
        speed_kmh: float,
        cadence_rpm: int,
        power_watts: int,
    ):
        # 1. FTMS Indoor Bike Data (0x2AD2):
        # Flags = 0x0044 (Bit 0=0 -> Inst Speed [uint16]; Bit 2=1 -> Inst Cadence [uint16];
        # Bit 6=1 -> Inst Power [sint16]). Omitting Bit 5 (Resistance Level) avoids the
        # FTMS spec discrepancy (uint8 vs sint16) that shifts Power bytes in MyWhoosh.
        flags = 0x0044
        speed_raw = max(0, min(65535, int(round(max(0.0, speed_kmh) * 100.0))))
        cadence_raw = max(0, min(65535, int(max(0, cadence_rpm) * 2)))
        power_raw = max(0, min(4000, int(power_watts)))

        bike_payload = struct.pack(
            "<HHHh",
            flags,
            speed_raw,
            cadence_raw,
            power_raw,
        )
        self.bike_data_char.send_notification(bike_payload)

        # 2. Cycling Power Measurement (0x2A63 in Service 0x1818):
        # Flags = 0x0020 (Bit 5: Crank Revolution Data Present)
        if cadence_rpm > 0:
            revs_per_tick = (cadence_rpm / 60.0) * 0.25
            self._crank_frac += revs_per_tick
            whole_revs = int(self._crank_frac)
            if whole_revs > 0:
                self._crank_frac -= whole_revs
                self._cum_crank_revs = (self._cum_crank_revs + whole_revs) & 0xFFFF
                sec_per_rev = 60.0 / cadence_rpm
                dt_1024 = int(round(whole_revs * sec_per_rev * 1024.0))
                self._last_crank_event_1024 = (self._last_crank_event_1024 + dt_1024) & 0xFFFF

        cps_payload = struct.pack(
            "<hhHH",
            0x0020,
            power_raw,
            self._cum_crank_revs,
            self._last_crank_event_1024,
        )
        self.cps_meas_char.send_notification(cps_payload)


class FtmsServiceObject(dbus.service.Object):
    def __init__(self, bus: dbus.Bus, path: str, uuid: str = FTMS_SERVICE_UUID):
        self.path = path
        self.uuid = uuid
        super().__init__(bus, self.path)

    def get_path(self) -> dbus.ObjectPath:
        return dbus.ObjectPath(self.path)

    def get_properties(self) -> Dict[str, Dict[str, object]]:
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": dbus.Boolean(True),
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface: str):
        if interface != GATT_SERVICE_IFACE:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                f"Unknown interface {interface}",
            )
        return self.get_properties()[GATT_SERVICE_IFACE]


# ---------------------------------------------------------------------------
# BlueZ BLE Central Engine (Laptop -> Domyos Bike 900 Hardware)
# ---------------------------------------------------------------------------
class DomyosCentralClient:
    """Discovers and controls physical Domyos Bike 900 hardware over BlueZ D-Bus."""

    def __init__(self, bus: dbus.Bus, adapter_path: str, state: BridgeState):
        self.bus = bus
        self.adapter_path = adapter_path
        self.state = state
        self.adapter = dbus.Interface(
            bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), ADAPTER_IFACE
        )
        self.obj_manager = dbus.Interface(
            bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE
        )

        self.device_path: Optional[str] = None
        self.ftms_cp_path: Optional[str] = None
        self.prop_write_path: Optional[str] = None
        self.ftms_res_multiplier: int = 1
        self.ftms_use_1byte_res: bool = False
        self.last_sent_level: int = -1
        self.is_connecting: bool = False
        self._gatt_ready: bool = False
        self._uuid_by_path: Dict[str, str] = {}
        self._pending_level: int = -1
        self._debounce_id: int = 0
        self._res_fmt_retried: bool = False
        self._poll_count: int = 0
        self._connect_time: float = time.monotonic()
        self._rx_partial: bytes = b""
        self._last_ftms_write: float = 0.0
        self._ftms_backoff_until: float = 0.0

        self.bus.add_signal_receiver(
            self._on_interfaces_added,
            dbus_interface=DBUS_OM_IFACE,
            signal_name="InterfacesAdded",
        )
        self.bus.add_signal_receiver(
            self._on_properties_changed,
            dbus_interface=DBUS_PROP_IFACE,
            signal_name="PropertiesChanged",
            path_keyword="path",
        )

    def start_scanning(self):
        try:
            self.adapter.SetDiscoveryFilter(
                dbus.Dictionary({"Transport": "le"}, signature="sv")
            )
        except Exception:
            pass
        try:
            self.adapter.StartDiscovery()
            self.state.central_status = "Scanning BLE for Domyos Bike 900..."
        except Exception as exc:
            if "InProgress" in str(exc):
                self.state.central_status = "Scanning BLE for Domyos Bike 900..."
            else:
                self.state.central_status = f"Scan note: {exc}"

        self._check_existing_devices()

    def stop(self):
        try:
            self.adapter.StopDiscovery()
        except Exception:
            pass
        if self.device_path:
            try:
                dev = dbus.Interface(
                    self.bus.get_object(BLUEZ_SERVICE_NAME, self.device_path),
                    DEVICE_IFACE,
                )
                dev.Disconnect()
            except Exception:
                pass

    def periodic_scan_sweep(self):
        """Called periodically to catch devices whose Name/UUIDs resolved after initial scan."""
        if not self.device_path and not self.is_connecting and not self.state.simulation_mode:
            self._check_existing_devices()

    def _check_existing_devices(self):
        try:
            objects = self.obj_manager.GetManagedObjects()
            for path, ifaces in objects.items():
                if DEVICE_IFACE in ifaces:
                    self._evaluate_candidate_device(str(path), ifaces[DEVICE_IFACE])
        except Exception:
            pass

    def _on_interfaces_added(self, path, interfaces):
        if DEVICE_IFACE in interfaces:
            self._evaluate_candidate_device(str(path), interfaces[DEVICE_IFACE])

    def _evaluate_candidate_device(self, path: str, props: dict):
        if self.device_path or self.is_connecting or self.state.simulation_mode:
            return
        if not path.startswith(self.adapter_path):
            return

        name = str(props.get("Name") or props.get("Alias") or "").strip()
        if name.upper() == "DOMYOSBRIDGE":
            return

        uuids = [str(u).lower() for u in props.get("UUIDs", [])]
        name_upper = name.upper()
        matches_name = any(
            name_upper.startswith(p) or "DOMYOS" in name_upper
            for p in TARGET_NAME_PREFIXES
        )
        matches_uuid = (
            DOMYOS_ISSC_SERVICE_UUID in uuids
            or DOMYOS_FFF0_SERVICE_UUID in uuids
            or (FTMS_SERVICE_UUID in uuids and bool(name))
        )

        if matches_name or matches_uuid:
            self.is_connecting = True
            self.device_path = path
            self.state.domyos_name = name or path.split("/")[-1]
            self.state.domyos_rssi = int(props["RSSI"]) if "RSSI" in props else None
            self.state.central_status = f"Connecting to {self.state.domyos_name}..."
            self.state.last_event = f"Detected {self.state.domyos_name} — Connecting..."
            GLib.idle_add(self._connect_device, path)

    def _connect_device(self, path: str) -> bool:
        try:
            dev = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE_NAME, path), DEVICE_IFACE
            )
            dev.Connect(
                reply_handler=self._on_connect_success,
                error_handler=self._on_connect_error,
            )
        except Exception as exc:
            self._on_connect_error(exc)
        return False

    def _on_connect_success(self):
        self.is_connecting = False
        self.state.central_status = "Connected (Resolving GATT Services...)"
        try:
            props = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE_NAME, self.device_path, introspect=False),
                DBUS_PROP_IFACE,
            )
            if bool(props.Get(DEVICE_IFACE, "ServicesResolved")):
                self._setup_discovered_gatt()
        except Exception:
            pass

    def _reset_telemetry(self):
        s = self.state
        s.speed_kmh = 0.0
        s.cadence_rpm = 0
        s.power_watts = 0

    def _on_connect_error(self, error):
        self.is_connecting = False
        self.state.central_status = f"Re-scanning ({error})"
        self.device_path = None
        GLib.timeout_add_seconds(2, self._retry_scan)

    def _retry_scan(self) -> bool:
        if not self.device_path and not self.is_connecting and not self.state.simulation_mode:
            self.start_scanning()
        return False

    def _setup_discovered_gatt(self):
        if not self.device_path or self._gatt_ready:
            return
        prefix = self.device_path + "/"
        objects = self.obj_manager.GetManagedObjects()
        found_any = False

        for path, ifaces in objects.items():
            path_str = str(path)
            if not path_str.startswith(prefix):
                continue
            if GATT_CHRC_IFACE not in ifaces:
                continue

            found_any = True
            uuid = str(ifaces[GATT_CHRC_IFACE].get("UUID", "")).lower()
            self._uuid_by_path[path_str] = uuid
            chrc = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE_NAME, path_str), GATT_CHRC_IFACE
            )

            if uuid in (
                INDOOR_BIKE_DATA_UUID,
                FTMS_STATUS_UUID,
                DOMYOS_ISSC_NOTIFY_UUID,
                DOMYOS_FFF1_NOTIFY_UUID,
            ):
                try:
                    chrc.StartNotify(
                        reply_handler=lambda: None,
                        error_handler=lambda _: None,
                    )
                except Exception:
                    pass

            if uuid == FTMS_CONTROL_POINT_UUID and self.state.protocol_mode in ("auto", "ftms", "both"):
                self.ftms_cp_path = path_str
                self.state.domyos_protocol = (
                    "BOTH (FTMS + UART)" if (self.prop_write_path and self.state.protocol_mode == "both") else "FTMS 0x1826"
                )
                self.state.central_status = f"Connected & Ready ({self.state.domyos_protocol})"
                try:
                    chrc.StartNotify(reply_handler=lambda: None, error_handler=lambda _: None)
                    # Request Control (0x00) & Start/Resume (0x07)
                    self._write_characteristic(path_str, bytes([0x00]))
                    self._write_characteristic(path_str, bytes([0x07]))
                except Exception:
                    pass

            elif uuid in (DOMYOS_ISSC_WRITE_UUID, DOMYOS_FFF2_WRITE_UUID) and self.state.protocol_mode in ("auto", "uart", "both"):
                self.prop_write_path = path_str
                if self.ftms_cp_path and self.state.protocol_mode == "both":
                    self.state.domyos_protocol = "BOTH (FTMS + UART)"
                    self.state.central_status = "Connected & Ready (BOTH: FTMS + UART)"
                elif not self.ftms_cp_path:
                    self.state.domyos_protocol = "Domyos UART"
                    self.state.central_status = "Connected & Ready (Domyos UART)"
                for idx, init_frame in enumerate(DOMYOS_INIT_SEQUENCE):
                    GLib.timeout_add(
                        350 * (idx + 1),
                        self._delayed_write,
                        path_str,
                        init_frame,
                    )

        if found_any:
            self._gatt_ready = True
            self._connect_time = time.monotonic()
            self._rx_partial = b""
            self._poll_count = 0
            try:
                self.adapter.StopDiscovery()
            except Exception:
                pass
            self.last_sent_level = -1
            GLib.timeout_add(350 * (len(DOMYOS_INIT_SEQUENCE) + 2), self._send_initial_level)

    def _delayed_write(self, path_str: str, frame: bytes) -> bool:
        if path_str == self.prop_write_path:
            self._write_uart_frame(path_str, frame)
        return False

    def _write_uart_frame(self, path_str: str, frame: bytes):
        """Domyos UART frames are split into 20-byte writes (like QZ) to fit the ATT MTU."""
        for i in range(0, len(frame), DOMYOS_ATT_CHUNK):
            self._write_characteristic(path_str, frame[i:i + DOMYOS_ATT_CHUNK])

    def _send_initial_level(self) -> bool:
        self.send_target_resistance(self.state.target_level, force=True)
        return False

    def send_target_resistance(self, level: int, force: bool = False):
        clamped = max(1, min(self.state.max_level, int(level)))
        self.state.target_level = clamped
        if self.state.simulation_mode:
            self.state.current_level = clamped
            return

        if not force and clamped == self.last_sent_level:
            return
        self._pending_level = clamped
        if self._debounce_id == 0:
            self._debounce_id = GLib.timeout_add(120, self._flush_level)

    def _flush_level(self) -> bool:
        self._debounce_id = 0
        clamped = self._pending_level
        if clamped < 1 or (not self.ftms_cp_path and not self.prop_write_path):
            return False

        use_ftms = bool(self.ftms_cp_path) and (
            self.state.protocol_mode in ("auto", "ftms", "both")
        )
        use_uart = bool(self.prop_write_path) and (
            self.state.protocol_mode in ("uart", "both")
            or (self.state.protocol_mode == "auto" and not self.ftms_cp_path)
        )

        if use_uart:
            self.last_sent_level = clamped
            # QZ forceResistance(): single 23-byte 0xF0 0xAD frame, level in byte 10
            self._write_uart_frame(
                self.prop_write_path, build_domyos_23byte_ad_frame(clamped)
            )

        if use_ftms:
            # QZ ftmsbike (DOMYOS): rate-limited to one command per 1.5 s, with a
            # 3 s back-off after CONTROL_NOT_PERMITTED.
            now = time.monotonic()
            wait = max(
                self._last_ftms_write + FTMS_MIN_WRITE_INTERVAL_S - now,
                self._ftms_backoff_until - now,
            )
            if wait > 0:
                self._debounce_id = GLib.timeout_add(int(wait * 1000) + 20, self._flush_level)
                return False
            self._last_ftms_write = now
            self.last_sent_level = clamped
            # QZ forceResistance(): {0x04, level} with the raw 1..max_level level as uint8
            self._write_characteristic(self.ftms_cp_path, bytes([0x04, clamped]))
        return False

    def poll_proprietary_console(self):
        """
        QZ update loop: a 0xF0 0xAC 0x9C no-op every cycle, replaced by the 0xF0 0xCB
        display frame once per second. Called every 250 ms; the console answers with
        its 26-byte 0xF0 0xBC telemetry frames.
        """
        use_uart = bool(self.prop_write_path) and (
            self.state.protocol_mode in ("uart", "both")
            or (self.state.protocol_mode == "auto" and not self.ftms_cp_path)
        )
        if not (use_uart and self._gatt_ready):
            return
        self._poll_count += 1
        if self._poll_count % 4 == 0:
            elapsed = int(time.monotonic() - self._connect_time)
            frame = build_domyos_display_frame(elapsed)
        else:
            frame = DOMYOS_NOOP_FRAME
        self._write_uart_frame(self.prop_write_path, frame)

    def _write_characteristic(self, char_path: str, payload: bytes, wtype: str = "request"):
        try:
            chrc = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE_NAME, char_path, introspect=False),
                GATT_CHRC_IFACE,
            )
            chrc.WriteValue(
                dbus.Array([dbus.Byte(b) for b in payload], signature="y"),
                dbus.Dictionary({"type": wtype}, signature="sv"),
                reply_handler=lambda: None,
                error_handler=self._on_write_error,
            )
        except Exception:
            self.last_sent_level = -1

    def _on_write_error(self, _err):
        self.last_sent_level = -1

    def refresh_target(self):
        """Periodically re-assert the target level (recovers from dropped writes)."""
        if self.device_path and self._gatt_ready and not self.state.simulation_mode:
            self.last_sent_level = -1
            self.send_target_resistance(self.state.target_level)

    def _on_properties_changed(self, interface, changed, invalidated, path=None):
        if not path:
            return
        path_str = str(path)

        if not self.device_path and interface == DEVICE_IFACE:
            if "Name" in changed or "Alias" in changed or "UUIDs" in changed:
                try:
                    prop_iface = dbus.Interface(
                        self.bus.get_object(BLUEZ_SERVICE_NAME, path_str),
                        DBUS_PROP_IFACE,
                    )
                    all_props = prop_iface.GetAll(DEVICE_IFACE)
                    self._evaluate_candidate_device(path_str, all_props)
                except Exception:
                    self._evaluate_candidate_device(path_str, changed)
            return

        if not self.device_path or not (
            path_str == self.device_path or path_str.startswith(self.device_path + "/")
        ):
            return

        if interface == DEVICE_IFACE:
            if "RSSI" in changed:
                self.state.domyos_rssi = int(changed["RSSI"])
            if "ServicesResolved" in changed:
                if bool(changed["ServicesResolved"]):
                    self._setup_discovered_gatt()
                else:
                    self._gatt_ready = False
            if "Connected" in changed and not bool(changed["Connected"]):
                self.state.central_status = "Disconnected — Re-scanning..."
                self._gatt_ready = False
                self._uuid_by_path.clear()
                self._res_fmt_retried = False
                self._reset_telemetry()
                self.device_path = None
                self.ftms_cp_path = None
                self.prop_write_path = None
                self.last_sent_level = -1
                GLib.timeout_add_seconds(2, self._retry_scan)

        elif interface == GATT_CHRC_IFACE and "Value" in changed:
            raw = bytes(int(b) for b in changed["Value"])
            uuid = self._uuid_by_path.get(path_str)
            if not uuid:
                return

            if uuid == INDOOR_BIKE_DATA_UUID:
                self._parse_ftms_bike_data(raw)
            elif uuid == FTMS_CONTROL_POINT_UUID:
                if len(raw) >= 3 and raw[0] == 0x80 and raw[1] == 0x04:
                    if raw[2] == 0x05:
                        # CONTROL_NOT_PERMITTED: back off 3 s, then request control again (QZ)
                        self._ftms_backoff_until = time.monotonic() + 3.0
                        self.last_sent_level = -1
                        if self.ftms_cp_path:
                            self._write_characteristic(self.ftms_cp_path, bytes([0x00]))
                            self._write_characteristic(self.ftms_cp_path, bytes([0x07]))
                    elif raw[2] == 0x01:
                        self._ftms_backoff_until = 0.0
            elif uuid in (DOMYOS_ISSC_NOTIFY_UUID, DOMYOS_FFF1_NOTIFY_UUID):
                self._parse_domyos_proprietary(raw)

    def _parse_ftms_bike_data(self, data: bytes):
        if len(data) < 2:
            return
        (flags,) = struct.unpack_from("<H", data, 0)
        offset = 2

        if (flags & (1 << 0)) == 0 and len(data) >= offset + 2:
            (raw_speed,) = struct.unpack_from("<H", data, offset)
            self.state.speed_kmh = raw_speed / 100.0
            offset += 2
        if (flags & (1 << 1)) != 0 and len(data) >= offset + 2:
            offset += 2
        if (flags & (1 << 2)) != 0 and len(data) >= offset + 2:
            (raw_cad,) = struct.unpack_from("<H", data, offset)
            self.state.cadence_rpm = int(round(raw_cad / 2.0))
            offset += 2
        if (flags & (1 << 3)) != 0 and len(data) >= offset + 2:
            offset += 2
        if (flags & (1 << 4)) != 0 and len(data) >= offset + 3:
            offset += 3
        # QZ (ftmsbike, DOMYOS): the Domyos FTMS bike sends neither power nor a usable
        # resistance level, so mirror the last commanded level and estimate power.
        self.state.current_level = self.state.target_level
        self.state.power_watts = estimate_domyos_watts(
            self.state.cadence_rpm, self.state.current_level, self.state.max_level
        )

    def _parse_domyos_proprietary(self, data: bytes):
        """
        Reassembles and parses the 26-byte 0xF0 0xBC status frame (QZ characteristicChanged).
        Consoles split it into a 20-byte notification plus a 6-byte tail.
        Offsets (QZ): speed = bytes 6-7 / 10 km/h, cadence = byte 9, resistance = byte 14.
        Bytes 10-11 are kcal and 12-13 distance, so power is always estimated.
        """
        if (
            len(self._rx_partial) == 20
            and self._rx_partial[:2] == b"\xF0\xBC"
            and len(data) == 6
        ):
            data = self._rx_partial + data
        self._rx_partial = data if len(data) == 20 else b""

        if len(data) != 26 or data[0] != 0xF0 or data[1] != 0xBC:
            return
        raw_speed = (data[6] << 8) | data[7]
        self.state.speed_kmh = max(0.0, min(80.0, raw_speed / 10.0))
        self.state.cadence_rpm = max(0, min(240, data[9]))
        raw_res = data[14]
        self.state.current_level = max(1, min(self.state.max_level, raw_res))
        self.state.power_watts = estimate_domyos_watts(
            self.state.cadence_rpm, self.state.current_level, self.state.max_level
        )


# ---------------------------------------------------------------------------
# Bridge Orchestrator & Interactive Terminal + Mouse Wheel Shifter UI
# ---------------------------------------------------------------------------
class DomyosLinuxBridge:
    def __init__(
        self,
        simulate: bool = False,
        protocol: str = "auto",
        max_level: int = 32,
    ):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.adapter_path = self._find_adapter_path()
        base_gear = 8 if max_level == 15 else 18
        self.state = BridgeState(
            simulation_mode=simulate,
            protocol_mode=protocol,
            max_level=max_level,
            base_gear=base_gear,
            virtual_gear=base_gear,
            max_virtual_gear=max_level,
            current_level=base_gear,
            target_level=base_gear,
            last_event=f"Ready (Mode: {protocol.upper()}, Base Lvl {base_gear}/{max_level}) — Scroll Mouse Wheel to shift",
        )
        if simulate:
            self.state.central_status = "SIMULATION MODE (Virtual Rider)"
            self.state.domyos_name = "Simulated Domyos Bike 900"
            self.state.domyos_protocol = "Simulated"
            self.state.current_level = self.state.target_level

        self._ensure_adapter_powered_and_named()

        self.gatt_app = FtmsGattApplication(
            bus=self.bus,
            on_resistance_cmd=self.on_mywhoosh_resistance,
            on_simulation_cmd=self.on_mywhoosh_simulation_grade,
            on_target_power_cmd=self.on_mywhoosh_target_power,
            on_subscription_change=self.on_mywhoosh_subscription,
        )
        self.advertisement = FtmsAdvertisement(self.bus)
        self.central = DomyosCentralClient(self.bus, self.adapter_path, self.state)
        self.mainloop = GLib.MainLoop()
        self._sim_phase = 0.0
        self._evdev_fds: List[int] = []
        self._rxbuf: str = ""
        self._tick_count: int = 0
        self._sub_count: int = 0

    def _find_adapter_path(self) -> str:
        om = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE
        )
        for path, ifaces in om.GetManagedObjects().items():
            if ADAPTER_IFACE in ifaces:
                return str(path)
        raise RuntimeError(
            "No Bluetooth adapter found via BlueZ. Ensure Bluetooth is enabled."
        )

    def _ensure_adapter_powered_and_named(self):
        props = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter_path),
            DBUS_PROP_IFACE,
        )
        try:
            props.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
            props.Set(ADAPTER_IFACE, "Alias", dbus.String("DomyosBridge"))
            props.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(True))
        except Exception:
            pass

    def register_peripheral(self):
        gatt_mgr = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter_path),
            GATT_MANAGER_IFACE,
        )
        ad_mgr = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter_path),
            LE_ADVERTISING_MANAGER_IFACE,
        )

        gatt_mgr.RegisterApplication(
            self.gatt_app.get_path(),
            dbus.Dictionary({}, signature="sv"),
            reply_handler=self._on_gatt_registered,
            error_handler=self._on_peripheral_error,
        )
        ad_mgr.RegisterAdvertisement(
            self.advertisement.get_path(),
            dbus.Dictionary({}, signature="sv"),
            reply_handler=self._on_ad_registered,
            error_handler=self._on_peripheral_error,
        )

    def _on_gatt_registered(self):
        self.state.last_event = "FTMS 0x1826 + CPS 0x1818 GATT Server registered"

    def _on_ad_registered(self):
        self.state.peripheral_status = "Advertising 'DomyosBridge' (FTMS 0x1826)"

    def _on_peripheral_error(self, error):
        self.state.peripheral_status = f"Peripheral error: {error}"

    def on_mywhoosh_subscription(self, subscribed: bool):
        # Several characteristics share this callback: count active subscriptions.
        if subscribed:
            self._sub_count += 1
        else:
            self._sub_count = max(0, self._sub_count - 1)
        subscribed = self._sub_count > 0
        self.state.mywhoosh_subscribed = subscribed
        self.state.peripheral_status = (
            "MyWhoosh Connected & Subscribed"
            if subscribed
            else "Advertising 'DomyosBridge' (FTMS 0x1826)"
        )
        self.state.last_event = (
            "MyWhoosh subscribed to telemetry stream"
            if subscribed
            else "MyWhoosh unsubscribed"
        )

    def shift_virtual_gear(self, delta: int, source_label: str = "Gear Shift"):
        """
        Shifts the QZ-style Virtual Gear (1..32) up or down and immediately
        recomputes the physical Domyos 1..32 resistance step while preserving
        active MyWhoosh automatic route slope/ERG tracking.
        """
        old_gear = self.state.virtual_gear
        new_gear = max(1, min(self.state.max_virtual_gear, old_gear + delta))
        self.state.virtual_gear = new_gear
        arrow = "▲ UP" if delta > 0 else "▼ DOWN"

        if self.state.control_mode.startswith("ERG") and self.state.last_erg_watts > 0:
            gear_bias = new_gear - self.state.base_gear
            cadence = self.state.cadence_rpm if self.state.cadence_rpm >= 35 else 80
            base_lvl = min(
                range(1, self.state.max_level + 1),
                key=lambda lvl: abs(
                    estimate_domyos_watts(cadence, lvl, self.state.max_level) - self.state.last_erg_watts
                ),
            )
            target = max(1, min(self.state.max_level, base_lvl + gear_bias))
            self.central.send_target_resistance(target)
        else:
            self._recalculate_slope_resistance()

        self.state.last_event = (
            f"{source_label} {arrow}: Virtual Gear {new_gear}/{self.state.max_virtual_gear} "
            f"(Slope {self.state.slope_percent:+.1f}%) -> Domyos Lvl {self.state.target_level}"
        )

    def _recalculate_slope_resistance(self):
        grade_pct = self.state.slope_percent
        factor = (
            self.state.slope_sensitivity
            if grade_pct >= 0.0
            else self.state.slope_sensitivity * 0.6
        )
        computed = int(round(self.state.virtual_gear + (grade_pct * factor)))
        clamped = max(1, min(self.state.max_level, computed))
        self.central.send_target_resistance(clamped)

    def on_mywhoosh_resistance(self, level: int):
        self.state.control_mode = "DIRECT (0x04)"
        gear_offset = self.state.virtual_gear - self.state.base_gear
        clamped = max(1, min(self.state.max_level, level + gear_offset))
        self.state.last_event = (
            f"MyWhoosh [0x04] Resistance {level} (Gear {self.state.virtual_gear}) -> Lvl {clamped}"
        )
        self.central.send_target_resistance(clamped)

    def on_mywhoosh_simulation_grade(self, grade_pct: float, wind_mps: float):
        self.state.control_mode = "SIM (Auto Route Slope + Mouse Gear)"
        self.state.slope_percent = grade_pct
        self.state.wind_speed_mps = wind_mps
        self._recalculate_slope_resistance()
        self.state.last_event = (
            f"Auto Route Slope {grade_pct:+.1f}% + Gear {self.state.virtual_gear} "
            f"-> Domyos Lvl {self.state.target_level}"
        )

    def on_mywhoosh_target_power(self, target_watts: int):
        self.state.control_mode = f"ERG ({target_watts}W + Mouse Bias)"
        self.state.last_erg_watts = target_watts
        cadence = self.state.cadence_rpm if self.state.cadence_rpm >= 35 else 80
        base_lvl = min(
            range(1, self.state.max_level + 1),
            key=lambda lvl: abs(estimate_domyos_watts(cadence, lvl, self.state.max_level) - target_watts),
        )
        gear_bias = self.state.virtual_gear - self.state.base_gear
        final_lvl = max(1, min(self.state.max_level, base_lvl + gear_bias))
        self.state.last_event = (
            f"MyWhoosh [0x05] ERG {target_watts}W (Gear {self.state.virtual_gear}) -> Domyos Lvl {final_lvl}"
        )
        self.central.send_target_resistance(final_lvl)

    def _on_250ms_tick(self) -> bool:
        if self.state.simulation_mode:
            self._sim_phase += 0.20
            self.state.current_level = self.state.target_level
            self.state.cadence_rpm = int(round(82 + 4 * math.sin(self._sim_phase)))
            self.state.power_watts = estimate_domyos_watts(
                self.state.cadence_rpm, self.state.current_level, self.state.max_level
            )
            self.state.speed_kmh = round(
                18.0 + (self.state.power_watts * 0.065), 1
            )

        if not self.state.simulation_mode:
            self.central.poll_proprietary_console()

        self.gatt_app.broadcast_telemetry(
            speed_kmh=self.state.speed_kmh,
            cadence_rpm=self.state.cadence_rpm,
            power_watts=self.state.power_watts,
        )
        self._render_dashboard()
        return True

    def _on_1000ms_tick(self) -> bool:
        if not self.state.simulation_mode:
            self.central.periodic_scan_sweep()
            self._tick_count += 1
            if self._tick_count % 5 == 0:
                self.central.refresh_target()
        return True

    def _setup_global_evdev_mouse(self):
        """
        Optionally attaches directly to /dev/input/by-id/*-event-mouse if readable
        so mouse wheel shifting works globally even when the terminal is unfocused.
        """
        candidates = sorted(
            set(
                glob.glob("/dev/input/by-id/*-event-mouse")
                + glob.glob("/dev/input/by-path/*-event-mouse")
            )
        )
        for dev_path in candidates:
            if not os.access(dev_path, os.R_OK):
                continue
            try:
                fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
                self._evdev_fds.append(fd)
                GLib.io_add_watch(fd, GLib.IO_IN, self._handle_evdev_mouse)
                self.state.global_mouse_attached = True
            except Exception:
                pass

    def _handle_evdev_mouse(self, fd: int, condition: int) -> bool:
        try:
            data = os.read(fd, EVDEV_EVENT_STRUCT.size * 16)
        except BlockingIOError:
            return True
        except Exception:
            try:
                self._evdev_fds.remove(fd)
                os.close(fd)
            except Exception:
                pass
            self.state.global_mouse_attached = bool(self._evdev_fds)
            return False

        offset = 0
        while offset + EVDEV_EVENT_STRUCT.size <= len(data):
            _, _, ev_type, ev_code, ev_val = EVDEV_EVENT_STRUCT.unpack_from(data, offset)
            offset += EVDEV_EVENT_STRUCT.size
            if ev_type == EV_REL and ev_code == REL_WHEEL and ev_val != 0:
                self.shift_virtual_gear(1 if ev_val > 0 else -1, "Global Mouse Wheel")
        return True

    def _on_signal(self) -> bool:
        self.mainloop.quit()
        return False

    def _handle_stdin_input(self, fd: int, condition: int) -> bool:
        try:
            raw = os.read(fd, 64)
        except BlockingIOError:
            return True
        if not raw:
            return True

        self._rxbuf = (self._rxbuf + raw.decode("latin-1", errors="ignore"))[-256:]
        text = self._rxbuf
        if len(raw) == 64 and (
            text.endswith(("\x1b", "\x1b[", "\x1b[<"))
            or re.search(r"\x1b\[<[\d;]*$", text)
        ):
            return True
        self._rxbuf = ""
        text = re.sub(r"\x1b(?!\[)", "", text)
        if not text:
            return True
        raw = text.encode("latin-1", errors="ignore")

        # 1. Check SGR mouse events (\033[<Btn;X;Y;M or m)
        sgr_matches = list(SGR_MOUSE_RE.finditer(text))
        if sgr_matches:
            for match in sgr_matches:
                btn = int(match.group(1)) & ~(4 | 8 | 16)
                action = match.group(4)
                if btn == 64:
                    if not self.state.global_mouse_attached:
                        self.shift_virtual_gear(+1, "Mouse Wheel")
                elif btn == 65:
                    if not self.state.global_mouse_attached:
                        self.shift_virtual_gear(-1, "Mouse Wheel")
                elif btn == 0 and action == "M":
                    self.shift_virtual_gear(-1, "Left Click")
                elif btn == 2 and action == "M":
                    self.shift_virtual_gear(+1, "Right Click")
                elif btn == 1 and action == "M":
                    self.state.virtual_gear = self.state.base_gear
                    self._recalculate_slope_resistance()
                    self.state.last_event = (
                        f"Middle Click: Reset Virtual Gear to {self.state.base_gear} -> Lvl {self.state.target_level}"
                    )
            return True

        # 2. Check legacy X10 mouse events (\033[M + 3 bytes)
        if raw.startswith(b"\x1b[M") and len(raw) >= 6:
            btn = raw[3]
            if btn == 96 and not self.state.global_mouse_attached:
                self.shift_virtual_gear(+1, "Mouse Wheel")
            elif btn == 97 and not self.state.global_mouse_attached:
                self.shift_virtual_gear(-1, "Mouse Wheel")
            return True

        # 3. Check Arrow Up / Down escape sequences
        if "\x1b[A" in text:
            self.shift_virtual_gear(+1, "Arrow Key")
            return True
        if "\x1b[B" in text:
            self.shift_virtual_gear(-1, "Arrow Key")
            return True

        # 4. Single-character hotkeys
        ch = text[0]
        if ch in ("q", "Q", "\x03"):
            self.shutdown()
            return False
        elif ch in ("k", "K", "]"):
            self.shift_virtual_gear(+1, "Keyboard")
        elif ch in ("j", "J", "["):
            self.shift_virtual_gear(-1, "Keyboard")
        elif ch in ("-", "_"):
            self.state.slope_sensitivity = max(
                0.25, round(self.state.slope_sensitivity - 0.25, 2)
            )
            self._recalculate_slope_resistance()
            self.state.last_event = (
                f"Slope Sensitivity -> +{self.state.slope_sensitivity:.2f} Lvl / 1%"
            )
        elif ch in ("=", "+"):
            self.state.slope_sensitivity = min(
                4.0, round(self.state.slope_sensitivity + 0.25, 2)
            )
            self._recalculate_slope_resistance()
            self.state.last_event = (
                f"Slope Sensitivity -> +{self.state.slope_sensitivity:.2f} Lvl / 1%"
            )
        elif ch in ("s", "S"):
            self.state.simulation_mode = not self.state.simulation_mode
            self.state.last_event = (
                "Switched to Simulation Rider Mode"
                if self.state.simulation_mode
                else "Switched to Live Hardware Mode"
            )
        elif ch in ("p", "P"):
            modes = ("auto", "ftms", "uart", "both")
            idx = (modes.index(self.state.protocol_mode) + 1) % len(modes)
            self.state.protocol_mode = modes[idx]
            self.central.last_sent_level = -1
            self.central.send_target_resistance(self.state.target_level, force=True)
            self.state.last_event = f"Switched Hardware Protocol Mode -> {self.state.protocol_mode.upper()}"
        return True

    def _render_dashboard(self):
        s = self.state
        rssi_str = f"{s.domyos_rssi} dBm" if s.domyos_rssi is not None else "N/A"
        lvl_0 = max(1, min(s.max_level, s.virtual_gear))
        lvl_5 = max(1, min(s.max_level, int(round(s.virtual_gear + 5 * s.slope_sensitivity))))
        lvl_10 = max(1, min(s.max_level, int(round(s.virtual_gear + 10 * s.slope_sensitivity))))

        gear_bar = "".join(
            "█" if g <= s.virtual_gear else "░"
            for g in range(1, s.max_virtual_gear + 1)
        )
        w80 = estimate_domyos_watts(80, s.current_level, s.max_level)

        lines = [
            "\033[H\033[J",
            "========================================================================",
            "  DOMYOS BIKE 900 BRIDGE (Linux BlueZ) • FTMS Virtual Smart Trainer",
            "========================================================================",
            f"  CENTRAL (Bike 900)   : {s.central_status}",
            f"  Hardware / Protocol  : {s.domyos_name} ({s.domyos_protocol} | Mode: {s.protocol_mode.upper()}) | RSSI: {rssi_str}",
            f"  PERIPHERAL (MyWhoosh): {s.peripheral_status}",
            f"  Active Control Mode  : {s.control_mode}",
            "------------------------------------------------------------------------",
            f"  VIRTUAL GEAR (Mouse) : Gear {s.virtual_gear:2d} / {s.max_virtual_gear}  [{gear_bar}]",
            f"  MYWHOOSH ROUTE SLOPE : {s.slope_percent:+5.1f}%         (Sensitivity: +{s.slope_sensitivity:.2f} Lvl / 1%)",
            f"  COMBINED TARGET LVL  : Level {s.target_level:2d} / {s.max_level}    (= Gear {s.virtual_gear} + Route Slope)",
            f"  CURRENT BIKE LEVEL   : Level {s.current_level:2d} / {s.max_level}",
            f"  CALIBRATED POWER     : {s.power_watts:4d} W           (80 RPM @ Lvl {s.current_level}: {w80} W)",
            f"  CADENCE / SPEED      : {s.cadence_rpm:4d} RPM         (Speed: {s.speed_kmh:4.1f} km/h)",
            "------------------------------------------------------------------------",
            f"  Auto Slope Preview   : 0% -> Lvl {lvl_0}  |  +5% -> Lvl {lvl_5}  |  +10% -> Lvl {lvl_10}",
            f"  Last Event           : {s.last_event}",
            "========================================================================",
            "  SHIFTER : Mouse Wheel UP/DOWN (or Right/Left Click, Up/Down Arrows)",
            "  HOTKEYS : [-/+] Slope Sens | [p] Cycle Protocol | [s] Sim Rider | [q] Quit",
        ]
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self):
        self.register_peripheral()
        if not self.state.simulation_mode:
            self.central.start_scanning()

        self._setup_global_evdev_mouse()
        GLib.timeout_add(250, self._on_250ms_tick)
        GLib.timeout_add(1000, self._on_1000ms_tick)

        fd = sys.stdin.fileno()
        old_settings = None
        old_flags = None
        if os.isatty(fd):
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)
            sys.stdout.write("\033[?1000h\033[?1006h")
            sys.stdout.flush()
            GLib.io_add_watch(fd, GLib.IO_IN, self._handle_stdin_input)

        try:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                GLib.unix_signal_add(GLib.PRIORITY_HIGH, sig, self._on_signal)
            self.mainloop.run()
        except KeyboardInterrupt:
            pass
        finally:
            if os.isatty(fd):
                sys.stdout.write("\033[?1006l\033[?1000l\n")
                sys.stdout.flush()
            if old_flags is not None:
                fcntl.fcntl(fd, fcntl.F_SETFL, old_flags)
            if old_settings is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            for ev_fd in self._evdev_fds:
                try:
                    os.close(ev_fd)
                except Exception:
                    pass
            self.shutdown()

    def shutdown(self):
        self.central.stop()
        try:
            ad_mgr = dbus.Interface(
                self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter_path),
                LE_ADVERTISING_MANAGER_IFACE,
            )
            ad_mgr.UnregisterAdvertisement(self.advertisement.get_path())
        except Exception:
            pass
        if self.mainloop.is_running():
            self.mainloop.quit()


def main():
    parser = argparse.ArgumentParser(
        description="Domyos Bike 900 to MyWhoosh FTMS BLE Bridge for Linux"
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Start in simulated rider mode to test MyWhoosh pairing without the bike turned on",
    )
    parser.add_argument(
        "--protocol",
        choices=("auto", "ftms", "uart", "both"),
        default="auto",
        help="Hardware control protocol: 'auto' (default), 'ftms' (0x1826 only), 'uart' (Domyos 0xF0 only), or 'both' (drive FTMS and UART simultaneously)",
    )
    parser.add_argument(
        "--max-level",
        type=int,
        choices=(15, 32),
        default=32,
        help="Maximum physical resistance level on the bike console (32 default for FTMS Domyos 900, 15 for older Changyow UART consoles)",
    )
    args = parser.parse_args()

    bridge = DomyosLinuxBridge(
        simulate=args.simulate,
        protocol=args.protocol,
        max_level=args.max_level,
    )
    bridge.run()


if __name__ == "__main__":
    main()
