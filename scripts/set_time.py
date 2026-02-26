# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "bleak>=2.1.1",
# ]
# ///
"""Set RTC time on M5Stack Core2 via BLE."""

import asyncio
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "M5Core2"
TIME_CHAR_UUID = "12345678-1234-5678-1234-56789abc0011"


async def main() -> None:
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if device is None:
        print(f"Device '{DEVICE_NAME}' not found. Is it advertising?")
        sys.exit(1)

    print(f"Found: {device.name} ({device.address})")

    async with BleakClient(device) as client:
        print(f"Connected (MTU={client.mtu_size})")

        now = datetime.now()
        # Protocol: [year-2000, month, day, weekday(0=Mon), hours, minutes, seconds]
        data = bytes([
            now.year - 2000,
            now.month,
            now.day,
            now.weekday(),  # 0=Monday in Python, matches ISO weekday
            now.hour,
            now.minute,
            now.second,
        ])

        await client.write_gatt_char(TIME_CHAR_UUID, data)
        print(f"Time set to {now.strftime('%Y-%m-%d %H:%M:%S')} (weekday={now.weekday()})")


if __name__ == "__main__":
    asyncio.run(main())
