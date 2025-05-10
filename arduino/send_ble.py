import asyncio
from bleak import BleakClient
import sys

# Replace this with the UUID you found from scan.py
ADDRESS = "86BA41FA-438D-45CC-D665-DFC46D05FBFF"

SERVICE_UUID = "180C"
CHAR_UUID = "2A56"

# Allow speed value as argument, default to 1000
try:
    speed = int(sys.argv[1])
except (IndexError, ValueError):
    print("Usage: python send_ble.py <speed> (700–2000)")
    sys.exit(1)

# Clamp speed within valid range
if not (700 <= speed <= 2000):
    print("Speed must be between 700 and 2000")
    sys.exit(1)

async def main():
    print(f"Connecting to {ADDRESS}...")
    async with BleakClient(ADDRESS) as client:
        print("Connected")
        await client.write_gatt_char(CHAR_UUID, speed.to_bytes(4, byteorder='little'))
        print(f"✅ Speed set to: {speed}")

asyncio.run(main())
