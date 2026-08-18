import struct
with open("test_packet.bin", "rb") as f:
    data = f.read()
unpacked = struct.unpack('<Q34dI', data)
print(f"mass: {unpacked[17]}, cg: {unpacked[18]}, thrust_z: {unpacked[21]}")
