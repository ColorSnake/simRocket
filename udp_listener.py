import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 9090))
sock.settimeout(10.0)

try:
    data, addr = sock.recvfrom(4096)
    unpacked = struct.unpack('<Q34dI', data[:284])
    print(f"Timestamp: {unpacked[0]}")
    print(f"mass_kg: {unpacked[17]}")
    print(f"cg_z: {unpacked[18]}")
    print(f"thrust_z: {unpacked[21]}")
except Exception as e:
    print(e)
