import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 9090))
sock.settimeout(5.0)

try:
    data, addr = sock.recvfrom(4096)
    unpacked = struct.unpack('<Q34dI', data[:284])
    num_engines = unpacked[35]
    print(f"num_engines: {num_engines}")
    for i in range(num_engines):
        offset = 284 + i * 24
        eng_data = struct.unpack('<3d', data[offset:offset+24])
        print(f"engine {i} thrust: {eng_data}")
except Exception as e:
    print(e)
