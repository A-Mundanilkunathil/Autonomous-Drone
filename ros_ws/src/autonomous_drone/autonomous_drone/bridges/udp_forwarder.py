import socket
import sys

# Listen on Mac
listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listen_sock.bind(('0.0.0.0', 5005))

# Forward to VM
forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vm_ip = '192.168.64.10'
vm_port = 5005

print(f"UDP Forwarder started: Mac:5005 -> VM:{vm_ip}:{vm_port}")
print("Press Ctrl+C to stop")

packet_count = 0
try:
    while True:
        data, addr = listen_sock.recvfrom(2048)
        forward_sock.sendto(data, (vm_ip, vm_port))
        packet_count += 1
        if packet_count % 100 == 0:
            print(f"Forwarded {packet_count} packets...")
except KeyboardInterrupt:
    print(f"\nStopped. Total packets forwarded: {packet_count}")
    sys.exit(0)