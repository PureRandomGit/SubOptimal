import socket
import csv
import datetime
import os
import threading

UDP_IP = ""       # Listen on all interfaces
UDP_PORT = 4444
CMD_PORT = 4445   # Port on ESP32 that receives motor commands
LOG_DIR = "logs"

esp32_ip = None
esp32_ip_lock = threading.Lock()


def command_sender(sock: socket.socket):
    """Read motor commands from stdin and send them to the ESP32 via UDP.

    Formats accepted:
      bl:0.5,br:0.5,tl:0.5,tr:0.5   -- set individual motors (0.0 – 1.0)
      all:0.5                         -- set all motors to the same speed
      stop                            -- stop all motors
    """
    print("Motor command formats:")
    print("  bl:0.5,br:0.5,tl:0.5,tr:0.5  (individual)")
    print("  all:0.5                        (all motors)")
    print("  stop                           (all motors off)")

    while True:
        try:
            cmd = input("> ").strip()
        except EOFError:
            break
        if not cmd:
            continue

        # Expand shorthands
        if cmd == "stop":
            cmd = "bl:0,br:0,tl:0,tr:0"
        elif cmd.startswith("all:"):
            val = cmd.split(":", 1)[1]
            cmd = f"bl:{val},br:{val},tl:{val},tr:{val}"

        with esp32_ip_lock:
            ip = esp32_ip

        if ip is None:
            print("  [!] ESP32 IP not yet known – waiting for first telemetry packet")
            continue

        sock.sendto(cmd.encode("utf-8"), (ip, CMD_PORT))
        print(f"  [>] Sent to {ip}:{CMD_PORT}: {cmd}")

def parse_packet(data: str) -> dict:
    """Parse 'key:value,key:value' format into a dict."""
    result = {}
    for pair in data.split(","):
        if ":" in pair:
            key, value = pair.split(":", 1)
            try:
                result[key.strip()] = float(value.strip())
            except ValueError:
                result[key.strip()] = value.strip()
    return result

def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"run_{timestamp}.csv")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_thread = threading.Thread(target=command_sender, args=(cmd_sock,), daemon=True)
    cmd_thread.start()

    print(f"Listening on UDP port {UDP_PORT}")
    print(f"Logging to {log_path}")

    fieldnames = ["timestamp", "heading", "error", "bl", "br", "tl", "tr"]

    with open(log_path, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        while True:
            data, addr = sock.recvfrom(256)
            with esp32_ip_lock:
                global esp32_ip
                esp32_ip = addr[0]
            decoded = data.decode("utf-8").strip()
            parsed = parse_packet(decoded)
            parsed["timestamp"] = datetime.datetime.now().isoformat()

            print(decoded)
            writer.writerow({k: parsed.get(k, "") for k in fieldnames})
            csvfile.flush() # Write immediately so data isn't lost on crash

if __name__ == "__main__":
    main()