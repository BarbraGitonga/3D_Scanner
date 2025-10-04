import csv
import socket

HOST = "192.168.4.1"
PORT = 23

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(f"Connecting to {HOST}:{PORT} ...")
    s.connect((HOST, PORT))
    print("Connected!")

    with open("data1.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Roll", "Pitch", "Yaw", "Distance"])  # header row

        buffer = ""  # store partial TCP data

        while True:
            chunk = s.recv(100).decode(errors="ignore")
            if not chunk:
                break

            buffer += chunk
            while "\n" in buffer:  # process complete lines
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                print("Received:", line)
                try:
                    roll, pitch, yaw, distance = map(float, line.split(","))
                    writer.writerow([roll, pitch, yaw, distance])
                    f.flush()
                except ValueError:
                    print("Invalid data format:", line)
