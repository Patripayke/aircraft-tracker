import socket
import time
import threading
import math
import numpy as np
from queue import Queue
from collections import deque
from flask import Flask, jsonify
from flask_cors import CORS

HOST = "127.0.0.1"
PORT = 30003

MY_LAT = 27.9
MY_LON = -82.3

aircraft = {}
lock = threading.Lock()
data_queue = Queue(maxsize=10000)
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

msg_count = 0
start_time = time.time()


# =========================
# Kalman Filter (position smoothing)
# =========================
class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1000
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.1

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self.last_time = None

    def predict(self, dt):
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        z = np.array(z).reshape(2, 1)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def step(self, lat, lon):
        now = time.time()

        if self.last_time is None:
            self.x[0] = lat
            self.x[1] = lon
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        self.predict(dt)
        self.update([lat, lon])

    def get_state(self):
        return self.x.flatten()


# =========================
# Prediction (physics-based)
# =========================
def predict_position(lat, lon, speed_knots, heading_deg, dt):
    R = 6371e3

    speed_mps = speed_knots * 0.514444
    distance = speed_mps * dt

    heading = math.radians(heading_deg)

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance / R) +
        math.cos(lat1) * math.sin(distance / R) * math.cos(heading)
    )

    lon2 = lon1 + math.atan2(
        math.sin(heading) * math.sin(distance / R) * math.cos(lat1),
        math.cos(distance / R) - math.sin(lat1) * math.sin(lat2)
    )

    return math.degrees(lat2), math.degrees(lon2)


# =========================
# Networking
# =========================
def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    return s


def parse_sbs(line):
    parts = line.split(",")

    if len(parts) < 22:
        return None

    return {
        "hex": parts[4],
        "callsign": parts[10].strip(),
        "altitude": parts[11],
        "speed": parts[12],
        "heading": parts[13],
        "lat": parts[14],
        "lon": parts[15],
    }


def stream_thread():
    sock = connect()
    buffer = ""

    while True:
        data = sock.recv(4096).decode(errors="ignore")

        if not data:
            break

        buffer += data

        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            parsed = parse_sbs(line.strip())

            if parsed and parsed["hex"]:
                data_queue.put(parsed)


# =========================
# Distance
# =========================
def distance(lat1, lon1, lat2, lon2):
    R = 6371
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c * 0.621371


# =========================
# Consumer
# =========================
def consumer_thread():
    global msg_count

    while True:
        data = data_queue.get()
        update_aircraft(data)
        msg_count += 1


# =========================
# Update Aircraft
# =========================
def update_aircraft(data):
    hexcode = data["hex"]

    with lock:
        if hexcode not in aircraft:
            aircraft[hexcode] = {
                "kf": KalmanFilter(),
                "trail": deque(maxlen=20),  # trajectory history
                "heading_filtered": None
            }

        ac = aircraft[hexcode]

        if data.get("callsign"):
            ac["callsign"] = data["callsign"]

        if data["altitude"]:
            try:
                alt = int(float(data["altitude"]))
                if alt > 0:
                    ac["altitude"] = alt
            except:
                pass

        if data["speed"]:
            try:
                ac["speed"] = float(data["speed"])
            except:
                pass

        # Heading smoothing (low-pass filter)
        if data["heading"]:
            try:
                raw_hdg = float(data["heading"])
                if ac["heading_filtered"] is None:
                    ac["heading_filtered"] = raw_hdg
                else:
                    ac["heading_filtered"] = 0.8 * ac["heading_filtered"] + 0.2 * raw_hdg
                ac["heading"] = ac["heading_filtered"]
            except:
                pass

        if data["lat"] and data["lon"]:
            lat = float(data["lat"])
            lon = float(data["lon"])

            kf = ac["kf"]
            kf.step(lat, lon)

            state = kf.get_state()

            ac["lat"] = state[0]
            ac["lon"] = state[1]

            ac["distance"] = distance(MY_LAT, MY_LON, ac["lat"], ac["lon"])

            # Add to trail
            ac["trail"].append((ac["lat"], ac["lon"]))

            # Prediction
            if "speed" in ac and "heading" in ac:
                pred_lat, pred_lon = predict_position(
                    ac["lat"],
                    ac["lon"],
                    ac["speed"],
                    ac["heading"],
                    10
                )

                ac["pred_lat"] = pred_lat
                ac["pred_lon"] = pred_lon

        ac["last_seen"] = time.time()


# =========================
# Cleanup
# =========================
def cleanup_thread():
    while True:
        time.sleep(5)

        with lock:
            now = time.time()
            to_delete = []

            for h, ac in aircraft.items():
                if now - ac["last_seen"] > 60:
                    to_delete.append(h)

            for h in to_delete:
                del aircraft[h]


# =========================
# Display
# =========================
def display_thread():
    global msg_count

    while True:
        time.sleep(1)

        with lock:
            print("\033[2J\033[H", end="")

            elapsed = time.time() - start_time
            rate = msg_count / elapsed if elapsed > 0 else 0

            print("REAL-TIME AIRSPACE SYSTEM\n")
            print(f"Tracked Aircraft: {len(aircraft)}")
            print(f"Throughput: {rate:.1f} msgs/sec\n")

            sorted_ac = sorted(
                aircraft.items(),
                key=lambda x: x[1].get("distance", 9999)
            )

            print(f"{'CALLSIGN':<10} {'HEX':<8} {'ALT':<8} {'SPD':<6} {'HDG':<6} {'DIST':<8}")
            print("-" * 70)

            for h, ac in sorted_ac:
                if "lat" not in ac:
                    continue

                cs = ac.get("callsign", "UNKNOWN")
                alt = ac.get("altitude", "N/A")
                spd = int(ac.get("speed", 0)) if "speed" in ac else "N/A"
                hdg = int(ac.get("heading", 0)) if "heading" in ac else "N/A"
                dist = ac.get("distance")

                dist_str = f"{dist:.1f}mi" if dist else "N/A"

                print(f"{cs:<10} {h:<8} {alt:<8} {spd:<6} {hdg:<6} {dist_str}")

                # Show prediction
                if "pred_lat" in ac:
                    print(f"   -> predicted: ({ac['pred_lat']:.4f}, {ac['pred_lon']:.4f})")

                # Show trajectory (compressed)
                if len(ac["trail"]) > 3:
                    trail_str = " -> ".join(
                        [f"({round(p[0],3)},{round(p[1],3)})" for p in list(ac["trail"])[-5:]]
                    )
                    print(f"   path: {trail_str}")



# =========================
# API Routes
# =========================
@app.route("/aircraft")
def get_aircraft():
    with lock:
        data = []

        for h, ac in aircraft.items():
            if "lat" not in ac:
                continue

            # Convert trail deque to list of coordinate pairs
            trail_data = list(ac["trail"]) if "trail" in ac else []

            data.append({
                "hex": h,
                "callsign": ac.get("callsign", "UNKNOWN"),
                "lat": float(ac["lat"]),
                "lon": float(ac["lon"]),
                "alt": ac.get("altitude"),
                "speed": ac.get("speed"),
                "heading": ac.get("heading"),
                "pred_lat": ac.get("pred_lat"),
                "pred_lon": ac.get("pred_lon"),
                "distance": ac.get("distance"),
                "trail": trail_data  # Add the trail
            })

        print(f"API: Returning {len(data)} aircraft")  # Debug log
        return jsonify(data)


@app.route("/status")
def get_status():
    """Debug endpoint to check if API is working"""
    with lock:
        return jsonify({
            "status": "ok",
            "tracked_aircraft": len(aircraft),
            "total_messages": msg_count,
            "uptime_seconds": time.time() - start_time
        })


# =========================
# Main
# =========================
def main():
    print("Starting flight tracker...")
    print(f"Connecting to dump1090 at {HOST}:{PORT}")
    print(f"API will be available at http://127.0.0.1:5000")
    
    threading.Thread(target=stream_thread, daemon=True).start()
    threading.Thread(target=consumer_thread, daemon=True).start()
    threading.Thread(target=cleanup_thread, daemon=True).start()
    threading.Thread(target=display_thread, daemon=True).start()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
    daemon=True).start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()