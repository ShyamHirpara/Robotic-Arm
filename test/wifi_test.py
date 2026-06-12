"""Standalone Pico W WiFi diagnostic — scan visible networks, then try joining.

Run from the PC with:
    py -m mpremote connect COM14 run test/wifi_test.py
"""
import network
import time

NETWORKS = [
    ("ARS",    "9925512860"),
    ("ARS_5G", "9925512860"),
]
TIMEOUT_MS = 15000

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.config(pm=0xa11140)   # disable power-save so the Pico answers pings
time.sleep(1)

print("--- Scan (Pico W radio is 2.4 GHz only) ---")
try:
    nets = wlan.scan()
    if not nets:
        print("  no networks visible")
    for n in sorted(nets, key=lambda x: x[3], reverse=True):
        print("  SSID={!r}  ch={}  RSSI={}".format(n[0].decode(), n[2], n[3]))
except Exception as e:
    print("  scan failed:", e)

for ssid, pwd in NETWORKS:
    print("--- Joining {!r} ---".format(ssid))
    wlan.connect(ssid, pwd)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < TIMEOUT_MS:
        if wlan.isconnected():
            break
        time.sleep_ms(250)
    if wlan.isconnected():
        print("CONNECTED ifconfig:", wlan.ifconfig())
        print("Staying online for 30 s — ping me now ...")
        for i in range(30):
            time.sleep(1)
        print("DONE")
        break
    print("FAILED status={}".format(wlan.status()))
    wlan.disconnect()
else:
    print("RESULT: could not join any network")
