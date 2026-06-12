"""Pico-side network diagnostic: report WLAN state, test gateway and PC reachability."""
import network
import socket
import time

w = network.WLAN(network.STA_IF)
print("connected:", w.isconnected())
print("ifconfig :", w.ifconfig())

if not w.isconnected():
    print("not connected — rejoining ARS for the test")
    w.active(True)
    w.config(pm=0xa11140)
    w.connect("ARS", "9925512860")
    t0 = time.ticks_ms()
    while not w.isconnected() and time.ticks_diff(time.ticks_ms(), t0) < 15000:
        time.sleep_ms(250)
    print("rejoin   :", w.isconnected(), w.ifconfig())


def tcp_test(label, ip, port):
    s = socket.socket()
    s.settimeout(5)
    try:
        s.connect((ip, port))
        print(label, "-> TCP CONNECT OK")
    except Exception as e:
        print(label, "-> FAIL:", e)
    finally:
        s.close()


tcp_test("gateway 192.168.1.1:80 ", "192.168.1.1", 80)
tcp_test("PC      192.168.1.36:135", "192.168.1.36", 135)
