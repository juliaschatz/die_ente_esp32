#!/usr/bin/python

import pygame, socket, os, time, select

# set SDL to use the dummy NULL video driver, so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"
# init pygame
pygame.init()
# create a 1x1 pixel screen, its not used so it doesnt matter
screen = pygame.display.set_mode((1, 1))
# init the joystick control

pygame.joystick.init()
_joystick = pygame.joystick.Joystick(0)
_joystick.init()

wport = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wport.connect(("192.168.4.1", 4357))
#wport.open()
# Main event loop
ready = False
while True:
    pygame.event.get()
    lx = _joystick.get_axis(0)
    ly = _joystick.get_axis(1)
    rtrigger = _joystick.get_axis(4)
    rx = _joystick.get_axis(2)
    reset = 1 if _joystick.get_button(7) else 0

    yaw = int(127 * lx)
    pitch = int(127 * ly)
    roll = int(127 * rx)
    throttle = int(127 * (rtrigger+1)/2)
    special = reset

    print(f"{yaw} {pitch} {roll} {throttle}")

    if abs(lx) < 0.2:
        yaw = 0
    if abs(ly) < 0.2:
        pitch = 0
    if abs(rx) < 0.2:
        roll = 0
    ready = ready or throttle == 0
    
    if ready:
        data = b''.join(map(lambda d: d.to_bytes(1, "big", signed=True), [yaw, pitch, roll, throttle, special]))
        wport.send(data)

    readable, _, _ = select.select([wport], [], [], 0.0)
    if wport in readable:
        resp = wport.recv(64).decode("U8", errors="ignore")
        print(resp, end="")

    time.sleep(20 / 1000)  # Sleep 20 ms to let esp32 catch up