import RPi.GPIO as g
import time

def init():
    g.setmode(g.BOARD)
    g.setup(31, g.OUT)
    g.setup(33, g.OUT)
    g.setup(35, g.OUT)
    g.setup(37, g.OUT)

def destroy():
    g.output(31, False)
    g.output(33, False)
    g.output(35, False)
    g.output(37, False)

def forward(t):
    init()

    g.output(31, True)
    g.output(33, False)

    g.output(35, False)
    g.output(37, True)

    time.sleep(t)

    destroy()
    g.cleanup()

forward(2)