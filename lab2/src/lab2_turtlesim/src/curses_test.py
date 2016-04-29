#!/usr/bin/env python

import curses
import signal
import sys
import time

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1) #enables keypad
stdscr.nodelay(1) #non-blocking!

#stdscr.timeout(1)

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        curses.endwin()
        print("Done")
        sys.exit(0)



stdscr.addstr(0,10,"Use ctrl-c to quit")
stdscr.refresh()

signal.signal(signal.SIGINT, signal_handler)
#signal.pause()

key = ''
string = "foo"
while True:

    if string is "foo":
        string = "FOO"
    else:
        string = "foo"
    stdscr.addstr(1,10,string)
    #heartbeat

    key = stdscr.getch() #returns -1 for no character

    if key is not -1:
        stdscr.addch(20,25,key)
        stdscr.refresh()
        if key == curses.KEY_UP: 
            stdscr.addstr(2, 20, "Up")
        else:
            stdscr.addstr(2,20, "  ")
        if key == curses.KEY_DOWN: 
            stdscr.addstr(3, 20, "Down")
        else:
            stdscr.addstr(3,20,"    ")

    time.sleep(0.25)



