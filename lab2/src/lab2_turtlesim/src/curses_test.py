import curses
import signal
import sys
stdscr = curses.initscr()
#curses.cbreak()
#stdscr.keypad(1)
stdscr.nodelay(1) #non-blocking!

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
while key != ord('q'):
    key = stdscr.getch()
    stdscr.addch(20,25,key)
    stdscr.refresh()
    if key == curses.KEY_UP: 
        stdscr.addstr(2, 20, "Up")
    elif key == curses.KEY_DOWN: 
        stdscr.addstr(3, 20, "Down")

curses.endwin()