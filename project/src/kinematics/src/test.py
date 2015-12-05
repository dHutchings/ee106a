import curses

screen  = curses.initscr()

try:
    curses.noecho()
    curses.curs_set(0)
    screen.keypad(1)
    screen.addstr("Press a key")
    event = screen.getch()
finally:
    curses.endwin()


if event == curses.KEY_LEFT:
    print("LEFT")
elif event == curses.KEY_RIGHT:
    print("RIGHT")
else:
    print(event)