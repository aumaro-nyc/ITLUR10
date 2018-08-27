# ITLUR10

Contains movement test scripts for UR10 robot


# invokeTest.py
- real time control of RoboDK simulation model via keyboard. Real UR10 robot position is updated whenever user presses 'm'.

# keyboardMove.py
- Controls UR10 robot in real time via keyboard input. Smoothness of movement varies greatly.

# toggleTest.py
- Updates robot position on an interval (every n movement commands). Helps with smoothness of movement, but still not ideal.
