Butt Heat Controller
====================

Custom controller for Haval Dargo's seat heaters (and A/C in the future).

Hardware and connections
------------------------

This shall run on ESP32C3 board with TJA1050 (5V power supply) or similar module
Connect to 5V supply (or more likely IGN 12V) and BD-CAN-1 (see the schematics).

Controls: two rotary encoders (left and right A/C control, click -> seat heat control, long press -> seat memory recall)
and SSD1306 0.91" 128x32 tiny display which fits above the "MODE" knob in the car.

Power (IGN) connection:
H08/P08 pin29 wire A239 R-Y, IG1, fuse F122 10A
(H08/P08 pin32 wire A221 <s> ACC?)

BD-CAN-1 connection:
H104/P104 pin1,pin2 (Green-Red, Blue-Black)

"Montage map" from some alarm systems vendor: https://integrator.tecel.ru/api/attachment/181021
(it describes how to access the connectors under left kick panel)

Thingiverse project with a better description: https://www.thingiverse.com/thing:7438047

Logic
-----

1. Listen for 2D1#00xx000000000000 messages, where `xx` is a heater status byte. They come like 10 per second.
2. Control: send 36D#00yy200000000040, where yy is a heater status *with offset*.
3. After sending a message, do not change internal state for external messages for e.g. 500ms, so that the machine will have time to react.
4. Bus error state: show "spinner" with LEDs.

Tasks:

1. Bus RX. Also responsible for bus reconnects?
2. Bus TX. Not sure if it is really required though.
3. Router task. Gets events from rotary-encoder queue (the driver uses single queue for everything) and routes them to the respective tasks.
4. Button and rotary encoders input. Also sends CAN msgs? One task per button and per encoder. Each task gets events from both input and can and is responsible for interpreting them and notifying the display.
5. Display output task.

Communication:
RX -> Button (separate task per each)
Button -> TX (or just send directly?)
Button -> Display
RX -> Display (error state)

Types:
- ac_temp_t (LO, HI, unknown, half-degrees)
- butt_temp_t (0, 1, 2, 3, unknown)

States:
1. Loading, or bus error. Spinner. Denoted by spinner_sem taken by non-LED task.
2. Normal: show info based on led_from_can_queue.
3. Input (250ms after button press): show info based on led_from_btn_queue


What can go wrong
-----------------

The PCB schematics' maximum input voltage is 18V. Anything below is okay — so jump starting the car should not damage this device.
What happens if this voltage is exceeded for some reason? At 20-26V, D3 will die, together with a fuse. Recovery: replace D3 and fuse.
Above 40V: U4 and C3 may also die.

CAN bus. This device is connected to BD-CAN-1 — one of the 6 CAN buses in Haval Dargo. PCB version uses TJA1051T/3 chip — it has a built-in timeout, so even if MCU hangs, CAN bus is never held busy.
If another CAN chip is used in your build, *and* MCU hangs in CAN-low condition (never occured on the prototype) — in the worst case BD-CAN-1 will become irresponsible. What does it mean for the car? This bus is used for the following devices, which will stop communicating with each other:
- Multimedia system
- Front doors (buttons)
- Front seats (seat moving buttons)
- Wireless phone charger
- Climate control system
- KBCM module
- Most noticeable: multi-switch under steering wheel

Note: nothing critical (steering, brake, ignition) is affected.

How to fix this if it ever happens? Power cycle will most likely help. If it doesn't then disconnect the device. That's why the device should be connected with a detachable connector rather than wired in directly.
