Butt Heat Controller
====================

Custom controller for Haval Dargo's seat heaters (and A/C in the future).

Hardware and connections
------------------------

This shall run on ESP32C3 board with TJA1050 or similar module
Connect to 5V supply and BD-CAN-1 (see the schematics).

Controls: two tact buttons (left and right seat)
and SSD1306 0.91" 128x32 tiny display which fits above the "MODE" knob in the car.

Logic
-----

1. Listen for 2D1#00xx000000000000 messages, where `xx` is a heater status byte. They come like 10 per second.
2. Control: send 36D#00yy200000000040, where yy is a heater status *with offset*.
3. After sending a message, do not change internal state for external messages for e.g. 500ms, so that the machine will have time to react.
4. Bus error state: show "spinner" with LEDs.

Tasks:

1. Bus RX. Also responsible for bus reconnects?
2. Button input. Also sends CAN msgs?
3. LED output task.
4. Control task - do we need it?

Communication:
Button -> LED
Button -> TX (or just send directly?)
RX -> LED

States:
1. Loading, or bus error. Spinner. Denoted by spinner_sem taken by non-LED task.
2. Normal: show info based on led_from_can_queue.
3. Input (250ms after button press): show info based on led_from_btn_queue
