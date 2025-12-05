Butt Heat Controller
====================

Custom controller for Haval Dargo's seat heaters (and A/C in the future).

Hardware and connections
------------------------

This shall run on ESP32C3 board with TJA1050 or similar module
Connect to 5V supply and BD-CAN-1 (see the schematics).

Controls: two tact buttons (left and right seat)
and 4 or 6 LEDs to show current state.

Logic
-----

1. Listen for 2D1#00xx000000000000 messages, where `xx` is a heater status byte. They come like 10 per second.
2. Control: send 36D#00yy200000000040, where yy is a heater status *with offset*.
3. After sending a message, do not change internal state for external messages for e.g. 500ms, so that the machine will have time to react.
4. Bus error state: show "spinner" with LEDs.

Tasks:

1. Bus RX. Also responsible for bus reconnects?
2. Button input. Also sends CAN msgs?
