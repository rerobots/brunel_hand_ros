brunel_hand_ros
===============

Introduction
------------

This ROS package defines several nodes and message types for working with the
[Brunel Hand](https://www.openbionics.com/shop/brunel-hand)
by [Open Bionics](https://www.openbionics.com).


Internal interface notes
------------------------

For interaction with the hand firmware directly over USB, the serial connection
speed is 115200 bps (bits per second), and no carrier-watch.  E.g., using
[C-Kermit](http://www.kermitproject.org/ck90.html) and supposing that your host
computer associated /dev/ttyACM0 with the USB device,

    $ kermit -l /dev/ttyACM0
    set speed 115200
    set carrier-watch off
    set terminal lf-display CRLF
    connect

After connecting, try `#` to get the system diagnostics, and try `?` to get the
list of serial commands.


References
----------

* https://www.openbionics.com/obtutorials/artichoke-v-1-1-control-methods
* https://www.openbionics.com/obtutorials/artichoke-v1-2-firmware-user-guide
