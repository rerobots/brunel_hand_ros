brunel_hand_ros
===============

Introduction
------------

This ROS package defines several nodes and message types for working with the
[Brunel Hand](https://openbionicslabs.com/shop/brunel-hand)
by [Open Bionics](https://www.openbionics.com).
It is documented on the ROS Wiki at <https://wiki.ros.org/brunel_hand_ros>


Parameters
----------

* `hand_dev_file`: device file corresponding to serial connection with Brunel
  Hand. Default value is /dev/ttyACM0


topics:

/fpose
/motion_primitive
/raw_input


Installation and dependencies
-----------------------------

This package is being developed with the ROS Kinetic distribution as its initial
target, but backward and forward ports are planned.

Dependencies:

* [serial](http://wjwwood.io/serial/) (on GitHub: https://github.com/wjwwood/serial/)


Getting started
---------------

As an initial demonstration, try

    roslaunch brunel_hand_ros toggle_fist.launch

This will start examples/toggle-fist.py and the main interface node, named
`proxy`. The hand should begin to alternate between open and closed fists.


Internal interface notes
------------------------

For interaction with the hand firmware directly over USB, the serial connection
speed is 115200 bps (bits per second), and no carrier-watch.  E.g., using
[C-Kermit](http://www.kermitproject.org/ck90.html) and supposing that your host
computer associated /dev/ttyACM0 with the USB device,

    kermit -l /dev/ttyACM0
    > set speed 115200
    > set carrier-watch off
    > set terminal lf-display CRLF
    > connect

After connecting, try `#` to get the system diagnostics, and try `?` to get the
list of serial commands.


References
----------

* https://www.openbionics.com/obtutorials/artichoke-v-1-1-control-methods
* https://www.openbionics.com/obtutorials/artichoke-v1-2-firmware-user-guide


Participating
-------------

All participation must follow our code of conduct, elaborated in the file
CODE_OF_CONDUCT.md in the same directory as this README.

### Reporting errors, requesting features

Please first check for prior reports that are similar or related in the issue
tracker at https://github.com/rerobots/brunel_hand_ros/issues
If your observations are indeed new, please [open a new
issue](https://github.com/rerobots/brunel_hand_ros/issues/new)

Reports of security flaws are given the highest priority and should be sent to
<security@rerobots.net>, optionally encrypted with the public key available at
https://rerobots.net/contact Please do so before opening a public issue to allow
us an opportunity to find a fix.

### Contributing changes or new code

Contributions are welcome! There is no formal declaration of code style. Just
try to follow the style and structure currently in the repository.

Contributors, who are not rerobots employees, must agree to the [Developer
Certificate of Origin](https://developercertificate.org/). Your agreement is
indicated explicitly in commits by adding a Signed-off-by line with your real
name. (This can be done automatically using `git commit --signoff`.)


License
-------

This is free software, released under the Apache License, Version 2.0.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
