# Homing (Joint Position Calibration) {#homing}

## What is "Homing" and Why is it Needed?

When using relative encoders (like the quadrature encoders, we are using), we
only get relative position changes (e.g. "motor moved by 13 degree").  By
accumulating these changes, the position can be computed.  However, when turning
the robot on, the motor controller does not know the absolute position of the
motor so it just defines the current position to be zero.  This means that while
the position is consistent while the robot is running, the same physical
position can get different position values when the robot is turned off in
between.

To solve this problem, the so called "homing" is performed in the beginning when
turning the robot on.  The idea is to search for a fixed physical position that
can always be found, independent of where the joint is located when turned on
(e.g. by having a switch that is triggered when the joint gets at that
position).  Once this "home position" is found, it can be set as zero position.
This way, the same physical position will always correspond to the same position
value, even if the robot is turned off between runs.

## How is the Homing Implemented for the BLMC Robots?

In the `BlmcJointModule[s]` class, the following procedure is implemented:

1. From the current position move slowly in one direction (depending on
   parameters) until the next occurrence of the encoder index is reached.
2. Set this position as home position.
3. Set the zero position to home_position + home_offset.

The _encoder index_ is a special tick on the encoder wheel that appears only
once per motor revolution.  This means once the encoder index is found, the
absolute position of the motor is known.  However, there is typically a gear box
so that multiple motor revolutions are needed for one joint revolution.  So
while the motor position is now known, the absolute position of the joint is
still unknown as there are multiple occurrences of the encoder index within the
range of the joint.

To solve this issue, it needs to be ensured that the "correct" encoder index is
found.  For robots with joint end-stops (e.g. the Finger robot) this is done by
first moving in one direction until the end-stop is reached (detected by waiting
for the velocity to become zero).  Then the encoder index search is started from
the end-stop position, thus guaranteeing that always the same index is found.
For robots without end-stops (e.g. Solo) this can be solved by first moving the
robot manually to a defined position before starting the homing.

### Home Position vs Zero Position  – Meaning of the Home Offset

The home position is typically given by the mechanics (e.g. based on where the
encoder index is located) and can not be influenced by the user.  This position
is often not the one where the user wants to have the zero position.  To place
the zero at a different position, the "home offset" parameter can be used.  Once
the home position is found, the zero is set to

    zero_position = home_position + home_offset

This means, the zero can be placed wherever is best for the application by
simply setting the home offset accordingly.

## How to Determine the Home Offset

To determine the desired home offset value (e.g. when setting up a new robot),
simply follow these steps:

1. First set the home offset to zero.
2. Start the robot and perform the homing.
3. Manually move the robot to the desired zero position and print the position
   of the joints (after homing these positions are now relative to the home
   position).
4. Set the joint positions of the desired zero position as home offset.

When restarting now, the actual zero position after homing should be at the
desired one.
