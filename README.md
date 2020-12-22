# JogWheel

## What it is

This is the repository for a jog wheel based on a stepper motor used not as a
motor but as a sensor (and as the jog wheel's mechanical guts). A stepper 
motor like the one used here has magnets and coils that move relative to 
one another when the output shaft turns. It's this relative motion that 
produces the cogging -- the bumps in torque -- you feel when you turn an 
unpowered stepper. It's also the cogging that makes such motors attractive 
to use in a jog wheel. It gives the wheel a nice feel.

JogWheel plugs into a USB port which it uses to send a sequence of keystrokes
and/or mouse events when you turn its wheel, one sequence for clockwise and 
anther for counterclockwise. The more you turn the wheel the more it repeats 
the sequences. Additionally, you can have multiple sets of sequences and can 
select which of them to use by clicking buttons on the JogWheel.

You can set the sequences with command line interface via a serial terminal. 
The sequences and other configuration information is saved in EEPROM, so that 
it isn't lost when the device is powered down.

## How it works
 
The ends of the stepper motor coils are exposed as the motor's leads. Because 
the motor is designed to spin, the coils and magnets are arranged so that the 
magnets pass the coils at different times as the shaft spins, this makes it 
possible to drive the motor by sequentially energizing and deenergizing the 
coils with the correct polarity and timing. Here, though, we take advantage 
of the fact that if you turn the stepper's shaft, it moves magnets past the 
coils. Doing that induces voltage pulses across the coils. By sensing the 
number and relative timing of the voltage pulses we can deduce how much the 
shaft has been turned, and in which direction.
 
The firmware is for a two-coil, bipolar stepper motor. in these steppers, 
the ends of its two coils, A and B, are exposed as four wires, A-, A+, B- 
and B+. When the shaft is turned clockwise, a pulse first appears on coil A 
and then on B. When it's turned counterclockwise, the pulse on B precedes 
the one on A. A bit of passive support circuitry cleans up the shape of the 
pulses, ensures the pulses don't go below about -0.6v and aren't bigger than 
about 4.5v. (Not surprisingly, the voltage pulses coming directly out of the 
motor go in both positive and negative directions and get many times bigger 
as you spin the shaft more quickly.) In the repository, the firmware is 
tuned for a stepper motor I extracted from a deceased printer, but it should 
work for any small two-coil bipolar stepper by tuning some of the 
compile-time #defines
 
