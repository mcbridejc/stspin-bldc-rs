# stspin-bldc-rs

Motor control software targetting the STSPIN32F0A, written in Rust. It's designed to run on the
[STEVAL-ESC002V1](https://www.st.com/en/evaluation-tools/steval-esc002v1.html) eval board, which is
sold by ST for around $30 USD.

It's very simple: it performs open-loop commutation with an acceleration limit, and takes speed
commands via the serial port on J2. There are a few parameters to control power output and
acceleration, which can also be adjusted via serial commands.

It's designed for driving a spincoater with sensorless quadcopter motors. For lower speeds, BEMF sensing
doesn't work, so we need open-loop startup anyway. For the spin-coater, it may operate at speeds too low
to reliably do BEMF sensing and the load is small and consistent so open loop control is both simple and
effective. Drive duty cycle has two modes: an accelerating voltage and a lower hold voltage.

![STEVAL-ESC002V1](/doc/ESC002V1.jpg?raw=true "STEVAL-ESC002V1")

## Building

`cargo build` to compile.
`cargo run` to program flash and launch in gdb via openocd.

## Control of the motor

Control is performed by sending binary serial messages to the MCU via the serial port on J2. The
main message is an RPM command, but there are also messages for adjusting some control parameters on
the fly. A simple python script is provided in `cmd.py` for sending serial commands to the device.

This project was initially created to be used with this [spincoater
controller](https://github.com/mcbridejc/spincoater-controller).


## Programming

This eval board does not come with a USB port or any built in debugger, so it needs to be programmed
via the SWD connector (J1), or via the serial bootloader on J3. A mating connector for J1 is
provided with the board. You'll need a programmer, like an STLinkV2 or similar to program via SWD.
If you are considering programming this, you probably already know all of this!
