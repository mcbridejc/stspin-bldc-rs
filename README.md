# stspin-bldc-rs

Motor control software targetting the STSPIN32F0A. It's tested on the STEVAL-ESC002V1 eval board. 

Presently, it's very simple: it just performs open-loop commutation with an acceleration limit,
and takes speed commands via the serial port on J2. 

It's designed for driving a spincoater with sensorless quadcopter motors. For lower speeds, BEMF sensing
doesn't work, so we need open-loop startup anyway. For the spin-coater, it may operate at speeds too low
to reliably do BEMF sensing and the load is small and consistent so open loop control is both simple and
effective. Drive duty cycle has two modes: an accelerating voltage and a lower hold voltage.


