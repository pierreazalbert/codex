# Codex

![codex-photo](codex-photo-sd.jpg)

**Codex** is a **Co**mmunication **De**vice for **Ex**traterrestrial Beings.

### What is Codex?

The plans to build Codex were sent to us from outer space, by extraterrestrial beigns trying to communicate with Earth. 

According to their plans, the device is capable of recording human speech, and can translate it into extraterrestrial language, which is made of movement and light patterns.

### How does it work?

The code show in this repository is our attempt at recreating the basic function of the Codex, in order to create a prototype that will hopefully allow us to reply to those who sent us the plans. 

The Codex operates in a similar fashion to what humans call FSMs, Finite State Machines. By default the Codex is idle, and waits for a user to approach it in order to initialise extraterrestrial communication. Once the channel is ready the Codex will start moving, indicating that it is ready to record and translate human speech. As soon as the user starts speaking, the Codex will move faster, showing that it is processing the message and translating it into extraterrestrial language. When the message ends the Codex will mark a pause, then it will play the translated message in extraterrestrial-readable light patterns. The state transition diagram of the Codex FSM is shown below. 

![codex-diagram](codex-diagram.pdf)

### How can I build my own Codex?

The human-made Codex software is written for Arduino. The main file `codex.ino` contains the implementation of the Finite State Machine that controls the Codex communication device. This file depends on two external libraries: [Adafruit's NeoPixel library](https://github.com/adafruit/Adafruit_NeoPixel), and the [Ultrasonic library by Erick Simões](https://github.com/ErickSimoes/Ultrasonic). The dependencies are included in this repository.

In order to help you assemble the hardware components of the Codex, we have prepared a simplified, human-friendly assembly guide based on the original plans sent to us from outer space. This repository contains the laser cutting and 3D printing files required to manufacture the most  parts, as well as a list of the other components which can be bought from regular suppliers.