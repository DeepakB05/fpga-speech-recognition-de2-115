# FPGA Speech Recognition on the DE2-115

A 64-word, speaker-dependent isolated-word recognizer written in Verilog for the Terasic DE2-115 board (Intel/Altera Cyclone IV E, EP4CE115F29C7). Audio comes from the on-board WM8731 codec. Recognized words are shown on the 7-segment displays, the LEDs and a VGA monitor, and can be assembled into an on-screen sentence. There is no processor and no C code: the whole chain is hardware.

## Team

|Role|Name|
|-|-|
|Team members|Deepak, John Julian, Chaitanya|
|Project guide|Dr. Jagannadha Naidu K|
|Institution|VIT University|

## Features

* Train and recognize up to 64 words (24-byte feature template per word in block RAM).
* Hardware feature extraction: frame energy, high-frequency tilt and zero-crossing rate, folded into 8 time segments.
* Template matching by L1 distance, about 61 microseconds for all 64 slots.
* VGA text display (640 x 480): recognized word in large letters, prompt for the word to train, and a 3 x 20 character sentence area.
* Hands-free mode, adjustable speech-trigger threshold and reject limit from the switches.

## Repository layout

|File|Purpose|
|-|-|
|`speech\_rec64.v`|Top module `speech\_top` plus `i2s\_rx`, `i2c\_wm8731\_cfg`, `feat\_ext`|
|`audio\_pll.v`|Hand-written ALTPLL wrapper, 50 MHz to 18.4 MHz codec clock|
|`speech.sdc`|Timing constraints (50 MHz clock)|
|`speech\_pins.qsf`, `speech\_pins\_vga.qsf`|Pin assignments (clock, keys, switches, LEDs, HEX, audio, I2C, VGA)|
|`speech\_top.qpf`, `speech\_top.qsf`|Quartus project files|
|`docs/`|Design reference, block diagram, photos|

## Build and program

1. Install Quartus Prime Lite 20.1 (with Cyclone IV support).
2. Open `speech\_top.qpf`. Top-level entity: `speech\_top`. Device: EP4CE115F29C7.
3. If pins are missing, use Assignments > Import Assignments with both `.qsf` pin files.
4. Set Unused Pins to *As input tri-stated* and compile.
5. Set the board's RUN/PROG switch to RUN, then program `output\_files/speech\_top.sof` with the USB-Blaster.
6. Plug a microphone into the pink MIC jack and a monitor into the VGA port.

## Controls

|Control|Function|
|-|-|
|`KEY\[0]`|Train: record a word into slot `SW\[5:0]`|
|`KEY\[1]`|Recognize one word|
|`KEY\[2]`|Clear all trained words|
|`KEY\[3]`|Reset (keeps trained words)|
|`SW\[5:0]`|Slot number 0 to 63|
|`SW\[7]`|Hands-free listening|
|`SW\[8]`|Flip up to clear the sentence|
|`SW\[13:10]`|Reject limit (distance limit = value x 128, 0 = off)|
|`SW\[14]`|Append recognized words to the sentence|
|`SW\[17:15]`|Speech trigger threshold (start at `110`, adjust using the mic level bar)|

Outputs: `HEX1:HEX0` show the recognized slot, `HEX7..HEX4` show the match distance in hex, `LEDR\[15:8]` is the mic level bar, `LEDG\[8]` means listening and `LEDG\[7]` means capturing.

The 64 word names are in the `WORDS` table inside `speech\_rec64.v` (exactly 8 characters each).

## How it works

Mic, WM8731 codec (I2S), `i2s\_rx`, `feat\_ext` (512-sample frames, 64 frames per word, 24-byte vector), then a control FSM with template RAM and matcher, then HEX/LED outputs, a sentence builder and a VGA text pipeline. See `docs/` for the full design reference.

## Limitations

Speaker-dependent, isolated words only, fixed 0.68 s word window, templates are lost when the board is reprogrammed, and accuracy drops as the vocabulary grows. Possible improvements: MFCC features, DTW matching, several templates per word, storing templates in flash or an SD card.

