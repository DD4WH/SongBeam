# SongBeam
Songrecorder using beamforming technology

0_SongBeam256.ino supports standalone realtime beamforming with Teensy 4.1 and SongBeam by delay-and-sum beamforming.

It is a prototype sketch using MQS for the output of the beamformed audio. Plan is to properly connect a DAC PCM5102 later.

The prototype sketch uses the following steps:

* collect audio blocks (BLOCK_SIZE 128 samples) from all the 4 mics via I2S
* bandpass filter the audio of each mic separately (64 taps, 1.2kHz to 8kHz)
* normalize the four audio channels separately
* define the desired steering angle of the SongBeam (-90 to 90 degrees)
* the sketch precalculates the FIR coefficients for four fractional delay filters applied to the four mic channels in order to align the audio signals among all mics and then sums up the output
* frequency domain application of the four FIR filters (FFT-convolution with FIR filter masks and subsequent iFFT) to the four mic channels
* sum up all the delayed audio channels for the beamformed output signal

