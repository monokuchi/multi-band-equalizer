<a id="readme-top"></a>

<!-- PROJECT LOGO -->
<div align="center">
  <h1>Multi-Band Equalizer</h1>
  <p>
    Multi-band parametric equalizer implemented using a STM32H7 ARM7 Microcontroller along with a Cirrus Logic CS4272-CZZ Audio Codec
  </p>
</div>


<!-- PROJECT DESCRIPTION -->
<h2>Description</h2> 

<header>
    <ul>
        <li><h3>Multi-Band-Equalizer Front Side</h3></li>
    </ul>
</header>

![Multi-Band-Equalizer PCB Front Side][multi-band-equalizer-pcb-front]


<header>
    <ul>
        <li><h3>Multi-Band-Equalizer Back Side</h3></li>
    </ul>
</header>

![Multi-Band-Equalizer PCB Back Side][multi-band-equalizer-pcb-back]


<header>
    <ul>
        <li><h3>CS4272-CZZ Breakout Board</h3></li>
    </ul>
</header>

![CS4272-CZZ_Breakout_Board PCB Front Side][cs4272-czz_breakout_board-front]

10-band parametric equalizer using digital 2 stage IIR peaking filters.

Features:
* Efficent pipelining of stereo audio samples to/from the MCU and the codec by utilizing double buffering as well as DMA streams
* Fast computation of cascaded IIR filter stages by taking advantage of ARM's CMSIS DSP libaries (Decreases compute time by ~64%!)
* Generic design of hardware allows for this PCB to used for more then an EQ. All you need to do is change the firmware to alter the processing of the audio samples

Check out this *[script][python-script-url]* for simulating the cascaded filter responses

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
[multi-band-equalizer-pcb-front]: pcb/multi-band-equalizer/References/multi-band-equalizer_front.png
[multi-band-equalizer-pcb-back]: pcb/multi-band-equalizer/References/multi-band-equalizer_back.png
[cs4272-czz_breakout_board-front]: pcb/CS4272-CZZ_Breakout_Board/References/CS4272-CZZ_Breakout_Board.png
[python-script-url]: scripts/peaking_filter.py

