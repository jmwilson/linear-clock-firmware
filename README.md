# Firmware for [Linear Clock: Solar](https://jmw.name/projects/linear-clock/)

The project structure was created using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html), which generates the initialization code that runs on the microcontroller. In the generated files, the project-specific additions are found between `USER CODE BEGIN` and `USER CODE END` blocks. Noteworthy files:

* `Src/astronomy.cc`: code for computing the sun’s relative position in the horizontal coordinate system, and the day fraction used in the clock’s display
* `Src/tlc5926.cc`: control sequencies used for configuring the TI TLC5926 shift register drivers
* `Src/ublox.cc`: a state-machine based driver for parsing incoming UBX protocol messages from the ublox GNSS module
* `sunclock.ioc`: the STM32CubeMX project file containing the microcontroller pinout and peripheral configuration
