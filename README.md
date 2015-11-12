# e.Brew
Controller for single-kettle brewing with PID
# Hardware requirements
- Arduino Nano
- passive buzzer
- Adafruit RGB 16x2 LCD+Keypad Kit
- relay
- DS18B20 sensors

# Short description
This controller was build to control the heating of a single kettle for homebrewing beer (probably you could use this sketch for other stuff?). It is based on the Adafruit Sous Viduino code (https://github.com/adafruit/Sous_Viduino) and extended to fit my needs to turn it into a brewcontroller.

# Key features
- PID temperature controller (with autotune option)
- Buzzer for alerting or information
- 6 temperature schedules with 6 timers
- 6 hop timers
- Optional: overheat monitor for SSR/hardware used in the kettle of controllerbox
- User config values are stored in EEPROM with default values

# Usage
First thing after booting up is checking for a DS18B20 sensor. If none is found, a alarm will sound and the display will show a error. Running the program without a working DS18B20 sensor is not possible.
After bootup you are in the "e.Brew Ready" screen, the home/off position. From here you can navigate to all menu's using the keaypad buttons.

UP:     Mash/boil/hop menu

DOWN:   Setup menu

Left:   Clear memory

Right:  Start program

Select: Show current temperature
