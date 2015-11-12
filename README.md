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

UP:     Mash menu<br />
Here you can set up to 6 temperatures with 6 timers and 6 hop alarms. You enter this menu on 'Temperatur 1'. Use the UP and DOWN buttons to increase/decrease the value. Use SELECT and UP or DOWN simultaneously to make jumps of 10's. Use the LEFT and RICHT buttons to move forward and backward in the menu. Selecting LEFT on the first item will bring you back to home, selecting RIGHT on the last item will bring you also back to home. Every step has a 10 minute timeout, after that you will switch back to home menu (settings are saved).<br />
There are some rules for the temperature/timer/hop submenus:
- Next temperature step is allways at least 1 degree more then previous temperaturem you can't go lower
- After 99 degress the next step is 'Boil'
- Boil is allways the last temperature slot
- Each time slot is at least 1 minute
- You can only use hopalarms when the last temperaturs is Boil
- Next hop alarm is allways at least 1 minute after the last alarm
- First hop alarm can't be higher then boil time

DOWN:   Setup menu<br />
Here you can configure settings that are stored in EEPROM. Use the UP and DOWN buttons to increase/decrease the value. Use SELECT and UP or DOWN simultaneously to make jumps of 10's. Use the LEFT and RICHT buttons to move forward and backward in the menu. Selecting LEFT on the first item will bring you back to home, selecting RIGHT on the last item will bring you also back to home. Every step has a 10 minute timeout, after that you will switch back to home menu (settings are saved).<br />
You can change the following settings:
- Timer margin (offset for starting mashtime before reaching setpoint. Default value: 0,50 degree. When the setpoint is set to 50 degrees, it will start counting the minuts at 49.5 degrees).
- Set Kp (value for PID tuning. Default value : 850)
- Set Ki (value for PID tuning. Default value : 0.5)
- Set Kd (value for PID tuning. Default value : 0.1)
- Select temperature sensor. When more then 1 temperature sensor is connected, you can select the sensor for in the wort. Use the UP and DOWN buttons to select the correct sensor based on temperature reading. The other sensor is automaticly used for overhating alarm.
- Set overheat (Set the temperature for overheat alarm. When this temperature is reached a continuous alarm will sound and heating will be cut off. Default value : 60 degrees). 

LEFT:   Clear memory<br />
This will clear all temperature/time/hop settings so far, including current positions of runned programs. Thiw sill NOT reset the values from the setup menu. After clearing memory you will hear 2 beeps.

RIGHT:  Start program<br />
This will start the the program with values from the Mash menu. You can't start the program with a working DS18B20 sensor or at least 1 temperature and 1 timer. Pressing LEFT during the program will bring you back to home. Press SELECT and RIGHT together for autotune.

SELECT: Show current temperature<br />
This will continuous show the temperarature of the connected temperatues. Press LEFT to get back to home.
