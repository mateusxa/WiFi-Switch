# WiFi-Switch

This project is a switch to turn on a lamp on a Relay and dimmer a RGB LED Strip

## Dimmering LEDs
### Triyed to keep all timers with same top value and same frequency to make dimming easier

1. Timer 1 <br />
    This timer controls the __RED__ LED PWM, it has a __prescaler of 16__ and __top value of 255__

2. Timer 2 <br />
    This timer controls the __GREEN__ LED PWM, it has a __prescaler of 16__ and __top value of 255__

2. Timer 3 <br />
    This timer controls the __BLUE__ LED PWM, it has a __prescaler of 16__ and __top value of 255__

## Controlling LED Strip
To control LED Strip we have to enable which led we want to control and then increase or decrease PWM duty cycle.
<br />
I'm using __interrupts__ on the buttons to select the mode and increase it's duty cycle of selected mode.
