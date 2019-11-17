# Some custom DonkeyCar parts

Feel free to use my custom parts for the donkeycar platform.
To clone everything, use 
```git clone https://github.com/blanck/parts.git ~/mycar/parts```

## Joystick/Gamepad controllers

Since I use different gamepads for different cars I have added the mapping in separate files

### Matricom (or other bluetooth/usb-dongle-based) gamepad controller
add this line after the other from rows in **manage.py**
```from parts.matricom import JoystickController```

### Arrogant Bastard wireless gamepad EG-C3059W
add this line after the other from rows in **manage.py**
```from parts.arrogant import JoystickController```



## USB camera part

A part using v4l2-ctl drivers to use a USB camera image instead of PiCamera

```cam = USBCamera(resolution=(160, 120), framerate = 15)```


## GPIO Motorcontroller for 4WD

A very basic motor controller to use for 4-wheel-drive motor controllers connected to separate GPIO channels

```from parts.gpio import GPIOCtrl, PWMThrottle, PWMSteering```

and

```
	steering_controller = GPIOCtrl()
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)
    V.add(steering, inputs=['throttle','angle'])
```
