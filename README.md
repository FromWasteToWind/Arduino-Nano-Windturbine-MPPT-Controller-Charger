# Arduino-Nano-Wind-Turbine-MPPT-Controller
This project explains how to build a windturbine MPPT-controller with battery charging functionality, based on a Arduino Nano or Nano Every.
The controller can also be be used for direct injection to a grid-tie inverter like the Victron Multiplus II.

For a windturbine, a controller has two goals: 
To protect the wind turbine against over speed of the turbine that may destroy it,
To adapt the power delivered to charge a battery or to drive an injector. 

There are many controllers on the market. However they are mostly adapted for solar panels only, and even if the powercurve is similar, the way to regulate is different. Many are not MPPT, and PWM regulators are much less efficient than MPPT. Furthermore  windturbine MPPT-controllers are very expensive. 

## Features:  
Efficiency:  
- Maximum Power Point Tracking (MPPT) with variable step perterb and observe (P&O) algorithm through DC-DC converter
- DC-DC converter allows for low cut-in and lower current losses because the generator voltage can raise higher than the battery voltage.

Battery charging:  
- lead-acid or lithium based batteries.

Safety measures:  
- Auto and manual brake through short-circuit of all three phases with a relay for each phase. A few 100ms time-interval between each relay to prevent a hard stop.

Communication:  
- LED's for the state of the controller and its outputs, 
- LCD display, to show current values of the input/output/current/power etc. Buttons are included to switch between display values and reset max values - or
even change some working parameters of the controller.
- SIM800 module with 2G to send out data to wherever






