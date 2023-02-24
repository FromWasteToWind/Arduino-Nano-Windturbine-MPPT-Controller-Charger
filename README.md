# Arduino-Nano-Wind-Turbine-MPPT-Regulator
This is a project that explains how to build a wind turbine MPPT-controller with battery charging functionality, based on a Arduino Nano or Nano Every.
This controller can be used for direct injection or lead-acid/li-ion battery charging.

For a wind turbine, a regulator has two goals: 
To protect the wind turbine against over speed of the turbine that may destroy it,
To adapt the power delivered to charge a battery or to drive an injector. 

## Features:  
Maximum Power Point Tracking (MPPT) with perterb and observe (P&O) algorithm and DC-DC bus
Battery charging: lead-acid or lithium based batteries.

Safety measures:  
- Short-circuit all three phases through relay brakes. A few 100ms time-interval between each relay to prevent a hard stop.

There are many regulators on the market. However they are mostly adapted for solar panels only, and even if the curve of delivered power is similar, the way to regulate is different. Many are not MPPT, the PWM regulators are very less efficient than MPPT, and also specific wind turbine MPPT regulators are very expensive. 



