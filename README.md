# Arduino-Uno-Wind-Turbine-MPPT-Regulator
For a wind turbine, a regulator has two goals: 
To protect the wind turbine against over speed of the turbine that may destroy it,
To adapt the power delivered to charge a battery or to drive an injector. 

There are many regulators on the market. However they are mostly adapted for solar panels only, and if the curve of delivered power is similar, the way to regulate is different – to resume solar panels use buck converter, wind turbines use boost converter. 
Many are not MPPT, the PWM regulators are very less efficient than MPPT, and also specific wind turbine MPPT regulators are very expensive. 

So, It can be a very good project to self-build our own regulator, after having been built our own Piggott wind Turbine
