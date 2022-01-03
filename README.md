# kettle_hk700
Haier HK-700 alternative firmware (Beta)

This is a story of fixing long afterboil process on Haier HK-700.  
I was very selfconfident in fixing original firmware, so much confident that I erased read-protection bit in STM32 controller. 
And it was start of rewriting HK-700 firmware from scratch. 

Main issue of HK-700 is that thermo-resistor is hidden in bottom of kettle and there is huge delay between actual water temperature and resistor value. 
So after-boiling can run for minute after actual boiling starts. This delay is highly depends on amount of water in kettle and initial temperature.

And it was fun to investigate boiling process, to make figures of time-temperature, make analisys and produce some expressions wich can help to predict boiling. 
