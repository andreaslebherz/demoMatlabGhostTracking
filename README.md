# demoMatlabGhosting - Ghost detection Simulator
Matlab demonstration of ghost detections in sumilated scenarios. This repository is based on:

https://de.mathworks.com/help/driving/ug/radar-ghost-multipath.html 

It therefore requires the 'Automated Driving Toolbox' and the 'Radar Toolbox'. This demo was developed under MatLab version R2021a.


Ghost detections appear when radar-waves are refelected between a side-barrier or guardrail and a traffic-object such as a car. Ghost detections are colored based on their bounce-index:
* Red: Direct reflexion
* Yellow: 1-2 reflexion (e.g. emitter -> car -> guardrail -> receiver)
* Magenta: 1-2-1 reflection (e.g. emitter -> guardrail -> car -> guardrail -> receiver)

The sensor configuration consists of 5R1C and is implemented scenario independent. 

Scenario files should be placed in the /scenario folder. 

Output .csv (detections) and .gif (simulation) are generated in the /output folder.

In mainDemoGhosting.m:

    1. Set EXPORT_CSV flag
    2. Set scenarioHandle to <scenario>.m
    3. Set filename to <scenario>.m (This has to match scenarioHandle)
    4. Set display.FollowActorID for .gif visualization
    5. Run
    