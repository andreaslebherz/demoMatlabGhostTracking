# demoMatlabGhostTracking - Ghost detection and tracking Simulator
Matlab demonstration of object tracking including ghost detections in simulated scenarios. This repository is based on:

https://de.mathworks.com/help/driving/ug/extended-object-tracking.html
https://de.mathworks.com/help/driving/ug/radar-ghost-multipath.html 


It therefore requires the 'Automated Driving Toolbox', 'Sensor Fusion and Tracking Toolbox' and the 'Radar Toolbox'. This demo was developed under MatLab version R2021a.

Ghost detections appear when radar-waves are refelected between a side-barrier or guardrail and a traffic-object such as a car. Ghost detections are colored based on their bounce-index:
* Red: Direct reflexion
* Yellow: 1-2 reflexion (e.g. emitter -> car -> guardrail -> receiver)
* Magenta: 1-2-1 reflection (e.g. emitter -> guardrail -> car -> guardrail -> receiver)

The sensor configuration consists of 5R1C and is implemented scenario independent. The tracker does not differ between moving and static objects and leads to faulty results when used with scenarios that contain barriers/guardrails. 

Scenario files should be placed in the /scenario folder. 

Output .csv (detections) and .gif (simulation) are generated in the /output folder.

In mainDemoGhostTracking.m:

    1. Set EVAL_POT and EVAL_JPDA flags
    2. Set EXPORT_CSV and PLOT_METRICS flags
    3. Set scenarioHandle to <scenario>.m
    4. Set filename to <scenario>.m (This has to match scenarioHandle)
    5. Set display.FollowActorID for .gif visualization
    6. Run
