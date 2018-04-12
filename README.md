# lh_simulator

This simulator is a proof of concept. It's meant to be used as a tester for posers to be used in Libsurvive. https://github.com/cnlohr/libsurvive

The simulator works by transforming an object into spherical coordinates. By making this calculation at the beginning of the sweep, we can start a timing variable at zero and calculate the time it would take for the laser to hit the sensor if it were to stay fixed. By feeding this timing data into a path function, you can calculate the pose of the object at this new time. The timing variable is set to this new time. By calculating the angular difference within this 2 poses and a sensor and comparing it with a set tolerance, the program determines if the angular position of this sensor is precise enough. If it is, the program returns. Else, the timing variable is set again and the whole process repeats itself until the condition is met.
