# lh_tools

These are a series of functions to be ported and implemented on [LibSurvive](https://github.com/cnlohr/libsurvive/issues/8).

For now, these are a proof of concept that shows some promise.

simulator.py: generates data that simulates the operation of a lighthouse system. The model is not yet finished, 
OOTX data and noise sources must be implemented.

poser.py: its able to calculate the complete pose of an object, reprojects and calculates RMS error. 
Latest test showed an RMS error of 420 ticks or 3.2 mRad. Only works with some data source since the parser needs to be updated.
The principle of operation will be posted here soon.

sim_tools.py: will contain the whole model of the lighthouse system. Its not being used right now.

poser_tools.py: contains all functions needed to calculate pose from 2 data sets (model information and raw data).

tools.py: contains parsers and helper functions. Most functions needs to be rewritten.

/data/: contains data that will work with poser.py. Right now, it contains data shared by [@mwturvey](https://github.com/mwturvey) 
on [LibSurvive](https://github.com/cnlohr/libsurvive/issues/8).

 
## Principle of operation

To be posted soon...

## Examples
