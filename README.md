# lh_tools

These are a series of functions to be ported and implemented on [LibSurvive](https://github.com/cnlohr/libsurvive/issues/8).

For now, these are a proof of concept that shows some promise.

simulator.py: generates data that simulates the operation of a lighthouse system. The model is not yet finished, 
OOTX data and noise sources must be implemented.

poser.py: its able to calculate the complete pose of an object, reprojects and calculates RMS error. 
Latest test showed an RMS error of 195 ticks or 1.5 mRad. Only works with some data source since the parser needs to be updated.
The principle of operation will be posted here soon.

sim_tools.py: will contain the whole model of the lighthouse system. Its not being used right now.

poser_tools.py: contains all functions needed to calculate pose from 2 data sets (model information and raw data).

tools.py: contains parsers and helper functions. Most functions needs to be rewritten.

/data/: contains data that will work with poser.py. Right now, it contains data shared by [@mwturvey](https://github.com/mwturvey) 
on [LibSurvive](https://github.com/cnlohr/libsurvive/issues/8).

 
## Principle of operation

### The LH Coordinate System
Defining a coordinate system allows us to transform any set of points from cartesian to lighthouse equivalent and backwards. For any cartesian point (x, y, z) there’s an equivalent lighthouse transformation (ρ, θ, ϕ) where rho is the radial distance of the point to the origin (or the absolute distance), theta is the “horizontal” sweep of the lighthouse and phi is the “vertical” sweep.
Here’s how those axes relate to each other:


Where h1 is the projection of the vector in the x-y plane and h2 is the projection of the vector in the z-y plane.
The equations used to pass from cartesian to lighthouse are the following:
Type equation here.


The equation used to pass from lighthouse to cartesian are the following:
Type equation here.


### The Poser – Finding R
So far, these equations cannot be directly with the raw data to calculate the pose of an object since we are missing rho: the radius or distance. It makes sense considering that without a defined radius, the sensors will be laying at any point within a straight line radiating from the coordinate system. Like this:

To narrow down the radius, we need to add in more information… The calibration model. More specifically, the distance between sensors. 


The distance between rays are diverging from 0 to infinite as the radius increases. This means that there must be at least a pair of radii that the distance between them is equal to the distance of the pair of sensors in the model. To find the distance between 2 sensors in LH coordinates you can use this equation:

The good news is that this works, the bad news is that the number of radii that satisfy this condition is infinite. Bounded, but still, infinite.

The radii can be anywhere between 0 and rho-max. Imagine a sensor sliding along its ray, this sensor have an imaginary construction circle around it and its radius is the distance between these 2 sensors. At exactly rho1 = 0 the is only one intersection of the construction circle on the other sensor’s ray. This means that when 0 <= rho1< d then rho2 only have one solution. This is never the case since the sensor would be literally inside of the lighthouse. As rho1 increases, its construction circle now crosses the ray at 2 points. This complicates things since not only we know that rho1 can increase and STILL have a solution, but now we know it has 2 solutions.

Adding a third sensor in the mix will increase the amount of equation to be solved by 2 since you can calculate the distance between 1-2, 2-3, 3-1 using the model data. This is a 3x3 system of non-linear equations and it does have solutions. However, each radius has 2 equally valid solutions. This means that the system will yield 6 solutions instead of a single solution. 

By adding yet another sensor in the equations you can increase the number of equations that should yield a single solution. This a 4x3 system of non-linear equations which is over-determined. Finding solutions for a non-linear equation requires an iterative method, let alone an over-determined system of non-linear equations. My testing showed that Levenberg-Marquardt with least squares works great and will find a solution in about 40 iterations and reaches a tolerance of 1.49012e-08 m.

Note: I have limit the number of sensor to 4 and the number of equations to 4 also. The main reason was performance and fear that adding all possible sensors will drastically increase the compute time. I haven’t tested this idea but it should yield better precision.

The initial estimate is currently set to 3 m which I find to be a very common distance to the lighthouse. In the next pose you could use the previous result as an initial estimate and it will decrease the compute time drastically.

Calculating the remaining radii is simpler since we already have 4 radii. If you try to simply solve the equation you will end up with 2 solutions (the line intersecting a circle situation). You could write a new system of equations or you could develop an algorithm capable of comparing which solution is true. If you choose to ignore this, the error in radius could be up to twice the distance between sensors.

Now that all sensor data have their radii and, consequently, are fully represented in LH coordinates, we are now able to transform them into cartesian coordinates using the equation defined before.

### The Poser – Wahba’s Problem and Kabsch Algorithm
Up to this point, we have all points defined in 3D space viewed from the lighthouse. Now to get a pose we need to compute translation and rotation using this data and the calibration model data. This problem is well known and its known as [Wahba’s problem](https://en.wikipedia.org/wiki/Wahba%27s_problem). One of the most popular solution is called [Kabsch Algorithm]( https://en.wikipedia.org/wiki/Kabsch_algorithm) which is quite literally the next few steps of the poser. My approach is heavily based on [Nghia Ho’s code]( http://nghiaho.com/?page_id=671). (Great website btw).

We start by calculating the centroid of all visible points in both the model data and lighthouse data. To put it simply, the centroid is the average position of the points. This is used to re-center or translate the points so that both centroid is located at the origin. This will become the point of rotation. To achieve this, each point in the model data and the lighthouse data is vectorially subtracted with their centroid.

Once both data sets are centered on the same point, we can find rotation using SVD or magic. I’m not going to try to explain SVD simply because I don’t understand it. What you should know is that SVD is a way to decompose or factorize a matrix. If the matrix you feed into SVD is a cross-covariance matrix (a matrix that describes the similarity between 2 matrices) it will return a scaling matrix and 2 rotational matrices. The rotational matrices can be combined to create a single rotational matrix. The scaling matrix can be discarded but it’s actually a helpful indicator of error (the points should have the same scale).

The cross-covariance matrix is easily calculated by transposing a data set and multiplying it with the other data set. This is fed into SVD when the matrix is decomposed into U, V and S. The rotational matrix is calculated by transposing V and U and multiplying them together. Multiplying this rotational matrix with a centroid you rotate its reference frame. Subtracting this with the other centroid will return the translation vector.

###Potential Issues - Fixes
- The rotation is not calculated about the origin but on an arbitrary centroid. This will probably contribute to error since reprojection is rotated about the origin. This can be fixed by reprojecting the origin and redoing the whole process again but using the origin rather than a centroid. It may be possible to find a function that will change the rotation center. Another solution is to create yet another system of equations in order to solver for rho theta and phi of the origin.
- The resulting vector and rotational vector are referenced to lighthouse not the world coordinates.
- The error gained when calculating the remaining radii can throw off the pose, particularly the rotation. This must be solved.
- Calculating all radii in a single system of equations could increase precision. This must be tested
- The poser should apply OOTX data to increase precision.


## Examples
