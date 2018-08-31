# TwistSpline
A smoothly reparameterizing Bezier spline that also interpolates orientations.

The idea here is to build a header-only C++ template library for the spline, and a reference implementation (maya plugin).

This is an *ALPHA* release, and is definitely *NOT* production ready. The inputs and outputs are all matrices.
Parenting isn't taken into account (double transforms everywhere); degenerate cases aren't handled nicely.
The tangent nodes should be constraints, and they currently aren't; and I'm sure there's more.

## Demos

This gif demonstrates the smoothly interpolating pin values for this test setup.
Any control can be pinned with an arbitrary value at any time.
Not shown: You can unpin *both ends* as long as at least one control has weight.
![Stretching Demo](https://tbttfox.github.io/Images/TwistSpline_StretchDemo.gif)

This gif deomonstrates the smoothly interpolating twist values (and their interaction with position pinning)
Again, any control can "pin" its twist value, and twist solves smoothly through length preservation
![Twisting Demo](https://tbttfox.github.io/Images/TwistSpline_TwistDemo.gif)
