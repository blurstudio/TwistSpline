# TwistSpline
A smoothly reparameterizing Bezier spline that also interpolates orientations.

The idea here is to build a header-only C++ template library for the spline, and a reference implementation (maya plugin).

This is an *ALPHA* release, and is definitely *NOT* production ready. Degenerate cases aren't handled nicely like:
  * Zero length tangents
  * CV's in a line, but ordered 1,3,2
  * Mis-ordered pin values
  * I'm sure there's more I haven't found yet
 
Also, I'm planning on giving the names of the node connections a once-over.

## Demos

This gif demonstrates the smoothly interpolating pin values for this test setup.
Any control can be pinned with an arbitrary value at any time.
Not shown: You can unpin *both ends* as long as at least one control has weight.
![Stretching Demo](https://tbttfox.github.io/Images/TwistSpline_StretchDemo.gif)

This gif deomonstrates the smoothly interpolating twist values (and their interaction with position pinning)
Again, any control can pin its twist value, and twist solves smoothly through length preservation
![Twisting Demo](https://tbttfox.github.io/Images/TwistSpline_TwistDemo.gif)


Not Shown: The Rider node can constrain between multiple splines. This means that a tool could easily be written to build and switch to another spline. For an artist, this would give the ability to add or remove spline controls on the fly in-scene.

## Compiling

Hopefully compiling should be easy using Cmake. There are no dependencies other than Maya and the STL. I only have access to Maya on Windows, so I can't really test on Linux or OSX, but I'm fairly certain everything is cross-platform. If anybody wants to set those up, please feel free, and send a PR.
If you are compiling for Windows, there's a `mayaConfigure.bat` file that should give an easy way to build a Visual Studio solution, or compile. You may have to edit it a bit to suit your needs, but it should be pretty straightforward.
