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

## Usage

Currently, the easiest way to handle building a spline and all of its required connections is to use `twistSplineBuilder.py` and run the `makeTwistSpline` function inside. You will probably have to edit this file to fit in your pipeline.

`makeTwistSpline(prefix, numCVs, numRiders, spread=1, maxParam=None)`

`prefix` is the naming prefix for all the DAG objects.
`numCVs` is the total number of CVs
`numRiders` is the number of evenly spaced locators riding the spline. Locators are used because joints don't show orientation.
`spread` is the amount of space between each controller (cv and tangent). So a spread of 1 makes the CVs 3 apart.
`maxParam` is the parameter value of the last CV. It defaults to (3 * spread * (numCVs - 1))

# Node Documentation

## TwistSpline Node

This node implements the TwistSpline, which is a dynamically re-parameterizing bezier spline that interpolates twist along its length.

This TwistSpline can have any number of Control Vertices, and each control vertex has an incoming and outgoing tangent. The incoming tangent is ignored for the first CV, and the outgoing tangent is ignored for the last CV.

The OutputSpline plug is a custom type specific to this set of nodes.

The per-CV plugs control how the spline behaves and reparameterizes. The ControlVertex plug recieves the worldpace position and orientation of the CV itself. The InTangent and OutTangent are world matrices that define the bezier tangent positions and orientations for this CV. ParamValue and ParamWeight are used as a pair. ParamValue is the parameter that the currentCV will have when ParamWeight is 1.0. If ParamWeight is 0.0, then the parameter of this CV will float between it's neighbor's parameters (as a percentage of the lengths of its neighboring bezier spline segments). The parameterization algorithm works as long as any paramWeight is greater than 0.

TwistValue and TwistWeight work in a similar way. Twists are interpolated between values that have weight, and skip those that don't. Because this is handled as an interpolated float value, twist behavior allows for arbitrary values, and can twist past 360deg multiple times along the spline.

UseOrient is the "weight" of the ControlVertex worldspace orientation as part of the twist. Because of the limitations of Matrices and Quaternions, you can never get more than a 180 degree twist using orient behavior and popping can occur if you're not careful.

DebugDisplay and DebugScale show the orientation frames calculated along the spline and control their size.


| Long Name (short name) | Type | Default |
| --- | --- | --- |
|outputSpline (os) | TwistSpline | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`The custom spline data output` | | |
|splineLength (sl) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;`The total length of the spline` | | |
|debugDisplay (dd) | bool | False |
|&nbsp;&nbsp;&nbsp;&nbsp;`Whether to show the debug axes` | | |
|debugScale (ds) | double | 1.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;`The size of the debug axes` | | |
|vertexData (vd) | compound | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;Inputs for all of the per-cv data | | |
|&nbsp;&nbsp;&nbsp;&nbsp;inTangent (int) | matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The bezier tangent pointing towards the start of the spline` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;outTangent (ot) | matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The bezier tangent pointing towards the end of the spline` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;controlVertex (cv) | matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The control vertex` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;paramValue (pv) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The parameter value of this CV` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;paramWeight (pw) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`Whether to use the paramValue, or float` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;twistValue (tv) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The twist value of this CV` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;twistWeight (tw) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`Whether to use the twist value, or interpolate` | | |
|&nbsp;&nbsp;&nbsp;&nbsp;useOrient (uo) | double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`Whether to use the orientation of this CV as part of the twist` | | |

## TwistTangent Node

This node Controls the behavior of a single bezier tangent control.

This node assumes the tangent being controlled is an outgoing tangent. So for incoming tangents, the "previous" and "next" cvs are swapped.

These nodes usually come in pairs. One for each twist spline segment. The in/out linear targets are connected in a cycle between them. Don't worry, it's not an *actual* cycle as the plugs aren't connected that way internally to the node.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
|out (out)| double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`The final output from this node. Usually connected to a tangent input on a TwistSpline node`|
|&nbsp;&nbsp;&nbsp;&nbsp;outX (ox)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output X component`|
|&nbsp;&nbsp;&nbsp;&nbsp;outY (oy)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output Y component`|
|&nbsp;&nbsp;&nbsp;&nbsp;outZ (oz)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output Z component`|
|smoothTan (st)| double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`The pure smooth tangent in un-oriented CV space. Not affected by weight or auto`|
|&nbsp;&nbsp;&nbsp;&nbsp;smoothTanX (stx)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The smoothTangent X component`|
|&nbsp;&nbsp;&nbsp;&nbsp;smoothTanY (sty)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The smoothTangent Y component`|
|&nbsp;&nbsp;&nbsp;&nbsp;smoothTanZ (stz)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The smoothTangent Z component`|
|outLinearTarget (lt)| double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`The target for next linear tangent`|
|&nbsp;&nbsp;&nbsp;&nbsp;outLinearTargetX (ltx)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget X component`|
|&nbsp;&nbsp;&nbsp;&nbsp;outLinearTargetY (lty)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget Y component`|
|&nbsp;&nbsp;&nbsp;&nbsp;outLinearTargetZ (ltz)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget Z component`|
| | | |
|parentInverseMatrix (pim)| matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;`The parent inverse matrix from the object connected to the output`|
|inTangent (it)| matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;`The user defined floating tangent matrix`|
|previousVertex (pv)| matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;`The CV that is on the opposite side of the current vertex`|
|currentVertex (cv)| matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;`The CV that this tangent is connect to`|
|nextVertex (nv)| matrix | Identity |
|&nbsp;&nbsp;&nbsp;&nbsp;`The CV that this tangent points towards`|
|inLinearTarget (nlt)| double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`Connect the outLinearTarget from another TwistTangent node controlling the same bezier segment here`|
|&nbsp;&nbsp;&nbsp;&nbsp;inLinearTargetX (nltx)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget X component`|
|&nbsp;&nbsp;&nbsp;&nbsp;inLinearTargetY (nlty)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget Y component`|
|&nbsp;&nbsp;&nbsp;&nbsp;inLinearTargetZ (nltz)| double | 0.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The linearTarget Z component`|
|auto (a)| double | 1.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;`Whether or not the output is controlled automatically by the CVs`|
|smooth (s)| double | 1.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;`Whether an automatic output is smooth or linear`|
|weight (w)| double | 1.0 |
|&nbsp;&nbsp;&nbsp;&nbsp;`The Length of the auto tangent. A weight of 1 is 1/3 of the distance between the current and next CVs`|




## RiderConstraint Node

A node that gets transformations at a given parameters along the spline. Generally, you will make many rider objects, and skin your geometry to those riders.

This node contains many convenience options. Normalization allows easy 0 to 1 parameterization no matter the length of the spline (as a spline that is unpinned will default to parameterizing by length). The globalOffset and globalSpread parameters are common sliding options. And useCycle allows you to constrain in loops so any rider that goes past the end parameter will loop back to the beginning, and vice-versa.

The multiple spline inputs allows you to switch between controlling splines. For instance, you could build a second spline with extra controls as the need arose, and swap control of the riders to that new spline.

Note: Despite its name, this isn't actually implemented as a constraint, but it behaves similarly.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
|rotateOrder (ro) | enum | XYZ |
|&nbsp;&nbsp;&nbsp;&nbsp;`Enum of the rotation order standard to Maya`|
|globalOffset (go) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;`A value added to all input parameters. This shifts everything connected to this constraint along the spline.`|
|globalSpread (gs) | double | 1.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;`A value multiplied by all input parameters. This spreads everything out (happens before the offset)`|
|useCycle (uc) |  | false | 
|&nbsp;&nbsp;&nbsp;&nbsp;`Whether or not to cycle the parameters once they go past the end. If not, they extrapolate linearly.`|
|normalize (n) | boolen | True | 
|&nbsp;&nbsp;&nbsp;&nbsp;`If true, then the input parameters are remapped so that (0, normValue) maps to (0, restLength) of the spline`|
|normValue (nv) | double | 1.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;`The remapped maximum value when normalizing`|
|inputSplines (is) | compound | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;`The group that is a spline and its corresponding weight.`|
|&nbsp;&nbsp;&nbsp;&nbsp;spline (s) | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`An input spline`|
|&nbsp;&nbsp;&nbsp;&nbsp;weight (w) | double | 1.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The constraint weight of that spline`|
|params (ps) | 
|&nbsp;&nbsp;&nbsp;&nbsp;`The parameters for the constraints, and their parent inverse matrices`|
|&nbsp;&nbsp;&nbsp;&nbsp;param (p) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The parameter where an object will stick to the spline.`|
|&nbsp;&nbsp;&nbsp;&nbsp;parentInverseMatrix (pim) | matrix | identity |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The parentInverseMatrix of the object sticking to the spline`|
| | |
|outputs (out) | 
|&nbsp;&nbsp;&nbsp;&nbsp;translate (t) | double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output translation`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;translateX (tx) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output translation X Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;translateY (ty) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output translation Y Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;translateZ (tz) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output translation Z Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;rotate (rot) | double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output rotation`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;rotateX (rotx) | angle | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output rotation X Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;rotateY (roty) | angle | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output rotation Y Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;rotateZ (rotz) | angle | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output rotation Z Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;scale (scl) | double3 | n/a |
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output scale`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;scaleX (sclx) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output scale X Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;scaleY (scly) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output scale Y Component`|
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;scaleZ (sclz) | double | 0.0 | 
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`The output scale Z Component`|











