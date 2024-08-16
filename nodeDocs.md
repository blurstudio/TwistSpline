## TwistSpline Node

This node implements the TwistSpline, which is a dynamically re-parameterizing bezier spline that interpolates twist along its length.

This TwistSpline can have any number of Control Vertices, and each control vertex has an incoming and outgoing tangent. The incoming tangent is ignored for the first CV, and the outgoing tangent is ignored for the last CV.

The OutputSpline plug is a custom type specific to this set of nodes.

The per-CV plugs control how the spline behaves and reparameterizes. The ControlVertex plug recieves the worldpace position and orientation of the CV itself. The InTangent and OutTangent are world matrices that define the bezier tangent positions and orientations for this CV. ParamValue and ParamWeight are used as a pair. ParamValue is the parameter that the currentCV will have when ParamWeight is 1.0. If ParamWeight is 0.0, then the parameter of this CV will float between it's neighbor's parameters (as a percentage of the lengths of its neighboring bezier spline segments). The parameterization algorithm works as long as any paramWeight is greater than 0.

TwistValue and TwistWeight work in a similar way. Twists are interpolated between values that have weight, and skip those that don't. Because this is handled as an interpolated float value, twist behavior allows for arbitrary values, and can twist past 360deg multiple times along the spline.

UseOrient is the "weight" of the ControlVertex worldspace orientation as part of the twist. Because of the limitations of Matrices and Quaternions, you can never get more than a 180 degree twist using orient behavior and popping can occur if you're not careful.

DebugDisplay and DebugScale show the orientation frames calculated along the spline and control their size.

The twistMultipler plug exists for backwards compatibility with bug in earlier versions of the code. The old multi-node auto-tangent setup accidentally built a left-handed twist matrix, thus the twist values went the wrong way. The new single-node auto-tangent builds the correct matrices, and thus this multipler must be set to 1.0 in that case.

ScaleCompensation exists because each input parameter would need a multipler attached to it to allow uniform scaling without changing behavior. This plug and the one on the rider handles all of that internally.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
|outputSpline (os) | TwistSpline | n/a |
|___`The custom spline data output` | | |
|outputNurbsCurve (onc) | NurbsCurve | n/a |
|___`A Maya-native NURBS curve output. Note: It will reparameterize by length, but it will not include twist data` | | |
|splineLength (sl) | double | 0.0 |
|___`The total length of the spline` | | |
|scaleCompensation (sclcmp) | double | 1.0 |
|___`The overall scale of the spline. This is required to make riding/pinning/offsets work correctly` | | |
|splineDisplay (sd) | bool | True |
|___`Whether to draw the spline` | | |
|debugDisplay (dd) | bool | False |
|___`Whether to show the debug axes` | | |
|debugScale (ds) | double | 1.0 |
|___`The size of the debug axes` | | |
|twistMultiplier (tm) | double | -1.0 |
|___`A multiplier on top of the intput TwistValue inputs. Exists for backwards compatibility` | | |
|vertexData (vd) | compound | n/a |
|___Inputs for all of the per-cv data | | |
|___inTangent (int) | matrix | Identity |
|______`The bezier tangent pointing towards the start of the spline` | | |
|___outTangent (ot) | matrix | Identity |
|______`The bezier tangent pointing towards the end of the spline` | | |
|___controlVertex (cv) | matrix | Identity |
|______`The control vertex` | | |
|___paramValue (pv) | double | 0.0 |
|______`The parameter value of this CV` | | |
|___paramWeight (pw) | double | 0.0 |
|______`Whether to use the paramValue, or float` | | |
|___twistValue (tv) | double | 0.0 |
|______`The twist value of this CV` | | |
|___twistWeight (tw) | double | 0.0 |
|______`Whether to use the twist value, or interpolate` | | |
|___useOrient (uo) | double | 0.0 |
|______`Whether to use the orientation of this CV as part of the twist` | | |


## TwistMultiTangent Node

This node controls the behavior of all automatic bezier tangents for a control.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
| vertData (vd)| compound | n/a |
| ___ `The input vertex user data` | | |
| ___vertMat (vm) | matrix | Identity |
| ______ `The Control Vertex world matrix input` | | |
| ___inParentInverseMatrix (ipim) | matrix | Identity |
| ______ `The parentInverseMatrix of the inTangent` | | |
| ___outParentInverseMatrix (opim) | matrix | Identity |
| ______ `The parentInverseMatrix of the outTangent` | | |
| ___twistParentInverseMatrix (tpim) | matrix | Identity |
| ______ `The parentInverseMatrix of the twist axis` | | |
| ___inTanWeight (itw) | double | 1.0 |
| ______`The Length of the in auto tangent. A weight of 1 is 1/3 of the distance between the current and next CVs`| | |
| ___outTanWeight (otw) | double | 1.0 |
| ______`The Length of the out auto tangent. A weight of 1 is 1/3 of the distance between the current and next CVs`| | |
| ___inSmooth (ism) | double | 1.0 |
| ______ `Whether an automatic in tangent is smooth or linear` | | |
| ___outSmooth (osm) | double | 1.0 |
| ______ `Whether an automatic out tangent is smooth or linear` | | |
| ___inAuto (iat) | double | 1.0 |
| ______ `Whether or not the output is controlled automatically by the CVs` | | |
| ___outAuto (oat) | double | 1.0 |
| ______ `Whether or not the output is controlled automatically by the CVs` | | |
| ___inTanMat (itm) | matrix | Identity |
| ______ `The worldspace rest position of the inTangent` | | |
| ___outTanMat (otm) | matrix | Identity |
| ______ `The worldspace rest position of the outTangent` | | |
| startTension (st) | double | 2.0 |
| ___ `How much the first tangent overshoots` | | |
| endTension (et) | double | 2.0 |
| ___ `How much the last tangent overshoots` | | |
| maxVertices (mv) | Int | 999 |
| ___ `The maximum number of vertices that will be calculated` | | |
| closed (cl) | Bool | False |
| ___ `Whether or not the tangents behave as a closed loop` | | |
| vertTans (vt) | compound |
| ___ `The output tangent data` | | |
| ___inVertTan (ivt) | double3 | n/a |
| ______ `The output inTangent` | | |
| ______inVertTanX (ivx) | double | 0.0 |
| _________ `The output inTangent X component` | | |
| ______inVertTanY (ivy) | double | 0.0 |
| _________ `The output inTangent Y component` | | |
| ______inVertTanZ (ivz) | double | 0.0 |
| _________ `The output inTangent Z component` | | |
| ___outVertTan (ovt) | double3 | n/a |
| ______ `The output outTangent` | | |
| ______outVertTanX (ovx) | double | 0.0 |
| _________ `The output outTangent X component` | | |
| ______outVertTanY (ovy) | double | 0.0 |
| _________ `The output outTangent Y component` | | |
| ______outVertTanZ (ovz) | double | 0.0 |
| _________ `The output outTangent Z component` | | |
| ___inTanLen (itl) | double | 0.0 |
| ______ `The computed length of the inTangent` | | |
| ___outTanLen (otl) | double | 0.0 |
| ______ `The computed length of the outTangent` | | |
| ___twistUp (tu) | double3 | n/a |
| ______ `The direction of "up" for the twist axis` | | |
| ______twistUpX (tux) | double | 0.0 |
| _________ `The direction of "up" for the twist axis X component` | | |
| ______twistUpY (tuy) | double | 0.0 |
| _________ `The direction of "up" for the twist axis Y component` | | |
| ______twistUpZ (tuz) | double | 0.0 |
| _________ `The direction of "up" for the twist axis Z component` | | |
| ___twistMat (tm) | matrix | Identity |
| ______ `The matrix for the twist axis that will keep it aligned with the auto tangents` | | |


## TwistTangent Node

### THIS NODE IS DEPRECATED. YOU SHOULD SWITCH TO THE SINGLE-NODE TANGENT SETUP 

This node Controls the behavior of a single bezier tangent control.

This node assumes the tangent being controlled is an outgoing tangent. So for incoming tangents, the "previous" and "next" cvs are swapped.

These nodes usually come in pairs. One for each twist spline segment. The in/out linear targets are connected in a cycle between them. Don't worry, it's not an *actual* cycle as the plugs aren't connected that way internally to the node.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
|out (out)| double3 | n/a |
|___`The final output from this node. Usually connected to a tangent input on a TwistSpline node`|
|___outX (ox)| double | 0.0 |
|______`The output X component`|
|___outY (oy)| double | 0.0 |
|______`The output Y component`|
|___outZ (oz)| double | 0.0 |
|______`The output Z component`|
|smoothTan (st)| double3 | n/a |
|___`The pure smooth tangent in un-oriented CV space. Not affected by weight or auto`|
|___smoothTanX (stx)| double | 0.0 |
|______`The smoothTangent X component`|
|___smoothTanY (sty)| double | 0.0 |
|______`The smoothTangent Y component`|
|___smoothTanZ (stz)| double | 0.0 |
|______`The smoothTangent Z component`|
|outLinearTarget (lt)| double3 | n/a |
|___`The target for next linear tangent`|
|___outLinearTargetX (ltx)| double | 0.0 |
|______`The linearTarget X component`|
|___outLinearTargetY (lty)| double | 0.0 |
|______`The linearTarget Y component`|
|___outLinearTargetZ (ltz)| double | 0.0 |
|______`The linearTarget Z component`|
| | | |
|parentInverseMatrix (pim)| matrix | Identity |
|___`The parent inverse matrix from the object connected to the output`|
|inTangent (it)| matrix | Identity |
|___`The user defined floating tangent matrix`|
|previousVertex (pv)| matrix | Identity |
|___`The CV that is on the opposite side of the current vertex`|
|currentVertex (cv)| matrix | Identity |
|___`The CV that this tangent is connect to`|
|nextVertex (nv)| matrix | Identity |
|___`The CV that this tangent points towards`|
|inLinearTarget (nlt)| double3 | n/a |
|___`Connect the outLinearTarget from another TwistTangent node controlling the same bezier segment here`|
|___inLinearTargetX (nltx)| double | 0.0 |
|______`The linearTarget X component`|
|___inLinearTargetY (nlty)| double | 0.0 |
|______`The linearTarget Y component`|
|___inLinearTargetZ (nltz)| double | 0.0 |
|______`The linearTarget Z component`|
|auto (a)| double | 1.0 |
|___`Whether or not the output is controlled automatically by the CVs`|
|smooth (s)| double | 1.0 |
|___`Whether an automatic output is smooth or linear`|
|weight (w)| double | 1.0 |
|___`The Length of the auto tangent. A weight of 1 is 1/3 of the distance between the current and next CVs`|

## RiderConstraint Node

A node that gets transformations at a given parameters along the spline. Generally, you will make many rider objects, and skin your geometry to those riders.

This node contains many convenience options. Normalization allows easy 0 to 1 parameterization no matter the length of the spline (as a spline that is unpinned will default to parameterizing by length). The globalOffset and globalSpread parameters are common sliding options. And useCycle allows you to constrain in loops so any rider that goes past the end parameter will loop back to the beginning, and vice-versa.

The multiple spline inputs allows you to switch between controlling splines. For instance, you could build a second spline with extra controls as the need arose, and swap control of the riders to that new spline.

ScaleCompensation exists because each input parameter would need a multipler attached to it to allow uniform scaling without changing behavior. This plug and the one on the spline handles all of that internally.

Note: Despite its name, this isn't actually implemented as a constraint, but it behaves similarly.

| Long Name (short name) | Type | Default |
| --- | --- | --- |
|rotateOrder (ro) | enum | XYZ |
|___`Enum of the rotation order standard to Maya`|
|globalOffset (go) | double | 0.0 |
|___`A value added to all input parameters. This shifts everything connected to this constraint along the spline.`|
|globalSpread (gs) | double | 1.0 |
|___`A value multiplied by all input parameters. This spreads everything out (happens before the offset)`|
|scaleCompensation (sclcmp) | double | 1.0 |
|___`The overall scale of the spline. This is required to make riding/pinning/offsets work correctly` | | |
|useCycle (uc) |  | false |
|___`Whether or not to cycle the parameters once they go past the end. If not, they extrapolate linearly.`|
|normalize (n) | boolen | True |
|___`If true, then the input parameters are remapped so that (0, normValue) maps to (0, restLength) of the spline`|
|normValue (nv) | double | 1.0 |
|___`The remapped maximum value when normalizing`|
|inputSplines (is) | compound | n/a |
|___`The group that is a spline and its corresponding weight.`|
|___spline (s) |
|______`An input spline`|
|___weight (w) | double | 1.0 |
|______`The constraint weight of that spline`|
|params (ps) |
|___`The parameters for the constraints, and their parent inverse matrices`|
|___param (p) | double | 0.0 |
|______`The parameter where an object will stick to the spline.`|
|___parentInverseMatrix (pim) | matrix | identity |
|______`The parentInverseMatrix of the object sticking to the spline`|
| | |
|outputs (out) |
|___translate (t) | double3 | n/a |
|______`The output translation`|
|______translateX (tx) | double | 0.0 |
|_________`The output translation X Component`|
|______translateY (ty) | double | 0.0 |
|_________`The output translation Y Component`|
|______translateZ (tz) | double | 0.0 |
|_________`The output translation Z Component`|
|___rotate (rot) | double3 | n/a |
|______`The output rotation`|
|______rotateX (rotx) | angle | 0.0 |
|_________`The output rotation X Component`|
|______rotateY (roty) | angle | 0.0 |
|_________`The output rotation Y Component`|
|______rotateZ (rotz) | angle | 0.0 |
|_________`The output rotation Z Component`|
|___scale (scl) | double3 | n/a |
|______`The output scale`|
|______scaleX (sclx) | double | 0.0 |
|_________`The output scale X Component`|
|______scaleY (scly) | double | 0.0 |
|_________`The output scale Y Component`|
|______scaleZ (sclz) | double | 0.0 |
|_________`The output scale Z Component`|

