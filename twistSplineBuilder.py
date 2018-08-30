'''
MIT License

Copyright (c) 2018 Blur Studio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

from maya import cmds

#cmds.loadPlugin("TwistSpline.mll")


def makeTwistSpline(pfx, numCVs, numJoints=10, maxParam=None):
    if maxParam is None:
        maxParam = (numCVs - 1) * 10

    # Make a CV Cube base
    cube = cmds.polyCube(constructionHistory=False)[0]
    # Make a Tan sphere base
    sphere = cmds.polySphere(radius=0.25, sx=8, sy=8, constructionHistory=False)[0]

    # make the requested number of CV's
    cvs = []
    for i in range(numCVs):
        cv = cmds.duplicate(cube, name="{}SplineControl_{}".format(pfx, i+1))[0]
        cmds.xform(cv, translation=(3*i, 0, 0))
        cvs.append(cv)

    # make the tangents and auto-tangents
    oTans = []
    iTans = []
    aoTans = []
    aiTans = []
    for i in range(numCVs - 1):
        # make oTan, and iTan
        oTan = cmds.duplicate(sphere, name="{}OutTangent_{}".format(pfx, i+1))[0]
        iTan = cmds.duplicate(sphere, name="{}InTangent_{}".format(pfx, i+2))[0]
        cmds.xform(oTan, translation=(3*i+1, 0, 0))
        cmds.xform(iTan, translation=(3*i+2, 0, 0))
        aoTan = cmds.spaceLocator(name="{}AutoOutTangent_{}".format(pfx, i+1))[0]
        aiTan = cmds.spaceLocator(name="{}AutoInTangent_{}".format(pfx, i+2))[0]
        cmds.xform(aoTan, translation=(3*i+1, 0, 0))
        cmds.xform(aiTan, translation=(3*i+2, 0, 0))
        oTans.append(oTan)
        iTans.append(iTan)
        aoTans.append(aoTan)
        aiTans.append(aiTan)
        cmds.parent(oTan, cvs[i])
        cmds.parent(iTan, cvs[i+1])

    # make the joints (or locators?)
    # just make them at origin. The constraint will put them in place
    jPars = []
    joints = []
    for i in range(numJoints):
        jp = cmds.spaceLocator(name="{}JointPar_{}".format(pfx, i+1))[0] # just for viewing
        cmds.setAttr("{}Shape.localScaleX".format(jp), 0.25)
        cmds.setAttr("{}Shape.localScaleY".format(jp), 0.25)
        cmds.setAttr("{}Shape.localScaleZ".format(jp), 0.25)
        j = cmds.createNode("joint", name="{}Joint_{}".format(pfx, i+1))
        cmds.parent(j, jp)
        jPars.append(jp)
        joints.append(j)

    cmds.hide(joints)

    # connect all the in/out tangents
    otts = []
    itts = []
    for i in range(numCVs-1):
        ott = cmds.createNode("twistTangent")
        otts.append(ott)
        cmds.connectAttr("{}.worldMatrix[0]".format(oTans[i]), "{}.inTangent".format(ott))
        if i > 0:
            cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i-1]), "{}.previousVertex".format(ott))
        cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]),   "{}.currentVertex".format(ott))
        cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+1]), "{}.nextVertex".format(ott))
        cmds.setAttr("{}.auto".format(ott), 1.0)

        itt = cmds.createNode("twistTangent")
        itts.append(itt)
        cmds.connectAttr("{}.worldMatrix[0]".format(iTans[i]), "{}.inTangent".format(itt))
        if i+2 < len(cvs):
            cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+2]), "{}.previousVertex".format(itt))
        cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+1]),   "{}.currentVertex".format(itt))
        cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]), "{}.nextVertex".format(itt))
        cmds.setAttr("{}.auto".format(itt), 1.0)

        cmds.connectAttr("{}.outLinearTarget".format(itt), "{}.inLinearTarget".format(ott))
        cmds.connectAttr("{}.outLinearTarget".format(ott), "{}.inLinearTarget".format(itt))

    cmds.setAttr("{}.smooth".format(otts[0]), 0.0)
    cmds.setAttr("{}.smooth".format(itts[-1]), 0.0)

    # connect the tangent constraints to the auto-tangent objects (for visualization)
    for a, t in zip(aoTans, otts):
        cmds.connectAttr("{0}.out".format(t), "{}.translate".format(a))

    for a, t in zip(aiTans, itts):
        cmds.connectAttr("{0}.out".format(t), "{}.translate".format(a))

    # build the spline object and set the spline Params
    spline = cmds.createNode("twistSpline")

    cmds.connectAttr("{}.worldMatrix[0]".format(cvs[0]), "{}.firstVertex".format(spline))
    for i in range(numCVs-1):
        cmds.connectAttr("{}.worldMatrix[0]".format(aoTans[i]), "{}.vertexData[{}].inTangent".format(spline, i))
        cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+1]), "{}.vertexData[{}].controlVertex".format(spline, i))
        cmds.connectAttr("{}.worldMatrix[0]".format(aiTans[i]), "{}.vertexData[{}].outTangent".format(spline, i))
        cmds.setAttr("{}.vertexData[{}].restLength".format(spline, i), ((i+1.0)*maxParam) / (numCVs-1.0))
    cmds.setAttr("{}.vertexData[{}].lock".format(spline, numCVs-2), 1.0)


    # build the constraint object
    rider = cmds.createNode("riderConstraint")

    # connect the constraints
    cmds.connectAttr("{}.outputSpline".format(spline), "{}.inputSplines[0].spline".format(rider))
    for i in range(len(jPars)):
        cmds.setAttr("{0}.param[{1}]".format(rider, i), i / (numJoints-1.0))
        cmds.connectAttr("{0}.outputs[{1}].translate".format(rider, i), "{}.translate".format(jPars[i]))
        cmds.connectAttr("{0}.outputs[{1}].rotate".format(rider, i), "{}.rotate".format(jPars[i]))
        cmds.connectAttr("{0}.outputs[{1}].scale".format(rider, i), "{}.scale".format(jPars[i]))



    cmds.delete((cube, sphere))



cmds.file(force=True, newFile=True)
makeTwistSpline("spline_A", 4, 100)
makeTwistSpline("spline_B", 5, 0)




