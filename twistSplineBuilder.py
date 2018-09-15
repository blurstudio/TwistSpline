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

def mkTwistSplineControllers(pfx, numCVs, spread):
	# Make bases for the controllers
	v = 0.5
	cube = cmds.curve(degree=1, p=[
		( v, v, v), ( v,-v, v), (-v,-v, v), (-v, v, v),
		( v, v, v), ( v, v,-v), ( v,-v,-v), ( v,-v, v),
		( v,-v,-v), (-v,-v,-v), (-v,-v, v), (-v,-v,-v),
		(-v, v,-v), (-v, v, v), (-v, v,-v), ( v, v,-v),
		] 
	)
	v = 0.25
	circle = cmds.circle(radius=v, constructionHistory=False)[0]
	v = 0.25
	square = cmds.curve(degree=1, p=[( v, v, 0), ( v,-v, 0), (-v,-v, 0), (-v, v, 0), ( v, v, 0)])
	v = 0.5
	s = 0.5
	triangle = cmds.curve(degree=1, p=[( -v, s, 0), ( v, s, 0), (0,2*v+s, 0), (-v, s, 0)])

	v = 0.15
	s = 1.108
	o = 3
	star = cmds.curve(degree=3, periodic=True,
		p=[
			( v, -v+o, 0), ( 0, -s+o, 0), (-v, -v+o, 0), (-s,  0+o, 0),
			(-v,  v+o, 0), ( 0,  s+o, 0), ( v,  v+o, 0), ( s,  0+o, 0),
			( v, -v+o, 0), ( 0, -s+o, 0), (-v, -v+o, 0)
		],
		k=[-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
	)
	
	twistHideProps = ['.tx', '.ty', '.tz','.rx', '.ry', '.rz', '.sx', '.sy', '.sz', '.v'] 

	master = cmds.duplicate(star, name="{}SplineGlobal".format(pfx))[0]
	cmds.addAttr(master, longName="Offset", attributeType='double', defaultValue=0.0)
	cmds.setAttr(master + '.Offset', edit=True, keyable=True)
	cmds.addAttr(master, longName="Stretch", attributeType='double', defaultValue=1.0, minValue=0.0001)
	cmds.setAttr(master + '.Stretch', edit=True, keyable=True)
	for h in twistHideProps:
		cmds.setAttr(master + h, lock=True, keyable=False, channelBox=False)

	# make the requested number of CV's
	# don't hide the .rx attribute
	cvs = []
	tws = []
	for i in range(numCVs):
		cv = cmds.duplicate(cube, name="{}SplineControl_{}".format(pfx, i+1))[0]
		cmds.addAttr(cv, longName="Pin", attributeType='double', defaultValue=0.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(cv + '.Pin', edit=True, keyable=True)
		cmds.addAttr(cv, longName="PinParam", attributeType='double', defaultValue=0.0, minValue=0.0)
		cmds.setAttr(cv + '.PinParam', edit=True, keyable=True)
		tw = cmds.duplicate(triangle, name="{}SplineTwist_{}".format(pfx, i+1))[0]
		for h in twistHideProps:
			if h == ".rx": continue
			cmds.setAttr(tw + h, lock=True, keyable=False, channelBox=False)
		cmds.addAttr(tw, longName="UseTwist", attributeType='double', defaultValue=0.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(tw + '.UseTwist', edit=True, keyable=True)
		cmds.parent(tw, cv)
		cmds.xform(cv, translation=(spread*3*i, 0, 0))
		cvs.append(cv)
		tws.append(tw)

	cmds.group(cvs, name="Controls")

	# make the tangents and auto-tangents
	oTans = []
	iTans = []
	aoTans = []
	aiTans = []
	v = 0.25
	for i in range(numCVs - 1):
		# make oTan, and iTan
		oTan = cmds.duplicate(circle, name="{}OutTangent_{}".format(pfx, i+1))[0]
		cmds.addAttr(oTan, longName="Auto", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(oTan + '.Auto', edit=True, keyable=True)
		cmds.addAttr(oTan, longName="Smooth", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(oTan + '.Smooth', edit=True, keyable=True)
		cmds.addAttr(oTan, longName="Weight", attributeType='double', defaultValue=1.0, minValue=0.0001, maxValue=5.0)
		cmds.setAttr(oTan + '.Weight', edit=True, keyable=True)
		cmds.xform(oTan, translation=(spread*(3*i+1), 0, 0))

		iTan = cmds.duplicate(square, name="{}InTangent_{}".format(pfx, i+2))[0]
		cmds.addAttr(iTan, longName="Auto", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(iTan + '.Auto', edit=True, keyable=True)
		cmds.addAttr(iTan, longName="Smooth", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(iTan + '.Smooth', edit=True, keyable=True)
		cmds.addAttr(iTan, longName="Weight", attributeType='double', defaultValue=1.0, minValue=0.0001, maxValue=5.0)
		cmds.setAttr(iTan + '.Weight', edit=True, keyable=True)
		cmds.xform(iTan, translation=(spread*(3*i+2), 0, 0))

		aoTan = cmds.spaceLocator(name="{}AutoOutTangent_{}".format(pfx, i+1))[0]
		aiTan = cmds.spaceLocator(name="{}AutoInTangent_{}".format(pfx, i+2))[0]
		aoTanShape = cmds.listRelatives(aoTan)[0]
		aiTanShape = cmds.listRelatives(aiTan)[0]
		cmds.setAttr("{}.localScale".format(aoTanShape), v, v, v, type="double3")
		cmds.setAttr("{}.localScale".format(aiTanShape), v, v, v, type="double3")
		cmds.xform(aoTan, translation=(spread*(3*i+1), 0, 0))
		cmds.xform(aiTan, translation=(spread*(3*i+2), 0, 0))
		oTans.append(oTan)
		iTans.append(iTan)
		aoTans.append(aoTan)
		aiTans.append(aiTan)
		cmds.parent(oTan, cvs[i])
		cmds.parent(aoTan, cvs[i])
		cmds.parent(iTan, cvs[i+1])
		cmds.parent(aiTan, cvs[i+1])

	cmds.delete((cube, circle, square, triangle, star))
	return cvs, oTans, iTans, aoTans, aiTans, tws, master

def connectTwistSplineTangents(cvs, oTans, iTans, aoTans, aiTans):
	# connect all the in/out tangents
	otts = []
	itts = []
	for i in range(len(cvs)-1):
		ott = cmds.createNode("twistTangent")
		otts.append(ott)
		cmds.connectAttr("{}.worldMatrix[0]".format(oTans[i]), "{}.inTangent".format(ott))
		cmds.connectAttr("{}.Auto".format(oTans[i]), "{}.auto".format(ott))
		cmds.connectAttr("{}.Smooth".format(oTans[i]), "{}.smooth".format(ott))
		cmds.connectAttr("{}.Weight".format(oTans[i]), "{}.weight".format(ott))
		if i > 0:
			cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i-1]), "{}.previousVertex".format(ott))
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]),   "{}.currentVertex".format(ott))
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+1]), "{}.nextVertex".format(ott))
		cmds.setAttr("{}.Auto".format(oTans[i]), 1.0)

		itt = cmds.createNode("twistTangent")
		itts.append(itt)
		cmds.connectAttr("{}.worldMatrix[0]".format(iTans[i]), "{}.inTangent".format(itt))
		cmds.connectAttr("{}.Auto".format(iTans[i]), "{}.auto".format(itt))
		cmds.connectAttr("{}.Smooth".format(iTans[i]), "{}.smooth".format(itt))
		cmds.connectAttr("{}.Weight".format(iTans[i]), "{}.weight".format(itt))
		if i+2 < len(cvs):
			cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+2]), "{}.previousVertex".format(itt))
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i+1]),	 "{}.currentVertex".format(itt))
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]), "{}.nextVertex".format(itt))
		cmds.setAttr("{}.Auto".format(iTans[i]), 1.0)

		cmds.connectAttr("{}.outLinearTarget".format(itt), "{}.inLinearTarget".format(ott))
		cmds.connectAttr("{}.outLinearTarget".format(ott), "{}.inLinearTarget".format(itt))

	cmds.setAttr("{}.Smooth".format(oTans[0]), 0.0)
	cmds.setAttr("{}.Smooth".format(iTans[-1]), 0.0)

	# connect the tangent constraints to the auto-tangent objects (for visualization)
	# and connect the pims so we can reparent the auto tangents as we see fit
	for a, t in zip(aoTans, otts):
		cmds.connectAttr("{0}.out".format(t), "{}.translate".format(a))
		cmds.connectAttr("{}.parentInverseMatrix[0]".format(a), "{}.parentInverseMatrix".format(t))

	for a, t in zip(aiTans, itts):
		cmds.connectAttr("{0}.out".format(t), "{}.translate".format(a))
		cmds.connectAttr("{}.parentInverseMatrix[0]".format(a), "{}.parentInverseMatrix".format(t))

def buildTwistSpline(pfx, cvs, aoTans, aiTans, tws, maxParam):
	numCVs = len(cvs)

	# build the spline object and set the spline Params
	splineTfm = cmds.createNode("transform", name="{0}TwistSpline".format(pfx))
	spline = cmds.createNode("twistSpline", parent=splineTfm, name="{0}TwistSplineShape".format(pfx))
	for i in range(numCVs):
		if i > 0:
			cmds.connectAttr("{}.worldMatrix[0]".format(aiTans[i-1]), "{}.vertexData[{}].inTangent".format(spline, i))
		
		if i < numCVs - 1:
			cmds.connectAttr("{}.worldMatrix[0]".format(aoTans[i]), "{}.vertexData[{}].outTangent".format(spline, i))
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]), "{}.vertexData[{}].controlVertex".format(spline, i))
		cmds.connectAttr("{}.Pin".format(cvs[i]), "{}.vertexData[{}].paramWeight".format(spline, i))
		cmds.connectAttr("{}.PinParam".format(cvs[i]), "{}.vertexData[{}].paramValue".format(spline, i))
		cmds.connectAttr("{}.UseTwist".format(tws[i]), "{}.vertexData[{}].twistWeight".format(spline, i))
		cmds.connectAttr("{}.rotateX".format(tws[i]), "{}.vertexData[{}].twistValue".format(spline, i))
		cmds.setAttr("{}.PinParam".format(cvs[i]), (i*maxParam) / (numCVs-1.0))

	cmds.setAttr("{}.Pin".format(cvs[0]), 1.0)
	cmds.setAttr("{}.UseTwist".format(tws[0]), 1.0)

	return spline

def buildRiders(pfx, spline, master, numJoints):
	# make the joints (and locators so I can see orientations)
	# just make them at origin. The constraint will put them in place
	if numJoints == 0:
		return None, None, None

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

	# build the constraint object
	cnst = cmds.createNode("riderConstraint")
	cmds.connectAttr("{}.Offset".format(master), "{}.globalOffset".format(cnst))
	cmds.connectAttr("{}.Stretch".format(master), "{}.globalSpread".format(cnst))

	# connect the constraints
	cmds.connectAttr("{}.outputSpline".format(spline), "{}.inputSplines[0].spline".format(cnst))
	for i in range(len(jPars)):
		if len(jPars) == 1:
			cmds.setAttr("{0}.params[{1}].param".format(cnst, i), 0.5)
		else:
			cmds.setAttr("{0}.params[{1}].param".format(cnst, i), i / (numJoints-1.0))
		cmds.connectAttr("{}.parentInverseMatrix[0]".format(jPars[i]), "{0}.params[{1}].parentInverseMatrix".format(cnst, i))
		cmds.connectAttr("{0}.outputs[{1}].translate".format(cnst, i), "{}.translate".format(jPars[i]))
		cmds.connectAttr("{0}.outputs[{1}].rotate".format(cnst, i), "{}.rotate".format(jPars[i]))
		cmds.connectAttr("{0}.outputs[{1}].scale".format(cnst, i), "{}.scale".format(jPars[i]))

	group = cmds.group(jPars, name="Joints")
	return jPars, joints, group

def makeTwistSpline(pfx, numCVs, numJoints=10, maxParam=None, spread=1.0):
	if maxParam is None:
		maxParam = (numCVs - 1)
	maxParam *= 3.0 * spread

	cvs, oTans, iTans, aoTans, aiTans, tws, master = mkTwistSplineControllers(pfx, numCVs, spread)
	connectTwistSplineTangents(cvs, oTans, iTans, aoTans, aiTans)
	spline = buildTwistSpline(pfx, cvs, aoTans, aiTans, tws, maxParam)
	jPars, joints, group = buildRiders(pfx, spline, master, numJoints)


def addNewSplineToRider(pfx, numCVs, existingRider, maxParam=None, spread=1.0):
	if maxParam is None:
		maxParam = (numCVs - 1)
	maxParam *= 3.0 * spread

	cvs, oTans, iTans, aoTans, aiTans, tws, master = mkTwistSplineControllers(pfx, numCVs, spread)
	connectTwistSplineTangents(cvs, oTans, iTans, aoTans, aiTans)
	spline = buildTwistSpline(pfx, cvs, aoTans, aiTans, tws, maxParam)

	controlObject = cmds.createNode("transform", name="SwitchControl")

	cmds.connectAttr("{}.outputSpline".format(spline), "{}.inputSplines[1].spline".format(existingRider))

	cmds.addAttr(controlObject, longName="SplineBlend", at="double", min=0, max=1)
	cmds.setAttr("{0}.SplineBlend".format(controlObject), e=True, keyable=True)
	cmds.setAttr("{0}.SplineBlend".format(controlObject), 1)
	pma = cmds.shadingNode("plusMinusAverage", asUtility=True)
	cmds.setAttr("{}.operation".format(pma), 2)
	cmds.setAttr("{}.input1D[0]".format(pma), 1)
	cmds.connectAttr("{0}.SplineBlend".format(controlObject), "{}.input1D[1]".format(pma), force=True)
	cmds.connectAttr("{0}.SplineBlend".format(controlObject), "{0}.inputSplines[0].weight".format(existingRider), force=True)
	cmds.connectAttr("{0}.output1D".format(pma), "{0}.inputSplines[1].weight".format(existingRider), force=True)

	# For demo video purposes, shift things around so they line up better
	# This kind of stuff should be handled by a nice UI, but that's for later

	cmds.connectAttr( "spline_A_SplineControl_1.translate", "spline_B_SplineControl_1.translate")
	cmds.connectAttr( "spline_A_SplineControl_1.rotate",    "spline_B_SplineControl_1.rotate")
	cmds.connectAttr( "spline_A_SplineControl_1.PinParam",  "spline_B_SplineControl_1.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_1.Pin",       "spline_B_SplineControl_1.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_1.rotateX",    "spline_B_SplineTwist_1.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_1.UseTwist",    "spline_B_SplineTwist_1.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_2.translate", "spline_B_SplineControl_2.translate")
	cmds.connectAttr( "spline_A_SplineControl_2.rotate",    "spline_B_SplineControl_2.rotate")
	cmds.connectAttr( "spline_A_SplineControl_2.PinParam",  "spline_B_SplineControl_2.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_2.Pin",       "spline_B_SplineControl_2.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_2.rotateX",    "spline_B_SplineTwist_2.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_2.UseTwist",    "spline_B_SplineTwist_2.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_3.translate", "spline_B_SplineControl_4.translate")
	cmds.connectAttr( "spline_A_SplineControl_3.rotate",    "spline_B_SplineControl_4.rotate")
	cmds.connectAttr( "spline_A_SplineControl_3.PinParam",  "spline_B_SplineControl_4.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_3.Pin",       "spline_B_SplineControl_4.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_3.rotateX",    "spline_B_SplineTwist_4.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_3.UseTwist",    "spline_B_SplineTwist_4.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_4.translate", "spline_B_SplineControl_5.translate")
	cmds.connectAttr( "spline_A_SplineControl_4.rotate",    "spline_B_SplineControl_5.rotate")
	cmds.connectAttr( "spline_A_SplineControl_4.PinParam",  "spline_B_SplineControl_5.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_4.Pin",       "spline_B_SplineControl_5.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_4.rotateX",    "spline_B_SplineTwist_5.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_4.UseTwist",    "spline_B_SplineTwist_5.UseTwist")

	cmds.hide("spline_B_SplineControl_1", "spline_B_SplineControl_2", "spline_B_SplineControl_4", "spline_B_SplineControl_5")

if __name__ == "__main__":
	#cmds.file(force=True, newFile=True)
	#makeTwistSpline("spline_B_", 5, 0, spread=3 * 0.75)
	#makeTwistSpline("spline_B_", 5, 0, 3)

	riders = ["spline_A_JointPar_{0}".format(i + 1) for i in range(100)]
	addNewSplineToRider("spline_B_", 5, "riderConstraint1", spread=3 * 0.75)



