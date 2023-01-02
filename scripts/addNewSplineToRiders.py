


"""
# skeleton of what should happen when adding a second spline to blend inbetween
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
	cmds.connectAttr( "spline_A_SplineControl_1.rotate",	"spline_B_SplineControl_1.rotate")
	cmds.connectAttr( "spline_A_SplineControl_1.PinParam",	"spline_B_SplineControl_1.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_1.Pin",		"spline_B_SplineControl_1.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_1.rotateX",    "spline_B_SplineTwist_1.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_1.UseTwist",	"spline_B_SplineTwist_1.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_2.translate", "spline_B_SplineControl_2.translate")
	cmds.connectAttr( "spline_A_SplineControl_2.rotate",	"spline_B_SplineControl_2.rotate")
	cmds.connectAttr( "spline_A_SplineControl_2.PinParam",	"spline_B_SplineControl_2.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_2.Pin",		"spline_B_SplineControl_2.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_2.rotateX",    "spline_B_SplineTwist_2.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_2.UseTwist",	"spline_B_SplineTwist_2.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_3.translate", "spline_B_SplineControl_4.translate")
	cmds.connectAttr( "spline_A_SplineControl_3.rotate",	"spline_B_SplineControl_4.rotate")
	cmds.connectAttr( "spline_A_SplineControl_3.PinParam",	"spline_B_SplineControl_4.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_3.Pin",		"spline_B_SplineControl_4.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_3.rotateX",    "spline_B_SplineTwist_4.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_3.UseTwist",	"spline_B_SplineTwist_4.UseTwist")

	cmds.connectAttr( "spline_A_SplineControl_4.translate", "spline_B_SplineControl_5.translate")
	cmds.connectAttr( "spline_A_SplineControl_4.rotate",	"spline_B_SplineControl_5.rotate")
	cmds.connectAttr( "spline_A_SplineControl_4.PinParam",	"spline_B_SplineControl_5.PinParam")
	cmds.connectAttr( "spline_A_SplineControl_4.Pin",		"spline_B_SplineControl_5.Pin")
	cmds.connectAttr( "spline_A_SplineTwist_4.rotateX",    "spline_B_SplineTwist_5.rotateX")
	cmds.connectAttr( "spline_A_SplineTwist_4.UseTwist",	"spline_B_SplineTwist_5.UseTwist")

	cmds.hide("spline_B_SplineControl_1", "spline_B_SplineControl_2", "spline_B_SplineControl_4", "spline_B_SplineControl_5")

if __name__ == "__main__":
	#cmds.file(force=True, newFile=True)
	#makeTwistSpline("spline_B_", 5, 0, spread=3 * 0.75)
	#makeTwistSpline("spline_B_", 5, 0, 3)

	riders = ["spline_A_JointPar_{0}".format(i + 1) for i in range(100)]
	addNewSplineToRider("spline_B_", 5, "riderConstraint1", spread=3 * 0.75)
"""
