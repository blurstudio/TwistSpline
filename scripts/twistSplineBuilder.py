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

from maya import cmds, OpenMaya

# Naming Convention
DFM_ORG_FMT = "Org_X_X_{0}Jnts"  # Deformer Organizer
DFM_BFR_FMT = "Hbfr_X_{0}Rider_Part{1:02d}"  # Rider Buffer
DFM_FMT = "Dfm_X_{0}Rider_Part{1:02d}"  # Deformer
SPLINE_FMT = "Rig_X_{0}Spline_Drv"  # Spline name
MASTER_FMT = "Ctrl_X_{}SplineGlobal_Part"  # Global control
CTRL_ORG_FMT = "Org_X_{0}_Ctrls"  # Control organizer
BFR_CV_FMT = "Hbfr_X_{0}Spline_Part{1:02d}"  # CV Buffer
CTRL_CV_FMT = "Ctrl_X_{0}Spline_Part{1:02d}"  # CV
BFR_TWIST_FMT = "Hbfr_X_{0}Twist_Part{1:02d}"  # Twist Buffer
CTRL_TWIST_FMT = "Ctrl_X_{0}Twist_Part{1:02d}"  # Twist
CTRL_INTAN_FMT = "Ctrl_X_{0}InTangent_Part{1:02d}"  # In-Tangent
CTRL_OUTTAN_FMT = "Ctrl_X_{0}OutTangent_Part{1:02d}"  # Out-Tangent
BFR_AINTAN_FMT = "Hbfr_X_{0}AutoInTangent_Part{1:02d}"  # Auto In-Tangent Buffer
BFR_AOUTTAN_FMT = "Hbfr_X_{0}AutoOutTangent_Part{1:02d}"  # Auto Out-Tangent Buffer

def makeLinkLine(sourceNode, destNode, selectNode=None):
	""" Draw a line between two nodes. Clicking the line selects the target object

	Arguments:
		sourceNode (str): The start of the line
		destNode (str): The end of the line
		selectNode (str): The object that gets selected. Defaults to sourceNode

	Returns:
		str: The line's shape node

	"""
	if selectNode is None:
		selectNode = sourceNode

	lineTfm = cmds.curve(d=1, p=([0, 0, 0], [0, 0, 1]), k=(0, 1))
	lineShape = cmds.listRelatives(lineTfm, s=True, path=True)[0]
	cmds.parent(lineShape, selectNode, r=True, s=True, nc=True)
	cmds.delete(lineTfm)

	for idx, node in enumerate([destNode, sourceNode]):
		if node == selectNode:
			# If so, we can skip all the connections and just set the control point
			# to the local rotPivot, and leave it there
			rotPivot = cmds.xform(sourceNode, q=True, objectSpace=True, rotatePivot=True)
			cmds.setAttr(lineShape + ".controlPoints[{0}]".format(idx), *rotPivot)
		else:
			worldMatrix = cmds.createNode('pointMatrixMult', name=node + "_linkCurveWorldMat")
			inverseMatrix = cmds.createNode('pointMatrixMult', name=selectNode + "_linkCurveWorldMat")
			rotPivot = cmds.xform(node, q=True, objectSpace=True, rotatePivot=True)

			cmds.connectAttr(node + ".worldMatrix", worldMatrix + ".inMatrix", f=True)
			cmds.setAttr(worldMatrix + '.inPoint', *rotPivot)
			cmds.connectAttr(selectNode + ".worldInverseMatrix", inverseMatrix + ".inMatrix", f=True)

			cmds.connectAttr(worldMatrix + ".output", inverseMatrix + ".inPoint", f=True)
			cmds.connectAttr(inverseMatrix + ".output", lineShape + ".controlPoints[{0}]".format(idx), f=True)

	cmds.setAttr(lineShape + ".overrideEnabled", 1)
	cmds.setAttr(lineShape + ".overrideColor", 3)
	cmds.rename(lineShape, sourceNode + "Shape")

	for node in [worldMatrix, inverseMatrix]:
		cmds.setAttr(node + ".isHistoricallyInteresting", False)

	return lineShape

def _mkMasterHarbieControllers(scale=1.0):
	""" Make the master objects so we can duplicate them to be the controllers
	This function uses the internal blur locators

	Returns:
		cvCtrl: The CV controller master
		outTanCtrl: The out-tangent controller master
		inTanCtrl: The in-tangent controller master
		twistCtrl: The twist controller master
		masterCtrl: The top controller master
	"""
	# Blur Specific locators
	from dcc.maya.cast import toPath
	from dcc.maya.icon import createHarbieLocator as cloc

	cvCtrl = toPath(cloc("Cube", "TMP_SplineCV", None, None, size=1.0*scale, color=[1, 0.6, 0]))
	oTanCtrl = toPath(cloc("Cross", "TMP_OTan", None, None, size=0.4*scale, ro=[0, 0, 0], color=[1, 0.6, 0.6]))
	iTanCtrl = toPath(cloc("Square", "TMP_ITan", None, None, size=0.4*scale, ro=[90, 0, 0], color=[0, 1, 1]))
	twistCtrl = toPath(cloc("Compass", "TMP_SplineTwist", None, None, size=1.5*scale, ro=[-90, 90, 0], color=[0.5, 0, 1]))
	masterCtrl = toPath(cloc("CrossArrow", "TMP_SplineGlobal", None, None, size=3.0*scale, ro=[90, 0, 0], color=[0, 1, 0]))
	return cvCtrl, oTanCtrl, iTanCtrl, twistCtrl, masterCtrl

def _mkMasterControllers(scale=1.0):
	""" Make the master objects so we can duplicate them to be the controllers

	Returns:
		cvCtrl: The CV controller master
		outTanCtrl: The out-tangent controller master
		inTanCtrl: The in-tangent controller master
		twistCtrl: The twist controller master
		masterCtrl: The top controller master
	"""
	v = 0.5 * scale
	cvCtrl = cmds.curve(
		degree=1,
		p=[
			# top loop
			(v,  v,  v), (v, -v,  v), (-v, -v,  v), (-v,  v,  v), (v,  v,  v),
			# go to the bottom layer
			(v,  v, -v),
			# One side of the bottom, and a vertical leg *3
			(v, -v, -v),  (v, -v,  v),  (v, -v, -v),
			(-v, -v, -v), (-v, -v,  v), (-v, -v, -v),
			(-v,  v, -v), (-v,  v,  v), (-v,  v, -v),
			# Close the bottom
			(v,  v, -v),
		])

	v = 0.25 * scale
	outTanCtrl = cmds.circle(radius=v, constructionHistory=False)[0]

	v = 0.25 * scale
	inTanCtrl = cmds.curve(degree=1, p=[(v, v, 0), (v, -v, 0), (-v, -v, 0), (-v, v, 0), (v, v, 0)])

	v = 0.5 * scale
	s = 0.5 * scale
	twistCtrl = cmds.curve(degree=1, p=[(-v, s, 0), (v, s, 0), (0, 2 * v + s, 0), (-v, s, 0)])

	v = 0.15 * scale
	s = 1.108 * scale
	o = 3 * scale
	masterCtrl = cmds.curve(
		degree=3,
		periodic=True,
		p=[
			(v, -v + o, 0), (0, -s + o, 0), (-v, -v + o, 0), (-s, 0 + o, 0),
			(-v, v + o, 0), (0,  s + o, 0), (v,   v + o, 0), (s,  0 + o, 0),
			(v, -v + o, 0), (0, -s + o, 0), (-v, -v + o, 0)
		],
		k=[-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
	return cvCtrl, outTanCtrl, inTanCtrl, twistCtrl, masterCtrl

def mkTwistSplineControllers(pfx, numCVs, spread, closed=False):
	""" Make and position all the controller objects

	Arguments:
		pfx (str): The user name of the spline. Will be formatted into the given naming convention
		numCVs (int): The number of CVs to create for a spline
		spread (float): The distance between each controller (including tangents)
		closed (bool): Whether the spline forms a closed loop

	Returns:
		[str, ...]: All the CV's
		[str, ...]: All the CV Buffers
		[str, ...]: All the Out-Tangents
		[str, ...]: All the In-Tangents
		[str, ...]: All the Auto-Out-Tangents
		[str, ...]: All the Auto-In-Tangents
		[str, ...]: All the Twisters
		[str, ...]: All the Twister Buffers
		str: The base controller
	"""

	# Make bases for the controllers
	cvCtrl, outTanCtrl, inTanCtrl, twistCtrl, masterCtrl = _mkMasterControllers()

	master = cmds.duplicate(masterCtrl, name=MASTER_FMT.format(pfx))[0]
	cmds.addAttr(master, longName="Offset", attributeType='double', defaultValue=0.0)
	cmds.setAttr(master + '.Offset', edit=True, keyable=True)
	cmds.addAttr(master, longName="Stretch", attributeType='double', defaultValue=1.0, minValue=0.0001)
	cmds.setAttr(master + '.Stretch', edit=True, keyable=True)

	# make the requested number of CV's
	# don't hide the .rx attribute
	cvs, tws, cvBfrs, twBfrs = [], [], [], []
	controlsGrp = cmds.createNode("transform", name=CTRL_ORG_FMT.format(pfx))
	cmds.parentConstraint(master, controlsGrp, mo=True)
	cmds.scaleConstraint(master, controlsGrp, mo=True)

	for i in range(numCVs):
		cvBfr = cmds.createNode("transform", name=BFR_CV_FMT.format(pfx, i + 1), parent=controlsGrp)
		cv = cmds.duplicate(cvCtrl, name=CTRL_CV_FMT.format(pfx, i + 1))[0]
		twBfr = cmds.createNode("transform", name=BFR_TWIST_FMT.format(pfx, i + 1), parent=controlsGrp)
		tw = cmds.duplicate(twistCtrl, name=CTRL_TWIST_FMT.format(pfx, i + 1))[0]

		cmds.addAttr(cv, longName="Pin", attributeType='double', defaultValue=0.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(cv + '.Pin', edit=True, keyable=True)
		cmds.addAttr(cv, longName="PinParam", attributeType='double', defaultValue=0.0, minValue=0.0)
		cmds.setAttr(cv + '.PinParam', edit=True, keyable=True)

		for h in ['.tx', '.ty', '.tz', '.ry', '.rz', '.sx', '.sy', '.sz', '.v']:
			cmds.setAttr(tw + h, lock=True, keyable=False, channelBox=False)
		cmds.addAttr(tw, longName="UseTwist", attributeType='double', defaultValue=0.0, minValue=0.0, maxValue=1.0)
		cmds.setAttr(tw + '.UseTwist', edit=True, keyable=True)
		cv, = cmds.parent(cv, cvBfr)
		twBfr, = cmds.parent(twBfr, cv)
		tw, = cmds.parent(tw, twBfr)
		cmds.xform(cvBfr, translation=(spread * 3 * i, 0, 0))
		cvs.append(cv)
		tws.append(tw)
		cvBfrs.append(cvBfr)
		twBfrs.append(twBfr)

	# make the tangents and auto-tangents
	oTans, iTans, aoTans, aiTans = [], [], [], []

	segments = numCVs if closed else numCVs - 1
	for i in range(segments):
		# make oTan, and iTan
		otNum = i
		itNum = (i + 1) % numCVs

		oTan = cmds.duplicate(outTanCtrl, name=CTRL_OUTTAN_FMT.format(pfx, otNum + 1))[0]
		iTan = cmds.duplicate(inTanCtrl, name=CTRL_INTAN_FMT.format(pfx, itNum + 1))[0]
		for ndTan in [oTan, iTan]:
			cmds.addAttr(ndTan, longName="Auto", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
			cmds.setAttr(ndTan + '.Auto', edit=True, keyable=True)
			cmds.addAttr(ndTan, longName="Smooth", attributeType='double', defaultValue=1.0, minValue=0.0, maxValue=1.0)
			cmds.setAttr(ndTan + '.Smooth', edit=True, keyable=True)
			cmds.addAttr(ndTan, longName="Weight", attributeType='double', defaultValue=1.0, minValue=0.0001, maxValue=5.0)
			cmds.setAttr(ndTan + '.Weight', edit=True, keyable=True)

		cmds.xform(oTan, translation=(spread * (3 * otNum + 1), 0, 0))
		cmds.xform(iTan, translation=(spread * (3 * itNum - 1), 0, 0))

		aoTan = cmds.createNode("transform", name=BFR_AOUTTAN_FMT.format(pfx, otNum + 1), parent=cvs[otNum])
		aiTan = cmds.createNode("transform", name=BFR_AINTAN_FMT.format(pfx, itNum + 1), parent=cvs[itNum])

		cmds.xform(aoTan, translation=(spread * (3 * otNum + 1), 0, 0))
		cmds.xform(aiTan, translation=(spread * (3 * itNum - 1), 0, 0))
		oTan = cmds.parent(oTan, cvs[otNum])[0]
		iTan = cmds.parent(iTan, cvs[itNum])[0]

		oTans.append(oTan)
		iTans.append(iTan)
		aoTans.append(aoTan)
		aiTans.append(aiTan)

		for nd in [aiTan, aoTan]:
			cmds.setAttr(nd + ".overrideEnabled", 1)
			cmds.setAttr(nd + ".overrideDisplayType", 2)
			cmds.setAttr(nd + ".visibility", 0)

		makeLinkLine(aoTan, cvs[otNum], selectNode=oTan)
		makeLinkLine(aiTan, cvs[itNum], selectNode=iTan)

	cmds.delete((cvCtrl, outTanCtrl, inTanCtrl, twistCtrl, masterCtrl))
	return cvs, cvBfrs, oTans, iTans, aoTans, aiTans, tws, twBfrs, master




def createTwistSetup(pre, cur, post, buf, isFirst=False, isLast=False):
	""" Create an auto-twist setup for a set of CVs

	Arguments:
		pre (str): The previous CV for auto-tangent calculations
		cur (str): The CV that will have its auto-twist connected
		post (str): The next CV for auto-tangent calculations
		buf (str): The Buffer that will have its auto-twist connected
		isFirst (bool): Whether this segment is the first one in the spline
		isLast (bool): Whether this segment is the last one in the spline
	"""
	twt = cmds.createNode("twistTangent", name="twistAuto")
	dcm = cmds.createNode("decomposeMatrix", name="twistDecompose")

	if isLast:
		cmds.setAttr("{}.backpoint".format(twt), True)
	if isFirst or isLast:
		cmds.setAttr("{}.endpoint".format(twt), True)

	cmds.connectAttr("{}.worldMatrix[0]".format(pre), "{}.previousVertex".format(twt))
	cmds.connectAttr("{}.worldMatrix[0]".format(cur), "{}.currentVertex".format(twt))
	cmds.connectAttr("{}.worldMatrix[0]".format(post), "{}.nextVertex".format(twt))

	cmds.connectAttr("{}.parentInverseMatrix[0]".format(buf), "{}.parentInverseMatrix".format(twt))
	cmds.connectAttr("{}.outTwistMat".format(twt), "{}.inputMatrix".format(dcm))
	cmds.connectAttr("{}.outputRotate".format(dcm), "{}.rotate".format(buf))
	cmds.connectAttr("{}.outputScale".format(dcm), "{}.scale".format(buf))
	cmds.connectAttr("{}.outputTranslate".format(dcm), "{}.translate".format(buf))




def createTangentSegmentSetup(pre, start, end, nxt, oTan, iTan, aoTan, aiTan, isFirst=False, isLast=False):
	""" Create a single twist spline tangent setup for a single segment

	With 4 CV's given, there are 3 segments between them.
	This creates a tangent setup for the middle segment.

	Arguments:
		pre (str): The previous CV for auto-tangent calculations
		start (str): The CV that will have its out-tangent connected
		end (str): The CV that will have its in-tangent connected
		nxt (str): The last CV for auto-tangent calculations
		oTan (str): The Out-Tangent object
		iTan (str): The In-Tangent object
		aoTan (str): The Auto-Out-Tangent object
		aiTan (str): The Auto-In-Tangent object
		isFirst (bool): Whether this segment is the first one in the spline
		isLast (bool): Whether this segment is the last one in the spline
	"""
	# connect all the in/out tangents
	ott = cmds.createNode("twistTangent", name="twistTangentOut")  # TODO: make with name based on oTan
	cmds.connectAttr("{}.worldMatrix[0]".format(oTan), "{}.inTangent".format(ott))
	cmds.connectAttr("{}.Auto".format(oTan), "{}.auto".format(ott))
	cmds.connectAttr("{}.Smooth".format(oTan), "{}.smooth".format(ott))
	cmds.connectAttr("{}.Weight".format(oTan), "{}.weight".format(ott))
	if not isFirst:
		cmds.connectAttr("{}.worldMatrix[0]".format(pre), "{}.previousVertex".format(ott))
	else:
		cmds.setAttr("{}.Smooth".format(oTan), 0.0)

	cmds.connectAttr("{}.worldMatrix[0]".format(start), "{}.currentVertex".format(ott))
	cmds.connectAttr("{}.worldMatrix[0]".format(end), "{}.nextVertex".format(ott))
	cmds.setAttr("{}.Auto".format(oTan), 1.0)

	itt = cmds.createNode("twistTangent", name="twistTangentIn")  # TODO: make with name based on iTan
	cmds.connectAttr("{}.worldMatrix[0]".format(iTan), "{}.inTangent".format(itt))
	cmds.connectAttr("{}.Auto".format(iTan), "{}.auto".format(itt))
	cmds.connectAttr("{}.Smooth".format(iTan), "{}.smooth".format(itt))
	cmds.connectAttr("{}.Weight".format(iTan), "{}.weight".format(itt))
	if not isLast:
		cmds.connectAttr("{}.worldMatrix[0]".format(nxt), "{}.previousVertex".format(itt))
	else:
		cmds.setAttr("{}.Smooth".format(iTan), 0.0)

	cmds.connectAttr("{}.worldMatrix[0]".format(end), "{}.currentVertex".format(itt))
	cmds.connectAttr("{}.worldMatrix[0]".format(start), "{}.nextVertex".format(itt))
	cmds.setAttr("{}.Auto".format(iTan), 1.0)

	cmds.connectAttr("{}.outLinearTarget".format(itt), "{}.inLinearTarget".format(ott))
	cmds.connectAttr("{}.outLinearTarget".format(ott), "{}.inLinearTarget".format(itt))

	cmds.connectAttr("{0}.out".format(ott), "{}.translate".format(aoTan))
	cmds.connectAttr("{}.parentInverseMatrix[0]".format(aoTan), "{}.parentInverseMatrix".format(ott))

	cmds.connectAttr("{0}.out".format(itt), "{}.translate".format(aiTan))
	cmds.connectAttr("{}.parentInverseMatrix[0]".format(aiTan), "{}.parentInverseMatrix".format(itt))

def connectTwistSplineTangents(cvs, twBfrs, oTans, iTans, aoTans, aiTans, closed=False):
	""" Connect all of the tangent setups for the spline controller objects

	Arguments:
		cvs ([str, ...]): A list of all the CV controllers
		twBfrs ([str, ...]): A list of all the Twist Buffers
		oTans ([str, ...]): A list of all the out-tangents
		iTans ([str, ...]): A list of all the in-tangents
		aoTans ([str, ...]): A list of all the auto-out-tangents
		aiTans ([str, ...]): A list of all the auto-in-tangents
		closed (bool): Whether the spline forms a closed loop
	"""
	segments = len(cvs) if closed else len(cvs) - 1

	for i in range(segments):
		isFirst = (i == 0) and not closed
		isLast = (i == len(cvs) - 2) and not closed
		pre = cvs[i - 1]
		start = cvs[i]
		end = cvs[(i + 1) % len(cvs)]
		nxt = cvs[(i + 2) % len(cvs)]
		createTangentSegmentSetup(pre, start, end, nxt, oTans[i], iTans[i], aoTans[i], aiTans[i], isFirst=isFirst, isLast=isLast)

	for i in range(segments):
		isFirst = (i == 0) and not closed
		isLast = (i == len(cvs) - 2) and not closed

		cur = cvs[i]
		buf = twBfrs[i]

		pre = cvs[i - 1] if not isFirst else cvs[(i + 2) % len(cvs)]
		post = cvs[(i + 1) % len(cvs)] if not isLast else cvs[i - 2]

		createTwistSetup(pre, cur, post, buf, isFirst=isFirst, isLast=isLast)

def buildTwistSpline(pfx, cvs, aoTans, aiTans, tws, maxParam, closed=False):
	""" Given all the controller objects, build a twist spline

	Arguments:
		pfx (str): The user name of the spline. Will be formatted into the given naming convention
		cvs ([str, ...]): A list of the CV objects
		aoTans ([str, ...]): A list of the auto-out-tangents
		aiTans ([str, ...]): A list of the auto-in-tangents
		tws ([str, ...]): A list of the twist controllers
		maxParam (float): The U-Value of the last CV
		closed (bool): Whether the spline forms a closed loop

	Returns:
		str: The spline transform node
		str: The spline shape node

	"""
	numCVs = len(cvs)  # Total number of CV nodes
	shift = 0 if closed else 1  # convenience variable so I don't have if's everywhere
	usedCVs = numCVs + 1 - shift  # Total number of CV's connected to the spline node

	# build the spline object and set the spline Params
	splineTfm = cmds.createNode("transform", name=SPLINE_FMT.format(pfx))
	spline = cmds.createNode("twistSpline", parent=splineTfm, name=SPLINE_FMT.format(pfx) + "Shape")

	# Don't connect a first in tangent
	for i, aiTan in enumerate(aiTans):
		cmds.connectAttr("{}.worldMatrix[0]".format(aiTan), "{}.vertexData[{}].inTangent".format(spline, i+1))

	# Don't connect a last out tangent
	for i, aoTan in enumerate(aoTans):
		cmds.connectAttr("{}.worldMatrix[0]".format(aoTan), "{}.vertexData[{}].outTangent".format(spline, i))

	for u in range(usedCVs):
		i = u % numCVs
		cmds.connectAttr("{}.worldMatrix[0]".format(cvs[i]), "{}.vertexData[{}].controlVertex".format(spline, u))
		cmds.connectAttr("{}.Pin".format(cvs[i]), "{}.vertexData[{}].paramWeight".format(spline, u))
		if u != i:
			# The paramValue needs an offset if we're at the last connection of a closed spline
			adL = cmds.createNode("addDoubleLinear")
			cmds.setAttr("{0}.input2".format(adL), maxParam)
			cmds.connectAttr("{0}.PinParam".format(cvs[i]), "{0}.input1".format(adL), f=True)
			cmds.connectAttr("{0}.output".format(adL), "{}.vertexData[{}].paramValue".format(spline, u), f=True)
		else:
			cmds.connectAttr("{}.PinParam".format(cvs[i]), "{}.vertexData[{}].paramValue".format(spline, u))
			cmds.setAttr("{}.PinParam".format(cvs[i]), (u * maxParam) / (usedCVs - 1.0))

		cmds.connectAttr("{}.UseTwist".format(tws[i]), "{}.vertexData[{}].twistWeight".format(spline, u))
		cmds.connectAttr("{}.rotateX".format(tws[i]), "{}.vertexData[{}].twistValue".format(spline, u))

	cmds.setAttr("{}.Pin".format(cvs[0]), 1.0)
	cmds.setAttr("{}.UseTwist".format(tws[0]), 1.0)

	return splineTfm, spline

def buildRiders(pfx, spline, master, numJoints, closed=False):
	""" Build rider joints and constrain them to the spline

	Arguments:
		pfx (str): The user name of the spline. Will be formatted into the given naming convention
		spline (str): The spline shape node
		master (str): The master controller transform ndoe
		numJoints (int): The number of joints to create
		closed (bool): Whether or not the spline is a closed loop

	Returns:
		[str, ...]: The joint parent transforms
		[str, ...]: The joints
		str: The organizer parent
		str: The constraint node
	"""
	# make the joints at origin. The constraint will put them in place
	jointsGrp = cmds.createNode("transform", name=DFM_ORG_FMT.format(pfx))
	jPars, joints = [], []
	for i in range(numJoints):
		jp = cmds.createNode("transform", name=DFM_BFR_FMT.format(pfx, i + 1), parent=jointsGrp)
		j = cmds.createNode("joint", name=DFM_FMT.format(pfx, i + 1), parent=jp)
		cmds.setAttr(j + ".radius", 1.2)
		cmds.setAttr(jp + ".displayLocalAxis", 1)
		jPars.append(jp)
		joints.append(j)

	# build the constraint object
	cnst = cmds.createNode("riderConstraint")
	cmds.connectAttr("{}.Offset".format(master), "{}.globalOffset".format(cnst))
	cmds.connectAttr("{}.Stretch".format(master), "{}.globalSpread".format(cnst))
	if closed:
		cmds.setAttr("{}.useCycle".format(cnst), 1)

	# connect the constraints
	cmds.connectAttr("{}.outputSpline".format(spline), "{}.inputSplines[0].spline".format(cnst))
	for i in range(len(jPars)):
		if len(jPars) == 1:
			cmds.setAttr("{0}.params[{1}].param".format(cnst, i), 0.5)
		else:
			cmds.setAttr("{0}.params[{1}].param".format(cnst, i), i / (numJoints - 1.0))
		cmds.connectAttr("{}.parentInverseMatrix[0]".format(jPars[i]), "{0}.params[{1}].parentInverseMatrix".format(cnst, i))
		cmds.connectAttr("{0}.outputs[{1}].translate".format(cnst, i), "{}.translate".format(jPars[i]))
		cmds.connectAttr("{0}.outputs[{1}].rotate".format(cnst, i), "{}.rotate".format(jPars[i]))
		cmds.connectAttr("{0}.outputs[{1}].scale".format(cnst, i), "{}.scale".format(jPars[i]))

	return jPars, joints, jointsGrp, cnst

def makeTwistSpline(pfx, numCVs, numJoints=10, maxParam=None, spread=1.0, closed=False):
	""" Make a twist spline

	Arguments:
		pfx (str): The user name of the spline. Will be formatted into the given naming convention
		numCVs (int): The number of CV's to make that control the spline
		numJoints (int): The number of joints to make that ride the spline. Defaults to 10
		maxParam (int): The U-Value of the last CV. Defaults to 3*spread*(numCVs - 1)
		spread (float): The distance between each controller (including tangents). Defaults to 1
		closed (bool): Whether the spline forms a closed loop

	Returns:
		[str, ...]: All the CV's
		[str, ...]: All the CV's parent transforms
		[str, ...]: All the Out-Tangents
		[str, ...]: All the In-Tangents
		[str, ...]: All the joint parents
		[str, ...]: All the joints
		str: The joint organizer object (None if no joints requested)
		str: The spline object transform
		str: The base controller
		str: The rider constraint (None if no joints requested)
	"""
	if numCVs < 2:
		raise ValueError("Cannot create a TwistSpline with less than 2 CVs")

	if not cmds.pluginInfo("TwistSpline", query=True, loaded=True):
		cmds.loadPlugin("TwistSpline")

	if maxParam is None:
		maxParam = (numCVs - 1)

	maxParam *= 3.0 * spread

	cvs, cvBfrs, oTans, iTans, aoTans, aiTans, tws, twBfrs, master = mkTwistSplineControllers(pfx, numCVs, spread, closed=closed)
	connectTwistSplineTangents(cvs, twBfrs, oTans, iTans, aoTans, aiTans, closed=closed)
	splineTfm, splineShape = buildTwistSpline(pfx, cvs, aoTans, aiTans, tws, maxParam, closed=closed)

	jPars, joints, group, cnst = None, None, None, None
	if numJoints > 0:
		jPars, joints, group, cnst = buildRiders(pfx, splineShape, master, numJoints, closed=closed)

		# The scale of the overall spline should not affect the scale of the riders for now
		# Eventually, the rider constraint will handle interpolated scales
		#dnMultDivide = cmds.createNode("multiplyDivide")
		#cmds.setAttr("{0}.operation".format(dnMultDivide), 2)
		#cmds.setAttr("{0}.input1".format(dnMultDivide), 1, 1, 1)
		#cmds.connectAttr("{0}.scale".format(master), "{0}.input2".format(dnMultDivide), f=True)
		#cmds.connectAttr("{0}.outputX".format(dnMultDivide), "{0}.normValue".format(cnst), f=True)

	return cvs, cvBfrs, oTans, iTans, jPars, joints, group, splineTfm, master, cnst

def _bezierConvert(crv):
	""" Convert a curve to bezier non-destructively
	Arguments:
		crv (str): A curve shape or transform

	Returns:
		str: A Bezier curve shape
		list: Any temporary objects that need to be deleted later
	"""
	# Get the bezier shapes
	bezShapes = cmds.listRelatives(crv, path=True, type="bezierCurve")
	if bezShapes is not None:
		return bezShapes[0], []

	nurbsShapes = cmds.listRelatives(crv, path=True, type="nurbsCurve")
	if nurbsShapes is not None:
		tfm = cmds.listRelatives(nurbsShapes, path=True, parent=True)
		# nurbsCurveToBezier is *supposed* to take arguments, but always errors
		# So I have to do this with selection. I hate that. A Lot.
		dup = cmds.duplicate(tfm)  # duplicates and selects the object
		cmds.select(dup)
		cmds.nurbsCurveToBezier()  # Converts selection to bezier
		bezShapes = cmds.listRelatives(dup[0], path=True, type="bezierCurve")
		if bezShapes is not None:
			return bezShapes[0], dup

	return None, []

def convertToTwistSpline(pfx, crv, numJoints=10):
	""" Convert a given NURBS or Bezier curve to a TwistSpline

	Arguments:
		pfx (str): The user name of the spline. Will be formatted into the given naming convention
		crv (str): The transform or shape of a *bezier* spline
		numJoints (int): The number of joints to create that ride this spline
	"""
	# get nurbs curve shape
	crvShape, toDelete = _bezierConvert(crv)

	# Get the curve function set
	# There's no way to get the knots through pure MEL (nodes don't count)
	# So as long as I'm doing it this way, I'll do it all like this
	objects = OpenMaya.MSelectionList()
	OpenMaya.MGlobal.getSelectionListByName(crvShape, objects)
	meshDag = OpenMaya.MDagPath()
	objects.getDagPath(0, meshDag)
	curveFn = OpenMaya.MFnNurbsCurve(meshDag)

	# Get the curve data
	knots = OpenMaya.MDoubleArray()
	curveFn.getKnots(knots)
	params = list(knots)[1::3]
	numCVs = len(params)
	curveLen = curveFn.length()

	# Maya reports the wrong form of the curve through the API
	# So I've got to do it via mel
	#curveForm = curveFn.form()
	curveForm = cmds.getAttr("{0}.form".format(crvShape))
	isClosed = curveForm > 0  # 1->closed 2->periodic
	if isClosed:
		numCVs -= 1

	# Get the point data
	# I could do it with cmds if I wanted, but I would have to un-flatten the list
	# it's annoying either way
	#allPos = cmds.xform("{0}.cv[*]".format(crvShape), q=True, worldSpace=True, translation=True)
	allPos = OpenMaya.MPointArray()
	curveFn.getCVs(allPos)
	# Just testing a micro-optimization
	# 2 steps means not creating 3 MPoints per loop
	allPos = [allPos[i] for i in range(allPos.length())]
	allPos = [(p.x, p.y, p.z) for p in allPos]

	# Build the spline
	tempRet = makeTwistSpline(pfx, numCVs, numJoints=numJoints, maxParam=curveLen / 3.0, spread=1.0, closed=isClosed)
	cvs, bfrs, oTans, iTans, jPars, joints, group, spline, master, riderCnst = tempRet

	# Set the positions
	for pos, cv in zip(allPos[::3], bfrs):
		cmds.xform(cv, ws=True, a=True, t=pos)

	# Pin all the controllers so no length preservation happens
	# That way we can get the rotations at each param
	for cv in cvs:
		cmds.setAttr("{0}.Pin".format(cv), 1)

	for pos, cv in zip(allPos[1::3], oTans):
		cmds.xform(cv, ws=True, a=True, t=pos)
		cmds.setAttr("{0}.Auto".format(cv), 0)

	for pos, cv in zip(allPos[2::3], iTans):
		cmds.xform(cv, ws=True, a=True, t=pos)
		cmds.setAttr("{0}.Auto".format(cv), 0)

	# Make sure there is a rider constraint so I can follow the twist all along the spline
	tmpCnst = cmds.createNode("riderConstraint")
	cmds.connectAttr("{}.outputSpline".format(spline), "{}.inputSplines[0].spline".format(tmpCnst))

	# Get the rotations at each CV point
	newInd = 0
	rotations = []
	cmds.setAttr("{0}.normValue".format(tmpCnst), params[-1])
	for param in params:
		cmds.setAttr("{0}.params[{1}].param".format(tmpCnst, newInd), param)
		cmds.dgeval(tmpCnst)  # maybe a propagation bug somewhere in the constraint?
		rot = cmds.getAttr("{0}.outputs[{1}].rotate".format(tmpCnst, newInd))
		rotations.append(rot[0])
	cmds.delete(tmpCnst)

	# Update the rotations after I've got them all
	for rot, ctrl in zip(rotations, bfrs):
		cmds.setAttr("{0}.rotate".format(ctrl), *rot)

	# Un-pin everything but the first, so back to length preservation
	for cv in cvs[1:]:
		cmds.setAttr("{0}.Pin".format(cv), 0)

	# Re-set the tangent worldspace positions now that things have changed
	for pos, cv in zip(allPos[1::3], oTans):
		cmds.xform(cv, ws=True, a=True, t=pos)
		cmds.setAttr("{0}.Auto".format(cv), 0)

	for pos, cv in zip(allPos[2::3], iTans):
		cmds.xform(cv, ws=True, a=True, t=pos)
		cmds.setAttr("{0}.Auto".format(cv), 0)

	# Delete the extra joint group and the constraint if I had to make 'em
	if toDelete:
		cmds.delete(toDelete)

	# Lock the buffers
	for bfr in bfrs:
		for att in [x+y for x in 'trs' for y in 'xyz']:
			cmds.setAttr("{0}.{1}".format(bfr, att), lock=True)


def convertSelectedToTwistSpline(pfx, numJoints=10):
	sel = cmds.ls(sl=True)
	for s in sel:
		convertToTwistSpline(pfx, s, numJoints=numJoints)

