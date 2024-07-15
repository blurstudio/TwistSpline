/*
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
*/

#include <maya/M3dView.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MEvaluationManager.h>
#include <maya/MGlobal.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnPluginData.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MTypes.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MVectorArray.h>
#include <maya/MPointArray.h>
#include <maya/MBoundingBox.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MQuaternion.h>

#include <cassert>
#include <string>
#include <iostream>
#include <limits>

#include "twistSpline.h"
#include "twistSplineData.h"
#include "twistSplineNode.h"

#define MCHECKERROR(STAT)         \
    if (MS::kSuccess != STAT) {   \
        return MS::kFailure;      \
    }


MTypeId	TwistSplineNode::id(0x001226F7);
MString	TwistSplineNode::drawDbClassification("drawdb/geometry/twistSpline");
MString	TwistSplineNode::drawRegistrantId("TwistSplineNodePlugin");

MObject TwistSplineNode::aOutputSpline;
MObject TwistSplineNode::aSplineLength;
MObject TwistSplineNode::aNurbsData;

MObject TwistSplineNode::aVertexData;
MObject TwistSplineNode::aInTangent;
MObject TwistSplineNode::aOutTangent;
MObject TwistSplineNode::aControlVertex;
MObject TwistSplineNode::aParamValue;
MObject TwistSplineNode::aParamWeight;
MObject TwistSplineNode::aTwistValue;
MObject TwistSplineNode::aTwistWeight;
MObject TwistSplineNode::aUseOrient;

MObject TwistSplineNode::aGeometryChanging;
MObject TwistSplineNode::aSplineDisplay;
MObject TwistSplineNode::aDebugDisplay;
MObject TwistSplineNode::aDebugScale;
MObject TwistSplineNode::aMaxVertices;

TwistSplineNode::TwistSplineNode() {}
TwistSplineNode::~TwistSplineNode() {}

MStatus TwistSplineNode::initialize() {
	MFnCompoundAttribute cAttr;
	MFnMatrixAttribute mAttr;
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;

	//---------------- Output ------------------

	aOutputSpline = tAttr.create("outputSpline", "os", TwistSplineData::id);
	tAttr.setWritable(false);
	addAttribute(aOutputSpline);

	aNurbsData = tAttr.create("outputNurbsCurve", "onc", MFnNurbsCurveData::kNurbsCurve, MObject::kNullObj);
	tAttr.setWritable(false);
	addAttribute(aNurbsData);

	aSplineLength = uAttr.create("splineLength", "sl", MFnUnitAttribute::kDistance);
	nAttr.setWritable(false);
	addAttribute(aSplineLength);

	aGeometryChanging = nAttr.create("geometryChanging", "gch", MFnNumericData::kBoolean, true);
	nAttr.setStorable(false);
	nAttr.setHidden(true);
	nAttr.setConnectable(false);
	addAttribute(aGeometryChanging);

	//--------------- Input -------------------

	aSplineDisplay = nAttr.create("splineDisplay", "sd", MFnNumericData::kBoolean, true);
	addAttribute(aSplineDisplay);
	aDebugDisplay = nAttr.create("debugDisplay", "dd", MFnNumericData::kBoolean, false);
	addAttribute(aDebugDisplay);
	aDebugScale = nAttr.create("debugScale", "ds", MFnNumericData::kDouble, 1.0);
	addAttribute(aDebugScale);
	aMaxVertices = nAttr.create("maxVertices", "mv", MFnNumericData::kInt, 1000);
    nAttr.setMin(2);
	addAttribute(aMaxVertices);

	//--------------- Array -------------------

	aInTangent = mAttr.create("inTangent", "int");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);

	aOutTangent = mAttr.create("outTangent", "ot");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);

	aControlVertex = mAttr.create("controlVertex", "cv");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);

	aParamValue = nAttr.create("paramValue", "pv", MFnNumericData::kDouble, 0.0);
	aParamWeight = nAttr.create("paramWeight", "pw", MFnNumericData::kDouble, 0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	aTwistValue = uAttr.create("twistValue", "tv", MFnUnitAttribute::kAngle, 0.0);
	aTwistWeight = nAttr.create("twistWeight", "tw", MFnNumericData::kDouble, 0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	aUseOrient = nAttr.create("useOrient", "uo", MFnNumericData::kDouble, 0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);

	aVertexData = cAttr.create("vertexData", "vd");
	cAttr.setArray(true);
	cAttr.addChild(aInTangent); // matrix
	cAttr.addChild(aControlVertex); // matrix
	cAttr.addChild(aOutTangent); // matrix

	cAttr.addChild(aParamValue); // float unbounded
	cAttr.addChild(aParamWeight); // float 0-1
	cAttr.addChild(aTwistWeight); // float 0-1
	cAttr.addChild(aUseOrient); // float 0-1
	cAttr.addChild(aTwistValue); // float unbounded

	addAttribute(aVertexData);

	attributeAffects(aInTangent, aOutputSpline);
	attributeAffects(aControlVertex, aOutputSpline);
	attributeAffects(aOutTangent, aOutputSpline);
	attributeAffects(aParamValue, aOutputSpline);
	attributeAffects(aParamWeight, aOutputSpline);
	attributeAffects(aTwistWeight, aOutputSpline);
	attributeAffects(aTwistValue, aOutputSpline);
	attributeAffects(aUseOrient, aOutputSpline);
	attributeAffects(aMaxVertices, aOutputSpline);

	attributeAffects(aInTangent, aNurbsData);
	attributeAffects(aControlVertex, aNurbsData);
	attributeAffects(aOutTangent, aNurbsData);
	attributeAffects(aParamValue, aNurbsData);
	attributeAffects(aParamWeight, aNurbsData);
	attributeAffects(aTwistWeight, aNurbsData);
	attributeAffects(aTwistValue, aNurbsData);
	attributeAffects(aUseOrient, aNurbsData);
	attributeAffects(aMaxVertices, aNurbsData);

	attributeAffects(aInTangent, aSplineLength);
	attributeAffects(aControlVertex, aSplineLength);
	attributeAffects(aOutTangent, aSplineLength);
	attributeAffects(aMaxVertices, aSplineLength);

	// Geometry changing
	attributeAffects(aInTangent, aGeometryChanging);
	attributeAffects(aControlVertex, aGeometryChanging);
	attributeAffects(aOutTangent, aGeometryChanging);
	attributeAffects(aParamValue, aGeometryChanging);
	attributeAffects(aParamWeight, aGeometryChanging);
	attributeAffects(aTwistWeight, aGeometryChanging);
	attributeAffects(aTwistValue, aGeometryChanging);
	attributeAffects(aUseOrient, aGeometryChanging);
	attributeAffects(aMaxVertices, aGeometryChanging);

	return MS::kSuccess;
}

TwistSplineT* TwistSplineNode::getSplineData() const {
	MStatus stat;
	MObject output;

	// Getting this plug causes an update
	MPlug tPlug(thisMObject(), aOutputSpline);
	tPlug.getValue(output);
	MFnPluginData outData(output);
	auto tsd = dynamic_cast<TwistSplineData *>(outData.data());
	return tsd->getSpline();
}

MBoundingBox TwistSplineNode::boundingBox() const {
	TwistSplineT *ts = getSplineData();

	double minx, miny, minz, maxx, maxy, maxz;
	minx = miny = minz = std::numeric_limits<double>::max();
	maxx = maxy = maxz = std::numeric_limits<double>::min();

	MPointArray pts = ts->getVerts();

	for (unsigned i = 0; i < pts.length(); ++i) {
		MPoint &pt = pts[i];
		minx = pt[0] < minx ? pt[0] : minx;
		miny = pt[1] < miny ? pt[1] : miny;
		minz = pt[2] < minz ? pt[2] : minz;
		maxx = pt[0] > maxx ? pt[0] : maxx;
		maxy = pt[1] > maxy ? pt[1] : maxy;
		maxz = pt[2] > maxz ? pt[2] : maxz;
	}
	return MBoundingBox(MPoint(minx, miny, minz), MPoint(maxx, maxy, maxz));
}

void TwistSplineNode::getSplineDraw(bool &oDraw) const {
	MStatus stat;
	MObject mobj = thisMObject();

	oDraw = false;
	if (!mobj.isNull()) {
		MPlug drawPlug(mobj, aSplineDisplay);
		if (!drawPlug.isNull())
			drawPlug.getValue(oDraw);
	}
}

void TwistSplineNode::getDebugDraw(bool &oDraw, double &oScale) const {
	MStatus stat;
	MObject mobj = thisMObject();

	oDraw = false;
	oScale = 1.0;
	if (!mobj.isNull()) {
		MPlug drawPlug(mobj, aDebugDisplay);
		if (!drawPlug.isNull()) {
			drawPlug.getValue(oDraw);
		}

		MPlug scalePlug(mobj, aDebugScale);
		if (!scalePlug.isNull()) {
			scalePlug.getValue(oScale);
		}
	}

}

MStatus	TwistSplineNode::compute(const MPlug& plug, MDataBlock& data) {
	if (plug == aOutputSpline) {
		MStatus status;

		// Get all input Data
		MPointArray points;
		MPointArray scales;
		std::vector<MQuaternion> quats;
		std::vector<double> lockPositions;
		std::vector<double> lockVals;
		std::vector<double> twistLock;
		std::vector<double> userTwist;
		std::vector<double> orientLock;

		MDataHandle hMaxVertices = data.inputValue(aMaxVertices);
		int maxVertices = hMaxVertices.asInt();

		// loop over the input matrices
		MArrayDataHandle inputs = data.inputArrayValue(aVertexData);
		unsigned ecount = inputs.elementCount();

        // Trim the number of vertices to the maximum provided by the user
        if (maxVertices < ecount) { ecount = maxVertices; }

		// I'm OK with just looping over the physical indices here
		// because if it's unconnected, then I don't really care
		double sGet[4];
		sGet[3] = 1.0;
		bool gotLocks = false, gotOris = false;
		for (unsigned i = 0; i < ecount; ++i) {
			auto group = inputs.inputValue();

			lockPositions.push_back(group.child(aParamValue).asDouble());

			double lockIt = group.child(aParamWeight).asDouble();
			lockVals.push_back(lockIt);
			if (lockIt > 0.0) gotLocks = true;

			twistLock.push_back(group.child(aTwistWeight).asDouble());
			userTwist.push_back(group.child(aTwistValue).asDouble());

			double oriIt = group.child(aUseOrient).asDouble();
			orientLock.push_back(oriIt);
			if (oriIt > 0.0) gotOris = true;

			if (i > 0) {
				// Ignore the tangent data for the first vertex
				// because they come *before* it.  I'm just using the same interface
				// for conveninece here
				auto itMat = MTransformationMatrix(group.child(aInTangent).asMatrix());
				points.append(itMat.getTranslation(MSpace::kWorld));
				itMat.getScale(sGet, MSpace::kObject);
				scales.append(MPoint(sGet));
				quats.push_back(itMat.rotation());
			}

			auto cvMat = MTransformationMatrix(group.child(aControlVertex).asMatrix());
			points.append(cvMat.getTranslation(MSpace::kWorld));
			cvMat.getScale(sGet, MSpace::kObject);
			scales.append(MPoint(sGet));
			quats.push_back(cvMat.rotation());

			auto otMat = MTransformationMatrix(group.child(aOutTangent).asMatrix());
			points.append(otMat.getTranslation(MSpace::kWorld));
			otMat.getScale(sGet, MSpace::kObject);
			scales.append(MPoint(sGet));
			quats.push_back(otMat.rotation());

			inputs.next();
		}

		// Remove the last item from the group. It's the out tangent past the end
		if (!quats.empty())
			quats.pop_back();

		if (points.length() > 0)
			points.remove(points.length() - 1);

		if (scales.length() > 0)
			scales.remove(scales.length() - 1);

		if (!gotLocks && !lockVals.empty())
			lockVals[0] = 1.0;

		if (!gotOris && !orientLock.empty())
			orientLock[0] = 1.0;

		// Output Data Handles
		MDataHandle storageH = data.outputValue(aOutputSpline, &status);
		MCHECKERROR(status);

		// Build output data object
		TwistSplineData *outSplineData;
		MFnPluginData fnDataCreator;
		MTypeId tmpid(TwistSplineData::id);
		fnDataCreator.create(tmpid, &status);
		MCHECKERROR(status);
		outSplineData = dynamic_cast<TwistSplineData*>(fnDataCreator.data(&status));
		MCHECKERROR(status);
		TwistSplineT *outSpline = outSplineData->getSpline();

		//if (points.length() != 0)
			outSpline->setVerts(points, scales, quats, lockPositions, lockVals, userTwist, twistLock, orientLock);

        // Testing closest point
        //outSpline->buildKDTree();
        //auto cp = outSpline->getClosestPoint(MPoint());

		storageH.setMPxData(outSplineData);
		data.setClean(aOutputSpline);
	}
	else if (plug == aNurbsData) {
		MStatus status;

		// Get the splines that need computing.
		MDataHandle inHandle = data.inputValue(aOutputSpline);
		MDataHandle outHandle = data.outputValue(aNurbsData);

		MPxData* pd = inHandle.asPluginData();
		if (pd == nullptr) {
			outHandle.setMObject(MObject::kNullObj);
			return MS::kSuccess;
		}

		auto inSplineData = dynamic_cast<TwistSplineData *>(pd);
		TwistSplineT *spline = inSplineData->getSpline();
		if (spline == nullptr) {
			outHandle.setMObject(MObject::kNullObj);
			return MS::kSuccess;
		}

		// Construct the nurbs curve data.
		size_t degree = 3;
		size_t curKnot = 0;
		MPointArray cvs = spline->getVerts();
		size_t numVerts = size(cvs);
		MDoubleArray knots;
		int increments = degree;
		int numKnots = numVerts + degree + 1;
		// Ignore the first and last knots.
		for (int i = 1; i < numKnots - 1; ++i) {
			float knot = curKnot;
			knots.append(knot);
			// Increment the knots by 1 every degree
			if (i == increments) {
				curKnot += 1;
				increments += degree;
			}
		}

		MFnNurbsCurveData curveData;
		MObject curveDataObj = curveData.create();
		MFnNurbsCurve curve;
		curve.create(cvs, knots, degree, MFnNurbsCurve::kOpen, false, true,
					 curveDataObj, &status);
		MCHECKERROR(status);

		outHandle.setMObject(curveDataObj);
		data.setClean(aNurbsData);
	}
	else if (plug == aSplineLength) {
		// First, get the splines that need computing, and their weight.
		MDataHandle inHandle = data.inputValue(aOutputSpline);
		MDataHandle outHandle = data.outputValue(aSplineLength);

		MPxData* pd = inHandle.asPluginData();
		if (pd == nullptr) {
			outHandle.setDouble(0.0);
			return MS::kSuccess;
		}

		auto inSplineData = dynamic_cast<TwistSplineData *>(pd);
		TwistSplineT *spline = inSplineData->getSpline();

		if (spline == nullptr) {
			outHandle.setDouble(0.0);
			return MS::kSuccess;
		}
		outHandle.setDouble(spline->getTotalLength());
		data.setClean(aSplineLength);
	}
	else if (plug == aGeometryChanging) {
		MStatus status;
		MDataHandle boolHandle = data.outputValue(aGeometryChanging, &status);
		MCHECKERROR(status);
		boolHandle.setBool(true);
		boolHandle.setClean();
	}
	else {
		return MS::kUnknownParameter;
	}
	return MS::kSuccess;
}

/*
	Technique 1: Hack the EM to force evaluate and cache attributes.
	To improve performance, Evaluation Manager aggressively skips evaluation of
	attributes which are not connected to other nodes. In cases where an external
	user of DG data (in this case the renderer) needs to read unconnected values
	from a node during or after EM evaluation, we need to take extra steps to
	ensure the data is evaluated by the EM (and cached). The most notable rules
	used by the EM for skipping evaluation are:
		1. Output attributes without output-connections are not computed in EM
		(and are not eligible for caching).
		2. Input attributes are never cached in Evaluation Cache.

	In TwistSplineNode, attribute "geometryChanging" is virtually connected to the
	renderer. But EM does not understand these "virtual connection", and skips
	evaluation and caching for them. The current workaround are:
		1. To bypass rule 2, we made them a passing-through output attributes :
		inputSize->outputSize
		2. To bypass rule 1, repeat the affect relationship in setDependentsDirty().
			[*]
			[*] Note, this is a trick that relies on some internal hack to EM.
				Maya may provide better API for this in future updates.
				When proper force evaluation API is come, you won't need to override
				this method.
*/
MStatus TwistSplineNode::setDependentsDirty(const MPlug& plug,
											MPlugArray& plugArray) {
	// Repeating the affect relationship we have specified.
	// This method just mean to trick EM.
	// No need to do this outside of EM graph construction (for the sake of
	// performance)
	if (MEvaluationManager::graphConstructionActive()) {
		if (plug == aInTangent || plug == aOutTangent ||
			plug == aControlVertex || plug == aMaxVertices ||
			plug == aParamValue || plug == aParamWeight ||
			plug == aTwistValue || plug == aTwistWeight || plug == aUseOrient) {
			MObject thisNode = thisMObject();
			MPlug outputSplinePlug(thisNode, aOutputSpline);
			MPlug nurbsDataPlug(thisNode, aNurbsData);
			MPlug geometryChangingPlug(thisNode, aGeometryChanging);
			plugArray.append(outputSplinePlug);
			plugArray.append(nurbsDataPlug);
			plugArray.append(geometryChangingPlug);
		}
	}
	// Try not set any data or attribute value in this method
	// Because EM's parallel evaluation will not call this method at all
	// A widely used *bad* approach is to write "aGeometryChanged=true" when
	// some attribute changed. Use Technique 1.1 to avoid this.
	return MStatus::kSuccess;
}

MStatus TwistSplineNode::postEvaluation(const MDGContext& context,
										const MEvaluationNode& evaluationNode,
										PostEvaluationType evalType) {
	// For cache restoration only.
	// This method is responsible for fixing the 'geometryChanging' flag in
	// cache restore frames Because in cache store phase, PopulateGeometry &
	// Viewport-Caching happens before Evaluation-Cache store The value of
	// 'geometryChanging' will always be set to 'false' (it is already used by
	// render) Thus, we have to fix the geometryChanging attribute to the
	// correct value.
	MStatus status;
	// kEvaluateDirectly indicates we are restoring from cache.
	if (evalType == PostEvaluationEnum::kEvaluatedDirectly &&
		evaluationNode.dirtyPlugExists(aGeometryChanging, &status) && status) {
		MDataBlock data = forceCache();
		MDataHandle boolHandle = data.outputValue(aGeometryChanging, &status);
		if (status != MStatus::kSuccess) return status;
		boolHandle.setBool(true);
		boolHandle.setClean();
	}
	return MPxLocatorNode::postEvaluation(context, evaluationNode, evalType);
}

void TwistSplineNode::getCacheSetup(const MEvaluationNode& evalNode,
									MNodeCacheDisablingInfo& disablingInfo,
									MNodeCacheSetupInfo& cacheSetupInfo,
									MObjectArray& monitoredAttributes) const {
	MPxLocatorNode::getCacheSetup(evalNode, disablingInfo, cacheSetupInfo,
								  monitoredAttributes);
	assert(!disablingInfo.getCacheDisabled());
	cacheSetupInfo.setPreference(MNodeCacheSetupInfo::kWantToCacheByDefault,
								 true);
}

void* TwistSplineNode::creator() { return new TwistSplineNode(); }

// Must be called after MPxGeometryOverride::updateDG()
// Typically used by MPxGeometryOverride::requiresGeometryUpdate()
bool TwistSplineNode::isGeometryChanging() const {
	MDataBlock block = const_cast<TwistSplineNode*>(this)->forceCache();
	// Use inputValue() to trigger evaluation here
	// Because MPxGeometryOverride::requiresGeometryUpdate() can be called
	// outside of MPxGeometryOverride::initialize()/updateDG() This evaluation
	// is safe because this attribute cannot be connected And thus cannot reach
	// other nodes
	return block.inputValue(TwistSplineNode::aGeometryChanging).asBool();
}

void TwistSplineNode::postConstructor() {
	MFnDependencyNode nodeFn(thisMObject());
	nodeFn.setName("twistSplineShape#");
	nodeFn.setIcon("twistSpline.png");
}

// Workload for MPxGeometryOverride::updateDG()
// Updating all the attributes needed by the renderer
// Ensure these attributes can be accessed by outputValue() safely later
void TwistSplineNode::updateRenderAttributes() {
	MDataBlock datablock = forceCache();
	datablock.inputValue(TwistSplineNode::aGeometryChanging);
	datablock.inputValue(TwistSplineNode::aDebugDisplay);
	datablock.inputValue(TwistSplineNode::aDebugScale);
	datablock.inputValue(TwistSplineNode::aOutputSpline);
}

// Should only be called from MPxGeometryOverride::populateGeometry()
// Returns the parameters required to update geometry
// Set "TwistSplineNode::geometryChanging" to "false" to avoid duplicate update
// [[ensure : isGeometryChanging() == false]]
void TwistSplineNode::updatingGeometry() {
	MDataBlock block = const_cast<TwistSplineNode*>(this)->forceCache();

	// Reset the geometryChanging attribute to false so that we do not update
	// the geometry multiple times
	block.outputValue(TwistSplineNode::aGeometryChanging).set(false);
}
