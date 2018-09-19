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
#include <maya/MGlobal.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
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

#include <string>
#include <iostream>

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

MObject TwistSplineNode::aVertexData;
MObject TwistSplineNode::aInTangent;
MObject TwistSplineNode::aOutTangent;
MObject TwistSplineNode::aControlVertex;
MObject TwistSplineNode::aParamValue;
MObject TwistSplineNode::aParamWeight;
MObject TwistSplineNode::aTwistValue;
MObject TwistSplineNode::aTwistWeight;
MObject TwistSplineNode::aUseOrient;

MObject TwistSplineNode::aDebugDisplay;
MObject TwistSplineNode::aDebugScale;

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

	aSplineLength = nAttr.create("splineLength", "sl", MFnNumericData::kDouble);
	nAttr.setWritable(false);
	addAttribute(aSplineLength);

	//--------------- Input -------------------

	aDebugDisplay = nAttr.create("debugDisplay", "dd", MFnNumericData::kBoolean, false);
	addAttribute(aDebugDisplay);
	aDebugScale = nAttr.create("debugScale", "ds", MFnNumericData::kDouble, 1.0);
	addAttribute(aDebugScale);

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

	attributeAffects(aInTangent, aSplineLength);
	attributeAffects(aControlVertex, aSplineLength);
	attributeAffects(aOutTangent, aSplineLength);

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
		std::vector<MQuaternion> quats;
		std::vector<double> lockPositions;
		std::vector<double> lockVals;
		std::vector<double> twistLock;
		std::vector<double> userTwist;
		std::vector<double> orientLock;

		// loop over the input matrices
		MArrayDataHandle inputs = data.inputArrayValue(aVertexData);
		unsigned ecount = inputs.elementCount();

		// I'm OK with just looping over the physical indices here
		// because if it's unconnected, then I don't really care
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
				quats.push_back(itMat.rotation());
			}

			auto cvMat = MTransformationMatrix(group.child(aControlVertex).asMatrix());
			points.append(cvMat.getTranslation(MSpace::kWorld));
			quats.push_back(cvMat.rotation());

			auto otMat = MTransformationMatrix(group.child(aOutTangent).asMatrix());
			points.append(otMat.getTranslation(MSpace::kWorld));
			quats.push_back(otMat.rotation());

			inputs.next();
		}
		
		// Remove the last item from the group. It's the out tangent past the end
		quats.pop_back();
		points.remove(points.length() - 1);

		if (!gotLocks && !lockVals.empty())
			lockVals[0] = 1.0;

		// We *ALWAYS* orient lock the first vertex. It's what the entire spline is based on
		// Maybe there's a better way? Look into that sometime
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

		outSpline->setVerts(points, quats, lockPositions, lockVals, userTwist, twistLock, orientLock);

        // Testing closest point
        //outSpline->buildKDTree();
        //auto cp = outSpline->getClosestPoint(MPoint());

		storageH.setMPxData(outSplineData);
		data.setClean(aOutputSpline);
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

		auto inSplineData = (TwistSplineData *)pd;
		TwistSplineT *spline = inSplineData->getSpline();

		if (spline == nullptr) {
			outHandle.setDouble(0.0);
			return MS::kSuccess;
		}
		outHandle.setDouble(spline->getTotalLength());
		data.setClean(aSplineLength);
	}
	else {
		return MS::kUnknownParameter;
	}
	return MS::kSuccess;
}

void* TwistSplineNode::creator() {
	return new TwistSplineNode();
}

void TwistSplineNode::draw(M3dView &view, const MDagPath &path, M3dView::DisplayStyle style, M3dView::DisplayStatus dstat) {
	// TODO: build the legacy viewport draw mechanism
}





//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Viewport 2.0 override implementation
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// By setting isAlwaysDirty to false in MPxDrawOverride constructor, the
// draw override will be updated (via prepareForDraw()) only when the node
// is marked dirty via DG evaluation or dirty propagation. Additional
// callback is also added to explicitly mark the node as being dirty (via
// MRenderer::setGeometryDrawDirty()) for certain circumstances. Note that
// the draw callback in MPxDrawOverride constructor is set to NULL in order
// to achieve better performance.
TwistSplineDrawOverride::TwistSplineDrawOverride(const MObject& obj)
	: MHWRender::MPxDrawOverride(obj, NULL, false) {
	fModelEditorChangedCbId = MEventMessage::addEventCallback("modelEditorChanged", OnModelEditorChanged, this);

	MStatus status;
	MFnDependencyNode node(obj, &status);
	tsn = status ? dynamic_cast<TwistSplineNode*>(node.userNode()) : NULL;
}

TwistSplineDrawOverride::~TwistSplineDrawOverride() {
	tsn = NULL;

	if (fModelEditorChangedCbId != 0) {
		MMessage::removeCallback(fModelEditorChangedCbId);
		fModelEditorChangedCbId = 0;
	}
}

void TwistSplineDrawOverride::OnModelEditorChanged(void *clientData) {
	// Mark the node as being dirty so that it can update on display appearance
	// switch among wireframe and shaded.
	TwistSplineDrawOverride *ovr = static_cast<TwistSplineDrawOverride*>(clientData);
	if (ovr && ovr->tsn) {
		MHWRender::MRenderer::setGeometryDrawDirty(ovr->tsn->thisMObject());
	}
}

MHWRender::DrawAPI TwistSplineDrawOverride::supportedDrawAPIs() const {
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}

bool TwistSplineDrawOverride::isBounded(const MDagPath& /*objPath*/, const MDagPath& /*cameraPath*/) const {
	return true;
}

MBoundingBox TwistSplineDrawOverride::boundingBox(
		const MDagPath& objPath,
		const MDagPath& cameraPath) const {
	return tsn->boundingBox();
}


// Called by Maya each time the object needs to be drawn.
MUserData* TwistSplineDrawOverride::prepareForDraw(
		const MDagPath& objPath,
		const MDagPath& cameraPath,
		const MHWRender::MFrameContext& frameContext,
		MUserData* oldData) {
	MStatus status;
	// Any data needed from the Maya dependency graph must be retrieved and cached in this stage.
	TwistSplineDrawData* data = dynamic_cast<TwistSplineDrawData*>(oldData);
	if (!data) data = new TwistSplineDrawData();

	TwistSplineT* ts = tsn->getSplineData();
	data->splinePoints = ts->getPoints();
	tsn->getDebugDraw(data->debugDraw, data->debugScale);
	
	if (data->debugDraw){
		double scale = data->debugScale;

		MVectorArray &tans = ts->getTangents();
		data->tangents.setLength(tans.length() * 2);
		for (size_t i = 0; i < tans.length(); ++i) {
			MPoint &spi = data->splinePoints[i];
			data->tangents[2*i] = spi;
			data->tangents[(2*i) +1] = scale * tans[i] + spi;
		}

		MVectorArray &norms = ts->getNormals();
		data->normals.setLength(norms.length() * 2);
		for (size_t i = 0; i < norms.length(); ++i) {
			MPoint &spi = data->splinePoints[i];
			MVector &nn = norms[i];
			data->normals[2 * i] = spi;
			data->normals[(2 * i) + 1] = scale * nn + spi;
		}

		MVectorArray &binorms = ts->getBinormals();
		data->binormals.setLength(binorms.length() * 2);
		for (size_t i = 0; i < binorms.length(); ++i) {
			MPoint &spi = data->splinePoints[i];
			data->binormals[2 * i] = spi;
			data->binormals[(2 * i) + 1] = scale * binorms[i] + spi;
		}
	}
	
	// get correct color based on the state of object, e.g. active or dormant
	data->color = MHWRender::MGeometryUtilities::wireframeColor(objPath);
	return data;
}

// addUIDrawables() provides access to the MUIDrawManager, which can be used
// to queue up operations for drawing simple UI elements such as lines, circles and
// text. To enable addUIDrawables(), override hasUIDrawables() and make it return true.
void TwistSplineDrawOverride::addUIDrawables(
		const MDagPath& objPath,
		MHWRender::MUIDrawManager& drawManager,
		const MHWRender::MFrameContext& frameContext,
		const MUserData* userData) {
	TwistSplineDrawData* data = (TwistSplineDrawData*)userData;
	if (!data) return;

	bool draw2D = false; // ALWAYS false
	drawManager.beginDrawable();

	drawManager.setColor(data->color);

	drawManager.setDepthPriority(MHWRender::MRenderItem::sActiveLineDepthPriority);
	drawManager.lineStrip(data->splinePoints, draw2D);

	if (data->debugDraw){
		drawManager.setColor(MColor(.5, 0, 0));
		drawManager.lineList(data->tangents, draw2D);

		drawManager.setColor(MColor(0, 0, .5));
		drawManager.lineList(data->binormals, draw2D);

		drawManager.setColor(MColor(0, .5, 0));
		drawManager.lineList(data->normals, draw2D);
	}
	drawManager.endDrawable();
}

