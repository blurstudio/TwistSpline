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

#pragma once

#include <vector>

#include <maya/MPxLocatorNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MTypeId.h> 
#include <maya/MVector.h>
#include <maya/MEvaluationNode.h>
#include <maya/MFnMessageAttribute.h>

#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>
#include <maya/MDrawContext.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MGlobal.h>
#include <maya/MEventMessage.h>
#include <maya/MFnDependencyNode.h>

#include "twistSpline.h"
#include "twistSplineData.h"

static bool sUseLegacyDraw = (getenv("MAYA_ENABLE_VP2_PLUGIN_LOCATOR_LEGACY_DRAW") != NULL);
 
class TwistSplineNode : public MPxLocatorNode
{
public:
	TwistSplineNode();
	virtual	~TwistSplineNode(); 

	virtual	MStatus	compute( const MPlug& plug, MDataBlock& data );
	//virtual MStatus preEvaluation(const  MDGContext& context, const MEvaluationNode& evaluationNode);
	//virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray);
	static	void*	creator();
	static	MStatus	initialize();
	virtual void 	draw(M3dView &view, const MDagPath &path, M3dView::DisplayStyle style, M3dView::DisplayStatus);
	virtual bool	isBounded() const {
		return true;
	}
	virtual MBoundingBox boundingBox() const;
	TwistSplineT* getSplineData() const;
	void getDebugDraw(bool &oDraw, double &oScale) const;

public:
	static MObject aOutputSpline;
	static MObject aSplineLength;
	static MObject aMaxVertices;

	// array
	static MObject aVertexData;
		// array children
		static MObject aInTangent;
		static MObject aOutTangent;
		static MObject aControlVertex;
		static MObject aParamValue;
		static MObject aParamWeight;
		static MObject aTwistValue;
		static MObject aTwistWeight;
		static MObject aUseOrient;

	static MObject aDebugDisplay;
	static MObject aDebugScale;

	static MTypeId	id;
	static MString drawDbClassification;
	static MString drawRegistrantId;
};


class TwistSplineDrawData : public MUserData {
public:
	TwistSplineDrawData() : MUserData(false) {} // don't delete after draw
	virtual ~TwistSplineDrawData() {}
	MColor color;
	MPointArray splinePoints;
	MPointArray tangents;
	MPointArray normals;
	MPointArray binormals;
	bool debugDraw;
	double debugScale;
};

class TwistSplineDrawOverride : public MHWRender::MPxDrawOverride {
public:
	static MHWRender::MPxDrawOverride* Creator(const MObject& obj) {
		return new TwistSplineDrawOverride(obj);
	}

	virtual ~TwistSplineDrawOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;

	virtual bool isBounded( const MDagPath& objPath, const MDagPath& cameraPath) const; 

	virtual MBoundingBox boundingBox( const MDagPath& objPath, const MDagPath& cameraPath) const;
	void getDebugDraw(const MDagPath& objPath, bool &oDraw, bool &oScale) const;

	virtual MUserData* prepareForDraw(
		const MDagPath& objPath,
		const MDagPath& cameraPath,
		const MHWRender::MFrameContext& frameContext,
		MUserData* oldData);

	virtual bool hasUIDrawables() const { return true; }

	virtual void addUIDrawables(
		const MDagPath& objPath,
		MHWRender::MUIDrawManager& drawManager,
		const MHWRender::MFrameContext& frameContext,
		const MUserData* data);

	// Return true if internal tracing is desired.
	virtual bool traceCallSequence() const { return false; }

	virtual void handleTraceMessage( const MString &message ) const {
		MGlobal::displayInfo("twistSplineDrawOverride: " + message);

		// Some simple custom message formatting.
		fprintf(stderr, "twistSplineDrawOverride: ");
		fprintf(stderr, message.asChar());
		fprintf(stderr, "\n");
	}

private:
	TwistSplineDrawOverride(const MObject& obj);

	static void OnModelEditorChanged(void *clientData);

	TwistSplineNode *tsn;
	MCallbackId fModelEditorChangedCbId;
};

