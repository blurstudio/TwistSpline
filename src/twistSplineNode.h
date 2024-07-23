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

static constexpr const char gPluginNodeMessagePrefix[] = "TwistSpline: ";

class TwistSplineNode : public MPxLocatorNode
{
public:
	TwistSplineNode();
	virtual	~TwistSplineNode();

	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;
	MStatus postEvaluation(const MDGContext& context,
						   const MEvaluationNode& evaluationNode,
						   PostEvaluationType evalType) override;
	void getCacheSetup(const MEvaluationNode& evalNode,
					   MNodeCacheDisablingInfo& disablingInfo,
					   MNodeCacheSetupInfo& cacheSetupInfo,
					   MObjectArray& monitoredAttributes) const override;
	void postConstructor() override;
	static void* creator();
	static	MStatus	initialize();
	virtual bool isBounded() const { return true; }
	virtual MBoundingBox boundingBox() const;
	TwistSplineT* getSplineData() const;
	void getDebugDraw(bool &oDraw, double &oScale) const;
	void getSplineDraw(bool &oDraw) const;

	// Methods for the renderer to call
	bool isGeometryChanging() const;
	void updateRenderAttributes();
	void updatingGeometry();

public:
	static MObject aOutputSpline;
	static MObject aSplineLength;
	static MObject aMaxVertices;

	// NURBS curve output data
	static MObject aNurbsData;

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

	static MObject aGeometryChanging;
	static MObject aSplineDisplay;
	static MObject aDebugDisplay;
	static MObject aDebugScale;

	static MTypeId id;
	static MString drawDbClassification;
	static MString drawRegistrantId;
};
