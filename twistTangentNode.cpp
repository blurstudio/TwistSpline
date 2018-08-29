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

#include <vector>

#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MTypes.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MVector.h>

#include "twistTangentNode.h"

#define CHECKSTAT(m) if (!status) {status.perror(m); return status;};

MTypeId	TwistTangentNode::id(0x001226FA);

MObject TwistTangentNode::aOutLinearTarget;
MObject TwistTangentNode::aOutLinearTargetX;
MObject TwistTangentNode::aOutLinearTargetY;
MObject TwistTangentNode::aOutLinearTargetZ;

MObject TwistTangentNode::aSmoothTan;
MObject TwistTangentNode::aSmoothTanX;
MObject TwistTangentNode::aSmoothTanY;
MObject TwistTangentNode::aSmoothTanZ;

MObject TwistTangentNode::aOut;
MObject TwistTangentNode::aOutX;
MObject TwistTangentNode::aOutY;
MObject TwistTangentNode::aOutZ;

MObject TwistTangentNode::aInTangent;
MObject TwistTangentNode::aPrevVertex;
MObject TwistTangentNode::aCurrentVertex;
MObject TwistTangentNode::aNextVertex;
MObject TwistTangentNode::aNextLinearTarget;
MObject TwistTangentNode::aNextLinearTargetX;
MObject TwistTangentNode::aNextLinearTargetY;
MObject TwistTangentNode::aNextLinearTargetZ;
MObject TwistTangentNode::aAuto;
MObject TwistTangentNode::aSmooth;
MObject TwistTangentNode::aWeight;
MObject TwistTangentNode::aEndpoint;

TwistTangentNode::TwistTangentNode() {}
TwistTangentNode::~TwistTangentNode() {}

void* TwistTangentNode::creator() {
	return new TwistTangentNode();
}

MStatus TwistTangentNode::initialize() {
	MFnMatrixAttribute mAttr;
	MFnNumericAttribute nAttr;
	MFnEnumAttribute eAttr;
	MFnUnitAttribute uAttr;
	MStatus status;

	//----------------- Outputs -----------------

	aOutX = nAttr.create("outX", "ox", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aOutX")
	aOutY = nAttr.create("outY", "oy", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aOutY")
	aOutZ = nAttr.create("outZ", "oz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aOutZ")
	aOut = nAttr.create("out", "v", aOutX, aOutY, aOutZ, &status);
	CHECKSTAT("aOut")
	addAttribute(aOut);

	aSmoothTanX = nAttr.create("smoothTanX", "stx", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aSmoothTanX")
	aSmoothTanY = nAttr.create("smoothTanY", "sty", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aSmoothTanY")
	aSmoothTanZ = nAttr.create("smoothTanZ", "stz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aSmoothTanZ")
	aSmoothTan = nAttr.create("smoothTan", "st", aSmoothTanX, aSmoothTanY, aSmoothTanZ, &status);
	CHECKSTAT("aSmoothTan")
	addAttribute(aSmoothTan);

	aOutLinearTargetX = nAttr.create("outLinearTargetX", "ltx", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aLinearTargetX")
	aOutLinearTargetY = nAttr.create("outLinearTargetY", "lty", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aLinearTargetY")
	aOutLinearTargetZ = nAttr.create("outLinearTargetZ", "ltz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aLinearTargetZ")
	aOutLinearTarget = nAttr.create("outLinearTarget", "lt", aOutLinearTargetX, aOutLinearTargetY, aOutLinearTargetZ, &status);
	CHECKSTAT("aLinearTarget")
	addAttribute(aOutLinearTarget);

	//----------------- Matrices -----------------
	aInTangent = mAttr.create("inTangent", "it");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aInTangent);

	aPrevVertex = mAttr.create("previousVertex", "pv");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aPrevVertex);

	aCurrentVertex = mAttr.create("currentVertex", "cv");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aCurrentVertex);

	aNextVertex = mAttr.create("nextVertex", "nv");
	mAttr.setHidden(true);
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aNextVertex);

	aNextLinearTargetX = nAttr.create("inLinearTargetX", "nltx", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aNextLinearTargetX")
	aNextLinearTargetY = nAttr.create("inLinearTargetY", "nlty", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aNextLinearTargetY")
	aNextLinearTargetZ = nAttr.create("inLinearTargetZ", "nltz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aNextLinearTargetZ")
	aNextLinearTarget = nAttr.create("inLinearTarget", "nlt", aNextLinearTargetX, aNextLinearTargetY, aNextLinearTargetZ, &status);
	CHECKSTAT("aNextLinearTarget")
	addAttribute(aNextLinearTarget);

	//----------------- Weights -----------------
	aAuto = nAttr.create("auto", "a", MFnNumericData::kDouble, 0.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setKeyable(true);
	addAttribute(aAuto);

	aSmooth = nAttr.create("smooth", "s", MFnNumericData::kDouble, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setKeyable(true);
	addAttribute(aSmooth);

	aWeight = nAttr.create("weight", "w", MFnNumericData::kDouble, 1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(3.0);
	nAttr.setKeyable(true);
	addAttribute(aWeight);
	
	attributeAffects(aPrevVertex, aSmoothTan);
	attributeAffects(aPrevVertex, aSmoothTanX);
	attributeAffects(aPrevVertex, aSmoothTanY);
	attributeAffects(aPrevVertex, aSmoothTanZ);
	attributeAffects(aCurrentVertex, aSmoothTan);
	attributeAffects(aCurrentVertex, aSmoothTanX);
	attributeAffects(aCurrentVertex, aSmoothTanY);
	attributeAffects(aCurrentVertex, aSmoothTanZ);
	attributeAffects(aNextVertex, aSmoothTan);
	attributeAffects(aNextVertex, aSmoothTanX);
	attributeAffects(aNextVertex, aSmoothTanY);
	attributeAffects(aNextVertex, aSmoothTanZ);
	attributeAffects(aWeight, aSmoothTan);
	attributeAffects(aWeight, aSmoothTanX);
	attributeAffects(aWeight, aSmoothTanY);
	attributeAffects(aWeight, aSmoothTanZ);

	attributeAffects(aSmooth, aOutLinearTarget);
	attributeAffects(aSmooth, aOutLinearTargetX);
	attributeAffects(aSmooth, aOutLinearTargetY);
	attributeAffects(aSmooth, aOutLinearTargetZ);
	attributeAffects(aSmoothTan, aOutLinearTarget);
	attributeAffects(aSmoothTan, aOutLinearTargetX);
	attributeAffects(aSmoothTan, aOutLinearTargetY);
	attributeAffects(aSmoothTan, aOutLinearTargetZ);
	attributeAffects(aCurrentVertex, aOutLinearTarget);
	attributeAffects(aCurrentVertex, aOutLinearTargetX);
	attributeAffects(aCurrentVertex, aOutLinearTargetY);
	attributeAffects(aCurrentVertex, aOutLinearTargetZ);
	attributeAffects(aWeight, aOutLinearTarget);
	attributeAffects(aWeight, aOutLinearTargetX);
	attributeAffects(aWeight, aOutLinearTargetY);
	attributeAffects(aWeight, aOutLinearTargetZ);
	attributeAffects(aPrevVertex, aOutLinearTarget);
	attributeAffects(aPrevVertex, aOutLinearTargetX);
	attributeAffects(aPrevVertex, aOutLinearTargetY);
	attributeAffects(aPrevVertex, aOutLinearTargetZ);
	attributeAffects(aCurrentVertex, aOutLinearTarget);
	attributeAffects(aCurrentVertex, aOutLinearTargetX);
	attributeAffects(aCurrentVertex, aOutLinearTargetY);
	attributeAffects(aCurrentVertex, aOutLinearTargetZ);
	attributeAffects(aNextVertex, aOutLinearTarget);
	attributeAffects(aNextVertex, aOutLinearTargetX);
	attributeAffects(aNextVertex, aOutLinearTargetY);
	attributeAffects(aNextVertex, aOutLinearTargetZ);

	attributeAffects(aInTangent, aOut);
	attributeAffects(aInTangent, aOutX);
	attributeAffects(aInTangent, aOutY);
	attributeAffects(aInTangent, aOutZ);
	attributeAffects(aCurrentVertex, aOut);
	attributeAffects(aCurrentVertex, aOutX);
	attributeAffects(aCurrentVertex, aOutY);
	attributeAffects(aCurrentVertex, aOutZ);
	attributeAffects(aSmoothTan, aOut);
	attributeAffects(aSmoothTan, aOutX);
	attributeAffects(aSmoothTan, aOutY);
	attributeAffects(aSmoothTan, aOutZ);
	attributeAffects(aNextLinearTarget, aOut);
	attributeAffects(aNextLinearTarget, aOutX);
	attributeAffects(aNextLinearTarget, aOutY);
	attributeAffects(aNextLinearTarget, aOutZ);
	attributeAffects(aSmooth, aOut);
	attributeAffects(aSmooth, aOutX);
	attributeAffects(aSmooth, aOutY);
	attributeAffects(aSmooth, aOutZ);
	attributeAffects(aAuto, aOut);
	attributeAffects(aAuto, aOutX);
	attributeAffects(aAuto, aOutY);
	attributeAffects(aAuto, aOutZ);
	attributeAffects(aWeight, aOut);
	attributeAffects(aWeight, aOutX);
	attributeAffects(aWeight, aOutY);
	attributeAffects(aWeight, aOutZ);
	attributeAffects(aPrevVertex, aOut);
	attributeAffects(aPrevVertex, aOutX);
	attributeAffects(aPrevVertex, aOutY);
	attributeAffects(aPrevVertex, aOutZ);
	attributeAffects(aCurrentVertex, aOut);
	attributeAffects(aCurrentVertex, aOutX);
	attributeAffects(aCurrentVertex, aOutY);
	attributeAffects(aCurrentVertex, aOutZ);
	attributeAffects(aNextVertex, aOut);
	attributeAffects(aNextVertex, aOutX);
	attributeAffects(aNextVertex, aOutY);
	attributeAffects(aNextVertex, aOutZ);
	return MS::kSuccess;
}

MStatus	TwistTangentNode::compute(const MPlug& plug, MDataBlock& data) {
	// Don't care what plug it is, just compute everything and set the outputs clean
	MStatus status;
	// TODO: do this stuff with mVectors or mPoints instead of mMatrixes
	if (plug == aSmoothTan || plug == aSmoothTanX || plug == aSmoothTanY || plug == aSmoothTanZ) {
		// Calculate the weighted smooth tangents explicitly
		// Get the first matrix
		MDataHandle preH = data.inputValue(aPrevVertex);
		MDataHandle curH = data.inputValue(aCurrentVertex);
		MDataHandle nextH = data.inputValue(aNextVertex);
		MDataHandle weightH = data.inputValue(aWeight);

		MTransformationMatrix preTMat(preH.asMatrix());
		MTransformationMatrix curTMat(curH.asMatrix());
		MTransformationMatrix nextTMat(nextH.asMatrix());

		MVector preTfm = preTMat.getTranslation(MSpace::kWorld);
		MVector curTfm = curTMat.getTranslation(MSpace::kWorld);
		MVector nextTfm = nextTMat.getTranslation(MSpace::kWorld);
		double weight = weightH.asDouble();

		MVector preLeg = preTfm - curTfm;
		MVector nextLeg = nextTfm - curTfm;
		double preLegLen = preLeg.length();
		double nextLegLen = nextLeg.length();

		// We're pointing our tangent from pre->post 
		// This is an auto-tan point, so get the half-angle between
		// the perpendiculars of the legs
		MVector smo;
		MVector preNorm = preLeg / preLegLen;
		MVector postNorm = nextLeg / nextLegLen;
		double dot = preNorm * postNorm;
		if (abs(dot) == 1.0 || preLegLen == 0.0) { // Linear case
			smo = nextLeg / 3.0;
		}
		else { // Nonlinear
			MVector bin = preNorm ^ postNorm;
			bin.normalize();
			smo = ((bin ^ preNorm) + (bin ^ postNorm)).normal();
			smo *= nextLegLen / 3.0;
		}
		smo *= weight;
		MDataHandle matH = data.outputValue(aSmoothTan);
		matH.setMVector(smo);
		matH.setClean();
	}
	else if (plug == aOutLinearTarget || plug == aOutLinearTargetX || plug == aOutLinearTargetY || plug == aOutLinearTargetZ) {
		// Calculate 
		MDataHandle inH = data.inputValue(aSmoothTan);
		MVector smo = inH.asVector();
		MDataHandle smoothH = data.inputValue(aSmooth);
		double smooth = smoothH.asDouble();
		smo *= smooth;

		MDataHandle curH = data.inputValue(aCurrentVertex);
		MTransformationMatrix curTMat(curH.asMatrix());
		MVector curTfm = curTMat.getTranslation(MSpace::kWorld);

		MDataHandle matH = data.outputValue(aOutLinearTarget);
		matH.setMVector(smo + curTfm);
		matH.setClean();
	}
	else if (plug == aOut || plug == aOutX || plug == aOutY || plug == aOutZ) {
		// Get the first matrix
		MDataHandle inH = data.inputValue(aInTangent);
		MTransformationMatrix inTMat(inH.asMatrix());
		MVector inTfm = inTMat.getTranslation(MSpace::kWorld);

		MDataHandle curH = data.inputValue(aCurrentVertex);
		MTransformationMatrix curTMat(curH.asMatrix());
		MVector curTfm = curTMat.getTranslation(MSpace::kWorld);

		MDataHandle smoH = data.inputValue(aSmoothTan);
		MVector smo = smoH.asVector();

		MDataHandle nltH = data.inputValue(aNextLinearTarget);
		MVector nlt = nltH.asVector();

		MDataHandle smoothH = data.inputValue(aSmooth);
		double smooth = smoothH.asDouble();

		MDataHandle weightH = data.inputValue(aWeight);
		double weight = weightH.asDouble();

		MDataHandle autoH = data.inputValue(aAuto);
		double autoTan = autoH.asDouble();

		MVector result;
		MVector lin = (nlt - curTfm).normal() * smo.length();

		if (smooth == 0.0){
			result = lin;
		}
		else if (smooth == 1.0){
			result = smo;
		}
		else {
			result = lin + smooth*(smo - lin);
		}

		MVector freeLeg = inTfm - curTfm;
		result = freeLeg + autoTan*(result - freeLeg); // LERP with the free tangent
		result += curTfm;

		MDataHandle matH = data.outputValue(aOut);
		matH.setMVector(result);
		matH.setClean();
	}
	else {
		return MS::kUnknownParameter;
	}
	return MS::kSuccess;
}

