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

MObject TwistTangentNode::aParentInverseMatrix;
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
	aOut = nAttr.create("out", "out", aOutX, aOutY, aOutZ, &status);
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
	aParentInverseMatrix = mAttr.create("parentInverseMatrix", "pim");
	mAttr.setHidden(true);
	mAttr.setWritable(true);
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aParentInverseMatrix);
	
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
	
	// Affects code
	std::vector<MObject *> smoothAffectors, smoothAffecteds;
	std::vector<MObject *> linearAffectors, linearAffecteds;
	std::vector<MObject *> outAffectors, outAffecteds;

	smoothAffectors.push_back(&aPrevVertex);
	smoothAffectors.push_back(&aCurrentVertex);
	smoothAffectors.push_back(&aNextVertex);
	smoothAffectors.push_back(&aWeight);

	smoothAffecteds.push_back(&aSmoothTan);
	smoothAffecteds.push_back(&aSmoothTanX);
	smoothAffecteds.push_back(&aSmoothTanY);
	smoothAffecteds.push_back(&aSmoothTanZ);

	for (auto s : smoothAffectors) {
		for (auto t : smoothAffecteds) {
			attributeAffects(*s, *t);
		}
	}
	
	linearAffectors.push_back(&aSmooth);
	linearAffectors.push_back(&aSmoothTan);
	linearAffectors.push_back(&aCurrentVertex);
	linearAffectors.push_back(&aWeight);
	linearAffectors.push_back(&aPrevVertex);
	linearAffectors.push_back(&aCurrentVertex);
	linearAffectors.push_back(&aNextVertex);

	linearAffecteds.push_back(&aOutLinearTarget);
	linearAffecteds.push_back(&aOutLinearTargetX);
	linearAffecteds.push_back(&aOutLinearTargetY);
	linearAffecteds.push_back(&aOutLinearTargetZ);

	for (auto s : linearAffectors) {
		for (auto t : linearAffecteds) {
			attributeAffects(*s, *t);
		}
	}

	outAffectors.push_back(&aParentInverseMatrix);
	outAffectors.push_back(&aInTangent);
	outAffectors.push_back(&aCurrentVertex);
	outAffectors.push_back(&aSmoothTan);
	outAffectors.push_back(&aNextLinearTarget);
	outAffectors.push_back(&aSmooth);
	outAffectors.push_back(&aAuto);
	outAffectors.push_back(&aWeight);
	outAffectors.push_back(&aPrevVertex);
	outAffectors.push_back(&aCurrentVertex);
	outAffectors.push_back(&aNextVertex);

	outAffecteds.push_back(&aOut);
	outAffecteds.push_back(&aOutX);
	outAffecteds.push_back(&aOutY);
	outAffecteds.push_back(&aOutZ);

	for (auto s : outAffectors) {
		for (auto t : outAffecteds) {
			attributeAffects(*s, *t);
		}
	}

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
		if (abs(dot) >= 0.999999999 || preLegLen == 0.0) { // Linear case
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

		MDataHandle invParH = data.inputValue(aParentInverseMatrix);
		MMatrix invParMat = invParH.asMatrix();

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

		result = (1.0 - smooth) * lin + smooth * smo;

		MVector freeLeg = inTfm - curTfm;
		result = (1.0 - autoTan) * freeLeg + autoTan * result; // LERP with the free tangent
		result += curTfm;

		MPoint out = MPoint(result) * invParMat;

		MDataHandle outH = data.outputValue(aOut);
		outH.set3Double(out[0], out[1], out[2]);
		outH.setClean();
	}
	else {
		return MS::kUnknownParameter;
	}
	return MS::kSuccess;
}

