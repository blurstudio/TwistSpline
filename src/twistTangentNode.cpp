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

MObject TwistTangentNode::aOutTwistUpX;
MObject TwistTangentNode::aOutTwistUpY;
MObject TwistTangentNode::aOutTwistUpZ;
MObject TwistTangentNode::aOutTwistUp;

MObject TwistTangentNode::aOutTwistMat;

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
MObject TwistTangentNode::aBackpoint;
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

	// The final output point relative to the parentInverseMatrix
	// This is the data that will eventually get plugged into the spline
	aOutX = uAttr.create("outX", "ox", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aOutX")
	aOutY = uAttr.create("outY", "oy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aOutY")
	aOutZ = uAttr.create("outZ", "oz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aOutZ")
	aOut = nAttr.create("out", "out", aOutX, aOutY, aOutZ, &status);
	CHECKSTAT("aOut")
	addAttribute(aOut);

	// The position of the smooth tangent relative to the parentInverseMatrix
	aSmoothTanX = uAttr.create("smoothTanX", "stx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aSmoothTanX")
	aSmoothTanY = uAttr.create("smoothTanY", "sty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aSmoothTanY")
	aSmoothTanZ = uAttr.create("smoothTanZ", "stz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aSmoothTanZ")
	aSmoothTan = nAttr.create("smoothTan", "st", aSmoothTanX, aSmoothTanY, aSmoothTanZ, &status);
	CHECKSTAT("aSmoothTan")
	addAttribute(aSmoothTan);

	// The target that the adjacent tangent will point at when it's in auto-linear mode
	// relative to the parentInverseMatrix
	aOutLinearTargetX = uAttr.create("outLinearTargetX", "ltx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aLinearTargetX")
	aOutLinearTargetY = uAttr.create("outLinearTargetY", "lty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aLinearTargetY")
	aOutLinearTargetZ = uAttr.create("outLinearTargetZ", "ltz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aLinearTargetZ")
	aOutLinearTarget = nAttr.create("outLinearTarget", "lt", aOutLinearTargetX, aOutLinearTargetY, aOutLinearTargetZ, &status);
	CHECKSTAT("aLinearTarget")
	addAttribute(aOutLinearTarget);

	// The up-vector based off the prev and next vertices, relative to the parentInverseMatrix
	aOutTwistUpX = uAttr.create("outTwistUpX", "otx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aTwistUpX")
	aOutTwistUpY = uAttr.create("outTwistUpY", "oty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aTwistUpY")
	aOutTwistUpZ = uAttr.create("outTwistUpZ", "otz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aTwistUpZ")
	aOutTwistUp = nAttr.create("outTwistUp", "ot", aOutTwistUpX, aOutTwistUpY, aOutTwistUpZ, &status);
	CHECKSTAT("aTwistUp")
	addAttribute(aOutTwistUp);

	// The up-vector matrix based off the prev and next vertices relative to the parentInverseMatrix
	aOutTwistMat = mAttr.create("outTwistMat", "otm");
	mAttr.setDefault(MMatrix::identity);
	addAttribute(aOutTwistMat);

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

	aNextLinearTargetX = uAttr.create("inLinearTargetX", "nltx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aNextLinearTargetX")
	aNextLinearTargetY = uAttr.create("inLinearTargetY", "nlty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECKSTAT("aNextLinearTargetY")
	aNextLinearTargetZ = uAttr.create("inLinearTargetZ", "nltz", MFnUnitAttribute::kDistance, 0.0, &status);
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
	
	aBackpoint = nAttr.create("backpoint", "bp", MFnNumericData::kBoolean, false);
	addAttribute(aBackpoint);

	aEndpoint = nAttr.create("endpoint", "ep", MFnNumericData::kBoolean, false);
	addAttribute(aEndpoint);

	// Affects code
	std::vector<MObject *> smoothAffectors, smoothAffecteds;
	std::vector<MObject *> linearAffectors, linearAffecteds;
	std::vector<MObject *> outAffectors, outAffecteds;

	smoothAffectors.push_back(&aPrevVertex);
	smoothAffectors.push_back(&aCurrentVertex);
	smoothAffectors.push_back(&aNextVertex);
	smoothAffectors.push_back(&aWeight);
	smoothAffectors.push_back(&aBackpoint);
	smoothAffectors.push_back(&aEndpoint);

	smoothAffecteds.push_back(&aSmoothTan);
	smoothAffecteds.push_back(&aSmoothTanX);
	smoothAffecteds.push_back(&aSmoothTanY);
	smoothAffecteds.push_back(&aSmoothTanZ);
	smoothAffecteds.push_back(&aOutTwistUp);
	smoothAffecteds.push_back(&aOutTwistUpX);
	smoothAffecteds.push_back(&aOutTwistUpY);
	smoothAffecteds.push_back(&aOutTwistUpZ);
	smoothAffecteds.push_back(&aOutTwistMat);

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

MStatus TwistTangentNode::compute(const MPlug& plug, MDataBlock& data) {
	// Don't care what plug it is, just compute everything and set the outputs clean
	MStatus status;
	// TODO: do this stuff with mVectors or mPoints instead of mMatrixes
	if (
			plug == aSmoothTan || plug == aSmoothTanX || plug == aSmoothTanY || plug == aSmoothTanZ ||
			plug == aOutTwistUp || plug == aOutTwistUpX || plug == aOutTwistUpY || plug == aOutTwistUpZ ||
			plug == aOutTwistMat
		)
	{
		// Calculate the weighted smooth tangents explicitly
		// Get the first matrix
		MDataHandle preH = data.inputValue(aPrevVertex);
		MDataHandle curH = data.inputValue(aCurrentVertex);
		MDataHandle nextH = data.inputValue(aNextVertex);
		MDataHandle weightH = data.inputValue(aWeight);
		MDataHandle bpH = data.inputValue(aBackpoint);
		MDataHandle epH = data.inputValue(aEndpoint);

		MTransformationMatrix preTMat(preH.asMatrix());
		MTransformationMatrix curTMat(curH.asMatrix());
		MTransformationMatrix nextTMat(nextH.asMatrix());
		bool isBackpoint = bpH.asBool();
		bool isEndpoint = epH.asBool();

		MVector preTfm = preTMat.getTranslation(MSpace::kWorld);
		MVector curTfm = curTMat.getTranslation(MSpace::kWorld);
		MVector nextTfm = nextTMat.getTranslation(MSpace::kWorld);
		double weight = weightH.asDouble();

        MVector curNrm = curTMat.asMatrix()[1];

		MVector preLeg = preTfm - curTfm;
		MVector nextLeg = nextTfm - curTfm;
		double preLegLen = preLeg.length();
		double nextLegLen = nextLeg.length();

		// We're pointing our tangent from pre->post 
		// This is an auto-tan point, so get the half-angle between
		// the perpendiculars of the legs
		MVector smo, tan, nrm, bin;
		MVector preNorm = preLeg / preLegLen;
		MVector postNorm = nextLeg / nextLegLen;
		double dot = preNorm * postNorm;
		if (abs(dot) >= 0.999999999 || preLegLen == 0.0) { // Linear case
			tan = nextLeg.normal();
			
			// If we're in a straight line, default to using the local
			// y-axis as the up-direction
			//nrm = MVector(0.0, 1.0, 0.0);
            nrm = curNrm;
			bin = (nrm ^ tan).normal();
			nrm = (tan ^ bin).normal();
			smo = nextLeg / 3.0;
		}
		else if (!isEndpoint){ // Nonlinear

			bin = (preNorm ^ postNorm).normal();
			tan = ((bin ^ preNorm) + (bin ^ postNorm)).normal();
			smo = tan * (nextLegLen / 3.0);

			//nrm = MVector(0.0, 1.0, 0.0);
            nrm = curNrm;
			bin = (nrm ^ tan).normal();
			nrm = (tan ^ bin).normal();

		}
		else {
			// We are defining the twist of an endpoint, so in this
			// case, the Prev leg is pointing to the *third* CV just
			// to get the up-vector, and the tangent will just be 
			// the postNorm
			tan = postNorm;
			bin = (preNorm ^ postNorm).normal();

			// The smooth is still the same thing
			smo = ((bin ^ preNorm) + (bin ^ postNorm)).normal();
			smo = smo * (nextLegLen / 3.0);

			//nrm = MVector(0.0, 1.0, 0.0);
            nrm = curNrm;
			bin = (nrm ^ tan).normal();
			nrm = (tan ^ bin).normal();

		}
		// If we're a tangent node coming the opposite direction
		// Then ensure that the up-matrix still has +x pointing
		// the right direction
		if (isBackpoint){
			tan *= -1;
			bin *= -1;
		}

		smo *= weight;

		// x-> tan, y-> nrm, z->bin
		double mm[4][4] = {
			{tan[0],	tan[1],    tan[2],	  0.0},
			{nrm[0],	nrm[1],    nrm[2],	  0.0},
			{bin[0],	bin[1],    bin[2],	  0.0},
			{curTfm[0], curTfm[1], curTfm[2], 1.0}
		};
		MMatrix outMat(mm);

		MDataHandle vecH = data.outputValue(aOutTwistUp);
		vecH.setMVector(nrm);
		vecH.setClean();

		MDataHandle tanH = data.outputValue(aSmoothTan);
		tanH.setMVector(smo);
		tanH.setClean();



		MDataHandle invParH = data.inputValue(aParentInverseMatrix);
		MMatrix invParMat = invParH.asMatrix();

		MDataHandle matH = data.outputValue(aOutTwistMat);
		matH.setMMatrix(outMat * invParMat);
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

