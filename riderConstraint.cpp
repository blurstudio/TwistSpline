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

#include "riderConstraint.h"

#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MObjectArray.h>
#include <maya/MPxData.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>

#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MTypes.h>

#include <vector>
#include "twistSpline.h"
#include "twistSplineData.h"


#define ROTATE_ORDER_XYZ        0
#define ROTATE_ORDER_YZX        1
#define ROTATE_ORDER_ZXY        2
#define ROTATE_ORDER_XZY        3
#define ROTATE_ORDER_YXZ        4
#define ROTATE_ORDER_ZYX        5

#define CHECKSTAT(m) if (!status) {status.perror(m); return status;};

MTypeId     riderConstraint::id(0x001226FC);

MObject     riderConstraint::aRotateOrder;
MObject     riderConstraint::aGlobalOffset;
MObject     riderConstraint::aGlobalSpread;
MObject     riderConstraint::aUseCycle;
MObject     riderConstraint::aNormalize;
MObject     riderConstraint::aNormValue;

// inputs
MObject     riderConstraint::aInputSplines;
	MObject     riderConstraint::aSpline;
	MObject     riderConstraint::aWeight;

MObject     riderConstraint::aParams;
	MObject     riderConstraint::aParam;
	MObject     riderConstraint::aParentInverseMatrix;

// output
MObject     riderConstraint::aOutputs;
	MObject     riderConstraint::aTranslate;
	MObject     riderConstraint::aTranslateX;
	MObject     riderConstraint::aTranslateY;
	MObject     riderConstraint::aTranslateZ;
	MObject     riderConstraint::aRotate;
	MObject     riderConstraint::aRotateX;
	MObject     riderConstraint::aRotateY;
	MObject     riderConstraint::aRotateZ;
	MObject     riderConstraint::aScale;
	MObject     riderConstraint::aScaleX;
	MObject     riderConstraint::aScaleY;
	MObject     riderConstraint::aScaleZ;

MStatus riderConstraint::initialize() {
	MStatus status;
	MFnTypedAttribute tAttr;
	MFnNumericAttribute nAttr;
	MFnMatrixAttribute mAttr;
	MFnUnitAttribute uAttr;
	MFnEnumAttribute eAttr;
	MFnCompoundAttribute cAttr;

	// Single inputs
	aRotateOrder = eAttr.create("rotateOrder", "ro", ROTATE_ORDER_XYZ, &status);
	CHECKSTAT("aRotateOrder");
	eAttr.setKeyable(true);
	status = eAttr.addField("xyz", ROTATE_ORDER_XYZ);
	CHECKSTAT("aRotateOrder");
	status = eAttr.addField("yzx", ROTATE_ORDER_YZX);
	CHECKSTAT("aRotateOrder");
	status = eAttr.addField("zxy", ROTATE_ORDER_ZXY);
	CHECKSTAT("aRotateOrder");
	status = eAttr.addField("xzy", ROTATE_ORDER_XZY);
	CHECKSTAT("aRotateOrder");
	status = eAttr.addField("yxz", ROTATE_ORDER_YXZ);
	CHECKSTAT("aRotateOrder");
	status = eAttr.addField("zyx", ROTATE_ORDER_ZYX);
	CHECKSTAT("aRotateOrder");
	status = addAttribute(aRotateOrder);
	CHECKSTAT("aRotateOrder");

	aGlobalOffset = nAttr.create("globalOffset", "go", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aGlobalOffset");
	nAttr.setKeyable(true);
	status = addAttribute(aGlobalOffset);
	CHECKSTAT("aGlobalOffset");
	
	aGlobalSpread = nAttr.create("globalSpread", "gs", MFnNumericData::kDouble, 1.0, &status);
	CHECKSTAT("aGlobalSpread");
	nAttr.setKeyable(true);
	status = addAttribute(aGlobalSpread);
	CHECKSTAT("aGlobalSpread");

	aUseCycle = nAttr.create("useCycle", "uc", MFnNumericData::kBoolean, false, &status);
	CHECKSTAT("aUseCycle");
	nAttr.setKeyable(true);
	status = addAttribute(aUseCycle);
	CHECKSTAT("aUseCycle");


	aNormalize = nAttr.create("normalize", "n", MFnNumericData::kDouble, 1.0, &status);
	CHECKSTAT("aNormalize");
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setKeyable(true);
	status = addAttribute(aNormalize);
	CHECKSTAT("aNormalize");

	aNormValue = nAttr.create("normValue", "nv", MFnNumericData::kDouble, 1.0, &status);
	CHECKSTAT("aNormValue");
	nAttr.setMin(0.0);
	nAttr.setKeyable(true);
	status = addAttribute(aNormValue);
	CHECKSTAT("aNormValue");

	// Spline input array
	aSpline = tAttr.create("spline", "s", TwistSplineData::id, MObject::kNullObj, &status);
	tAttr.setHidden(true);
	CHECKSTAT("aSpline");

	aWeight = nAttr.create("weight", "w", MFnNumericData::kDouble, 1.0, &status);
	CHECKSTAT("aWeight");
	nAttr.setKeyable(true);

	aInputSplines = cAttr.create("inputSplines", "is", &status);
	CHECKSTAT("aInputSplines");
	cAttr.setArray(true);
	cAttr.addChild(aSpline);
	cAttr.addChild(aWeight);

	status = addAttribute(aInputSplines);
	CHECKSTAT("aInputSplines");

	// input param array
	aParam = nAttr.create("param", "p", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aParam");
	nAttr.setKeyable(true);

	aParentInverseMatrix = mAttr.create("parentInverseMatrix", "pim", MFnMatrixAttribute::kDouble, &status);
	CHECKSTAT("aParentInverseMatrix");
	mAttr.setHidden(true);

	aParams = cAttr.create("params", "ps", &status);
	cAttr.setArray(true);
	cAttr.addChild(aParam);
	cAttr.addChild(aParentInverseMatrix);
	status = addAttribute(aParams);
	CHECKSTAT("aParams");

	// Output: Matrices
	aTranslateX = nAttr.create("translateX", "tx", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aTranslateX");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aTranslateY = nAttr.create("translateY", "ty", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aTranslateY");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aTranslateZ = nAttr.create("translateZ", "tz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aTranslateZ");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aTranslate = nAttr.create("translate", "t", aTranslateX, aTranslateY, aTranslateZ, &status);
	nAttr.setHidden(true);
	CHECKSTAT("aTranslate");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	// Output Attributes
	aRotateX = uAttr.create("rotateX", "rotx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECKSTAT("aRotateX");
	uAttr.setWritable(false);
	uAttr.setStorable(false);

	aRotateY = uAttr.create("rotateY", "roty", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECKSTAT("aRotateY");
	uAttr.setWritable(false);
	uAttr.setStorable(false);

	aRotateZ = uAttr.create("rotateZ", "rotz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECKSTAT("aRotateZ");
	uAttr.setWritable(false);
	uAttr.setStorable(false);

	aRotate = nAttr.create("rotate", "rot", aRotateX, aRotateY, aRotateZ, &status);
	nAttr.setHidden(true);
	CHECKSTAT("aRotate");
	nAttr.setWritable(false);

	// Output Attributes
	aScaleX = nAttr.create("scaleX", "sclx", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aScaleX");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aScaleY = nAttr.create("scaleY", "scly", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aScaleY");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aScaleZ = nAttr.create("scaleZ", "sclz", MFnNumericData::kDouble, 0.0, &status);
	CHECKSTAT("aScaleZ");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aScale = nAttr.create("scale", "scl", aScaleX, aScaleY, aScaleZ, &status);
	nAttr.setHidden(true);
	CHECKSTAT("aScale");
	nAttr.setWritable(false);
	nAttr.setStorable(false);

	aOutputs = cAttr.create("outputs", "out", &status);
	CHECKSTAT("aOutputs");
	cAttr.setHidden(true);
	cAttr.setArray(true);
	cAttr.setUsesArrayDataBuilder(true);
	cAttr.addChild(aTranslate);
	cAttr.addChild(aRotate);
	cAttr.addChild(aScale);

	status = addAttribute(aOutputs);
	CHECKSTAT("aOutputs");

	attributeAffects(aRotateOrder, aRotate);
	attributeAffects(aRotateOrder, aRotateX);
	attributeAffects(aRotateOrder, aRotateY);
	attributeAffects(aRotateOrder, aRotateZ);

	std::vector<MObject *> iobjs, oobjs;
	iobjs.push_back(&aInputSplines);
	iobjs.push_back(&aSpline);
	iobjs.push_back(&aWeight);
	iobjs.push_back(&aParams);
	iobjs.push_back(&aParam);
	iobjs.push_back(&aParentInverseMatrix);
	iobjs.push_back(&aGlobalOffset);
	iobjs.push_back(&aUseCycle);
	iobjs.push_back(&aGlobalSpread);
	iobjs.push_back(&aNormalize);
	iobjs.push_back(&aNormValue);

	oobjs.push_back(&aOutputs);
	oobjs.push_back(&aTranslate);
	oobjs.push_back(&aRotate);
	oobjs.push_back(&aScale);
	oobjs.push_back(&aTranslateX);
	oobjs.push_back(&aRotateX);
	oobjs.push_back(&aScaleX);
	oobjs.push_back(&aTranslateY);
	oobjs.push_back(&aRotateY);
	oobjs.push_back(&aScaleY);
	oobjs.push_back(&aTranslateZ);
	oobjs.push_back(&aRotateZ);
	oobjs.push_back(&aScaleZ);

	for (auto &ii : iobjs) {
		for (auto &oo : oobjs) {
			attributeAffects(*ii, *oo);
		}
	}

	return MS::kSuccess;
}

MStatus riderConstraint::compute(const MPlug& plug, MDataBlock& data) {
	if (plug == aOutputs || 
		plug == aScale || plug == aScaleX || plug == aScaleY || plug == aScaleZ ||
		plug == aRotate || plug == aRotateX || plug == aRotateY || plug == aRotateZ ||
		plug == aTranslate || plug == aTranslateX || plug == aTranslateY || plug == aTranslateZ
		) { 
		// I can optimize things a lot more if we do everything at once
		// So, whatever plug is asked for, just compute it all
		MStatus status;
		std::vector<TwistSplineT*> splines;
		std::vector<double> weights;
		unsigned ecount, possibleMax;

		// First, get the splines that need computing, and their weight.
		MArrayDataHandle inSpAH = data.inputArrayValue(aInputSplines);
		ecount = inSpAH.elementCount();
		inSpAH.jumpToArrayElement(ecount - 1);
		possibleMax = inSpAH.elementIndex();
		weights.resize(possibleMax+1);
		splines.resize(possibleMax+1);
		inSpAH.jumpToArrayElement(0);

		for (unsigned i = 0; i < ecount; ++i) {
			auto inSpGroup = inSpAH.inputValue();
			unsigned realIndex = inSpAH.elementIndex();
			if (realIndex > possibleMax) {
				weights.resize(realIndex+1);
				splines.resize(realIndex+1);
				possibleMax = realIndex;
			}

			MDataHandle inSpwH = inSpGroup.child(aWeight);

			double inSpw = inSpwH.asDouble();
			if (inSpw <= 0.0) continue;

			MDataHandle inSpH = inSpGroup.child(aSpline);
			MPxData* pd = inSpH.asPluginData();
			if (pd == nullptr) continue;

			auto inSplineData = (TwistSplineData *)pd;
			TwistSplineT *spline = inSplineData->getSpline();
			if (spline == nullptr) continue;

			weights[realIndex] = inSpw;
			splines[realIndex] = spline;

			inSpAH.next();
		}

		if (splines.size() == 0) {
			MArrayDataHandle outHandle = data.outputArrayValue(aOutputs);
			outHandle.setAllClean();
			return MS::kSuccess;
		}

		// normalize the weights
		double s = 0.0;
		for (double &w : weights) s += w;
		for (double &w : weights) w /= s;

		// Get the params
		MArrayDataHandle inPAH = data.inputArrayValue(aParams, &status);
		std::vector<double> params;
		std::vector<MMatrix> invParMats;

		ecount = inPAH.elementCount();
		inPAH.jumpToArrayElement(ecount - 1);
		possibleMax = inPAH.elementIndex();
		params.resize(possibleMax + 1);
		invParMats.resize(possibleMax + 1);
		inPAH.jumpToArrayElement(0);

		for (unsigned i = 0; i < ecount; ++i) {
			unsigned realIndex = inPAH.elementIndex();
			if (realIndex > possibleMax) {
				params.resize(realIndex+1);
				possibleMax = realIndex;
			}
			auto inPGrp = inPAH.inputValue(&status);

			MDataHandle inPH = inPGrp.child(aParam);
			double inParam = inPH.asDouble();
			params[realIndex] = inParam;

			MDataHandle inPimH = inPGrp.child(aParentInverseMatrix);
			MMatrix inPim = inPimH.asMatrix();
			invParMats[realIndex] = inPim;

			inPAH.next();
		}

		// Deal with cycling and the global offset
		MDataHandle gOffsetH = data.inputValue(aGlobalOffset);
		MDataHandle gSpreadH = data.inputValue(aGlobalSpread);
		MDataHandle useCycleH = data.inputValue(aUseCycle);
		MDataHandle normalizeH = data.inputValue(aNormalize);
		MDataHandle normValueH = data.inputValue(aNormValue);

		double doNorm = normalizeH.asDouble();
		double normVal = normValueH.asDouble();
		double gOffset = gOffsetH.asDouble();
		double gSpread = gSpreadH.asDouble();
		bool useCycle = useCycleH.asBool();

		// Loop through the splines to get the transform groups
		// There is no single way to interpolate between orientations
		// I'm just going to do the simple thing (for now) and N-LERP
		// between the quaternion outputs in order

		std::vector<std::vector<MPoint>> trans, scales;
		std::vector<std::vector<MQuaternion>> quats;
		std::vector<std::vector<double>> twists;

		bool twisted = splines.size() == 1;

		for (size_t sIdx = 0; sIdx<splines.size(); ++sIdx) {
			TwistSplineT *spline = splines[sIdx];
			const auto &lp = spline->getLockPositions();
			double mp = lp[lp.size() - 1];
			const auto &rmp = spline->getRemap();
			double mrmp = rmp[rmp.size() - 1];

			std::vector<MPoint> ttrans, tscales;
			std::vector<MQuaternion> tquats;
			std::vector<double> ttwists;

			ttrans.reserve(params.size());
			tscales.reserve(params.size());
			tquats.reserve(params.size());
			ttwists.reserve(params.size());

			for (size_t pIdx = 0; pIdx<params.size(); ++pIdx) {
				// Calculate the transforms
				MVector tan, norm, binorm;
				MPoint tran, scale;

				double twist, p = params[pIdx];
				p *= gSpread;
				p += gOffset;
				if (doNorm > 0.0) {
					p = (doNorm) * (p * mp / normVal) + (1.0 - doNorm) * (p);
				}
				if (useCycle) {
					if (p >= 0.0)
						p = std::fmod(p, mrmp);
					else
						p = mrmp - std::fmod(-p, mrmp);
				}

				spline->matrixAtParam(p, tan, norm, binorm, tran, scale, twist, twisted);
				ttrans.push_back(std::move(tran));
				tscales.push_back(std::move(scale));
				ttwists.push_back(std::move(twist));

				// fill the matrix
				double mat[4][4];
				mat[0][0] = tan[0]; mat[1][0] = norm[0]; mat[2][0] = binorm[0]; mat[3][0] = 0.0;
				mat[0][1] = tan[1]; mat[1][1] = norm[1]; mat[2][1] = binorm[1]; mat[3][1] = 0.0;
				mat[0][2] = tan[2]; mat[1][2] = norm[2]; mat[2][2] = binorm[2]; mat[3][2] = 0.0;
				mat[0][3] = 0.0;    mat[1][3] = 0.0;     mat[2][3] = 0.0;       mat[3][3] = 1.0;
				MQuaternion q;
				q = MMatrix(mat);
				tquats.push_back(std::move(q));
			}
			trans.push_back(ttrans);
			scales.push_back(tscales);
			quats.push_back(tquats);
			twists.push_back(ttwists);
		}

		std::vector<MPoint> otrans, oscales;
		std::vector<MQuaternion> oquats;
		std::vector<double> otwists;
		if (splines.size() > 1) {
			// weight the twists trans and scales
			for (size_t pIdx = 0; pIdx < params.size(); ++pIdx) {
				MPoint tran, scale;
				double twist = 0.0;
				for (size_t sIdx = 0; sIdx < splines.size(); ++sIdx) {
					double w = weights[sIdx];
					tran += trans[sIdx][pIdx] * w;
					scale += scales[sIdx][pIdx] * w;
					twist += twists[sIdx][pIdx] * w;
				}
				otrans.push_back(std::move(tran));
				oscales.push_back(std::move(scale));
				otwists.push_back(std::move(twist));
			}

			// Weight the quaternions
			for (size_t pIdx = 0; pIdx < params.size(); ++pIdx) {
				MQuaternion prev = quats[0][pIdx].normal();
				for (size_t sIdx = 1; sIdx < splines.size(); ++sIdx) {
					MQuaternion cur = quats[sIdx][pIdx];
					double pw = weights[sIdx - 1]; // preWeight
					double cw = weights[sIdx]; // currentWeight
					double s = pw + cw;
					prev = slerp(prev, cur, cw / s);
				}
				MQuaternion xt(otwists[pIdx], MVector(1.0, 0.0, 0.0));
				prev = xt * prev;
				oquats.push_back(std::move(prev));
			}
		}
		else if (splines.size() == 1) {
			// don't weight anything if we don't have to
			otrans = trans[0];
			oscales = scales[0];
			oquats = quats[0];
		}

		MDataHandle hOrder = data.inputValue(aRotateOrder);
		short order = hOrder.asShort();

		// Now we can set all the outputs
		MArrayDataHandle outHandle = data.outputArrayValue(aOutputs);

		MArrayDataBuilder builder = outHandle.builder();
		for (size_t pIdx = 0; pIdx < params.size(); ++pIdx) {

			MDataHandle outH = builder.addElement(pIdx);
			MDataHandle tranH = outH.child(aTranslate);
			MDataHandle rotH = outH.child(aRotate);
			MDataHandle sclH = outH.child(aScale);
			
			MMatrix invPar = invParMats[pIdx];
			MPoint tran = otrans[pIdx] * invPar;
			MMatrix qmat = oquats[pIdx].asMatrix() * invPar;
			MPoint &scale = oscales[pIdx];

			// Ugh, the only way to convert to quat to euler *with a given rot order*
			// is expanding to matrix and decomposing
			MEulerRotation meu = MEulerRotation::decompose(qmat, (MEulerRotation::RotationOrder)order);
						
			tranH.set3Double(tran[0], tran[1], tran[2]);
			rotH.set3Double(meu.x, meu.y, meu.z);
			sclH.set3Double(scale[0], scale[1], scale[2]);
		}
		outHandle.set(builder);
		outHandle.setAllClean();

	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

