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

#include <cmath>
#include <utility>
#include <vector>

#include <maya/MArrayDataBuilder.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MTypes.h>
#include <maya/MVector.h>

#include "twistMultiTangentNode.h"

#define CHECKSTAT(m)                                                                               \
    if (!status) {                                                                                 \
        status.perror(m);                                                                          \
        return status;                                                                             \
    };

MTypeId TwistMultiTangentNode::id(0x00122715);

// inputs
MObject TwistMultiTangentNode::aVertMat;
MObject TwistMultiTangentNode::aInTanMat;
MObject TwistMultiTangentNode::aOutTanMat;
MObject TwistMultiTangentNode::aInTanWeight;
MObject TwistMultiTangentNode::aOutTanWeight;
MObject TwistMultiTangentNode::aInTanAuto;
MObject TwistMultiTangentNode::aOutTanAuto;
MObject TwistMultiTangentNode::aInSmooth;
MObject TwistMultiTangentNode::aOutSmooth;
MObject TwistMultiTangentNode::aInParentInv;
MObject TwistMultiTangentNode::aOutParentInv;
MObject TwistMultiTangentNode::aTwistParentInv;
MObject TwistMultiTangentNode::aVertData;

MObject TwistMultiTangentNode::aStartTension;
MObject TwistMultiTangentNode::aEndTension;
MObject TwistMultiTangentNode::aMaxVertices;
MObject TwistMultiTangentNode::aClosed;

// outputs
MObject TwistMultiTangentNode::aInTanX;
MObject TwistMultiTangentNode::aInTanY;
MObject TwistMultiTangentNode::aInTanZ;
MObject TwistMultiTangentNode::aInTan;
MObject TwistMultiTangentNode::aOutTanX;
MObject TwistMultiTangentNode::aOutTanY;
MObject TwistMultiTangentNode::aOutTanZ;
MObject TwistMultiTangentNode::aOutTan;
MObject TwistMultiTangentNode::aInTanLen;
MObject TwistMultiTangentNode::aOutTanLen;
MObject TwistMultiTangentNode::aOutTwistUpX;
MObject TwistMultiTangentNode::aOutTwistUpY;
MObject TwistMultiTangentNode::aOutTwistUpZ;
MObject TwistMultiTangentNode::aOutTwistUp;
MObject TwistMultiTangentNode::aOutTwistMat;
MObject TwistMultiTangentNode::aTangents;

TwistMultiTangentNode::TwistMultiTangentNode() {}
TwistMultiTangentNode::~TwistMultiTangentNode() {}

void *TwistMultiTangentNode::creator() { return new TwistMultiTangentNode(); }

MStatus TwistMultiTangentNode::initialize()
{
    MFnNumericAttribute num_attr;
    MFnCompoundAttribute cmp_attr;
    MFnMatrixAttribute mat_attr;
    MFnUnitAttribute unit_attr;
    MStatus status;

    aInTanMat = mat_attr.create("inTanMat", "itm");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aOutTanMat = mat_attr.create("outTanMat", "otm");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aInTanWeight =
        num_attr.create("inTanWeight", "itw", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(2.0);

    aOutTanWeight =
        num_attr.create("outTanWeight", "otw", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(2.0);

    aInSmooth = num_attr.create("inSmooth", "ism", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(1.0);

    aOutSmooth = num_attr.create("outSmooth", "osm", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(1.0);

    aInTanAuto = num_attr.create("inAuto", "iat", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(1.0);

    aOutTanAuto = num_attr.create("outAuto", "oat", MFnNumericData::kDouble, 1.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(false);
    num_attr.setSoftMin(0.0);
    num_attr.setSoftMax(1.0);

    aVertMat = mat_attr.create("vertMat", "vm");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aInParentInv = mat_attr.create("inParentInverseMatrix", "ipim");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aOutParentInv = mat_attr.create("outParentInverseMatrix", "opim");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aTwistParentInv = mat_attr.create("twistParentInverseMatrix", "tpim");
    mat_attr.setKeyable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aVertData = cmp_attr.create("vertData", "vd", &status);
    cmp_attr.setArray(true);
    cmp_attr.setDisconnectBehavior(cmp_attr.kDelete);
    cmp_attr.addChild(aVertMat);
    cmp_attr.addChild(aInParentInv);
    cmp_attr.addChild(aOutParentInv);
    cmp_attr.addChild(aTwistParentInv);
    cmp_attr.addChild(aInTanWeight);
    cmp_attr.addChild(aOutTanWeight);
    cmp_attr.addChild(aInSmooth);
    cmp_attr.addChild(aOutSmooth);
    cmp_attr.addChild(aInTanAuto);
    cmp_attr.addChild(aOutTanAuto);
    cmp_attr.addChild(aInTanMat);
    cmp_attr.addChild(aOutTanMat);

    addAttribute(aVertData);

    aStartTension =
        num_attr.create("startTension", "st", MFnNumericData::kDouble, 2.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(true);
    addAttribute(aStartTension);

    aEndTension =
        num_attr.create("endTension", "et", MFnNumericData::kDouble, 2.0, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(true);
    addAttribute(aEndTension);

    aMaxVertices = num_attr.create("maxVertices", "mv", MFnNumericData::kInt, 999, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(true);
    num_attr.setMin(2);
    addAttribute(aMaxVertices);

    aClosed = num_attr.create("closed", "cl", MFnNumericData::kBoolean, false, &status);
    num_attr.setKeyable(true);
    num_attr.setStorable(true);
    num_attr.setMin(2);
    addAttribute(aClosed);

    aInTanX =
        unit_attr.create("inVertTanX", "ivx", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aInTanY =
        unit_attr.create("inVertTanY", "ivy", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aInTanZ =
        unit_attr.create("inVertTanZ", "ivz", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);

    aInTan = num_attr.create("inVertTan", "ivt", aInTanX, aInTanY, aInTanZ, &status);
    num_attr.setKeyable(false);
    num_attr.setStorable(false);
    num_attr.setWritable(false);

    aOutTanX =
        unit_attr.create("outVertTanX", "ovx", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aOutTanY =
        unit_attr.create("outVertTanY", "ovy", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aOutTanZ =
        unit_attr.create("outVertTanZ", "ovz", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);

    aOutTan = num_attr.create("outVertTan", "ovt", aOutTanX, aOutTanY, aOutTanZ, &status);
    num_attr.setKeyable(false);
    num_attr.setStorable(false);
    num_attr.setWritable(false);

    aOutTwistUpX =
        unit_attr.create("twistUpX", "tux", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aOutTwistUpY =
        unit_attr.create("twistUpY", "tuy", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);
    aOutTwistUpZ =
        unit_attr.create("twistUpZ", "tuz", MFnUnitAttribute::kDistance, 0.0, &status);
    unit_attr.setKeyable(false);
    unit_attr.setStorable(false);
    unit_attr.setWritable(false);

    aOutTwistUp = num_attr.create(
        "twistUp", "tu", aOutTwistUpX, aOutTwistUpY, aOutTwistUpZ, &status
    );
    num_attr.setKeyable(false);
    num_attr.setStorable(false);
    num_attr.setWritable(false);

    aOutTwistMat = mat_attr.create("twistMat", "tm");
    mat_attr.setKeyable(false);
    mat_attr.setWritable(false);
    mat_attr.setStorable(false);
    mat_attr.setHidden(true);

    aInTanLen = num_attr.create("inTanLen", "itl", MFnNumericData::kDouble, 0.0, &status);
    num_attr.setKeyable(false);
    num_attr.setWritable(false);
    num_attr.setStorable(false);
    aOutTanLen = num_attr.create("outTanLen", "otl", MFnNumericData::kDouble, 0.0, &status);
    num_attr.setKeyable(false);
    num_attr.setWritable(false);
    num_attr.setStorable(false);

    aTangents = cmp_attr.create("vertTans", "vt", &status);
    cmp_attr.addChild(aInTan);
    cmp_attr.addChild(aOutTan);
    cmp_attr.addChild(aInTanLen);
    cmp_attr.addChild(aOutTanLen);
    cmp_attr.addChild(aOutTwistUp);
    cmp_attr.addChild(aOutTwistMat);

    cmp_attr.setArray(true);
    cmp_attr.setUsesArrayDataBuilder(true);
    cmp_attr.setKeyable(false);
    cmp_attr.setWritable(false);
    cmp_attr.setStorable(false);
    addAttribute(aTangents);

    std::vector<MObject *> ins, outs;
    ins.push_back(&aStartTension);
    ins.push_back(&aEndTension);
    ins.push_back(&aMaxVertices);
    ins.push_back(&aClosed);
    ins.push_back(&aVertMat);
    ins.push_back(&aInParentInv);
    ins.push_back(&aOutParentInv);
    ins.push_back(&aTwistParentInv);
    ins.push_back(&aInTanWeight);
    ins.push_back(&aOutTanWeight);
    ins.push_back(&aInTanMat);
    ins.push_back(&aOutTanMat);
    ins.push_back(&aInSmooth);
    ins.push_back(&aOutSmooth);
    ins.push_back(&aInTanAuto);
    ins.push_back(&aOutTanAuto);
    ins.push_back(&aVertData);

    outs.push_back(&aInTanX);
    outs.push_back(&aInTanY);
    outs.push_back(&aInTanZ);
    outs.push_back(&aInTan);
    outs.push_back(&aOutTanX);
    outs.push_back(&aOutTanY);
    outs.push_back(&aOutTanZ);
    outs.push_back(&aOutTan);
    outs.push_back(&aInTanLen);
    outs.push_back(&aOutTanLen);
    outs.push_back(&aOutTwistUpX);
    outs.push_back(&aOutTwistUpY);
    outs.push_back(&aOutTwistUpZ);
    outs.push_back(&aOutTwistUp);
    outs.push_back(&aOutTwistMat);
    outs.push_back(&aTangents);

    for (auto i : ins) {
        for (auto o : outs) {
            attributeAffects(*i, *o);
        }
    }

    return MS::kSuccess;
}

struct DirTanData {
    double weight;
    double smooth;
    double autoVal;
    MMatrix piMat;
    MMatrix userMat;
    MVector leg;
    double legLen;
    MVector normLeg;
    MVector smoothTan;
    MVector linearTan;
    MVector doneTan;
};

struct TanData {
    MMatrix tfmMat;
    MMatrix smoothMat;
    MVector tfm;
    MVector norm;
    MMatrix piTwist;
    DirTanData inTan;
    DirTanData outTan;
};

MMatrix buildMat(const MVector &tfm, const MVector &inrm, const MVector &tan)
{
    //"""Build a matrix from a position, tangent and normal Go through Graham-Schmidt to
    //orthonormalize it """
    MVector bin = (tan ^ inrm).normal();
    MVector nrm = (bin ^ tan).normal();

    MMatrix ret;
    ret[0][0] = tan[0];
    ret[0][1] = tan[1];
    ret[0][2] = tan[2];

    ret[1][0] = nrm[0];
    ret[1][1] = nrm[1];
    ret[1][2] = nrm[2];

    ret[2][0] = bin[0];
    ret[2][1] = bin[1];
    ret[2][2] = bin[2];

    ret[3][0] = tfm[0];
    ret[3][1] = tfm[1];
    ret[3][2] = tfm[2];
    return ret;
}

MVector
buildVertMatTangent(double nextLegLen, double preLegLen, MVector &nextLegNrm, MVector &preLegNrm)
{
    //"""Given all this pre-calculated data, figure out the normalized tangent to the curve at the
    //given vertex"""
    if (preLegLen == 0.0) {
        if (nextLegLen == 0.0) {
            return MVector(0, 1, 0);
        }
        return nextLegNrm;
    }

    if (nextLegLen == 0.0) {
        return -preLegNrm;
    }

    double dot = preLegNrm * nextLegNrm;

    // Pre and post legs both point TOWARD the curTFM
    if (dot > 0.999999999) {
        // pre/next legs are pointing the same direction
        // so we've got a 180 turnaround
        MVector y;
        if (std::abs(preLegNrm * y) > 0.999999999) {
            y = MVector(1, 0, 0);
        }
        else {
            y = MVector(0, 1, 0);
        }
        return (preLegNrm ^ y).normal();
    }
    else if (dot < -0.999999999) {
        // pre/next legs are pointing opposite directions
        // so we're in a straight line
        return -preLegNrm;
    }
    MVector bin = (preLegNrm ^ nextLegNrm).normal();
    return ((bin ^ preLegNrm) + (bin ^ nextLegNrm)).normal();
}

void buildSmoothMats(std::vector<TanData> &dat, double startTension, double endTension, bool closed)
{
    for (size_t i = 1; i + 1 < dat.size(); ++i) {
        auto &cur = dat[i];
        MVector tan = buildVertMatTangent(
            cur.outTan.legLen, cur.inTan.legLen, cur.outTan.normLeg, cur.inTan.normLeg
        );
        cur.smoothMat = buildMat(cur.tfm, cur.norm, tan);

        double preLegLen = cur.inTan.legLen * cur.inTan.weight / 3;
        double nextLegLen = cur.outTan.legLen * cur.outTan.weight / 3;
        cur.inTan.smoothTan = -tan * preLegLen;
        cur.outTan.smoothTan = tan * nextLegLen;
    }

    if (closed){
        auto &start = dat[0];
        auto &end = dat[dat.size() - 1];

        MVector tan = buildVertMatTangent(
            start.outTan.legLen, end.inTan.legLen, start.outTan.normLeg, end.inTan.normLeg
        );
        start.smoothMat = buildMat(start.tfm, start.norm, tan);
        end.smoothMat = start.smoothMat;
        double preLegLen = end.inTan.legLen * end.inTan.weight / 3;
        double nextLegLen = start.outTan.legLen * start.outTan.weight / 3;

        end.inTan.smoothTan = -tan * preLegLen;
        start.outTan.smoothTan = tan * nextLegLen;
    }
    else{
        dat[0].outTan.smoothTan = (dat[1].tfm + dat[1].inTan.smoothTan * startTension - dat[0].tfm) *
                                (dat[0].outTan.weight / 2);
        dat[0].smoothMat = buildMat(dat[0].tfm, dat[0].norm, dat[0].outTan.smoothTan.normal());

        size_t s = dat.size();
        dat[s - 1].inTan.smoothTan =
            (dat[s - 2].tfm + dat[s - 2].outTan.smoothTan * startTension - dat[s - 1].tfm) *
            (dat[s - 1].inTan.weight / 2);

        dat[s - 1].smoothMat = buildMat(dat[s - 1].tfm, dat[s - 1].norm, -dat[s - 1].inTan.smoothTan.normal());
    }
}

void buildLinearTangents(std::vector<TanData> &dat)
{
    for (size_t i = 1; i + 1 < dat.size(); ++i) {
        auto &prev = dat[i - 1];
        auto &cur = dat[i];
        auto &next = dat[i + 1];

        double preLegLen = cur.inTan.legLen * cur.inTan.weight / 3;
        double nextLegLen = cur.outTan.legLen * cur.outTan.weight / 3;

        MVector inDir = (prev.tfm + (prev.outTan.smoothTan * prev.outTan.smooth) - cur.tfm).normal();
        MVector outDir = (next.tfm + (next.inTan.smoothTan * next.inTan.smooth) - cur.tfm).normal();

        cur.inTan.linearTan = inDir * preLegLen;
        cur.outTan.linearTan = outDir * nextLegLen;
    }

    size_t s = dat.size();

    dat[0].outTan.linearTan = (dat[1].tfm + (dat[1].inTan.smoothTan * dat[1].inTan.smooth) - dat[0].tfm) / (3 - dat[1].inTan.smooth);
    dat[s - 1].inTan.linearTan = (dat[s - 2].tfm + (dat[s - 2].outTan.smoothTan * dat[s - 2].outTan.smooth) - dat[s - 1].tfm) / (3 - dat[s-2].outTan.smooth);
}

void buildDoneTangents(std::vector<TanData> &dat)
{
    for (auto &cur : dat) {
        MMatrix ti = cur.tfmMat.inverse();

        double inSmo = cur.inTan.smooth;
        double inAutoMul = cur.inTan.autoVal;
        MMatrix inUserMat = ti * cur.inTan.userMat;
        MVector inUserTan = MVector(inUserMat(3, 0), inUserMat(3, 1), inUserMat(3, 2));
        MVector inAutoTan = cur.inTan.smoothTan * inSmo + cur.inTan.linearTan * (1 - inSmo);
        cur.inTan.doneTan = inAutoTan * inAutoMul + inUserTan * (1 - inAutoMul);

        double outSmo = cur.outTan.smooth;
        double outAutoMul = cur.outTan.autoVal;
        MMatrix outUserMat = ti * cur.outTan.userMat;
        MVector outUserTan = MVector(outUserMat(3, 0), outUserMat(3, 1), outUserMat(3, 2));
        MVector outAutoTan = cur.outTan.smoothTan * outSmo + cur.outTan.linearTan * (1 - outSmo);
        cur.outTan.doneTan = outAutoTan * outAutoMul + outUserTan * (1 - outAutoMul);
    }
}

MStatus TwistMultiTangentNode::compute(const MPlug &plug, MDataBlock &data)
{
    if (plug != aInTanX && plug != aInTanY && plug != aInTanZ && plug != aInTan &&
        plug != aOutTanX && plug != aOutTanY && plug != aOutTanZ && plug != aOutTan &&
        plug != aInTanLen && plug != aOutTanLen && plug != aOutTwistUpX && plug != aOutTwistUpY &&
        plug != aOutTwistUpZ && plug != aOutTwistUp && plug != aOutTwistMat && plug != aTangents) {
        return MS::kUnknownParameter;
    }

    // Read everything
    MArrayDataHandle vertInputs = data.inputArrayValue(aVertData);
    unsigned int icount = vertInputs.elementCount();

    double startTension = data.inputValue(aStartTension).asDouble();
    double endTension = data.inputValue(aEndTension).asDouble();
    int maxVertices = data.inputValue(aMaxVertices).asInt();
    bool closed = data.inputValue(aClosed).asBool();

    icount = (icount < maxVertices) ? icount : (unsigned int)maxVertices;
    if (icount < 2) {
        data.setClean(aInTan);
        data.setClean(aInTanLen);
        data.setClean(aInTanX);
        data.setClean(aInTanY);
        data.setClean(aInTanZ);
        data.setClean(aOutTan);
        data.setClean(aOutTanLen);
        data.setClean(aOutTanX);
        data.setClean(aOutTanY);
        data.setClean(aOutTanZ);
        data.setClean(aOutTwistMat);
        data.setClean(aOutTwistUp);
        data.setClean(aOutTwistUpX);
        data.setClean(aOutTwistUpY);
        data.setClean(aOutTwistUpZ);
        data.setClean(aTangents);

        return MS::kSuccess;
    }

    // Don't care which plug I'm being asked for. Just compute all of 'em
    std::vector<TanData> tanData;

    for (size_t i = 0; i < icount; ++i) {
        TanData dat;

        vertInputs.jumpToArrayElement(i);
        MDataHandle inHandle = vertInputs.inputValue();

        dat.inTan.weight = inHandle.child(aInTanWeight).asDouble();
        dat.outTan.weight = inHandle.child(aOutTanWeight).asDouble();

        dat.inTan.smooth = inHandle.child(aInSmooth).asDouble();
        dat.outTan.smooth = inHandle.child(aOutSmooth).asDouble();

        dat.inTan.autoVal = inHandle.child(aInTanAuto).asDouble();
        dat.outTan.autoVal = inHandle.child(aOutTanAuto).asDouble();

        MMatrix vmat = inHandle.child(aVertMat).asMatrix();
        dat.tfmMat = vmat;
        dat.tfm = MVector(vmat(3, 0), vmat(3, 1), vmat(3, 2));
        dat.norm = MVector(vmat(1, 0), vmat(1, 1), vmat(1, 2));

        dat.inTan.piMat = inHandle.child(aInParentInv).asMatrix();
        dat.outTan.piMat = inHandle.child(aOutParentInv).asMatrix();
        dat.piTwist = inHandle.child(aTwistParentInv).asMatrix();

        dat.inTan.userMat = inHandle.child(aInTanMat).asMatrix();
        dat.outTan.userMat = inHandle.child(aOutTanMat).asMatrix();
        tanData.push_back(std::move(dat));
    }

    if (closed){
        auto &start = tanData[0];
        auto &end = tanData[tanData.size() - 1];
        start.inTan = end.inTan;
        end.outTan = start.outTan;
    }

    for (size_t i = 1; i < tanData.size(); ++i) {
        auto &prev = tanData[i - 1];
        auto &cur = tanData[i];

        cur.inTan.leg = prev.tfm - cur.tfm;
        cur.inTan.legLen = cur.inTan.leg.length();
        cur.inTan.normLeg = cur.inTan.leg / cur.inTan.legLen;

        prev.outTan.leg = -cur.inTan.leg;
        prev.outTan.legLen = cur.inTan.legLen;
        prev.outTan.normLeg = -cur.inTan.normLeg;
    }

    tanData[0].inTan.legLen = 0.0;
    tanData[tanData.size() - 1].outTan.legLen = 0.0;

    buildSmoothMats(tanData, startTension, endTension, closed);
    buildLinearTangents(tanData);
    buildDoneTangents(tanData);

    auto outputs = data.outputArrayValue(aTangents);
    auto builder = outputs.builder();
    for (size_t i = 0; i < tanData.size(); ++i) {
        auto outHandle = builder.addElement(i);
        auto &cur = tanData[i];

        auto inTanHandle = outHandle.child(aInTan);
        auto inPt = MPoint(cur.inTan.doneTan + cur.tfm) * cur.inTan.piMat;
        inTanHandle.set3Double(inPt[0], inPt[1], inPt[2]);

        auto inTanLenHandle = outHandle.child(aInTanLen);
        inTanLenHandle.setDouble(cur.inTan.doneTan.length());

        auto outTanHandle = outHandle.child(aOutTan);
        auto outPt = MPoint(cur.outTan.doneTan + cur.tfm) * cur.outTan.piMat;
        outTanHandle.set3Double(outPt[0], outPt[1], outPt[2]);

        auto outTanLenHandle = outHandle.child(aOutTanLen);
        outTanLenHandle.setDouble(cur.outTan.doneTan.length());

        auto outMatHandle = outHandle.child(aOutTwistMat);
        outMatHandle.setMMatrix(cur.smoothMat * cur.piTwist);

        auto outTwistHandle = outHandle.child(aOutTwistUp);
        outTwistHandle.set3Double(cur.tfmMat(1, 0), cur.tfmMat(1, 1), cur.tfmMat(1, 2));
    }

    data.setClean(aInTan);
    data.setClean(aInTanLen);
    data.setClean(aInTanX);
    data.setClean(aInTanY);
    data.setClean(aInTanZ);
    data.setClean(aOutTan);
    data.setClean(aOutTanLen);
    data.setClean(aOutTanX);
    data.setClean(aOutTanY);
    data.setClean(aOutTanZ);
    data.setClean(aOutTwistMat);
    data.setClean(aOutTwistUp);
    data.setClean(aOutTwistUpX);
    data.setClean(aOutTwistUpY);
    data.setClean(aOutTwistUpZ);
    data.setClean(aTangents);

    return MS::kSuccess;
}
