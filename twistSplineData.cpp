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

#include "twistSplineData.h"
#include "twistSpline.h"

const MTypeId TwistSplineData::id(0x001226FB);
const MString TwistSplineData::typeName("TwistSplineData");

void* TwistSplineData::creator(){
    return new TwistSplineData;
}

TwistSplineData::TwistSplineData() {
	_twistSpline = new TwistSplineT();
}

TwistSplineData::~TwistSplineData() {
}

void TwistSplineData::copy(const MPxData& other) {
	if (other.typeId() == TwistSplineData::id) {
		const TwistSplineData* otherData = (const TwistSplineData*) & other;
		const TwistSplineT * x = otherData->getSpline();
		_twistSpline = new TwistSplineT(*x);
		//_twistSpline->setVerts(x->getVerts(), x->getQuats(), x->getLockPositions(), x->getLockValues(), x->getUserTwists(), x->getTwistLocks(), x->getOrientLocks());
	}
	else {
		//  we need to convert to the other type based on its iff Tag
		cerr << "wrong data format!" << endl;
	}
	return;
}

const TwistSplineT* TwistSplineData::getSpline() const {
    return _twistSpline;
}

TwistSplineT* TwistSplineData::getSpline() {
    return _twistSpline;
}

MTypeId TwistSplineData::typeId() const {
    return TwistSplineData::id;
}

MString TwistSplineData::name() const { 
    return TwistSplineData::typeName; 
}

MStatus TwistSplineData::readASCII(const MArgList& args, unsigned& lastParsedElement) {
	return MS::kSuccess;
}

MStatus TwistSplineData::readBinary(istream& in, unsigned length) {
	return MS::kSuccess;
}

MStatus TwistSplineData::writeASCII(ostream& out) {
	return MS::kSuccess;
}

MStatus TwistSplineData::writeBinary(ostream& out) {
	return MS::kSuccess;
}

