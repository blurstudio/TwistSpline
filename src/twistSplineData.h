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

#include <maya/MIOStream.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MQuaternion.h>
#include "twistSpline.h"
#include "twistSpline_maya.h"

typedef TwistSpline<MPointArray, MPoint, MVectorArray, MVector, std::vector<MQuaternion>, MQuaternion, double, unsigned> TwistSplineT;

class TwistSplineData : public MPxData {
public:
    TwistSplineData();
    virtual ~TwistSplineData();

    // Override methods in MPxData.
    virtual MStatus readASCII(const MArgList&, unsigned& lastElement);
    virtual MStatus readBinary(istream& in, unsigned length);
    virtual MStatus writeASCII(ostream& out);
    virtual MStatus writeBinary(ostream& out);
	virtual void copy(const MPxData& other);

    // Data access
	const TwistSplineT* getSpline() const;
    TwistSplineT* getSpline();

    // static methods and data.
    MTypeId typeId() const; 
    MString name() const;
    static const MString typeName;
    static const MTypeId id;
    static void* creator();

private:
    TwistSplineT* _twistSpline;
};

