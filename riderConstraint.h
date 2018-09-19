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
#include <maya/MPxNode.h>

class riderConstraint : public MPxNode {
public:
	riderConstraint() {}
	static void* creator() {
		return new riderConstraint();
	}
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	static  MStatus initialize();

public:
	// inputs
	static MObject aRotateOrder;
	static MObject aGlobalOffset;
	static MObject aGlobalSpread;
	static MObject aUseCycle;
	static MObject aNormalize;
	static MObject aNormValue;
	// inputs
	static MObject aInputSplines;
		static MObject aSpline;
		static MObject aWeight;

	static MObject aParams;
		static MObject aParam;
		static MObject aParentInverseMatrix;

	// output
	static MObject aOutputs;
		static MObject aOutMat;
		static MObject aTranslate;
		static MObject aTranslateX;
		static MObject aTranslateY;
		static MObject aTranslateZ;
		static MObject aRotate;
		static MObject aRotateX;
		static MObject aRotateY;
		static MObject aRotateZ;
		static MObject aScale;
		static MObject aScaleX;
		static MObject aScaleY;
		static MObject aScaleZ;

	static MTypeId id;
};

