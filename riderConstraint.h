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
	static MObject     riderConstraint::aRotateOrder;
	static MObject     riderConstraint::aGlobalOffset;
	static MObject     riderConstraint::aGlobalSpread;
	static MObject     riderConstraint::aUseCycle;
	static MObject     riderConstraint::aNormalize;
	static MObject     riderConstraint::aNormValue;
	// inputs
	static MObject     riderConstraint::aInputSplines;
		static MObject     riderConstraint::aSpline;
		static MObject     riderConstraint::aWeight;

	static MObject     riderConstraint::aParams;
		static MObject     riderConstraint::aParam;
		static MObject     riderConstraint::aParentInverseMatrix;

	// output
	static MObject     riderConstraint::aOutputs;
		static MObject     riderConstraint::aOutMat;
		static MObject     riderConstraint::aTranslate;
		static MObject     riderConstraint::aTranslateX;
		static MObject     riderConstraint::aTranslateY;
		static MObject     riderConstraint::aTranslateZ;
		static MObject     riderConstraint::aRotate;
		static MObject     riderConstraint::aRotateX;
		static MObject     riderConstraint::aRotateY;
		static MObject     riderConstraint::aRotateZ;
		static MObject     riderConstraint::aScale;
		static MObject     riderConstraint::aScaleX;
		static MObject     riderConstraint::aScaleY;
		static MObject     riderConstraint::aScaleZ;

	static MTypeId id;
};

