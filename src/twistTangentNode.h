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

#include <maya/MFnDependencyNode.h>
#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MTypeId.h>

class TwistTangentNode : public MPxNode {
public:
	TwistTangentNode();
	virtual	~TwistTangentNode();

	virtual	MStatus	compute( const MPlug& plug, MDataBlock& data );
	static	void*	creator();
	static	MStatus	initialize();
	void postConstructor() override;

public:
	// outputs
	static MObject aOutLinearTarget;
	static MObject aOutLinearTargetX;
	static MObject aOutLinearTargetY;
	static MObject aOutLinearTargetZ;

	static MObject aOutTwistUp;
	static MObject aOutTwistUpX;
	static MObject aOutTwistUpY;
	static MObject aOutTwistUpZ;

	static MObject aOutTwistMat;

	static MObject aSmoothTan;
	static MObject aSmoothTanX;
	static MObject aSmoothTanY;
	static MObject aSmoothTanZ;

	static MObject aOut;
	static MObject aOutX;
	static MObject aOutY;
	static MObject aOutZ;

	// inputs
	static MObject aParentInverseMatrix;
	static MObject aInTangent;
	static MObject aPrevVertex;
	static MObject aCurrentVertex;
	static MObject aNextVertex;
	static MObject aNextLinearTarget;
	static MObject aNextLinearTargetX;
	static MObject aNextLinearTargetY;
	static MObject aNextLinearTargetZ;
	static MObject aAuto;
	static MObject aSmooth;
	static MObject aWeight;
	static MObject aBackpoint;
	static MObject aEndpoint;

	static MTypeId	id;
};


