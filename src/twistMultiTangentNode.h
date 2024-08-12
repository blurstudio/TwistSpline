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

#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MTypeId.h> 
 
class TwistMultiTangentNode : public MPxNode {
public:
	TwistMultiTangentNode();
	virtual	~TwistMultiTangentNode(); 

	virtual	MStatus	compute( const MPlug& plug, MDataBlock& data );
	static	void*	creator();
	static	MStatus	initialize();

public:
	// inputs
    static MObject aVertMat;
    static MObject aInTanMat;
    static MObject aOutTanMat;
    static MObject aInTanWeight;
    static MObject aOutTanWeight;
    static MObject aInTanAuto;
    static MObject aOutTanAuto;
    static MObject aInSmooth;
    static MObject aOutSmooth;
    static MObject aInParentInv;
    static MObject aOutParentInv;
    static MObject aTwistParentInv;
    static MObject aVertData;

    static MObject aStartTension;
    static MObject aEndTension;
    static MObject aMaxVertices;
    static MObject aClosed;

	// outputs
    static MObject aInTanX;
    static MObject aInTanY;
    static MObject aInTanZ;
    static MObject aInTan;
    static MObject aOutTanX;
    static MObject aOutTanY;
    static MObject aOutTanZ;
    static MObject aOutTan;
    static MObject aInTanLen;
    static MObject aOutTanLen;
    static MObject aOutTwistUpX;
    static MObject aOutTwistUpY;
    static MObject aOutTwistUpZ;
    static MObject aOutTwistUp;
    static MObject aOutTwistMat;
    static MObject aTangents;

	static MTypeId	id;
};


