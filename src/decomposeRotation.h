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
// Visual studio crashes when trying to display information about this stuff
// So I'm putting it in a separate file

#define ROTATE_ORDER_XYZ        0
#define ROTATE_ORDER_YZX        1
#define ROTATE_ORDER_ZXY        2
#define ROTATE_ORDER_XZY        3
#define ROTATE_ORDER_YXZ        4
#define ROTATE_ORDER_ZYX        5

inline void decomposeRotation(short order, double rot[3], const MTransformationMatrix &tmat){
	// Manual xyz order decomposition
	// Visual studio is freaked out by the MTransformationMatrix stuff and
	// locks up incessantly, so I broke this file out for stability's sake
	MTransformationMatrix::RotationOrder rotOrder;
	switch ( order ) {
		case ROTATE_ORDER_XYZ: rotOrder = MTransformationMatrix::kXYZ; break;
		case ROTATE_ORDER_YZX: rotOrder = MTransformationMatrix::kYZX; break;
		case ROTATE_ORDER_ZXY: rotOrder = MTransformationMatrix::kZXY; break;
		case ROTATE_ORDER_XZY: rotOrder = MTransformationMatrix::kXZY; break;
		case ROTATE_ORDER_YXZ: rotOrder = MTransformationMatrix::kYXZ; break;
		case ROTATE_ORDER_ZYX: rotOrder = MTransformationMatrix::kZYX; break;
		default:               rotOrder = MTransformationMatrix::kInvalid; break;
	}
	tmat.getRotation(rot, rotOrder);
}



