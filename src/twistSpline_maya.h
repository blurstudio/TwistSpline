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

#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <cmath>

template <>
inline void resize<MPointArray>(MPointArray &a, unsigned size){
	a.setLength(size);
}

template <>
inline unsigned size<MPointArray>(const MPointArray &a) {
	return a.length();
}

template <>
inline void resize<MVectorArray>(MVectorArray &a, unsigned size) {
	a.setLength(size);
}

template <>
inline unsigned size<MVectorArray>(const MVectorArray &a) {
	return a.length();
}

template <typename Float=double>
inline Float dot(const MVector &a, const MVector &b) {
	return a * b;
}

template <typename Float=double>
inline Float length(const MVector &a) {
	return sqrt(dot(a, a));
}

template <>
inline MVector normalized(const MVector &a) {
	return a / length(a);
}

template <>
inline MVector cross(const MVector &a, const MVector &b) {
	return a ^ b;
}

/// Like projection except you get the perpendicular component of the vector
template <>
inline MVector reject(const MVector &onto, const MVector &n){
	return (n - (((n * onto) / (onto * onto)) * onto)).normal();
}

template <>
inline MVector rotateBy(const MVector &vec, const MQuaternion &quat){
	return vec.rotateBy(quat);
}

