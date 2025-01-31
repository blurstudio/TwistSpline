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


// To make my choice of abstraction work, I have to write wrappers for the template abstractions
template <typename PointArray, typename IndexType = unsigned>
inline void resize(PointArray &a, IndexType size) {
	a.resize(size);
}

template <typename PointArray, typename IndexType = unsigned>
inline IndexType size(const PointArray &a) {
	return static_cast<IndexType>(a.size());
}

template <typename Vector, typename Float = double>
inline Float dot(const Vector &a, const Vector &b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <typename Vector, typename Float = double>
inline Float length(const Vector &a) {
	return sqrt(dot<Vector, Float>(a, a));
}

template <typename Vector, typename Float = double>
inline Vector normalized(const Vector &a) {
	return a / length<Vector, Float>(a);
}

template <typename Vector>
inline Vector cross(const Vector &a, const Vector &b) {
	return Vector((a[1] * b[2] - a[2] * b[1]), (a[0] * b[2] - a[2] * b[0]), (a[0] * b[1] - a[1] * b[0]));
}

/// Like projection except you get the perpendicular component of the vector
template <typename Vector, typename Float=double>
inline Vector reject(const Vector &onto, const Vector &n) {
	return normalized<Vector, Float>(n - ((dot<Vector, Float>(n, onto) / dot<Vector, Float>(onto, onto)) * onto));
}

template <typename Quat>
inline Quat qmult(const Quat &q, const Quat &r){
	return Quat(r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
		r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
		r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
		r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]);
}

template <typename Vector, typename Quat>
inline Vector rotateBy(const Vector &vec, const Quat &quat){
	Quat r(0.0, vec[0], vec[1], vec[2]);
	Quat c(quat[0], -quat[1], -quat[2], -quat[3]);
	Quat ret = qmult(qmult(quat, r), c);
	return Vector(ret[1], ret[2], ret[3]);
}


