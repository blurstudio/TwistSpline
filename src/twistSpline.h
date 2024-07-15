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
#include <vector>
#include <array>
#include <memory>
#include <utility>
#include <cmath>
#include <numeric>
#include <algorithm>
#include "twistSplineUtils.h"


/**
  * A helper function for quickly finding a single index and segment percentage for an input tValue
  *
  * t: The input t-value
  * samp: A std::vector of floats of t-values
  *
  * segT: The t-value for the found segment. Can be outside the (0, 1) range if the input
  *       t-value is outside the range of the samp vector.
  * segIdx: The index of the segment
  */
template <typename Float=double>
void linearIndex(Float t, const std::vector<Float> &samp, Float &segT, size_t &segIdx) {
	// Remap a length T-Value to a param TValue
	auto ubIt = std::upper_bound(samp.begin(), samp.end(), t);
	// get the new T value and the segment index
	if (ubIt == samp.begin()) {
		segIdx = 0;
		if (samp.size() > 1) {
			segT = t / samp[1];
		}
		else {
			segT = 0.0;
		}
	}
	else if (ubIt == samp.end()) {
		// subtract 1 because (size-1) is the end index
		// subtract another 1 because we're indexing into
		// the segments between the samples
		segIdx = samp.size() - 2;
		if (samp.size() > 1) {
			segT = (t - samp[segIdx]) / (samp[segIdx + 1] - samp[segIdx]);
		}
		else {
			segT = 1.0;
		}
	}
	else {
		auto lbIt = ubIt - 1;
		segT = (t - *lbIt) / (*ubIt - *lbIt);
		segIdx = lbIt - samp.begin();
		if (segIdx < samp.size() - 2 && segT > 0.99999999999) {
			// Snap values close to the end to the next segment
			// This removes a lot of jitter
			segT = 0.0;
			++segIdx;
		}
	}
}

/**
  * A helper function for quickly finding multiple index and segment percentage for an input tValue
  * using a sweeping point style algorithm
  *
  * params: The input t-values
  * samp: A std::vector of floats of t-values
  *
  * segT: The t-value for each found segment. Can be outside the (0, 1) range if the input
  *       t-value is outside the range of the samp vector.
  * segIdx: The index for each segment
  */
template <typename Float = double>
void multiLinearIndexes(const std::vector<Float> &params, const std::vector<Float> &samp, std::vector<Float> &segTs, std::vector<size_t> &segIdxs) {
	// The params aren't necessarily sorted, so to do the "merge", I have to sort them first.
	// but, so I can 'unsort' them, I'm doing this the annoying way, by index
	std::vector<size_t> pOrder(params.size());
	std::iota(pOrder.begin(), pOrder.end(), 0);
	sort(pOrder.begin(), pOrder.end(), [&params](size_t i1, size_t i2) {return params[i1] < params[i2]; });

	size_t segIdx = 0;
	segTs.resize(params.size());
	segIdxs.resize(params.size());
	bool capped = false;
	for (size_t i = 0; i < params.size(); ++i) {
		Float t = params[pOrder[i]];
		if (!capped && t >= params[segIdx + 1]) {
			if (segIdx + 1 >= params.size()) {
				capped = true;
			}
			++segIdx;
		}
		segIdxs[pOrder[i]] = segIdx;
		segTs[pOrder[i]] = (t - params[segIdx - 1]) / (params[segIdx] - params[segIdx - 1]);
	}
}







/**
  * A Spline segment is a 4 vertex cubic bezier spline. Each segment is independent of the others, and
  * segment interdependency will be handled by a setup or rig.
  *
  * This class doesn't own any Point objects, it just references them.
  * Also, this class should not do any memory management as it will be
  * completely managed by it's parent TwistSpline instance
  */
template <typename PointArray, typename Point, typename VectorArray, typename Vector, typename QuatArray, typename Quat, typename Float = double>
class TwistSplineSegment {
private:
	std::array<Point*, 4> verts; // Convenience pointers to the transforms
	std::array<Point*, 4> sclVerts; // Convenience pointers to the scales
	std::array<Quat*, 4> quats; // Pointer to the position/orientation of the verts

	std::array<Vector, 3> d1verts; // first derivative position segments
	std::array<Vector, 2> d2verts; // second derivative position segments
	std::array<Vector, 3> s1verts; // first derivative scale segments
	std::array<Vector, 2> s2verts; // second derivative scale segments

	PointArray points; // The point lookup table along the curve
	PointArray scales; // The scale lookup table along the curve
	VectorArray tangents;
	VectorArray rnormals; // Raw normals frame-walked starting from the inorm
	VectorArray rbinormals;
	VectorArray tnormals; // Normals twisted around the spline by the user
	VectorArray tbinormals;
	std::vector<Float> twistVals; // the angle, sin, and cos for each twist along the spline

	VectorArray units; // unit vectors along the curve. Makes closest point lookup faster
	Vector iNorm;
	std::vector<Float> sampleLengths; // The sampleLength param for each point in the points
	size_t lutSteps;

public:
	/**
	  * Constructor from scratch
	  *
	  * p0, p1, p2, p3: Four pointers to Point objects that are the vertices of the segment
	  * q0, q1, q2, q3: Four quaternion pointers that define the orientations of along the segment
	  * s0, s1, s2, s3: Four Point pointers that define the xyz scale values along the segment
	  */
	TwistSplineSegment(
			Point *p0, Point *p1, Point *p2, Point *p3,
			Point *s0, Point *s1, Point *s2, Point *s3,
			Quat *q0, Quat *q1, Quat *q2, Quat *q3,
			const Vector &iNorm, size_t lutSteps) {
		verts[0] = p0; verts[1] = p1; verts[2] = p2; verts[3] = p3;
		sclVerts[0] = s0; sclVerts[1] = s1; sclVerts[2] = s2; sclVerts[3] = s3;
		quats[0] = q0; quats[1] = q1; quats[2] = q2; quats[3] = q3;
		this->iNorm = iNorm;
		this->lutSteps = lutSteps;

		// resize the point-wise arrays
		size_t pointSteps = lutSteps + 1;
		resize(points, pointSteps);
		resize(scales, pointSteps);
		resize(tangents, pointSteps);
		resize(rnormals, pointSteps);
		resize(rbinormals, pointSteps);
		resize(sampleLengths, pointSteps);

		// resize the line-wise arrays
		resize(units, lutSteps);

		buildDverts();
		buildLut();
	}

	/**
	  * Almost-Copy Constructor. The parent TwistSpline will have already copied the input points
	  * so instead of re-copying them, I take them as inputs to this constructor.
	  */
	TwistSplineSegment(TwistSplineSegment const &old, std::array<Point*, 4> &verts, std::array<Point*, 4> &sclVerts, std::array<Quat*, 4> &quats){
		this->d1verts = old.d1verts;
		this->d2verts = old.d2verts;
		this->s1verts = old.s1verts;
		this->s2verts = old.s2verts;
		this->points = old.points;
		this->scales = old.scales;
		this->tangents = old.tangents;
		this->rnormals = old.rnormals;
		this->rbinormals = old.rbinormals;
		this->tnormals = old.tnormals;
		this->tbinormals = old.tbinormals;
		this->twistVals = old.twistVals;
		this->units = old.units;
		this->iNorm = old.iNorm;
		this->sampleLengths = old.sampleLengths;
		this->lutSteps = old.lutSteps;
		this->verts = verts;
		this->quats = quats;
		this->sclVerts = sclVerts;
	}

	~TwistSplineSegment() {}

	/**
	  * Get access to the internal data of the spline segment
	  */
	VectorArray& getUnits() { return units; }
	size_t getLutSteps() { return lutSteps; }
	PointArray& getPoints() { return points; }
	PointArray& getScales() { return scales; }
	VectorArray& getTangents() { return tangents; }
	VectorArray& getRawNormals() { return rnormals; }
	VectorArray& getTwistNormals() { return tnormals; }
	VectorArray& getRawBinormals() { return rbinormals; }
	VectorArray& getTwistBinormals() { return tbinormals; }
	std::vector<Float>& getTwistVals() { return twistVals; }
	std::vector<Float>& getSampleLengths() { return sampleLengths; }

	/// Get the arc-length of the spline
	Float getLength() const {
		size_t ss = sampleLengths.size();
		if (ss) {
			return sampleLengths[ss - 1];
		}
		return 0.0;
	}

	static Point compute(const std::array<Point*, 4> pts, Float t) {
		if (t <= 0.0) return *(pts[0]);
		if (t >= 1.0) return *(pts[3]);
		// When calculating a single t-value, just use the bernstein because the pre-computation is heavier than one step
		Float mt = 1.0 - t;
		Float a = mt * mt * mt;
		Float b = mt * mt * t * 3.0;
		Float c = mt * t * t * 3.0;
		Float d = t * t * t;
		return a * (*pts[0]) + b * (*pts[1]) + c * (*pts[2]) + d * (*pts[3]);
	}

	/// Mathematically compute the Point at a given t-value
	Point computeTran(Float t) const {
		return compute(verts, t);
	}

	/// Mathematically compute the Scale at a given t-value
	Point computeScale(Float t) const {
		return compute(sclVerts, t);
	}

	static Vector computeTangent(const std::array<Vector, 3> &dverts, Float t) {
		if (t <= 0.0) return dverts[0];
		if (t >= 1.0) return dverts[2];
		// When calculating a single t-value, just use the bernstein because the pre-computation is heavier than one step
		Float mt = 1.0 - t;
		Float a = mt * mt;
		Float b = mt * t * 2.0;
		Float c = t * t;

		return a * dverts[0] + b * dverts[1] + c * dverts[2];
	}

	/// Mathematically compute the Tangent at a given t-value
	Vector computeTranTangent(Float t) const {
		return computeTangent(d1verts, t);
	}

	/// Mathematically compute the scale tangent at a given t-value
	Vector computeScaleTangent(Float t) const {
		return computeTangent(s1verts, t);
	}

	/// Reject the first tangent
	Vector rejectInitialNormal(const Vector &n) const {
		return reject<Vector, Float>(computeTranTangent(0.0), n);
	}

	/// Get the initial normal of the spline
	Vector initialNormal() const {
		Vector y = Vector(0.0, 1.0, 0.0);
		Vector n = rotateBy(y, *(quats[0]));
		return rejectInitialNormal(n);
	}

	/// Get the angle between the last normal and the y-axis of the last CV
	Float postAngle() const {
		Vector y = Vector(0.0, 1.0, 0.0);
		Vector n = rotateBy(y, *(quats[3]));
		const Vector &tLast = tangents[lutSteps];
		const Vector &bLast = rbinormals[lutSteps];
		const Vector &fLast = rnormals[lutSteps];
		Vector cvLast = reject<Vector, Float>(tLast, n);

		Float dd = dot<Vector, Float>(cvLast, fLast);
		dd = std::max(std::min(dd, 1.0), -1.0);
		Float angle = acos(dd);
		Float disc = dot<Vector, Float>(bLast, cvLast);
		// One of these may be better? do some testing
		//if (disc >= 0) return -angle;
		if (disc >= 0) return -angle;
		return angle;
	}

	/// Build the xyz first and second derivatives of the spline
	void buildDverts() {
		for (size_t i = 0; i < 3; ++i)
			d1verts[i] = 3 * (*(verts[i + 1]) - (*(verts[i])));

		for (size_t i = 0; i < 2; ++i)
			d2verts[i] = 2 * (d1verts[i + 1] - d1verts[i]);

		// Also for scales
		for (size_t i = 0; i < 3; ++i)
			s1verts[i] = 3 * (*(sclVerts[i + 1]) - (*(sclVerts[i])));

		for (size_t i = 0; i < 2; ++i)
			s2verts[i] = 2 * (s1verts[i + 1] - s1verts[i]);
	}

	/**
	  * Calculate a lookup table for some given points
	  * Fancy math transforms the evaluation of a bernstein polynomial into evaluation of a finite Taylor Series
	  * Which means (in this case) that we can pre-compute a bunch of stuff and make our loop nothing but some additions
	  *
	  * The intuition of this algorithm:
	  * The der3 value is some constant, and to step between der2 values, you add (der3 * stepLength)
	  * So it follows that stepping to der1 values is just some linear function of the der2 values
	  * Same with the output. There's complicated proof, but I think of it like forces pulling each other around
	  */
	static void computeSplinePoints(const std::array<Point*, 4> pts, size_t lutSteps, PointArray &pOut, VectorArray &tOut) {
		// Do the pre-calculations of the derivative vectors
		Vector a = (*pts[3]) - 3 * (*pts[2]) + 3 * (*pts[1]) - (*pts[0]);
		Vector b = 3 * (*pts[2]) - 6 * (*pts[1]) + 3 * (*pts[0]);
		Vector c = 3 * (*pts[1]) - 3 * (*pts[0]);
		Vector d = (*pts[0]);

		// Pre calculate the powers of the step length
		Float h = 1.0 / (Float)lutSteps;
		Float h2 = h * h;
		Float h3 = h2 * h;

		size_t pointSteps = lutSteps + 1;

		// Loop for positions
		Vector fd = a * h3 + b * h2 + c * h;
		Vector fd2 = 6 * a*h3 + 2 * b*h2;
		Vector fd3 = 6 * a*h3;
		pOut[0] = *pts[0];
		for (size_t i = 1; i < pointSteps; i++) {
			d += fd;
			fd += fd2;
			fd2 += fd3;
			pOut[i] = d;
		}

		if (size(tOut)){
			// loop for tangents
			Vector tan = 3*((*pts[1]) - (*pts[0]));
			Vector td = 3 * a*h2 + 2 * b*h;
			Vector td2 = 6 * a*h2;
			tOut[0] = normalized<Vector, Float>(tan);
			for (size_t i = 1; i < pointSteps; i++) {
				tan += td;
				td += td2;
				tOut[i] = normalized<Vector, Float>(tan);
			}
		}
	}

	/**
	  * Then compute the normals, binormals, and arc-length
	  * Algorithm taken from "Computation of Rotation Minimizing Frames"
	  * by W. Wang et. al.
	  * This is called the "Double Refleciton Method". It takes one frame
	  * Reflects it to the next sampled position, then reflects it again
	  * so that the reflected tangent matches the computed one. The resulting
	  * up-vector is a surprisingly good approximation of the true RMF
	  */
	static void doubleReflect(const Vector &iNorm, const PointArray &points, const VectorArray &tangents, size_t lutSteps,
			VectorArray &normals, VectorArray &binormals,
			std::vector<Float> &sampleLengths, VectorArray &units) {

		normals[0] = reject<Vector, Float>(tangents[0], iNorm);
		binormals[0] = cross(tangents[0], normals[0]);
		sampleLengths[0] = 0.0;

		// A chunk of this loop could be parallelized
		for (size_t i = 0; i < lutSteps; i++) {
			Vector v1 = points[i + 1] - points[i];
			Float v12 = dot<Vector, Float>(v1, v1);
			Float c1 = 2.0 / v12;
			Vector tLi = tangents[i] - c1 * dot<Vector, Float>(v1, tangents[i]) * v1;
			Vector v2 = tangents[i + 1] - tLi;
			Float dv2 = dot<Vector, Float>(v2, v2);
			if (abs(dv2) < 1.0e-15) {
				normals[i + 1] = normals[i];
				binormals[i + 1] = binormals[i];
				continue;
			}
			Float c2 = 2.0 / dv2;

			// While I'm in here, I can calculate the sampleLengths and unit parameters
			Float len = sqrt(v12);
			sampleLengths[i + 1] = sampleLengths[i] + len;
			units[i] = v1 / len;

			// non-parallelizable Section
			Vector rLi = normals[i] - c1 * dot<Vector, Float>(v1, normals[i]) * v1;
			normals[i + 1] = rLi - c2 * dot<Vector, Float>(v2, rLi) * v2;
			binormals[i + 1] = cross(tangents[i + 1], normals[i + 1]);
		}
	}

	/// Build the entire base lookup table of points and matrices
	void buildLut() {
		computeSplinePoints(verts, lutSteps, points, tangents);
		VectorArray _unused;
		computeSplinePoints(sclVerts, lutSteps, scales, _unused);
		doubleReflect(iNorm, points, tangents, lutSteps, rnormals, rbinormals, sampleLengths, units);
	}

	/**
	  * Get the matrix at the given parameter based on the given LUT's
	  * rawT: The input t-value
	  * tan: The tangent output
	  * norm: The normal output
	  * binorm: The binormal output
	  * tran: The position output
	  * scl: The scale output
	  * twist: The twist output
	  * normals: The input normals LUT
	  * binormals: The input binormals LUT
	  */
	void matrixAtParam(Float rawT, Vector &tan, Vector &norm, Vector &binorm, Point &tran, Point &scl, Float &twist, const VectorArray &normals, const VectorArray &binormals) const {
		// rawT is 0-1 normalized length
		// lenT is unnormalized length
		// segT is 0-1 param value
		// t is 0-1 param of a lut segment
		Float len = getLength();
		Float t, segT, lenT = rawT * len;
		size_t i;
		linearIndex(lenT, sampleLengths, t, i);
		segT = (t + i) / lutSteps;
		if (abs(t) < 1.0e-11) { // t == 0
			tran = points[i];
			scl = scales[i];
			tan = tangents[i];
			norm = normals[i];
			binorm = binormals[i];
			twist = twistVals[i];
			return;
		}
		if (t < 0.0) {
			Vector dv = (dot(d1verts[0], d1verts[0]) < 0.00000000001) ? d1verts[0] : normalized(d1verts[0]);
			Vector sv = (dot(s1verts[0], s1verts[0]) < 0.00000000001) ? s1verts[0] : normalized(s1verts[0]);
			tran = (dv * lenT) + points[0];
			scl = (sv * lenT) + scales[0];
			tan = tangents[0];
			norm = normals[0];
			binorm = binormals[0];
			twist = twistVals[0];
			return;
		}
		if (t > 0.99999999999) {
			Vector dv = (dot(d1verts[2], d1verts[2]) < 0.00000000001) ? d1verts[2] : normalized(d1verts[2]);
			Vector sv = (dot(s1verts[2], s1verts[2]) < 0.00000000001) ? s1verts[2] : normalized(s1verts[2]);
			tran = (dv * (lenT - len)) + points[size(points) - 1];
			scl = (sv * (lenT - len)) + scales[size(scales) - 1];
			tan = tangents[size(tangents) - 1];
			norm = normals[size(normals) - 1];
			binorm = tbinormals[size(tbinormals) - 1];
			twist = twistVals[twistVals.size() - 1];
			return;
		}

		tran = computeTran(segT);
		scl = computeScale(segT);
		tan = normalized(computeTranTangent(segT));
#if 0
		// This could be done by lerp+normalization
		// Bad (can end up over-extrapolating in extreme cases)
		auto x1 = normals[i];
		auto d = normals[i + 1] - x1;
		Float tt = dot<Vector, Float>(x1, tan);
		Float bb = dot<Vector, Float>(d, tan);
		if (abs(bb) < 1.0e-15) {
			norm = normals[i];
		}
		else {
			norm = normalized(x1 - d * (tt / bb));
		}
		binorm = normalized(cross(tan, norm));
#elif 0
		// Or by the double reflection method
		Vector preNorm, postNorm;
		{
			Vector v1 = tran - points[i];
			Float c1 = 2.0 / dot<Vector, Float>(v1, v1);
			Vector tLi = tangents[i] - c1 * dot<Vector, Float>(v1, tangents[i]) * v1;
			Vector v2 = tangents[i + 1] - tLi;
			Float c2 = 2.0 / dot<Vector, Float>(v2, v2);
			Vector rLi = normals[i] - c1 * dot<Vector, Float>(v1, normals[i]) * v1;
			preNorm = rLi - c2 * dot<Vector, Float>(v2, rLi) * v2;
		}
		{
			size_t e = i + 1;
			Vector v1 = tran - points[e];
			Float c1 = 2.0 / dot<Vector, Float>(v1, v1);
			Vector tLi = tangents[e] - c1 * dot<Vector, Float>(v1, tangents[e]) * v1;
			Vector v2 = tangents[i] - tLi;
			Float c2 = 2.0 / dot<Vector, Float>(v2, v2);
			Vector rLi = normals[e] - c1 * dot<Vector, Float>(v1, normals[e]) * v1;
			postNorm = rLi - c2 * dot<Vector, Float>(v2, rLi) * v2;
		}
		norm = normalized((1-t) * preNorm + t * postNorm);
		binorm = normalized(cross(tan, norm));
# else
		// KEEP IT HERE!  The other options cause jitter
		// Simple interpolation and rebuild the basis
		auto n1 = normals[i];
		auto n2 = normals[i + 1];
		auto nn = n1 * (1-t) + n2 * t;
		binorm = normalized(cross(tan, nn));
		norm = cross(binorm, tan);
#endif

		auto tw1 = twistVals[i];
		auto tw2 = twistVals[i + 1];
		twist = tw1 * (1 - t) + tw2 * t;
	}

	/**
	  * Get the matrix at the given parameter based on the TWISTED normals and binormals
	  * rawT: The input t-value
	  * tan: The tangent output
	  * norm: The normal output
	  * binorm: The binormal output
	  * tran: The position output
	  * scl: The scale output
	  * twist: The twist output
	  */
	void twistedMatrixAtParam(Float rawT, Vector &tan, Vector &norm, Vector &binorm, Point &tran, Point &scl, Float &twist) const {
		matrixAtParam(rawT, tan, norm, binorm, tran, scl, twist, tnormals, tbinormals);
	}

	/**
	  * Get the matrix at the given parameter based on the UN-TWISTED normals and binormals
	  * rawT: The input t-value
	  * tan: The tangent output
	  * norm: The normal output
	  * binorm: The binormal output
	  * tran: The position output
	  * scl: The scale output
	  * twist: The twist output
	  */
	void rawMatrixAtParam(Float rawT, Vector &tan, Vector &norm, Vector &binorm, Point &tran, Point &scl, Float &twist) const {
		matrixAtParam(rawT, tan, norm, binorm, tran, scl, twist, rnormals, rbinormals);
	}

	/**
	  * Apply the twist values around the tangent vectors
	  * linearly interpolating between startAngle and endAngle
	  */
	void applyTwist(Float startAngle, Float endAngle){ // inRadians
        startAngle *= -1;
        endAngle *= -1;

		resize(tnormals, size(rnormals));
		resize(tbinormals, size(rbinormals));
		resize(twistVals, size(rnormals));
		Float len = getLength();

		for (size_t i=0; i <= lutSteps; ++i){
			Float perc = sampleLengths[i] / len; // parameterize by length
            Float angle = ((endAngle - startAngle) * perc) + startAngle;
			twistVals[i] = angle;
			const Vector &x = rnormals[i];
			const Vector &y = rbinormals[i];
			const Vector &n = tangents[i];

			/*
				Everything should already be normalized
				dot(n, x) is 0 b/c the norm and binorm are perpendicular
				and I've already calculated cross(n, x) as the binormal
				so I don't need to do this part:
			//Float ca = cos(angle);
			//Float sa = sin(angle);
			//tnormals[i] = x * ca + n * dot<Vector, Float>(n, x) * (1 - ca) + cross(x, n) * sa;
			*/
			tnormals[i] = x * cos(angle) + y * sin(angle);
			tbinormals[i] = cross(n, tnormals[i]);
		}
	}

};









// For later:
// I'll bet I can check the derivatives of the spline
// to find where it's even possible to be closer, and turn a closest
// point lookup into something closer to a binary search.

// Float default declared in the class prototype
template <typename PointArray, typename Point, typename VectorArray, typename Vector, typename QuatArray, typename Quat, typename Float>
class TwistSpline {
private:
	std::vector<std::unique_ptr<TwistSplineSegment<PointArray, Point, VectorArray, Vector, QuatArray, Quat, Float>>> segments;
	PointArray verts; // all verts, ordered, including tangents
	PointArray scales; // The scale of each vert
	QuatArray quats; // The orientations of the verts

	std::vector<Float> lockPositions; // The locking t-values
	std::vector<Float> lockValues; // The [0-1] lock property for t-values
	std::vector<Float> userTwists; // The user defined twist offsets
	std::vector<Float> twistLocks; // The [0-1] lock property for user twist offsets
	std::vector<Float> orientLocks; // The [0-1] lock property for cv quaternion twist
	std::vector<Float> remap; // The remapped segment endpoint u-values

	size_t projSteps{}; // The number of steps for a closest point lookup
	size_t lutSteps{}; // The number of sub-segments per spline segment
	Float totalLength{};

public:
	TwistSpline() { lutSteps = 20; }
	~TwistSpline() {}

	PointArray getVerts() const { return verts; }
	PointArray getScaleVerts() const {return scales; }
	QuatArray getQuats() const { return quats; }

	std::vector<Float> getLockPositions() const { return lockPositions; }
	std::vector<Float> getLockValues() const { return lockValues; }

	std::vector<Float> getTwistLocks() const { return twistLocks; }
	std::vector<Float> getOrientLocks() const { return orientLocks; }
	std::vector<Float> getUserTwists() const { return userTwists; }
	std::vector<Float> getRemap() const { return remap; }
	Float getTotalLength() const { return totalLength; }



	/// Copy constructor
	TwistSpline(TwistSpline const &old){
		this->verts = old.verts;
		this->quats = old.quats;
		this->scales = old.scales;
		this->lockPositions = old.lockPositions;
		this->lockValues = old.lockValues;
		this->userTwists = old.userTwists;
		this->twistLocks = old.twistLocks;
		this->orientLocks = old.orientLocks;
		this->remap = old.remap;
		this->projSteps = old.projSteps;
		this->lutSteps = old.lutSteps;
		this->totalLength = old.totalLength;
		size_t numVerts = size(verts);
		if (numVerts < 2) {
			segments.clear();
			return;
		}

		size_t numSegs = ((numVerts - 1) / 3);
		segments.resize(numSegs);
		for (size_t i=0; i<numSegs; ++i){
			std::array<Point*, 4> vv, ss;
			std::array<Quat*, 4> qq;
			vv = {&(verts[3*i]), &(verts[3*i + 1]), &(verts[3*i + 2]), &(verts[3*i + 3])};
			ss = {&(scales[3*i]), &(scales[3*i + 1]), &(scales[3*i + 2]), &(scales[3*i + 3])};
			qq = {&(quats[3*i]), &(quats[3*i + 1]), &(quats[3*i + 2]), &(quats[3*i + 3])};
			segments[i] = std::unique_ptr<TwistSplineSegment<PointArray, Point, VectorArray, Vector, QuatArray, Quat, Float>>(
				new TwistSplineSegment<PointArray, Point, VectorArray, Vector, QuatArray, Quat, Float>(*(old.segments[i]), vv, ss, qq)
			);
		}
	}

	/// Get the entire point lookup table
	PointArray getPoints() const {
		PointArray pLut;
		resize(pLut, segments.size() * (lutSteps + 1));
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getPoints();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Get the entire scale lookup table
	PointArray getScales() const {
		PointArray sLut;
		resize(sLut, segments.size() * (lutSteps + 1));
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getScales();
			for (size_t j = 0; j < size(pa); ++j) {
				sLut[c++] = pa[j];
			}
		}
		return sLut;
	}

	/// Get the entire tangent lookup table
	VectorArray getTangents() const {
		VectorArray pLut;
		resize(pLut, segments.size() * (lutSteps + 1));
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getTangents();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Get the entire normal lookup table
	VectorArray getNormals() const {
		VectorArray pLut;
		resize(pLut, segments.size() * (lutSteps + 1));
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getTwistNormals();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Get the entire binormal lookup table
	VectorArray getBinormals() const {
		VectorArray pLut;
		resize(pLut, segments.size() * (lutSteps + 1));
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getTwistBinormals();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Get the entire unit closest-point-helper lookup table
	VectorArray getUnits() const {
		VectorArray pLut;
		resize(pLut, segments.size() * lutSteps);
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getUnits();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Get the entire arclength lookup table
	std::vector<Float> getSampleLengths() const {
		std::vector<Float> pLut;
		resize(pLut, segments.size() * lutSteps);
		size_t c = 0;
		for (auto &seg : segments) {
			auto &pa = seg->getSampleLengths();
			for (size_t j = 0; j < size(pa); ++j) {
				pLut[c++] = pa[j];
			}
		}
		return pLut;
	}

	/// Build the internal segment objects
	void buildSegments() {
		size_t numVerts = size(verts);
		if (numVerts < 2) {
			segments.clear();
			return;
		}
		size_t numSegs = ((numVerts - 1) / 3);
		segments.resize(numSegs);
		for (size_t i = 0; i < numSegs; ++i) {
			Vector iNorm;
			if (i == 0) {
				Vector y = Vector(0.0, 1.0, 0.0);
				iNorm = rotateBy(y, quats[3 * i]);
			}
			else {
				const auto &pre = segments[i-1];
				size_t last = pre->getLutSteps() - 1;
				Vector d = pre->getRawNormals()[last];
				Vector a = pre->getTangents()[last];
				Vector b = normalized(verts[3 * i + 1] - verts[3 * i]);
				Vector n = (length(a + b) == 0.0) ? b : normalized(a + b);
				iNorm = d - 2 * dot<Vector, Float>(d, n) * n;
			}
			segments[i] = std::unique_ptr<TwistSplineSegment<PointArray, Point, VectorArray, Vector, QuatArray, Quat, Float>>(
				new TwistSplineSegment<PointArray, Point, VectorArray, Vector, QuatArray, Quat, Float>(
					&(verts[3*i]), &(verts[3*i + 1]), &(verts[3*i + 2]), &(verts[3*i + 3]),
					&(scales[3*i]), &(scales[3*i + 1]), &(scales[3*i + 2]), &(scales[3*i + 3]),
					&(quats[3*i]), &(quats[3*i + 1]), &(quats[3*i + 2]), &(quats[3*i + 3]),
					iNorm, lutSteps
				)
			);
		}
	}

	/** Let the user set the input values
	  * verts: All the vertices for the spline
	  * quats: All the orientations of the vertices
	  * scales: All the scales of the vertices (TODO)
	  * lockPositions: The parameters that each vertex would get locked to
	  * lockValues: The 0-1 percentages of how locked each vertex is in parametrization
	  * userTwists: The twist values given by the user
	  * twistLocks: The 0-1 percentages of how much to use the given twist
	  * orientLocks: The 0-1 percentages of how much to use the quaternions over the RMF matrices
	  */
	void setVerts(
			const PointArray &verts,
			const PointArray &scales,
			const QuatArray &quats,
			const std::vector<Float> &lockPositions,
			const std::vector<Float> &lockValues,
			const std::vector<Float> &userTwists,
			const std::vector<Float> &twistLocks,
			const std::vector<Float> &orientLocks) {
		this->verts = verts;
		this->scales = scales;
		this->quats = quats;
		this->lockPositions = lockPositions;
		this->lockValues = lockValues;
		this->userTwists = userTwists;
		this->twistLocks = twistLocks;
		this->orientLocks = orientLocks;
		buildSegments();
		if (segments.empty())
			return;

		std::vector<Float> ulens(1);
		Float runner = 0;
		for (auto &seg : segments) {
			runner += seg->getLength();
			ulens.push_back(runner);
		}
		totalLength = runner;

		solveParamMatrix(lockPositions, ulens, lockValues, remap);
		solveTwist();
	}


	/**
	  * Solve the locks of a parameter of the spline
	  * Build a tridiagonal matrix that represents each vertex param as a relation to its neighbor params
	  * Then solve it
	  */
	void solveParamMatrix(
			const std::vector<Float> &rv /*restVals*/,
			const std::vector<Float> &cv /*currentVals*/,
			const std::vector<Float> &lv /*lockVals*/,
			std::vector<Float> &res) {
		// first, build the param matrix
		std::vector<std::array<Float, 3>> mat;
		size_t len = rv.size();
		if (len <= 1) return;
		mat.resize(len);
		res.resize(len);

		// Build the matrix
		// Start case
		mat[0][0] = 0.0;
		mat[0][1] = -1.0;
		mat[0][2] = 1.0 - lv[0];
		res[0] = -lv[0] * rv[0] + (1.0 - lv[0]) * (cv[1] - cv[0]);

		// Mid Cases
		for (size_t i = 1; i < len - 1; ++i) {
			Float A = (cv[i] - cv[i - 1]) / (cv[i + 1] - cv[i - 1]);
			mat[i][0] = (1.0 - lv[i]) * (1.0 - A);
			mat[i][1] = -1.0;
			mat[i][2] = (1.0 - lv[i]) * A;
			res[i] = -lv[i] * rv[i];
		}

		// End case
		size_t e = len - 1;
		mat[e][0] = 1.0 - lv[e];
		mat[e][1] = -1.0;
		mat[e][2] = 0.0;
		res[e] = -lv[e] * rv[e] - (1.0 - lv[e]) * (cv[e] - cv[e - 1]);

		// Then pass it to the solver
		solveTridiagonalMatrix(mat, res);
	}

	/**
	  * Solve the locks of a parameter of the spline
	  * Build a tridiagonal matrix that represents each vertex twist as a relation to its neighbor twists
	  * Twists are handled differently from other parameters
	  */
	void solveTwistParamMatrix(
			const std::vector<Float> &rv /*restVals*/,
			const std::vector<Float> &cv /*currentVals*/,
			const std::vector<Float> &lv /*lockVals*/,
			std::vector<Float> &res) {
		// first, build the param matrix
		std::vector<std::array<Float, 3>> mat;
		size_t len = rv.size();
		if (len <= 1) return;
		mat.resize(len);
		res.resize(len);

		// Build the matrix
		// Start case
		mat[0][0] = 0.0;
		mat[0][1] = -1.0;
		mat[0][2] = 1.0 - lv[0];
		res[0] = -lv[0] * rv[0];

		// Mid Cases
		for (size_t i = 1; i < len - 1; ++i) {
			Float A = (cv[i] - cv[i - 1]) / (cv[i + 1] - cv[i - 1]);
			mat[i][0] = (1.0 - lv[i]) * (1.0 - A);
			mat[i][1] = -1.0;
			mat[i][2] = (1.0 - lv[i]) * A;
			res[i] = -lv[i] * rv[i];
		}

		// End case
		size_t e = len - 1;
		mat[e][0] = 1.0 - lv[e];
		mat[e][1] = -1.0;
		mat[e][2] = 0.0;
		res[e] = -lv[e] * rv[e];

		// Then pass it to the solver
		solveTridiagonalMatrix(mat, res);
	}


	/**
	  * Solve a tridiagonal matrix in linear time
	  * If you set up parameter values in a specific way, they can be thought of as a matrix
	  * That only has numbers along the 3 most central diagonals, which can be solved efficiently
	  */
	static void solveTridiagonalMatrix(std::vector<std::array<Float, 3>> &mat, std::vector<Float> &res) {
		// Run the first special case substitution
		mat[0][2] = mat[0][2] / mat[0][1];
		res[0] = res[0] / mat[0][1];

		// Do the middle forward substitutions
		for (size_t i = 1; i < res.size(); ++i) {
			mat[i][2] = mat[i][2] / (mat[i][1] - mat[i][0] * mat[i - 1][2]);
			res[i] = (res[i] - mat[i][0] * res[i - 1]) / (mat[i][1] - mat[i][0] * mat[i - 1][2]);
		}

		// Do the back substitution
		for (size_t i = res.size() - 1; i-- > 0; ) {
			res[i] = res[i] - mat[i][2] * res[i + 1];
		}
	}

	/// Get the matrix at a given parameter. Specify whether to use the twist values or not
	void matrixAtParam(Float t, Vector &tan, Vector &norm, Vector &binorm, Point &tran, Point &scl, Float &twist, bool twisted) const {
		// Return the matrix for the given t-value
		// map the arc-length t-value through the remap vector
		Float segT;
		size_t segIdx;

		linearIndex(t, remap, segT, segIdx);

		// Then ask the segment about this new t-value
		if (twisted)
			segments[segIdx]->twistedMatrixAtParam(segT, tan, norm, binorm, tran, scl, twist);
		else
			segments[segIdx]->rawMatrixAtParam(segT, tan, norm, binorm, tran, scl, twist);
	}

	/// Get the matrices at multiple parameters
	void matricesAtParams(const std::vector<Float> &params,
			VectorArray &tans, VectorArray &norms, VectorArray &binorms,
			PointArray &trans, PointArray &scls, std::vector<Float> &twists, bool twisted) {

		std::vector<Float> segTs;
		std::vector<size_t> segIdxs;
		multiLinearIndexes(params, remap, segTs, segIdxs);

		resize(tans, params.size());
		resize(norms, params.size());
		resize(binorms, params.size());
		resize(trans, params.size());
		resize(scls, params.size());
		twists.resize(params.size());

		for (size_t i = 0; i < params.size(); ++i) {
			size_t segIdx = segIdxs[i];
			Float segT = segTs[i];

			Vector tan, norm, binorm;
			Point tran, scl;
			Float twist;

			if (twisted) {
				segments[segIdx]->twistedMatrixAtParam(segT, tan, norm, binorm, tran, scl, twist);
			}
			else {
				segments[segIdx]->rawMatrixAtParam(segT, tan, norm, binorm, tran, scl, twist);
			}

		}
	}

	/// Solve the twist parameters and apply the twist to the segments
	void solveTwist() {
		std::vector<Float> segLens(segments.size() + 1);
		std::vector<Float> orientVals(segments.size() + 1);
		for (size_t i=0; i<segments.size(); ++i){
			segLens[i+1] = segments[i]->getLength() + segLens[i];
			//restVals[i+1] = segments[i]->postAngle() + restVals[i];
			orientVals[i + 1] = segments[i]->postAngle();
		}

		// Get the angle difference between the last
		// frame of a spline and the final CV of the spline
		// Those are the orientVals
		std::vector<Float> oriMap;
		solveTwistParamMatrix(orientVals, segLens, orientLocks, oriMap);

		std::vector<Float> twistMap;
		solveTwistParamMatrix(userTwists, segLens, twistLocks, twistMap);

		for (size_t i=0; i<segments.size(); ++i){
			segments[i]->applyTwist(oriMap[i]+ twistMap[i], oriMap[i+1]+ twistMap[i+1]);
		}
	}
};
