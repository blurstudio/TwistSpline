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
#include <tuple>
#include <algorithm>
#include <math.h>
#include "twistSplineUtils.h"


struct ltGetter {
    size_t splitDim;
    ltGetter(size_t splitDim): splitDim(splitDim){}
	template <typename Point>
    inline bool operator() (const Point &p, const Point &q){
        return (*p)[splitDim] < (*q)[splitDim];
    }
};

// closest point to line segment with precomputed point/unitVector, and dist2 calc
template <typename Point, typename Vector, typename Float = double>
Float closestToSegment(const Point &s, const Point &e, const Vector &v, const Point &q, Point &out){
    Vector qs = q - s;
    Float qsv = dot<Float>(qs, v);
    if (qsv < 0.0) {
        out = s;
        return dot<Float>(qs, qs); // 6add, 6mul
    }

    Vector qe = q - e;
    Float qev = dot<Float>(qe, v);
    if (qev > 0.0){
        out = e;
        return dot<Float>(qe, qe); // 10add, 6mul
    }

    out = s + qsv * v;
    Vector oq = out-q;
    return dot<Float>(oq, oq); // 18add, 12mul
}

template <typename Point, typename Vector, typename Float = double>
Float closestToSegment(const Point &s, const Point &e, const Vector &v, const Point &q){
    Point out;
    return closestToSegment<Point, Vector, Float>(s, e, v, q, out);
}

template <typename Point, typename Vector, typename Float = double>
Float closestToSegment(const std::tuple<Point*, Point*, Vector*, size_t, size_t> &line, const Point &q, Point &out){
    return closestToSegment<Point, Vector, Float>(*std::get<0>(line), *std::get<1>(line), *std::get<2>(line), q, out);
}

template <typename Point, typename Vector, typename Float=double>
class KDNode {
private:

    const Float splitVal;
	const size_t splitDim;
	const std::vector<std::tuple<Point *, Point *, Vector *, size_t, size_t>> lines;
	const KDNode<Point, Vector, Float> *gt = nullptr;
	const KDNode<Point, Vector, Float> *lt = nullptr;
	const bool isLeaf;

    KDNode(
        const Float splitVal,
        const KDNode<Point, Vector, Float> *lt,
        const KDNode<Point, Vector, Float> *gt,
        const size_t splitDim,
        const std::vector<std::tuple<Point *, Point *, Vector *, size_t, size_t>> &lines,
		const bool isLeaf):
        splitVal(splitVal), lt(lt), gt(gt), splitDim(splitDim), lines(lines), isLeaf(isLeaf) {}

public:
	void closestPoint(
			const Point &q, Point &closest, Float &curD2,
			Float rd2, std::array<Float, 3> &off) const{
        if (isLeaf){
            // Check the "bucket" lines, and set the closest point, current distance, and wheter something was found
            for (auto &line : lines){
                Point test;
                Float newD = closestToSegment(line, q, test);
                if (newD < curD2){
                    closest = test;
                    curD2 = newD;
                }
            }
            return;
        }

        // Get the distance from the split line to the query point
		Float sDist = q[splitDim] - splitVal;
        Float sDist2 = sDist * sDist;
		Float oDist2 = off[splitDim];

		// Make sure to check the closer side first
		if (sDist < 0.0) {
			lt->closestPoint(q, closest, curD2, rd2, off);
            rd2 += sDist2 - oDist2;
			if (curD2 > rd2){
                off[splitDim] = sDist2;
				gt->closestPoint(q, closest, curD2, rd2, off);
                off[splitDim] = oDist2;
			}
		}
		else {
			gt->closestPoint(q, closest, curD2, rd2, off);
			if (curD2 > rd2){
                off[splitDim] = sDist2;
				lt->closestPoint(q, closest, curD2, rd2, off);
                off[splitDim] = oDist2;
			}
		}
    }

    // Sliding midpoint splitting rule
	static KDNode<Point, Vector, Float>* split(
            const std::vector<Point *> &points,
            const std::vector<std::tuple<Point *, Point *, Vector *, size_t, size_t>> &lines,
            const Point &maxBounds, const Point &minBounds
            ){

        if (points.size() <= 1){
            return new KDNode<Point, Vector, Float>(0.0, nullptr, nullptr, 0, lines, true);
        }
        std::vector<Point *> lows, highs, opoints;

        // Get the longest side
        Vector span = maxBounds - minBounds;
        Float sx=fabs(span[0]), sy=fabs(span[1]), sz=fabs(span[2]);
        size_t splitDim = 2;
		if (sx > sy) {
			if (sx > sz) splitDim = 0;
		}
		else {
			if (sy > sz) splitDim = 1;
		}
        // Come up with the initial splitVal
        Float splitVal = (maxBounds[splitDim] + minBounds[splitDim]) / 2.0;

        // partition the points keeping track of the possible slided splitVal
        Float hpossV, lpossV;

		lpossV = minBounds[splitDim];
		hpossV = maxBounds[splitDim];

		Point *hposs = nullptr, *lposs = nullptr;
        for (auto &p: points){
            Float checkVal = (*p)[splitDim];
            if (checkVal > splitVal){
                if (checkVal <= hpossV){
					if (hposs) highs.push_back(hposs);
                    hposs = p;
                    hpossV = checkVal;
                }
				else {
					highs.push_back(p);
				}
            }
            else {
                if (checkVal >= lpossV){
					if (lposs) lows.push_back(lposs);
                    lposs = p;
                    lpossV = checkVal;
                }
				else {
					lows.push_back(p);
				}
            }
        }

		if (!lposs) {// if no possiblity has been found for one side
			if (hposs) { // but one has been found for the other
				lows.push_back(hposs);
				splitVal = hpossV;
			}
		}
		else if (!hposs) {
			if (lposs) {
				highs.push_back(lposs);
				splitVal = lpossV;
			}
		}
		else {
			if (hposs) highs.push_back(hposs);
			if (lposs) lows.push_back(lposs);
		}

        // Partition the lines
        std::vector<std::tuple<Point *, Point *, Vector *, size_t, size_t>> highLines, lowLines;
        for (auto &n: lines){
			Float startCheck = (*std::get<0>(n))[splitDim];
			Float endCheck = (*std::get<1>(n))[splitDim];
			bool isHigh=false, isLow=false;

			if (startCheck > splitVal) isHigh = true;
            else if (startCheck < splitVal) isLow = true;
			//else isHigh = isLow = true;

            if (endCheck > splitVal) isHigh = true;
            else if (endCheck < splitVal) isLow = true;
			//else isHigh = isLow = true;
			
			if (isHigh) highLines.push_back(n);
			if (isLow) lowLines.push_back(n);
        }

        KDNode<Point, Vector, Float> *lt, *gt;
        Point gtMin(minBounds), ltMax(maxBounds);
        gtMin[splitDim] = splitVal;
        ltMax[splitDim] = splitVal;

        lt = KDNode::split(lows, lowLines, ltMax, minBounds);
        gt = KDNode::split(highs, highLines, maxBounds, gtMin);

		const std::vector<std::tuple<Point *, Point *, Vector *, size_t, size_t>> emptyLines;
        return new KDNode<Point, Vector, Float>(splitVal, lt, gt, splitDim, emptyLines, false);
    }

	~KDNode() {
		if (lt) delete lt;
		if (gt) delete gt;
	}
};

