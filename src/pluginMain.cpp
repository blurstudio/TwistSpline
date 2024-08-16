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

#include "twistSplineNode.h"
#include "twistSplineData.h"
#include "riderConstraint.h"
#include "twistTangentNode.h"
#include "drawOverride.h"
#include <maya/MFnPlugin.h>

MStatus initializePlugin( MObject obj ) {
	MStatus   status;
	MFnPlugin plugin( obj, "BlurStudio", "1.2.0", "Any");

	status = plugin.registerData("twistSplineData", TwistSplineData::id, TwistSplineData::creator);
	if (!status) {
		status.perror("Failed to register twistSplineData");
		return status;
	}

	status = plugin.registerNode(
		"twistSpline",
		TwistSplineNode::id,
		&TwistSplineNode::creator,
		&TwistSplineNode::initialize,
		MPxNode::kLocatorNode,
		&TwistSplineNode::drawDbClassification
	);

	if (!status) {
		status.perror("Failed to register twistSplineNode");
		return status;
	}

	status = plugin.registerNode(
		"riderConstraint",
		riderConstraint::id,
		riderConstraint::creator,
		riderConstraint::initialize
	);
	if (!status) {
		status.perror("Failed to register riderConstraint Node");
		return status;
	}



	status = plugin.registerNode(
		"twistTangent",
		TwistTangentNode::id,
		TwistTangentNode::creator,
		TwistTangentNode::initialize
	);
	if (!status) {
		status.perror("Failed to register TwistTangent Node");
		return status;
	}

	status = MHWRender::MDrawRegistry::registerGeometryOverrideCreator(
		TwistSplineNode::drawDbClassification,
		TwistSplineNode::drawRegistrantId,
		TwistSplineGeometryOverride::Creator
	);
	if (!status) {
		status.perror("Faild to register TwistSpline GeometryOverrideCreator");
		return status;
	}
	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MStatus   nodeStat, dataStat, drawStat, riderStat, tangentStat;
	MFnPlugin plugin(obj);

	dataStat = plugin.deregisterData(TwistSplineData::id);
	if (!dataStat) dataStat.perror("Failed to de-register twistSplineData");

	nodeStat = plugin.deregisterNode(TwistSplineNode::id);
	if (!nodeStat) nodeStat.perror("Failed to de-register twistSplineNode");

	riderStat = plugin.deregisterNode(riderConstraint::id);
	if (!riderStat) nodeStat.perror("Failed to de-register riderConstraint Node");

	tangentStat = plugin.deregisterNode(TwistTangentNode::id);
	if (!tangentStat) nodeStat.perror("Failed to de-register twistTangent Node");

	drawStat = MDrawRegistry::deregisterGeometryOverrideCreator(
		TwistSplineNode::drawDbClassification,
		TwistSplineNode::drawRegistrantId);
	if (!drawStat) drawStat.perror("deregisterGeometryOverrideCreator");

	if (!nodeStat) return nodeStat;
	if (!dataStat) return dataStat;
	if (!riderStat) return riderStat;
	if (!tangentStat) return tangentStat;
	return drawStat;
}
