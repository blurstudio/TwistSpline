#include "drawOverride.h"

TwistSplineGeometryOverride::TwistSplineGeometryOverride(const MObject& obj)
	: MPxGeometryOverride(obj), mTwistSplineNode(nullptr) {
	MStatus status;
	MFnDependencyNode dependNode(obj, &status);
	if (status != MStatus::kSuccess) return;

	MPxNode* twistSplineNode = dependNode.userNode(&status);
	if (status != MStatus::kSuccess) twistSplineNode = nullptr;

	mTwistSplineNode = dynamic_cast<TwistSplineNode*>(twistSplineNode);
}

TwistSplineGeometryOverride::~TwistSplineGeometryOverride() {}

void TwistSplineGeometryOverride::updateDG() {
	/*
		Pull (evaluate) all attributes in TwistSplineNodeNode node we will be
		using. Here is the list of attributes we are pulling :
		- geometryChanging: Needed by requiresGeometryUpdate() to check if we
		   needs to update the vertex and index buffer;
		- debugDisplay: Needed by addUIDrawables() to draw debug mode;
		- debugScale: Needed by addUIDrawables() to draw debug mode scale;
		- outputSpline: Needed by addUIDrawables() to draw the main spline;

		It is very important that all the attributes pulled from this method are
		cached (with Technique 1) otherwise Evaluation Cache will not work. In
		fact, this method is not needed by EM Evaluation modes.
	*/
	mTwistSplineNode->updateRenderAttributes();
}

bool TwistSplineGeometryOverride::requiresUpdateRenderItems(
	const MDagPath& path) const {
	/*
		Override this function if you need a more complicated animated-material
		behavior For example, you will need to change this to `return true` if
		object's associated shader is changing at every frame. (This could be
		very expensive).

		Note 1: this method have to return 'false' in most case, otherwise
		VP2 caching may not work.

		Note 2: for rendering simple animated-material like animated-color or
		animated-texture-uv check FootPrintGeometryOverrideAnimatedMaterial
		example.
	*/
	return false;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Geometry update and VP2 cache implementations
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

/*
	Return true when the aOutputSpline changes, which requires us to re-generate
	the geometry.

	Note: this method must return the exact same value in a cache-restoration
	frame as it was in the corresponding cache-store frame.
*/
bool TwistSplineGeometryOverride::requiresGeometryUpdate() const {
	/*
		Checking the "TwistSplineNodeNode::geometryChanging" attribute if any
		attribute affecting the geometry is changing,
		"TwistSplineNodeNode::geometryChanging" should be affected and dirtied.
		Also check TwistSplineNodeNodometry(). Warning: this method may be
		called outside of regular { update() : cleanUp() } scope. Thus, we must
		invoke node-local evaluation to ensure the correctness.
	*/
	return mTwistSplineNode->isGeometryChanging();
}

// Generate the geometry(vertex / index) from TwistSplineNodeNode's parameter
// data
// [[ensure : requiresGeometryUpdate() == false]]
void TwistSplineGeometryOverride::populateGeometry(
	const MGeometryRequirements& requirements,
	const MRenderItemList& renderItems, MGeometry& data) {
	// This call will ensure the post-condition that requiresGeometryUpdate() is
	// false
	mTwistSplineNode->updatingGeometry();
}

void TwistSplineGeometryOverride::addUIDrawables(
	const MDagPath& path, MHWRender::MUIDrawManager& drawManager,
	const MHWRender::MFrameContext& frameContext) {
	// Retrieve the spline data to draw elements;
	auto splineData = mTwistSplineNode->getSplineData();
	auto splinePoints = splineData->getPoints();

	bool splineDraw;
	bool debugDraw;
	double debugScale;
	mTwistSplineNode->getSplineDraw(splineDraw);
	mTwistSplineNode->getDebugDraw(debugDraw, debugScale);

	MFnDependencyNode node(path.node());
	bool draw2D = false;  // ALWAYS false

	drawManager.beginDrawable(MUIDrawManager::kSelectable);

	if (splineDraw) {
		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);
		MColor color = MHWRender::MGeometryUtilities::wireframeColor(path);
		drawManager.setColor(color);
		drawManager.lineStrip(splinePoints, draw2D);
	}

	if (debugDraw) {
		MPointArray tangents;
		MVectorArray tans = splineData->getTangents();
		tangents.setLength(tans.length() * 2);
		for (size_t i = 0; i < tans.length(); ++i) {
			MPoint& spi = splinePoints[i];
			tangents[2 * i] = spi;
			tangents[(2 * i) + 1] = debugScale * tans[i] + spi;
		}
		drawManager.setColor(MColor(.5, 0, 0));
		drawManager.lineList(tangents, draw2D);

		MPointArray normals;
		MVectorArray norms = splineData->getNormals();
		normals.setLength(norms.length() * 2);
		for (size_t i = 0; i < norms.length(); ++i) {
			MPoint& spi = splinePoints[i];
			MVector& nn = norms[i];
			normals[2 * i] = spi;
			normals[(2 * i) + 1] = debugScale * nn + spi;
		}
		drawManager.setColor(MColor(0, .5, 0));
		drawManager.lineList(normals, draw2D);

		MPointArray binormals;
		MVectorArray binorms = splineData->getBinormals();
		binormals.setLength(binorms.length() * 2);
		for (size_t i = 0; i < binorms.length(); ++i) {
			MPoint& spi = splinePoints[i];
			binormals[2 * i] = spi;
			binormals[(2 * i) + 1] = debugScale * binorms[i] + spi;
		}
		drawManager.setColor(MColor(0, 0, .5));
		drawManager.lineList(binormals, draw2D);
	}
	drawManager.endDrawable();
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// Debug functions
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

// Return true if internal tracing is desired.
bool TwistSplineGeometryOverride::traceCallSequence() const {
	/*
		Tracing will look something like the following when in shaded mode (on
		selection change):
		- TwistSplineGeometryOverride: Geometry override DG update: twistSpline1
		- TwistSplineGeometryOverride: Start geometry override render item
	   update: |transform1|twistSpline1
		- TwistSplineGeometryOverride: - Call API to update render items
		- TwistSplineGeometryOverride: End geometry override render item update:
		|transform1|twistSpline1
		- TwistSplineGeometryOverride: End geometry override clean up:
		twistSpline1

		This is based on the existing stream and indexing dirty flags being used
		which attempts to minimize the amount of render item, vertex buffer and
		indexing update.
	*/
	return false;
}

inline void TwistSplineGeometryOverride::handleTraceMessage(
	const MString& message) const {
	MGlobal::displayInfo(gPluginNodeMessagePrefix + message);

	// Some simple custom message formatting.
	fputs(gPluginNodeMessagePrefix, stderr);
	fputs(message.asChar(), stderr);
	fputs("\n", stderr);
}
