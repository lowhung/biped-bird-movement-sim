#pragma once

#include <nanogui/glutil.h>
#include "scenarios/Scenario.h"
#include "Curve.h"
#include "render/Shader.h"

// animates a bird traveling along a curve

class PLUGIN_EXPORT cBirdScenario : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBirdScenario();
	virtual ~cBirdScenario();

	virtual void Init();

	virtual void Update(double time_elapsed);
	virtual void LoadParams(const std::string& param_file);

	virtual double GetPlaybackProgress() const;
	virtual void SetPlaybackProgress(double val);

protected:
	
	cShader mShader;
	cCurve mCurve;

	Eigen::MatrixXd mCurveSamples;
	tMatrix mCharTransform;
	cDrawMesh mCharMesh;

	virtual int GetNumAnchors() const;
	virtual int GetNumCurveSamples() const;

	virtual void LoadShaders();
	virtual int GetVertBufferSize() const;

	virtual void LoadMesh();

	virtual void UpdateCurve();
	virtual void UpdateCharacter();

	virtual void SetColor(const tVector& col);

	virtual void SetupDraw();
	virtual void SetupShader();

	virtual void DrawScene();
	virtual void DrawCurve();
	virtual void DrawAnchors();
	virtual void DrawAnchorTangents();
	virtual void DrawCharacter();
	virtual void DrawObjects();

	virtual tVector GetClearColor() const;
};
