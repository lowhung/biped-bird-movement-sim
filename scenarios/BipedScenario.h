#pragma once

#include <nanogui/glutil.h>
#include "scenarios/Scenario.h"
#include "Curve.h"
#include "render/Shader.h"
#include "ArticulatedFigure.h"

// animates a biped using keyframes

class PLUGIN_EXPORT cBipedScenario : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBipedScenario();
	virtual ~cBipedScenario();

	virtual void Init();
	virtual void LoadParams(const std::string& param_file);

	virtual void Update(double time_elapsed);

	virtual double GetPlaybackProgress() const;
	virtual void SetPlaybackProgress(double val);
	
protected:
	
	cShader mShader;
	cCurve mCurve;

	std::unique_ptr<cArticulatedFigure> mChar;

	virtual void InitCamera();

	virtual void LoadShaders();
	virtual void BuildCharacter(std::unique_ptr<cArticulatedFigure>& out_char) const;
	virtual void UpdateCharacter();

	virtual void SetColor(const tVector& col);

	virtual void SetupDraw();
	virtual void SetupShader();

	virtual void DrawScene();
	virtual void DrawGround();
	virtual void DrawCharacter();

	virtual tVector GetClearColor() const;
};
