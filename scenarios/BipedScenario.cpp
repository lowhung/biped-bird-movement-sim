#include "scenarios/BipedScenario.h"
#include <iostream>
#include <fstream>

#include "render/OBJParser.h"

cBipedScenario::cBipedScenario()
{
	// new curve parameter files can be added here
	mParamFiles.push_back("data/char_params/biped_walk.txt");
	mParamFiles.push_back("data/char_params/biped_inplace_walk.txt");
}

cBipedScenario::~cBipedScenario()
{
	mShader.free();
}

void cBipedScenario::Init()
{
	cScenario::Init();
	LoadShaders();

	if (mParamFiles.size() > 0)
	{
		LoadParams(mParamFiles[0]);
	}
	
	BuildCharacter(mChar);
	mChar->Init();
}

void cBipedScenario::Update(double time_elapsed)
{
	cScenario::Update(time_elapsed);
	UpdateCharacter();
}

double cBipedScenario::GetPlaybackProgress() const
{
	double max_time = mCurve.GetMaxTime();
	double progress = mTime / max_time;
	progress = std::fmod(progress, 1);
	if (progress < 0)
	{
		progress += 1;
	}
	return progress;
}

void cBipedScenario::SetPlaybackProgress(double val)
{
	double max_time = mCurve.GetMaxTime();
	double time = max_time * val;
	SetTime(time);
}

void cBipedScenario::InitCamera()
{
	double h = 4;
	double w = (h * mWinSize.x()) / mWinSize.y();
	double near_z = 0.1;
	double far_z = 20;

	mCamera = cCamera(tVector(0, 2, 10, 0), tVector(0, 0.8, 0, 0), tVector(0, 1, 0, 0),
						w, h, near_z, far_z);
	mCamera.SetProj(cCamera::eProjPerspective);
}

void cBipedScenario::LoadShaders()
{
	mShader.initFromFiles("a_simple_shader", "data/shaders/Mesh_VS.glsl", "data/shaders/Mesh_PS.glsl");
}

void cBipedScenario::LoadParams(const std::string& param_file)
{
	bool succ = mCurve.Load(param_file);
	if (succ)
	{
		printf("Loaded curve from %s\n", param_file.c_str());
	}
	else
	{
		printf("Failed to load curve from %s\n", param_file.c_str());
	}
}

void cBipedScenario::BuildCharacter(std::unique_ptr<cArticulatedFigure>& out_char) const
{
	out_char = std::unique_ptr<cArticulatedFigure>(new cArticulatedFigure());
}

void cBipedScenario::UpdateCharacter()
{
	double max_time = mCurve.GetMaxTime();
	double curr_time = mTime;
	curr_time = std::fmod(curr_time, max_time);
	if (curr_time < 0)
	{
		curr_time += max_time;
	}

	Eigen::VectorXd pose;
	mCurve.Eval(curr_time, pose);
	mChar->SetPose(pose);
}

void cBipedScenario::SetColor(const tVector& col)
{
	cDrawUtil::SetColor(col);
}

void cBipedScenario::SetupDraw()
{
	cScenario::SetupDraw();
	SetupShader();
}


void cBipedScenario::SetupShader()
{
	tVector light_dir = tVector(0.1, 1, 0.5, 0).normalized();
	const tVector light_col = tVector(0.5, 0.5, 0.5, 0);
	const tVector ambient_col = tVector(0.5, 0.5, 0.5, 0);

	mShader.Bind();

	tMatrix view_mat = mCamera.BuildWorldViewMatrix();
	light_dir = view_mat * light_dir;

	mShader.setUniform("gLightDir", Eigen::Vector3f(light_dir[0], light_dir[1], light_dir[2]));
	mShader.setUniform("gLightColour", Eigen::Vector3f(light_col[0], light_col[1], light_col[2]));
	mShader.setUniform("gAmbientColour", Eigen::Vector3f(ambient_col[0], ambient_col[1], ambient_col[2]));
}

void cBipedScenario::DrawScene()
{
	cScenario::DrawScene();
	DrawGround();
	DrawCharacter();
}

void cBipedScenario::DrawGround()
{
	const double size = 100;
	SetColor(tVector(0.7, 0.7, 0.7, 1));
	cDrawUtil::DrawPlane(tVector(0, 1, 0, 0), size);
}

void cBipedScenario::DrawCharacter()
{
	mChar->Draw();
}

tVector cBipedScenario::GetClearColor() const
{
	return tVector(0.8, 0.8, 0.8, 0);
}