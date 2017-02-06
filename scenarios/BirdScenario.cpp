#include "scenarios/BirdScenario.h"
#include <iostream>
#include <fstream>

#include "render/OBJParser.h"

const double gLineWidth = 1;
const double gPointSize = 10;
const int gSegmentSamples = 20;

cBirdScenario::cBirdScenario()
{
	// new curve parameter files can be added here
	mParamFiles.push_back("data/curve_params/catmull_rom.txt");
	mParamFiles.push_back("data/curve_params/b_spline.txt");
}

cBirdScenario::~cBirdScenario()
{
	mShader.free();
}

void cBirdScenario::Init()
{
	cScenario::Init();
	LoadShaders();

	if (mParamFiles.size() > 0)
	{
		LoadParams(mParamFiles[0]);
	}

	mCharTransform.setIdentity();
	LoadMesh();
}

void cBirdScenario::Update(double time_elapsed)
{
	cScenario::Update(time_elapsed);
	UpdateCharacter();
}

double cBirdScenario::GetPlaybackProgress() const
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

void cBirdScenario::SetPlaybackProgress(double val)
{
	double max_time = mCurve.GetMaxTime();
	double time = max_time * val;
	SetTime(time);
}

int cBirdScenario::GetNumAnchors() const
{
	return mCurve.GetNumAnchors();
}

int cBirdScenario::GetNumCurveSamples() const
{
	return GetNumAnchors() * gSegmentSamples;
}

void cBirdScenario::LoadShaders()
{
	mShader.initFromFiles("a_simple_shader", "data/shaders/Mesh_VS.glsl", "data/shaders/Mesh_PS.glsl");
}

int cBirdScenario::GetVertBufferSize() const
{
	// layout of vertex buffer is
	// [anchor tangent vertices | anchor position vertices | curve sample vertices | character vertices]
	int num_anchors = GetNumAnchors();
	int num_curve_samples = GetNumCurveSamples();
	int num_quad_verts = 4;
	return num_anchors + 2 * num_anchors + num_curve_samples + num_quad_verts;
}

void cBirdScenario::LoadMesh()
{
	std::string mesh_file = "data/meshes/humming_bird.obj";
	bool succ = cOBJParser::LoadMesh(mesh_file, mCharMesh);
	if (!succ)
	{
		printf("Failed to load mesh from %s\n", mesh_file.c_str());
	}
}

void cBirdScenario::LoadParams(const std::string& param_file)
{
	bool succ = mCurve.Load(param_file);
	if (succ)
	{
		printf("Loaded curve from %s\n", param_file.c_str());
		UpdateCurve();
	}
	else
	{
		printf("Failed to load curve from %s\n", param_file.c_str());
	}
}

void cBirdScenario::UpdateCurve()
{
	int num_curve_samples = GetNumCurveSamples();
	double max_time = mCurve.GetMaxTime();
	mCurveSamples.resize(3, num_curve_samples);

	for (int i = 0; i < num_curve_samples; ++i)
	{
		double t = static_cast<double>(i) / (num_curve_samples - 1);
		t *= max_time;
		Eigen::VectorXd pos;
		mCurve.Eval(t, pos);
		assert(pos.size() == mCurveSamples.rows());

		mCurveSamples.col(i) = pos;
	}
}

void cBirdScenario::UpdateCharacter()
{
	// TODO (CPSC426): Implement Frenet Frame for a bird traveling along a cruve
	// Update the character transformation matrix that that it
	// moves along the curve and oriented such that it is facing along
	// the tangent to the curve

	double max_time = mCurve.GetMaxTime();
	double curr_time = mTime;
	curr_time = std::fmod(curr_time, max_time);

	Eigen::VectorXd pos_data;
	mCurve.Eval(curr_time, pos_data);

	tVector pos = tVector(pos_data[0], pos_data[1], pos_data[2], 0);
	mCharTransform.setIdentity();
	mCharTransform.block(0, 3, 3, 1) = pos.segment(0, 3);
}

void cBirdScenario::SetColor(const tVector& col)
{
	cDrawUtil::SetColor(col);
}

void cBirdScenario::SetupDraw()
{
	cScenario::SetupDraw();
	SetupShader();
}

void cBirdScenario::SetupShader()
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

void cBirdScenario::DrawScene()
{
	DrawCurve();
	DrawAnchors();
	DrawAnchorTangents();
	DrawCharacter();

	// examples for drawing different objects
	// feel free to comment out when no longer needed
	DrawObjects();
}

void cBirdScenario::DrawCurve()
{
	cDrawUtil::SetLineWidth(gLineWidth);
	SetColor(tVector(0, 1, 0, 1));
	
	for (int i = 0; i < mCurveSamples.cols() - 1; ++i)
	{
		tVector a = tVector(mCurveSamples(0, i), mCurveSamples(1, i), mCurveSamples(2, i), 0);
		tVector b = tVector(mCurveSamples(0, i + 1), mCurveSamples(1, i + 1), mCurveSamples(2, i + 1), 0);
		cDrawUtil::DrawLine(a, b);
	}
}

void cBirdScenario::DrawAnchors()
{
	glPointSize(static_cast<GLfloat>(gPointSize));
	SetColor(tVector(0.1, 0.1, 0.1, 1));
	
	int num_anchors = mCurve.GetNumAnchors();
	for (int i = 0; i < num_anchors; ++i)
	{
		const auto& pos = mCurve.GetAnchorPos(i);
		cDrawUtil::DrawPoint(tVector(pos[0], pos[1], pos[2], 0));
	}
}

void cBirdScenario::DrawAnchorTangents()
{
	cDrawUtil::SetLineWidth(gLineWidth);
	SetColor(tVector(0, 0, 1, 1));
	
	int num_anchors = mCurve.GetNumAnchors();
	for (int i = 0; i < num_anchors; ++i)
	{
		const auto& pos = mCurve.GetAnchorPos(i);
		const auto& tangent = mCurve.GetAnchorTangent(i);
		cDrawUtil::DrawLine(tVector(pos[0], pos[1], pos[2], 0), 
							tVector(pos[0], pos[1], pos[2], 0) + tVector(tangent[0], tangent[1], tangent[2], 0));
	}
}

void cBirdScenario::DrawCharacter()
{
	SetColor(tVector(0.85, 0.85, 0.9, 1));

	cDrawUtil::PushMatrix();
	cDrawUtil::MultMatrix(mCharTransform);
	cDrawUtil::Scale(tVector(4, 4, 4, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(0, 1, 0, 0));
	mCharMesh.Draw(GL_TRIANGLES);
	cDrawUtil::PopMatrix();
}

void cBirdScenario::DrawObjects()
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(0, 0.5, -1, 0));
	SetColor(tVector(0.85, 0.2, 0.2, 1));
	cDrawUtil::DrawSphere(0.1);
	cDrawUtil::PopMatrix();
	
	SetColor(tVector(0.2, 0.85, 0.2, 1));
	cDrawUtil::DrawBox(tVector(0.5, 0.5, -1, 0), tVector(0.2, 0.2, 0.2, 0));

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(-0.5, 0.5, -1, 0));
	SetColor(tVector(0.15, 0.2, 0.85, 1));
	cDrawUtil::DrawCylinder(0.2, 0.05);
	cDrawUtil::PopMatrix();
}

tVector cBirdScenario::GetClearColor() const
{
	return tVector(0.25, 0.25, 0.25, 0);
}
