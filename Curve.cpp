#include "Curve.h"
#include <fstream>
#include <iostream>

const std::string gTypeKey = "Type";
const std::string gAnchorsKey = "Anchors";
const std::string gAnchorPosKey = "Pos";
const std::string gSegmentDurationKey = "SegmentDuration";


cCurve::tAnchor::tAnchor()
{
	mPos.setZero();
	mTangent.setZero();
}

cCurve::tAnchor::~tAnchor()
{
}

cCurve::eCurveType cCurve::ParseCurveType(const std::string& str)
{
	// convert a string into the corresponding curve type
	eCurveType curve_type = eCurveTypeCatmullRom;

	if (str == "catmull_rom")
	{
		curve_type = eCurveTypeCatmullRom;
	}
	else if (str == "b_spline")
	{
		curve_type = eCurveTypeBSpline;
	}
	else
	{
		printf("Unsupported curve type %s\n", str.c_str());
		assert(false); // unsupported curve type
	}

	return curve_type;
}

cCurve::cCurve()
{
	mSegmentDuration = 1;
	mCurveType = eCurveTypeCatmullRom;
}

cCurve::~cCurve()
{
}

bool cCurve::Load(const std::string& file)
{
	// load a set of anchor points and other parameters from a text file
	// file should be formatted as a JSON
	bool succ = true;
	Clear();

	std::ifstream f_stream(file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		// parse misc parameters from the file
		std::string type_str = root.get(gTypeKey, "").asString();
		mCurveType = ParseCurveType(type_str);
		mSegmentDuration = root.get(gSegmentDurationKey, 1).asDouble();

		// parse the list of anchors
		if (!root[gAnchorsKey].isNull())
		{
			auto anchors_json = root.get(gAnchorsKey, 0);
			succ &= ParseAnchors(anchors_json);
		}
	}

	if (succ)
	{
		// provides some examples of how to work with the anchor data structure
		PrintAnchors();
	}
	else
	{
		Clear();
	}

	return succ;
}

void cCurve::Clear()
{
	mAnchors.clear();
	mSegmentDuration = 1;
}

int cCurve::GetNumAnchors() const
{
	return static_cast<int>(mAnchors.size());
}

const Eigen::VectorXd& cCurve::GetAnchorPos(int i) const
{
	return mAnchors[i].mPos;
}

const Eigen::VectorXd& cCurve::GetAnchorTangent(int i) const
{
	return mAnchors[i].mTangent;
}

int cCurve::GetNumSegments() const
{
	// computes the number of curve segments from the number of anchors
	int num_anchors = GetNumAnchors();
	int num_segs = 0;
	switch (mCurveType)
	{
	case eCurveTypeCatmullRom:
		num_segs = num_anchors - 1;
		break;
	case eCurveTypeBSpline:
		num_segs = num_anchors + 1;
		break;
	default:
		assert(false); // unsuppoted curve type
		break;
	}

	return num_segs;
}

int cCurve::GetDim() const
{
	// dimension of each anchor
	int dim = 0;
	if (GetNumAnchors() > 0)
	{
		const auto& pos = GetAnchorPos(0);
		dim = pos.size();
	}
	return dim;
}

void cCurve::Eval(double time, Eigen::VectorXd& out_result) const
{
	// TODO (CPSC426): Evaluates a parametric curve at the given time
	out_result = Eigen::VectorXd::Zero(GetDim());
	// Set each of out_result's dimensions to be time

	const tAnchor& anchor_0 = mAnchors[0];
	const tAnchor& anchor_1 = mAnchors[1];
	const tAnchor& anchor_2 = mAnchors[2];
	const tAnchor& anchor_3 = mAnchors[3];
	const tAnchor& anchor_4 = mAnchors[4];

	std::cout << GetNumSegments() << std::endl;

// Basis Matrix Construction B-SPLINE
Eigen::Matrix4d basis_matrix_bird_bs;
basis_matrix_bird_bs << -1.0, 3.0, -3.0, 1.0,
						 3.0,-6.0,  3.0, 0.0,
						-3.0, 0.0,  3.0, 0.0,
						 1.0, 4.0,  1.0, 0.0;

// Basis Matrix Construction CATMULL_ROM
Eigen::Matrix4d basis_matrix_bird_cr;
basis_matrix_bird_cr << -1.0,  3.0, -3.0,  1.0,
						 2.0, -5.0,  4.0, -1.0,
						-1.0,  0.0,  1.0,  0.0,
						 0.0,  2.0,  0.0,  0.0;

// Geometric Matrix Construction 
Eigen::MatrixXd geometric_matrix_bird_0(4, GetDim());
geometric_matrix_bird_0 <<	anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
							anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
							anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
						    anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2];

Eigen::MatrixXd geometric_matrix_bird_1(4, GetDim());
geometric_matrix_bird_1 <<	anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
							anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
							anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
						    anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2];

Eigen::MatrixXd geometric_matrix_bird_2(4, GetDim());
geometric_matrix_bird_2 << 	anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
							anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
							anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
						    anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2];

Eigen::MatrixXd geometric_matrix_bird_3(4, GetDim());
geometric_matrix_bird_3 <<	anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
							anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
							anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2],
						    anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2];

Eigen::MatrixXd geometric_matrix_bird_4(4, GetDim());
geometric_matrix_bird_4 <<	anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
							anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2],
							anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2],
						    anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2];

Eigen::MatrixXd geometric_matrix_bird_5(4, GetDim());
geometric_matrix_bird_5 <<	anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2],
							anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2],
							anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2],
						    anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2];

// Time Vector Construction
Eigen::Vector4d time_vector;
int n_seg = (int) time;
float t_seg = time - n_seg;
time_vector << t_seg*t_seg*t_seg, t_seg*t_seg, t_seg, 1;

switch (mCurveType)
{

case eCurveTypeCatmullRom:
if (n_seg == 0){
	out_result =  0.5 * time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_1;
}
else if (n_seg == 1) {
	out_result = 0.5 * time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_2;
}
else if (n_seg == 2) {
	out_result = 0.5 * time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_3;
}
else if (n_seg == 3) {
	out_result = 0.5 * time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_4;
} break;

case eCurveTypeBSpline:
if (n_seg == 0){
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_0;
}
else if (n_seg == 1) {
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_1;
}
else if (n_seg == 2) {
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_2;
}
else if (n_seg == 3) {
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_3;
}
else if (n_seg == 4) {
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_4;
}
else if (n_seg == 5) {
	out_result = 0.1666666667 * time_vector.transpose() * basis_matrix_bird_bs * geometric_matrix_bird_5;
}
		break;
default:
	assert(false); // unsuppoted curve type
	break;
}
	// first build the basis matrix M for the current curve style (mCurveType)
	// then build the T polynomial vector
	// finally build the geometry matrix G for the curve segment
}

void cCurve::EvalTangent(double time, Eigen::VectorXd& out_result) const
{
	// TODO (CPSC426): Evaluates the first derivative of a curve
	out_result = Eigen::VectorXd::Zero(GetDim()); // stub
	out_result[0] = 1;

// 	const tAnchor& anchor_0 = mAnchors[0];
// 	const tAnchor& anchor_1 = mAnchors[1];
// 	const tAnchor& anchor_2 = mAnchors[2];
// 	const tAnchor& anchor_3 = mAnchors[3];
// 	const tAnchor& anchor_4 = mAnchors[4];

// 	// std::cout << GetAnchorTime(0) << std::endl;
// 	// std::cout << GetAnchorTime(1) << std::endl;
// 	// std::cout << GetAnchorTime(2) << std::endl;
// 	// std::cout << GetAnchorTime(3) << std::endl;

// // Basis Matrix Construction B-SPLINE
// Eigen::Matrix4d basis_matrix_bird_bs;
// basis_matrix_bird_bs << -1.0, 3.0, -3.0, 1.0,
// 						 3.0,-6.0,  3.0, 0.0,
// 						-3.0, 0.0,  3.0, 0.0,
// 						 1.0, 4.0,  1.0, 0.0;

// // Basis Matrix Construction CATMULL_ROM
// Eigen::Matrix4d basis_matrix_bird_cr;
// basis_matrix_bird_cr << -1.0,  3.0, -3.0,  1.0,
// 						 2.0, -5.0,  4.0, -1.0,
// 						-1.0,  0.0,  1.0,  0.0,
// 						 0.0,  2.0,  0.0,  0.0;

// Eigen::Vector4d tangent_time_vector;
// tangent_time_vector << 3*time*time, 2*time, 1.00000000, 0.00000;

// Eigen::MatrixXd geometric_matrix_bird_1(4, GetDim());
// geometric_matrix_bird_1 <<	anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
// 							anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
// 							anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
// 						    anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2];

// Eigen::MatrixXd geometric_matrix_bird_2(4, GetDim());
// geometric_matrix_bird_2 << 	anchor_0.mPos[0], anchor_0.mPos[1], anchor_0.mPos[2],
// 							anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
// 							anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
// 						    anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2];

// Eigen::MatrixXd geometric_matrix_bird_3(4, GetDim());
// geometric_matrix_bird_3 <<	anchor_1.mPos[0], anchor_1.mPos[1], anchor_1.mPos[2],
// 							anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
// 							anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2],
// 						    anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2];

// Eigen::MatrixXd geometric_matrix_bird_4(4, GetDim());
// geometric_matrix_bird_4 <<	anchor_2.mPos[0], anchor_2.mPos[1], anchor_2.mPos[2],
// 							anchor_3.mPos[0], anchor_3.mPos[1], anchor_3.mPos[2],
// 							anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2],
// 						    anchor_4.mPos[0], anchor_4.mPos[1], anchor_4.mPos[2];

// switch(mCurveType){
// 	case eCurveTypeCatmullRom:
// 	out_result[0] = tangent_time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_1;
// 	out_result[1] = tangent_time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_2;
// 	out_result[2] = tangent_time_vector.transpose() * basis_matrix_bird_cr * geometric_matrix_bird_3;
// 	break;
// 	case eCurveTypeBSpline:
// 	break;
// 	default: 
// 		assert(false);
// 		break;
// }


}

void cCurve::EvalNormal(double time, Eigen::VectorXd& out_result) const
{
	// TODO (CPSC426): Evaluates the second derivative of a curve
	out_result = Eigen::VectorXd::Zero(GetDim()); // stub

	Eigen::Vector4d normal_time_vector;
	normal_time_vector << 6 * time, 1, 0, 0;
}


double cCurve::GetMaxTime() const
{
	// returns the total time needed to travel along the curve from start to end
	return mSegmentDuration * GetNumSegments();
}

void cCurve::Add(const tAnchor& anchor)
{
	mAnchors.push_back(anchor);
}

bool cCurve::ParseAnchors(const Json::Value& root)
{
	// parses an array of anchors from root

	assert(root.isArray());
	bool succ = true;

	int num_anchors = root.size();
	mAnchors.resize(num_anchors);

	// anchors are stored as a list of points
	// the points can be of any dimension, but the dimensions of
	// all points should be the same
	for (int i = 0; i < num_anchors; ++i)
	{
		const auto& anchor_json = root.get(i, 0);
		tAnchor& curr_anchor = mAnchors[i];
		succ &= ParseAnchor(anchor_json, curr_anchor);
	}

	if (succ)
	{
		// compute and store the tangents at the achor points
		// these achnor tangets are currently used only for visualization
		ComputeAnchorTangents();
	}

	return succ;
}

bool cCurve::ParseAnchor(const Json::Value& root, tAnchor& out_anchor) const
{
	// parse anchors specified using a JSON format
	bool succ = true;
	if (!root[gAnchorPosKey].isNull())
	{
		const auto& pos_json = root.get(gAnchorPosKey, 0);
		int curr_dim = pos_json.size();
		out_anchor.mPos.resize(curr_dim);

		int dim = GetDim();
		succ = curr_dim == dim;
		if (!succ)
		{
			printf("Anchor dimension mismatch, expecting %i got %i\n", dim, curr_dim);
			assert(false);
		}

		if (succ)
		{
			// each anchor is defined as a list of numbers
			for (int i = 0; i < curr_dim; ++i)
			{
				out_anchor.mPos[i] = pos_json.get(i, 0).asDouble();
			}
		}
	}
	else
	{
		succ = false;
	}

	return succ;
}

void cCurve::PrintAnchors() const
{
	// prints out the positions of all anchors

	int num_anchors = GetNumAnchors();
	int dim = GetDim(); // dimension of each anchor
	printf("Curve Anchors:\n");
	for (int i = 0; i < num_anchors; ++i)
	{
		const tAnchor& anchor = mAnchors[i];
		printf("Anchor %i:\t", i);

		// print the position of each anchor
		for (int j = 0; j < dim; ++j)
		{
			printf("%.3f\t", anchor.mPos[j]);
		}
		printf("\n");
	}
}

void cCurve::GetAnchors(int seg, int& anchor_beg, int& anchor_end) const
{
	// compute the indices of the start and end anchors for a given curve segment
	// can be helpful when building the basis matrix
	switch (mCurveType)
	{
	case eCurveTypeCatmullRom:
		anchor_beg = seg - 1;
		anchor_end = anchor_beg + 3;
		break;
	case eCurveTypeBSpline:
		anchor_beg = seg - 2;
		anchor_end = anchor_beg + 3;
		break;
	default:
		assert(false); // unsuppoted curve type
		break;
	}
}

double cCurve::GetAnchorTime(int i) const
{
	// computes the time for a given anchor
	// i.e. roughly the time when a point will be at a particular anchor i
	int num_anchors = GetNumAnchors();
	double time = i / (num_anchors - 1.0);
	time *= GetMaxTime();
	return time;
}

void cCurve::ComputeAnchorTangents()
{
	// computes and stores the tangents at the anchor points
	for (int i = 0; i < GetNumAnchors(); ++i)
	{
		double time = GetAnchorTime(i);
		tAnchor& curr_anchor = mAnchors[i];
		EvalTangent(time, curr_anchor.mTangent);
	}
}

double cCurve::GetSegDuration(int seg) const
{
	// get the duration of each curve segment
	// for now, they are assumed to all have the same duration
	return mSegmentDuration;
}
