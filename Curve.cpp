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
	out_result[0] = time; // stub, position just moves along the x-axis with time

	// first build the basis matrix M for the current curve tyle (mCurveType)
	// then build the T polynomial vector
	// finally build the geometry matrix G for the curve segment
}

void cCurve::EvalTangent(double time, Eigen::VectorXd& out_result) const
{
	// TODO (CPSC426): Evaluates the first derivative of a curve
	out_result = Eigen::VectorXd::Zero(GetDim()); // stub
	out_result[0] = 1; // stub, all tangents are horizontal
}

void cCurve::EvalNormal(double time, Eigen::VectorXd& out_result) const
{
	// TODO (CPSC426): Evaluates the second derivative of a curve
	out_result = Eigen::VectorXd::Zero(GetDim()); // stub
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
