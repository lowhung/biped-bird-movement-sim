#pragma once
#include <vector>
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include <json/json.h>

class cCurve
{
public:
	enum eCurveType
	{
		eCurveTypeCatmullRom,
		eCurveTypeBSpline,
		eCurveTypeMax
	};

	struct tAnchor
	{
		tAnchor();
		~tAnchor();

		// anchors are specified by a position and a tangent
		Eigen::VectorXd mPos;
		Eigen::VectorXd mTangent;
	};

	cCurve();
	virtual ~cCurve();

	virtual bool Load(const std::string& file);
	virtual void Clear();
	virtual int GetNumAnchors() const;
	virtual const Eigen::VectorXd& GetAnchorPos(int i) const;
	virtual const Eigen::VectorXd& GetAnchorTangent(int i) const;
	virtual int GetNumSegments() const;
	virtual int GetDim() const;

	virtual void Eval(double time, Eigen::VectorXd& out_result) const;
	virtual void EvalTangent(double time, Eigen::VectorXd& out_result) const;
	virtual void EvalNormal(double time, Eigen::VectorXd& out_result) const;
	virtual double GetMaxTime() const;

	virtual void Add(const tAnchor& anchor);

protected:
	
	static eCurveType ParseCurveType(const std::string& str);

	eCurveType mCurveType;
	double mSegmentDuration; // assume constant duration for each segment

	std::vector<tAnchor, Eigen::aligned_allocator<tAnchor>> mAnchors;

	virtual bool ParseAnchors(const Json::Value& root);
	virtual bool ParseAnchor(const Json::Value& root, tAnchor& out_anchor) const;
	virtual void PrintAnchors() const;
	virtual void GetAnchors(int seg, int& anchor_beg, int& anchor_end) const;
	virtual double GetAnchorTime(int i) const;
	virtual void ComputeAnchorTangents();

	virtual double GetSegDuration(int seg) const;
};
