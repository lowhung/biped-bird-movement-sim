#pragma once
#include <vector>
#include "Eigen/Dense"
#include "Eigen/StdVector"
#include <json/json.h>
#include "util/MathUtil.h"

class cArticulatedFigure
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tJointDef
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int mParentJoint;
		tVector mAttachPt;
		tVector mLinkAttachPt;
		tVector mLinkSize;
		tVector mCol;

		tJointDef();
		tJointDef(int parent, const tVector& attach_pt, const tVector& link_attach_pt, 
					const tVector& link_size, const tVector& col);
	};

	cArticulatedFigure();
	virtual ~cArticulatedFigure();

	virtual void Init();
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual int GetNumDOFs() const;
	virtual int GetNumJoints() const;

	virtual void Draw();

protected:
	
	Eigen::VectorXd mPose;
	std::vector<tJointDef, Eigen::aligned_allocator<tJointDef>> mJoints;

	virtual int GetJointParamOffset(int joint_id) const;
	virtual int GetJointParamSize(int joint_id) const;
};