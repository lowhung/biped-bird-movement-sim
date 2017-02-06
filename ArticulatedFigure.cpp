#include "ArticulatedFigure.h"
#include "render/DrawUtil.h"

const int gInvalidJoint = -1;
const int gRootDOF = 6;
const int gJointDOF = 3;

cArticulatedFigure::tJointDef::tJointDef()
{
	mParentJoint = gInvalidJoint;
	mAttachPt.setZero();
	mLinkAttachPt.setZero();
	mLinkSize.setZero();
	mCol = tVector(0.5, 0.5, 0.5, 1);
}

cArticulatedFigure::tJointDef::tJointDef(int parent, const tVector& attach_pt, const tVector& link_attach_pt, 
										const tVector& link_size, const tVector& col)
{
	mParentJoint = parent;
	mAttachPt = attach_pt;
	mLinkAttachPt = link_attach_pt;
	mLinkSize = link_size;
	mCol = col;
}

cArticulatedFigure::cArticulatedFigure()
{
}

cArticulatedFigure::~cArticulatedFigure()
{
}

void cArticulatedFigure::Init()
{
	// build articulated figure
	// the character is specified by a kinematic tree
	// each joint specifies:
	//		- the parent
	//		- the attachment point of the joint wrt the parent
	//		- attachment of the link wrt the joint (for rendering)
	//		- size of the link (for rendering)
	//		- color of the link (for rendering)
	// joints should always ordered such that the parent comes before the child
	mJoints.clear();
	// root
	mJoints.push_back(tJointDef(gInvalidJoint, tVector::Zero(), tVector(0, 0.03, 0, 0),
					tVector(0.12, 0.15, 0.27, 0), tVector(0.4706, 0.549, 0.6863, 1)));
	// waist
	mJoints.push_back(tJointDef(0, tVector(0, 0.1, 0, 0), tVector(0, 0.17, 0, 0),
					tVector(0.13, 0.33, 0.29, 0), tVector(0.4706, 0.549, 0.6863, 1)));
	// right hip
	mJoints.push_back(tJointDef(0, tVector(0, 0, 0.085, 0), tVector(0, -0.21075, 0, 0),
					tVector(0.09, 0.4215, 0.09, 0), tVector(0.6392, 0.6941, 0.7372, 1)));
	// right knee
	mJoints.push_back(tJointDef(2, tVector(0, -0.4215, 0, 0), tVector(0, -0.19493, 0, 0),
					tVector(0.07, 0.43, 0.07, 0), tVector(0.6392, 0.6941, 0.7372, 1)));
	// right ankle
	mJoints.push_back(tJointDef(3, tVector(0, -0.40987, 0, 0), tVector(0.0518, -0.0224, 0, 0),
					tVector(0.177, 0.05, 0.09, 0), tVector(0.6392, 0.6941, 0.7372, 1)));
	// left hip
	mJoints.push_back(tJointDef(0, tVector(0, 0, -0.085, 0), tVector(0, -0.21075, 0, 0),
					tVector(0.09, 0.4215, 0.09, 0), tVector(0.3529, 0.41176, 0.47059, 1)));
	// left knee
	mJoints.push_back(tJointDef(5, tVector(0, -0.4215, 0, 0), tVector(0, -0.19493, 0, 0),
					tVector(0.07, 0.43, 0.07, 0), tVector(0.3529, 0.41176, 0.47059, 1)));
	// left ankle
	mJoints.push_back(tJointDef(6, tVector(0, -0.40987, 0, 0), tVector(0.0518, -0.0224, 0, 0),
					tVector(0.177, 0.05, 0.09, 0), tVector(0.3529, 0.41176, 0.47059, 1)));


	mPose = Eigen::VectorXd::Zero(GetNumDOFs());
}

void cArticulatedFigure::SetPose(const Eigen::VectorXd& pose)
{
	// the pose of the character is given as a vector that specifies
	// the orientation of the root and every joint
	// the first 6 numbers specifies the 3D rotation and 3D position of hte root
	// after that, each sequence of 3 numbers will specify the 3D rotation of a joint
	// rotations are in Euler angles specified with order XYZ
	// the rotation transform is computed as rot(Z) * rot(Y) * rot(X)
	assert(pose.size() == GetNumDOFs());
	mPose = pose;
}

int cArticulatedFigure::GetNumDOFs() const
{
	return (GetNumJoints() - 1) * gJointDOF + gRootDOF;
}

int cArticulatedFigure::GetNumJoints() const
{
	return static_cast<int>(mJoints.size());
}

void cArticulatedFigure::Draw()
{
	std::stack<int> joint_stack;

	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const tJointDef& curr_joint = mJoints[j];
		bool is_root = (j == 0);
		int parent_id = curr_joint.mParentJoint;

		while (!joint_stack.empty() && joint_stack.top() != parent_id)
		{
			joint_stack.pop();
			cDrawUtil::PopMatrix();
		}

		joint_stack.push(j);
		if (is_root)
		{
			int param_offset = GetJointParamOffset(j);

			tVector trans = tVector(mPose[param_offset], mPose[param_offset + 1], mPose[param_offset + 2], 0);
			tVector euler = tVector(mPose[param_offset + 3], mPose[param_offset + 4], mPose[param_offset + 5], 0);
			
			cDrawUtil::PushMatrix();
			cDrawUtil::Translate(trans);
			cDrawUtil::Rotate(euler);
			cDrawUtil::Translate(curr_joint.mAttachPt);

			cDrawUtil::SetColor(curr_joint.mCol);
			cDrawUtil::PushMatrix();
			cDrawUtil::DrawBox(curr_joint.mLinkAttachPt, curr_joint.mLinkSize);
			cDrawUtil::PopMatrix();
		}
		else
		{
			int param_offset = GetJointParamOffset(j);

			tVector euler = tVector(mPose[param_offset], mPose[param_offset + 1], mPose[param_offset + 2], 0);

			cDrawUtil::PushMatrix();
			cDrawUtil::Translate(curr_joint.mAttachPt);
			cDrawUtil::Rotate(euler);

			cDrawUtil::SetColor(curr_joint.mCol);
			cDrawUtil::PushMatrix();
			cDrawUtil::DrawBox(curr_joint.mLinkAttachPt, curr_joint.mLinkSize);
			cDrawUtil::PopMatrix();
		}
	}

	while (!joint_stack.empty())
	{
		joint_stack.pop();
		cDrawUtil::PopMatrix();
	}
}

int cArticulatedFigure::GetJointParamOffset(int joint_id) const
{
	bool is_root = joint_id == 0;
	if (is_root)
	{
		return 0;
	}
	return gRootDOF + (joint_id - 1) * gJointDOF;
}

int cArticulatedFigure::GetJointParamSize(int joint_id) const
{
	bool is_root = joint_id == 0;
	if (is_root)
	{
		return gRootDOF;
	}
	return gJointDOF;
}