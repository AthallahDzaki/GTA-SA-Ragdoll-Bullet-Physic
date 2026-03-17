/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#include "Ragdoll.h"

//#define RIGID 1

namespace
{
	const int kUsedBodyParts[] =
	{
		RagDoll::BODYPART_PELVIS,
		RagDoll::BODYPART_SPINE,
		RagDoll::BODYPART_NECK,
		RagDoll::BODYPART_HEAD,
		RagDoll::BODYPART_LEFT_UPPER_LEG,
		RagDoll::BODYPART_LEFT_LOWER_LEG,
		RagDoll::BODYPART_LEFT_FOOT,
		RagDoll::BODYPART_RIGHT_UPPER_LEG,
		RagDoll::BODYPART_RIGHT_LOWER_LEG,
		RagDoll::BODYPART_RIGHT_FOOT,
		RagDoll::BODYPART_LEFT_UPPER_ARM,
		RagDoll::BODYPART_LEFT_LOWER_ARM,
		RagDoll::BODYPART_LEFT_WRIST,
		RagDoll::BODYPART_LEFT_HAND,
		RagDoll::BODYPART_RIGHT_UPPER_ARM,
		RagDoll::BODYPART_RIGHT_LOWER_ARM,
		RagDoll::BODYPART_RIGHT_WRIST,
		RagDoll::BODYPART_RIGHT_HAND,
	};
}

RagDoll::RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset,
	btScalar scale_ragdoll)	: m_ownerWorld (ownerWorld)
{
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		m_shapes[i] = 0;
		m_bodies[i] = 0;
	}

	for (int i = 0; i < JOINT_COUNT; ++i)
	{
		m_joints[i] = 0;
	}


	// Setup the geometry
	m_shapes[BODYPART_PELVIS] = new btCapsuleShape(
	btScalar(scale_ragdoll*0.15), btScalar(scale_ragdoll*0.20));
	m_shapes[BODYPART_SPINE] = new btCapsuleShape(
	btScalar(scale_ragdoll*0.15), btScalar(scale_ragdoll*0.28));
	m_shapes[BODYPART_NECK] = new btCapsuleShape(btScalar(scale_ragdoll*0.06), btScalar(scale_ragdoll*0.10));
	m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(scale_ragdoll*0.10), btScalar(scale_ragdoll*0.05));
	m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.07), btScalar(scale_ragdoll*0.45));
	m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.37));
	m_shapes[BODYPART_LEFT_FOOT] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.12));
	m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.07), btScalar(scale_ragdoll*0.45));
	m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.37));
	m_shapes[BODYPART_RIGHT_FOOT] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.12));
	m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.33));
	m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.04), btScalar(scale_ragdoll*0.18));
	m_shapes[BODYPART_LEFT_WRIST] = new btCapsuleShape(btScalar(scale_ragdoll*0.03), btScalar(scale_ragdoll*0.10));
	m_shapes[BODYPART_LEFT_HAND] = new btCapsuleShape(btScalar(scale_ragdoll*0.035), btScalar(scale_ragdoll*0.12));
	m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.05), btScalar(scale_ragdoll*0.33));
	m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(scale_ragdoll*0.04), btScalar(scale_ragdoll*0.18));
	m_shapes[BODYPART_RIGHT_WRIST] = new btCapsuleShape(btScalar(scale_ragdoll*0.03), btScalar(scale_ragdoll*0.10));
	m_shapes[BODYPART_RIGHT_HAND] = new btCapsuleShape(btScalar(scale_ragdoll*0.035), btScalar(scale_ragdoll*0.12));

	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.), btScalar(0.)));
	m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.2), btScalar(0.)));
	m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.43), btScalar(0.)));
	m_bodies[BODYPART_NECK] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_NECK]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(scale_ragdoll*1.68), btScalar(0.)));
	m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(0.65*scale_ragdoll),
btScalar(0.)));
	m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(0.2*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(-0.12*scale_ragdoll), btScalar(0.06*scale_ragdoll)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_FOOT] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_FOOT]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(0.65*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(0.2*scale_ragdoll), btScalar(0.)));
	m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(-0.12*scale_ragdoll), btScalar(0.06*scale_ragdoll)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_FOOT] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_FOOT]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.35*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.62*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.80*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_WRIST] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_WRIST]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.95*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
	m_bodies[BODYPART_LEFT_HAND] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_HAND]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.35*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.62*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.80*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_WRIST] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_WRIST]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.95*scale_ragdoll), btScalar(1.45*scale_ragdoll), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-SIMD_HALF_PI);
	m_bodies[BODYPART_RIGHT_HAND] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_HAND]);

	// Setup some damping on the m_bodies
	for (int i = 0; i < (int)(sizeof(kUsedBodyParts) / sizeof(kUsedBodyParts[0])); ++i)
	{
		const int bodyPart = kUsedBodyParts[i];
		m_bodies[bodyPart]->setDamping(0.05f, 0.85f);
		m_bodies[bodyPart]->setDeactivationTime(0.8f);
		m_bodies[bodyPart]->setSleepingThresholds(1.6f, 2.5f);
	}

///////////////////////////// SETTING THE CONSTRAINTS /////////////////////////////////////////////7777
	// Now setup the constraints
	btGeneric6DofConstraint * joint6DOF;
	btTransform localA, localB;
	bool useLinearReferenceFrameA = true;
/// ******* SPINE NECK ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.20*scale_ragdoll), btScalar(0.)));

		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.08*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_NECK], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.2f,-SIMD_EPSILON,-SIMD_PI*0.2f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.3f,SIMD_EPSILON,SIMD_PI*0.2f));
#endif
		m_joints[JOINT_UPPERTORSO_TO_NECK] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_UPPERTORSO_TO_NECK], true);
	}
/// *************************** ///

/// ******* NECK HEAD ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.08*scale_ragdoll), btScalar(0.)));

		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.10*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_NECK], *m_bodies[BODYPART_HEAD], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.5f,SIMD_EPSILON,SIMD_PI*0.3f));
#endif
		m_joints[JOINT_NECK_TO_HEAD] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_NECK_TO_HEAD], true);
	}
/// *************************** ///




/// ******* LEFT SHOULDER ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(-0.17*scale_ragdoll), btScalar(0.10*scale_ragdoll), btScalar(0.)));

		localB.getBasis().setEulerZYX(SIMD_HALF_PI,0,-SIMD_HALF_PI);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.8f,-SIMD_EPSILON,-SIMD_PI*0.5f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.8f,SIMD_EPSILON,SIMD_PI*0.5f));
#endif
		m_joints[JOINT_UPPERTORSO_TO_LEFTUPPERTORSO] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_UPPERTORSO_TO_LEFTUPPERTORSO], true);
	}
/// *************************** ///


/// ******* RIGHT SHOULDER ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.17*scale_ragdoll), btScalar(0.10*scale_ragdoll), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,SIMD_HALF_PI);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else

		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.8f,-SIMD_EPSILON,-SIMD_PI*0.5f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.8f,SIMD_EPSILON,SIMD_PI*0.5f));
#endif
		m_joints[JOINT_UPPERTORSO_TO_RIGHTUPPERTORSO] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_UPPERTORSO_TO_RIGHTUPPERTORSO], true);
	}
/// *************************** ///


/// ******* LEFT ELBOW ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.165*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.09*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_LEFTSHOULDER_TO_LEFTELBOW] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFTSHOULDER_TO_LEFTELBOW], true);
	}
/// *************************** ///

/// ******* LEFT WRIST ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.09*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.05*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_LEFT_LOWER_ARM], *m_bodies[BODYPART_LEFT_WRIST], localA, localB, useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.25f,-SIMD_EPSILON,-SIMD_PI*0.25f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.25f,SIMD_EPSILON,SIMD_PI*0.25f));
#endif
		m_joints[JOINT_LEFTELBOW_TO_LEFTWRIST] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFTELBOW_TO_LEFTWRIST], true);
	}
/// *************************** ///

/// ******* LEFT HAND ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.05*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.06*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_LEFT_WRIST], *m_bodies[BODYPART_LEFT_HAND], localA, localB, useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.3f,SIMD_EPSILON,SIMD_PI*0.3f));
#endif
		m_joints[JOINT_LEFTWRIST_TO_LEFTHAND] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFTWRIST_TO_LEFTHAND], true);
	}
/// *************************** ///

/// ******* RIGHT ELBOW ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.165*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.09*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7,SIMD_EPSILON,SIMD_EPSILON));
#endif

		m_joints[JOINT_RIGHTSHOULDER_TO_RIGHTELBOW] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHTSHOULDER_TO_RIGHTELBOW], true);
	}
/// *************************** ///

/// ******* RIGHT WRIST ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.09*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.05*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_RIGHT_LOWER_ARM], *m_bodies[BODYPART_RIGHT_WRIST], localA, localB, useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.25f,-SIMD_EPSILON,-SIMD_PI*0.25f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.25f,SIMD_EPSILON,SIMD_PI*0.25f));
#endif
		m_joints[JOINT_RIGHTELBOW_TO_RIGHTWRIST] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHTELBOW_TO_RIGHTWRIST], true);
	}
/// *************************** ///

/// ******* RIGHT HAND ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.05*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.06*scale_ragdoll), btScalar(0.)));
		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_RIGHT_WRIST], *m_bodies[BODYPART_RIGHT_HAND], localA, localB, useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.3f,-SIMD_EPSILON,-SIMD_PI*0.3f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.3f,SIMD_EPSILON,SIMD_PI*0.3f));
#endif
		m_joints[JOINT_RIGHTWRIST_TO_RIGHTHAND] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHTWRIST_TO_RIGHTHAND], true);
	}
/// *************************** ///


/// ******* PELVIS ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15*scale_ragdoll), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,SIMD_HALF_PI,0);
		localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.2,-SIMD_EPSILON,-SIMD_PI*0.3));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.2,SIMD_EPSILON,SIMD_PI*0.6));
#endif
		m_joints[JOINT_PELVIS_TO_UPPERTORSO] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_TO_UPPERTORSO], true);
	}
/// *************************** ///

/// ******* LEFT HIP ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(-0.18*scale_ragdoll), btScalar(-0.10*scale_ragdoll), btScalar(0.)));

		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_HALF_PI*0.5,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_HALF_PI*0.8,SIMD_EPSILON,SIMD_HALF_PI*0.6f));
#endif
		m_joints[JOINT_PELVIS_TO_LEFTHIP] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_TO_LEFTHIP], true);
	}
/// *************************** ///


/// ******* RIGHT HIP ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.18*scale_ragdoll), btScalar(-0.10*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225*scale_ragdoll), btScalar(0.)));

		joint6DOF = new btGeneric6DofConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_HALF_PI*0.5,-SIMD_EPSILON,-SIMD_HALF_PI*0.6f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_HALF_PI*0.8,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_PELVIS_TO_RIGHTHIP] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_TO_RIGHTHIP], true);
	}
/// *************************** ///


/// ******* LEFT KNEE ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB,useLinearReferenceFrameA);
//
#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_LEFTHIP_TO_LEFTKNEE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFTHIP_TO_LEFTKNEE], true);
	}
/// *************************** ///

/// ******* RIGHT KNEE ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185*scale_ragdoll), btScalar(0.)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.7f,SIMD_EPSILON,SIMD_EPSILON));
#endif
		m_joints[JOINT_RIGHTHIP_TO_RIGHTKNEE] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHTHIP_TO_RIGHTKNEE], true);
	}
/// *************************** ///

/// ******* LEFT ANKLE/FOOT ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.20*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.08*scale_ragdoll), btScalar(-0.02*scale_ragdoll)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_LEFT_LOWER_LEG], *m_bodies[BODYPART_LEFT_FOOT], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.25f,-SIMD_EPSILON,-SIMD_PI*0.10f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.35f,SIMD_EPSILON,SIMD_PI*0.10f));
#endif
		m_joints[JOINT_LEFTKNEE_TO_LEFTFOOT] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFTKNEE_TO_LEFTFOOT], true);
	}
/// *************************** ///

/// ******* RIGHT ANKLE/FOOT ******** ///
	{
		localA.setIdentity(); localB.setIdentity();

		localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.20*scale_ragdoll), btScalar(0.)));
		localB.setOrigin(btVector3(btScalar(0.), btScalar(0.08*scale_ragdoll), btScalar(-0.02*scale_ragdoll)));
		joint6DOF =  new btGeneric6DofConstraint (*m_bodies[BODYPART_RIGHT_LOWER_LEG], *m_bodies[BODYPART_RIGHT_FOOT], localA, localB,useLinearReferenceFrameA);

#ifdef RIGID
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_EPSILON,-SIMD_EPSILON,-SIMD_EPSILON));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_EPSILON,SIMD_EPSILON,SIMD_EPSILON));
#else
		joint6DOF->setAngularLowerLimit(btVector3(-SIMD_PI*0.25f,-SIMD_EPSILON,-SIMD_PI*0.10f));
		joint6DOF->setAngularUpperLimit(btVector3(SIMD_PI*0.35f,SIMD_EPSILON,SIMD_PI*0.10f));
#endif
		m_joints[JOINT_RIGHTKNEE_TO_RIGHTFOOT] = joint6DOF;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHTKNEE_TO_RIGHTFOOT], true);
	}
/// *************************** ///

}


RagDoll::~RagDoll()
{
	int i;

	// Remove all constraints
	for (i = 0; i < JOINT_COUNT; ++i)
	{
		if (m_joints[i])
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}
	}

	// Remove all bodies and shapes
	for (i = 0; i < (int)(sizeof(kUsedBodyParts) / sizeof(kUsedBodyParts[0])); ++i)
	{
		const int bodyPart = kUsedBodyParts[i];
		if (!m_bodies[bodyPart])
		{
			continue;
		}

		m_ownerWorld->removeRigidBody(m_bodies[bodyPart]);

		delete m_bodies[bodyPart]->getMotionState();

		delete m_bodies[bodyPart]; m_bodies[bodyPart] = 0;
		delete m_shapes[bodyPart]; m_shapes[bodyPart] = 0;
	}
}


btRigidBody* RagDoll::localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}
