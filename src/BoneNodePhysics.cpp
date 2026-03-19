#include "BoneNodePhysics.h"
#include "BoneHelper.h"
#include "CPed.h"
#include "CMatrix.h"
#include "rwcore.h"
#include "rphanim.h"
#include "RwBulletBridge.h"
#include "ePedBones.h"
#include "RpHAnimBlendInterpFrame.h"
#include "CSprite.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <string>
#include <d3d9.h>

extern void DebugLog(const std::string& msg);
extern bool g_PhysicsShutdown;   // set in main.cpp before world is deleted

static bool g_DisableBulletToBones = false;
static bool g_DebugTPoseFlying = false;
static constexpr int kInitialTPoseFrames = 30;


// ============================================================
//  Shape + Mass per bone
// ============================================================
static btCollisionShape* MakeShapeForBone(int boneTag, float& massOut) {
    massOut = 5.0f;
    switch (boneTag) {
        // Torso
        case BONE_PELVIS:
        case BONE_PELVIS1:
            massOut = 18.0f; return new btCapsuleShapeZ(0.14f, 0.18f);
        case BONE_SPINE1:
            massOut = 14.0f; return new btCapsuleShapeZ(0.13f, 0.22f);
        case BONE_UPPERTORSO:
            massOut = 18.0f; return new btCapsuleShapeZ(0.16f, 0.28f);

        // Head / Neck
        case BONE_NECK:
            massOut = 3.0f;  return new btCapsuleShapeZ(0.07f, 0.12f);
        case BONE_HEAD:
        case BONE_HEAD1:
        case BONE_HEAD2:
            massOut = 5.0f;  return new btSphereShape(0.11f);

        // Arms
        case BONE_RIGHTUPPERTORSO:
        case BONE_RIGHTSHOULDER:
        case BONE_LEFTUPPERTORSO:
        case BONE_LEFTSHOULDER:
            massOut = 3.0f;  return new btCapsuleShape(0.06f, 0.12f);
        case BONE_RIGHTELBOW:
        case BONE_LEFTELBOW:
            massOut = 3.5f;  return new btCapsuleShape(0.055f, 0.24f);
        case BONE_RIGHTWRIST:
        case BONE_LEFTWRIST:
            massOut = 2.0f;  return new btCapsuleShape(0.045f, 0.20f);
        case BONE_RIGHTHAND:
        case BONE_LEFTHAND:
            massOut = 0.5f;  return new btBoxShape(btVector3(0.07f, 0.035f, 0.10f));

        // Legs
        case BONE_RIGHTHIP:
        case BONE_LEFTHIP:
            massOut = 10.0f; return new btCapsuleShape(0.08f, 0.38f);
        case BONE_RIGHTKNEE:
        case BONE_LEFTKNEE:
            massOut = 6.0f;  return new btCapsuleShape(0.07f, 0.35f);
        case BONE_RIGHTANKLE:
        case BONE_LEFTANKLE:
            massOut = 2.0f;  return new btCapsuleShape(0.05f, 0.12f);
        case BONE_RIGHTFOOT:
        case BONE_LEFTFOOT:
            massOut = 1.5f;  return new btBoxShape(btVector3(0.09f, 0.045f, 0.13f));

        default:
            std::cout << "Failed to create shape for bone tag " << boneTag << std::endl; return nullptr;

    }
}

// ============================================================
//  Destructor — do NOT touch s_DynamicsWorld here (may be gone)
//  World removal is done in DeactivatePhysicsForPed.
// ============================================================
BonePhysicsData::~BonePhysicsData() {
    // During shutdown, Bullet's internal allocators may already be freed.
    // Deleting bodies/shapes then would corrupt the heap.
    // DeactivatePhysicsForPed already removed everything from the world;
    // we only need to free the raw memory here.
    if (g_PhysicsShutdown) {
        // Still delete — but only if pointers look valid (not already freed)
        // We rely on DeactivatePhysicsForPed having been called first.
        rigidBody   = nullptr;
        motionState = nullptr;
        shape       = nullptr;
        constraint  = nullptr;
        return;
    }
    if (constraint)  { delete constraint;  constraint  = nullptr; }
    if (rigidBody)   { delete rigidBody;   rigidBody   = nullptr; }
    if (motionState) { delete motionState; motionState = nullptr; }
    if (shape)       { delete shape;       shape       = nullptr; }
}

// ============================================================
//  Initialize / Shutdown
// ============================================================
void BoneNodePhysics::Initialize(btDiscreteDynamicsWorld* world) {
    s_DynamicsWorld = world;
    DebugLog("BoneNodePhysics initialized");
}

void BoneNodePhysics::Shutdown() {
    DebugLog("BoneNodePhysics::Shutdown start");

    // Deactivate all peds (removes from world safely)
    // Collect keys first to avoid iterator invalidation
    std::vector<CPed*> peds;
    for (auto& [ped, _] : s_PedBones) peds.push_back(ped);
    for (CPed* ped : peds) DeactivatePhysicsForPed(ped);

    s_PedBones.clear();
    s_PedHierarchies.clear();
    s_DynamicsWorld = nullptr;
    DebugLog("BoneNodePhysics::Shutdown done");
}

void BoneNodePhysics::CalculateBulletPhysics() {
    for (auto& [ped, bones] : s_PedBones) {
        if (!ped) continue;

        // auto tPoseIt = g_InitialTPoseFrameLeft.find(ped);
        // if (tPoseIt != g_InitialTPoseFrameLeft.end() && tPoseIt->second > 0) {
        //     --tPoseIt->second;

        //     for (auto& b : bones) {
        //         if (!b->isActive || !b->rigidBody) continue;
        //         b->rigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
        //         b->rigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
        //         b->rigidBody->clearForces();
        //     }

        //     if (tPoseIt->second == 0) {
        for (auto& b : bones) {
            if (!b->isActive || !b->rigidBody) continue;
            const int flags = b->rigidBody->getCollisionFlags();
            b->rigidBody->setCollisionFlags(flags & ~btCollisionObject::CF_KINEMATIC_OBJECT);
            if (s_DynamicsWorld) {
                b->rigidBody->setGravity(s_DynamicsWorld->getGravity());
            }
            b->rigidBody->activate(true);
            b->rigidBody->setActivationState(ACTIVE_TAG);
        }
        //     }

        //     continue;
        // }

        // Helper: find a BonePhysicsData by tag
        auto findBone = [&](int tag) -> BonePhysicsData* {
            for (auto& b : bones)
                if (b->boneTag == tag) return b.get();
            return nullptr;
        };

        // ---- 0. Clamp bone velocities ----
        const float MAX_BONE_SPEED    = 20.0f;
        const float MAX_BONE_SPEED_SQ = MAX_BONE_SPEED * MAX_BONE_SPEED;
        for (auto& b : bones) {
            if (!b->isActive || !b->rigidBody) continue;
            btVector3 v = b->rigidBody->getLinearVelocity();
            if (v.length2() > MAX_BONE_SPEED_SQ)
                b->rigidBody->setLinearVelocity(v.normalized() * MAX_BONE_SPEED);
        }

        // ---- 1. Move ped entity to pelvis Bullet position ----
        auto* pelvis = findBone(BONE_PELVIS);
        if (!pelvis || !pelvis->rigidBody || !pelvis->isActive) continue;

        btVector3 pelvisPos = pelvis->rigidBody->getWorldTransform().getOrigin();
        CVector newPos(pelvisPos.x(), pelvisPos.y(), pelvisPos.z());
        ped->SetPosn(newPos);

        // CMatrix* pedMat = ped->GetMatrix();
        // if (pedMat) {
        //     pedMat->GetPosition() = newPos;
        //     pedMat->UpdateRW();
        // }
        ped->m_vecMoveSpeed = CVector(0, 0, 0);
        ped->m_vecTurnSpeed = CVector(0, 0, 0);
    }
}

// ============================================================
//  CreatePhysicsForPedBone
//  Places the rigid body at the exact world position of the bone.
// ============================================================
bool BoneNodePhysics::CreatePhysicsForPedBone(CPed* ped, int boneTag, int boneIndex, int parentTag) {
    if (!s_DynamicsWorld || !ped) return false;

    // We need the hierarchy to place the body at the right world position
    auto hierIt = s_PedHierarchies.find(ped);
    if (hierIt == s_PedHierarchies.end()) return false;
    RpHAnimHierarchy* hier = hierIt->second;

    // Get world matrix of this bone
    RwMatrix* boneMtx = &RpHAnimHierarchyGetMatrixArray(hier)[boneIndex];
    if (!boneMtx) return false;

    float mass = 0.0f;
    btCollisionShape* shape = MakeShapeForBone(boneTag, mass);
    if (!shape) return false;

    // Build initial transform from bone world matrix
    // (hierarchy matrices are already in world space after UpdateMatrices)
    btTransform startTF = RwMatrixToBtTransform(*boneMtx);

    btVector3 inertia(0, 0, 0);
    if (mass > 0.0f) shape->calculateLocalInertia(mass, inertia);

    // Use motion state so Bullet knows the initial transform
    auto* ms = new btDefaultMotionState(startTF);

    btRigidBody::btRigidBodyConstructionInfo ci(mass, ms, shape, inertia);
    ci.m_linearDamping  = 0.4f;    // increased from 0.05 to prevent runaway velocity
    ci.m_angularDamping = 0.85f;
    ci.m_friction       = 0.75f;
    ci.m_restitution    = 0.05f;

    auto data             = std::make_unique<BonePhysicsData>();
    data->ownerPed        = ped;
    data->boneTag         = boneTag;
    data->boneIndex       = boneIndex;
    data->parentTag       = parentTag;
    data->shape           = shape;
    data->motionState     = ms;
    data->rigidBody       = new btRigidBody(ci);
    data->rigidBody->setActivationState(WANTS_DEACTIVATION);

    // Enable CCD to prevent tunneling through thin collision geometry
    data->rigidBody->setCcdMotionThreshold(0.1f);
    data->rigidBody->setCcdSweptSphereRadius(0.05f);

    // Ragdoll bones collide with world but NOT with each other (avoids internal collisions)
    short group = static_cast<short>(1 << 8);   // ragdoll layer
    short mask  = static_cast<short>(~group);    // collide with everything except ragdoll
    s_DynamicsWorld->addRigidBody(data->rigidBody, group, mask);

    s_PedBones[ped].push_back(std::move(data));
    return true;
}

// ============================================================
//  CreateConstraintsForPed
//  Uses CURRENT world positions to compute pivot offsets so
//  that joints are EXACTLY at the anatomical connection point.
// ============================================================
void BoneNodePhysics::CreateConstraintsForPed(CPed* ped) {
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;
    auto& bones = it->second;

    auto findBone = [&](int tag) -> BonePhysicsData* {
        for (auto& b : bones)
            if (b->boneTag == tag) return b.get();
        return nullptr;
    };

    // OpenGL demo body-part mapping -> GTA SA bones

    // Special Bone
    auto rightUpperTorso = findBone(BONE_RIGHTUPPERTORSO); // extra GTA upper torso bone (for better shoulder joints)
    auto leftUpperTorso  = findBone(BONE_LEFTUPPERTORSO); // extra GTA upper torso bone (for better shoulder joints)

    auto* pelvis1        = findBone(BONE_PELVIS1);        // extra GTA pelvis root
    auto* pelvis         = findBone(BONE_PELVIS);         // BODYPART_PELVIS
    auto* spine1         = findBone(BONE_SPINE1);         // extra GTA spine bone
    auto* neck           = findBone(BONE_UPPERTORSO);     // BODYPART_UPPER_TORSO (UPPERTORSO WAS A NECK BONE IN GTA, SO WE USE THE SPINE1 BONE FOR THE UPPER TORSO JOINT)
    auto* head           = findBone(BONE_NECK);           // BODYPART_NECK (NECK WAS A HEAD BONE IN GTA, SO WE USE THE HEAD BONE FOR THE NECK JOINT)
    //auto* head           = findBone(BONE_HEAD);         // BODYPART_HEAD (HEAD WAS A JAW BONE IN GTA, SO WE USE THE NECK BONE FOR THE HEAD JOINT)
    auto* leftShoulder   = findBone(BONE_LEFTSHOULDER);   // BODYPART_LEFT_UPPER_ARM
    auto* leftElbow      = findBone(BONE_LEFTELBOW);      // BODYPART_LEFT_LOWER_ARM
    auto* leftWrist      = findBone(BONE_LEFTWRIST);      // BODYPART_LEFT_WRIST
    auto* leftHand       = findBone(BONE_LEFTHAND);       // BODYPART_LEFT_HAND
    auto* rightShoulder  = findBone(BONE_RIGHTSHOULDER);  // BODYPART_RIGHT_UPPER_ARM
    auto* rightElbow     = findBone(BONE_RIGHTELBOW);     // BODYPART_RIGHT_LOWER_ARM
    auto* rightWrist     = findBone(BONE_RIGHTWRIST);     // BODYPART_RIGHT_WRIST
    auto* rightHand      = findBone(BONE_RIGHTHAND);      // BODYPART_RIGHT_HAND
    auto* leftHip        = findBone(BONE_LEFTHIP);        // BODYPART_LEFT_UPPER_LEG
    auto* leftKnee       = findBone(BONE_LEFTKNEE);       // BODYPART_LEFT_LOWER_LEG
    auto* leftAnkle      = findBone(BONE_LEFTANKLE);      // BODYPART_LEFT_FOOT (GTA has ankle)
    auto* leftFoot       = findBone(BONE_LEFTFOOT);       // foot
    auto* rightHip       = findBone(BONE_RIGHTHIP);       // BODYPART_RIGHT_UPPER_LEG
    auto* rightKnee      = findBone(BONE_RIGHTKNEE);      // BODYPART_RIGHT_LOWER_LEG
    auto* rightAnkle     = findBone(BONE_RIGHTANKLE);     // BODYPART_RIGHT_FOOT (GTA has ankle)
    auto* rightFoot      = findBone(BONE_RIGHTFOOT);      // foot

    if (!pelvis1 || !pelvis || !spine1 || !head) {
        DebugLog("CreateConstraintsForPed: missing mapped GTA bones (pelvis1/pelvis/spine/head)");
        return;
    }

    btScalar scale = 1.0f;
    BonePhysicsData* scaleSpine = spine1;
    if (pelvis1->rigidBody && scaleSpine && scaleSpine->rigidBody) {
        btVector3 p = pelvis1->rigidBody->getWorldTransform().getOrigin();
        btVector3 s = scaleSpine->rigidBody->getWorldTransform().getOrigin();
        btScalar dist = (s - p).length();
        if (dist > SIMD_EPSILON) {
            scale = btClamped(dist / btScalar(0.20f), btScalar(0.6f), btScalar(2.5f));
        }
    }

    auto addJoint = [&](BonePhysicsData* a, BonePhysicsData* b,
                        const btVector3& aOrigin, const btVector3& bOrigin,
                        const btVector3& aEulerZYX, const btVector3& bEulerZYX,
                        const btVector3& angularLower, const btVector3& angularUpper) -> bool {
        if (!a || !b || !a->rigidBody || !b->rigidBody) return false;

        btTransform localA, localB;
        localA.setIdentity();
        localB.setIdentity();

        localA.getBasis().setEulerZYX(aEulerZYX.x(), aEulerZYX.y(), aEulerZYX.z());
        localB.getBasis().setEulerZYX(bEulerZYX.x(), bEulerZYX.y(), bEulerZYX.z());

        localA.setOrigin(aOrigin * scale);
        localB.setOrigin(bOrigin * scale);

        auto* joint6DOF = new btGeneric6DofConstraint(
            *a->rigidBody, *b->rigidBody, localA, localB, true);

        joint6DOF->setLinearLowerLimit(btVector3(0.0f, 0.0f, 0.0f));
        joint6DOF->setLinearUpperLimit(btVector3(0.0f, 0.0f, 0.0f));
        joint6DOF->setAngularLowerLimit(angularLower);
        joint6DOF->setAngularUpperLimit(angularUpper);

        s_DynamicsWorld->addConstraint(joint6DOF, true);
        b->constraint = joint6DOF;
        return true;
    };

    constexpr btScalar eps = SIMD_EPSILON;
    int created = 0;

    // PELVIS1 -> PELVIS (GTA extra pelvis bone)
    if (pelvis1 && addJoint(pelvis1, pelvis,
                 btVector3(0.0f, 0.12f, 0.0f),
                 btVector3(0.0f,-0.12f, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(-SIMD_PI * 0.2f, -eps, -SIMD_PI * 0.3f),
                 btVector3( SIMD_PI * 0.2f,  eps,  SIMD_PI * 0.6f))) {
        ++created;
    }

    // PELVIS -> SPINE1 (GTA extra spine bone)
    if (spine1 && addJoint(pelvis, spine1,
                 btVector3(0.0f, 0.12f, 0.0f),
                 btVector3(0.0f,-0.12f, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(-SIMD_PI * 0.2f, -eps, -SIMD_PI * 0.3f),
                 btVector3( SIMD_PI * 0.2f,  eps,  SIMD_PI * 0.6f))) {
        ++created;
    }

    // SPINE1 -> UPPER TORSO
    if (spine1 && neck && addJoint(spine1, neck,
                 btVector3(0.0f, 0.12f, 0.0f),
                 btVector3(0.0f,-0.12f, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(0.0f, SIMD_HALF_PI, 0.0f),
                 btVector3(-SIMD_PI * 0.2f, -eps, -SIMD_PI * 0.3f),
                 btVector3( SIMD_PI * 0.2f,  eps,  SIMD_PI * 0.6f))) {
        ++created;
    }

    // // UPPER TORSO -> NECK
    // if (upperTorso && neck && addJoint(upperTorso, neck,
    //              btVector3(0.0f, 0.20f, 0.0f),
    //              btVector3(0.0f,-0.08f, 0.0f),
    //              btVector3(0.0f, 0.0f, 0.0f),
    //              btVector3(0.0f, 0.0f, 0.0f),
    //              btVector3(-SIMD_PI * 0.2f, -eps, -SIMD_PI * 0.2f),
    //              btVector3( SIMD_PI * 0.3f,  eps,  SIMD_PI * 0.2f))) {
    //     ++created;
    // }

    // NECK -> HEAD (forehead)
    if (neck && head && addJoint(neck, head,
                 btVector3(0.0f, 0.08f, 0.0f),
                 btVector3(0.0f,-0.06f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.25f, -eps, -SIMD_PI * 0.25f),
                 btVector3( SIMD_PI * 0.35f,  eps,  SIMD_PI * 0.25f))) {
        ++created;
    }

    // LEFT SHOULDER -> UPPER TORSO (GTA extra upper torso bone for better shoulder joint) This joint exist in GTA Skeleton and follow UPPERTORSO movement, but it is not parent of SHOULDER bone, so we can use it to create more natural shoulder joint without affecting torso movement.
    if (leftShoulder && addJoint(leftShoulder, neck,
                 btVector3(-0.2f, 0.15f, 0.0f),
                 btVector3( 0.0f,-0.18f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(SIMD_HALF_PI, 0.0f, -SIMD_HALF_PI),
                 btVector3(-SIMD_PI * 0.35f, -eps, -SIMD_PI * 0.25f),
                 btVector3( SIMD_PI * 0.35f,  eps,  SIMD_PI * 0.25f))) {
        ++created;
    }

    // LEFT UPPER TORSO -> LEFT SHOULDER (matches demo shoulder joint behavior).
    if (leftUpperTorso && leftShoulder && addJoint(leftUpperTorso, leftShoulder,
                 btVector3(0.0f, 0.10f, 0.0f),
                 btVector3(0.0f,-0.18f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(SIMD_HALF_PI, 0.0f, -SIMD_HALF_PI),
                 btVector3(-SIMD_PI * 0.8f, -eps, -SIMD_PI * 0.5f),
                 btVector3( SIMD_PI * 0.8f,  eps,  SIMD_PI * 0.5f))) {
        ++created;
    }

    // UPPER TORSO -> RIGHT UPPER TORSO (clavicle): keep clavicle attached to torso.
    if (neck && rightUpperTorso && addJoint(neck, rightUpperTorso,
                 btVector3( 0.17f, 0.10f, 0.0f),
                 btVector3( 0.0f,-0.08f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, SIMD_HALF_PI),
                 btVector3(-SIMD_PI * 0.35f, -eps, -SIMD_PI * 0.25f),
                 btVector3( SIMD_PI * 0.35f,  eps,  SIMD_PI * 0.25f))) {
        ++created;
    }

    // RIGHT UPPER TORSO -> RIGHT SHOULDER (matches demo shoulder joint behavior).
    if (rightUpperTorso && rightShoulder && addJoint(rightUpperTorso, rightShoulder,
                 btVector3(0.0f, 0.10f, 0.0f),
                 btVector3(0.0f,-0.18f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, SIMD_HALF_PI),
                 btVector3(-SIMD_PI * 0.8f, -eps, -SIMD_PI * 0.5f),
                 btVector3( SIMD_PI * 0.8f,  eps,  SIMD_PI * 0.5f))) {
        ++created;
    }

    // LEFT ELBOW
    if (addJoint(leftShoulder, leftElbow,
                 btVector3(0.0f, 0.18f, 0.0f),
                 btVector3(0.0f,-0.14f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-eps, -eps, -eps),
                 btVector3(SIMD_PI * 0.7f, eps, eps))) {
        ++created;
    }

    // RIGHT ELBOW
    if (addJoint(rightShoulder, rightElbow,
                 btVector3(0.0f, 0.18f, 0.0f),
                 btVector3(0.0f,-0.14f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-eps, -eps, -eps),
                 btVector3(SIMD_PI * 0.7f, eps, eps))) {
        ++created;
    }

    // LEFT WRIST
    if (leftElbow && leftWrist && addJoint(leftElbow, leftWrist,
                 btVector3(0.0f, 0.09f, 0.0f),
                 btVector3(0.0f,-0.05f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.25f, -eps, -SIMD_PI * 0.25f),
                 btVector3( SIMD_PI * 0.25f,  eps,  SIMD_PI * 0.25f))) {
        ++created;
    }

    // RIGHT WRIST
    if (rightElbow && rightWrist && addJoint(rightElbow, rightWrist,
                 btVector3(0.0f, 0.09f, 0.0f),
                 btVector3(0.0f,-0.05f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.25f, -eps, -SIMD_PI * 0.25f),
                 btVector3( SIMD_PI * 0.25f,  eps,  SIMD_PI * 0.25f))) {
        ++created;
    }

    // LEFT HAND
    if (leftWrist && leftHand && addJoint(leftWrist, leftHand,
                 btVector3(0.0f, 0.05f, 0.0f),
                 btVector3(0.0f,-0.06f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.3f, -eps, -SIMD_PI * 0.3f),
                 btVector3( SIMD_PI * 0.3f,  eps,  SIMD_PI * 0.3f))) {
        ++created;
    }

    // RIGHT HAND
    if (rightWrist && rightHand && addJoint(rightWrist, rightHand,
                 btVector3(0.0f, 0.05f, 0.0f),
                 btVector3(0.0f,-0.06f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.3f, -eps, -SIMD_PI * 0.3f),
                 btVector3( SIMD_PI * 0.3f,  eps,  SIMD_PI * 0.3f))) {
        ++created;
    }

    // LEFT HIP
    if (addJoint(pelvis, leftHip,
                 btVector3(-0.18f,-0.10f, 0.0f),
                 btVector3( 0.0f, 0.225f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_HALF_PI * 0.5f, -eps, -eps),
                 btVector3( SIMD_HALF_PI * 0.8f,  eps,  SIMD_HALF_PI * 0.6f))) {
        ++created;
    }

    // RIGHT HIP
    if (addJoint(pelvis, rightHip,
                 btVector3( 0.18f,-0.10f, 0.0f),
                 btVector3( 0.0f, 0.225f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_HALF_PI * 0.5f, -eps, -SIMD_HALF_PI * 0.6f),
                 btVector3( SIMD_HALF_PI * 0.8f,  eps,  eps))) {
        ++created;
    }

    // LEFT KNEE
    if (addJoint(leftHip, leftKnee,
                 btVector3(0.0f,-0.225f,0.0f),
                 btVector3(0.0f, 0.185f,0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-eps, -eps, -eps),
                 btVector3(SIMD_PI * 0.7f, eps, eps))) {
        ++created;
    }

    // RIGHT KNEE
    if (addJoint(rightHip, rightKnee,
                 btVector3(0.0f,-0.225f,0.0f),
                 btVector3(0.0f, 0.185f,0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-eps, -eps, -eps),
                 btVector3(SIMD_PI * 0.7f, eps, eps))) {
        ++created;
    }

    // LEFT ANKLE
    if (leftKnee && leftAnkle && addJoint(leftKnee, leftAnkle,
                 btVector3(0.0f,-0.20f, 0.0f),
                 btVector3(0.0f, 0.08f,-0.02f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.25f, -eps, -SIMD_PI * 0.10f),
                 btVector3( SIMD_PI * 0.35f,  eps,  SIMD_PI * 0.10f))) {
        ++created;
    }

    // RIGHT ANKLE
    if (rightKnee && rightAnkle && addJoint(rightKnee, rightAnkle,
                 btVector3(0.0f,-0.20f, 0.0f),
                 btVector3(0.0f, 0.08f,-0.02f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.25f, -eps, -SIMD_PI * 0.10f),
                 btVector3( SIMD_PI * 0.35f,  eps,  SIMD_PI * 0.10f))) {
        ++created;
    }

    // LEFT FOOT
    if (leftAnkle && leftFoot && addJoint(leftAnkle, leftFoot,
                 btVector3(0.0f,-0.06f, 0.0f),
                 btVector3(0.0f, 0.05f,-0.02f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.15f, -eps, -SIMD_PI * 0.10f),
                 btVector3( SIMD_PI * 0.15f,  eps,  SIMD_PI * 0.10f))) {
        ++created;
    }

    // RIGHT FOOT
    if (rightAnkle && rightFoot && addJoint(rightAnkle, rightFoot,
                 btVector3(0.0f,-0.06f, 0.0f),
                 btVector3(0.0f, 0.05f,-0.02f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(0.0f, 0.0f, 0.0f),
                 btVector3(-SIMD_PI * 0.15f, -eps, -SIMD_PI * 0.10f),
                 btVector3( SIMD_PI * 0.15f,  eps,  SIMD_PI * 0.10f))) {
        ++created;
    }

    char buf[160];
    sprintf_s(buf, "Created %d fake 6DoF constraints (OpenGL demo style), scale=%.2f", created, (float)scale);
    DebugLog(buf);
}

// ============================================================
//  ActivatePhysicsForPed
// ============================================================
void BoneNodePhysics::ActivatePhysicsForPed(CPed* ped) {
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;

    int activated = 0;
    for (auto& b : it->second) {
        b->isActive = true;
        if (b->rigidBody) {
            b->rigidBody->activate(true);
            b->rigidBody->setActivationState(DISABLE_DEACTIVATION);
            b->rigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
            b->rigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
            b->rigidBody->clearForces();

            // Hold in TPose at spawn: keep bones kinematic for a short duration.
            const int flags = b->rigidBody->getCollisionFlags();
            b->rigidBody->setCollisionFlags(flags | btCollisionObject::CF_KINEMATIC_OBJECT);
            b->rigidBody->setGravity(btVector3(0.0f, 0.0f, 0.0f));
        }
        ++activated;
    }

    char buf[128];
    sprintf_s(buf, "Activated %d bones", activated);
    DebugLog(buf);
}

// ============================================================
//  DeactivatePhysicsForPed
//  SAFELY removes constraints then bodies from world BEFORE delete.
// ============================================================
void BoneNodePhysics::DeactivatePhysicsForPed(CPed* ped) {
    DebugLog("Deactivate Physic START");
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;
    DebugLog("Found Ped");

    if (s_DynamicsWorld) {
        // Remove constraints first (constraint references bodies)
        for (auto& b : it->second) {
            if (b->constraint) {
                s_DynamicsWorld->removeConstraint(b->constraint);
            }
        }
        // Then remove bodies
        for (auto& b : it->second) {
            if (b->rigidBody) {
                b->rigidBody->setActivationState(WANTS_DEACTIVATION);
                s_DynamicsWorld->removeRigidBody(b->rigidBody);
            }
        }
    }

    // unique_ptr destructors now safely delete the objects
    s_PedBones.erase(it);
    s_PedHierarchies.erase(ped);
}

// ============================================================
//  SyncAllToBullet
//  Read GTA bone world matrices and push them into Bullet.
//  Called ONCE when ragdoll is created (before activation).
//  Also captures initial quaternions for delta-rotation sync.
// ============================================================
void BoneNodePhysics::SyncAllToBullet(CPed* ped) {
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;

    auto hierIt = s_PedHierarchies.find(ped);
    if (hierIt == s_PedHierarchies.end()) return;
    RpHAnimHierarchy* hier = hierIt->second;

    // // Ensure hierarchy matrices are up to date
    // RpHAnimHierarchySetFlags(hier,
    //     (RpHAnimHierarchyFlag)(RpHAnimHierarchyGetFlags(hier)
    //         | rpHANIMHIERARCHYUPDATELTMS
    //         | rpHANIMHIERARCHYUPDATEMODELLINGMATRICES));
    // RpHAnimHierarchyUpdateMatrices(hier);

    RwMatrix* matrices = RpHAnimHierarchyGetMatrixArray(hier);

    // Helper: find a BonePhysicsData by tag
    auto findBone = [&](int tag) -> BonePhysicsData* {
        for (auto& b : it->second)
            if (b->boneTag == tag) return b.get();
        return nullptr;
    };

    auto* clumpData = RpClumpGetAnimBlendClumpData(ped->m_pRwClump);

    for (auto& b : it->second) {
        if (!b->rigidBody) continue;

        RwMatrix* boneMtx = &matrices[b->boneIndex];
        btTransform tf = RwMatrixToBtTransform(*boneMtx);

        b->rigidBody->setWorldTransform(tf);
        b->rigidBody->setInterpolationWorldTransform(tf);
        if (b->motionState) b->motionState->setWorldTransform(tf);

        b->rigidBody->setLinearVelocity(btVector3(0, 0, 0));
        b->rigidBody->setAngularVelocity(btVector3(0, 0, 0));

        // ---- Capture initial Bullet local rotation ----
        if (b->parentTag >= 0) {
            auto* parentData = findBone(b->parentTag);
            if (parentData && parentData->rigidBody) {
                btTransform parentTF = parentData->rigidBody->getWorldTransform();
                btTransform localTF  = parentTF.inverse() * tf;
                b->initBulletLocalQuat = localTF.getRotation();
            }
            else {
                b->initBulletLocalQuat = tf.getRotation();
            }
        }
        else {
            b->initBulletLocalQuat = tf.getRotation();
        }
        b->initBulletLocalQuat.normalize();

        // ---- Capture initial GTA interp frame quaternion ----
        b->initGtaQuat = btQuaternion::getIdentity();
        if (clumpData && b->boneIndex >= 0) {
            auto* frameData = &clumpData->m_pFrames[b->boneIndex];
            if (frameData && frameData->m_pIFrame) {
                auto* frame = reinterpret_cast<RpHAnimBlendInterpFrame*>(frameData->m_pIFrame);
                b->initGtaQuat = RtQuatToBtQuat(frame->orientation);
            }
        }
    }

    DebugLog("SyncAllToBullet: pushed " + std::to_string(it->second.size()) + " bones (with init quats)");
}

void BoneNodePhysics::SyncBulletToBoneHelperRender(CPed* ped) {
    if (!ped) return;
    if (g_DisableBulletToBones) return;

    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;

    auto findBoneData = [&](int tag) -> BonePhysicsData* {
        for (auto& b : it->second) {
            if (b->boneTag == tag) return b.get();
        }
        return nullptr;
    };

    auto* clumpData = RpClumpGetAnimBlendClumpData(ped->m_pRwClump);

    auto applyBulletToBone = [&](int tag, bool applyRotation) {
        auto* b = findBoneData(tag);
        if (!b || !b->isActive || !b->rigidBody) return;

        const btTransform worldTF = b->rigidBody->getWorldTransform();
        const btVector3 btPos = worldTF.getOrigin();
        const btQuaternion btQuat = worldTF.getRotation();

        // Semua Rotasi GTA Itu 0, 0, 0 Di Frame Awal (Pose T), Jadi Kita Hitung Delta Rotasi Dari Pose Tersebut, Lalu Terapkan Delta Itu Ke Pose GTA Awal, Baru Kita Dapat Rotasi GTA Yang Benar Sesuai Rotasi Bullet Saat Ini. Dengan Cara Ini, Kita Bisa Mempertahankan Pose Awal GTA Sambil Tetap Sinkron Dengan Rotasi Bullet.
        
        // if (b->hasInitPose && clumpData && b->boneIndex >= 0) {
        //     btQuaternion bulletLocalQuat = btQuat;
        //     if (b->parentTag >= 0) {
        //         auto* parentData = findBoneData(b->parentTag);
        //         if (parentData && parentData->isActive && parentData->rigidBody) {
        //             const btQuaternion parentWorldQuat = parentData->rigidBody->getWorldTransform().getRotation();
        //             bulletLocalQuat = parentWorldQuat.inverse() * btQuat;
        //         }
        //     }
        //     bulletLocalQuat.normalize();

        //     btQuaternion deltaQuat = bulletLocalQuat * b->initBulletLocalQuat.inverse();
        //     deltaQuat.normalize();

        //     btQuaternion correctedGtaQuat = deltaQuat * b->initGtaQuat;
        //     correctedGtaQuat.normalize();

        //     auto* frameData = &clumpData->m_pFrames[b->boneIndex];
        //     if (frameData && frameData->m_pIFrame) {
        //         auto* frame = reinterpret_cast<RpHAnimBlendInterpFrame*>(frameData->m_pIFrame);
        //         frame->orientation = BtQuatToRtQuat(correctedGtaQuat);
        //     }
        // }

        //BoneHelper::SetBoneRWMatrix(ped, tag, boneMtx);
        BoneHelper::SetBonePosition(ped, tag, { btPos.x(), btPos.y(), btPos.z() });

        // std::cout << "Bone Matrix Data for tag " << tag << ": " << std::endl;
        // std::cout << "BoneMTX : " << boneMtx.right.x << ", " << boneMtx.right.y << ", " << boneMtx.right.z << std::endl;
        // std::cout << "          " << boneMtx.up.x << ", " << boneMtx.up.y << ", " << boneMtx.up.z << std::endl;
        // std::cout << "          " << boneMtx.at.x << ", " << boneMtx.at.y << ", " << boneMtx.at.z << std::endl;
        // std::cout << "          " << boneMtx.pos.x << ", " << boneMtx.pos.y << ", " << boneMtx.pos.z << std::endl;

        RwV3d bonePos = { btPos.x(), btPos.y(), btPos.z() };
        BoneHelper::SetBonePosition(ped, tag, bonePos);

        // RtQuat rtQuat;
        // rtQuat.imag.x = btQuat.x();
        // rtQuat.imag.y = btQuat.y();
        // rtQuat.imag.z = btQuat.z();
        // rtQuat.real   = btQuat.w();

        // RwV3d angles = {0.0f, 0.0f, 0.0f};
        // BoneHelper::QuatToEuler(&rtQuat, &angles);
        // if (applyRotation) {
        //     BoneHelper::SetBoneRotation(ped, tag, angles);
        // }

        // Head group mapping: apply HEAD to HEAD1/HEAD2 only.
        if (tag == BONE_NECK) {
            BoneHelper::SetBonePosition(ped, BONE_HEAD, bonePos);
            BoneHelper::SetBonePosition(ped, BONE_HEAD1, bonePos);
            BoneHelper::SetBonePosition(ped, BONE_HEAD2, bonePos);
        }

        if (tag == BONE_PELVIS) {
            bonePos.y += 0.12f; // GTA has an extra pelvis bone (PELVIS1) above the main pelvis, so we need to offset the position up to match that.
            BoneHelper::SetBonePosition(ped, BONE_PELVIS1, bonePos);
        }

        if (tag == BONE_LEFTHAND) {
            BoneHelper::SetBonePosition(ped, BONE_LEFTTHUMB, bonePos);
            BoneHelper::SetBoneRotation(ped, BONE_LEFTTHUMB, {0.0f, 0.0f, 0.0f}); // Reset GTA Euler overrides for thumb (prevents weird twisting). We will still apply Bullet rotation via RWMatrix, so the thumb will follow the hand properly, just without the extra GTA Euler rotation.
        }

        if (tag == BONE_RIGHTHAND) {
            BoneHelper::SetBonePosition(ped, BONE_RIGHTTHUMB, bonePos);
            BoneHelper::SetBoneRotation(ped, BONE_RIGHTTHUMB, {0.0f, 0.0f, 0.0f}); // Reset GTA Euler overrides for thumb (prevents weird twisting). We will still apply Bullet rotation via RWMatrix, so the thumb will follow the hand properly, just without the extra GTA Euler rotation.
        }
    };

    if (g_DebugTPoseFlying) {
        // During debug/initial TPose window, use Bullet as the single source of truth.
        // Do not inject manual GTA Euler overrides here.
        for (auto& b : it->second) {
            applyBulletToBone(b->boneTag, true);
        }
        return;
    }

    for (auto& b : it->second) {
        applyBulletToBone(b->boneTag, true);
    }
}

// ============================================================
//  RegisterHierarchy
// ============================================================
void BoneNodePhysics::RegisterHierarchy(CPed* ped, RpHAnimHierarchy* hier) {
    s_PedHierarchies[ped] = hier;
}

// ============================================================
//  GetBoneRigidBody / FindPedForRigidBody (drag support)
// ============================================================
btRigidBody* BoneNodePhysics::GetBoneRigidBody(CPed* ped, int boneTag) {
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return nullptr;
    for (auto& b : it->second)
        if (b->boneTag == boneTag && b->rigidBody) return b->rigidBody;
    return nullptr;
}

CPed* BoneNodePhysics::FindPedForRigidBody(btRigidBody* body) {
    for (auto& [ped, bones] : s_PedBones) {
        for (auto& b : bones)
            if (b->rigidBody == body) return ped;
    }
    return nullptr;
}

// ============================================================
//  Stats
// ============================================================
int BoneNodePhysics::GetBoneCount() {
    int total = 0;
    for (auto& [_, bones] : s_PedBones) total += (int)bones.size();
    return total;
}

int BoneNodePhysics::GetActiveBoneCount() {
    int total = 0;
    for (auto& [_, bones] : s_PedBones)
        for (auto& b : bones) if (b->isActive) ++total;
    return total;
}

// ============================================================
//  D3D9 line helper (screen-space, anti-aliased)
// ============================================================
static void DrawLineD3D9(IDirect3DDevice9* dev, float x1, float y1,
                         float x2, float y2, float /*width*/, D3DCOLOR color) {
    struct D3DLVERTEX {
        float x, y, z, rhw;
        D3DCOLOR color;
    };

    D3DLVERTEX verts[] = {
        {x1, y1, 0.0f, 1.0f, color},
        {x2, y2, 0.0f, 1.0f, color}
    };

    dev->SetFVF(D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
    dev->SetTexture(0, nullptr);
    dev->SetVertexShader(nullptr);
    dev->SetPixelShader(nullptr);
    dev->SetRenderState(D3DRS_ZENABLE, FALSE);
    dev->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    dev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, TRUE);
    dev->DrawPrimitiveUP(D3DPT_LINELIST, 1, verts, sizeof(D3DLVERTEX));
    dev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    dev->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    dev->SetRenderState(D3DRS_ZENABLE, TRUE);
}

static void DrawDigitD3D9(IDirect3DDevice9* dev, float x, float y, float size,
                          D3DCOLOR color, int digit) {
    const float w = size;
    const float h = size * 1.6f;
    const float midY = y + h * 0.5f;

    auto segA = [&]() { DrawLineD3D9(dev, x, y, x + w, y, 1.0f, color); };
    auto segB = [&]() { DrawLineD3D9(dev, x + w, y, x + w, midY, 1.0f, color); };
    auto segC = [&]() { DrawLineD3D9(dev, x + w, midY, x + w, y + h, 1.0f, color); };
    auto segD = [&]() { DrawLineD3D9(dev, x, y + h, x + w, y + h, 1.0f, color); };
    auto segE = [&]() { DrawLineD3D9(dev, x, midY, x, y + h, 1.0f, color); };
    auto segF = [&]() { DrawLineD3D9(dev, x, y, x, midY, 1.0f, color); };
    auto segG = [&]() { DrawLineD3D9(dev, x, midY, x + w, midY, 1.0f, color); };

    static const int kMask[10] = {
        0b1111110, // 0: A B C D E F
        0b0110000, // 1: B C
        0b1101101, // 2: A B D E G
        0b1111001, // 3: A B C D G
        0b0110011, // 4: B C F G
        0b1011011, // 5: A C D F G
        0b1011111, // 6: A C D E F G
        0b1110000, // 7: A B C
        0b1111111, // 8: A B C D E F G
        0b1111011  // 9: A B C D F G
    };

    if (digit < 0 || digit > 9) {
        segG();
        return;
    }

    const int mask = kMask[digit];
    if (mask & 0b1000000) segA();
    if (mask & 0b0100000) segB();
    if (mask & 0b0010000) segC();
    if (mask & 0b0001000) segD();
    if (mask & 0b0000100) segE();
    if (mask & 0b0000010) segF();
    if (mask & 0b0000001) segG();
}

static void DrawNumberD3D9(IDirect3DDevice9* dev, float x, float y, float size,
                           D3DCOLOR color, int value) {
    std::string text = std::to_string(value);
    float cursorX = x;
    for (char ch : text) {
        if (ch == '-') {
            DrawDigitD3D9(dev, cursorX, y, size, color, -1);
            cursorX += size * 0.8f;
            continue;
        }
        int digit = ch - '0';
        DrawDigitD3D9(dev, cursorX, y, size, color, digit);
        cursorX += size * 1.2f;
    }
}

static void DrawCircleD3D9(IDirect3DDevice9* dev, float cx, float cy,
                           float radius, D3DCOLOR color, int segments = 20) {
    if (radius < 1.0f) radius = 1.0f;
    if (segments < 8) segments = 8;

    const float step = (SIMD_2_PI) / (float)segments;
    float prevX = cx + std::cos(0.0f) * radius;
    float prevY = cy + std::sin(0.0f) * radius;

    for (int i = 1; i <= segments; ++i) {
        float a = step * (float)i;
        float x = cx + std::cos(a) * radius;
        float y = cy + std::sin(a) * radius;
        DrawLineD3D9(dev, prevX, prevY, x, y, 1.0f, color);
        prevX = x;
        prevY = y;
    }
}

static D3DCOLOR GetBoneDebugColor(int boneTag) {
    // Head = Red
    if (boneTag == BONE_HEAD || boneTag == BONE_HEAD1 || boneTag == BONE_HEAD2 || boneTag == BONE_NECK)
        return D3DCOLOR_ARGB(255, 255, 70, 70);

    // Spine/Torso = Green
    if (boneTag == BONE_PELVIS || boneTag == BONE_PELVIS1 ||
        boneTag == BONE_SPINE1 || boneTag == BONE_UPPERTORSO)
        return D3DCOLOR_ARGB(255, 80, 255, 80);

    // Arms = Cyan
    if (boneTag == BONE_RIGHTSHOULDER || boneTag == BONE_RIGHTELBOW ||
        boneTag == BONE_RIGHTWRIST || boneTag == BONE_RIGHTHAND ||
        boneTag == BONE_LEFTSHOULDER || boneTag == BONE_LEFTELBOW ||
        boneTag == BONE_LEFTWRIST || boneTag == BONE_LEFTHAND)
        return D3DCOLOR_ARGB(255, 0, 255, 255);

    // Legs = Yellow
    if (boneTag == BONE_RIGHTHIP || boneTag == BONE_RIGHTKNEE ||
        boneTag == BONE_RIGHTANKLE || boneTag == BONE_RIGHTFOOT ||
        boneTag == BONE_LEFTHIP || boneTag == BONE_LEFTKNEE ||
        boneTag == BONE_LEFTANKLE || boneTag == BONE_LEFTFOOT)
        return D3DCOLOR_ARGB(255, 255, 230, 0);

    // Fallback = White
    return D3DCOLOR_ARGB(255, 255, 255, 255);
}

// ============================================================
//  DrawDebugBoneLines
//  Renders screen-space lines from each child bone to its
//  parent bone, forming a visible skeleton wireframe.
//  Uses D3D9 DrawPrimitiveUP + CSprite::CalcScreenCoors.
// ============================================================
void BoneNodePhysics::DrawDebugBoneLines() {
    auto* dev = static_cast<IDirect3DDevice9*>(RwD3D9GetCurrentD3DDevice());
    if (!dev) return;

    struct DebugLink { int parentTag; int childTag; };
    static const DebugLink kMappedLinks[] = {
        // OpenGL demo mapping (10 joints): pelvis-spine, spine-head,
        // spine-shoulders, shoulders-elbows, pelvis-hips, hips-knees.
        {BONE_PELVIS1,       BONE_PELVIS}, // KANDUNG KEMIH -> PERUT
        {BONE_PELVIS,        BONE_SPINE1}, // PERUT  -> TULANG PUNGGUNG

        {BONE_SPINE1,        BONE_UPPERTORSO}, // TULANG PUNGGUNG -> PUNGGUNG ATAS
        {BONE_UPPERTORSO,    BONE_NECK}, // PUNGGUNG ATAS -> LEHER
        {BONE_NECK,          BONE_HEAD2}, // LEHER -> DAHI
        {BONE_HEAD2,         BONE_HEAD1}, // DAHI -> ALIS
        {BONE_HEAD1,         BONE_HEAD}, // ALIS -> RAHANG/HEAD

        {BONE_UPPERTORSO,    BONE_LEFTUPPERTORSO}, // PUNGGUNG ATAS -> PUNGGUNG ATAS KIRI
        {BONE_LEFTUPPERTORSO,BONE_LEFTSHOULDER}, // PUNGGUNG ATAS KIRI -> BAHU KIRI
        {BONE_LEFTSHOULDER,  BONE_LEFTELBOW}, // BAHU KIRI -> SIKU KIRI
        {BONE_LEFTELBOW,     BONE_LEFTWRIST}, // SIKU KIRI -> PERGELANGAN KIRI
        {BONE_LEFTWRIST,     BONE_LEFTHAND}, // PERGELANGAN KIRI -> TANGAN KIRI

        {BONE_UPPERTORSO,    BONE_RIGHTUPPERTORSO}, // PUNGGUNG ATAS -> PUNGGUNG ATAS KANAN
        {BONE_RIGHTUPPERTORSO,BONE_RIGHTSHOULDER}, // PUNGGUNG ATAS KANAN -> BAHU KANAN
        {BONE_RIGHTSHOULDER, BONE_RIGHTELBOW}, // BAHU KANAN -> SIKU KANAN
        {BONE_RIGHTELBOW,    BONE_RIGHTWRIST}, // SIKU KANAN -> PERGELANGAN KANAN
        {BONE_RIGHTWRIST,    BONE_RIGHTHAND}, // PERGELANGAN KANAN -> TANGAN KANAN

        {BONE_PELVIS,       BONE_LEFTHIP}, // PERUT -> PANGGUL KIRI
        {BONE_LEFTHIP,      BONE_LEFTKNEE}, // PANGGUL KIRI -> LUTUT KIRI
        {BONE_LEFTKNEE,     BONE_LEFTANKLE}, // LUTUT KIRI -> PERGELANGAN KIRI
        {BONE_LEFTANKLE,    BONE_LEFTFOOT}, // PERGELANGAN KIRI -> KAKI KIRI

        {BONE_PELVIS,       BONE_RIGHTHIP}, // PERUT -> PANGGUL KANAN
        {BONE_RIGHTHIP,     BONE_RIGHTKNEE}, // PANGGUL KANAN -> LUTUT KANAN
        {BONE_RIGHTKNEE,    BONE_RIGHTANKLE}, // LUTUT KANAN -> PERGELANGAN KANAN
        {BONE_RIGHTANKLE,   BONE_RIGHTFOOT} // PERGELANGAN KANAN -> KAKI KANAN
    };

    // static inline std::unordered_map<CPed*, std::vector<std::unique_ptr<BonePhysicsData>>> s_PedBones;
    
    if (s_DrawBoneMode == eBoneDrawMode::None) return;

    // s_DrawBoneMode is bitwise, so we can combine multiple modes. For example, we can draw both Bullet bones and GTA bones at the same time for comparison.
    if ((s_DrawBoneMode == eBoneDrawMode::BulletBone) || (s_DrawBoneMode == eBoneDrawMode::Both)) {

        for (auto& [ped, data] : s_PedBones) {
            if (!ped) continue;
            
            for (auto& b : data) {
                if (!b->isActive || !b->rigidBody) continue;
                if (b->boneIndex < 0) continue;
                
                const btTransform worldTF = b->rigidBody->getWorldTransform();
                const btVector3 btPos = worldTF.getOrigin();
                const btQuaternion btQuat = worldTF.getRotation();

                // Draw The rigid body bone with D3D9DrawLine
                RwV3d bonePos = { btPos.x(), btPos.y(), btPos.z() };
                RwV3d boneRot = { btQuat.x(), btQuat.y(), btQuat.z() };
                
                RwV3d screenPos = {0};
                float w, h;
                if (CSprite::CalcScreenCoors(bonePos, &screenPos, &w, &h, true, true)) {
                    float size = std::clamp(h * 0.1f, 8.0f, 20.0f);
                    D3DCOLOR color = D3DCOLOR_ARGB(255, 255, 0, 0);

                    float length = std::clamp(h * 0.25f, 10.0f, 40.0f);
                    float angleRad = boneRot.y * (SIMD_PI / 180.0f);
                    float dx = std::sin(angleRad) * length;
                    float dy = std::cos(angleRad) * length;
                    DrawLineD3D9(dev, screenPos.x, screenPos.y, screenPos.x + dx, screenPos.y - dy, 2.0f, color);
                }
            }
        }
    }

    if ((s_DrawBoneMode == eBoneDrawMode::GameBone) || (s_DrawBoneMode == eBoneDrawMode::Both)) {

        for (auto& [ped, hier] : s_PedHierarchies) {
            if (!ped) continue;

            auto getBoneWorldPos = [&](int tag, RwV3d& outPos) -> bool {
                if (!BoneHelper::IsValidBone(ped, tag)) return false;
                outPos = BoneHelper::GetBonePosition(ped, tag);
                return true;
            };

            // Draw head marker (circle) so it's easy to identify.
            RwV3d headWp = {0}, headSp = {0};
            float headW = 0.0f, headH = 0.0f;
            if (getBoneWorldPos(BONE_HEAD, headWp)
                && CSprite::CalcScreenCoors(headWp, &headSp, &headW, &headH, true, true)) {
                float radius = std::clamp(headH * 0.18f, 5.0f, 28.0f);
                DrawCircleD3D9(dev, headSp.x, headSp.y, radius, D3DCOLOR_ARGB(255, 255, 60, 60), 24);
            }

            // Draw GTA bone IDs as screen-space numbers for all existing GTA bones.
            for (int tag = BONE_PELVIS1; tag <= BONE_RIGHTFOOT; ++tag) {
                RwV3d wp = {0};
                if (!getBoneWorldPos(tag, wp)) continue;

                RwV3d sp = {0};
                float w = 0.0f, h = 0.0f;
                if (!CSprite::CalcScreenCoors(wp, &sp, &w, &h, true, true)) continue;

                float size = std::clamp(h * 0.08f, 6.0f, 16.0f);
                D3DCOLOR color = GetBoneDebugColor(tag);
                DrawNumberD3D9(dev, sp.x + size * 0.2f, sp.y - size * 1.2f, size, color, tag);

                // Draw facing line from bone forward vector.
                RwV3d rotation = BoneHelper::GetBoneRotation(ped, tag);
                float lineLength = std::clamp(h * 0.25f, 10.0f, 40.0f);
                float angleRad = rotation.y * (SIMD_PI / 180.0f);
                float dx = std::sin(angleRad) * lineLength;
                float dy = std::cos(angleRad) * lineLength;
                DrawLineD3D9(dev, sp.x, sp.y, sp.x + dx, sp.y - dy, 1.0f, color);
            }

            for (const auto& link : kMappedLinks) {
                RwV3d wp1 = {0}, wp2 = {0};
                if (!getBoneWorldPos(link.parentTag, wp1)) continue;
                if (!getBoneWorldPos(link.childTag, wp2)) continue;
                RwV3d sp1 = {0}, sp2 = {0};
                float w, h;

                // Project to screen coordinates
                if (!CSprite::CalcScreenCoors(wp1, &sp1, &w, &h, true, true)) continue;
                if (!CSprite::CalcScreenCoors(wp2, &sp2, &w, &h, true, true)) continue;

                D3DCOLOR color = GetBoneDebugColor(link.childTag);

                DrawLineD3D9(dev, sp1.x, sp1.y, sp2.x, sp2.y, 1.0f, color);
            }
        }
    }
}