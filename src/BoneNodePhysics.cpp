#include "BoneNodePhysics.h"
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
#include <d3d9.h>

extern void DebugLog(const std::string& msg);
extern bool g_PhysicsShutdown;   // set in main.cpp before world is deleted

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
            return nullptr;
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

    struct JointDef {
        int parentTag;
        int childTag;
        // Cone-twist limits (in radians): swing1, swing2, twist
        float swing1, swing2, twist;
    };

    static const JointDef joints[] = {
        // Spine chain
        {BONE_PELVIS,      BONE_SPINE1,        0.30f, 0.20f, 0.20f},
        {BONE_SPINE1,      BONE_UPPERTORSO,    0.30f, 0.20f, 0.20f},
        {BONE_UPPERTORSO,  BONE_NECK,          0.30f, 0.25f, 0.20f},
        {BONE_NECK,        BONE_HEAD,          0.40f, 0.40f, 0.15f},

        // Right arm
        {BONE_UPPERTORSO,  BONE_RIGHTSHOULDER, 1.40f, 1.00f, 0.50f},
        {BONE_RIGHTSHOULDER, BONE_RIGHTELBOW,  1.30f, 0.20f, 0.20f},
        {BONE_RIGHTELBOW,  BONE_RIGHTWRIST,    1.20f, 0.15f, 0.30f},
        {BONE_RIGHTWRIST,  BONE_RIGHTHAND,     0.50f, 0.40f, 0.20f},

        // Left arm
        {BONE_UPPERTORSO,  BONE_LEFTSHOULDER,  1.40f, 1.00f, 0.50f},
        {BONE_LEFTSHOULDER, BONE_LEFTELBOW,    1.30f, 0.20f, 0.20f},
        {BONE_LEFTELBOW,   BONE_LEFTWRIST,     1.20f, 0.15f, 0.30f},
        {BONE_LEFTWRIST,   BONE_LEFTHAND,      0.50f, 0.40f, 0.20f},

        // Right leg
        {BONE_PELVIS,      BONE_RIGHTHIP,      0.80f, 0.60f, 0.30f},
        {BONE_RIGHTHIP,    BONE_RIGHTKNEE,     1.50f, 0.10f, 0.10f},
        {BONE_RIGHTKNEE,   BONE_RIGHTANKLE,    0.60f, 0.10f, 0.10f},
        {BONE_RIGHTANKLE,  BONE_RIGHTFOOT,     0.40f, 0.30f, 0.15f},

        // Left leg
        {BONE_PELVIS,      BONE_LEFTHIP,       0.80f, 0.60f, 0.30f},
        {BONE_LEFTHIP,     BONE_LEFTKNEE,      1.50f, 0.10f, 0.10f},
        {BONE_LEFTKNEE,    BONE_LEFTANKLE,     0.60f, 0.10f, 0.10f},
        {BONE_LEFTANKLE,   BONE_LEFTFOOT,      0.40f, 0.30f, 0.15f},
    };

    int created = 0;
    for (const auto& j : joints) {
        auto* parent = findBone(j.parentTag);
        auto* child  = findBone(j.childTag);
        if (!parent || !child || !parent->rigidBody || !child->rigidBody) continue;

        // ---- Compute pivot offsets from current world transforms ----
        // The pivot point is the child's world origin expressed in:
        //   parentFrame — local space of parent body
        //   childFrame  — local space of child body (identity = child origin)
        btTransform parentWorldTF = parent->rigidBody->getWorldTransform();
        btTransform childWorldTF  = child->rigidBody->getWorldTransform();

        // Pivot is at the child's centre of mass (its world origin)
        // expressed relative to parent's local frame:
        btTransform parentFrame = parentWorldTF.inverse() * childWorldTF;
        btTransform childFrame;
        childFrame.setIdentity();   // pivot is exactly at child's CoM

        auto* ct = new btConeTwistConstraint(
            *parent->rigidBody, *child->rigidBody,
            parentFrame, childFrame
        );
        ct->setLimit(j.swing1, j.swing2, j.twist,
                     0.9f,   // softness
                     0.3f,   // bias factor
                     1.0f);  // relaxation factor

        s_DynamicsWorld->addConstraint(ct, true);   // true = disable self-collision
        child->constraint = ct;
        created++;
    }

    char buf[128];
    sprintf_s(buf, "Created %d cone-twist constraints", created);
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
            // Small downward nudge so simulation starts immediately
            b->rigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, -0.5f));
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

    // Ensure hierarchy matrices are up to date
    RpHAnimHierarchySetFlags(hier,
        (RpHAnimHierarchyFlag)(RpHAnimHierarchyGetFlags(hier)
            | rpHANIMHIERARCHYUPDATELTMS
            | rpHANIMHIERARCHYUPDATEMODELLINGMATRICES));
    RpHAnimHierarchyUpdateMatrices(hier);

    RwMatrix* matrices = RpHAnimHierarchyGetMatrixArray(hier);

    // Helper: find a BonePhysicsData by tag
    auto findBone = [&](int tag) -> BonePhysicsData* {
        for (auto& b : it->second)
            if (b->boneTag == tag) return b.get();
        return nullptr;
    };

    // Get initial GTA interp frame quaternions
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
            } else {
                b->initBulletLocalQuat = tf.getRotation();
            }
        } else {
            // Root bone: local = world rotation (relative to ped entity)
            b->initBulletLocalQuat = tf.getRotation();
        }
        b->initBulletLocalQuat.normalize();

        // ---- Capture initial GTA interp frame quaternion ----
        if (clumpData && b->boneIndex >= 0) {
            auto* frameData = &clumpData->m_pFrames[b->boneIndex];
            if (frameData && frameData->m_pIFrame) {
                auto* frame = reinterpret_cast<RpHAnimBlendInterpFrame*>(frameData->m_pIFrame);
                RtQuat& q = frame->orientation;
                b->initGtaQuat = btQuaternion(q.imag.x, q.imag.y, q.imag.z, q.real);
                b->initGtaQuat.normalize();
            }
        }
    }

    DebugLog("SyncAllToBullet: pushed " + std::to_string(it->second.size()) + " bones (with init quats)");
}

// ============================================================
//  SyncAllFromBullet  (called every Bullet step)
//
//  Only handles physics-side updates:
//   - Velocity clamping
//   - Move ped entity to pelvis position (keeps streaming/culling happy)
//   - Update heading from spine direction
//
//  Matrix write is done in WriteBulletMatricesToHierarchy(),
//  called from pedRenderEvent.before so it fires AFTER GTA's
//  animation update and can never be overwritten by the idle anim.
// ============================================================
void BoneNodePhysics::SyncAllFromBullet() {
    for (auto& [ped, bones] : s_PedBones) {
        if (!ped) continue;

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

        CMatrix* pedMat = ped->GetMatrix();
        if (pedMat) {
            pedMat->GetPosition() = newPos;
            pedMat->UpdateRW();
        }
        ped->m_vecMoveSpeed = CVector(0, 0, 0);
        ped->m_vecTurnSpeed = CVector(0, 0, 0);

        // ---- 2. Update heading from spine→neck (like prototype) ----
        auto* neck = findBone(BONE_NECK);
        if (neck && neck->rigidBody) {
            btVector3 neckPos = neck->rigidBody->getWorldTransform().getOrigin();
            float dx = neckPos.x() - pelvisPos.x();
            float dy = neckPos.y() - pelvisPos.y();
            float len2D = std::sqrt(dx * dx + dy * dy);
            if (len2D > 0.001f)
                ped->SetHeading(std::atan2(-dx, dy));
        }
    }
}

// ============================================================
//  WriteBulletMatricesToHierarchy
//  Called from pedRenderEvent.before — overwrites GTA skinning
//  matrices with Bullet world-space transforms just before render.
// ============================================================
void BoneNodePhysics::WriteBulletMatricesToHierarchy(CPed* ped, RpHAnimHierarchy* /*hier*/, RwMatrix* matrices) {
    auto it = s_PedBones.find(ped);
    if (it == s_PedBones.end()) return;

    for (auto& b : it->second) {
        if (!b->isActive || !b->rigidBody) continue;
        if (b->boneIndex < 0) continue;

        // Write Bullet world-space transforms directly (hierarchy matrices are world-space)
        BtTransformToRwMatrix(b->rigidBody->getWorldTransform(), matrices[b->boneIndex]);
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
    dev->SetPixelShader(nullptr);
    dev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, TRUE);
    dev->DrawPrimitiveUP(D3DPT_LINELIST, 1, verts, sizeof(D3DLVERTEX));
    dev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, FALSE);
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

    for (auto& [ped, bones] : s_PedBones) {
        if (!ped) continue;

        auto findBone = [&](int tag) -> BonePhysicsData* {
            for (auto& b : bones)
                if (b->boneTag == tag) return b.get();
            return nullptr;
        };

        for (auto& b : bones) {
            if (!b->isActive || !b->rigidBody) continue;
            if (b->parentTag < 0) continue;  // root has no parent line

            auto* parent = findBone(b->parentTag);
            if (!parent || !parent->rigidBody) continue;

            btVector3 childBt  = b->rigidBody->getWorldTransform().getOrigin();
            btVector3 parentBt = parent->rigidBody->getWorldTransform().getOrigin();

            // Convert Bullet world positions to RwV3d
            RwV3d wp1 = {parentBt.x(), parentBt.y(), parentBt.z()};
            RwV3d wp2 = {childBt.x(),  childBt.y(),  childBt.z()};
            RwV3d sp1 = {0}, sp2 = {0};
            float w, h;

            // Project to screen coordinates
            if (!CSprite::CalcScreenCoors(wp1, &sp1, &w, &h, true, true)) continue;
            if (!CSprite::CalcScreenCoors(wp2, &sp2, &w, &h, true, true)) continue;

            // Green for spine/torso, Cyan for limbs
            D3DCOLOR color = D3DCOLOR_ARGB(255, 0, 255, 0);  // green
            if (b->boneTag == BONE_RIGHTSHOULDER || b->boneTag == BONE_RIGHTELBOW ||
                b->boneTag == BONE_RIGHTWRIST || b->boneTag == BONE_RIGHTHAND ||
                b->boneTag == BONE_LEFTSHOULDER || b->boneTag == BONE_LEFTELBOW ||
                b->boneTag == BONE_LEFTWRIST || b->boneTag == BONE_LEFTHAND ||
                b->boneTag == BONE_RIGHTHIP || b->boneTag == BONE_RIGHTKNEE ||
                b->boneTag == BONE_RIGHTANKLE || b->boneTag == BONE_RIGHTFOOT ||
                b->boneTag == BONE_LEFTHIP || b->boneTag == BONE_LEFTKNEE ||
                b->boneTag == BONE_LEFTANKLE || b->boneTag == BONE_LEFTFOOT) {
                color = D3DCOLOR_ARGB(255, 0, 255, 255);  // cyan
            }

            DrawLineD3D9(dev, sp1.x, sp1.y, sp2.x, sp2.y, 1.0f, color);
        }
    }
}