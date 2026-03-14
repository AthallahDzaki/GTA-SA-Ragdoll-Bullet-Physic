#pragma once

#include <btBulletDynamicsCommon.h>
#include <unordered_map>
#include <vector>
#include <memory>

#include <game_sa/common.h>

class CPed;
struct RpHAnimHierarchy;

struct BonePhysicsData {
    btRigidBody*           rigidBody    = nullptr;
    btCollisionShape*      shape        = nullptr;
    btTypedConstraint*     constraint   = nullptr;   // constraint TO parent
    btDefaultMotionState*  motionState  = nullptr;

    int    boneTag   = -1;
    int    boneIndex = -1;
    int    parentTag = -1;   // tag of parent bone in the ragdoll chain
    CPed*  ownerPed  = nullptr;

    bool   isActive  = false;

    // --- Delta rotation approach ---
    // Captured at creation time to compute delta changes at runtime.
    // GTA applies bind-pose on top of the interp frame quaternion,
    // so we must only write the CHANGE in rotation, not the absolute rotation.
    btQuaternion initBulletLocalQuat = btQuaternion::getIdentity();  // initial Bullet local rotation
    btQuaternion initGtaQuat         = btQuaternion::getIdentity();  // initial GTA interp frame quaternion

    ~BonePhysicsData();
};

class BoneNodePhysics {
private:
    static inline btDiscreteDynamicsWorld* s_DynamicsWorld = nullptr;
    static inline std::unordered_map<CPed*, std::vector<std::unique_ptr<BonePhysicsData>>> s_PedBones;
    static inline std::unordered_map<CPed*, RpHAnimHierarchy*> s_PedHierarchies;

public:
    static void Initialize(btDiscreteDynamicsWorld* world);
    static void Shutdown();

    static bool CreatePhysicsForPedBone(CPed* ped, int boneTag, int boneIndex, int parentTag);
    static void CreateConstraintsForPed(CPed* ped);

    static void ActivatePhysicsForPed(CPed* ped);
    static void DeactivatePhysicsForPed(CPed* ped);

    static void SyncAllToBullet(CPed* ped);
    static void SyncAllFromBullet();

    static void RegisterHierarchy(CPed* ped, RpHAnimHierarchy* hier);

    // Returns the rigid body of a specific bone (used by drag system)
    static btRigidBody* GetBoneRigidBody(CPed* ped, int boneTag);
    // Returns any active rigid body belonging to a ragdoll ped (for raycasting)
    static CPed* FindPedForRigidBody(btRigidBody* body);

    static int GetBoneCount();
    static int GetActiveBoneCount();

    // Draw 3D debug lines connecting parent/child bones (skeleton wireframe)
    static void DrawDebugBoneLines();

    // Called from pedRenderEvent.before — writes Bullet world transforms directly
    // into the RpHAnim skinning matrix array, overriding GTA's animation pose.
    static void WriteBulletMatricesToHierarchy(CPed* ped, RpHAnimHierarchy* hier, RwMatrix* matrices);
};