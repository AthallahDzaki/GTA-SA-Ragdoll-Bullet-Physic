#include "plugin_sa.h"
#include "game_sa/common.h"
#include "CTimer.h"
#include "CPed.h"
#include "CPlayerPed.h"
#include "CCivilianPed.h"
#include "CWorld.h"
#include "CStreaming.h"
#include "CColModel.h"
#include "CColTriangle.h"
#include "CVector.h"
#include "CMatrix.h"
#include "CCamera.h"
#include "rphanim.h"
#include "rpskin.h"
#include "CFont.h"
#include "extensions/FontPrint.h"
#include "extensions/KeyCheck.h"

#include "BoneHelper.h"
#include "BoneNodePhysics.h"
#include "ePedBones.h"
// #include "tBoneInfo.h"

#include <btBulletDynamicsCommon.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <fstream>
#include <chrono>

using namespace plugin;

// ============ DEBUG LOGGING ============
class DebugLog {
private:
    static inline std::ofstream logFile;
    static inline bool initialized = false;

public:
    static void Init() {
        if (!initialized) {
            logFile.open("BulletPhysics.log", std::ios::out | std::ios::trunc);
            initialized = true;
            Log("=== Bullet Physics Debug Log Started ===");
        }
    }

    static void Log(const std::string& message) {
        if (!initialized) Init();
        auto now = std::chrono::system_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        logFile << "[" << ms << "] " << message << std::endl;
        logFile.flush();
        std::cout << message << std::endl;
    }

    static void Close() {
        if (initialized) {
            Log("=== Bullet Physics Debug Log Ended ===");
            logFile.close();
            initialized = false;
        }
    }
};

// Free function required by BoneNodePhysics.cpp
void DebugLog(const std::string& msg) {
    DebugLog::Log(msg);
}

// ============ GLOBAL BULLET WORLD ============
btDiscreteDynamicsWorld*             g_DynamicsWorld   = nullptr;
btDefaultCollisionConfiguration*     g_CollisionConfig = nullptr;
btCollisionDispatcher*               g_Dispatcher      = nullptr;
btBroadphaseInterface*               g_Broadphase      = nullptr;
btSequentialImpulseConstraintSolver* g_Solver          = nullptr;

static constexpr short kCollisionGroupRagdoll = static_cast<short>(1 << 8);
static constexpr short kCollisionGroupWorld   = static_cast<short>(1 << 9);
static constexpr short kWorldCollisionMask    = static_cast<short>(
    btBroadphaseProxy::DefaultFilter | kCollisionGroupRagdoll);
    
std::unordered_map<int, btRigidBody*> g_LoadedCollisionBodies;
std::vector<btTriangleMesh*>          g_TriangleMeshes;

// Per-ragdoll ground planes (one flat plane per ped, at actual ground height).
// Prevents falling into the void before mesh collision is loaded.
struct GroundPlaneEntry {
    btCollisionShape* shape   = nullptr;
    btRigidBody*      body    = nullptr;
};
std::vector<GroundPlaneEntry> g_GroundPlanes;

float g_CollisionLoadRadius = 100.0f;
bool  g_BulletEnabled       = false;
bool  g_PhysicsShutdown     = false;  // set TRUE before deleting any Bullet objects
int   g_FrameCounter        = 0;
int   g_WorldCollisionCount = 0;

std::unordered_set<CPed*> g_RagdollPeds;

// ============ DRAG SYSTEM STATE ============
// Velocity-based drag: each frame we push the dragged body toward a target point.
btRigidBody* g_DraggedBody     = nullptr;
float        g_DragDistance    = 6.0f;
bool         g_DragActive      = false;

// ============ HELPERS ============
inline btVector3 ToBullet(const CVector& v) { return btVector3(v.x, v.y, v.z); }

RpHAnimHierarchy* GetPedHierarchy(CPed* ped) {
    if (!ped || !ped->m_pRwClump) return nullptr;
    return GetAnimHierarchyFromSkinClump(ped->m_pRwClump);
}

bool IsPedRagdoll(CPed* ped) {
    return g_RagdollPeds.find(ped) != g_RagdollPeds.end();
}

// ============ GROUND PLANE PER RAGDOLL ============
// Query GTA SA's collision to get actual ground Z, then add a Bullet static
// plane at that height.  This is a fallback so the ped doesn't fall into the
// void while mesh collision is still being built.
static void AddGroundPlaneAtPos(const CVector& worldPos) {
    if (!g_DynamicsWorld) return;

    bool   found   = false;
    float  groundZ = CWorld::FindGroundZFor3DCoord(worldPos.x, worldPos.y,
                                                     worldPos.z + 2.0f,
                                                     &found, nullptr);
    if (!found) groundZ = worldPos.z - 1.0f;   // fallback: just below spawn

    // A horizontal plane at groundZ: normal = (0,0,1), plane constant = groundZ
    auto* shape = new btStaticPlaneShape(btVector3(0.0f, 0.0f, 1.0f), groundZ);

    btTransform tf;
    tf.setIdentity();
    auto* ms   = new btDefaultMotionState(tf);
    auto* body = new btRigidBody(0.0f, ms, shape);
    body->setFriction(0.7f);
    body->setRestitution(0.05f);

    g_DynamicsWorld->addRigidBody(body, kCollisionGroupWorld, kWorldCollisionMask);
    g_GroundPlanes.push_back({shape, body});

    char buf[128];
    sprintf_s(buf, "Ground plane added at Z=%.2f (found=%d)", groundZ, (int)found);
    DebugLog::Log(buf);
}

static void CleanupGroundPlanes() {
    if (!g_DynamicsWorld) return;
    for (auto& gp : g_GroundPlanes) {
        if (gp.body)  g_DynamicsWorld->removeRigidBody(gp.body);
        delete gp.body;
        delete gp.shape;
    }
    g_GroundPlanes.clear();
}

// ============ RAGDOLL CREATION ============
//
// Bone chain definition: each entry is { boneTag, boneParentTag }
// parentTag = -1 means this is a root bone (pelvis)
struct BoneEntry { int tag; int parentTag; };
static const BoneEntry g_BoneChain[] = {
    {BONE_PELVIS,       -1},
    {BONE_SPINE1,       BONE_PELVIS},
    {BONE_UPPERTORSO,   BONE_SPINE1},
    {BONE_NECK,         BONE_UPPERTORSO},
    {BONE_HEAD,         BONE_NECK},

    {BONE_RIGHTSHOULDER,BONE_UPPERTORSO},
    {BONE_RIGHTELBOW,   BONE_RIGHTSHOULDER},
    {BONE_RIGHTWRIST,   BONE_RIGHTELBOW},
    {BONE_RIGHTHAND,    BONE_RIGHTWRIST},

    {BONE_LEFTSHOULDER, BONE_UPPERTORSO},
    {BONE_LEFTELBOW,    BONE_LEFTSHOULDER},
    {BONE_LEFTWRIST,    BONE_LEFTELBOW},
    {BONE_LEFTHAND,     BONE_LEFTWRIST},

    {BONE_RIGHTHIP,     BONE_PELVIS},
    {BONE_RIGHTKNEE,    BONE_RIGHTHIP},
    {BONE_RIGHTANKLE,   BONE_RIGHTKNEE},
    {BONE_RIGHTFOOT,    BONE_RIGHTANKLE},

    {BONE_LEFTHIP,      BONE_PELVIS},
    {BONE_LEFTKNEE,     BONE_LEFTHIP},
    {BONE_LEFTANKLE,    BONE_LEFTKNEE},
    {BONE_LEFTFOOT,     BONE_LEFTANKLE},
};

void CreateBulletRagdollForPed(CPed* ped) {
    if (!ped) return;
    DebugLog::Log("=== Creating ragdoll for ped ===");

    RpHAnimHierarchy* hier = GetPedHierarchy(ped);
    if (!hier) {
        DebugLog::Log("ERROR: No hierarchy");
        return;
    }

    BoneNodePhysics::RegisterHierarchy(ped, hier);

    // Update hierarchy so bone world matrices are current before reading them
    RpHAnimHierarchySetFlags(hier,
        (RpHAnimHierarchyFlag)(RpHAnimHierarchyGetFlags(hier)
            | rpHANIMHIERARCHYUPDATELTMS
            | rpHANIMHIERARCHYUPDATEMODELLINGMATRICES));
    RpHAnimHierarchyUpdateMatrices(hier);

    int created = 0;
    for (const auto& entry : g_BoneChain) {
        int boneIndex = RpHAnimIDGetIndex(hier, entry.tag);
        if (boneIndex < 0) continue;

        if (BoneNodePhysics::CreatePhysicsForPedBone(ped, entry.tag, boneIndex, entry.parentTag))
            ++created;
    }

    char buf[128];
    sprintf_s(buf, "Created %d bone rigid bodies", created);
    DebugLog::Log(buf);

    BoneNodePhysics::CreateConstraintsForPed(ped);
    BoneNodePhysics::SyncAllToBullet(ped);
    BoneNodePhysics::ActivatePhysicsForPed(ped);

    // Add a ground plane at the ped's spawn point so it lands on the road
    // and doesn't fall through the world while mesh collision loads.
    AddGroundPlaneAtPos(ped->GetPosition());

    DebugLog::Log("=== Ragdoll creation complete ===");
}

// ============ BULLET WORLD INITIALIZATION ============
void InitBulletWorld() {
    DebugLog::Log("InitBulletWorld called");
    if (g_DynamicsWorld) { DebugLog::Log("Already initialized"); return; }

    try {
        g_CollisionConfig = new btDefaultCollisionConfiguration();
        g_Dispatcher      = new btCollisionDispatcher(g_CollisionConfig);
        g_Broadphase      = new btDbvtBroadphase();
        g_Solver          = new btSequentialImpulseConstraintSolver();

        g_DynamicsWorld = new btDiscreteDynamicsWorld(
            g_Dispatcher, g_Broadphase, g_Solver, g_CollisionConfig);

        // GTA SA uses Z-up world
        g_DynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -9.81f));

        // NOTE: We do NOT add a static ground plane at Z=0 because GTA terrain
        // is dynamic and at varying heights. World collision is added via
        // UpdateCollisionPerArea from actual CColModel geometry instead.

        g_BulletEnabled = true;
        DebugLog::Log("Bullet world initialized");

        BoneNodePhysics::Initialize(g_DynamicsWorld);
    }
    catch (const std::exception& e) {
        DebugLog::Log(std::string("ERROR init Bullet: ") + e.what());
    }
}

// ============ WORLD COLLISION LOADING ============
// Loads triangles from GTA's static geometry near the player into Bullet.
// Called every 60 frames; scans all buildings within the load radius.
void UpdateCollisionPerArea() {
    CPed* player = FindPlayerPed();
    if (!player || !g_DynamicsWorld) return;

    CVector pos = player->GetPosition();

    // --- Remove stale sectors ---
    std::vector<int> toRemove;
    for (const auto& [key, body] : g_LoadedCollisionBodies) {
        int sx   = key / 10000;
        int sy   = key % 10000;
        float dx = sx * 50.0f - pos.x;
        float dy = sy * 50.0f - pos.y;
        if (std::sqrt(dx * dx + dy * dy) > g_CollisionLoadRadius * 1.5f) {
            g_DynamicsWorld->removeRigidBody(body);
            delete body->getCollisionShape();
            delete body;
            toRemove.push_back(key);
        }
    }
    for (int k : toRemove) g_LoadedCollisionBodies.erase(k);

    // --- Load new geometry ---
    const int MAX_TRIANGLES_PER_CALL = 2000;
    const int MAX_ENTITIES           = 80;

    btTriangleMesh* triMesh     = new btTriangleMesh();
    bool            hasGeometry = false;
    int             triCount    = 0;
    int             entCount    = 0;

    for (auto ent : CPools::ms_pBuildingPool) {
        if (entCount++ >= MAX_ENTITIES) break;
        if (!ent || !ent->GetColModel()) continue;

        float dist = (ent->GetPosition() - pos).Magnitude();
        if (dist > g_CollisionLoadRadius) continue;

        CColModel* col = ent->GetColModel();
        if (!col->m_pColData || col->m_pColData->m_nNumTriangles == 0) continue;

        CMatrix* mat = ent->GetMatrix();
        if (!mat) continue;

        // Use sector key based on entity position (50m grid)
        int sx  = static_cast<int>(ent->GetPosition().x / 50.0f);
        int sy  = static_cast<int>(ent->GetPosition().y / 50.0f);
        int key = sx * 10000 + sy;
        if (g_LoadedCollisionBodies.contains(key)) continue;

        for (int t = 0; t < col->m_pColData->m_nNumTriangles; ++t) {
            if (triCount++ >= MAX_TRIANGLES_PER_CALL) break;

            const auto& tri  = col->m_pColData->m_pTriangles[t];
            const auto& verts = col->m_pColData->m_pVertices;

            triMesh->addTriangle(
                ToBullet((*mat) * verts[tri.m_nVertA].Uncompressed()),
                ToBullet((*mat) * verts[tri.m_nVertB].Uncompressed()),
                ToBullet((*mat) * verts[tri.m_nVertC].Uncompressed())
            );
            hasGeometry = true;
        }
    }

    if (!hasGeometry) {
        delete triMesh;
        return;
    }

    g_TriangleMeshes.push_back(triMesh);
    auto* shape = new btBvhTriangleMeshShape(triMesh, true);
    auto* body  = new btRigidBody(0.0f, nullptr, shape);
    body->setFriction(0.8f);
    body->setRestitution(0.1f);

    // Key by player sector for now (approximate)
    int sx  = static_cast<int>(pos.x / 50.0f);
    int sy  = static_cast<int>(pos.y / 50.0f);
    int key = sx * 10000 + sy;

    if (!g_LoadedCollisionBodies.contains(key)) {
        g_DynamicsWorld->addRigidBody(body, kCollisionGroupWorld, kWorldCollisionMask);
        g_LoadedCollisionBodies[key] = body;
    } else {
        delete shape;
        delete body;
        g_TriangleMeshes.pop_back();
        delete triMesh;
    }

    g_WorldCollisionCount = (int)g_LoadedCollisionBodies.size();
}

// ============ PED RAGDOLL MAINTENANCE ============
void UpdateRagdollPeds() {
    for (auto* ped : g_RagdollPeds) {
        if (!ped) continue;

        // Suppress GTA's own physics and AI
        ped->SkipPhysics();
        ped->m_vecMoveSpeed = CVector(0, 0, 0);
        ped->m_vecTurnSpeed = CVector(0, 0, 0);
        ped->bUpdateAnimHeading = false;
        // ped->bDontRender = true;  // wireframe only (hide GTA ped mesh)

        if (ped->m_pIntelligence)
            ped->m_pIntelligence->ClearTasks(false, false);

        // Prevent GTA from streaming the ped out
        ped->m_nAreaCode          = 0;
        ped->bStreamingDontDelete = true;
        ped->bImBeingRendered     = true;
    }
}

// ============ RENDER HOOK: write Bullet matrices then freeze hierarchy update ============
//
// Strategy for preventing GTA from overwriting our Bullet matrices:
//  1. (.before) Write Bullet world-space transforms into hierarchy matrix array.
//  2. (.before) CLEAR rpHANIMHIERARCHYUPDATELTMS + UPDATE_MODELLING flags.
//             GTA calls RpHAnimHierarchyUpdateMatrices INSIDE the render chain,
//             but it checks these flags first — cleared → no-op → our matrices survive.
//  3. (.after)  RESTORE the flags so next frame's animation blend can write normally
//             (we'll override again in .before before render).

static constexpr RpHAnimHierarchyFlag kUpdateFlags =
    (RpHAnimHierarchyFlag)(rpHANIMHIERARCHYUPDATELTMS | rpHANIMHIERARCHYUPDATEMODELLINGMATRICES);

void OnBeforePedRender(CPed* ped) {
    if (!ped || g_RagdollPeds.find(ped) == g_RagdollPeds.end()) return;
    if (!ped->m_pRwClump) return;

    RpHAnimHierarchy* hier = GetAnimHierarchyFromSkinClump(ped->m_pRwClump);
    if (!hier) return;

    RwMatrix* matrices = RpHAnimHierarchyGetMatrixArray(hier);
    if (!matrices) return;

    // Step 1: Force GTA to update ALL hierarchy matrices from animation NOW
    // (so non-Bullet bones like fingers get correct positions from animation)
    RpHAnimHierarchyFlag curFlags = RpHAnimHierarchyGetFlags(hier);
    RpHAnimHierarchySetFlags(hier, (RpHAnimHierarchyFlag)((int)curFlags | (int)kUpdateFlags));
    RpHAnimHierarchyUpdateMatrices(hier);

    // Step 2: Overwrite ONLY the Bullet-controlled bones with ragdoll transforms
    BoneNodePhysics::WriteBulletMatricesToHierarchy(ped, hier, matrices);

    // Step 3: Clear update flags — GTA's internal UpdateMatrices call inside render is now a no-op
    // Our matrices survive to the actual skinning/render
    RpHAnimHierarchySetFlags(hier, (RpHAnimHierarchyFlag)((int)curFlags & ~(int)kUpdateFlags));
}

void OnAfterPedRender(CPed* ped) {
    if (!ped || g_RagdollPeds.find(ped) == g_RagdollPeds.end()) return;
    if (!ped->m_pRwClump) return;

    RpHAnimHierarchy* hier = GetAnimHierarchyFromSkinClump(ped->m_pRwClump);
    if (!hier) return;

    // Step 3: Restore update flags so next frame's anim blend + UpdateMatrices works
    // (we'll override again in OnBeforePedRender before render)
    RpHAnimHierarchyFlag flags = RpHAnimHierarchyGetFlags(hier);
    RpHAnimHierarchySetFlags(hier, (RpHAnimHierarchyFlag)((int)flags | (int)kUpdateFlags));
}

// ============ SPAWN RAGDOLL PED ============
void SpawnRagdollPed() {
    CPlayerPed* player = FindPlayerPed();
    if (!player) return;

    // Model 7 = MALE01 (generic male pedestrian)
    int modelId = 7;
    CStreaming::RequestModel(modelId, 0x2);
    CStreaming::LoadAllRequestedModels(false);

    if (CStreaming::ms_aInfoForModel[modelId].m_nLoadState != LOADSTATE_LOADED) {
        DebugLog::Log("ERROR: Model 7 not loaded");
        return;
    }

    // Spawn slightly in front of player, at their height + small offset
    CVector playerPos = player->GetPosition();
    CVector camDir    = TheCamera.m_mCameraMatrix.GetForward();
    camDir.z          = 0.0f;   // flatten direction to horizontal
    float len         = std::sqrt(camDir.x * camDir.x + camDir.y * camDir.y);
    if (len > 0.001f) { camDir.x /= len; camDir.y /= len; }

    CVector spawnPos = playerPos + camDir * 3.5f;
    spawnPos.z      += 1.5f;    // small lift so ped is above ground, not inside it

    CPed* newPed = new CCivilianPed(PED_TYPE_CIVMALE, modelId);
    if (!newPed) return;

    newPed->SetPosn(spawnPos);
    newPed->SetOrientation(0.0f, 0.0f, 0.0f);
    CWorld::Add(newPed);

    // Stop all GTA tasks / animations
    if (newPed->m_pIntelligence)
        newPed->m_pIntelligence->ClearTasks(true, true);

    // Disable GTA's own physics engine for this ped so Bullet controls it
    newPed->SkipPhysics();

    for (const auto& entry : g_BoneChain) {
        RwV3d at = BoneHelper::GetBoneRwMatrix(newPed, entry.tag)->at;
        std::cout << "Ped ctor: " << newPed << " at (" << at.x << "," << at.y << "," << at.z << ")" << std::endl;
    }
    
    newPed->bDontApplySpeed     = true;
    newPed->bInfiniteMass       = false;   // must be false or GTA ignores forces
    newPed->bIsStatic           = false;   // must be false or GTA won't call physics
    newPed->bUsesCollision      = false;   // don't let GTA do collision response
    newPed->bDontAcceptIKLookAts = true;
    // newPed->bDisableCollisionForce = true;
    // newPed->bCollidable         = false;
    // newPed->bDisableTurnForce   = true;
    // newPed->bDisableMoveForce   = true;
    // newPed->bStayInSamePlace = true;

    newPed->m_fHealth = 0.0f;

    newPed->m_vecMoveSpeed      = CVector(0, 0, 0);
    newPed->m_vecTurnSpeed      = CVector(0, 0, 0);

    g_RagdollPeds.insert(newPed);
    CreateBulletRagdollForPed(newPed);   // also adds ground plane inside

    // Force-load collision geometry around spawn right now (don't wait 60 frames).
    // This builds the BVH mesh from nearby buildings into Bullet immediately.
    UpdateCollisionPerArea();

    DebugLog::Log("Ragdoll ped spawned at " +
        std::to_string(spawnPos.x) + "," +
        std::to_string(spawnPos.y) + "," +
        std::to_string(spawnPos.z));

    // float dt = CTimer::ms_fTimeStep * (1.0f / 50.0f);
    // dt = std::min(dt, 0.05f);   // cap at 50ms to avoid explosion on lag
    // g_DynamicsWorld->stepSimulation(dt, 10, 1.0f / 120.0f);
}

void CleanupAllRagdolls() {
    for (auto* ped : g_RagdollPeds) {
        if (!ped) continue;
        BoneNodePhysics::DeactivatePhysicsForPed(ped);
        if (!g_PhysicsShutdown) {
            if(!ped) continue; 
            CWorld::Remove(ped);
            delete ped;
        }
    }
    CleanupGroundPlanes();
    g_RagdollPeds.clear();
    DebugLog::Log("All ragdolls cleaned up");
}

// ============ DRAG SYSTEM ============
// Velocity-based dragging: each frame we compute the vector from the
// dragged body to the camera target point and apply it as a velocity.
// This is more stable than btPoint2PointConstraint for interactive use.

static btRigidBody* RaycastGetRagdollBody() {
    if (!g_DynamicsWorld) return nullptr;

    CVector camPos  = TheCamera.GetPosition();
    CVector camDir  = TheCamera.m_mCameraMatrix.GetForward();
    CVector rayEnd  = camPos + camDir * 100.0f;

    btVector3 from = ToBullet(camPos);
    btVector3 to   = ToBullet(rayEnd);

    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    g_DynamicsWorld->rayTest(from, to, cb);

    if (cb.hasHit()) {
        btRigidBody* body = const_cast<btRigidBody*>(
            btRigidBody::upcast(cb.m_collisionObject));
        if (body && body->getMass() > 0.0f) {
            // Only drag bodies that belong to one of our ragdoll peds
            if (BoneNodePhysics::FindPedForRigidBody(body))
                return body;
        }
    }
    return nullptr;
}

void UpdateDragSystem() {

    CPlayerPed* player = FindPlayerPed();
    if (!player) return;
    bool isPlayerAiming = KeyCheck::Check(VK_RBUTTON);

    if (isPlayerAiming) {
        // Start drag
        g_DraggedBody = RaycastGetRagdollBody();
        if (g_DraggedBody) {
            g_DragActive = true;
            DebugLog::Log("Drag started");
        }
    } else if (!isPlayerAiming && g_DragActive) {
        // End drag — zero velocity so body doesn't fly off
        if (g_DraggedBody) {
            g_DraggedBody->setLinearVelocity(btVector3(0, 0, 0));
            g_DraggedBody->setAngularVelocity(btVector3(0, 0, 0));
            DebugLog::Log("Drag released");
        }
        g_DraggedBody = nullptr;
        g_DragActive  = false;
    }

    if (!g_DragActive || !g_DraggedBody) return;

    // Compute desired world position: in front of camera at g_DragDistance
    CVector camPos  = TheCamera.GetPosition();
    CVector camDir  = TheCamera.m_mCameraMatrix.GetForward();
    CVector target  = camPos + camDir * g_DragDistance;
    btVector3 btTarget = ToBullet(target);

    // Current body position
    btVector3 bodyPos = g_DraggedBody->getWorldTransform().getOrigin();

    // Drive toward target with a spring-like velocity
    btVector3 delta     = btTarget - bodyPos;
    float     distSq    = delta.length2();
    const float kSpring = 80.0f;   // spring constant — tune as needed
    const float kDamp   = 0.7f;    // damping on current velocity

    btVector3 currentVel = g_DraggedBody->getLinearVelocity();
    btVector3 newVel     = delta * kSpring - currentVel * kDamp;

    // Clamp velocity to prevent physics explosion
    const float MAX_VEL = 30.0f;
    if (newVel.length() > MAX_VEL)
        newVel = newVel.normalized() * MAX_VEL;

    g_DraggedBody->setLinearVelocity(newVel);
    g_DraggedBody->activate(true);
}

// ============ INPUT ============
void HandleInput() {
    KeyCheck::Update();
    // ALT+8 — Spawn new ragdoll ped
    static bool wasSpawnPressed = false;
    bool isSpawnPressed = KeyPressed(VK_MENU) && KeyPressed('8');
    if (isSpawnPressed && !wasSpawnPressed)
        SpawnRagdollPed();
    wasSpawnPressed = isSpawnPressed;

    // ALT+9 — Remove all ragdolls
    static bool wasCleanPressed = false;
    bool isCleanPressed = KeyPressed(VK_MENU) && KeyPressed('9');
    if (isCleanPressed && !wasCleanPressed)
        CleanupAllRagdolls();
    wasCleanPressed = isCleanPressed;

    UpdateDragSystem();
}

// ============ DEBUG UI ============
void DrawDebugInfo() {
    CFont::SetBackground(false, false);
    CFont::SetScale(0.35f, 0.7f);
    CFont::SetOrientation(eFontAlignment::ALIGN_LEFT);
    CFont::SetColor(CRGBA(255, 255, 255, 220));
    CFont::SetDropShadowPosition(1);
    CFont::SetEdge(1);
    CFont::SetProportional(true);
    CFont::SetFontStyle(eFontStyle::FONT_SUBTITLES);

    // All Y values in GTA screen coordinates (pixels, 0 = top)
    const float xPos   = 10.0f;
    float       yPos   = 140.0f;
    const float yStep  = 16.0f;

    char buf[256];

    auto printLine = [&](const char* text) {
        CFont::PrintString(xPos, yPos, const_cast<char*>(text));
        yPos += yStep;
    };

    sprintf_s(buf, "~w~Bullet: %s", g_BulletEnabled ? "~g~ON" : "~r~OFF");
    printLine(buf);

    sprintf_s(buf, "~w~WorldCol: ~y~%d", g_WorldCollisionCount);
    printLine(buf);

    sprintf_s(buf, "~w~Bones: ~y~%d ~w~(~g~%d ~w~active)",
        BoneNodePhysics::GetBoneCount(),
        BoneNodePhysics::GetActiveBoneCount());
    printLine(buf);

    sprintf_s(buf, "~w~Peds: ~y~%d", (int)g_RagdollPeds.size());
    printLine(buf);

    sprintf_s(buf, "~w~Drag: %s", g_DragActive ? "~g~YES" : "~r~NO");
    printLine(buf);

    yPos += 4.0f;
    printLine("~y~ALT+8 ~w~Spawn Ragdoll");
    printLine("~y~ALT+9 ~w~Clear All");
    printLine("~y~[G]   ~w~Hold to Drag");

    if(FindPlayerPed()) {
        sprintf_s(buf, "~w~Position: ~y~%f, %f, %f", FindPlayerPed()->GetPosition().x, FindPlayerPed()->GetPosition().y, FindPlayerPed()->GetPosition().z); 
        printLine(buf);
    }
    // NOTE: DrawDebugBoneLines is called from drawingEvent (separate hook)
    // to avoid D3D9 state changes corrupting CFont rendering.
}

// ============ PHYSICS UPDATE ============
void ProcessBulletPhysics() {
    if (!g_BulletEnabled || !g_DynamicsWorld) return;
    ++g_FrameCounter;

    try {
        // GTA's ms_fTimeStep is in game ticks (1/50s base). Convert to seconds.
        float dt = CTimer::ms_fTimeStep * (1.0f / 50.0f);
        dt = std::min(dt, 0.05f);   // cap at 50ms to avoid explosion on lag

        UpdateRagdollPeds();

        // Step Bullet simulation
        // maxSubSteps=10, fixedTimeStep=1/120 for stable joint solving
        g_DynamicsWorld->stepSimulation(dt, 10, 1.0f / 120.0f);

        // Update world collision every 60 frames (~1.2s at 50fps)
        if (g_FrameCounter % 60 == 0)
            UpdateCollisionPerArea();

        HandleInput();
    }
    catch (const std::exception& e) {
        DebugLog::Log(std::string("ERROR in ProcessBulletPhysics: ") + e.what());
        g_BulletEnabled = false;
    }
}

// ============ CLEANUP ============
void CleanupBulletWorld() {
    DebugLog::Log("=== CleanupBulletWorld START ===");

    try {
        // Set shutdown flag FIRST — BonePhysicsData destructors check this
        // to avoid deleting Bullet objects after the world is gone.
        g_PhysicsShutdown = true;

        // Stop drag
        g_DraggedBody = nullptr;
        g_DragActive  = false;

        // Removes constraints + bodies from world safely, then frees GTA peds.
        CleanupAllRagdolls();

        // Clears remaining ped physics data (should already be empty)
        BoneNodePhysics::Shutdown();

        if (g_DynamicsWorld) {
            for (auto& [key, body] : g_LoadedCollisionBodies) {
                if (body) {
                    g_DynamicsWorld->removeRigidBody(body);
                    delete body->getCollisionShape();
                    delete body;
                }
            }
            g_LoadedCollisionBodies.clear();
        }

        // Triangle meshes (used by BVH collision shapes) — already freed above
        // via getCollisionShape() delete, but the btTriangleMesh is separate.
        for (auto* mesh : g_TriangleMeshes) delete mesh;
        g_TriangleMeshes.clear();

        // Now safe to delete the Bullet world itself
        delete g_DynamicsWorld;   g_DynamicsWorld   = nullptr;
        delete g_Solver;          g_Solver          = nullptr;
        delete g_Broadphase;      g_Broadphase      = nullptr;
        delete g_Dispatcher;      g_Dispatcher      = nullptr;
        delete g_CollisionConfig; g_CollisionConfig = nullptr;

        g_BulletEnabled = false;
        DebugLog::Log("=== CleanupBulletWorld COMPLETE ===");
    }
    catch (const std::exception& e) {
        DebugLog::Log("ERROR in CleanupBulletWorld: " + std::string(e.what()));
    }

    DebugLog::Close();
}

void OnPedDestroyed(CPed* ped) {
    if(!ped) return; // Null Dtor?

}

// ============ PLUGIN ENTRY POINT ============
class BulletRagdollPlugin {
public:
    BulletRagdollPlugin() {
        DebugLog::Init();
        DebugLog::Log("=== BulletRagdollPlugin Init ===");

        Events::initRwEvent        += InitBulletWorld;
        Events::gameProcessEvent   += ProcessBulletPhysics;
        Events::drawAfterFadeEvent += DrawDebugInfo;

        // Bone lines use D3D9 DrawPrimitiveUP — must run in drawingEvent,
        // NOT in drawAfterFadeEvent alongside CFont, or it corrupts text.
        Events::drawingEvent += []{ BoneNodePhysics::DrawDebugBoneLines(); };
        BoneHelper::Initialise();

        Events::shutdownRwEvent    += CleanupBulletWorld;
        Events::pedDtorEvent       += OnPedDestroyed;

        DebugLog::Log("=== Events registered ===");
    }

    ~BulletRagdollPlugin() {
        DebugLog::Log("BulletRagdollPlugin destructor");
    }
} g_BulletRagdollPlugin;