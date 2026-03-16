#pragma once

#include <string>
#include <unordered_map>
#include <vector>

// ============================================================
//  Simple INI file parser (no external dependencies)
// ============================================================
class IniFile {
public:
    bool Load(const std::string& path);
    bool Save(const std::string& path) const;

    std::string GetString(const std::string& section, const std::string& key, const std::string& def) const;
    float       GetFloat (const std::string& section, const std::string& key, float def) const;
    int         GetInt   (const std::string& section, const std::string& key, int def) const;
    bool        GetBool  (const std::string& section, const std::string& key, bool def) const;

private:
    std::string MakeKey(const std::string& section, const std::string& key) const;
    std::unordered_map<std::string, std::string> m_Values;
};

// ============================================================
//  Config struct — all tunable parameters with sane defaults
// ============================================================
struct RagdollConfig {
    // [Physics]
    float fGravityZ             = -9.81f;
    int   iSolverIterations     = 30;
    float fSolverERP            = 0.8f;
    int   iMaxSubSteps          = 10;
    float fFixedTimeStep        = 1.0f / 120.0f;
    float fMaxDeltaTime         = 0.05f;
    float fSimulationSpeed      = 1.0f;

    // [Ragdoll]
    float fLinearDamping        = 0.4f;
    float fAngularDamping       = 0.85f;
    float fFriction             = 0.75f;
    float fRestitution          = 0.05f;
    float fConstraintSoftness   = 0.6f;
    float fConstraintBias       = 0.6f;
    float fConstraintRelaxation = 1.0f;
    float fMaxBoneSpeed         = 15.0f;
    float fStretchTolerance     = 1.05f;
    int   iDistanceIterations   = 6;
    float fConstraintDamping    = 0.5f;
    int   iConstraintSolverIter = 50;
    float fShapeMargin          = 0.04f;
    float fCCDThreshold         = 0.1f;
    float fCCDSweptSphereRadius = 0.05f;

    // [World]
    float fCollisionLoadRadius     = 100.0f;
    int   iMaxEntities             = 80;
    int   iMaxTrianglesPerCall     = 2000;
    int   iCollisionUpdateInterval = 60;
    float fSurfaceFriction         = 0.8f;
    float fSurfaceRestitution      = 0.1f;
    float fWorldShapeMargin        = 0.04f;
    float fWorldMinX               = -3000.0f;
    float fWorldMaxX               =  3000.0f;
    float fWorldMinY               = -3000.0f;
    float fWorldMaxY               =  3000.0f;

    // [Drag]
    float fDragSpring       = 80.0f;
    float fDragDamping      = 0.7f;
    float fDragMaxVelocity  = 30.0f;
    float fDragDistance     = 6.0f;

    // [Reactions]
    float fBulletPower          = 5.0f;
    float fExplosionPower       = 15.0f;
    float fGetUpThreshold       = 0.5f;
    float fGetUpDelay           = 3.0f;

    // [Vehicle]
    bool  bVehicleCollision     = true;
    bool  bUseSpheresShape      = false;
    float fVehicleFriction      = 0.6f;
    float fVehicleRestitution   = 0.2f;

    // [DynamicObjects]
    bool  bDynamicObjects           = false;
    float fDynamicObjectMass        = 10.0f;
    float fDynamicObjectFriction    = 0.5f;
    float fDynamicObjectRestitution = 0.1f;

    void LoadFromIni(const IniFile& ini);
};

// ============================================================
//  Global config instance + helpers
// ============================================================
extern RagdollConfig g_Config;

void LoadConfig();          // Load from INI (or create default)
void SaveDefaultConfig();   // Write INI with defaults + comments
std::string GetConfigPath();
