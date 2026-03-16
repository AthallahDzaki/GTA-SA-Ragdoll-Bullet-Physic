#include "Config.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cstdio>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

RagdollConfig g_Config;

// ============================================================
//  IniFile implementation
// ============================================================
static std::string Trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static std::string ToLower(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    return r;
}

std::string IniFile::MakeKey(const std::string& section, const std::string& key) const {
    return ToLower(section) + "." + ToLower(key);
}

bool IniFile::Load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string currentSection;
    std::string line;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;

        // Section header
        if (line.front() == '[' && line.back() == ']') {
            currentSection = Trim(line.substr(1, line.size() - 2));
            continue;
        }

        // Key = Value
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key   = Trim(line.substr(0, eq));
        std::string value = Trim(line.substr(eq + 1));

        // Strip inline comment
        auto semi = value.find(';');
        if (semi != std::string::npos) value = Trim(value.substr(0, semi));
        auto hash = value.find('#');
        if (hash != std::string::npos) value = Trim(value.substr(0, hash));

        m_Values[MakeKey(currentSection, key)] = value;
    }
    return true;
}

bool IniFile::Save(const std::string& /*path*/) const {
    // Not needed — we use SaveDefaultConfig() for writing
    return false;
}

std::string IniFile::GetString(const std::string& section, const std::string& key, const std::string& def) const {
    auto it = m_Values.find(MakeKey(section, key));
    return (it != m_Values.end()) ? it->second : def;
}

float IniFile::GetFloat(const std::string& section, const std::string& key, float def) const {
    auto it = m_Values.find(MakeKey(section, key));
    if (it == m_Values.end()) return def;
    try { return std::stof(it->second); }
    catch (...) { return def; }
}

int IniFile::GetInt(const std::string& section, const std::string& key, int def) const {
    auto it = m_Values.find(MakeKey(section, key));
    if (it == m_Values.end()) return def;
    try { return std::stoi(it->second); }
    catch (...) { return def; }
}

bool IniFile::GetBool(const std::string& section, const std::string& key, bool def) const {
    auto it = m_Values.find(MakeKey(section, key));
    if (it == m_Values.end()) return def;
    std::string v = ToLower(it->second);
    if (v == "true" || v == "1" || v == "yes" || v == "on") return true;
    if (v == "false" || v == "0" || v == "no" || v == "off") return false;
    return def;
}

// ============================================================
//  RagdollConfig::LoadFromIni
// ============================================================
void RagdollConfig::LoadFromIni(const IniFile& ini) {
    // [Physics]
    fGravityZ           = ini.GetFloat("Physics", "fGravityZ", fGravityZ);
    iSolverIterations   = ini.GetInt  ("Physics", "iSolverIterations", iSolverIterations);
    fSolverERP          = ini.GetFloat("Physics", "fSolverERP", fSolverERP);
    iMaxSubSteps        = ini.GetInt  ("Physics", "iMaxSubSteps", iMaxSubSteps);
    fFixedTimeStep      = ini.GetFloat("Physics", "fFixedTimeStep", fFixedTimeStep);
    fMaxDeltaTime       = ini.GetFloat("Physics", "fMaxDeltaTime", fMaxDeltaTime);
    fSimulationSpeed    = ini.GetFloat("Physics", "fSimulationSpeed", fSimulationSpeed);

    // [Ragdoll]
    fLinearDamping        = ini.GetFloat("Ragdoll", "fLinearDamping", fLinearDamping);
    fAngularDamping       = ini.GetFloat("Ragdoll", "fAngularDamping", fAngularDamping);
    fFriction             = ini.GetFloat("Ragdoll", "fFriction", fFriction);
    fRestitution          = ini.GetFloat("Ragdoll", "fRestitution", fRestitution);
    fConstraintSoftness   = ini.GetFloat("Ragdoll", "fConstraintSoftness", fConstraintSoftness);
    fConstraintBias       = ini.GetFloat("Ragdoll", "fConstraintBias", fConstraintBias);
    fConstraintRelaxation = ini.GetFloat("Ragdoll", "fConstraintRelaxation", fConstraintRelaxation);
    fMaxBoneSpeed         = ini.GetFloat("Ragdoll", "fMaxBoneSpeed", fMaxBoneSpeed);
    fStretchTolerance     = ini.GetFloat("Ragdoll", "fStretchTolerance", fStretchTolerance);
    iDistanceIterations   = ini.GetInt  ("Ragdoll", "iDistanceIterations", iDistanceIterations);
    fConstraintDamping    = ini.GetFloat("Ragdoll", "fConstraintDamping", fConstraintDamping);
    iConstraintSolverIter = ini.GetInt  ("Ragdoll", "iConstraintSolverIter", iConstraintSolverIter);
    fShapeMargin          = ini.GetFloat("Ragdoll", "fShapeMargin", fShapeMargin);
    fCCDThreshold         = ini.GetFloat("Ragdoll", "fCCDThreshold", fCCDThreshold);
    fCCDSweptSphereRadius = ini.GetFloat("Ragdoll", "fCCDSweptSphereRadius", fCCDSweptSphereRadius);

    // [World]
    fCollisionLoadRadius     = ini.GetFloat("World", "fCollisionLoadRadius", fCollisionLoadRadius);
    iMaxEntities             = ini.GetInt  ("World", "iMaxEntities", iMaxEntities);
    iMaxTrianglesPerCall     = ini.GetInt  ("World", "iMaxTrianglesPerCall", iMaxTrianglesPerCall);
    iCollisionUpdateInterval = ini.GetInt  ("World", "iCollisionUpdateInterval", iCollisionUpdateInterval);
    fSurfaceFriction         = ini.GetFloat("World", "fSurfaceFriction", fSurfaceFriction);
    fSurfaceRestitution      = ini.GetFloat("World", "fSurfaceRestitution", fSurfaceRestitution);
    fWorldShapeMargin        = ini.GetFloat("World", "fWorldShapeMargin", fWorldShapeMargin);
    fWorldMinX               = ini.GetFloat("World", "fWorldMinX", fWorldMinX);
    fWorldMaxX               = ini.GetFloat("World", "fWorldMaxX", fWorldMaxX);
    fWorldMinY               = ini.GetFloat("World", "fWorldMinY", fWorldMinY);
    fWorldMaxY               = ini.GetFloat("World", "fWorldMaxY", fWorldMaxY);

    // [Drag]
    fDragSpring      = ini.GetFloat("Drag", "fDragSpring", fDragSpring);
    fDragDamping     = ini.GetFloat("Drag", "fDragDamping", fDragDamping);
    fDragMaxVelocity = ini.GetFloat("Drag", "fDragMaxVelocity", fDragMaxVelocity);
    fDragDistance    = ini.GetFloat("Drag", "fDragDistance", fDragDistance);

    // [Reactions]
    fBulletPower    = ini.GetFloat("Reactions", "fBulletPower", fBulletPower);
    fExplosionPower = ini.GetFloat("Reactions", "fExplosionPower", fExplosionPower);
    fGetUpThreshold = ini.GetFloat("Reactions", "fGetUpThreshold", fGetUpThreshold);
    fGetUpDelay     = ini.GetFloat("Reactions", "fGetUpDelay", fGetUpDelay);

    // [Vehicle]
    bVehicleCollision   = ini.GetBool ("Vehicle", "bVehicleCollision", bVehicleCollision);
    bUseSpheresShape    = ini.GetBool ("Vehicle", "bUseSpheresShape", bUseSpheresShape);
    fVehicleFriction    = ini.GetFloat("Vehicle", "fFriction", fVehicleFriction);
    fVehicleRestitution = ini.GetFloat("Vehicle", "fRestitution", fVehicleRestitution);

    // [DynamicObjects]
    bDynamicObjects           = ini.GetBool ("DynamicObjects", "bEnabled", bDynamicObjects);
    fDynamicObjectMass        = ini.GetFloat("DynamicObjects", "fMass", fDynamicObjectMass);
    fDynamicObjectFriction    = ini.GetFloat("DynamicObjects", "fFriction", fDynamicObjectFriction);
    fDynamicObjectRestitution = ini.GetFloat("DynamicObjects", "fRestitution", fDynamicObjectRestitution);
}

// ============================================================
//  Path resolution: INI sits next to the ASI DLL
// ============================================================
std::string GetConfigPath() {
    char path[MAX_PATH]{};
    HMODULE hm = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                       GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCSTR>(&g_Config), &hm);
    GetModuleFileNameA(hm, path, MAX_PATH);

    std::string s(path);
    auto dot = s.rfind('.');
    if (dot != std::string::npos) s = s.substr(0, dot);
    return s + ".ini";
}

// ============================================================
//  SaveDefaultConfig — writes a commented INI with all defaults
// ============================================================
void SaveDefaultConfig() {
    std::string path = GetConfigPath();
    std::ofstream f(path);
    if (!f.is_open()) return;

    f << "; =============================================================\n";
    f << "; RagdollPhysics.ini - Bullet Ragdoll Physics Configuration\n";
    f << "; Lines starting with ; or # are comments\n";
    f << "; Reload in-game with ALT+0\n";
    f << "; =============================================================\n\n";

    f << "[Physics]\n";
    f << "fGravityZ = -9.81\n";
    f << "iSolverIterations = 30\n";
    f << "fSolverERP = 0.8\n";
    f << "iMaxSubSteps = 10\n";
    f << "fFixedTimeStep = 0.00833\n";
    f << "fMaxDeltaTime = 0.05\n";
    f << "fSimulationSpeed = 1.0\n\n";

    f << "[Ragdoll]\n";
    f << "fLinearDamping = 0.4\n";
    f << "fAngularDamping = 0.85\n";
    f << "fFriction = 0.75\n";
    f << "fRestitution = 0.05\n";
    f << "fConstraintSoftness = 0.6\n";
    f << "fConstraintBias = 0.6\n";
    f << "fConstraintRelaxation = 1.0\n";
    f << "fMaxBoneSpeed = 15.0\n";
    f << "fStretchTolerance = 1.05\n";
    f << "iDistanceIterations = 6\n";
    f << "fConstraintDamping = 0.5\n";
    f << "iConstraintSolverIter = 50\n";
    f << "fShapeMargin = 0.04\n";
    f << "fCCDThreshold = 0.1\n";
    f << "fCCDSweptSphereRadius = 0.05\n\n";

    f << "[World]\n";
    f << "fCollisionLoadRadius = 100.0\n";
    f << "iMaxEntities = 80\n";
    f << "iMaxTrianglesPerCall = 2000\n";
    f << "iCollisionUpdateInterval = 60\n";
    f << "fSurfaceFriction = 0.8\n";
    f << "fSurfaceRestitution = 0.1\n";
    f << "fWorldShapeMargin = 0.04\n";
    f << "fWorldMinX = -3000.0\n";
    f << "fWorldMaxX = 3000.0\n";
    f << "fWorldMinY = -3000.0\n";
    f << "fWorldMaxY = 3000.0\n\n";

    f << "[Drag]\n";
    f << "fDragSpring = 80.0\n";
    f << "fDragDamping = 0.7\n";
    f << "fDragMaxVelocity = 30.0\n";
    f << "fDragDistance = 6.0\n\n";

    f << "[Reactions]\n";
    f << "fBulletPower = 5.0\n";
    f << "fExplosionPower = 15.0\n";
    f << "fGetUpThreshold = 0.5\n";
    f << "fGetUpDelay = 3.0\n\n";

    f << "[Vehicle]\n";
    f << "bVehicleCollision = true\n";
    f << "bUseSpheresShape = false\n";
    f << "fFriction = 0.6\n";
    f << "fRestitution = 0.2\n\n";

    f << "[DynamicObjects]\n";
    f << "bEnabled = false\n";
    f << "fMass = 10.0\n";
    f << "fFriction = 0.5\n";
    f << "fRestitution = 0.1\n";
}

// ============================================================
//  LoadConfig — main entry point
// ============================================================
void LoadConfig() {
    std::string path = GetConfigPath();

    IniFile ini;
    if (!ini.Load(path)) {
        // File doesn't exist — write defaults
        SaveDefaultConfig();
        // Re-load to populate g_Config (or just keep defaults)
        g_Config = RagdollConfig{};
        return;
    }

    g_Config = RagdollConfig{};   // reset to defaults first
    g_Config.LoadFromIni(ini);
}
