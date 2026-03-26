#pragma once
#include "rwcore.h"
#include <btBulletDynamicsCommon.h>

// RenderWare matrix convention:
// Right
// Up
// At
//
// Bullet btMatrix3x3 constructor is row-major: (r0c0, r0c1, r0c2, r1c0, ...).
// RW's right/at/up are COLUMN vectors (basis axes). To place them as columns
// in btMatrix3x3, each row becomes (right[i], at[i], up[i]).

// struct RwMatrixTag
// {
//     /* These are padded to be 16 byte quantities per line */
//     RwV3d               right;
//     RwUInt32            flags;
//     RwV3d               up;
//     RwUInt32            pad1;
//     RwV3d               at;
//     RwUInt32            pad2;
//     RwV3d               pos;
//     RwUInt32            pad3;
// };

inline btTransform RwMatrixToBtTransform(const RwMatrix& m) {
    btMatrix3x3 basis(
        m.right.x, m.at.x, m.up.x,
        m.right.y, m.at.y, m.up.y,
        m.right.z, m.at.z, m.up.z
    );

    btTransform t;
    t.setBasis(basis);
    t.setOrigin(btVector3(m.pos.x, m.pos.y, m.pos.z));
    return t;
}

inline void BtTransformToRwMatrix(const btTransform& t, RwMatrix& m) {
    const btMatrix3x3& b = t.getBasis();
    const btVector3&   p = t.getOrigin();



    m.pos.x = p.x(); m.pos.y = p.y(); m.pos.z = p.z();
    m.flags  = 0;
}

inline void glMatrixToRwMatrix(const float* glMat, RwMatrix& rwMat) {
    // -0.605069, -0.796173, 5.84424e-05, 0
    // -4.21107e-05, -4.13656e-05, -1, 0
    // 0.796173, -0.605069, -8.46386e-06, 0
    // 2510.75, -1660.86, 12.6074, 1
    rwMat.right.x = glMat[0]; rwMat.at.x = glMat[1]; rwMat.up.x = glMat[2];
    rwMat.right.y = glMat[4]; rwMat.at.y = glMat[5]; rwMat.up.y = glMat[6];
    rwMat.right.z = glMat[8]; rwMat.at.z = glMat[9]; rwMat.up.z = glMat[10];

    // glMat[3], glMat[7], glMat[11] are just padding for OpenGL's column-major format, so we ignore them.
}