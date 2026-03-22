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

// struct RwMatrix
// {
//     RwV3d        right;
//     unsigned int flags;
//     RwV3d        up;
//     unsigned int pad1;
//     RwV3d        at;
//     unsigned int pad2;
//     RwV3d        pos;
//     unsigned int pad3;
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

    // Reverse: extract columns from basis rows
    m.right.x = b[0][0]; m.at.x = b[0][1]; m.up.x = b[0][2];
    m.right.y = b[1][0]; m.at.y = b[1][1]; m.up.y = b[1][2];
    m.right.z = b[2][0]; m.at.z = b[2][1]; m.up.z = b[2][2];

    m.pos.x = p.x(); m.pos.y = p.y(); m.pos.z = p.z();
    m.flags  = 0;
}