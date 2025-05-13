#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ConvexMesh.h"
#include "TriangleMesh.h"

namespace physecs {
    enum GeometryType { SPHERE, CAPSULE, BOX, CONVEX_MESH, TRIANGLE_MESH };

    struct SphereGeometry {
        float radius;
    };

    struct CapsuleGeometry {
        float halfHeight;
        float radius;
    };

    struct BoxGeometry {
        glm::vec3 halfExtents;
    };

    struct ConvexMeshGeometry {
        ConvexMesh* mesh;
        glm::vec3 scale;
    };

    struct TriangleMeshGeometry {
        TriangleMesh* mesh;
    };

    struct Geometry {
        GeometryType type;
        union {
            SphereGeometry sphere;
            CapsuleGeometry capsule;
            BoxGeometry box;
            ConvexMeshGeometry convex;
            TriangleMeshGeometry triangleMesh;
        };
    };

    struct Material {
        float friction;
        float restitution;
        float damping;
    };

    struct Collider {
        glm::vec3 position;
        glm::quat orientation;
        Geometry geometry;
        Material material;
        bool isTrigger;
        bool enableSimulation;
        int data;
    };
}
