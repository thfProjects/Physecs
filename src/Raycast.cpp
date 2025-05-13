#include "Raycast.h"

using namespace physecs;

bool intersectRayAABB(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 boxMin, glm::vec3 boxMax, float& t) {
    float tMin = 0;
    float tMax = std::numeric_limits<float>().max();
    for (int i = 0; i < 3; ++i) {
        if (glm::abs(rayDir[i]) < 0.001f) {
            //ray is parallel to axis
            if (rayOrig[i] < boxMin[i] || rayOrig[i] > boxMax[i]) return false;
        }
        else {
            //check if farthest entry is further than nearest exit
            float ood = 1 / rayDir[i];
            float t1 = (boxMin[i] - rayOrig[i]) * ood;
            float t2 = (boxMax[i] - rayOrig[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);

            if (t1 > tMin) tMin = t1;
            if (t2 < tMax) tMax = t2;

            if (tMin > tMax) return false;
        }
    }

    t = tMin;
    return true;
}

static bool intersectRaySphere(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, float radius, float& t) {
    glm::vec3 m = rayOrig - pos;
    float b = glm::dot(m, rayDir);
    float c = glm::dot(m, m) - radius * radius;
    // Exit if ray origin outside sphere (c > 0) and ray pointing away from sphere (b > 0)
    if (c > 0.f && b > 0.f) return false;
    float discr = b*b - c;
    // A negative discriminant corresponds to ray missing sphere
    if (discr < 0.f) return false;
    // Ray now found to intersect sphere, compute smallest t value of intersection
    t = -b - glm::sqrt(discr);
    // If t is negative, ray started inside sphere so clamp t to zero
    if (t < 0.f) t = 0.f;
    return true;
}

static bool intersectRayCapsule(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, glm::quat ori, float halfHeight, float radius, float& t) {
    glm::quat inverseOri = glm::inverse(ori);
    glm::vec3 p = inverseOri * (rayOrig - pos);
    glm::vec3 d = inverseOri * rayDir;

    float pd = glm::dot(p, d);
    float hdy = halfHeight * d.y;
    float thpy = 2 * halfHeight * p.y;
    float phr = glm::dot(p, p) + halfHeight * halfHeight - radius * radius;

    float tMin = std::numeric_limits<float>().max();
    bool hit = false;

    float b, c, discr;

    //case y >= h, top hemisphere
    b = pd - hdy;
    c = phr - thpy;
    discr = b*b - c;
    if (discr >= 0.f) {
        t = -b - glm::sqrt(discr);
        if (t < 0.f) t = 0.f;
        if ((p + t * d).y >= halfHeight) {
            hit = true;
            tMin = t;
        }
    }

    //case -h <= y < h, cylinder
    float a = d.x * d.x + d.z * d.z;
    b = p.x * d.x + p.z * d.z;
    c = p.x * p.x + p.z * p.z - radius * radius;
    discr = b*b - a*c;
    if (discr >= 0.f) {
        t = (-b - glm::sqrt(discr)) / a;
        if (t < 0.f) t = 0.f;
        float y = (p + t * d).y;
        if (-halfHeight <= y && y < halfHeight && t < tMin) {
            hit = true;
            tMin = t;
        }
    }

    //case y < -h, bottom hemisphere
    b = pd + hdy;
    c = phr + thpy;
    discr = b*b - c;
    if (discr >= 0.f) {
        t = -b - glm::sqrt(discr);
        if (t < 0.f) t = 0.f;
        if ((p + t * d).y < -halfHeight && t < tMin) {
            hit = true;
            tMin = t;
        }
    }

    t = tMin;
    return hit;
}

static bool intersectRayBox(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, glm::quat ori, glm::vec3 halfExtents, float& t) {
    glm::quat inverseOri = glm::inverse(ori);
    glm::vec3 p = inverseOri * (rayOrig - pos);
    glm::vec3 d = inverseOri * rayDir;
    return intersectRayAABB(p, d, -halfExtents, halfExtents, t);
}

static bool intersectRayConvexMesh(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, glm::quat ori, ConvexMesh* mesh, glm::vec3 scale, float& t) {
    float tMin = 0;
    float tMax = std::numeric_limits<float>().max();
    for (auto& face : mesh->faces) {
        glm::vec3 planeOrig = pos + ori * (scale * face.centroid);
        glm::vec3 planeNormal = ori * ((glm::vec3(1) / scale) * face.normal);
        float denom = glm::dot(rayDir, planeNormal);
        float dist = glm::dot(planeOrig - rayOrig, planeNormal);
        if (denom == 0.f) {
            //ray is parallel to plane
            if (dist > 0) return false;
        }
        else {
            t = dist / denom;
            if (denom < 0) {
                //ray is entering half-space
                if (t > tMin) tMin = t;
            }
            else {
                //ray is exiting half-space
                if (t < tMax) tMax = t;
            }
            if (tMin > tMax) return false;
        }
    }
    t = tMin;
    return true;
}

bool intersectRayGeometry(glm::vec3 rayOrig, glm::vec3 rayDir, glm::vec3 pos, glm::quat ori, const Geometry &geometry, float &t) {
    switch (geometry.type) {
        case SPHERE: return intersectRaySphere(rayOrig, rayDir, pos, geometry.sphere.radius, t);
        case CAPSULE: return intersectRayCapsule(rayOrig, rayDir, pos, ori, geometry.capsule.halfHeight, geometry.capsule.radius, t);
        case BOX: return intersectRayBox(rayOrig, rayDir, pos, ori, geometry.box.halfExtents, t);
        case CONVEX_MESH: return intersectRayConvexMesh(rayOrig, rayDir, pos, ori, geometry.convex.mesh, geometry.convex.scale, t);
        default: return false;
    }
}
