#include "ContactManifoldClustering.h"

#include <glm/gtx/norm.inl>

namespace {
    struct ContactIndex {
        int manifoldIndex;
        int contactPointIndex;
    };

    thread_local std::vector<ContactIndex> centroidContacts[3];
}

void physecs::clusterContactManifolds(std::vector<ContactManifold> &manifolds) {

    // 3-means clustering by contact normal

    glm::vec3 centroids[3] = {
        glm::vec3(1, 0, 0),
        glm::vec3(0, 1, 0),
        glm::vec3(0, 0, 1)
    };

    constexpr int numIterations = 3;

    for (int i = 0; i < numIterations; ++i) {
        // assign manifolds
        for (int j = 0; j < manifolds.size(); ++j) {
            auto& manifold = manifolds[j];
            int maxIndex = 0;
            float maxDot = -1.f;
            for (int k = 0; k < 3; ++k) {
                const float dot = glm::dot(centroids[k], manifold.normal);
                if (dot > maxDot) {
                    maxDot = dot;
                    maxIndex = k;
                }
            }
            manifold.triangleIndex = maxIndex;
        }

        // update centroids
        for (int k = 0; k < 3; ++k) {
            centroids[k] = glm::vec3(0);
        }

        for (int j = 0; j < manifolds.size(); ++j) {
            centroids[manifolds[j].triangleIndex] += manifolds[j].normal;
        }

        for (int k = 0; k < 3; ++k) {
            if (glm::length2(centroids[k]) > 0.f) {
                centroids[k] = glm::normalize(centroids[k]);
            }
        }
    }

    // merge centroids

    int centroidIndices[3] = { 0, 1, 2 };
    int numCentroids = 3;

    constexpr float almostAligned = 0.9f;

    if (glm::dot(centroids[0], centroids[1]) > almostAligned) {
        centroidIndices[1] = 0;
        centroidIndices[2] = 1;

        centroids[0] = glm::normalize(centroids[0] + centroids[1]);

        --numCentroids;
    }

    if (glm::dot(centroids[0], centroids[2]) > almostAligned) {
        centroidIndices[2] = 0;

        centroids[0] = glm::normalize(centroids[0] + centroids[2]);

        --numCentroids;
    }

    if (numCentroids == 3 && glm::dot(centroids[1], centroids[2]) > almostAligned) {
        centroidIndices[2] = 1;

        centroids[1] = glm::normalize(centroids[1] + centroids[2]);

        --numCentroids;
    }

    ContactManifold newManifolds[3];

    for (int i = 0; i < numCentroids; ++i) {
        auto& centroidContactsList = centroidContacts[i];

        centroidContactsList.clear();

        for (int j = 0; j < manifolds.size(); ++j) {
            const auto& manifold = manifolds[j];
            if (centroidIndices[manifold.triangleIndex] == i) {
                for (int k = 0; k < manifold.numPoints; ++k)
                    centroidContactsList.push_back({ j, k });
            }
        }

        if (centroidContactsList.size() > 4) {
            // reduce contacts

            const auto& firstContact = manifolds[centroidContactsList[0].manifoldIndex].points[centroidContactsList[0].contactPointIndex];
            int firstContactOnHullIndex = 0;
            float maxDistance = 0.f;
            for (int j = 0; j < centroidContactsList.size(); ++j) {
                const auto& contactIndices = centroidContactsList[j];
                const auto& contact = manifolds[contactIndices.manifoldIndex].points[contactIndices.contactPointIndex];
                const float distance = glm::distance2(firstContact.position1, contact.position1);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    firstContactOnHullIndex = j;
                }
            }

            const auto& firstContactOnHull = manifolds[centroidContactsList[firstContactOnHullIndex].manifoldIndex].points[centroidContactsList[firstContactOnHullIndex].contactPointIndex];
            int maxDistanceIndex = 0;
            for (int j = 0; j < centroidContactsList.size(); ++j) {
                const auto& contactIndices = centroidContactsList[j];
                const auto& contact = manifolds[contactIndices.manifoldIndex].points[contactIndices.contactPointIndex];
                const float distance = glm::distance2(firstContactOnHull.position1, contact.position1);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxDistanceIndex = j;
                }
            }

            const auto& maxDistanceContact = manifolds[centroidContactsList[maxDistanceIndex].manifoldIndex].points[centroidContactsList[maxDistanceIndex].contactPointIndex];

            glm::vec3 d = maxDistanceContact.position1 - firstContactOnHull.position1;
            glm::vec3 n = centroids[i];

            glm::vec3 side = glm::normalize(glm::cross(n, d));
            glm::vec3 fwd = glm::cross(side, n);

            glm::mat3 clusterFrame = glm::mat3(side, n, fwd);
            glm::mat3 worldToCluster = glm::transpose(clusterFrame);

            int minXIndex = 0, maxXIndex = 0;
            for (int j = 0; j < centroidContactsList.size(); ++j) {
                const auto& contactIndices = centroidContactsList[j];
                const auto& minXIndices = centroidContactsList[minXIndex];
                const auto& maxXIndices = centroidContactsList[maxXIndex];

                const auto& contact = manifolds[contactIndices.manifoldIndex].points[contactIndices.contactPointIndex];
                const auto& minXContact = manifolds[minXIndices.manifoldIndex].points[minXIndices.contactPointIndex];
                const auto& maxXContact = manifolds[maxXIndices.manifoldIndex].points[maxXIndices.contactPointIndex];

                if ((worldToCluster * contact.position1).x > (worldToCluster * maxXContact.position1).x) {
                    maxXIndex = j;
                }
                if ((worldToCluster * contact.position1).x < (worldToCluster * minXContact.position1).x) {
                    minXIndex = j;
                }
            }

            auto& newManifold = newManifolds[i];
            newManifold.numPoints = 4;
            newManifold.normal = n;
            newManifold.triangleIndex = i;
            newManifold.points[0] = firstContactOnHull;
            newManifold.points[1] = maxDistanceContact;
            newManifold.points[2] = manifolds[centroidContactsList[minXIndex].manifoldIndex].points[centroidContactsList[minXIndex].contactPointIndex];
            newManifold.points[3] = manifolds[centroidContactsList[maxXIndex].manifoldIndex].points[centroidContactsList[maxXIndex].contactPointIndex];
        }
        else {
            auto& newManifold = newManifolds[i];
            newManifold.numPoints = centroidContactsList.size();
            newManifold.normal = centroids[i];
            newManifold.triangleIndex = i;
            for (int j = 0; j < centroidContactsList.size(); ++j) {
                const auto& contactIndices = centroidContactsList[j];
                const auto& contact = manifolds[contactIndices.manifoldIndex].points[contactIndices.contactPointIndex];
                newManifold.points[j] = contact;
            }
        }
    }

    manifolds.clear();
    for (int i = 0; i < numCentroids; ++i) {
        manifolds.push_back(newManifolds[i]);
    }
}
