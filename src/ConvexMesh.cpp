#include "ConvexMesh.h"
#include <unordered_map>
//#include "QuickHull.h"

// CustomPhysics::ConvexMesh::ConvexMesh(const std::vector<glm::vec3> &pointCloud) {
//     QhConvex convex(pointCloud);
//
//     std::vector<int> indices;
//     for (int i = 0; i < convex.getVertices().size(); ++i) {
//         auto vertex = convex.getVertices()[i];
//         if (vertex.getRefCount()) indices.push_back(i);
//     }
//
//     vertices = std::vector<glm::vec3>(indices.size());
//     std::unordered_map<int, int> indexMap;
//     for (int i = 0; i < indices.size(); ++i) {
//         vertices[i] = convex.getVertices()[indices[i]].getPosition();
//         indexMap[indices[i]] = i;
//     }
//
//     faces = std::vector<ConvexMeshFace>(convex.getFaceCount());
//     auto face = convex.getFaceListHead();
//     for (int i = 0; i < convex.getFaceCount(); ++i) {
//         std::vector<int> faceIndices(face->getNumVerts());
//         auto edge = face->getEdge();
//         for (int j = 0; j < face->getNumVerts(); ++j) {
//             faceIndices[j] = indexMap.at(edge->getTail() - convex.getVertices().data());
//             edge = edge->getNext();
//         }
//         auto normal = face->getNormal();
//         auto centroid = face->getCentroid();
//         faces[i] = { faceIndices, normal, centroid };
//         face = face->getNext();
//     }
// }
