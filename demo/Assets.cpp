#include "Assets.h"
#include <Mesh.h>
#include <Texture.h>
#include <unordered_map>
#include <glm/glm.hpp>
#include "tiny_gltf.h"

namespace {
    std::unordered_map<std::string, std::pair<unsigned int, unsigned int>> meshes;
    std::unordered_map<std::string, unsigned int> textures;
	tinygltf::TinyGLTF tinyGltf;
}

static std::pair<unsigned int, unsigned int> readMesh(const std::string &path) {
	tinygltf::Model model;
	std::string err;
	std::string warn;

	bool ret = tinyGltf.LoadBinaryFromFile(&model, &err, &warn, path);

	if (!warn.empty()) {
		printf("Warn: %s\n", warn.c_str());
	}

	if (!err.empty()) {
		printf("Err: %s\n", err.c_str());
	}

	if (!ret) {
		printf("Failed to parse glTF\n");
		return {};
	}

	tinygltf::Primitive primitive = model.meshes[0].primitives[0];

	//indices
	const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
	const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
	const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];

	const unsigned short* indicesData = reinterpret_cast<const unsigned short*>(&indexBuffer.data[indexBufferView.byteOffset + indexAccessor.byteOffset]);

	std::vector<unsigned int> indices(indicesData, indicesData + indexAccessor.count);

	//vertex data
	const tinygltf::Accessor& positionAccessor = model.accessors[primitive.attributes["POSITION"]];
	const tinygltf::BufferView& positionBufferView = model.bufferViews[positionAccessor.bufferView];
	const tinygltf::Buffer& positionBuffer = model.buffers[positionBufferView.buffer];

	const float* positions = reinterpret_cast<const float*>(&positionBuffer.data[positionBufferView.byteOffset + positionAccessor.byteOffset]);

	const tinygltf::Accessor& normalAccessor = model.accessors[primitive.attributes["NORMAL"]];
	const tinygltf::BufferView& normalBufferView = model.bufferViews[normalAccessor.bufferView];
	const tinygltf::Buffer& normalBuffer = model.buffers[normalBufferView.buffer];

	const float* normals = reinterpret_cast<const float*>(&normalBuffer.data[normalBufferView.byteOffset + normalAccessor.byteOffset]);

	const tinygltf::Accessor& texCoordAccessor = model.accessors[primitive.attributes["TEXCOORD_0"]];
	const tinygltf::BufferView& texCoordBufferView = model.bufferViews[texCoordAccessor.bufferView];
	const tinygltf::Buffer& texCoordBuffer = model.buffers[texCoordBufferView.buffer];

	const float* texCoords = reinterpret_cast<const float*>(&texCoordBuffer.data[texCoordBufferView.byteOffset + texCoordAccessor.byteOffset]);

	std::vector<Vertex> vertices;

	for (int i = 0; i < positionAccessor.count; ++i) {
		glm::vec3 position = glm::vec3(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
		glm::vec3 normal = glm::vec3(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]);
		glm::vec2 texCoord = glm::vec2(texCoords[i * 2 + 0], texCoords[i * 2 + 1]);

		vertices.push_back({ position, normal, texCoord });
	}

	return { createMesh(vertices, indices), (unsigned int) indices.size() };
}

void Assets::loadMesh(const std::string &name) {
	meshes[name] = readMesh("meshes/" + name + ".glb");
}

void Assets::loadTexture(const std::string &name) {
	textures[name] = createTexture(name);
}

std::pair<unsigned int, unsigned> Assets::getMesh(const std::string &name) {
	return meshes.at(name);
}

unsigned int Assets::getTexture(const std::string &name) {
	return textures.at(name);
}
