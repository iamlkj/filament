/*
 * Copyright (C) 2020 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DracoMesh.h"

#include <draco/compression/decode.h>

#include <utils/Log.h>

#include <vector>

using std::unique_ptr;
using std::vector;

using namespace utils;

namespace gltfio {

struct DracoMeshDetails {
    unique_ptr<draco::Mesh> mesh;
    vector<unique_ptr<cgltf_buffer_view>> views;
    vector<unique_ptr<cgltf_buffer>> buffers;
};

DracoMesh::DracoMesh(DracoMeshDetails* details) : mDetails(details) {}
DracoMesh::~DracoMesh() {}

unique_ptr<DracoMesh> DracoMesh::decode(const uint8_t* data, size_t dataSize) {
    draco::DecoderBuffer buffer;
    buffer.Init((const char*) data, dataSize);
    draco::Decoder decoder;
    const auto geotype = decoder.GetEncodedGeometryType(&buffer);
    if (!geotype.ok()) {
        return nullptr;
    }
    auto meshStatus = decoder.DecodeMeshFromBuffer(&buffer);
    if (!meshStatus.ok()) {
        return nullptr;
    }
    auto wrapper = new DracoMesh(new DracoMeshDetails { std::move(meshStatus).value() });
    DracoMeshDetails* details = wrapper->mDetails.get();
    draco::Mesh* mesh = details->mesh.get();
    return unique_ptr<DracoMesh>(wrapper);
}

void DracoMesh::getFaceIndices(cgltf_accessor* destination) const {
    if (destination->buffer_view) {
        return;
    }

    mDetails->views.emplace_back(new cgltf_buffer_view);
    mDetails->buffers.emplace_back(new cgltf_buffer);

    cgltf_buffer_view* view = mDetails->views.back().get();
    cgltf_buffer* buffer = mDetails->buffers.back().get();

    // TODO: fix memory leaks

    draco::Mesh* mesh = mDetails->mesh.get();
    switch (destination->component_type) {
        case cgltf_component_type_r_16u: {
            const cgltf_size numBytes = mesh->num_faces() * 3 * sizeof(uint16_t);
            uint16_t* indices = new uint16_t[mesh->num_faces() * 3];

            *buffer = {
                .size = numBytes,
                .uri = nullptr,
                .data = (void*) indices
            };

            *view = {
                .buffer = buffer,
                .offset = 0,
                .size = numBytes,
                .stride = {},
                .type = cgltf_buffer_view_type_indices
            };

            for (uint32_t faceId = 0; faceId < mesh->num_faces(); ++faceId) {
                draco::Mesh::Face face = mesh->face(draco::FaceIndex(faceId));
                *indices++ = face[0].value();
                *indices++ = face[1].value();
                *indices++ = face[2].value();
            }

            break;
        }
        case cgltf_component_type_r_32u:
        case cgltf_component_type_r_8u:
            // TODO: convert the component type if necessary.
            assert(false && "Unsupported index type.");
            break;
        default:
            slog.e << "Unknown component type for Draco indices." << io::endl;
            break;
    }

    assert(destination->count == mesh->num_faces() * 3);

    destination->buffer_view = view;
}

bool DracoMesh::getVertexAttributes(uint32_t attributeId, cgltf_accessor* destination) {
    // TODO: implement proper fallback behavior as described here:
    // https://github.com/KhronosGroup/glTF/tree/master/extensions/2.0/Khronos/KHR_draco_mesh_compression#conformance

    // Return early if we've already decompressed this data.
    if (destination->buffer_view) {
        return true;
    }

    // Return early if no such attribute exists.
    draco::Mesh* mesh = mDetails->mesh.get();
    const draco::PointAttribute* attr = mesh->GetAttributeByUniqueId(attributeId);
    if (!attr) {
        return false;
    }

    mDetails->views.emplace_back(new cgltf_buffer_view);
    mDetails->buffers.emplace_back(new cgltf_buffer);

    cgltf_buffer_view* view = mDetails->views.back().get();
    cgltf_buffer* buffer = mDetails->buffers.back().get();

    *buffer = {
        .size = attr->buffer()->data_size(),
        .uri = nullptr,
        .data = attr->buffer()->data(),
    };

    *view = {
        .buffer = buffer,
        .offset = (cgltf_size) attr->byte_offset(),
        .size = attr->buffer()->data_size() - (cgltf_size) attr->byte_offset(),
	    .stride = (cgltf_size) attr->byte_stride(),
	    .type = cgltf_buffer_view_type_vertices
    };

    // It is expected that Draco compression can change the vertex count, and on some conformance
    // models the accessor count is the uncompressed (rather than decompressed) count.
    //
    // There is a messy story with respect to conformance and specification. See:
    //      https://github.com/KhronosGroup/glTF-Sample-Models/issues/234
    //      https://github.com/KhronosGroup/glTF-Sample-Models/issues/147
    //
    // To summarize, one day we may able to replace the following line with an assert.
    destination->count = mesh->num_points();

    // Returns the number of components in the given cgltf vector type.
    // Returns -1 if the given type is a matrix.
    auto numComponents = [](cgltf_type ctype) {
        int i = (int) ctype;
        return i <= 4 ? i : -1;
    };

    // TODO: use Draco's ConvertValue facility and remove these asserts.
    assert(attr->data_type() == draco::DT_FLOAT32);
    assert(attr->num_components() == numComponents(destination->type));
    assert(destination->component_type == cgltf_component_type_r_32f);

    destination->buffer_view = view;
    return true;

}

} // namespace gltfio
