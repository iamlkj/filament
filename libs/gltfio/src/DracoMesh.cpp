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

#include <tsl/robin_map.h>

#include <vector>

using std::unique_ptr;
using std::vector;
using tsl::robin_map;

namespace gltfio {

struct DracoMeshDetails {
    unique_ptr<draco::Mesh> mesh;
    robin_map<uint32_t, cgltf_buffer_view*> attributes;
    vector<cgltf_buffer_view> views;
    vector<cgltf_buffer> buffers;
    cgltf_buffer_view indicesView;
    cgltf_buffer indicesBuffer;
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

    // Reserve space for all the cgltf wrappers that we'll use for the decompressed data.
    details->views.resize(mesh->num_attributes());
    details->buffers.resize(mesh->num_attributes());

    // Populate the index buffer wrapper if there are any face indices in the Draco mesh.
    if (mesh->num_faces() > 0) {
        const cgltf_size numBytes = mesh->num_faces() * 3 * sizeof(uint32_t);
        details->indicesBuffer = {
            .size = numBytes,
            .uri = nullptr,
            .data = (void*) &mesh->face(draco::FaceIndex(0))[0]
        };
        details->indicesView = {
            .buffer = &details->indicesBuffer,
            .offset = 0,
            .size = numBytes,
            .stride = {},
            .type = cgltf_buffer_view_type_indices
        };

        // TODO: lazily create a uint32 or uint16 buffer.
        uint16_t* dbg = new uint16_t[mesh->num_faces() * 3];
        details->indicesView.size /= 2;
        details->indicesBuffer.size /= 2;
        details->indicesBuffer.data = dbg;
        for (uint32_t faceId = 0; faceId < mesh->num_faces(); ++faceId) {
            draco::Mesh::Face face = mesh->face(draco::FaceIndex(faceId));
            *dbg++ = face[0].value();
            *dbg++ = face[1].value();
            *dbg++ = face[2].value();
        }
    }

    return unique_ptr<DracoMesh>(wrapper);
}

cgltf_buffer_view* DracoMesh::getFaceIndices() const {
    return mDetails->indicesView.buffer ? &mDetails->indicesView : nullptr;
}

cgltf_buffer_view* DracoMesh::getAttribute(uint32_t attributeId) const {
    // Check if we already created a cgltf_buffer_view wrapper.
    auto iter = mDetails->attributes.find(attributeId);
    if (iter != mDetails->attributes.end()) {
        return iter->second;
    }

    // Return early if no such attribute exists.
    draco::Mesh* mesh = mDetails->mesh.get();
    const draco::PointAttribute* attr = mesh->GetAttributeByUniqueId(attributeId);
    if (!attr) {
        return nullptr;
    }

    size_t slot = mDetails->attributes.size();
    assert(slot < mDetails->buffers.size());

    // Grab a free cgltf_buffer wrapper and fill it in.
    cgltf_buffer* buffer = &mDetails->buffers[slot];
    *buffer = {
        .size = attr->buffer()->data_size(),
        .uri = nullptr,
        .data = attr->buffer()->data(),
    };

    // Grab a free cgltf_buffer_view wrapper and fill it in.
    cgltf_buffer_view* view = &mDetails->views[slot];
    *view = {
        .buffer = buffer,
        .offset = (cgltf_size) attr->byte_offset(),
        .size = attr->buffer()->data_size(),
	    .stride = (cgltf_size) attr->byte_stride(),
	    .type = cgltf_buffer_view_type_vertices
    };

    // TODO: look at data_type() and use ConvertValue if necessary

    mDetails->attributes[attributeId] = view;
    return view;
}

} // namespace gltfio
