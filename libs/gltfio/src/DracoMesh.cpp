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

using std::unique_ptr;

namespace gltfio {

struct DracoMeshDetails {
    unique_ptr<draco::Mesh> mesh;
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
    auto mesh = decoder.DecodeMeshFromBuffer(&buffer);
    if (!mesh.ok()) {
        return nullptr;
    }
    auto retval = new DracoMesh(new DracoMeshDetails { std::move(mesh).value() });
    return unique_ptr<DracoMesh>(retval);
}

bool DracoMesh::getAttribute(uint32_t attrId, uint8_t** uncompressedData,
        size_t* uncompressedSize) const {
    draco::Mesh* mesh = mDetails->mesh.get();
    const draco::PointAttribute* attr = mesh->GetAttributeByUniqueId(attrId);
    if (!attr) {
        return false;
    }
    *uncompressedData = attr->buffer()->data();
    *uncompressedSize = attr->buffer()->data_size();
    return true;
}

} // namespace gltfio
