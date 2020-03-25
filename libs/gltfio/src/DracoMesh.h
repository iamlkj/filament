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

#ifndef GLTFIO_DRACO_MESH_H
#define GLTFIO_DRACO_MESH_H

#include <cgltf.h>

#include <memory>

namespace gltfio {

using DracoMeshHandle = std::unique_ptr<class DracoMesh>;

// The class decodes a Draco mesh upon construction and retains the results.
// For convenience, each decoded attribute is exposed as a cgltf_buffer_view.
class DracoMesh {
public:
    static DracoMeshHandle decode(const uint8_t* compressedData, size_t compressedSize);
    void getFaceIndices(cgltf_accessor* destination) const;
    bool getVertexAttributes(uint32_t attributeId, cgltf_accessor* destination);
    ~DracoMesh();
private:
    DracoMesh(struct DracoMeshDetails* details);
    std::unique_ptr<struct DracoMeshDetails> mDetails;
};

} // namespace gltfio

#endif // GLTFIO_DRACO_MESH_H
