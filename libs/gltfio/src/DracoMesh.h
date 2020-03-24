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

#include <memory>

namespace gltfio {

using DracoMeshHandle = std::unique_ptr<class DracoMesh>;

class DracoMesh {
public:
    static DracoMeshHandle decode(const uint8_t* compressedData, size_t compressedSize);
    bool getAttribute(uint32_t attrId, uint8_t** uncompressedData, size_t* uncompressedSize) const;
    ~DracoMesh();
private:
    DracoMesh(struct DracoMeshDetails* details);
    std::unique_ptr<struct DracoMeshDetails> mDetails;
};

} // namsepace gltfio

#endif // GLTFIO_DRACO_MESH_H
