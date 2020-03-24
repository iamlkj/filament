#include <memory>

class DracoMesh {
public:
    static DracoMesh decode(const char* compressedData, size_t compressedSize);
    void getAttribute(uint32_t attributeId, uint8_t** uncompressedData, size_t uncompressedSize);
private:
    DracoMesh(DracoMeshDetails* details);
    std::unique_ptr<DracoMeshDetails> mDetails;
};
