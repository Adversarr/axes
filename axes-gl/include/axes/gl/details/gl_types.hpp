#pragma once
#include <glad/glad.h>
namespace ax::gl {

enum class Type : GLenum {
  kBool = GL_BOOL,
  kByte = GL_BYTE,
  kUnsignedByte = GL_UNSIGNED_BYTE,
  kShort = GL_SHORT,
  kUnsignedShort = GL_UNSIGNED_SHORT,
  kInt = GL_INT,
  kUnsignedInt = GL_UNSIGNED_INT,
  kFixed = GL_FIXED,
  kHalfFloat = GL_HALF_FLOAT,
  kFloat = GL_FLOAT,
  kDouble = GL_DOUBLE,
};

enum class BufferBindingType : GLenum {
  kArray = GL_ARRAY_BUFFER,
  kElementArray = GL_ELEMENT_ARRAY_BUFFER,
  kUniform = GL_UNIFORM_BUFFER,
  kShaderStorage = GL_SHADER_STORAGE_BUFFER,
  kAtomicCounter = GL_ATOMIC_COUNTER_BUFFER,
  kDispatchIndirect = GL_DISPATCH_INDIRECT_BUFFER,
  kDrawIndirect = GL_DRAW_INDIRECT_BUFFER,
  kPixelPack = GL_PIXEL_PACK_BUFFER,
  kPixelUnpack = GL_PIXEL_UNPACK_BUFFER,
  kQuery = GL_QUERY_BUFFER,
  kTexture = GL_TEXTURE_BUFFER,
  kTransformFeedback = GL_TRANSFORM_FEEDBACK_BUFFER,
  kCopyRead = GL_COPY_READ_BUFFER,
  kCopyWrite = GL_COPY_WRITE_BUFFER,
};

enum class BufferUsage : GLenum {
  kStreamDraw = GL_STREAM_DRAW,
  kStreamRead = GL_STREAM_READ,
  kStreamCopy = GL_STREAM_COPY,
  kStaticDraw = GL_STATIC_DRAW,
  kStaticRead = GL_STATIC_READ,
  kStaticCopy = GL_STATIC_COPY,
  kDynamicDraw = GL_DYNAMIC_DRAW,
  kDynamicRead = GL_DYNAMIC_READ,
  kDynamicCopy = GL_DYNAMIC_COPY,
};

enum class ShaderType : GLenum {
  kVertex = GL_VERTEX_SHADER,
  kFragment = GL_FRAGMENT_SHADER,
  kGeometry = GL_GEOMETRY_SHADER,
  kTessControl = GL_TESS_CONTROL_SHADER,
  kTessEvaluation = GL_TESS_EVALUATION_SHADER,
  kCompute = GL_COMPUTE_SHADER,
};

enum class PrimitiveType : GLenum {
  kPoints = GL_POINTS,
  kLines = GL_LINES,
  kLineLoop = GL_LINE_LOOP,
  kLineStrip = GL_LINE_STRIP,
  kTriangles = GL_TRIANGLES,
  kTriangleStrip = GL_TRIANGLE_STRIP,
  kTriangleFan = GL_TRIANGLE_FAN,
  kLinesAdjacency = GL_LINES_ADJACENCY,
  kLineStripAdjacency = GL_LINE_STRIP_ADJACENCY,
  kTrianglesAdjacency = GL_TRIANGLES_ADJACENCY,
  kTriangleStripAdjacency = GL_TRIANGLE_STRIP_ADJACENCY,
  kPatches = GL_PATCHES,
};

}  // namespace ax::gl
