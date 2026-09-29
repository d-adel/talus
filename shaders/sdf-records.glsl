#ifndef TALUS_SDF_RECORDS_GLSL
#define TALUS_SDF_RECORDS_GLSL

#include "sdf-types.glsl"

layout(std140, set = 2, binding = 0) readonly buffer SdfRecordTable {
  SdfRecordShared records[];
} sdfRecordTable;

#endif
