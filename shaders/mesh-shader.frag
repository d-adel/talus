#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"
#include "scene-resources.glsl"
#include "push-constants.glsl"

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragNormal;

layout(location = 0) out vec4 outColor;

void main() {
    Material material = materialTable.materials[push.materialIndex];

    vec3 lightDir = light.direction.xyz;

    vec4 albedo = material.baseColor
        * texture(
            sampler2D(
                textures[material.baseColorTexture],
                samplers[material.baseColorSampler]
            ),
            fragTexCoord
        );
    vec3 ambient = vec3(0.005, 0.005, 0.005);
    vec3 diffuse = albedo.rgb * max(0.0, dot(normalize(fragNormal), lightDir));
    outColor = vec4(ambient + diffuse, 1);
}
