#pragma once

#include LIGHT_COUNT

struct PointLight
{
    vec4 position;
    vec3 color;
    float strength;
};

layout(std140) uniform PointLightUniforms {
    PointLight PointLightArray[LIGHT_COUNT];
};

#include LIGHT_GETTER
