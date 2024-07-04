/*
 * Copyright (c) 2021, 117 <https://twitter.com/117scape>
 * Copyright (c) 2023, Hooder <ahooder@protonmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include utils/constants.glsl

const float bias = 0.002;
const float lightSize = 0.001 * 4;
const int shadowSamples = 32;

#if SHADOW_MODE != SHADOW_MODE_OFF
void getShadowDepthAndAlpha(float shadowMapTexel, out float shadowDepth, out float shadowAlpha) {
    int alphaDepth = int(shadowMapTexel * SHADOW_COMBINED_MAX);
    float depth = float(alphaDepth & SHADOW_DEPTH_MAX) / SHADOW_DEPTH_MAX;
    float alpha = 1.0 - float(alphaDepth >> SHADOW_DEPTH_BITS) / SHADOW_ALPHA_MAX;

    shadowDepth = depth;
    shadowAlpha = alpha;
}

// Pre-defined set of sample points for blocker search and PCF
const vec2 poissonDisk[64] = vec2[](
    vec2(-0.499557, 0.035246), vec2(0.227272, -0.179687),
    vec2(0.171875, 0.40625), vec2(-0.132812, -0.375),
    vec2(0.453125, -0.007812), vec2(-0.367188, -0.296875),
    vec2(-0.421875, 0.242188), vec2(0.375, 0.257812),
    vec2(-0.25, -0.039062), vec2(0.296875, -0.40625),
    vec2(-0.085938, 0.117188), vec2(0.140625, -0.4375),
    vec2(-0.492188, -0.15625), vec2(0.226562, 0.132812),
    vec2(-0.335938, 0.429688), vec2(0.46875, -0.210938),
    vec2(0.078125, 0.046875), vec2(-0.210938, 0.085938),
    vec2(0.054688, 0.273438), vec2(-0.257812, -0.132812),
    vec2(0.320312, 0.40625), vec2(-0.492188, 0.382812),
    vec2(-0.09375, -0.492188), vec2(0.375, 0.039062),
    vec2(0.015625, -0.296875), vec2(-0.179688, 0.257812),
    vec2(0.46875, -0.328125), vec2(-0.273438, -0.40625),
    vec2(0.429688, 0.164062), vec2(-0.351562, 0.09375),
    vec2(-0.101562, 0.492188), vec2(0.132812, -0.203125),
    vec2(-0.445312, -0.46875), vec2(0.3125, -0.085938),
    vec2(-0.117188, -0.273438), vec2(0.234375, 0.28125),
    vec2(-0.023438, 0.445312), vec2(0.492188, -0.492188),
    vec2(-0.210938, -0.484375), vec2(0.367188, -0.1875),
    vec2(-0.4375, 0.03125), vec2(0.203125, -0.070312),
    vec2(0.070312, 0.140625), vec2(-0.164062, -0.117188),
    vec2(0.28125, -0.3125), vec2(-0.03125, 0.351562),
    vec2(0.375, 0.375), vec2(-0.492188, -0.375),
    vec2(0.140625, 0.476562), vec2(-0.3125, 0.1875),
    vec2(0.46875, -0.078125), vec2(-0.25, -0.234375),
    vec2(0.09375, 0.40625), vec2(-0.367188, -0.007812),
    vec2(0.445312, 0.320312), vec2(-0.15625, 0.367188),
    vec2(-0.46875, -0.210938), vec2(0.3125, -0.429688),
    vec2(-0.085938, 0.273438), vec2(0.234375, -0.234375),
    vec2(-0.320312, 0.351562), vec2(0.476562, -0.46875),
    vec2(-0.273438, 0.125), vec2(0.078125, -0.015625)
);

// Function to perform optimized blocker search
float findBlocker(vec4 projCoords, float currentDepth, float searchRadius) {
    float blockerDepthSum = 0.0;
    int blockerCount = 0;

    for (int i = 0; i < shadowSamples; i++) {
        vec2 offset = poissonDisk[i] * searchRadius;
        float shadowSample = texture(shadowMap, projCoords.xy + offset).r;
        float depth;
        float alpha;
        getShadowDepthAndAlpha(shadowSample, depth, alpha);
        if (depth < currentDepth) {
            blockerDepthSum += depth;
            blockerCount++;
        }
    }

    if (blockerCount == 0) return -1.0;
    return blockerDepthSum / float(blockerCount);
}

// Function to calculate shadow with PCSS
float pcss(vec4 projCoords, float currentDepth, float penumbraSize) {
    float shadow = 0.0;

    for (int i = 0; i < shadowSamples; i++) {
        vec2 offset = poissonDisk[i] * penumbraSize;
        float shadowSample = texture(shadowMap, projCoords.xy + offset).r;
        float depth;
        float alpha;
        getShadowDepthAndAlpha(shadowSample, depth, alpha);

        if (currentDepth > depth)
            shadow += alpha;
    }

    return shadow / float(shadowSamples);
}

float renderPCSSShadows(vec4 projCoords, float fadeOut) {
    vec2 shadowRes = textureSize(shadowMap, 0);
    float currentDepth = projCoords.z - bias;

    float shadowRenderDistance = SHADOW_MAX_DISTANCE / shadowDistance;

    // Blocker search to find average blocker depth
    float searchRadius = lightSize * shadowRenderDistance;
    float blockerDepth = findBlocker(projCoords, currentDepth, searchRadius);

    if (blockerDepth <= -1.0)
        return 0.0; // No blockers, fully lit

    // Estimate penumbra size
    float penumbraSize = (currentDepth - blockerDepth) * lightSize / blockerDepth;
    penumbraSize *= shadowRenderDistance;

    // Calculate shadow using PCSS
    float shadow = pcss(projCoords, currentDepth, penumbraSize);

    return (shadow) * (1.0 - fadeOut);
}

float sampleShadowMap(vec3 fragPos, int waterTypeIndex, vec2 distortion, float lightDotNormals, vec3 normals) {
    vec4 projCoords = lightProjectionMatrix * vec4(fragPos, 1);
    projCoords = projCoords / projCoords.w;
    projCoords = projCoords * 0.5 + 0.5;
    projCoords.xy += distortion;

    // Fade out shadows near shadow texture edges
    vec2 uv = projCoords.xy * 2.0 - 1.0;
    float fadeOut = smoothstep(0.75, 1.0, dot(uv, uv));

    if (fadeOut >= 1.0)
        return 0.0;

    float pcssShadowSample = renderPCSSShadows(projCoords, fadeOut);
    return pcssShadowSample;
}
#else
#define sampleShadowMap(fragPos, waterTypeIndex, distortion, lightDotNormals) 0
#endif
