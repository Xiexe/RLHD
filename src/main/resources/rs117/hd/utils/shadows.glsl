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

#if SHADOW_MODE != SHADOW_MODE_OFF
float sampleShadowMap(vec3 fragPos, int waterTypeIndex, vec2 distortion, float lightDotNormals) {
    vec4 shadowPos = lightProjectionMatrix * vec4(fragPos, 1);
    shadowPos = (shadowPos / shadowPos.w) * .5 + .5;
    shadowPos.xy += distortion;

    // Fade out shadows near shadow texture edges
    vec2 uv = shadowPos.xy * 2.0 - 1.0;
    float fadeOut = smoothstep(0.5, 1.0, dot(uv, uv));

    if (fadeOut >= 1.0)
        return 0.0;

    vec2 shadowRes = textureSize(shadowMap, 0);
    vec2 texelSize = 1.0 / shadowRes;

    float shadowBias = 0.0009;
    float fragDepth = shadowPos.z;

    // Approximate blocker depth / we could use a mipmap here for better accuracy / performance maybe. It seems like a common approach.
    float shadowTexel = textureLod(shadowMap, shadowPos.xy, 1).r;
    float blockerDepth = shadowTexel * SHADOW_COMBINED_MAX;

    // Estimate penumbra size based on blocker depth
    float penumbraSize = (fragDepth - blockerDepth) / blockerDepth;

    // Filter the shadow using a variable kernel size based on penumbra size
    float filterRadiusDivsor = 2048;
    float filterRadius = penumbraSize / filterRadiusDivsor;
    float shadow = 0.0;

    int filterSamples = 8;
    for (int x = 0; x < filterSamples; x++) {
        for (int y = 0; y < filterSamples; y++) {
            vec2 offset = (vec2(x, y) / float(filterSamples) - 0.5) * filterRadius;

            float shadowTexel = texture(shadowMap, shadowPos.xy + offset).r;
            #if SHADOW_TRANSPARENCY
                int alphaDepth = int(shadowTexel * SHADOW_COMBINED_MAX);
                float depth = float(alphaDepth & SHADOW_DEPTH_MAX) / SHADOW_DEPTH_MAX;
                float alpha = 1.0 - float(alphaDepth >> SHADOW_DEPTH_BITS) / SHADOW_ALPHA_MAX;
            #else
                float depth = shadowTexel;
                float alpha = 1.0;
            #endif

            if ((fragDepth - shadowBias) > depth)
                shadow += alpha;
        }
    }
    shadow /= float(filterSamples * filterSamples);

    return shadow * (1.0 - fadeOut);
}
#else
#define sampleShadowMap(fragPos, waterTypeIndex, distortion, lightDotNormals) 0
#endif
