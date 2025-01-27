/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"

using namespace Falcor;

class HelloDXR : public IRenderer
{
public:
    void onLoad(RenderContext* pRenderContext) override;
    void onFrameRender(RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo) override;
    void onResizeSwapChain(uint32_t width, uint32_t height) override;
    bool onKeyEvent(const KeyboardEvent& keyEvent) override;
    bool onMouseEvent(const MouseEvent& mouseEvent) override;
    void onGuiRender(Gui* pGui) override;

private:
    static constexpr int kMaxGroupKind = 4;
    enum class ExamplePixelShaders
    {
        ConstColor = 0,
        ColorInterp = 1,
        Textured = 2,
        Count
    };

    struct GuiData
    {
        int32_t mSystemIndex = -1;
        uint32_t mPixelShaderIndex = 0;
        bool mSortSystem = false;
        int32_t mMaxParticles = 50;
        int32_t mMaxEmitPerFrame = 50;
        Gui::DropdownList mTexDropdown;
    } mGuiData;

    struct PixelShaderData
    {
        PixelShaderData(glm::vec4 color) { type = ExamplePixelShaders::ConstColor; colorData.color1 = color; }
        PixelShaderData(ColorInterpPsPerFrame data) { type = ExamplePixelShaders::ColorInterp; colorData = data; }
        PixelShaderData(uint32_t newTexIndex, ColorInterpPsPerFrame data)
        {
            type = ExamplePixelShaders::Textured;
            texIndex = newTexIndex;
            colorData = data;
        }

        ExamplePixelShaders type;
        ColorInterpPsPerFrame colorData;
        uint32_t texIndex;
    };

    RasterScenePass::SharedPtr mpRasterPass;
    Scene::SharedPtr mpScene;
    SceneBuilder::SharedPtr mpSceneBuilder;

    RtProgram::SharedPtr mpRaytraceProgram = nullptr;
    Camera::SharedPtr mpCamera;
    ParticleSystem::SharedPtr mpPSys;

    bool mRayTrace = true;
    bool mUseDOF = false;
    bool mbPS = true;
    bool mbUpdateAABB = false;
    bool mbShowRasterPS = false;
    bool mbCreatePS = false;
    bool mbSpriteRT = true;
    bool mbMeasureDiff = false;
    uint32_t mWidth = 1;
    uint32_t mHeight = 1;
    RtProgramVars::SharedPtr mpRtVars;
    //RtSceneRenderer::SharedPtr mpRtRenderer;
    Texture::SharedPtr mpRtOut;
    std::vector<ParticleSystem::SharedPtr> mpParticleSystems;
    std::vector<PixelShaderData> mPsData;
    std::vector<Texture::SharedPtr> mpTextures;
    Buffer::SharedPtr mpRotateBuffer;
    Buffer::SharedPtr mpRangeBuffer;

    GpuTimer::SharedPtr mpTimer;
    Texture::SharedPtr mpTexture;
    Sampler::SharedPtr mpLinearSampler;

    double mRTTime = 0.0;
    size_t mSpriteCount = 0;
#ifdef _DEBUG
    double mMaxPrecent = 0.0;
    uint32_t mMaxTotTime = 0;
    uint32_t mMaxPartTime = 0;
#endif

#ifdef _DEBUG
    Buffer::SharedPtr mpDBuffer;
    Buffer::SharedPtr mpbaDBuffer;
#endif
    uint32_t mSampleIndex = 0xdeadbeef;

    void setPerFrameVars(const Fbo* pTargetFbo);
    void renderRT(RenderContext* pContext, const Fbo::SharedPtr& pTargetFbo);
    void loadScene(const std::string& filename, const Fbo* pTargetFbo);
    void renderParticleSystem(RenderContext* pContext, const Fbo::SharedPtr& pTargetFbo);
    void createParticleSystem(::HelloDXR::ExamplePixelShaders shadertype);
};
