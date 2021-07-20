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
#include "HelloDXR.h"

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);
static const std::string kDefaultScene = "Arcade/Arcade_Wall.pyscene";
static const Gui::DropdownList kPixelShaders
{
    {0, "ConstColor"},
    {1, "ColorInterp"},
    {2, "Textured"}
};

static const char* kConstColorPs = "Samples/HelloDXR/ParticleConstColor.ps.slang";
static const char* kColorInterpPs = "Samples/HelloDXR/ParticleInterpColor.ps.slang";
static const char* kTexturedPs = "Samples/HelloDXR/ParticleTexture.ps.slang";

enum CustomTypeID
{
    eTypeSprite = 1,
};

void HelloDXR::onGuiRender(Gui* pGui)
{
    Gui::Window w(pGui, "Hello DXR Settings", { 300, 400 }, { 10, 80 });

    w.checkbox("Ray Trace", mRayTrace);
    w.checkbox("Use Depth of Field", mUseDOF);
    if (w.button("Load Scene"))
    {
        std::string filename;
        if (openFileDialog(Scene::getFileExtensionFilters(), filename))
        {
            loadScene(filename, gpFramework->getTargetFbo().get());
        }
    }
    w.var("Max Particles", mGuiData.mMaxParticles, 0);
    w.var("Max Emit Per Frame", mGuiData.mMaxEmitPerFrame, 0);
    w.checkbox("Sorted", mGuiData.mSortSystem);
    w.dropdown("PixelShader", kPixelShaders, mGuiData.mPixelShaderIndex);

    if (w.button("Create"))
    {
        auto pContext = gpDevice->getRenderContext();
        switch ((ExamplePixelShaders)mGuiData.mPixelShaderIndex)
        {
        case ExamplePixelShaders::ConstColor:
        {
            ParticleSystem::SharedPtr pSys = ParticleSystem::create(pContext, mGuiData.mMaxParticles,
                mGuiData.mMaxEmitPerFrame, kConstColorPs, ParticleSystem::kDefaultSimulateShader, mGuiData.mSortSystem);
            mpParticleSystems.push_back(pSys);
            mPsData.push_back(glm::vec4(1.f, 0.f, 0.f, 1.f));
            mpParticleSystems[mpParticleSystems.size() - 1]->getDrawVars()["PsPerFrame"].setBlob(&mPsData[mPsData.size() - 1].colorData.color1, sizeof(glm::vec4));
            break;
        }
        case ExamplePixelShaders::ColorInterp:
        {
            ParticleSystem::SharedPtr pSys = ParticleSystem::create(pContext, mGuiData.mMaxParticles,
                mGuiData.mMaxEmitPerFrame, kColorInterpPs, ParticleSystem::kDefaultSimulateShader, mGuiData.mSortSystem);
            mpParticleSystems.push_back(pSys);
            ColorInterpPsPerFrame perFrame;
            perFrame.color1 = glm::vec4(1.f, 0.f, 0.f, 1.f);
            perFrame.colorT1 = pSys->getParticleDuration();
            perFrame.color2 = glm::vec4(0.f, 0.f, 1.f, 1.f);
            perFrame.colorT2 = 0.f;
            mPsData.push_back(perFrame);
            mpParticleSystems[mpParticleSystems.size() - 1]->getDrawVars()["PsPerFrame"].setBlob(&mPsData[mPsData.size() - 1].colorData, sizeof(ColorInterpPsPerFrame));
            break;
        }
        case ExamplePixelShaders::Textured:
        {
            ParticleSystem::SharedPtr pSys = ParticleSystem::create(pContext, mGuiData.mMaxParticles,
                mGuiData.mMaxEmitPerFrame, kTexturedPs, ParticleSystem::kDefaultSimulateShader, mGuiData.mSortSystem);
            mpParticleSystems.push_back(pSys);
            ColorInterpPsPerFrame perFrame;
            perFrame.color1 = glm::vec4(1.f, 1.f, 1.f, 1.f);
            perFrame.colorT1 = pSys->getParticleDuration();
            perFrame.color2 = glm::vec4(1.f, 1.f, 1.f, 0.1f);
            perFrame.colorT2 = 0.f;
            mPsData.push_back(PixelShaderData(0, perFrame));
            pSys->getDrawVars()->setTexture("gTex", mpTextures[0]);
            mpParticleSystems[mpParticleSystems.size() - 1]->getDrawVars()["PsPerFrame"].setBlob(&mPsData[mPsData.size() - 1].colorData, sizeof(ColorInterpPsPerFrame));
            break;
        }
        default:
        {
            should_not_get_here();
        }
        }
    }

    mpScene->renderUI(w);
}

void HelloDXR::loadScene(const std::string& filename, const Fbo* pTargetFbo)
{
    auto pNewSB = SceneBuilder::create(filename);
    mpSceneBuilder = pNewSB->copy();
    mpScene = pNewSB->getScene();
    if (!mpScene) return;

    mpCamera = mpScene->getCamera();

    // Update the controllers
    float radius = mpScene->getSceneBounds().radius();
    mpScene->setCameraSpeed(radius * 0.25f);
    float nearZ = std::max(0.1f, radius / 750.0f);
    float farZ = radius * 10;
    mpCamera->setDepthRange(nearZ, farZ);
    mpCamera->setAspectRatio((float)pTargetFbo->getWidth() / (float)pTargetFbo->getHeight());

    mpRasterPass = RasterScenePass::create(mpScene, "Samples/HelloDXR/HelloDXR.ps.slang", "", "main");

    RtProgram::Desc rtProgDesc;
    rtProgDesc.addShaderLibrary("Samples/HelloDXR/HelloDXR.rt.slang").setRayGen("rayGen");
    rtProgDesc.addHitGroup(0, "primaryClosestHit", "primaryAnyHit").addMiss(0, "primaryMiss");
    rtProgDesc.addHitGroup(1, "", "shadowAnyHit").addMiss(1, "shadowMiss");
    rtProgDesc.addIntersection(eTypeSprite, "spriteIntersection");
    rtProgDesc.addDefines(mpScene->getSceneDefines());
    rtProgDesc.setMaxTraceRecursionDepth(3); // 1 for calling TraceRay from RayGen, 1 for calling it from the primary-ray ClosestHitShader for reflections, 1 for reflection ray tracing a shadow ray

    mpRaytraceProgram = RtProgram::create(rtProgDesc);
    mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
    mpRaytraceProgram->setScene(mpScene);
}

void HelloDXR::onLoad(RenderContext* pRenderContext)
{
    if (gpDevice->isFeatureSupported(Device::SupportedFeatures::Raytracing) == false)
    {
        logFatal("Device does not support raytracing!");
    }

    loadScene(kDefaultScene, gpFramework->getTargetFbo().get());
}

void HelloDXR::setPerFrameVars(const Fbo* pTargetFbo)
{
    PROFILE("setPerFrameVars");
    auto cb = mpRtVars["PerFrameCB"];
    cb["invView"] = glm::inverse(mpCamera->getViewMatrix());
    cb["viewportDims"] = float2(pTargetFbo->getWidth(), pTargetFbo->getHeight());
    float fovY = focalLengthToFovY(mpCamera->getFocalLength(), Camera::kDefaultFrameHeight);
    cb["tanHalfFovY"] = std::tan(fovY * 0.5f);
    cb["sampleIndex"] = mSampleIndex++;
    cb["useDOF"] = mUseDOF;
    mpRtVars->getRayGenVars()["gOutput"] = mpRtOut;
}

void HelloDXR::renderRT(RenderContext* pContext, const Fbo* pTargetFbo)
{
    PROFILE("renderRT");
    setPerFrameVars(pTargetFbo);

    pContext->clearUAV(mpRtOut->getUAV().get(), kClearColor);
    mpScene->raytrace(pContext, mpRaytraceProgram.get(), mpRtVars, uint3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1));
    pContext->blit(mpRtOut->getSRV(), pTargetFbo->getRenderTargetView(0));
}

void HelloDXR::renderParticleSystem(RenderContext* pContext, const Fbo::SharedPtr& pTargetFbo)
{
    PROFILE("renderParticleSystem");
    auto pSB = mpSceneBuilder->copy();
    for (auto it = mpParticleSystems.begin(); it != mpParticleSystems.end(); ++it)
    {
        (*it)->update(pContext, static_cast<float>(gpFramework->getGlobalClock().getDelta()), mpCamera->getViewMatrix());
        (*it)->render(pContext, pTargetFbo,mpCamera->getViewMatrix(), mpCamera->getProjMatrix());
        std::vector<AABB> aabbs;
        (*it)->getParticlesVertexInfo(aabbs);
        for (const auto& aabb:aabbs)
        {
            pSB->addCustomPrimitive(eTypeSprite, aabb);
        }
    }
    static int fC = 0;
    if (++fC % 60 == 0)
    {
        mpScene = pSB->getScene();
    }

}

void HelloDXR::onFrameRender(RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo)
{
    pRenderContext->clearFbo(pTargetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);

    if (mpScene)
    {
        mpScene->update(pRenderContext, gpFramework->getGlobalClock().getTime());
        if (mRayTrace) renderRT(pRenderContext, pTargetFbo.get());
        else mpRasterPass->renderScene(pRenderContext, pTargetFbo);
        renderParticleSystem(pRenderContext, pTargetFbo);
    }

    TextRenderer::render(pRenderContext, gpFramework->getFrameRate().getMsg(), pTargetFbo, { 20, 20 });
}

bool HelloDXR::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.key == KeyboardEvent::Key::Space && keyEvent.type == KeyboardEvent::Type::KeyPressed)
    {
        mRayTrace = !mRayTrace;
        return true;
    }
    if (mpScene && mpScene->onKeyEvent(keyEvent)) return true;
    return false;
}

bool HelloDXR::onMouseEvent(const MouseEvent& mouseEvent)
{
    return mpScene && mpScene->onMouseEvent(mouseEvent);
}

void HelloDXR::onResizeSwapChain(uint32_t width, uint32_t height)
{
    float h = (float)height;
    float w = (float)width;

    if (mpCamera)
    {
        mpCamera->setFocalLength(18);
        float aspectRatio = (w / h);
        mpCamera->setAspectRatio(aspectRatio);
    }

    mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA16Float, 1, 1, nullptr, Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource);
}

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
    HelloDXR::UniquePtr pRenderer = std::make_unique<HelloDXR>();
    SampleConfig config;
    config.windowDesc.title = "HelloDXR";
    config.windowDesc.resizableWindow = true;

    Sample::run(config, pRenderer);
}
