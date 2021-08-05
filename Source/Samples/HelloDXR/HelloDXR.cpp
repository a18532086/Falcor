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
S # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.d
 **************************************************************************/
#include "HelloDXR.h"
#define USE_EMITTER_AABB

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
    eTypeSprite = 0,
};

namespace
{
    struct PS_Data
    {
        float3 center;
        float scale;
        float rotate;
    };
    struct Range
    {
        uint32_t offset;
        uint32_t size;
    };
};

void HelloDXR::createParticleSystem(ExamplePixelShaders shadertype)
{
    uint32_t particleMaxSize = mGuiData.mMaxParticles;
    for (auto pPS : mpParticleSystems)
    {

        particleMaxSize += pPS->getMaxParticles();
    }
#if defined(USE_EMITTER_AABB)
    mpRotateBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "particlePool", particleMaxSize);
#else
    mpRotateBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "rotatePool", particleMaxSize);
#endif

#if defined(USE_EMITTER_AABB)
    mpRangeBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "particleSystemRanges", static_cast<uint32_t>(mpParticleSystems.size()) + 1);
#endif

    auto pContext = gpDevice->getRenderContext();
    switch (shadertype)
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
    w.text("Ray Tracing Time: " + std::to_string(mRTTime) + "ms");

    if (w.button("Create"))
    {
        createParticleSystem((ExamplePixelShaders)mGuiData.mPixelShaderIndex);
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
    rtProgDesc.addShaderLibrary("Samples/HelloDXR/HelloDXR.rt.slang").setRayGen("rayGenIR");
    rtProgDesc.addHitGroup(0, "primaryClosestHitIR", "primaryAnyHitIR").addMiss(0, "primaryMissIR");
    rtProgDesc.addHitGroup(1, "", "shadowAnyHit").addMiss(1, "shadowMiss");
    rtProgDesc.addAABBHitGroup(0, "primaryClosestHitIR", "primaryAnyHitIR");
    rtProgDesc.addAABBHitGroup(1, "", "shadowAnyHit");
#if defined(USE_EMITTER_AABB)
    rtProgDesc.addIntersection(eTypeSprite, "spriteEmitterAABBIntersectionSimple");
#else
    rtProgDesc.addIntersection(eTypeSprite, "spriteIntersection");
#endif
    rtProgDesc.addDefines(mpScene->getSceneDefines());
#ifdef _DEBUG
    rtProgDesc.addDefine("_DEBUG", "1");
#endif
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
    if (mpTimer == nullptr)
    {
        mpTimer = GpuTimer::create();
    }
    mpTimer->begin();
    mpScene->raytrace(pContext, mpRaytraceProgram.get(), mpRtVars, uint3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1));
    mpTimer->end();
    mRTTime = mpTimer->getElapsedTime();

    pContext->blit(mpRtOut->getSRV(), pTargetFbo->getRenderTargetView(0));
}

void HelloDXR::renderParticleSystem(RenderContext* pContext, const Fbo::SharedPtr& pTargetFbo)
{
    PROFILE("renderParticleSystem");
    static double fC = 0.0;
    static constexpr double kRefreshRate = 0.5;
    SceneBuilder::SharedPtr pSB;
    std::vector<float> rots;
    if (mbPS)
    {
        fC += gpFramework->getGlobalClock().getDelta();
    }
    const bool bshouldUpdateAABB = fC >= kRefreshRate && !(fC = 0.0) || mbUpdateAABB;
    if (bshouldUpdateAABB)
    {
        pSB = mpSceneBuilder->copy();
        mbUpdateAABB = false;
    }
    size_t bufOffset = 0;
    uint32_t rangeOffset = 0;
    std::vector<Range> ranges;
    for (auto it = mpParticleSystems.begin(); it != mpParticleSystems.end(); ++it)
    {
        if (mbPS)
        {
            (*it)->update(pContext, static_cast<float>(gpFramework->getGlobalClock().getDelta()), mpCamera->getViewMatrix());
        }
        if (mbCreatePS)
        {
            for (int i = 0; i < 7;++i)
            {
                (*it)->update(pContext, 0.1f, mpCamera->getViewMatrix());
            }
        }
        if (mbShowRasterPS)
        {
            (*it)->render(pContext, pTargetFbo,mpCamera->getViewMatrix(), mpCamera->getProjMatrix());
        }

        if (bshouldUpdateAABB)
        {
            std::vector<Particle> particleInfos;
            (*it)->getParticlesVertexInfo(particleInfos);
#if defined(USE_EMITTER_AABB)
            std::vector<PS_Data> pds;
            AABB maxAABB(float3(FLT_MAX), float3(FLT_MIN));
            for (const auto& particle_info : particleInfos)
            {
                float3 offset = float3(particle_info.scale * sqrtf(2.0f));
                AABB aabb(particle_info.pos - offset, particle_info.pos + offset);
                maxAABB.minPoint = min(aabb.minPoint, maxAABB.minPoint);
                maxAABB.maxPoint = max(aabb.maxPoint, maxAABB.maxPoint);
                pds.push_back({ particle_info.pos,particle_info.scale,particle_info.rot });
            }
            pSB->addCustomPrimitive(eTypeSprite, maxAABB);
            mpRotateBuffer->setBlob(pds.data(), bufOffset, pds.size() * sizeof(PS_Data));
            bufOffset += pds.size() * sizeof(PS_Data);
            ranges.push_back({ rangeOffset,static_cast<uint32_t>(particleInfos.size()) });
            rangeOffset += static_cast<uint32_t>(particleInfos.size());
#else
            for (const auto& particle_info : particleInfos)
            {
                float3 offset = float3(particle_info.scale * sqrtf(2.0f));
                AABB aabb(particle_info.pos - offset, particle_info.pos + offset);
                pSB->addCustomPrimitive(eTypeSprite, aabb);
            }
            rots.resize(particleInfos.size());
            std::transform(particleInfos.cbegin(), particleInfos.cend(), rots.begin(), [](const Particle& p) {return p.rot;});
            mpRotateBuffer->setBlob(rots.data(), bufOffset, rots.size() * sizeof(float));
            bufOffset += rots.size() * sizeof(float);
#endif
        }
        
    }

    if (mbCreatePS)
    {
        mbCreatePS = false;
    }

    if (bshouldUpdateAABB)
    {
        mpScene = pSB->getScene();
        mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
#ifdef _DEBUG
        std::vector<uint8_t> vc(mpDBuffer->getSize(), 0);
        mpDBuffer->setBlob(vc.data(), 0, vc.size());
#endif
#if defined(USE_EMITTER_AABB)
        if (!ranges.empty())
        {
            mpRangeBuffer->setBlob(ranges.data(), 0, sizeof(Range) * ranges.size());
        }
        mpRtVars->setBuffer("particleSystemRanges", mpRangeBuffer);
        mpRtVars->setBuffer("particlePool", mpRotateBuffer);
#else
        mpRtVars->setBuffer("rotatePool", mpRotateBuffer);
#endif
        mpRtVars->setBuffer("directViewDirBuffer", mpDirViewBuffer);
#ifdef _DEBUG
        mpRtVars->setBuffer("dBuffer", mpDBuffer);
#endif
        mpRaytraceProgram->setScene(mpScene);
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
    if (keyEvent.key == KeyboardEvent::Key::P && keyEvent.type == KeyboardEvent::Type::KeyPressed)
    {
        mbPS = !mbPS;
        return true;
    }
    if (keyEvent.key == KeyboardEvent::Key::O && keyEvent.type == KeyboardEvent::Type::KeyPressed)
    {
        mbUpdateAABB = true;
        return true;
    }
    if (keyEvent.key == KeyboardEvent::Key::I && keyEvent.type == KeyboardEvent::Type::KeyPressed)
    {
        mbShowRasterPS = !mbShowRasterPS;
        return true;
    }
    if (keyEvent.key == KeyboardEvent::Key::U && keyEvent.type == KeyboardEvent::Type::KeyPressed)
    {
        createParticleSystem(ExamplePixelShaders::ConstColor);
        createParticleSystem(ExamplePixelShaders::ConstColor);
        createParticleSystem(ExamplePixelShaders::ConstColor);
        mpCamera->setPosition(float3(-1.646739, 2.510957, 2.434580));
        mpCamera->setTarget(float3(-1.033757, 2.431306, 1.648509));
        mpCamera->setUpVector(float3(0,1,0));
        mbCreatePS = true;
        mbPS = false;
        mbUpdateAABB = true;
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
    if (mpRaytraceProgram)
    {
        mpDirViewBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "directViewDirBuffer", height * width, ResourceBindFlags::UnorderedAccess);
#ifdef _DEBUG
        mpDBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "dBuffer", height * width, ResourceBindFlags::UnorderedAccess);
#endif
    }

}

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
    HelloDXR::UniquePtr pRenderer = std::make_unique<HelloDXR>();
    SampleConfig config;
    config.windowDesc.title = "HelloDXR";
    config.windowDesc.resizableWindow = true;

    Sample::run(config, pRenderer);
}
