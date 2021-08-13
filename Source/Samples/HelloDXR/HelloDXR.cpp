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

static const float4 kClearColor(0.f, 0.f, 0.f, 1.f);
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
static const std::string kDefaultTexture = "smoke-puff.png";

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
        float cosRot;
        float sinRot;
    };
    struct Range
    {
        uint32_t offset;
        uint32_t size;
    };
#ifdef _DEBUG
    struct DebugInfo {
        uint totaltime;
        uint partTime;
        float partPrecent;
        uint aabbIdxCnt;
    };
#endif
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
    mpRangeBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "particleSystemRanges", static_cast<uint32_t>(mpParticleSystems.size()) + 1);
    mpCSVars->setBuffer("particlePool", mpRotateBuffer);
    mpCSVars->setBuffer("particleSystemRanges", mpRangeBuffer);
#else
    mpRotateBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "rotatePool", particleMaxSize);
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
#ifdef _DEBUG
    w.text("Blend Total Time: " + std::to_string(mMaxTotTime) );
    w.text("Blend Sort Time: " + std::to_string(mMaxPartTime) );
    w.text("Blend Sort Percent: " + std::to_string(mMaxPrecent) + " %");
#endif

    if (w.button("Create"))
    {
        createParticleSystem((ExamplePixelShaders)mGuiData.mPixelShaderIndex);
    }

    mpScene->renderUI(w);
}

void HelloDXR::loadScene(const std::string& filename, const Fbo* pTargetFbo)
{
    mpTexture = Texture::createFromFile(kDefaultTexture, true, false);
    Sampler::Desc samplerDesc;
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    mpLinearSampler = Sampler::create(samplerDesc);

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
    rtProgDesc.addShaderLibrary("Samples/HelloDXR/HelloDXR.rt.slang").setRayGen("rayGenIR2CS");
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

    rtProgDesc.setCompilerFlags(Shader::CompilerFlags::GenerateDebugInfo);
    //rtProgDesc.setCompilerFlags(Shader::CompilerFlags::DumpIntermediates);

    ComputeProgram::Desc csProgDesc;
    csProgDesc.addShaderLibrary("Samples/HelloDXR/Blend.cs.slang");
    csProgDesc.csEntry("doBlendCS");
    csProgDesc.setCompilerFlags(Shader::CompilerFlags::GenerateDebugInfo);

    mpRaytraceProgram = RtProgram::create(rtProgDesc);
    mpCSProgram = ComputeProgram::create(csProgDesc);
#ifdef _DEBUG
    mpCSProgram->addDefine("_DEBUG", "1");
#endif
    ComputeProgram::Desc blitProgDesc;
    blitProgDesc.addShaderLibrary("Samples/HelloDXR/BlitUnormToRT.cs.slang");
    blitProgDesc.csEntry("main");
    blitProgDesc.setCompilerFlags(Shader::CompilerFlags::GenerateDebugInfo);
    mpBlitProgram = ComputeProgram::create(blitProgDesc);
    mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
    mpCSVars = ComputeVars::create(mpCSProgram.get());
    mpBlitVars = ComputeVars::create(mpBlitProgram.get());
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
    mpRtVars["gTex"] = mpTexture;
    mpRtVars["gSampler"] = mpLinearSampler;
    mpRtVars->getRayGenVars()["gOutput"] = mpRtOut;

    mpCSVars["gTex"] = mpTexture;
    mpCSVars["gSampler"] = mpLinearSampler;


}

void HelloDXR::renderRT(RenderContext* pContext, const Fbo* pTargetFbo)
{
    PROFILE("renderRT");
    setPerFrameVars(pTargetFbo);

    pContext->clearUAV(mpRtOut->getUAV().get(), kClearColor);
    //std::vector<byte> vc(mpUonrmRTBuffer->getSize(),0);
    //mpUonrmRTBuffer->setBlob(vc.data(), 0, vc.size());
    pContext->copyBufferRegion(mpUonrmRTBuffer.get(), 0, mpUonrmRTClearBuffer.get(), 0, mpUonrmRTBuffer->getSize());
    if (mpTimer == nullptr)
    {
        mpTimer = GpuTimer::create();
    }
    mpTimer->begin();
    mpScene->raytrace(pContext, mpRaytraceProgram.get(), mpRtVars, uint3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1));
    mpTimer->end();
    mRTTime = mpTimer->getElapsedTime();
    mpCSProgram->dispatchCompute(pContext, mpCSVars.get(), uint3(mWidth / 16 , mHeight * 2 /16, 1));
    mpBlitProgram->dispatchCompute(pContext, mpBlitVars.get(), uint3(mWidth / 32 , mHeight / 8, 1));
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
                pds.push_back({ particle_info.pos,particle_info.scale, cos(particle_info.rot) , sin(particle_info.rot) });
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
        mpRtVars->setBuffer("RGS2CSBuffer", mpRGS2CSBuffer);
#else
        mpRtVars->setBuffer("rotatePool", mpRotateBuffer);
#endif
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
        //mpCamera->setPosition(float3(-0.584588, 2.589808, 1.666172));
        //mpCamera->setTarget(float3(-0.036677, 2.358391, 0.862282));
        //mpCamera->setUpVector(float3(0,1,0));
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

    mWidth = width;
    mHeight = height;

    if (mpCamera)
    {
        mpCamera->setFocalLength(18);
        float aspectRatio = (w / h);
        mpCamera->setAspectRatio(aspectRatio);
    }

    mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA16Float, 1, 1, nullptr, Resource::BindFlags::UnorderedAccess | Resource::BindFlags::ShaderResource);
    if (mpRaytraceProgram)
    {
#ifdef _DEBUG
        mpDBuffer = Buffer::createStructured(mpRaytraceProgram.get(), "dBuffer", height * width, ResourceBindFlags::UnorderedAccess);
#endif
    }
    if (mpCSProgram)
    {
        mpLockBuffer = Buffer::createStructured(mpCSProgram.get(), "lockBuffer", height * width, ResourceBindFlags::UnorderedAccess);
        mpRGS2CSBuffer = Buffer::createStructured(mpCSProgram.get(), "RGS2CSBuffer", height * width * 2, ResourceBindFlags::UnorderedAccess);
        mpUonrmRTBuffer = Buffer::createStructured(mpCSProgram.get(), "unormFBO", height * width * 2 * 3, ResourceBindFlags::UnorderedAccess);
        std::vector<char> vClear(mpUonrmRTBuffer->getSize(), 0);
        mpUonrmRTClearBuffer = Buffer::create(mpUonrmRTBuffer->getSize(), ResourceBindFlags::None, Buffer::CpuAccess::None,vClear.data());
        mpCSVars->setBuffer("lockBuffer", mpLockBuffer);
        mpCSVars->setBuffer("RGS2CSBuffer", mpRGS2CSBuffer);
        mpCSVars->setBuffer("unormFBO", mpUonrmRTBuffer);
        mpCSVars["CSCB"]["gDispatchX"] = mWidth;
#ifdef _DEBUG
        mpCSVars->setBuffer("dBuffer", mpDBuffer);
#endif
    }

    if (mpBlitProgram)
    {
        mpBlitVars->setBuffer("unormFBO", mpUonrmRTBuffer);
        mpBlitVars->setTexture("gOutput", mpRtOut);

        mpBlitVars["CSCB"]["gDispatchX"] = mWidth;
    }

}


#ifndef MAX_AABB_CNT_PERRAY
#define MAX_AABB_CNT_PERRAY 16
#endif

#ifndef AABB_CNT_MSB
#define AABB_CNT_MSB 4
#endif

#ifndef MAX_AABB_IDX_CNT
#define MAX_AABB_IDX_CNT 256
#endif

#ifndef AABB_IDX_MSB
#define AABB_IDX_MSB 8
#endif

#ifndef MAX_OIT_CNT_PERRAY
#define MAX_OIT_CNT_PERRAY 4
#endif

#ifndef SPRITE_HITINFO_BIT_SIZE
#define SPRITE_HITINFO_BIT_SIZE (32 + AABB_CNT_MSB + AABB_IDX_MSB * MAX_AABB_CNT_PERRAY)
/* float + aabbHitCnt + aabbIdx[MAX_AABB_CNT_PERRAY] */
#endif

#ifndef SPRITE_PACKED_HITINFO_SIZE
#define SPRITE_PACKED_HITINFO_SIZE ((SPRITE_HITINFO_BIT_SIZE-1)/128+1)
#endif

typedef std::array<uint4, SPRITE_PACKED_HITINFO_SIZE> PackedSpriteHitInfo;

uint getOrVal(uint val, uint bitStride, uint offset) {
    uint mask = bitStride == 32 ? 0xffffffff : ((1 << bitStride) - 1);
    return (val & mask) << offset;
}


struct SpriteRayHitInfo {
    float hitT;
    uint aabbHitCnt;
    uint aabbIdx[MAX_AABB_CNT_PERRAY];
    static void setPackedSpriteHitInfo(PackedSpriteHitInfo& packInfo, uint& offset, uint stride, uint val) {
        static const uint kElementSize = 128;
        static const uint kElementSizeMask = 127;
        static const uint kElementSizeBit = 7;
        static const uint kComponent = 32;
        static const uint kComponentMask = 31;
        static const uint kComponentBit = 5;
        static const uint kFullBit = ~0;
        uint mask = getOrVal(kFullBit, stride, offset & kComponentMask);
        uint orVal = getOrVal(val, stride, offset & kComponentMask);
        uint fIdx = offset >> kElementSizeBit;
        uint sIdx = offset >> kComponentBit & 0x03;
        int elementRedundancyBit = ((offset & kElementSizeMask) + stride) - kElementSize;
        int componentRedundancyBit = ((offset & kComponentMask) + stride) - kComponent;
        packInfo[fIdx][sIdx] &= ~mask;
        packInfo[fIdx][sIdx] |= orVal;
        if (elementRedundancyBit > 0) {
            mask = getOrVal(kFullBit, elementRedundancyBit, 0);
            orVal = getOrVal(val, elementRedundancyBit, 0);
            packInfo[fIdx + 1][0] &= ~mask;
            packInfo[fIdx + 1][0] |= orVal;
        }
        else if (componentRedundancyBit > 0) {
            mask = getOrVal(kFullBit, componentRedundancyBit, 0);
            orVal = getOrVal(val, componentRedundancyBit, 0);
            packInfo[fIdx][sIdx + 1] &= ~mask;
            packInfo[fIdx][sIdx + 1] |= orVal;
        }
        offset += stride;
    }
    uint getElementFromPackedSpriteHitInfo(const PackedSpriteHitInfo& packInfo, uint& offset, uint stride)
    {
        static const uint kElementSize = 128;
        static const uint kElementSizeMask = 127;
        static const uint kElementSizeBit = 7;
        static const uint kComponent = 32;
        static const uint kComponentMask = 31;
        static const uint kComponentBit = 5;
        static const uint kFullBit = ~0;
        uint retVal = 0;
        uint mask = getOrVal(kFullBit, stride, offset & kComponentMask);
        uint fIdx = offset >> kElementSizeBit;
        uint sIdx = offset >> kComponentBit & 0x03;
        int elementRedundancyBit = ((offset & kElementSizeMask) + stride) - kElementSize;
        int componentRedundancyBit = ((offset & kComponentMask) + stride) - kComponent;
        retVal |= (packInfo[fIdx][sIdx] & mask) >> (offset & kComponentMask);
        if (elementRedundancyBit > 0) {
            mask = getOrVal(kFullBit, elementRedundancyBit, 0);
            retVal |= (packInfo[fIdx + 1][0] & mask) << (stride - elementRedundancyBit);
        }
        else if (componentRedundancyBit > 0) {
            mask = getOrVal(kFullBit, componentRedundancyBit, 0);
            retVal |= (packInfo[fIdx][sIdx + 1] & mask) << (stride - componentRedundancyBit);
        }
        offset += stride;
        return retVal;
    }
    void pack(PackedSpriteHitInfo& packInfo) {
        // PackedSpriteHitInfo packInfo;
        uint offset = 0;
        setPackedSpriteHitInfo(packInfo, offset, 32, asuint(hitT));
        setPackedSpriteHitInfo(packInfo, offset, AABB_CNT_MSB, aabbHitCnt);
        for (uint i = 0; i < MAX_AABB_CNT_PERRAY; ++i) {
            setPackedSpriteHitInfo(packInfo, offset, AABB_IDX_MSB, aabbIdx[i]);
        }
    }
    void encode(const PackedSpriteHitInfo& packInfo)
    {
        uint offset = 0;
        hitT = asfloat(getElementFromPackedSpriteHitInfo(packInfo, offset, 32));
        aabbHitCnt = getElementFromPackedSpriteHitInfo(packInfo, offset, AABB_CNT_MSB);
        for (uint i = 0; i < MAX_AABB_CNT_PERRAY; ++i) {
            aabbIdx[i] = getElementFromPackedSpriteHitInfo(packInfo, offset, AABB_IDX_MSB);
        }
    }
};

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
    //SpriteRayHitInfo sInfo = {
    //    3.72009f,
    //    3,
    //};
    //std::fill(std::begin(sInfo.aabbIdx), std::end(sInfo.aabbIdx), 0xff);
    //PackedSpriteHitInfo packInfo;
    //sInfo.pack(packInfo);
    //SpriteRayHitInfo tmp;
    //tmp.encode(packInfo);
    HelloDXR::UniquePtr pRenderer = std::make_unique<HelloDXR>();
    SampleConfig config;
    config.windowDesc.title = "HelloDXR";
    config.windowDesc.resizableWindow = true;

    Sample::run(config, pRenderer);
}
