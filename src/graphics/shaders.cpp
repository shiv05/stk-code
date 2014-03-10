//  SuperTuxKart - a fun racing game with go-kart
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#define SHADER_NAMES

#include "graphics/callbacks.hpp"
#include "graphics/irr_driver.hpp"
#include "graphics/shaders.hpp"
#include "io/file_manager.hpp"
#include "utils/log.hpp"
#include "graphics/glwrap.hpp"
#include <assert.h>
#include <IGPUProgrammingServices.h>

using namespace video;

Shaders::Shaders()
{
    // Callbacks
    memset(m_callbacks, 0, sizeof(m_callbacks));

    m_callbacks[ES_SKYBOX] = new SkyboxProvider();
    m_callbacks[ES_WATER] = new WaterShaderProvider();
    m_callbacks[ES_GRASS] = new GrassShaderProvider();
    m_callbacks[ES_BUBBLES] = new BubbleEffectProvider();
    m_callbacks[ES_MOTIONBLUR] = new MotionBlurProvider();
    m_callbacks[ES_GAUSSIAN3V] = m_callbacks[ES_GAUSSIAN3H] = new GaussianBlurProvider();
    m_callbacks[ES_MIPVIZ] = new MipVizProvider();
    m_callbacks[ES_COLORIZE] = new ColorizeProvider();
    m_callbacks[ES_SUNLIGHT] = new SunLightProvider();
    m_callbacks[ES_MLAA_COLOR1] = new MLAAColor1Provider();
    m_callbacks[ES_MLAA_BLEND2] = new MLAABlend2Provider();
    m_callbacks[ES_MLAA_NEIGH3] = new MLAANeigh3Provider();
    m_callbacks[ES_SHADOWPASS] = new ShadowPassProvider();
    m_callbacks[ES_SHADOW_IMPORTANCE] = new ShadowImportanceProvider();
    m_callbacks[ES_COLLAPSE] = new CollapseProvider();
    m_callbacks[ES_MULTIPLY_ADD] = new MultiplyProvider();
    m_callbacks[ES_SHADOWGEN] = new ShadowGenProvider();
    m_callbacks[ES_DISPLACE] = new DisplaceProvider();

    for(s32 i=0 ; i < ES_COUNT ; i++)
        m_shaders[i] = -1;

    loadShaders();
}

GLuint quad_vbo;

static void initQuadVBO()
{
	const float quad_vertex[] = {
		-1., -1., 0., 0., // UpperLeft
		-1., 1., 0., 1., // LowerLeft
		1., -1., 1., 0., // UpperRight
		1., 1., 1., 1., // LowerRight
	};
	glGenBuffers(1, &quad_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, quad_vbo);
	glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad_vertex, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// It should be possible to merge it with previous one...
GLuint quad_buffer;

static void initQuadBuffer()
{
	const float quad_vertex[] = {
		-1., -1., -1., 1., // UpperLeft
		-1., 1., -1., -1., // LowerLeft
		1., -1., 1., 1., // UpperRight
		1., 1., 1., -1., // LowerRight
	};
	glGenBuffers(1, &quad_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, quad_buffer);
	glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad_vertex, GL_STATIC_DRAW);
}

GLuint SharedObject::billboardvbo = 0;

static void initBillboardVBO()
{
    float quad[] = {
        -.5, -.5, 0., 1.,
        -.5, .5, 0., 0.,
        .5, -.5, 1., 1.,
        .5, .5, 1., 0.,
    };
    glGenBuffers(1, &(SharedObject::billboardvbo));
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::billboardvbo);
    glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad, GL_STATIC_DRAW);
}

void Shaders::loadShaders()
{
    const std::string &dir = file_manager->getAsset(FileManager::SHADER, "");

    IGPUProgrammingServices * const gpu = irr_driver->getVideoDriver()->getGPUProgrammingServices();

    #define glsl(a, b, c) gpu->addHighLevelShaderMaterialFromFiles((a).c_str(), (b).c_str(), (IShaderConstantSetCallBack*) c)
    #define glslmat(a, b, c, d) gpu->addHighLevelShaderMaterialFromFiles((a).c_str(), (b).c_str(), (IShaderConstantSetCallBack*) c, d)
    #define glsl_noinput(a, b) gpu->addHighLevelShaderMaterialFromFiles((a).c_str(), (b).c_str(), (IShaderConstantSetCallBack*) 0)

    // Save previous shaders (used in case some shaders don't compile)
    int saved_shaders[ES_COUNT];
    memcpy(saved_shaders, m_shaders, sizeof(m_shaders));

    // Ok, go
    m_shaders[ES_NORMAL_MAP] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");
    m_shaders[ES_NORMAL_MAP_LIGHTMAP] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");

    m_shaders[ES_SKYBOX] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                   m_callbacks[ES_SKYBOX], EMT_TRANSPARENT_ALPHA_CHANNEL);

    m_shaders[ES_SPLATTING] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");

    m_shaders[ES_WATER] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_WATER], EMT_TRANSPARENT_ALPHA_CHANNEL);
    m_shaders[ES_WATER_SURFACE] = glsl(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_WATER]);

    m_shaders[ES_SPHERE_MAP] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");

    m_shaders[ES_GRASS] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_GRASS], EMT_TRANSPARENT_ALPHA_CHANNEL);
    m_shaders[ES_GRASS_REF] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_GRASS], EMT_TRANSPARENT_ALPHA_CHANNEL_REF);

    m_shaders[ES_BUBBLES] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_BUBBLES], EMT_TRANSPARENT_ALPHA_CHANNEL);

    m_shaders[ES_RAIN] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_RAIN], EMT_TRANSPARENT_ALPHA_CHANNEL);

    m_shaders[ES_MOTIONBLUR] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_MOTIONBLUR]);

    m_shaders[ES_GAUSSIAN3H] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_GAUSSIAN3H], EMT_SOLID);
    m_shaders[ES_GAUSSIAN3V] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_GAUSSIAN3V], EMT_SOLID);

    m_shaders[ES_MIPVIZ] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_MIPVIZ], EMT_SOLID);

    m_shaders[ES_COLORIZE] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_COLORIZE], EMT_SOLID);

    m_shaders[ES_OBJECTPASS] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");
    m_shaders[ES_OBJECT_UNLIT] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");
    m_shaders[ES_OBJECTPASS_REF] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");
    m_shaders[ES_OBJECTPASS_RIMLIT] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");

    m_shaders[ES_SUNLIGHT] = glsl_noinput(dir + "pass.vert", dir + "pass.frag");

    m_shaders[ES_MLAA_COLOR1] = glsl(dir + "mlaa_offset.vert", dir + "mlaa_color1.frag",
                                    m_callbacks[ES_MLAA_COLOR1]);
    m_shaders[ES_MLAA_BLEND2] = glsl(dir + "pass.vert", dir + "mlaa_blend2.frag",
                                    m_callbacks[ES_MLAA_BLEND2]);
    m_shaders[ES_MLAA_NEIGH3] = glsl(dir + "mlaa_offset.vert", dir + "mlaa_neigh3.frag",
                                    m_callbacks[ES_MLAA_NEIGH3]);

    m_shaders[ES_SHADOWPASS] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_SHADOWPASS]);

    m_shaders[ES_SHADOW_IMPORTANCE] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_SHADOW_IMPORTANCE]);

    m_shaders[ES_COLLAPSE] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_COLLAPSE]);
    m_shaders[ES_SHADOW_WARPH] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_COLLAPSE]);
    m_shaders[ES_SHADOW_WARPV] = glsl(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_COLLAPSE]);

    m_shaders[ES_MULTIPLY_ADD] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_MULTIPLY_ADD], EMT_ONETEXTURE_BLEND);

    m_shaders[ES_PENUMBRAH] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_GAUSSIAN3H], EMT_SOLID);
    m_shaders[ES_PENUMBRAV] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_GAUSSIAN3H], EMT_SOLID);
    m_shaders[ES_SHADOWGEN] = glslmat(dir + "pass.vert", dir + "pass.frag",
                                    m_callbacks[ES_SHADOWGEN], EMT_SOLID);

    m_shaders[ES_CAUSTICS] = glslmat(dir + "pass.vert", dir + "pass.frag",
        m_callbacks[ES_CAUSTICS], EMT_SOLID);

    m_shaders[ES_DISPLACE] = glsl(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_DISPLACE]);

    m_shaders[ES_PASSFAR] = glsl(dir + "pass.vert", dir + "pass.frag",
                                  m_callbacks[ES_COLORIZE]);

    // Check that all successfully loaded
    for (s32 i = 0; i < ES_COUNT; i++) {

        // Old Intel Windows drivers fail here.
        // It's an artist option, so not necessary to play.
        if (i == ES_MIPVIZ)
            continue;

        check(i);
    }

    #undef glsl
    #undef glslmat
    #undef glsl_noinput

    // In case we're reloading and a shader didn't compile: keep the previous, working one
    for(s32 i=0 ; i < ES_COUNT ; i++)
    {
        if(m_shaders[i] == -1)
            m_shaders[i] = saved_shaders[i];
    }

	initGL();
	initQuadVBO();
	initQuadBuffer();
    initBillboardVBO();
	FullScreenShader::BloomBlendShader::init();
	FullScreenShader::BloomShader::init();
	FullScreenShader::ColorLevelShader::init();
	FullScreenShader::FogShader::init();
	FullScreenShader::Gaussian3HBlurShader::init();
	FullScreenShader::Gaussian3VBlurShader::init();
	FullScreenShader::Gaussian6HBlurShader::init();
	FullScreenShader::Gaussian6VBlurShader::init();
    FullScreenShader::PenumbraHShader::init();
    FullScreenShader::PenumbraVShader::init();
	FullScreenShader::GlowShader::init();
	FullScreenShader::PassThroughShader::init();
	FullScreenShader::SSAOShader::init();
	FullScreenShader::SunLightShader::init();
    FullScreenShader::DiffuseEnvMapShader::init();
    FullScreenShader::ShadowedSunLightShader::init();
    FullScreenShader::MotionBlurShader::init();
    FullScreenShader::GodFadeShader::init();
    FullScreenShader::GodRayShader::init();
	MeshShader::ColorizeShader::init();
	MeshShader::NormalMapShader::init();
	MeshShader::ObjectPass1Shader::init();
	MeshShader::ObjectRefPass1Shader::init();
	MeshShader::ObjectPass2Shader::init();
	MeshShader::DetailledObjectPass2Shader::init();
	MeshShader::ObjectRimLimitShader::init();
	MeshShader::UntexturedObjectShader::init();
	MeshShader::ObjectRefPass2Shader::init();
	MeshShader::ObjectUnlitShader::init();
	MeshShader::SphereMapShader::init();
	MeshShader::SplattingShader::init();
	MeshShader::GrassPass1Shader::init();
	MeshShader::GrassPass2Shader::init();
    MeshShader::CausticsShader::init();
	MeshShader::BubbleShader::init();
	MeshShader::TransparentShader::init();
    MeshShader::TransparentFogShader::init();
	MeshShader::BillboardShader::init();
    MeshShader::PointLightShader::init();
	MeshShader::DisplaceShader::init();
    MeshShader::DisplaceMaskShader::init();
    MeshShader::ShadowShader::init();
    MeshShader::RefShadowShader::init();
    MeshShader::GrassShadowShader::init();
    MeshShader::SkyboxShader::init();
	ParticleShader::FlipParticleRender::init();
	ParticleShader::HeightmapSimulationShader::init();
	ParticleShader::SimpleParticleRender::init();
	ParticleShader::SimpleSimulationShader::init();
	UIShader::ColoredRectShader::init();
	UIShader::ColoredTextureRectShader::init();
	UIShader::TextureRectShader::init();
}

Shaders::~Shaders()
{
    u32 i;
    for (i = 0; i < ES_COUNT; i++)
    {
        if (i == ES_GAUSSIAN3V || !m_callbacks[i]) continue;
        delete m_callbacks[i];
    }
}

E_MATERIAL_TYPE Shaders::getShader(const ShaderType num) const
{
    assert(num < ES_COUNT);

    return (E_MATERIAL_TYPE) m_shaders[num];
}

void Shaders::check(const int num) const
{
    if (m_shaders[num] == -1)
    {
        Log::error("shaders", "Shader %s failed to load. Update your drivers, if the issue "
                              "persists, report a bug to us.", shader_names[num] + 3);
    }
}

namespace MeshShader
{

    // Solid Normal and depth pass shaders
	GLuint ObjectPass1Shader::Program;
	GLuint ObjectPass1Shader::attrib_position;
	GLuint ObjectPass1Shader::attrib_normal;
	GLuint ObjectPass1Shader::uniform_MVP;
	GLuint ObjectPass1Shader::uniform_TIMV;

	void ObjectPass1Shader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass1.vert").c_str(), file_manager->getAsset("shaders/object_pass1.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_normal = glGetAttribLocation(Program, "Normal");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
	}

	void ObjectPass1Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
	}

	GLuint ObjectRefPass1Shader::Program;
	GLuint ObjectRefPass1Shader::attrib_position;
	GLuint ObjectRefPass1Shader::attrib_normal;
	GLuint ObjectRefPass1Shader::attrib_texcoord;
	GLuint ObjectRefPass1Shader::uniform_MVP;
	GLuint ObjectRefPass1Shader::uniform_TIMV;
    GLuint ObjectRefPass1Shader::uniform_TM;
	GLuint ObjectRefPass1Shader::uniform_tex;

	void ObjectRefPass1Shader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/objectref_pass1.vert").c_str(), file_manager->getAsset("shaders/objectref_pass1.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_normal = glGetAttribLocation(Program, "Normal");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
		uniform_tex = glGetUniformLocation(Program, "tex");
	}

    void ObjectRefPass1Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView, const core::matrix4 &TextureMatrix, unsigned TU_tex)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
		glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
		glUniform1i(uniform_tex, TU_tex);
	}

    GLuint GrassPass1Shader::Program;
    GLuint GrassPass1Shader::attrib_position;
    GLuint GrassPass1Shader::attrib_texcoord;
    GLuint GrassPass1Shader::attrib_normal;
    GLuint GrassPass1Shader::attrib_color;
    GLuint GrassPass1Shader::uniform_MVP;
    GLuint GrassPass1Shader::uniform_TIMV;
    GLuint GrassPass1Shader::uniform_tex;
    GLuint GrassPass1Shader::uniform_windDir;

    void GrassPass1Shader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/grass_pass1.vert").c_str(), file_manager->getAsset("shaders/objectref_pass1.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        attrib_normal = glGetAttribLocation(Program, "Normal");
        attrib_color = glGetAttribLocation(Program, "Color");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_windDir = glGetUniformLocation(Program, "windDir");
    }

    void GrassPass1Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView, const core::vector3df &windDirection, unsigned TU_tex)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
        glUniform3f(uniform_windDir, windDirection.X, windDirection.Y, windDirection.Z);
        glUniform1i(uniform_tex, TU_tex);
    }

    GLuint NormalMapShader::Program;
    GLuint NormalMapShader::attrib_position;
    GLuint NormalMapShader::attrib_texcoord;
    GLuint NormalMapShader::attrib_tangent;
    GLuint NormalMapShader::attrib_bitangent;
    GLuint NormalMapShader::uniform_MVP;
    GLuint NormalMapShader::uniform_TIMV;
    GLuint NormalMapShader::uniform_normalMap;

    void NormalMapShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/normalmap.vert").c_str(), file_manager->getAsset("shaders/normalmap.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        attrib_tangent = glGetAttribLocation(Program, "Tangent");
        attrib_bitangent = glGetAttribLocation(Program, "Bitangent");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
        uniform_normalMap = glGetUniformLocation(Program, "normalMap");
    }

    void NormalMapShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView, unsigned TU_normalMap)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
        glUniform1i(uniform_normalMap, TU_normalMap);
    }

    // Solid Lit pass shaders

	GLuint ObjectPass2Shader::Program;
	GLuint ObjectPass2Shader::attrib_position;
	GLuint ObjectPass2Shader::attrib_texcoord;
	GLuint ObjectPass2Shader::uniform_MVP;
    GLuint ObjectPass2Shader::uniform_TM;
	GLuint ObjectPass2Shader::uniform_screen;
	GLuint ObjectPass2Shader::uniform_ambient;
    GLuint ObjectPass2Shader::TU_Albedo;

	void ObjectPass2Shader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/object_pass2.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_Albedo = 3;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUseProgram(0);
	}

    void ObjectPass2Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TextureMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}

	GLuint DetailledObjectPass2Shader::Program;
	GLuint DetailledObjectPass2Shader::attrib_position;
	GLuint DetailledObjectPass2Shader::attrib_texcoord;
	GLuint DetailledObjectPass2Shader::attrib_second_texcoord;
	GLuint DetailledObjectPass2Shader::uniform_MVP;
	GLuint DetailledObjectPass2Shader::uniform_screen;
	GLuint DetailledObjectPass2Shader::uniform_ambient;
    GLuint DetailledObjectPass2Shader::TU_Albedo;
    GLuint DetailledObjectPass2Shader::TU_detail;

	void DetailledObjectPass2Shader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/detailledobject_pass2.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		attrib_second_texcoord = glGetAttribLocation(Program, "SecondTexcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_Detail = glGetUniformLocation(Program, "Detail");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
        uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_Albedo = 3;
        TU_detail = 4;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUniform1i(uniform_Detail, TU_detail);
        glUseProgram(0);
	}

	void DetailledObjectPass2Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}

	GLuint ObjectUnlitShader::Program;
	GLuint ObjectUnlitShader::attrib_position;
	GLuint ObjectUnlitShader::attrib_texcoord;
	GLuint ObjectUnlitShader::uniform_MVP;
	GLuint ObjectUnlitShader::TU_tex;

	void ObjectUnlitShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/object_unlit.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		GLuint uniform_tex = glGetUniformLocation(Program, "tex");
        TU_tex = 3;

        glUseProgram(Program);
        glUniform1i(uniform_tex, TU_tex);
        glUseProgram(0);
	}

	void ObjectUnlitShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
	}

	GLuint ObjectRimLimitShader::Program;
	GLuint ObjectRimLimitShader::attrib_position;
	GLuint ObjectRimLimitShader::attrib_texcoord;
	GLuint ObjectRimLimitShader::attrib_normal;
	GLuint ObjectRimLimitShader::uniform_MVP;
	GLuint ObjectRimLimitShader::uniform_TIMV;
    GLuint ObjectRimLimitShader::uniform_TM;
	GLuint ObjectRimLimitShader::uniform_screen;
	GLuint ObjectRimLimitShader::uniform_ambient;
    GLuint ObjectRimLimitShader::TU_Albedo;

	void ObjectRimLimitShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/objectpass_rimlit.vert").c_str(), file_manager->getAsset("shaders/objectpass_rimlit.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		attrib_normal = glGetAttribLocation(Program, "Normal");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_Albedo = 3;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUseProgram(0);
	}

    void ObjectRimLimitShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView, const core::matrix4 &TextureMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}

	GLuint UntexturedObjectShader::Program;
	GLuint UntexturedObjectShader::attrib_position;
	GLuint UntexturedObjectShader::attrib_color;
	GLuint UntexturedObjectShader::uniform_MVP;
	GLuint UntexturedObjectShader::uniform_screen;
	GLuint UntexturedObjectShader::uniform_ambient;

	void UntexturedObjectShader::init()
	{
	  Program = LoadProgram(file_manager->getAsset("shaders/untextured_object.vert").c_str(), file_manager->getAsset("shaders/untextured_object.frag").c_str());
	  attrib_position = glGetAttribLocation(Program, "Position");
	  attrib_color = glGetAttribLocation(Program, "Color");
	  uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
      GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
      GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
      GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
	  uniform_screen = glGetUniformLocation(Program, "screen");
	  uniform_ambient = glGetUniformLocation(Program, "ambient");

      glUseProgram(Program);
      glUniform1i(uniform_DiffuseMap, 0);
      glUniform1i(uniform_SpecularMap, 1);
      glUniform1i(uniform_SSAO, 2);
      glUseProgram(0);
	}

	void UntexturedObjectShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix)
	{
	  glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
	  glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
	  const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
	  glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}


	GLuint ObjectRefPass2Shader::Program;
	GLuint ObjectRefPass2Shader::attrib_position;
	GLuint ObjectRefPass2Shader::attrib_texcoord;
	GLuint ObjectRefPass2Shader::uniform_MVP;
    GLuint ObjectRefPass2Shader::uniform_TM;
	GLuint ObjectRefPass2Shader::uniform_screen;
	GLuint ObjectRefPass2Shader::uniform_ambient;
    GLuint ObjectRefPass2Shader::TU_Albedo;

	void ObjectRefPass2Shader::init()
	{
		initGL();
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/objectref_pass2.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
		GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_Albedo = 3;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUseProgram(0);
	}

	void ObjectRefPass2Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TextureMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}

	GLuint GrassPass2Shader::Program;
	GLuint GrassPass2Shader::attrib_position;
	GLuint GrassPass2Shader::attrib_texcoord;
	GLuint GrassPass2Shader::attrib_color;
	GLuint GrassPass2Shader::uniform_MVP;
	GLuint GrassPass2Shader::uniform_screen;
	GLuint GrassPass2Shader::uniform_ambient;
	GLuint GrassPass2Shader::uniform_windDir;
    GLuint GrassPass2Shader::TU_Albedo;

	void GrassPass2Shader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/grass_pass2.vert").c_str(), file_manager->getAsset("shaders/objectref_pass2.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		attrib_color = glGetAttribLocation(Program, "Color");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
		uniform_windDir = glGetUniformLocation(Program, "windDir");
        TU_Albedo = 3;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUseProgram(0);
	}

	void GrassPass2Shader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::vector3df &windDirection)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
		glUniform3f(uniform_windDir, windDirection.X, windDirection.Y, windDirection.Z);
	}

	GLuint SphereMapShader::Program;
	GLuint SphereMapShader::attrib_position;
	GLuint SphereMapShader::attrib_normal;
	GLuint SphereMapShader::uniform_MVP;
	GLuint SphereMapShader::uniform_TIMV;
    GLuint SphereMapShader::uniform_invproj;
    GLuint SphereMapShader::uniform_screen;
    GLuint SphereMapShader::TU_tex;

	void SphereMapShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass1.vert").c_str(), file_manager->getAsset("shaders/objectpass_spheremap.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_normal = glGetAttribLocation(Program, "Normal");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_TIMV = glGetUniformLocation(Program, "TransposeInverseModelView");
        GLuint uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_invproj = glGetUniformLocation(Program, "invproj");
        uniform_screen = glGetUniformLocation(Program, "screen");
        TU_tex = 3;

        glUseProgram(Program);
        glUniform1i(uniform_tex, TU_tex);
        glUseProgram(0);
	}

    void SphereMapShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TransposeInverseModelView, const core::matrix4 &InvProj, const core::vector2df& screen)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniformMatrix4fv(uniform_TIMV, 1, GL_FALSE, TransposeInverseModelView.pointer());
        glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProj.pointer());
        glUniform2f(uniform_screen, screen.X, screen.Y);
	}

	GLuint SplattingShader::Program;
	GLuint SplattingShader::attrib_position;
	GLuint SplattingShader::attrib_texcoord;
	GLuint SplattingShader::attrib_second_texcoord;
	GLuint SplattingShader::uniform_MVP;
	GLuint SplattingShader::uniform_screen;
	GLuint SplattingShader::uniform_ambient;
    GLuint SplattingShader::TU_tex_layout;
    GLuint SplattingShader::TU_tex_detail0;
    GLuint SplattingShader::TU_tex_detail1;
    GLuint SplattingShader::TU_tex_detail2;
    GLuint SplattingShader::TU_tex_detail3;

	void SplattingShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/splatting.vert").c_str(), file_manager->getAsset("shaders/splatting.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		attrib_second_texcoord = glGetAttribLocation(Program, "SecondTexcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        GLuint uniform_tex_layout = glGetUniformLocation(Program, "tex_layout");
        GLuint uniform_tex_detail0 = glGetUniformLocation(Program, "tex_detail0");
        GLuint uniform_tex_detail1 = glGetUniformLocation(Program, "tex_detail1");
        GLuint uniform_tex_detail2 = glGetUniformLocation(Program, "tex_detail2");
        GLuint uniform_tex_detail3 = glGetUniformLocation(Program, "tex_detail3");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_tex_layout = 3;
        TU_tex_detail0 = 4;
        TU_tex_detail1 = 5;
        TU_tex_detail2 = 6;
        TU_tex_detail3 = 7;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_tex_layout, TU_tex_layout);
        glUniform1i(uniform_tex_detail0, TU_tex_detail0);
        glUniform1i(uniform_tex_detail1, TU_tex_detail1);
        glUniform1i(uniform_tex_detail2, TU_tex_detail2);
        glUniform1i(uniform_tex_detail3, TU_tex_detail3);
        glUseProgram(0);
	}

	void SplattingShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
		const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
		glUniform3f(uniform_ambient, s.r, s.g, s.b);
	}

    GLuint CausticsShader::Program;
    GLuint CausticsShader::attrib_position;
    GLuint CausticsShader::attrib_texcoord;
    GLuint CausticsShader::uniform_MVP;
    GLuint CausticsShader::uniform_dir;
    GLuint CausticsShader::uniform_dir2;
    GLuint CausticsShader::uniform_screen;
    GLuint CausticsShader::uniform_ambient;
    GLuint CausticsShader::TU_Albedo;
    GLuint CausticsShader::TU_caustictex;

    void CausticsShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/caustics.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_dir = glGetUniformLocation(Program, "dir");
        uniform_dir2 = glGetUniformLocation(Program, "dir2");
        GLuint uniform_Albedo = glGetUniformLocation(Program, "Albedo");
        GLuint uniform_caustictex = glGetUniformLocation(Program, "caustictex");
        GLuint uniform_DiffuseMap = glGetUniformLocation(Program, "DiffuseMap");
        GLuint uniform_SpecularMap = glGetUniformLocation(Program, "SpecularMap");
        GLuint uniform_SSAO = glGetUniformLocation(Program, "SSAO");
        uniform_screen = glGetUniformLocation(Program, "screen");
        uniform_ambient = glGetUniformLocation(Program, "ambient");
        TU_Albedo = 3;
        TU_caustictex = 4;

        glUseProgram(Program);
        glUniform1i(uniform_DiffuseMap, 0);
        glUniform1i(uniform_SpecularMap, 1);
        glUniform1i(uniform_SSAO, 2);
        glUniform1i(uniform_Albedo, TU_Albedo);
        glUniform1i(uniform_caustictex, TU_caustictex);
        glUseProgram(0);
    }

    void CausticsShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::vector2df &dir, const core::vector2df &dir2, const core::vector2df &screen)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniform2f(uniform_dir, dir.X, dir.Y);
        glUniform2f(uniform_dir2, dir2.X, dir2.Y);
        glUniform2f(uniform_screen, screen.X, screen.Y);
        const video::SColorf s = irr_driver->getSceneManager()->getAmbientLight();
        glUniform3f(uniform_ambient, s.r, s.g, s.b);
    }

	GLuint BubbleShader::Program;
	GLuint BubbleShader::attrib_position;
	GLuint BubbleShader::attrib_texcoord;
	GLuint BubbleShader::uniform_MVP;
	GLuint BubbleShader::uniform_tex;
	GLuint BubbleShader::uniform_time;
	GLuint BubbleShader::uniform_transparency;

	void BubbleShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/bubble.vert").c_str(), file_manager->getAsset("shaders/bubble.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_time = glGetUniformLocation(Program, "time");
		uniform_transparency = glGetUniformLocation(Program, "transparency");
	}
    void BubbleShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, unsigned TU_tex, float time, float transparency)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniform1i(uniform_tex, TU_tex);
		glUniform1f(uniform_time, time);
		glUniform1f(uniform_transparency, transparency);
	}

	GLuint TransparentShader::Program;
	GLuint TransparentShader::attrib_position;
	GLuint TransparentShader::attrib_texcoord;
	GLuint TransparentShader::uniform_MVP;
    GLuint TransparentShader::uniform_TM;
	GLuint TransparentShader::uniform_tex;

	void TransparentShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/transparent.vert").c_str(), file_manager->getAsset("shaders/transparent.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
		uniform_tex = glGetUniformLocation(Program, "tex");
	}

    void TransparentShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TextureMatrix, unsigned TU_tex)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
		glUniform1i(uniform_tex, TU_tex);
	}

    GLuint TransparentFogShader::Program;
    GLuint TransparentFogShader::attrib_position;
    GLuint TransparentFogShader::attrib_texcoord;
    GLuint TransparentFogShader::uniform_MVP;
    GLuint TransparentFogShader::uniform_TM;
    GLuint TransparentFogShader::uniform_tex;
    GLuint TransparentFogShader::uniform_fogmax;
    GLuint TransparentFogShader::uniform_startH;
    GLuint TransparentFogShader::uniform_endH;
    GLuint TransparentFogShader::uniform_start;
    GLuint TransparentFogShader::uniform_end;
    GLuint TransparentFogShader::uniform_col;
    GLuint TransparentFogShader::uniform_screen;
    GLuint TransparentFogShader::uniform_ipvmat;

    void TransparentFogShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/transparent.vert").c_str(), file_manager->getAsset("shaders/transparentfog.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_TM = glGetUniformLocation(Program, "TextureMatrix");
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_fogmax = glGetUniformLocation(Program, "fogmax");
        uniform_startH = glGetUniformLocation(Program, "startH");
        uniform_endH = glGetUniformLocation(Program, "endH");
        uniform_start = glGetUniformLocation(Program, "start");
        uniform_end = glGetUniformLocation(Program, "end");
        uniform_col = glGetUniformLocation(Program, "col");
        uniform_screen = glGetUniformLocation(Program, "screen");
        uniform_ipvmat = glGetUniformLocation(Program, "ipvmat");
    }

    void TransparentFogShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &TextureMatrix, const core::matrix4 &ipvmat, float fogmax, float startH, float endH, float start, float end, const core::vector3df &col, const core::vector3df &campos, unsigned TU_tex)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_TM, 1, GL_FALSE, TextureMatrix.pointer());
        glUniform1f(uniform_fogmax, fogmax);
        glUniform1f(uniform_startH, startH);
        glUniform1f(uniform_endH, endH);
        glUniform1f(uniform_start, start);
        glUniform1f(uniform_end, end);
        glUniform3f(uniform_col, col.X, col.Y, col.Z);
        glUniform2f(uniform_screen, UserConfigParams::m_width, UserConfigParams::m_height);
        glUniformMatrix4fv(uniform_ipvmat, 1, GL_FALSE, ipvmat.pointer());
        glUniform1i(uniform_tex, TU_tex);
    }

    GLuint PointLightShader::Program;
    GLuint PointLightShader::attrib_Position;
    GLuint PointLightShader::attrib_Color;
    GLuint PointLightShader::attrib_Energy;
    GLuint PointLightShader::attrib_Corner;
    GLuint PointLightShader::uniform_ntex;
    GLuint PointLightShader::uniform_dtex;
    GLuint PointLightShader::uniform_spec;
    GLuint PointLightShader::uniform_screen;
    GLuint PointLightShader::uniform_invproj;
    GLuint PointLightShader::uniform_VM;
    GLuint PointLightShader::uniform_PM;

    void PointLightShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/pointlight.vert").c_str(), file_manager->getAsset("shaders/pointlight.frag").c_str());
        attrib_Position = glGetAttribLocation(Program, "Position");
        attrib_Color = glGetAttribLocation(Program, "Color");
        attrib_Energy = glGetAttribLocation(Program, "Energy");
        attrib_Corner = glGetAttribLocation(Program, "Corner");
        uniform_ntex = glGetUniformLocation(Program, "ntex");
        uniform_dtex = glGetUniformLocation(Program, "dtex");
        uniform_spec = glGetUniformLocation(Program, "spec");
        uniform_invproj = glGetUniformLocation(Program, "invproj");
        uniform_screen = glGetUniformLocation(Program, "screen");
        uniform_VM = glGetUniformLocation(Program, "ViewMatrix");
        uniform_PM = glGetUniformLocation(Program, "ProjectionMatrix");
    }

    void PointLightShader::setUniforms(const core::matrix4 &ViewMatrix, const core::matrix4 &ProjMatrix, const core::matrix4 &InvProjMatrix, const core::vector2df &screen, unsigned spec, unsigned TU_ntex, unsigned TU_dtex)
    {
        glUniform1f(uniform_spec, 200);
        glUniform2f(uniform_screen, screen.X, screen.Y);
        glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProjMatrix.pointer());
        glUniformMatrix4fv(uniform_VM, 1, GL_FALSE, ViewMatrix.pointer());
        glUniformMatrix4fv(uniform_PM, 1, GL_FALSE, ProjMatrix.pointer());

        glUniform1i(uniform_ntex, TU_ntex);
        glUniform1i(uniform_dtex, TU_dtex);
    }
	
	GLuint BillboardShader::Program;
	GLuint BillboardShader::attrib_corner;
	GLuint BillboardShader::attrib_texcoord;
	GLuint BillboardShader::uniform_MV;
	GLuint BillboardShader::uniform_P;
	GLuint BillboardShader::uniform_tex;
	GLuint BillboardShader::uniform_Position;
	GLuint BillboardShader::uniform_Size;

	void BillboardShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/billboard.vert").c_str(), file_manager->getAsset("shaders/billboard.frag").c_str());
		attrib_corner = glGetAttribLocation(Program, "Corner");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		uniform_MV = glGetUniformLocation(Program, "ModelViewMatrix");
		uniform_P = glGetUniformLocation(Program, "ProjectionMatrix");
		uniform_Position = glGetUniformLocation(Program, "Position");
		uniform_Size = glGetUniformLocation(Program, "Size");
		uniform_tex = glGetUniformLocation(Program, "tex");
	}

	void BillboardShader::setUniforms(const core::matrix4 &ModelViewMatrix, const core::matrix4 &ProjectionMatrix, const core::vector3df &Position, const core::dimension2d<float> &size, unsigned TU_tex)
	{
		glUniformMatrix4fv(uniform_MV, 1, GL_FALSE, ModelViewMatrix.pointer());
		glUniformMatrix4fv(uniform_P, 1, GL_FALSE, ProjectionMatrix.pointer());
		glUniform3f(uniform_Position, Position.X, Position.Y, Position.Z);
		glUniform2f(uniform_Size, size.Width, size.Height);
		glUniform1i(uniform_tex, TU_tex);
	}

	GLuint ColorizeShader::Program;
	GLuint ColorizeShader::attrib_position;
	GLuint ColorizeShader::uniform_MVP;
	GLuint ColorizeShader::uniform_col;

	void ColorizeShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/colorize.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_col = glGetUniformLocation(Program, "col");
	}

	void ColorizeShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, float r, float g, float b)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniform3f(uniform_col, r, g, b);
	}

    GLuint ShadowShader::Program;
    GLuint ShadowShader::attrib_position;
    GLuint ShadowShader::uniform_MVP;

    void ShadowShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/shadow.vert").c_str(), file_manager->getAsset("shaders/shadow.geom").c_str(), file_manager->getAsset("shaders/white.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
    }

    void ShadowShader::setUniforms(const std::vector<core::matrix4> &ModelViewProjectionMatrix)
    {
        size_t size = ModelViewProjectionMatrix.size();
        float *tmp = new float[16 * size];
        for (unsigned i = 0; i < size; i++) {
            memcpy(&tmp[16 * i], ModelViewProjectionMatrix[i].pointer(), 16 * sizeof(float));
        }
        glUniformMatrix4fv(uniform_MVP, size, GL_FALSE, tmp);
        delete[] tmp;
    }

    GLuint RefShadowShader::Program;
    GLuint RefShadowShader::attrib_position;
    GLuint RefShadowShader::attrib_texcoord;
    GLuint RefShadowShader::uniform_MVP;
    GLuint RefShadowShader::uniform_tex;

    void RefShadowShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/shadow.vert").c_str(), file_manager->getAsset("shaders/shadow.geom").c_str(), file_manager->getAsset("shaders/object_unlit.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_tex = glGetUniformLocation(Program, "tex");
    }

    void RefShadowShader::setUniforms(const std::vector<core::matrix4> &ModelViewProjectionMatrix, unsigned TU_tex)
    {
        size_t size = ModelViewProjectionMatrix.size();
        float *tmp = new float[16 * size];
        for (unsigned i = 0; i < size; i++) {
            memcpy(&tmp[16 * i], ModelViewProjectionMatrix[i].pointer(), 16 * sizeof(float));
        }
        glUniformMatrix4fv(uniform_MVP, size, GL_FALSE, tmp);
        glUniform1i(uniform_tex, TU_tex);
        delete[] tmp;
    }

    GLuint GrassShadowShader::Program;
    GLuint GrassShadowShader::attrib_position;
    GLuint GrassShadowShader::attrib_texcoord;
    GLuint GrassShadowShader::attrib_color;
    GLuint GrassShadowShader::uniform_MVP;
    GLuint GrassShadowShader::uniform_tex;
    GLuint GrassShadowShader::uniform_windDir;

    void GrassShadowShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/grass_pass2.vert").c_str(), file_manager->getAsset("shaders/object_unlit.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_tex = glGetUniformLocation(Program, "tex");
        attrib_color = glGetAttribLocation(Program, "Color");
        uniform_windDir = glGetUniformLocation(Program, "windDir");
    }

    void GrassShadowShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::vector3df &windDirection, unsigned TU_tex)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniform1i(uniform_tex, TU_tex);
        glUniform3f(uniform_windDir, windDirection.X, windDirection.Y, windDirection.Z);
    }

    GLuint DisplaceMaskShader::Program;
    GLuint DisplaceMaskShader::attrib_position;
    GLuint DisplaceMaskShader::uniform_MVP;

    void DisplaceMaskShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/displace.vert").c_str(), file_manager->getAsset("shaders/white.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
    }

    void DisplaceMaskShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
    }

	GLuint DisplaceShader::Program;
	GLuint DisplaceShader::attrib_position;
	GLuint DisplaceShader::attrib_texcoord;
	GLuint DisplaceShader::attrib_second_texcoord;
	GLuint DisplaceShader::uniform_MVP;
	GLuint DisplaceShader::uniform_MV;
	GLuint DisplaceShader::uniform_displacement_tex;
    GLuint DisplaceShader::uniform_mask_tex;
    GLuint DisplaceShader::uniform_color_tex;
	GLuint DisplaceShader::uniform_dir;
	GLuint DisplaceShader::uniform_dir2;
    GLuint DisplaceShader::uniform_screen;

	void DisplaceShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/displace.vert").c_str(), file_manager->getAsset("shaders/displace.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "Position");
		attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
		attrib_second_texcoord = glGetAttribLocation(Program, "SecondTexcoord");
		uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
		uniform_MV = glGetUniformLocation(Program, "ModelViewMatrix");
		uniform_displacement_tex = glGetUniformLocation(Program, "displacement_tex");
        uniform_color_tex = glGetUniformLocation(Program, "color_tex");
        uniform_mask_tex = glGetUniformLocation(Program, "mask_tex");
		uniform_dir = glGetUniformLocation(Program, "dir");
		uniform_dir2 = glGetUniformLocation(Program, "dir2");
        uniform_screen = glGetUniformLocation(Program, "screen");
	}

	void DisplaceShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &ModelViewMatrix, const core::vector2df &dir, const core::vector2df &dir2, const core::vector2df &screen, unsigned TU_displacement_tex, unsigned TU_mask_tex, unsigned TU_color_tex)
	{
		glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
		glUniformMatrix4fv(uniform_MV, 1, GL_FALSE, ModelViewMatrix.pointer());
		glUniform2f(uniform_dir, dir.X, dir.Y);
		glUniform2f(uniform_dir2, dir2.X, dir2.Y);
        glUniform2f(uniform_screen, screen.X, screen.Y);
		glUniform1i(uniform_displacement_tex, TU_displacement_tex);
        glUniform1i(uniform_mask_tex, TU_mask_tex);
        glUniform1i(uniform_color_tex, TU_color_tex);
	}

    GLuint SkyboxShader::Program;
    GLuint SkyboxShader::attrib_position;
    GLuint SkyboxShader::uniform_MVP;
    GLuint SkyboxShader::uniform_tex;
    GLuint SkyboxShader::uniform_screen;
    GLuint SkyboxShader::uniform_InvProjView;

    void SkyboxShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/object_pass2.vert").c_str(), file_manager->getAsset("shaders/sky.frag").c_str());
        attrib_position = glGetAttribLocation(Program, "Position");
        uniform_MVP = glGetUniformLocation(Program, "ModelViewProjectionMatrix");
        uniform_InvProjView = glGetUniformLocation(Program, "InvProjView");
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_screen = glGetUniformLocation(Program, "screen");
    }

    void SkyboxShader::setUniforms(const core::matrix4 &ModelViewProjectionMatrix, const core::matrix4 &InvProjView, const core::vector2df &screen, unsigned TU_tex)
    {
        glUniformMatrix4fv(uniform_MVP, 1, GL_FALSE, ModelViewProjectionMatrix.pointer());
        glUniformMatrix4fv(uniform_InvProjView, 1, GL_FALSE, InvProjView.pointer());
        glUniform1i(uniform_tex, TU_tex);
        glUniform2f(uniform_screen, screen.X, screen.Y);
    }
}


namespace ParticleShader
{
	GLuint SimpleSimulationShader::Program;
	GLuint SimpleSimulationShader::attrib_position;
	GLuint SimpleSimulationShader::attrib_velocity;
	GLuint SimpleSimulationShader::attrib_lifetime;
	GLuint SimpleSimulationShader::attrib_initial_position;
	GLuint SimpleSimulationShader::attrib_initial_velocity;
	GLuint SimpleSimulationShader::attrib_initial_lifetime;
	GLuint SimpleSimulationShader::attrib_size;
	GLuint SimpleSimulationShader::attrib_initial_size;
	GLuint SimpleSimulationShader::uniform_sourcematrix;
	GLuint SimpleSimulationShader::uniform_dt;
	GLuint SimpleSimulationShader::uniform_level;
	GLuint SimpleSimulationShader::uniform_size_increase_factor;

	void SimpleSimulationShader::init()
	{
		const char *varyings[] = {
			"new_particle_position",
			"new_lifetime",
			"new_particle_velocity",
			"new_size",
		};
		Program = LoadTFBProgram(file_manager->getAsset("shaders/pointemitter.vert").c_str(), varyings, 4);

		uniform_dt = glGetUniformLocation(Program, "dt");
		uniform_sourcematrix = glGetUniformLocation(Program, "sourcematrix");
		uniform_level = glGetUniformLocation(Program, "level");
		uniform_size_increase_factor = glGetUniformLocation(Program, "size_increase_factor");

		attrib_position = glGetAttribLocation(Program, "particle_position");
		attrib_lifetime = glGetAttribLocation(Program, "lifetime");
		attrib_velocity = glGetAttribLocation(Program, "particle_velocity");
		attrib_size = glGetAttribLocation(Program, "size");
		attrib_initial_position = glGetAttribLocation(Program, "particle_position_initial");
		attrib_initial_lifetime = glGetAttribLocation(Program, "lifetime_initial");
		attrib_initial_velocity = glGetAttribLocation(Program, "particle_velocity_initial");
		attrib_initial_size = glGetAttribLocation(Program, "size_initial");
	}

	GLuint HeightmapSimulationShader::Program;
	GLuint HeightmapSimulationShader::attrib_position;
	GLuint HeightmapSimulationShader::attrib_velocity;
	GLuint HeightmapSimulationShader::attrib_lifetime;
	GLuint HeightmapSimulationShader::attrib_initial_position;
	GLuint HeightmapSimulationShader::attrib_initial_velocity;
	GLuint HeightmapSimulationShader::attrib_initial_lifetime;
	GLuint HeightmapSimulationShader::attrib_size;
	GLuint HeightmapSimulationShader::attrib_initial_size;
	GLuint HeightmapSimulationShader::uniform_sourcematrix;
	GLuint HeightmapSimulationShader::uniform_dt;
	GLuint HeightmapSimulationShader::uniform_level;
	GLuint HeightmapSimulationShader::uniform_size_increase_factor;
	GLuint HeightmapSimulationShader::uniform_track_x;
	GLuint HeightmapSimulationShader::uniform_track_z;
	GLuint HeightmapSimulationShader::uniform_track_x_len;
	GLuint HeightmapSimulationShader::uniform_track_z_len;
	GLuint HeightmapSimulationShader::uniform_heightmap;

	void HeightmapSimulationShader::init()
	{
		const char *varyings[] = {
			"new_particle_position",
			"new_lifetime",
			"new_particle_velocity",
			"new_size",
		};
		Program = LoadTFBProgram(file_manager->getAsset("shaders/particlesimheightmap.vert").c_str(), varyings, 4);

		uniform_dt = glGetUniformLocation(Program, "dt");
		uniform_sourcematrix = glGetUniformLocation(Program, "sourcematrix");
		uniform_level = glGetUniformLocation(Program, "level");
		uniform_size_increase_factor = glGetUniformLocation(Program, "size_increase_factor");

		attrib_position = glGetAttribLocation(Program, "particle_position");
		attrib_lifetime = glGetAttribLocation(Program, "lifetime");
		attrib_velocity = glGetAttribLocation(Program, "particle_velocity");
		attrib_size = glGetAttribLocation(Program, "size");
		attrib_initial_position = glGetAttribLocation(Program, "particle_position_initial");
		attrib_initial_lifetime = glGetAttribLocation(Program, "lifetime_initial");
		attrib_initial_velocity = glGetAttribLocation(Program, "particle_velocity_initial");
		attrib_initial_size = glGetAttribLocation(Program, "size_initial");

		uniform_heightmap = glGetUniformLocation(Program, "heightmap");
		uniform_track_x = glGetUniformLocation(Program, "track_x");
		uniform_track_x_len = glGetUniformLocation(Program, "track_x_len");
		uniform_track_z = glGetUniformLocation(Program, "track_z");
		uniform_track_z_len = glGetUniformLocation(Program, "track_z_len");
	}

	GLuint SimpleParticleRender::Program;
	GLuint SimpleParticleRender::attrib_pos;
	GLuint SimpleParticleRender::attrib_lf;
	GLuint SimpleParticleRender::attrib_quadcorner;
	GLuint SimpleParticleRender::attrib_texcoord;
	GLuint SimpleParticleRender::attrib_sz;
	GLuint SimpleParticleRender::uniform_matrix;
	GLuint SimpleParticleRender::uniform_viewmatrix;
	GLuint SimpleParticleRender::uniform_tex;
	GLuint SimpleParticleRender::uniform_dtex;
	GLuint SimpleParticleRender::uniform_screen;
	GLuint SimpleParticleRender::uniform_invproj;
	
	void SimpleParticleRender::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/particle.vert").c_str(), file_manager->getAsset("shaders/particle.frag").c_str());
		attrib_pos = glGetAttribLocation(Program, "position");
		attrib_sz = glGetAttribLocation(Program, "size");
		attrib_lf = glGetAttribLocation(Program, "lifetime");
		attrib_quadcorner = glGetAttribLocation(Program, "quadcorner");
		attrib_texcoord = glGetAttribLocation(Program, "texcoord");


		uniform_matrix = glGetUniformLocation(Program, "ProjectionMatrix");
		uniform_viewmatrix = glGetUniformLocation(Program, "ViewMatrix");
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_invproj = glGetUniformLocation(Program, "invproj");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_dtex = glGetUniformLocation(Program, "dtex");
	}

	void SimpleParticleRender::setUniforms(const core::matrix4 &ViewMatrix, const core::matrix4 &ProjMatrix, const core::matrix4 InvProjMatrix, float width, float height, unsigned TU_tex, unsigned TU_dtex)
	{
		glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProjMatrix.pointer());
		glUniform2f(uniform_screen, width, height);
		glUniformMatrix4fv(uniform_matrix, 1, GL_FALSE, irr_driver->getProjMatrix().pointer());
		glUniformMatrix4fv(uniform_viewmatrix, 1, GL_FALSE, irr_driver->getViewMatrix().pointer());
		glUniform1i(uniform_tex, TU_tex);
		glUniform1i(uniform_dtex, TU_dtex);
	}

	GLuint FlipParticleRender::Program;
	GLuint FlipParticleRender::attrib_pos;
	GLuint FlipParticleRender::attrib_lf;
	GLuint FlipParticleRender::attrib_quadcorner;
	GLuint FlipParticleRender::attrib_texcoord;
	GLuint FlipParticleRender::attrib_sz;
	GLuint FlipParticleRender::attrib_rotationvec;
	GLuint FlipParticleRender::attrib_anglespeed;
	GLuint FlipParticleRender::uniform_matrix;
	GLuint FlipParticleRender::uniform_viewmatrix;
	GLuint FlipParticleRender::uniform_tex;
	GLuint FlipParticleRender::uniform_dtex;
	GLuint FlipParticleRender::uniform_screen;
	GLuint FlipParticleRender::uniform_invproj;

	void FlipParticleRender::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/flipparticle.vert").c_str(), file_manager->getAsset("shaders/particle.frag").c_str());
		attrib_pos = glGetAttribLocation(Program, "position");
		attrib_sz = glGetAttribLocation(Program, "size");
		attrib_lf = glGetAttribLocation(Program, "lifetime");
		attrib_quadcorner = glGetAttribLocation(Program, "quadcorner");
		attrib_texcoord = glGetAttribLocation(Program, "texcoord");
		attrib_anglespeed = glGetAttribLocation(Program, "anglespeed");
		attrib_rotationvec = glGetAttribLocation(Program, "rotationvec");

		uniform_matrix = glGetUniformLocation(Program, "ProjectionMatrix");
		uniform_viewmatrix = glGetUniformLocation(Program, "ViewMatrix");
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_invproj = glGetUniformLocation(Program, "invproj");
		uniform_screen = glGetUniformLocation(Program, "screen");
		uniform_dtex = glGetUniformLocation(Program, "dtex");
	}

	void FlipParticleRender::setUniforms(const core::matrix4 &ViewMatrix, const core::matrix4 &ProjMatrix, const core::matrix4 InvProjMatrix, float width, float height, unsigned TU_tex, unsigned TU_dtex)
	{
		glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProjMatrix.pointer());
		glUniform2f(uniform_screen, width, height);
		glUniformMatrix4fv(uniform_matrix, 1, GL_FALSE, irr_driver->getProjMatrix().pointer());
		glUniformMatrix4fv(uniform_viewmatrix, 1, GL_FALSE, irr_driver->getViewMatrix().pointer());
		glUniform1i(uniform_tex, TU_tex);
		glUniform1i(uniform_dtex, TU_dtex);
	}
}

static GLuint createVAO(GLuint Program)
{
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	GLuint attrib_position = glGetAttribLocation(Program, "Position");
	GLuint attrib_texcoord = glGetAttribLocation(Program, "Texcoord");
	glBindBuffer(GL_ARRAY_BUFFER, quad_vbo);
	glEnableVertexAttribArray(attrib_position);
	glEnableVertexAttribArray(attrib_texcoord);
	glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
	glVertexAttribPointer(attrib_texcoord, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid*)(2 * sizeof(float)));
	glBindVertexArray(0);
	return vao;
}

namespace FullScreenShader
{
	GLuint BloomShader::Program;
	GLuint BloomShader::uniform_texture;
	GLuint BloomShader::uniform_low;
	GLuint BloomShader::vao;
	void BloomShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/bloom.frag").c_str());
		uniform_texture = glGetUniformLocation(Program, "tex");
		uniform_low = glGetUniformLocation(Program, "low");
		vao = createVAO(Program);
	}

	GLuint BloomBlendShader::Program;
	GLuint BloomBlendShader::uniform_texture;
	GLuint BloomBlendShader::uniform_low;
	GLuint BloomBlendShader::vao;
	void BloomBlendShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/bloomblend.frag").c_str());
		uniform_texture = glGetUniformLocation(Program, "tex");
		vao = createVAO(Program);
	}

	GLuint ColorLevelShader::Program;
	GLuint ColorLevelShader::uniform_tex;
	GLuint ColorLevelShader::uniform_inlevel;
	GLuint ColorLevelShader::uniform_outlevel;
	GLuint ColorLevelShader::vao;
    GLuint ColorLevelShader::uniform_invprojm;
    GLuint ColorLevelShader::uniform_dtex;
	void ColorLevelShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/color_levels.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_dtex = glGetUniformLocation(Program, "dtex");
		uniform_inlevel = glGetUniformLocation(Program, "inlevel");
		uniform_outlevel = glGetUniformLocation(Program, "outlevel");
        uniform_invprojm = glGetUniformLocation(Program, "invprojm");
		vao = createVAO(Program);
	}

	GLuint SunLightShader::Program;
	GLuint SunLightShader::uniform_ntex;
	GLuint SunLightShader::uniform_dtex;
	GLuint SunLightShader::uniform_direction;
	GLuint SunLightShader::uniform_col;
	GLuint SunLightShader::uniform_invproj;
	GLuint SunLightShader::vao;

	void SunLightShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/sunlight.frag").c_str());
		uniform_ntex = glGetUniformLocation(Program, "ntex");
		uniform_dtex = glGetUniformLocation(Program, "dtex");
		uniform_direction = glGetUniformLocation(Program, "direction");
		uniform_col = glGetUniformLocation(Program, "col");
		uniform_invproj = glGetUniformLocation(Program, "invproj");
		vao = createVAO(Program);
	}

	void SunLightShader::setUniforms(const core::vector3df &direction, const core::matrix4 &InvProjMatrix, float r, float g, float b, unsigned TU_ntex, unsigned TU_dtex)
	{
		glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProjMatrix.pointer());
		glUniform3f(uniform_direction, direction.X, direction.Y, direction.Z);
		glUniform3f(uniform_col, r, g, b);
		glUniform1i(uniform_ntex, TU_ntex);
		glUniform1i(uniform_dtex, TU_dtex);
	}

    GLuint DiffuseEnvMapShader::Program;
    GLuint DiffuseEnvMapShader::uniform_ntex;
    GLuint DiffuseEnvMapShader::uniform_blueLmn;
    GLuint DiffuseEnvMapShader::uniform_greenLmn;
    GLuint DiffuseEnvMapShader::uniform_redLmn;
    GLuint DiffuseEnvMapShader::vao;

    void DiffuseEnvMapShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/diffuseenvmap.frag").c_str());
        uniform_ntex = glGetUniformLocation(Program, "ntex");
        uniform_blueLmn = glGetUniformLocation(Program, "blueLmn[0]");
        uniform_greenLmn = glGetUniformLocation(Program, "greenLmn[0]");
        uniform_redLmn = glGetUniformLocation(Program, "redLmn[0]");
        vao = createVAO(Program);
    }

    void DiffuseEnvMapShader::setUniforms(const float *blueSHCoeff, const float *greenSHCoeff, const float *redSHCoeff, unsigned TU_ntex)
    {
        glUniform1i(uniform_ntex, TU_ntex);
        glUniform1fv(uniform_blueLmn, 9, blueSHCoeff);
        glUniform1fv(uniform_greenLmn, 9, greenSHCoeff);
        glUniform1fv(uniform_redLmn, 9, redSHCoeff);
    }

    GLuint ShadowedSunLightShader::Program;
    GLuint ShadowedSunLightShader::uniform_ntex;
    GLuint ShadowedSunLightShader::uniform_dtex;
    GLuint ShadowedSunLightShader::uniform_shadowtex;
    GLuint ShadowedSunLightShader::uniform_shadowmat;
    GLuint ShadowedSunLightShader::uniform_direction;
    GLuint ShadowedSunLightShader::uniform_col;
    GLuint ShadowedSunLightShader::uniform_invproj;
    GLuint ShadowedSunLightShader::vao;

    void ShadowedSunLightShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/sunlightshadow.frag").c_str());
        uniform_ntex = glGetUniformLocation(Program, "ntex");
        uniform_dtex = glGetUniformLocation(Program, "dtex");
        uniform_shadowtex = glGetUniformLocation(Program, "shadowtex");
        uniform_shadowmat = glGetUniformLocation(Program, "shadowmat[0]");
        uniform_direction = glGetUniformLocation(Program, "direction");
        uniform_col = glGetUniformLocation(Program, "col");
        uniform_invproj = glGetUniformLocation(Program, "invproj");
        vao = createVAO(Program);
    }

    void ShadowedSunLightShader::setUniforms(const std::vector<core::matrix4> &shadowmat, const core::vector3df &direction, const core::matrix4 &InvProjMatrix, float r, float g, float b, unsigned TU_ntex, unsigned TU_dtex, unsigned TU_shadowtex)
    {
        size_t size = shadowmat.size();
        float *tmp = new float[16 * size];
        for (unsigned i = 0; i < size; i++) {
            memcpy(&tmp[16 * i], shadowmat[i].pointer(), 16 * sizeof(float));
        }

        glUniformMatrix4fv(uniform_shadowmat, size, GL_FALSE, tmp);
        glUniformMatrix4fv(uniform_invproj, 1, GL_FALSE, InvProjMatrix.pointer());
        glUniform3f(uniform_direction, direction.X, direction.Y, direction.Z);
        glUniform3f(uniform_col, r, g, b);
        glUniform1i(uniform_ntex, TU_ntex);
        glUniform1i(uniform_dtex, TU_dtex);
        glUniform1i(uniform_shadowtex, TU_shadowtex);
        delete[] tmp;
    }

	GLuint Gaussian6HBlurShader::Program;
	GLuint Gaussian6HBlurShader::uniform_tex;
	GLuint Gaussian6HBlurShader::uniform_pixel;
	GLuint Gaussian6HBlurShader::vao;
	void Gaussian6HBlurShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/gaussian6h.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_pixel = glGetUniformLocation(Program, "pixel");
		vao = createVAO(Program);
	}

	GLuint Gaussian3HBlurShader::Program;
	GLuint Gaussian3HBlurShader::uniform_tex;
	GLuint Gaussian3HBlurShader::uniform_pixel;
	GLuint Gaussian3HBlurShader::vao;
	void Gaussian3HBlurShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/gaussian3h.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_pixel = glGetUniformLocation(Program, "pixel");
		vao = createVAO(Program);
	}

	GLuint Gaussian6VBlurShader::Program;
	GLuint Gaussian6VBlurShader::uniform_tex;
	GLuint Gaussian6VBlurShader::uniform_pixel;
	GLuint Gaussian6VBlurShader::vao;
	void Gaussian6VBlurShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/gaussian6v.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_pixel = glGetUniformLocation(Program, "pixel");
		vao = createVAO(Program);
	}

	GLuint Gaussian3VBlurShader::Program;
	GLuint Gaussian3VBlurShader::uniform_tex;
	GLuint Gaussian3VBlurShader::uniform_pixel;
	GLuint Gaussian3VBlurShader::vao;
	void Gaussian3VBlurShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/gaussian3v.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_pixel = glGetUniformLocation(Program, "pixel");
		vao = createVAO(Program);
	}

    GLuint PenumbraHShader::Program;
    GLuint PenumbraHShader::uniform_tex;
    GLuint PenumbraHShader::uniform_pixel;
    GLuint PenumbraHShader::vao;
    void PenumbraHShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/penumbrah.frag").c_str());
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_pixel = glGetUniformLocation(Program, "pixel");
        vao = createVAO(Program);
    }

    void PenumbraHShader::setUniforms(const core::vector2df &pixels, GLuint TU_tex)
    {
        glUniform2f(uniform_pixel, pixels.X, pixels.Y);
        glUniform1i(uniform_tex, TU_tex);
    }

    GLuint PenumbraVShader::Program;
    GLuint PenumbraVShader::uniform_tex;
    GLuint PenumbraVShader::uniform_pixel;
    GLuint PenumbraVShader::vao;
    void PenumbraVShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/penumbrav.frag").c_str());
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_pixel = glGetUniformLocation(Program, "pixel");
        vao = createVAO(Program);
    }

    void PenumbraVShader::setUniforms(const core::vector2df &pixels, GLuint TU_tex)
    {
        glUniform2f(uniform_pixel, pixels.X, pixels.Y);
        glUniform1i(uniform_tex, TU_tex);
    }

    GLuint ShadowGenShader::Program;
    GLuint ShadowGenShader::uniform_halft;
    GLuint ShadowGenShader::uniform_quarter;
    GLuint ShadowGenShader::uniform_height;
    GLuint ShadowGenShader::vao;
    void ShadowGenShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/shadowgen.frag").c_str());
        uniform_halft = glGetUniformLocation(Program, "halft");
        uniform_quarter = glGetUniformLocation(Program, "quarter");
        uniform_height = glGetUniformLocation(Program, "height");
        vao = createVAO(Program);
    }

    void ShadowGenShader::setUniforms(GLuint TU_halft, GLuint TU_quarter, GLuint TU_height)
    {
        glUniform1i(uniform_halft, TU_halft);
        glUniform1i(uniform_quarter, TU_quarter);
        glUniform1i(uniform_height, TU_height);
    }

	GLuint PassThroughShader::Program;
	GLuint PassThroughShader::uniform_texture;
	GLuint PassThroughShader::vao;
	void PassThroughShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/texturedquad.frag").c_str());
		uniform_texture = glGetUniformLocation(Program, "texture");
		vao = createVAO(Program);
	}

	GLuint GlowShader::Program;
	GLuint GlowShader::uniform_tex;
	GLuint GlowShader::vao;
	void GlowShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/glow.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		vao = createVAO(Program);
	}

	GLuint SSAOShader::Program;
	GLuint SSAOShader::uniform_ntex;
	GLuint SSAOShader::uniform_dtex;
	GLuint SSAOShader::uniform_noise_texture;
	GLuint SSAOShader::uniform_invprojm;
	GLuint SSAOShader::uniform_projm;
	GLuint SSAOShader::uniform_samplePoints;
	GLuint SSAOShader::vao;
	float SSAOShader::SSAOSamples[64];

	void SSAOShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/ssao.frag").c_str());
		uniform_ntex = glGetUniformLocation(Program, "ntex");
		uniform_dtex = glGetUniformLocation(Program, "dtex");
		uniform_noise_texture = glGetUniformLocation(Program, "noise_texture");
		uniform_invprojm = glGetUniformLocation(Program, "invprojm");
		uniform_projm = glGetUniformLocation(Program, "projm");
		uniform_samplePoints = glGetUniformLocation(Program, "samplePoints[0]");
		vao = createVAO(Program);

		// SSAOSamples[4 * i] and SSAOSamples[4 * i + 1] can be negative

		SSAOSamples[0] = 0.135061f;
		SSAOSamples[1] = 0.207948f;
		SSAOSamples[2] = 0.968770f;
		SSAOSamples[3] = 0.983032f;

		SSAOSamples[4] = 0.273456f;
		SSAOSamples[5] = -0.805390f;
		SSAOSamples[6] = 0.525898f;
		SSAOSamples[7] = 0.942808f;

		SSAOSamples[8] = 0.443450f;
		SSAOSamples[9] = -0.803786f;
		SSAOSamples[10] = 0.396585f;
		SSAOSamples[11] = 0.007996f;

		SSAOSamples[12] = 0.742420f;
		SSAOSamples[13] = -0.620072f;
		SSAOSamples[14] = 0.253621f;
		SSAOSamples[15] = 0.284829f;

		SSAOSamples[16] = 0.892464f;
		SSAOSamples[17] = 0.046221f;
		SSAOSamples[18] = 0.448744f;
		SSAOSamples[19] = 0.753655f;

		SSAOSamples[20] = 0.830350f;
		SSAOSamples[21] = -0.043593f;
		SSAOSamples[22] = 0.555535f;
		SSAOSamples[23] = 0.357463f;

		SSAOSamples[24] = -0.600612f;
		SSAOSamples[25] = -0.536421f;
		SSAOSamples[26] = 0.592889f;
		SSAOSamples[27] = 0.670583f;

		SSAOSamples[28] = -0.280658f;
		SSAOSamples[29] = 0.674894f;
		SSAOSamples[30] = 0.682458f;
		SSAOSamples[31] = 0.553362f;

		SSAOSamples[32] = -0.654493f;
		SSAOSamples[33] = -0.140866f;
		SSAOSamples[34] = 0.742830f;
		SSAOSamples[35] = 0.699820f;

		SSAOSamples[36] = 0.114730f;
		SSAOSamples[37] = 0.873130f;
		SSAOSamples[38] = 0.473794f;
		SSAOSamples[39] = 0.483901f;

		SSAOSamples[40] = 0.699167f;
		SSAOSamples[41] = 0.632210f;
		SSAOSamples[42] = 0.333879f;
		SSAOSamples[43] = 0.010956f;

		SSAOSamples[44] = 0.904603f;
		SSAOSamples[45] = 0.393410f;
		SSAOSamples[46] = 0.164080f;
		SSAOSamples[47] = 0.780297f;

		SSAOSamples[48] = 0.631662f;
		SSAOSamples[49] = -0.405195f;
		SSAOSamples[50] = 0.660924f;
		SSAOSamples[51] = 0.865596f;

		SSAOSamples[52] = -0.195668f;
		SSAOSamples[53] = 0.629185f;
		SSAOSamples[54] = 0.752223f;
		SSAOSamples[55] = 0.019013f;

		SSAOSamples[56] = -0.511316f;
		SSAOSamples[57] = 0.635504f;
		SSAOSamples[58] = 0.578524f;
		SSAOSamples[59] = 0.605457f;

		SSAOSamples[60] = -0.898843f;
		SSAOSamples[61] = 0.067382f;
		SSAOSamples[62] = 0.433061f;
		SSAOSamples[63] = 0.772942f;

		// Generate another random distribution, if needed
/*		for (unsigned i = 0; i < 16; i++) {
			// Use double to avoid denorm and get a true uniform distribution
			// Generate z component between [0.1; 1] to avoid being too close from surface
			double z = rand();
			z /= RAND_MAX;
			z = 0.1 + 0.9 * z;

			// Now generate x,y on the unit circle
			double x = rand();
			x /= RAND_MAX;
			x = 2 * x - 1;
			double y = rand();
			y /= RAND_MAX;
			y = 2 * y - 1;
			double xynorm = sqrt(x * x + y * y);
			x /= xynorm;
			y /= xynorm;
			// Now resize x,y so that norm(x,y,z) is one
			x *= sqrt(1. - z * z);
			y *= sqrt(1. - z * z);

			// Norm factor
			double w = rand();
			w /= RAND_MAX;
			SSAOSamples[4 * i] = (float)x;
			SSAOSamples[4 * i + 1] = (float)y;
			SSAOSamples[4 * i + 2] = (float)z;
			SSAOSamples[4 * i + 3] = (float)w;
		}*/
	}

	void SSAOShader::setUniforms(const core::matrix4& projm, const core::matrix4 &invprojm, unsigned TU_ntex, unsigned TU_dtex, unsigned TU_noise)
	{
		glUniformMatrix4fv(FullScreenShader::SSAOShader::uniform_invprojm, 1, GL_FALSE, invprojm.pointer());
		glUniformMatrix4fv(FullScreenShader::SSAOShader::uniform_projm, 1, GL_FALSE, projm.pointer());
		glUniform4fv(FullScreenShader::SSAOShader::uniform_samplePoints, 16, FullScreenShader::SSAOShader::SSAOSamples);

		glUniform1i(FullScreenShader::SSAOShader::uniform_ntex, TU_ntex);
		glUniform1i(FullScreenShader::SSAOShader::uniform_dtex, TU_dtex);
		glUniform1i(FullScreenShader::SSAOShader::uniform_noise_texture, TU_noise);
	}

	GLuint FogShader::Program;
	GLuint FogShader::uniform_tex;
	GLuint FogShader::uniform_fogmax;
	GLuint FogShader::uniform_startH;
	GLuint FogShader::uniform_endH;
	GLuint FogShader::uniform_start;
	GLuint FogShader::uniform_end;
	GLuint FogShader::uniform_col;
	GLuint FogShader::uniform_ipvmat;
	GLuint FogShader::vao;

	void FogShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/fog.frag").c_str());
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_fogmax = glGetUniformLocation(Program, "fogmax");
		uniform_startH = glGetUniformLocation(Program, "startH");
		uniform_endH = glGetUniformLocation(Program, "endH");
		uniform_start = glGetUniformLocation(Program, "start");
		uniform_end = glGetUniformLocation(Program, "end");
		uniform_col = glGetUniformLocation(Program, "col");
		uniform_ipvmat = glGetUniformLocation(Program, "ipvmat");
		vao = createVAO(Program);
	}

	void FogShader::setUniforms(const core::matrix4 &ipvmat, float fogmax, float startH, float endH, float start, float end, const core::vector3df &col, unsigned TU_ntex)
	{
		glUniform1f(uniform_fogmax, fogmax);
		glUniform1f(uniform_startH, startH);
		glUniform1f(uniform_endH, endH);
		glUniform1f(uniform_start, start);
		glUniform1f(uniform_end, end);
		glUniform3f(uniform_col, col.X, col.Y, col.Z);
		glUniformMatrix4fv(uniform_ipvmat, 1, GL_FALSE, ipvmat.pointer());
		glUniform1i(uniform_tex, TU_ntex);
	}

    GLuint MotionBlurShader::Program;
    GLuint MotionBlurShader::uniform_boost_amount;
    GLuint MotionBlurShader::uniform_center;
    GLuint MotionBlurShader::uniform_color_buffer;
    GLuint MotionBlurShader::uniform_direction;
    GLuint MotionBlurShader::uniform_mask_radius;
    GLuint MotionBlurShader::uniform_max_tex_height;
    GLuint MotionBlurShader::vao;

    void MotionBlurShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/motion_blur.frag").c_str());
        uniform_boost_amount = glGetUniformLocation(Program, "boost_amount");
        uniform_center = glGetUniformLocation(Program, "center");
        uniform_color_buffer = glGetUniformLocation(Program, "color_buffer");
        uniform_direction = glGetUniformLocation(Program, "direction");
        uniform_mask_radius = glGetUniformLocation(Program, "mask_radius");
        uniform_max_tex_height = glGetUniformLocation(Program, "max_tex_height");
        vao = createVAO(Program);
    }

    void MotionBlurShader::setUniforms(float boost_amount, const core::vector2df &center, const core::vector2df &direction, float mask_radius, float max_tex_height, unsigned TU_cb)
    {
        glUniform1f(uniform_boost_amount, boost_amount);
        glUniform2f(uniform_center, center.X, center.Y);
        glUniform2f(uniform_direction, direction.X, direction.Y);
        glUniform1f(uniform_mask_radius, mask_radius);
        glUniform1f(uniform_max_tex_height, max_tex_height);
        glUniform1i(uniform_color_buffer, TU_cb);
    }

    GLuint GodFadeShader::Program;
    GLuint GodFadeShader::uniform_tex;
    GLuint GodFadeShader::uniform_col;
    GLuint GodFadeShader::vao;

    void GodFadeShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/godfade.frag").c_str());
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_col = glGetUniformLocation(Program, "col");
        vao = createVAO(Program);
    }

    void GodFadeShader::setUniforms(const SColor &col, unsigned TU_tex)
    {
        glUniform3f(uniform_col, col.getRed() / 255., col.getGreen() / 255., col.getBlue() / 255.);
        glUniform1i(uniform_tex, TU_tex);
    }

    GLuint GodRayShader::Program;
    GLuint GodRayShader::uniform_tex;
    GLuint GodRayShader::uniform_sunpos;
    GLuint GodRayShader::vao;

    void GodRayShader::init()
    {
        Program = LoadProgram(file_manager->getAsset("shaders/screenquad.vert").c_str(), file_manager->getAsset("shaders/godray.frag").c_str());
        uniform_tex = glGetUniformLocation(Program, "tex");
        uniform_sunpos = glGetUniformLocation(Program, "sunpos");
        vao = createVAO(Program);
    }

    void GodRayShader::setUniforms(const core::vector2df &sunpos, unsigned TU_tex)
    {
        glUniform2f(uniform_sunpos, sunpos.X, sunpos.Y);
        glUniform1i(uniform_tex, TU_tex);
    }
}

namespace UIShader
{
	GLuint TextureRectShader::Program;
	GLuint TextureRectShader::attrib_position;
	GLuint TextureRectShader::attrib_texcoord;
	GLuint TextureRectShader::uniform_tex;
	GLuint TextureRectShader::uniform_center;
	GLuint TextureRectShader::uniform_size;
	GLuint TextureRectShader::uniform_texcenter;
	GLuint TextureRectShader::uniform_texsize;
	GLuint TextureRectShader::vao;

	void TextureRectShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/texturedquad.vert").c_str(), file_manager->getAsset("shaders/texturedquad.frag").c_str());

		attrib_position = glGetAttribLocation(Program, "position");
		attrib_texcoord = glGetAttribLocation(Program, "texcoord");
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_center = glGetUniformLocation(Program, "center");
		uniform_size = glGetUniformLocation(Program, "size");
		uniform_texcenter = glGetUniformLocation(Program, "texcenter");
		uniform_texsize = glGetUniformLocation(Program, "texsize");
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);
		glEnableVertexAttribArray(attrib_position);
		glEnableVertexAttribArray(attrib_texcoord);
		glBindBuffer(GL_ARRAY_BUFFER, quad_buffer);
		glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
		glVertexAttribPointer(attrib_texcoord, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid *)(2 * sizeof(float)));
		glBindVertexArray(0);
	}

	void TextureRectShader::setUniforms(float center_pos_x, float center_pos_y, float width, float height, float tex_center_pos_x, float tex_center_pos_y, float tex_width, float tex_height, unsigned TU_tex)
	{
		glUniform1i(uniform_tex, TU_tex);
		glUniform2f(uniform_center, center_pos_x, center_pos_y);
		glUniform2f(uniform_size, width, height);
		glUniform2f(uniform_texcenter, tex_center_pos_x, tex_center_pos_y);
		glUniform2f(uniform_texsize, tex_width, tex_height);
	}

	GLuint ColoredTextureRectShader::Program;
	GLuint ColoredTextureRectShader::attrib_position;
	GLuint ColoredTextureRectShader::attrib_texcoord;
	GLuint ColoredTextureRectShader::attrib_color;
	GLuint ColoredTextureRectShader::uniform_tex;
	GLuint ColoredTextureRectShader::uniform_center;
	GLuint ColoredTextureRectShader::uniform_size;
	GLuint ColoredTextureRectShader::uniform_texcenter;
	GLuint ColoredTextureRectShader::uniform_texsize;
	GLuint ColoredTextureRectShader::colorvbo;
	GLuint ColoredTextureRectShader::vao;

	void ColoredTextureRectShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/colortexturedquad.vert").c_str(), file_manager->getAsset("shaders/colortexturedquad.frag").c_str());

		attrib_position = glGetAttribLocation(Program, "position");
		attrib_texcoord = glGetAttribLocation(Program, "texcoord");
		attrib_color = glGetAttribLocation(Program, "color");
		uniform_tex = glGetUniformLocation(Program, "tex");
		uniform_center = glGetUniformLocation(Program, "center");
		uniform_size = glGetUniformLocation(Program, "size");
		uniform_texcenter = glGetUniformLocation(Program, "texcenter");
		uniform_texsize = glGetUniformLocation(Program, "texsize");
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);
		glEnableVertexAttribArray(attrib_position);
		glEnableVertexAttribArray(attrib_texcoord);
		glEnableVertexAttribArray(attrib_color);
		glBindBuffer(GL_ARRAY_BUFFER, quad_buffer);
		glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
		glVertexAttribPointer(attrib_texcoord, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid *)(2 * sizeof(float)));
		const unsigned quad_color[] = {
			0, 0, 0, 255,
			255, 0, 0, 255,
			0, 255, 0, 255,
			0, 0, 255, 255,
		};
		glGenBuffers(1, &colorvbo);
		glBindBuffer(GL_ARRAY_BUFFER, colorvbo);
		glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(unsigned), quad_color, GL_DYNAMIC_DRAW);
		glVertexAttribIPointer(attrib_color, 4, GL_UNSIGNED_INT, 4 * sizeof(unsigned), 0);
		glBindVertexArray(0);
	}

	void ColoredTextureRectShader::setUniforms(float center_pos_x, float center_pos_y, float width, float height, float tex_center_pos_x, float tex_center_pos_y, float tex_width, float tex_height, unsigned TU_tex)
	{
		glUniform1i(uniform_tex, TU_tex);
		glUniform2f(uniform_center, center_pos_x, center_pos_y);
		glUniform2f(uniform_size, width, height);
		glUniform2f(uniform_texcenter, tex_center_pos_x, tex_center_pos_y);
		glUniform2f(uniform_texsize, tex_width, tex_height);
	}

	GLuint ColoredRectShader::Program;
	GLuint ColoredRectShader::attrib_position;
	GLuint ColoredRectShader::uniform_center;
	GLuint ColoredRectShader::uniform_size;
	GLuint ColoredRectShader::uniform_color;
	GLuint ColoredRectShader::vao;

	void ColoredRectShader::init()
	{
		Program = LoadProgram(file_manager->getAsset("shaders/coloredquad.vert").c_str(), file_manager->getAsset("shaders/coloredquad.frag").c_str());
		attrib_position = glGetAttribLocation(Program, "position");
		uniform_color = glGetUniformLocation(Program, "color");
		uniform_center = glGetUniformLocation(Program, "center");
		uniform_size = glGetUniformLocation(Program, "size");
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);
		glEnableVertexAttribArray(attrib_position);
		glBindBuffer(GL_ARRAY_BUFFER, quad_buffer);
		glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
		glBindVertexArray(0);
	}

	void ColoredRectShader::setUniforms(float center_pos_x, float center_pos_y, float width, float height, const video::SColor &color)
	{
		glUniform2f(uniform_center, center_pos_x, center_pos_y);
		glUniform2f(uniform_size, width, height);
		glUniform4i(uniform_color, color.getRed(), color.getGreen(), color.getBlue(), color.getAlpha());
	}
}
