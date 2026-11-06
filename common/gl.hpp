// Shared OpenGL ES 3 / WebGL2 helpers for the GPU demos (folders with a `requires-gl` marker). Header only, web build only.
// Same 960x540 logical canvas, same look and the same Hud API as hud.hpp, but drawn with GL instead of SDL_Renderer.
//
//   gd::gl::Window win;  win.create("title");              // ES 3.0 context, 960x540, vsync
//   gd::gl::Hud hud;     hud.load();                       // JetBrains Mono atlases as GL textures
//   GLuint prog = gd::gl::compile(fragmentSrc);            // sources WITHOUT #version, compile() prepends it
//   ...every frame:
//   win.begin();                                           // viewport (16:9 letterbox), clear to paper
//   glUseProgram(prog); gd::gl::drawFullscreen();          // fragment shader gets `in vec2 vUv` (0..1, y down)
//   hud.titleBar(960, "[ TITLE ]", "right"); hud.text(x, y, "hi", gd::kInk);   // pixels of the 960x540 canvas
//   win.swap();
//   mouse: win.toLogical(event) gives canvas pixels (960x540), whatever size the page scales the canvas to.
#pragma once

#include <SDL3/SDL.h>

#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "hud.hpp"  // Color, kTeal ... and the font atlas data

namespace gd::gl {

using gd::Color;

inline constexpr int kWidth = 960;
inline constexpr int kHeight = 540;

// The one place that picks the GLSL dialect. Change it here to move all demos to desktop GL.
inline constexpr const char* kGlslHeader = "#version 300 es\nprecision highp float;\nprecision highp int;\n";

/// Fullscreen triangle without any vertex buffer; hands `vUv` to the fragment shader (0..1, origin top left).
inline constexpr const char* kFullscreenVs = R"(
out vec2 vUv;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    vUv = vec2(p.x, 1.0 - p.y);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

inline GLuint compileStage(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    const char* parts[2] = {kGlslHeader, source};
    glShaderSource(shader, 2, parts, nullptr);  // note: error line numbers count the 3 header lines
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == 0) {
        GLint length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        glGetShaderInfoLog(shader, length, nullptr, log.data());
        SDL_Log("%s shader failed to compile:\n%s", type == GL_VERTEX_SHADER ? "vertex" : "fragment", log.c_str());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

/// Compiles and links a program. Returns 0 (after logging the info log) on any error.
inline GLuint compile(const char* vertexSource, const char* fragmentSource) {
    const GLuint vs = compileStage(GL_VERTEX_SHADER, vertexSource);
    const GLuint fs = compileStage(GL_FRAGMENT_SHADER, fragmentSource);
    if (vs == 0 || fs == 0) {
        glDeleteShader(vs);
        glDeleteShader(fs);
        return 0;
    }
    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == 0) {
        char log[1024] = {};
        glGetProgramInfoLog(program, sizeof log - 1, nullptr, log);
        SDL_Log("program failed to link:\n%s", log);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

/// Fragment shader only, with the fullscreen vertex shader.
inline GLuint compile(const char* fragmentSource) { return compile(kFullscreenVs, fragmentSource); }

inline GLint uniform(GLuint program, const char* name) { return glGetUniformLocation(program, name); }

/// Draws one triangle that covers the whole viewport. Bind your program and textures first.
inline void drawFullscreen() {
    static GLuint vao = 0;
    if (vao == 0) glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

/// A 2D texture with its upload parameters. R32F with GL_NEAREST needs no extension in WebGL2 (filtering floats does).
struct Texture {
    GLuint id = 0;
    int width = 0;
    int height = 0;
    GLenum format = GL_RGBA;
    GLenum type = GL_UNSIGNED_BYTE;

    static Texture create(int w, int h, GLenum internalFormat, GLenum format, GLenum type, GLenum filter = GL_NEAREST, const void* pixels = nullptr) {
        Texture t;
        t.width = w;
        t.height = h;
        t.format = format;
        t.type = type;
        glGenTextures(1, &t.id);
        glBindTexture(GL_TEXTURE_2D, t.id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(filter));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, static_cast<GLint>(filter));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat), w, h, 0, format, type, pixels);
        return t;
    }

    void bind(int unit = 0) const {
        glActiveTexture(GL_TEXTURE0 + static_cast<GLenum>(unit));
        glBindTexture(GL_TEXTURE_2D, id);
    }

    /// Replaces the whole texture (pixels laid out like in create).
    void update(const void* pixels) const { update(0, 0, width, height, pixels); }

    void update(int x, int y, int w, int h, const void* pixels) const {
        glBindTexture(GL_TEXTURE_2D, id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, format, type, pixels);
    }

    void destroy() {
        glDeleteTextures(1, &id);
        id = 0;
    }
};

/// Window with an OpenGL ES 3.0 context and the 960x540 logical canvas, letterboxed to any drawing buffer size.
struct Window {
    SDL_Window* window = nullptr;
    SDL_GLContext context = nullptr;
    int vx = 0, vy = 0, vw = kWidth, vh = kHeight;  // viewport in drawing buffer pixels
    float scale = 1.0F;                              // buffer pixels per logical pixel

    bool create(const char* title) {
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
        window = SDL_CreateWindow(title, kWidth, kHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
        if (window == nullptr) {
            SDL_Log("window failed: %s", SDL_GetError());
            return false;
        }
        context = SDL_GL_CreateContext(window);
        if (context == nullptr) {
            SDL_Log("OpenGL ES 3.0 context failed: %s", SDL_GetError());
            return false;
        }
        SDL_GL_SetSwapInterval(1);
        return true;
    }

    /// Start of every frame: fit the 16:9 canvas into the drawing buffer, paint the bars (and everything) paper colored.
    void begin() {
        int pw = kWidth, ph = kHeight;
        SDL_GetWindowSizeInPixels(window, &pw, &ph);
        pw = std::max(pw, 1);
        ph = std::max(ph, 1);
        scale = std::min(static_cast<float>(pw) / kWidth, static_cast<float>(ph) / kHeight);
        vw = static_cast<int>(std::lround(kWidth * scale));
        vh = static_cast<int>(std::lround(kHeight * scale));
        vx = (pw - vw) / 2;
        vy = (ph - vh) / 2;
        glViewport(0, 0, pw, ph);
        glClearColor(kPaper.r / 255.0F, kPaper.g / 255.0F, kPaper.b / 255.0F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(vx, vy, vw, vh);
    }

    void swap() const { SDL_GL_SwapWindow(window); }

    /// Window coordinates (what SDL events carry) to the 960x540 canvas.
    SDL_FPoint toLogical(float x, float y) const {
        int ww = kWidth, wh = kHeight, pw = kWidth, ph = kHeight;
        SDL_GetWindowSize(window, &ww, &wh);
        SDL_GetWindowSizeInPixels(window, &pw, &ph);
        const float px = x * static_cast<float>(pw) / static_cast<float>(std::max(ww, 1));
        const float py = y * static_cast<float>(ph) / static_cast<float>(std::max(wh, 1));
        return SDL_FPoint{(px - static_cast<float>(vx)) / scale, (py - static_cast<float>(vy)) / scale};
    }

    /// Position of a mouse motion or button event on the canvas (other events give 0, 0).
    SDL_FPoint toLogical(const SDL_Event& e) const {
        if (e.type == SDL_EVENT_MOUSE_MOTION) return toLogical(e.motion.x, e.motion.y);
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN || e.type == SDL_EVENT_MOUSE_BUTTON_UP) return toLogical(e.button.x, e.button.y);
        return SDL_FPoint{0.0F, 0.0F};
    }

    void destroy() {
        if (context != nullptr) SDL_GL_DestroyContext(context);
        if (window != nullptr) SDL_DestroyWindow(window);
        context = nullptr;
        window = nullptr;
    }
};

/// Same calls as gd::Hud, but without a renderer argument. Coordinates are pixels of the 960x540 canvas, y down.
class Hud {
public:
    enum class Style { Regular, Bold, Large };

    bool load() {
        program_ = compile(R"(
            layout(location = 0) in vec2 aPos;
            layout(location = 1) in vec2 aUv;
            uniform vec2 uScreen;
            out vec2 vUv;
            void main() {
                vUv = aUv;
                gl_Position = vec4(aPos.x / uScreen.x * 2.0 - 1.0, 1.0 - aPos.y / uScreen.y * 2.0, 0.0, 1.0);
            })",
                           R"(
            uniform sampler2D uTex;
            uniform vec4 uColor;
            in vec2 vUv;
            out vec4 outColor;
            void main() { outColor = texture(uTex, vUv) * uColor; })");
        if (program_ == 0) return false;
        glUseProgram(program_);
        glUniform2f(uniform(program_, "uScreen"), kWidth, kHeight);
        glUniform1i(uniform(program_, "uTex"), 0);
        color_ = uniform(program_, "uColor");

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, x)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));

        const unsigned char white[4] = {255, 255, 255, 255};
        white_ = Texture::create(1, 1, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, GL_NEAREST, white);
        regular_ = make(font::kRegular16Png, sizeof font::kRegular16Png, font::kRegular16CellW, font::kRegular16CellH);
        bold_ = make(font::kBold16Png, sizeof font::kBold16Png, font::kBold16CellW, font::kBold16CellH);
        large_ = make(font::kBold32Png, sizeof font::kBold32Png, font::kBold32CellW, font::kBold32CellH);
        return regular_.texture.id != 0 && bold_.texture.id != 0 && large_.texture.id != 0;
    }

    void destroy() {
        for (Face* face : {&regular_, &bold_, &large_}) face->texture.destroy();
        white_.destroy();
        glDeleteBuffers(1, &vbo_);
        glDeleteVertexArrays(1, &vao_);
        glDeleteProgram(program_);
    }

    float width(std::string_view s, Style style = Style::Regular) const {
        float n = 0;
        for (std::size_t i = 0; i < s.size(); i += length(s[i])) ++n;
        return n * static_cast<float>(pick(style).cellW);
    }

    float lineHeight(Style style = Style::Regular) const { return static_cast<float>(pick(style).cellH); }

    void text(float x, float y, std::string_view s, Color c, Style style = Style::Regular) {
        const Face& face = pick(style);
        std::vector<Vertex> v;
        const auto cw = static_cast<float>(face.cellW);
        const auto ch = static_cast<float>(face.cellH);
        for (std::size_t i = 0; i < s.size(); i += length(s[i])) {
            const int index = glyphIndex(s, i);
            if (index >= 0) {
                const float u0 = static_cast<float>((index % font::kColumns) * face.cellW) / static_cast<float>(face.texture.width);
                const float v0 = static_cast<float>((index / font::kColumns) * face.cellH) / static_cast<float>(face.texture.height);
                quad(v, std::round(x), std::round(y), cw, ch, u0, v0, u0 + cw / static_cast<float>(face.texture.width), v0 + ch / static_cast<float>(face.texture.height));
            }
            x += cw;
        }
        draw(v, face.texture, c);
    }

    void fillRect(float x, float y, float w, float h, Color c) {
        std::vector<Vertex> v;
        quad(v, x, y, w, h, 0.5F, 0.5F, 0.5F, 0.5F);
        draw(v, white_, c);
    }

    void strokeRect(float x, float y, float w, float h, Color c) {
        fillRect(x, y, w, 1.0F, c);
        fillRect(x, y + h - 1.0F, w, 1.0F, c);
        fillRect(x, y + 1.0F, 1.0F, h - 2.0F, c);
        fillRect(x + w - 1.0F, y + 1.0F, 1.0F, h - 2.0F, c);
    }

    /// Rectangle with a hard offset shadow, the 90s button look malura.de uses.
    void card(float x, float y, float w, float h, Color fill, Color edge) {
        fillRect(x + 4.0F, y + 4.0F, w, h, Color{17, 24, 39, 40});
        fillRect(x, y, w, h, fill);
        strokeRect(x, y, w, h, edge);
    }

    /// Teal strip with a bold title in brackets, like the section headings on malura.de.
    void titleBar(int w, std::string_view title, std::string_view right = {}) {
        fillRect(0, 0, static_cast<float>(w), 30.0F, kTeal);
        text(12.0F, 7.0F, title, kWhite, Style::Bold);
        if (!right.empty()) text(static_cast<float>(w) - width(right) - 12.0F, 7.0F, right, kTealSoft);
    }

private:
    struct Vertex {
        float x, y, u, v;
    };
    struct Face {
        Texture texture;
        int cellW = 0;
        int cellH = 0;
    };

    GLuint program_ = 0, vao_ = 0, vbo_ = 0;
    GLint color_ = -1;
    Texture white_;
    Face regular_, bold_, large_;

    const Face& pick(Style style) const { return style == Style::Regular ? regular_ : style == Style::Bold ? bold_ : large_; }

    static void quad(std::vector<Vertex>& v, float x, float y, float w, float h, float u0, float v0, float u1, float v1) {
        v.insert(v.end(), {{x, y, u0, v0}, {x + w, y, u1, v0}, {x + w, y + h, u1, v1}, {x, y, u0, v0}, {x + w, y + h, u1, v1}, {x, y + h, u0, v1}});
    }

    void draw(const std::vector<Vertex>& v, const Texture& tex, Color c) {
        if (v.empty()) return;
        glUseProgram(program_);
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(Vertex)), v.data(), GL_STREAM_DRAW);
        tex.bind(0);
        glUniform4f(color_, c.r / 255.0F, c.g / 255.0F, c.b / 255.0F, c.a / 255.0F);
        glEnable(GL_BLEND);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(v.size()));
        glDisable(GL_BLEND);
    }

    static Face make(const unsigned char* png, std::size_t size, int cellW, int cellH) {
        SDL_IOStream* io = SDL_IOFromConstMem(png, size);
        SDL_Surface* loaded = io != nullptr ? SDL_LoadPNG_IO(io, true) : nullptr;
        if (loaded == nullptr) return {};
        SDL_Surface* rgba = SDL_ConvertSurface(loaded, SDL_PIXELFORMAT_RGBA32);
        SDL_DestroySurface(loaded);
        if (rgba == nullptr) return {};
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        Texture t = Texture::create(rgba->w, rgba->h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, GL_NEAREST, rgba->pixels);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        SDL_DestroySurface(rgba);
        return Face{t, cellW, cellH};
    }

    // UTF-8 decoding and glyph lookup, the same as in gd::Hud (its helpers are private there).
    static std::size_t length(char first) {
        const auto b = static_cast<unsigned char>(first);
        return b < 0x80 ? 1 : b < 0xE0 ? 2 : b < 0xF0 ? 3 : 4;
    }

    static char32_t decode(std::string_view s, std::size_t i) {
        const auto b0 = static_cast<unsigned char>(s[i]);
        const std::size_t n = length(s[i]);
        if (n == 1 || i + n > s.size()) return b0;
        char32_t cp = b0 & (0xFF >> (n + 1));
        for (std::size_t k = 1; k < n; ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
        return cp;
    }

    static int glyphIndex(std::string_view s, std::size_t i) {
        static const std::vector<char32_t> charset = [] {
            std::vector<char32_t> out;
            const std::string_view all = font::kCharset;
            for (std::size_t k = 0; k < all.size(); k += length(all[k])) out.push_back(decode(all, k));
            return out;
        }();
        const auto it = std::find(charset.begin(), charset.end(), decode(s, i));
        return it == charset.end() ? -1 : static_cast<int>(it - charset.begin());
    }
};

}  // namespace gd::gl
