// Ray marching: sphere tracing through signed distance fields, all in one fragment shader.
//
// There is no geometry. Every pixel shoots a ray from the camera and asks the scene "how far is the nearest surface?",
// jumps exactly that far, and asks again, until the answer is tiny (hit), the ray is far away (miss) or the step budget is
// spent. Normals, soft shadows and ambient occlusion fall out of the same distance function with a few more lookups.
// logic.hpp holds the CPU mirror of that math, which is what test.cpp checks; the shader below repeats it in GLSL
// (shapes(), map(), the march loop with the same eps, softShadow(), ambientOcclusion(), heat()), keep them in sync.
//
// The scene is rendered into a texture whose size the "res" chip sets (1x = 960x540, 0.5x = 480x270) and then scaled onto
// the canvas, so a weak GPU can trade sharpness for speed. Drag orbits the camera, the wheel zooms, the chips change the
// view (shaded or step heatmap), shadows, smooth union strength and the step limit.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gl.hpp"
#include "logic.hpp"

namespace {

constexpr double kMaxFrameTime = 0.25;
constexpr std::array<float, 3> kBlend{0.0F, 0.3F, 0.8F};
constexpr std::array<int, 4> kStepLimits{16, 32, 64, 128};
constexpr std::array<float, 2> kResScale{1.0F, 0.5F};
constexpr float kPanelTop = 436.0F;

// Fragment shader. `vUv` is 0..1 with the origin at the top left, like the canvas.
constexpr const char* kMarchFs = R"GLSL(
uniform float uTime;
uniform float uK;          // smooth union strength
uniform int uMaxSteps;
uniform int uHeat;         // 1: show the step count instead of the shaded image
uniform int uShadows;
uniform vec3 uCam;
uniform vec3 uTarget;
in vec2 vUv;
out vec4 outColor;

const float TMAX = 30.0;
const float HIT_EPS = 0.001;
const float HIT_EPS_SLOPE = 0.0004;   // the threshold grows with the distance, like the size of a pixel
const float HEAT_STEPS = 48.0;

float sdSphere(vec3 p, float r) { return length(p) - r; }
float sdBox(vec3 p, vec3 b) {
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}
float sdRoundBox(vec3 p, vec3 b, float r) { return sdBox(p, b) - r; }
float sdTorus(vec3 p, float R, float r) { return length(vec2(length(p.xz) - R, p.y)) - r; }
float smin(float a, float b, float k) {
    if (k <= 0.0) return min(a, b);
    float h = max(k - abs(a - b), 0.0) / k;
    return min(a, b) - h * h * k * 0.25;
}
vec3 rotY(vec3 p, float a) { float c = cos(a), s = sin(a); return vec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z); }
vec3 rotX(vec3 p, float a) { float c = cos(a), s = sin(a); return vec3(p.x, c * p.y - s * p.z, s * p.y + c * p.z); }

// Distances to the three moving shapes (mirror of shapes() in logic.hpp).
void shapes(vec3 p, out float ds, out float db, out float dt) {
    float t = uTime;
    vec3 sphereAt = vec3(-0.9 + 0.9 * sin(0.8 * t), 0.8 + 0.35 * sin(1.3 * t + 1.0), 0.2 * sin(0.6 * t));
    vec3 boxAt = vec3(0.5 + 0.9 * cos(0.7 * t), 0.75 + 0.3 * sin(1.1 * t), 0.6 * cos(0.5 * t));
    vec3 torusAt = vec3(0.2 * sin(0.5 * t), 1.2 + 0.25 * sin(0.9 * t + 2.0), -0.3 + 0.5 * sin(0.4 * t));
    ds = sdSphere(p - sphereAt, 0.65);
    db = sdRoundBox(rotY(p - boxAt, 0.6 * t), vec3(0.4, 0.3, 0.4), 0.1);
    dt = sdTorus(rotX(p - torusAt, 0.6 * t), 0.75, 0.2);
}

// The whole scene: floor plus the smooth union of the shapes.
float map(vec3 p) {
    float ds, db, dt;
    shapes(p, ds, db, dt);
    return min(p.y, smin(smin(ds, db, uK), dt, uK));
}

// Returns the distance along the ray, or -1.0 for no hit. `steps` counts the map() calls, `exhausted` is set when the
// step budget ran out before the ray hit or left.
float march(vec3 ro, vec3 rd, out int steps, out bool exhausted) {
    float t = 0.0;
    steps = 0;
    exhausted = false;
    for (int i = 0; i < uMaxSteps; i++) {
        float d = map(ro + rd * t);
        steps = i + 1;
        if (d < HIT_EPS + HIT_EPS_SLOPE * t) return t;
        t += d;
        if (t > TMAX) return -1.0;
    }
    exhausted = true;
    return -1.0;
}

// Gradient of the field with four taps on the corners of a tetrahedron.
vec3 normalAt(vec3 p) {
    const vec2 e = vec2(1.0, -1.0) * 0.5773 * 0.001;
    return normalize(e.xyy * map(p + e.xyy) + e.yyx * map(p + e.yyx) + e.yxy * map(p + e.yxy) + e.xxx * map(p + e.xxx));
}

// Soft shadow: the smallest ratio of "distance to the field" to "distance travelled" on the way to the light.
float softShadow(vec3 ro, vec3 rd) {
    float res = 1.0;
    float t = 0.02;
    for (int i = 0; i < 40; i++) {
        float d = map(ro + rd * t);
        res = min(res, 8.0 * d / t);
        t += clamp(d, 0.02, 0.5);
        if (res < 0.001 || t > 12.0) break;
    }
    res = clamp(res, 0.0, 1.0);
    return res * res * (3.0 - 2.0 * res);
}

// Ambient occlusion: five taps along the normal; in the open d == h, anything crowding in makes d smaller.
float ambientOcclusion(vec3 p, vec3 n) {
    float occ = 0.0;
    float weight = 1.0;
    for (int i = 0; i < 5; i++) {
        float h = 0.01 + 0.12 * float(i) / 4.0;
        occ += (h - map(p + n * h)) * weight;
        weight *= 0.95;
    }
    return clamp(1.0 - 3.0 * occ, 0.0, 1.0);
}

vec3 heat(float steps) {
    const vec3 paper = vec3(0.98, 0.98, 0.97);
    const vec3 teal = vec3(0.051, 0.580, 0.533);
    const vec3 orange = vec3(0.918, 0.345, 0.047);
    float x = clamp(steps / HEAT_STEPS, 0.0, 1.0) * 2.0;
    return x < 1.0 ? mix(paper, teal, x) : mix(teal, orange, x - 1.0);
}

vec3 skyColor(vec3 rd) {
    return mix(vec3(0.90, 0.96, 0.95), vec3(0.72, 0.92, 0.89), clamp(rd.y * 1.6, 0.0, 1.0));
}

vec3 shade(vec3 p, vec3 rd, float t) {
    float ds, db, dt;
    shapes(p, ds, db, dt);
    float blob = smin(smin(ds, db, uK), dt, uK);
    vec3 n = normalAt(p);

    vec3 albedo;
    float shiny = 0.0;
    if (p.y < blob) {
        vec2 cell = floor(p.xz);
        albedo = mix(vec3(0.97, 0.97, 0.95), vec3(0.66, 0.90, 0.86), mod(cell.x + cell.y, 2.0));
        albedo = mix(albedo, vec3(0.88, 0.96, 0.94), smoothstep(8.0, 22.0, t));   // tiles melt together far away
    } else {
        // Each shape pulls the color with a weight that falls off with its distance, so the colors blend where the shapes do.
        vec3 w = exp(-vec3(ds, db, dt) * 9.0);
        w /= w.x + w.y + w.z;
        albedo = w.x * vec3(0.051, 0.580, 0.533) + w.y * vec3(0.918, 0.345, 0.047) + w.z * vec3(0.94, 0.93, 0.90);
        shiny = 1.0;
    }

    vec3 light = normalize(vec3(0.5, 0.8, 0.4));
    float diffuse = clamp(dot(n, light), 0.0, 1.0);
    float shadow = 1.0;
    if (uShadows == 1 && diffuse > 0.0) shadow = softShadow(p + n * 0.02, light);
    float ao = ambientOcclusion(p, n);
    float sky = 0.65 + 0.35 * n.y;

    vec3 color = albedo * (vec3(1.0, 0.97, 0.90) * diffuse * shadow * 0.85 + vec3(0.62, 0.78, 0.80) * sky * ao * 0.7);
    vec3 half_ = normalize(light - rd);
    color += vec3(1.0, 0.98, 0.92) * pow(clamp(dot(n, half_), 0.0, 1.0), 48.0) * diffuse * shadow * 0.35 * shiny;
    return color;
}

void main() {
    vec3 fw = normalize(uTarget - uCam);
    vec3 right = normalize(cross(fw, vec3(0.0, 1.0, 0.0)));
    vec3 up = cross(right, fw);
    vec2 ndc = vec2(vUv.x * 2.0 - 1.0, 1.0 - vUv.y * 2.0) * vec2(16.0 / 9.0, 1.0);
    vec3 rd = normalize(ndc.x * right + ndc.y * up + 1.8 * fw);

    int steps;
    bool exhausted;
    float t = march(uCam, rd, steps, exhausted);

    vec3 color;
    if (uHeat == 1) {
        color = exhausted ? vec3(0.067, 0.094, 0.153) : heat(float(steps));
    } else if (t < 0.0) {
        color = skyColor(rd);
    } else {
        color = shade(uCam + rd * t, rd, t);
        float fog = 1.0 - exp(-0.0025 * t * t);
        color = mix(color, skyColor(vec3(rd.x, 0.0, rd.z)), fog);
    }
    outColor = vec4(color, 1.0);
}
)GLSL";

// Copies the render target to the canvas. The target is rendered with the same top-down vUv, so flip back for sampling.
constexpr const char* kBlitFs = R"GLSL(
uniform sampler2D uTex;
in vec2 vUv;
out vec4 outColor;
void main() { outColor = vec4(texture(uTex, vec2(vUv.x, 1.0 - vUv.y)).rgb, 1.0); }
)GLSL";

enum class Group { View, Shadows, Res, Blend, Steps };

struct Chip {
    SDL_FRect rect;
    std::string label;
    Group group;
    int value;
};

struct Label {
    float x, y;
    const char* text;
};

struct Uniforms {
    GLint time, k, maxSteps, heat, shadows, cam, target;
};

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;
    GLuint march = 0;
    GLuint blit = 0;
    Uniforms u{};
    GLuint fbo = 0;
    gd::gl::Texture target;
    int targetW = 0;
    int targetH = 0;

    std::vector<Chip> chips;
    std::vector<Label> labels;

    bool heat = false;
    bool shadows = true;
    int res = 0;       // index into kResScale
    int blend = 1;     // index into kBlend
    int steps = 2;     // index into kStepLimits
    bool paused = false;

    float yaw = 0.55F;
    float pitch = 0.32F;
    float dist = 4.3F;
    bool dragging = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};

    Uint64 last = 0;
    double time = 7.2;  // animation time in seconds, accumulated from the frame times
    long maxFrames = -1;
    long frames = 0;
};

float chipWidth(const App& app, const std::string& label) { return app.hud.width(label, gd::gl::Hud::Style::Bold) + 20.0F; }

// Places a row label and its chips from x to the right; returns the x after the group.
float addGroup(App& app, float x, float y, const char* label, Group group, std::vector<std::string> labels) {
    app.labels.push_back(Label{x, y + 3.0F, label});
    x += app.hud.width(label) + 10.0F;
    int value = 0;
    for (const std::string& l : labels) {
        const float w = chipWidth(app, l);
        app.chips.push_back(Chip{SDL_FRect{x, y, w, 28.0F}, l, group, value++});
        x += w + 8.0F;
    }
    return x + 20.0F;
}

void buildChips(App& app) {
    constexpr float row1 = 448.0F;
    constexpr float row2 = 486.0F;
    float x = 24.0F;
    x = addGroup(app, x, row1, "view:", Group::View, {"shaded", "steps"});
    x = addGroup(app, x, row1, "shadows:", Group::Shadows, {"off", "on"});
    addGroup(app, x, row1, "res:", Group::Res, {"1x", "0.5x"});
    x = addGroup(app, 24.0F, row2, "blend k:", Group::Blend, {"0.0", "0.3", "0.8"});
    addGroup(app, x, row2, "max steps:", Group::Steps, {"16", "32", "64", "128"});
}

bool isActive(const App& app, const Chip& c) {
    switch (c.group) {
        case Group::View: return c.value == (app.heat ? 1 : 0);
        case Group::Shadows: return c.value == (app.shadows ? 1 : 0);
        case Group::Res: return c.value == app.res;
        case Group::Blend: return c.value == app.blend;
        case Group::Steps: return c.value == app.steps;
    }
    return false;
}

bool createTarget(App& app) {
    const float s = kResScale[static_cast<std::size_t>(app.res)];
    const int w = static_cast<int>(std::lround(gd::gl::kWidth * s));
    const int h = static_cast<int>(std::lround(gd::gl::kHeight * s));
    if (app.fbo == 0) glGenFramebuffers(1, &app.fbo);
    if (app.target.id != 0) app.target.destroy();
    app.target = gd::gl::Texture::create(w, h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, GL_LINEAR);
    app.targetW = w;
    app.targetH = h;
    glBindFramebuffer(GL_FRAMEBUFFER, app.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, app.target.id, 0);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        SDL_Log("render target incomplete: 0x%x", status);
        return false;
    }
    return true;
}

void click(App& app, SDL_FPoint p) {
    for (const Chip& c : app.chips) {
        if (!SDL_PointInRectFloat(&p, &c.rect)) continue;
        switch (c.group) {
            case Group::View: app.heat = c.value == 1; break;
            case Group::Shadows: app.shadows = c.value == 1; break;
            case Group::Res:
                if (app.res != c.value) {
                    app.res = c.value;
                    createTarget(app);
                }
                break;
            case Group::Blend: app.blend = c.value; break;
            case Group::Steps: app.steps = c.value; break;
        }
        return;
    }
    if (p.y < kPanelTop) {
        app.dragging = true;
        app.lastMouse = p;
    }
}

void drawChips(App& app) {
    gd::gl::Hud& hud = app.hud;
    hud.fillRect(0.0F, kPanelTop, 960.0F, 540.0F - kPanelTop, gd::Color{250, 250, 249, 240});
    hud.fillRect(0.0F, kPanelTop, 960.0F, 2.0F, gd::kTealDark);
    for (const Label& l : app.labels) hud.text(l.x, l.y, l.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool active = isActive(app, c);
        hud.card(c.rect.x, c.rect.y, c.rect.w, c.rect.h, active ? gd::kTeal : gd::kWhite, gd::kInk);
        hud.text(c.rect.x + 10.0F, c.rect.y + 3.0F, c.label, active ? gd::kWhite : gd::kInk, gd::gl::Hud::Style::Bold);
    }
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->win.create("ray-marching")) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    app->march = gd::gl::compile(kMarchFs);
    app->blit = gd::gl::compile(kBlitFs);
    if (app->march == 0 || app->blit == 0) return SDL_APP_FAILURE;
    GLuint p = app->march;
    app->u = Uniforms{gd::gl::uniform(p, "uTime"),     gd::gl::uniform(p, "uK"),      gd::gl::uniform(p, "uMaxSteps"), gd::gl::uniform(p, "uHeat"),
                      gd::gl::uniform(p, "uShadows"), gd::gl::uniform(p, "uCam"),    gd::gl::uniform(p, "uTarget")};
    glUseProgram(app->blit);
    glUniform1i(gd::gl::uniform(app->blit, "uTex"), 0);
    if (!createTarget(*app)) return SDL_APP_FAILURE;
    buildChips(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT || (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_ESCAPE)) return SDL_APP_SUCCESS;

    if (event->type == SDL_EVENT_MOUSE_BUTTON_DOWN && event->button.button == SDL_BUTTON_LEFT) {
        click(app, app.win.toLogical(*event));
    } else if (event->type == SDL_EVENT_MOUSE_MOTION && app.dragging) {
        const SDL_FPoint p = app.win.toLogical(*event);
        app.yaw -= (p.x - app.lastMouse.x) * 0.008F;
        app.pitch = std::clamp(app.pitch + (p.y - app.lastMouse.y) * 0.008F, 0.05F, 1.45F);
        app.lastMouse = p;
    } else if (event->type == SDL_EVENT_MOUSE_BUTTON_UP || event->type == SDL_EVENT_WINDOW_MOUSE_LEAVE || event->type == SDL_EVENT_WINDOW_FOCUS_LOST) {
        app.dragging = false;  // a release outside the canvas never arrives as BUTTON_UP, so leaving or losing focus ends the drag too
    } else if (event->type == SDL_EVENT_MOUSE_WHEEL) {
        app.dist = std::clamp(app.dist * std::exp(-event->wheel.y * 0.1F), 3.2F, 12.0F);
    } else if (event->type == SDL_EVENT_KEY_DOWN) {
        // , and . scrub the time and are meant to repeat while held; the toggles are not
        if (event->key.key == SDLK_COMMA) app.time = std::max(0.0, app.time - 0.1);  // scrub while paused
        if (event->key.key == SDLK_PERIOD) app.time += 0.1;
        if (event->key.repeat) return SDL_APP_CONTINUE;
        if (event->key.key == SDLK_H) app.heat = !app.heat;
        if (event->key.key == SDLK_S) app.shadows = !app.shadows;
        if (event->key.key == SDLK_SPACE) app.paused = !app.paused;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;
    if (!app.paused) app.time += frameTime;

    // Camera on a sphere around the target.
    const gd::march::Vec3 target{0.0F, 0.85F, 0.0F};
    const gd::march::Vec3 cam = target + gd::march::Vec3{std::sin(app.yaw) * std::cos(app.pitch), std::sin(app.pitch), std::cos(app.yaw) * std::cos(app.pitch)} * app.dist;

    // Pass 1: the ray marcher into the render target.
    glBindFramebuffer(GL_FRAMEBUFFER, app.fbo);
    glViewport(0, 0, app.targetW, app.targetH);
    glUseProgram(app.march);
    glUniform1f(app.u.time, static_cast<float>(gd::march::wrapTime(app.time)));
    glUniform1f(app.u.k, kBlend[static_cast<std::size_t>(app.blend)]);
    glUniform1i(app.u.maxSteps, kStepLimits[static_cast<std::size_t>(app.steps)]);
    glUniform1i(app.u.heat, app.heat ? 1 : 0);
    glUniform1i(app.u.shadows, app.shadows ? 1 : 0);
    glUniform3f(app.u.cam, cam.x, cam.y, cam.z);
    glUniform3f(app.u.target, target.x, target.y, target.z);
    gd::gl::drawFullscreen();

    // Pass 2: scale it onto the canvas, then the HUD on top.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    app.win.begin();
    glUseProgram(app.blit);
    app.target.bind(0);
    gd::gl::drawFullscreen();

    gd::gl::Hud& hud = app.hud;
    char right[64];
    SDL_snprintf(right, sizeof right, "max steps %d · render %d×%d", kStepLimits[static_cast<std::size_t>(app.steps)], app.targetW, app.targetH);
    hud.titleBar(gd::gl::kWidth, "[ RAY MARCHING ]", right);
    drawChips(app);

    const char* hint = "drag: orbit · wheel: zoom";
    hud.text(960.0F - 24.0F - hud.width(hint), 451.0F, hint, gd::kInk);
    if (app.heat) {
        // Legend of the heatmap: steps per pixel, dark = the budget ran out.
        constexpr int kStrips = 32;
        const float x0 = 646.0F;
        const float stripW = 4.5F;
        hud.text(x0 - 16.0F, 491.0F, "0", gd::kMuted);
        for (int i = 0; i < kStrips; ++i) {
            const gd::march::Vec3 c = gd::march::heat(gd::march::kHeatSteps * static_cast<float>(i) / (kStrips - 1));
            hud.fillRect(x0 + static_cast<float>(i) * stripW, 490.0F, stripW + 0.5F, 20.0F, gd::Color{static_cast<Uint8>(c.x * 255.0F), static_cast<Uint8>(c.y * 255.0F), static_cast<Uint8>(c.z * 255.0F)});
        }
        hud.strokeRect(x0, 490.0F, kStrips * stripW, 20.0F, gd::kInk);
        hud.text(x0 + kStrips * stripW + 8.0F, 491.0F, "48+", gd::kInk);
        hud.fillRect(x0 + kStrips * stripW + 48.0F, 490.0F, 20.0F, 20.0F, gd::kInk);
        hud.text(x0 + kStrips * stripW + 74.0F, 491.0F, "limit", gd::kInk);
    } else {
        char keys[64];
        if (app.paused) {
            SDL_snprintf(keys, sizeof keys, "paused t=%.1fs · , . step", app.time);
        } else {
            SDL_snprintf(keys, sizeof keys, "keys: H S space");
        }
        hud.text(960.0F - 24.0F - hud.width(keys), 491.0F, keys, gd::kMuted);
    }

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        // SDL_AppQuit also runs after a failed SDL_AppInit; without a GL context no glDelete* call is safe
        if (app->win.context != nullptr) {
            app->hud.destroy();
            if (app->target.id != 0) app->target.destroy();
            glDeleteFramebuffers(1, &app->fbo);
            glDeleteProgram(app->march);
            glDeleteProgram(app->blit);
        }
        app->win.destroy();
        delete app;
    }
}
