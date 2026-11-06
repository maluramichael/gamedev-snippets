// Voronoi diagrams in a fragment shader.
//
// Mode "sites": up to 48 points drift slowly over the canvas (fixed steps with an accumulator, bouncing off the walls) and
// go to the GPU as a uniform array. Per pixel the shader loops over all of them (brute force), keeps the nearest and the
// second nearest, colors the cell by a hash of the site's id and measures the exact distance to the cell border with a
// second pass over the bisectors. Mode "worley": no point list at all. Every lattice cell owns one feature point from an
// integer hash and a pixel only looks at the 3x3 cells around it, so the pattern covers an endless plane at the same cost.
// The CPU keeps what has to be exact and testable (drifting, Lloyd relaxation on a coarse grid, the hash), logic.hpp
// mirrors the shader math and test.cpp checks it. The pixel work is the GPU's.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gl.hpp"
#include "logic.hpp"

namespace {

using gd::voronoi::Body;
using gd::voronoi::Metric;
using gd::voronoi::Vec2;

constexpr int kMaxSites = 48;
constexpr int kMinSites = 3;
constexpr float kFieldTop = 30.0F;      // below the title bar
constexpr float kFieldBottom = 456.0F;  // above the chip bar
constexpr float kFieldW = 960.0F;
constexpr float kFieldH = kFieldBottom - kFieldTop;
constexpr int kGridW = 120;  // Lloyd grid: 8 px wide cells
constexpr int kGridH = 54;   // and 7.9 px high ones
constexpr float kGrab = 16.0F;
constexpr double kStepsPerSecond = 60.0;
constexpr double kMaxFrameTime = 0.25;
constexpr double kRelaxSeconds = 0.5;
constexpr float kTimeRate = 0.6F;                            // radians per second of the worley wobble
constexpr float kTimeWrap = 62.8318530718F;                  // 20 pi: both sine frequencies (1 and 1.3) repeat here, keeps the GPU's sin() accurate
constexpr std::array<const char*, 3> kMetricNames{"euclid", "manhattan", "chebyshev"};

// Shared GLSL: metric, the integer hash from logic.hpp (same constants), the palette.
constexpr const char* kCommonFs = R"(
uniform int uMetric;      // 0 euclid, 1 manhattan, 2 chebyshev
uniform float uEdges;     // cell borders on/off
uniform float uRings;     // distance isolines on/off
uniform float uExact;     // 1: exact border distance (euclid only), 0: the cheap (d2 - d1) / 2
in vec2 vUv;
out vec4 outColor;

float metricDist(vec2 a, vec2 b) {
    vec2 d = abs(a - b);
    if (uMetric == 1) return d.x + d.y;
    if (uMetric == 2) return max(d.x, d.y);
    return length(d);
}

uint pcg(uint v) {
    uint s = v * 747796405u + 2891336453u;
    uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}
float unit(uint h) { return float(h >> 8u) * (1.0 / 16777216.0); }

const vec3 kTeal = vec3(0.051, 0.580, 0.533);
const vec3 kTealDark = vec3(0.059, 0.463, 0.431);
const vec3 kTealSoft = vec3(0.80, 0.984, 0.945);
const vec3 kOrange = vec3(0.918, 0.345, 0.047);
const vec3 kOrangeSoft = vec3(0.99, 0.80, 0.62);
const vec3 kInk = vec3(0.067, 0.094, 0.153);

// Five brightness steps per family, mostly teal, some orange. The hash of the cell decides, so a cell keeps its color.
vec3 cellColor(uint h) {
    uint a = pcg(h + 1u);
    uint b = pcg(a);
    float level = floor(unit(b) * 5.0) / 4.0;
    if (unit(a) < 0.78) return mix(kTealDark, kTealSoft, level);
    return mix(kOrange, kOrangeSoft, level * 0.8);
}

// Cell color with the three optional layers: soft shading towards the cell edge, isolines of the distance, borders, point.
// dist = distance to the nearest point in canvas pixels, edge = distance to the border in pixels, dot = Euclidean pixels to the point.
vec3 shade(vec3 color, float dist, float edge, float dotDist) {
    color *= 1.0 - 0.12 * smoothstep(0.0, 140.0, dist);
    float f = fract(dist / 20.0);
    float ring = 1.0 - smoothstep(0.4, 1.3, min(f, 1.0 - f) * 20.0);
    color = mix(color, vec3(1.0), ring * 0.38 * uRings);
    float line = 1.0 - smoothstep(0.9, 2.1, edge);
    color = mix(color, kInk, line * 0.9 * uEdges);
    color = mix(color, vec3(1.0), 1.0 - smoothstep(5.5, 6.7, dotDist));
    color = mix(color, kInk, 1.0 - smoothstep(3.6, 4.6, dotDist));
    return color;
}
)";

constexpr const char* kSitesMain = R"(
uniform vec4 uSites[48];   // xy = canvas pixels, z = stable id (colors must not change when a site is deleted)
uniform int uCount;

void main() {
    vec2 p = vUv * vec2(960.0, 540.0);
    float d1 = 1e9, d2 = 1e9;
    int first = 0;
    for (int i = 0; i < 48; i++) {
        if (i >= uCount) break;
        float d = metricDist(uSites[i].xy, p);
        if (d < d1) { d2 = d1; d1 = d; first = i; }
        else if (d < d2) { d2 = d; }
    }
    vec2 a = uSites[first].xy;
    float edge;
    if (uMetric == 0 && uExact > 0.5) {
        // Exact: smallest distance to the perpendicular bisector of the nearest site and every other one.
        edge = 1e9;
        for (int j = 0; j < 48; j++) {
            if (j >= uCount) break;
            if (j == first) continue;
            vec2 b = uSites[j].xy;
            float len = length(b - a);
            if (len > 0.0) edge = min(edge, dot(0.5 * (a + b) - p, b - a) / len);
        }
    } else {
        edge = 0.5 * (d2 - d1);   // the bisectors of the other metrics are not lines, a cheap bound has to do
    }
    outColor = vec4(shade(cellColor(uint(uSites[first].z)), d1, edge, length(a - p)), 1.0);
}
)";

constexpr const char* kWorleyMain = R"(
uniform float uScale;    // lattice cells across the canvas width
uniform vec2 uOrigin;    // lattice coordinates of the canvas' top left corner
uniform float uTime;

uint hashCell(ivec2 c) { return pcg(pcg(uint(c.x) + pcg(0u)) + uint(c.y)); }

vec2 featureOffset(uint h, float t) {
    uint h2 = pcg(h), h3 = pcg(h2), h4 = pcg(h3);
    return vec2(0.15 + 0.7 * unit(h) + 0.15 * sin(t + 6.28318530718 * unit(h3)),
                0.15 + 0.7 * unit(h2) + 0.15 * sin(t * 1.3 + 6.28318530718 * unit(h4)));
}

void main() {
    float pxPerUnit = 960.0 / uScale;
    vec2 q = vUv * vec2(960.0, 540.0) / pxPerUnit + uOrigin;
    ivec2 c = ivec2(floor(q));
    float f1 = 1e9, f2 = 1e9;
    ivec2 owner = c;
    vec2 point = vec2(c);
    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            ivec2 cell = c + ivec2(i, j);
            vec2 f = vec2(cell) + featureOffset(hashCell(cell), uTime);
            float d = metricDist(f, q);
            if (d < f1) { f2 = f1; f1 = d; owner = cell; point = f; }
            else if (d < f2) { f2 = d; }
        }
    }
    float edge = 0.5 * (f2 - f1) * pxPerUnit;    // F2 - F1 is zero on every cell border, but grows slowly along a short one (wedges)
    if (uMetric == 0 && uExact > 0.5) {
        // Exact: bisectors against the 5x5 cells around the owner (a border neighbour can be two cells away).
        edge = 1e9;
        for (int j = -2; j <= 2; j++) {
            for (int i = -2; i <= 2; i++) {
                ivec2 cell = owner + ivec2(i, j);
                vec2 f = vec2(cell) + featureOffset(hashCell(cell), uTime);
                float len = length(f - point);
                if (len > 0.0) edge = min(edge, dot(0.5 * (point + f) - q, f - point) / len);
            }
        }
        edge *= pxPerUnit;
    }
    outColor = vec4(shade(cellColor(hashCell(owner)), f1 * pxPerUnit, edge, length(point - q) * pxPerUnit), 1.0);
}
)";

enum class Mode { Sites, Worley };
enum class Action { ModeSites, ModeWorley, MetricEuclid, MetricManhattan, MetricChebyshev, Edges, Exact, Rings, Drift, Relax };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Program {
    GLuint id = 0;
    GLint metric = -1, edges = -1, rings = -1, exact = -1;
    GLint sites = -1, count = -1;
    GLint scale = -1, origin = -1, time = -1;
};

struct App {
    gd::gl::Window win;
    gd::gl::Hud hud;
    Program sitesProgram, worleyProgram;
    std::vector<Chip> chips;

    Mode mode = Mode::Sites;
    Metric metric = Metric::Euclid;
    bool edges = true;
    bool exact = true;
    bool rings = false;
    bool driftOn = true;

    std::vector<Body> bodies;
    std::vector<float> ids;
    float nextId = 1.0F;
    int held = -1;  // site under the mouse

    // Lloyd relaxation, animated: positions glide from `relaxFrom` to `relaxTo`.
    std::vector<Vec2> relaxFrom, relaxTo;
    double relaxT = 1.0;
    double meanOffset = 0.0;  // mean distance of a site to the center of its cell, in pixels
    long statFrame = 0;

    float worleyScale = 9.0F;
    Vec2 worleyOrigin{0.0F, 0.0F};
    float worleyTime = 0.0F;
    bool panning = false;
    SDL_FPoint lastMouse{0.0F, 0.0F};

    Uint64 last = 0;
    double accumulator = 0.0;
    std::uint32_t rng = 20270205U;
    long maxFrames = -1;
    long frames = 0;

    float random() {  // xorshift32 -> [0, 1)
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng >> 8) / 16777216.0F;
    }
};

// ---- helpers -----------------------------------------------------------------------------------------------------

Vec2 clampToField(Vec2 p) { return Vec2{std::clamp(p.x, 0.0F, kFieldW), std::clamp(p.y, kFieldTop, kFieldBottom)}; }

std::vector<Vec2> positions(const App& app) {
    std::vector<Vec2> out;
    out.reserve(app.bodies.size());
    for (const Body& b : app.bodies) out.push_back(b.p);
    return out;
}

// Canvas pixels <-> Lloyd grid units (one unit per grid cell).
Vec2 toGrid(Vec2 p) { return Vec2{p.x / (kFieldW / kGridW), (p.y - kFieldTop) / (kFieldH / kGridH)}; }
Vec2 fromGrid(Vec2 g) { return Vec2{g.x * (kFieldW / kGridW), kFieldTop + g.y * (kFieldH / kGridH)}; }

// Where Lloyd relaxation would put the sites (canvas pixels).
std::vector<Vec2> lloydTargets(const std::vector<Vec2>& sites, Metric metric) {
    std::vector<Vec2> grid;
    grid.reserve(sites.size());
    for (const Vec2& s : sites) grid.push_back(toGrid(s));
    std::vector<Vec2> out = gd::voronoi::lloydStep(grid, kGridW, kGridH, metric);
    for (Vec2& p : out) p = fromGrid(p);
    return out;
}

double meanOffsetOf(const std::vector<Vec2>& sites, Metric metric) {
    if (sites.empty()) return 0.0;
    const std::vector<Vec2> target = lloydTargets(sites, metric);
    double sum = 0.0;
    for (std::size_t i = 0; i < sites.size(); ++i) sum += gd::voronoi::distance(Metric::Euclid, sites[i], target[i]);
    return sum / static_cast<double>(sites.size());
}

void addSite(App& app, Vec2 p) {
    const float angle = app.random() * 6.2831853F;
    const float speed = 10.0F + app.random() * 16.0F;  // px per second: a slow drift
    app.bodies.push_back(Body{clampToField(p), Vec2{std::cos(angle) * speed, std::sin(angle) * speed}});
    app.ids.push_back(app.nextId);
    app.nextId += 1.0F;
}

// New random layout: a jittered grid (no two sites on top of each other) pulled a bit together by two Lloyd steps,
// so the first frame is already evenly filled. The count stays as it is.
void reshuffle(App& app, int count) {
    app.bodies.clear();
    app.ids.clear();
    const int cols = 8;
    const int rows = (count + cols - 1) / cols;
    std::vector<Vec2> pts;
    for (int i = 0; i < count; ++i) {
        const float cx = (static_cast<float>(i % cols) + 0.15F + 0.7F * app.random()) * (kFieldW / cols);
        const float cy = kFieldTop + (static_cast<float>(i / cols) + 0.15F + 0.7F * app.random()) * (kFieldH / static_cast<float>(rows));
        pts.push_back(Vec2{cx, cy});
    }
    for (int k = 0; k < 2; ++k) pts = lloydTargets(pts, Metric::Euclid);
    for (const Vec2& p : pts) addSite(app, p);
    app.relaxT = 1.0;
    app.held = -1;
    app.meanOffset = meanOffsetOf(positions(app), app.metric);
}

void startRelax(App& app) {
    if (app.mode != Mode::Sites) return;
    app.relaxFrom = positions(app);
    app.relaxTo = lloydTargets(app.relaxFrom, app.metric);
    app.relaxT = 0.0;
}

bool chipActive(const App& app, Action a) {
    switch (a) {
        case Action::ModeSites: return app.mode == Mode::Sites;
        case Action::ModeWorley: return app.mode == Mode::Worley;
        case Action::MetricEuclid: return app.metric == Metric::Euclid;
        case Action::MetricManhattan: return app.metric == Metric::Manhattan;
        case Action::MetricChebyshev: return app.metric == Metric::Chebyshev;
        case Action::Edges: return app.edges;
        case Action::Exact: return app.exact;
        case Action::Rings: return app.rings;
        case Action::Drift: return app.driftOn;
        case Action::Relax: return app.relaxT < 1.0;
    }
    return false;
}

bool chipEnabled(const App& app, Action a) {
    if (a == Action::Relax) return app.mode == Mode::Sites;
    if (a == Action::Exact) return app.metric == Metric::Euclid;  // the other metrics have no straight bisectors
    return true;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::ModeSites: app.mode = Mode::Sites; break;
        case Action::ModeWorley: app.mode = Mode::Worley; app.held = -1; break;
        case Action::MetricEuclid: app.metric = Metric::Euclid; break;
        case Action::MetricManhattan: app.metric = Metric::Manhattan; break;
        case Action::MetricChebyshev: app.metric = Metric::Chebyshev; break;
        case Action::Edges: app.edges = !app.edges; break;
        case Action::Exact: app.exact = !app.exact; break;
        case Action::Rings: app.rings = !app.rings; break;
        case Action::Drift: app.driftOn = !app.driftOn; break;
        case Action::Relax: startRelax(app); break;
    }
    if (a == Action::MetricEuclid || a == Action::MetricManhattan || a == Action::MetricChebyshev) app.meanOffset = meanOffsetOf(positions(app), app.metric);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    constexpr float row1 = 464.0F, row2 = 502.0F;
    const std::array<Def, 10> defs{{{Action::ModeSites, "sites", 78.0F, row1},
                                   {Action::ModeWorley, "worley", 8.0F, row1},
                                   {Action::MetricEuclid, "euclid", 108.0F, row1},
                                   {Action::MetricManhattan, "manhattan", 8.0F, row1},
                                   {Action::MetricChebyshev, "chebyshev", 8.0F, row1},
                                   {Action::Edges, "edges", 78.0F, row2},
                                   {Action::Exact, "exact", 8.0F, row2},
                                   {Action::Rings, "rings", 8.0F, row2},
                                   {Action::Drift, "drift", 8.0F, row2},
                                   {Action::Relax, "relax", 40.0F, row2}}};
    float x = 20.0F;
    float lastY = 0.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 20.0F;  // new row
        lastY = d.y;
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::gl::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, 30.0F}});
        x += w;
    }
}

int nearestSite(const App& app, Vec2 p, float maxDistance) {
    int best = -1;
    float bestD = maxDistance;
    for (int i = 0; i < static_cast<int>(app.bodies.size()); ++i) {
        const float d = gd::voronoi::distance(Metric::Euclid, app.bodies[static_cast<std::size_t>(i)].p, p);
        if (d <= bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

void removeSite(App& app, int index) {
    app.bodies.erase(app.bodies.begin() + index);
    app.ids.erase(app.ids.begin() + index);
    app.held = -1;
    app.relaxT = 1.0;
    app.meanOffset = meanOffsetOf(positions(app), app.metric);
}

bool inField(SDL_FPoint p) { return p.y >= kFieldTop && p.y <= kFieldBottom && p.x >= 0.0F && p.x <= kFieldW; }

Program makeProgram(const std::string& main) {
    Program p;
    const std::string src = std::string(kCommonFs) + main;
    p.id = gd::gl::compile(src.c_str());
    if (p.id == 0) return p;
    p.metric = gd::gl::uniform(p.id, "uMetric");
    p.edges = gd::gl::uniform(p.id, "uEdges");
    p.rings = gd::gl::uniform(p.id, "uRings");
    p.exact = gd::gl::uniform(p.id, "uExact");
    p.sites = gd::gl::uniform(p.id, "uSites");
    p.count = gd::gl::uniform(p.id, "uCount");
    p.scale = gd::gl::uniform(p.id, "uScale");
    p.origin = gd::gl::uniform(p.id, "uOrigin");
    p.time = gd::gl::uniform(p.id, "uTime");
    return p;
}

void drawChip(App& app, const Chip& c) {
    using Style = gd::gl::Hud::Style;
    const bool enabled = chipEnabled(app, c.action);
    const bool on = enabled && chipActive(app, c.action);
    const gd::Color fill = !enabled ? gd::kGrid : on ? (c.action == Action::Relax ? gd::kOrange : gd::kTeal) : gd::kWhite;
    const gd::Color text = !enabled ? gd::kMuted : on ? gd::kWhite : gd::kInk;
    app.hud.card(c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, enabled ? gd::kInk : gd::kMuted);
    app.hud.text(c.rect.x + (c.rect.w - app.hud.width(c.label, Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, text, Style::Bold);
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
    if (!app->win.create("voronoi")) return SDL_APP_FAILURE;
    if (!app->hud.load()) {
        SDL_Log("hud failed to load");
        return SDL_APP_FAILURE;
    }
    app->sitesProgram = makeProgram(kSitesMain);
    app->worleyProgram = makeProgram(kWorleyMain);
    if (app->sitesProgram.id == 0 || app->worleyProgram.id == 0) return SDL_APP_FAILURE;
    layoutChips(*app);
    reshuffle(*app, 32);
    app->worleyOrigin = Vec2{3.0F, 2.0F};
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    if (event->type == SDL_EVENT_QUIT) return SDL_APP_SUCCESS;

    switch (event->type) {
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p = app.win.toLogical(*event);
            app.lastMouse = p;
            if (event->button.button == SDL_BUTTON_RIGHT) {
                if (app.mode == Mode::Sites && inField(p) && static_cast<int>(app.bodies.size()) > kMinSites) {
                    const int i = nearestSite(app, Vec2{p.x, p.y}, 1.0e9F);
                    if (i >= 0) removeSite(app, i);
                }
                break;
            }
            if (event->button.button != SDL_BUTTON_LEFT) break;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    if (chipEnabled(app, c.action)) apply(app, c.action);
                    onChip = true;
                }
            }
            if (onChip || !inField(p)) break;
            if (app.mode == Mode::Worley) {
                app.panning = true;
            } else {
                app.relaxT = 1.0;  // a click cancels a running relaxation
                int i = nearestSite(app, Vec2{p.x, p.y}, kGrab);
                if (i < 0 && static_cast<int>(app.bodies.size()) < kMaxSites) {
                    addSite(app, Vec2{p.x, p.y});
                    i = static_cast<int>(app.bodies.size()) - 1;
                }
                app.held = i;
            }
            break;
        }
        case SDL_EVENT_MOUSE_MOTION: {
            const SDL_FPoint p = app.win.toLogical(*event);
            if (app.held >= 0 && app.held < static_cast<int>(app.bodies.size())) {
                app.bodies[static_cast<std::size_t>(app.held)].p = clampToField(Vec2{p.x, p.y});
            } else if (app.panning) {
                const float perPixel = app.worleyScale / kFieldW;
                app.worleyOrigin.x -= (p.x - app.lastMouse.x) * perPixel;
                app.worleyOrigin.y -= (p.y - app.lastMouse.y) * perPixel;
            }
            app.lastMouse = p;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) {
                app.held = -1;
                app.panning = false;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.held = -1;
            app.panning = false;
            break;
        case SDL_EVENT_MOUSE_WHEEL: {
            if (app.mode != Mode::Worley) break;
            // Zoom around the cursor: the lattice point under it stays under it.
            const SDL_FPoint p = app.win.toLogical(event->wheel.mouse_x, event->wheel.mouse_y);
            const float under = 1.0F / kFieldW;
            const Vec2 world{app.worleyOrigin.x + p.x * under * app.worleyScale, app.worleyOrigin.y + p.y * under * app.worleyScale};
            app.worleyScale = std::clamp(app.worleyScale * std::pow(1.12F, -event->wheel.y), 2.0F, 40.0F);
            app.worleyOrigin = Vec2{world.x - p.x * under * app.worleyScale, world.y - p.y * under * app.worleyScale};
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_R: reshuffle(app, static_cast<int>(app.bodies.size())); break;
                case SDLK_L: startRelax(app); break;
                case SDLK_E: apply(app, Action::Edges); break;
                case SDLK_X: if (chipEnabled(app, Action::Exact)) apply(app, Action::Exact); break;
                case SDLK_M: apply(app, app.mode == Mode::Sites ? Action::ModeWorley : Action::ModeSites); break;
                case SDLK_SPACE: apply(app, Action::Drift); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;

    // Fixed steps: the same drift on a 30 Hz and a 240 Hz display.
    const double stepTime = 1.0 / kStepsPerSecond;
    app.accumulator += frameTime;
    while (app.accumulator >= stepTime) {
        const bool relaxing = app.relaxT < 1.0;
        if (app.driftOn && !relaxing) {
            for (std::size_t i = 0; i < app.bodies.size(); ++i) {
                if (static_cast<int>(i) == app.held) continue;
                Body b = app.bodies[i];
                b.p.y -= kFieldTop;  // drift() works in [0, w] x [0, h]
                gd::voronoi::drift(b, static_cast<float>(stepTime), kFieldW, kFieldH);
                b.p.y += kFieldTop;
                app.bodies[i] = b;
            }
        }
        if (app.driftOn) app.worleyTime = std::fmod(app.worleyTime + static_cast<float>(stepTime) * kTimeRate, kTimeWrap);
        app.accumulator -= stepTime;
    }

    if (app.relaxT < 1.0) {
        app.relaxT = std::min(1.0, app.relaxT + frameTime / kRelaxSeconds);
        const double t = 1.0 - std::pow(1.0 - app.relaxT, 3.0);  // ease out
        for (std::size_t i = 0; i < app.bodies.size() && i < app.relaxFrom.size(); ++i) {
            if (static_cast<int>(i) == app.held) continue;
            app.bodies[i].p.x = static_cast<float>(app.relaxFrom[i].x + (app.relaxTo[i].x - app.relaxFrom[i].x) * t);
            app.bodies[i].p.y = static_cast<float>(app.relaxFrom[i].y + (app.relaxTo[i].y - app.relaxFrom[i].y) * t);
        }
    }
    if (++app.statFrame % 12 == 0 && app.mode == Mode::Sites) app.meanOffset = meanOffsetOf(positions(app), app.metric);

    app.win.begin();
    const bool sitesMode = app.mode == Mode::Sites;
    const Program& prog = sitesMode ? app.sitesProgram : app.worleyProgram;
    glUseProgram(prog.id);
    glUniform1i(prog.metric, static_cast<GLint>(app.metric));
    glUniform1f(prog.edges, app.edges ? 1.0F : 0.0F);
    glUniform1f(prog.rings, app.rings ? 1.0F : 0.0F);
    glUniform1f(prog.exact, app.exact ? 1.0F : 0.0F);
    if (sitesMode) {
        std::array<float, kMaxSites * 4> data{};
        for (std::size_t i = 0; i < app.bodies.size(); ++i) {
            data[i * 4] = app.bodies[i].p.x;
            data[i * 4 + 1] = app.bodies[i].p.y;
            data[i * 4 + 2] = app.ids[i];
        }
        glUniform4fv(prog.sites, kMaxSites, data.data());
        glUniform1i(prog.count, static_cast<GLint>(app.bodies.size()));
    } else {
        glUniform1f(prog.scale, app.worleyScale);
        glUniform2f(prog.origin, app.worleyOrigin.x, app.worleyOrigin.y);
        glUniform1f(prog.time, app.worleyTime);
    }
    gd::gl::drawFullscreen();

    gd::gl::Hud& hud = app.hud;
    char buf[96];
    if (sitesMode) {
        hud.titleBar(gd::gl::kWidth, "[ VORONOI ]", "click: add · drag: move · right click: delete · R: reshuffle");
        SDL_snprintf(buf, sizeof buf, "%d sites · mean offset to cell center %.0f px", static_cast<int>(app.bodies.size()), app.meanOffset);
    } else {
        hud.titleBar(gd::gl::kWidth, "[ WORLEY NOISE ]", "wheel: zoom · drag: pan · 3×3 cells per pixel");
        SDL_snprintf(buf, sizeof buf, "%.1f cells across · %s", static_cast<double>(app.worleyScale), kMetricNames[static_cast<std::size_t>(app.metric)]);
    }
    const float labelW = hud.width(buf) + 20.0F;
    hud.card(12.0F, 40.0F, labelW, 28.0F, gd::kWhite, gd::kInk);
    hud.text(22.0F, 43.0F, buf, gd::kInk);

    hud.fillRect(0.0F, 456.0F, 960.0F, 84.0F, gd::Color{250, 250, 249, 242});
    hud.fillRect(0.0F, 456.0F, 960.0F, 2.0F, gd::kTealDark);
    hud.text(20.0F, 468.0F, "mode:", gd::kMuted);
    hud.text(20.0F + 78.0F + 70.0F + 8.0F + 80.0F + 30.0F, 468.0F, "metric:", gd::kMuted);
    hud.text(20.0F, 506.0F, "show:", gd::kMuted);
    for (const Chip& c : app.chips) drawChip(app, c);
    const char* keys = "keys: R L E X M space";
    hud.text(960.0F - 20.0F - hud.width(keys), 506.0F, keys, gd::kMuted);

    app.win.swap();
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        glDeleteProgram(app->sitesProgram.id);
        glDeleteProgram(app->worleyProgram.id);
        app->win.destroy();
        delete app;
    }
}
