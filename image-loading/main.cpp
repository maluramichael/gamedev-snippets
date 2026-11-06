// From a PNG file to a pixel on screen, with SDL3 main callbacks.
//
// The sprite is a 64x64 PNG that lies in the binary as a byte array (asset.hpp, made by make_asset.py), so the demo needs no
// file at run time. Four panels follow its way: (1) the bytes of the file, read by our own chunk parser (logic.hpp), (2) the
// SDL_Surface that SDL_LoadPNG_IO makes of them, in CPU memory, with its pixel format, pitch and byte order, (3) the same
// pixels as an SDL_Texture, which the renderer scales, flips and tints on the GPU, (4) a CPU loop over the surface rows
// that makes a new surface and from it a new texture. The damage chips break the file to see what the parser and SDL say,
// "pad rows" gives the surface a pitch larger than width * 4 and "width*4 bug" shows what a loop does that ignores it.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "asset.hpp"
#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kPanelTop = 36.0F;
constexpr float kPanelGap = 10.0F;
constexpr float kPanelHalf = 182.0F;  // panels 1 and 2 (left column)
constexpr float kPanelFull = kPanelHalf * 2.0F + kPanelGap;
constexpr float kColA = 20.0F;
constexpr float kColB = 332.0F;
constexpr float kColC = 644.0F;
constexpr float kColW = 300.0F;
constexpr float kBox = 256.0F;      // the square the image is drawn in, 4x of 64
constexpr float kBoxTop = 66.0F;
constexpr float kBoxInset = 22.0F;  // (kColW - kBox) / 2
constexpr int kPadBytes = 64;       // extra bytes at the end of every row in "pad rows" mode (a 64 byte row alignment)
constexpr int kRuns = 1000;         // the CPU loop is repeated and averaged, a single run is far below the timer's resolution in a browser
constexpr std::array<int, 3> kZooms{1, 2, 4};
constexpr std::array<std::pair<const char*, gd::Color>, 3> kMods{{{"white", {255, 255, 255}}, {"teal", {13, 148, 136}}, {"orange", {234, 88, 12}}}};
constexpr std::array<int, 3> kAlphas{255, 128, 64};
constexpr float kChipH = 30.0F;
constexpr std::array<float, 3> kChipRows{420.0F, 456.0F, 492.0F};

enum class Action {
    DamageNone, DamageSignature, DamageCrc, DamageCutOff,
    Nearest, Linear, Zoom1, Zoom2, Zoom4, FlipX, FlipY, Color, Alpha,
    OpNone, OpGray, OpInvert, OpTint, Pad, Bug
};

struct Chip {
    Action action;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    // Settings
    imgl::Damage damage = imgl::Damage::None;
    SDL_ScaleMode scale = SDL_SCALEMODE_NEAREST;
    int zoomIndex = 2;
    bool flipX = false;
    bool flipY = false;
    int mod = 0;
    int alpha = 0;
    imgl::Op op = imgl::Op::Grayscale;
    bool pad = false;
    bool bug = false;
    int px = 32;  // the inspected pixel
    int py = 36;
    bool picking = false;

    // The pipeline, rebuilt by rebuild() whenever a setting that changes bytes changes
    std::vector<Uint8> original;
    std::vector<Uint8> file;
    imgl::Parsed parsed;
    std::string loadError;
    SDL_Surface* loaded = nullptr;  // what SDL_LoadPNG_IO returned
    SDL_Surface* base = nullptr;    // the same, or a converted copy if the format is not 8 bits per channel
    bool converted = false;
    std::vector<Uint8> padBuf;
    SDL_Surface* padded = nullptr;  // a view of padBuf with a larger pitch
    SDL_Surface* work = nullptr;    // base or padded: the surface the CPU loop reads and the inspector looks at
    std::vector<Uint8> outBuf;
    SDL_Surface* result = nullptr;  // a view of outBuf
    SDL_Texture* texOriginal = nullptr;
    SDL_Texture* texResult = nullptr;
    imgl::ChannelBytes channels;
    std::string order;
    double usPerRun = -1.0;

    long maxFrames = -1;
    long frames = 0;
};

int zoom(const App& app) { return kZooms[static_cast<std::size_t>(app.zoomIndex)]; }

void release(App& app) {
    SDL_DestroyTexture(app.texOriginal);
    SDL_DestroyTexture(app.texResult);
    app.texOriginal = app.texResult = nullptr;
    SDL_DestroySurface(app.result);  // views first, then the surfaces that own their pixels
    SDL_DestroySurface(app.padded);
    if (app.converted) SDL_DestroySurface(app.base);
    SDL_DestroySurface(app.loaded);
    app.result = app.padded = app.base = app.loaded = app.work = nullptr;
    app.converted = false;
    app.padBuf.clear();
    app.outBuf.clear();
    app.usPerRun = -1.0;
}

/// Runs the whole pipeline again: damage the bytes, parse them, load them with SDL, optionally pad the rows, run the CPU loop.
/// Returns false only for an unexpected SDL failure; a file that SDL cannot load is a normal state (loaded == nullptr).
bool rebuild(App& app) {
    release(app);
    app.loadError.clear();
    app.file = imgl::damaged(app.original, app.damage);
    app.parsed = imgl::parse(app.file);

    SDL_IOStream* io = SDL_IOFromConstMem(app.file.data(), app.file.size());
    app.loaded = io != nullptr ? SDL_LoadPNG_IO(io, true) : nullptr;  // true: SDL closes the stream
    if (app.loaded == nullptr) {
        app.loadError = SDL_GetError();
        return true;
    }

    const SDL_PixelFormatDetails* d = SDL_GetPixelFormatDetails(app.loaded->format);
    if (d != nullptr && d->bytes_per_pixel == 4 && imgl::is8888(d->Rmask, d->Gmask, d->Bmask, d->Amask)) {
        app.base = app.loaded;
    } else {
        app.base = SDL_ConvertSurface(app.loaded, SDL_PIXELFORMAT_RGBA32);
        app.converted = true;
        if (app.base == nullptr) {
            SDL_Log("SDL_ConvertSurface failed: %s", SDL_GetError());
            return false;
        }
    }
    const int w = app.base->w;
    const int h = app.base->h;

    if (app.pad) {
        const int pitch = app.base->pitch + kPadBytes;
        app.padBuf = imgl::copyWithPitch(static_cast<const Uint8*>(app.base->pixels), w * 4, h, app.base->pitch, pitch, 0xCD);
        app.padded = SDL_CreateSurfaceFrom(w, h, app.base->format, app.padBuf.data(), pitch);
        if (app.padded == nullptr) {
            SDL_Log("SDL_CreateSurfaceFrom failed: %s", SDL_GetError());
            return false;
        }
        app.work = app.padded;
    } else {
        app.work = app.base;
    }

    const SDL_PixelFormatDetails* wd = SDL_GetPixelFormatDetails(app.work->format);
    app.channels = imgl::channelBytes(wd->Rmask, wd->Gmask, wd->Bmask, wd->Amask, SDL_BYTEORDER == SDL_LIL_ENDIAN);
    app.order = imgl::memoryOrder(app.channels);

    // The CPU loop: read the work surface row by row, write into a copy with the same layout.
    const int pitch = app.work->pitch;
    const auto* src = static_cast<const Uint8*>(app.work->pixels);
    app.outBuf.assign(src, src + static_cast<std::size_t>(pitch) * static_cast<std::size_t>(h));
    if (app.op != imgl::Op::None) {
        const Uint64 t0 = SDL_GetPerformanceCounter();
        for (int run = 0; run < kRuns; ++run) {
            imgl::process(src, app.outBuf.data(), w, h, pitch, app.op, app.channels, app.bug);
            // Every run writes the same bytes. Without a barrier the compiler may notice and do the work once.
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
        const Uint64 t1 = SDL_GetPerformanceCounter();
        app.usPerRun = static_cast<double>(t1 - t0) * 1e6 / static_cast<double>(SDL_GetPerformanceFrequency()) / kRuns;
    }
    app.result = SDL_CreateSurfaceFrom(w, h, app.work->format, app.outBuf.data(), pitch);
    if (app.result == nullptr) {
        SDL_Log("SDL_CreateSurfaceFrom failed: %s", SDL_GetError());
        return false;
    }

    // Surface to texture: the pixels are copied to video memory, the surfaces stay in RAM.
    app.texOriginal = SDL_CreateTextureFromSurface(app.renderer, app.base);
    app.texResult = SDL_CreateTextureFromSurface(app.renderer, app.result);
    if (app.texOriginal == nullptr || app.texResult == nullptr) {
        SDL_Log("SDL_CreateTextureFromSurface failed: %s", SDL_GetError());
        return false;
    }
    SDL_SetTextureBlendMode(app.texOriginal, SDL_BLENDMODE_BLEND);
    SDL_SetTextureBlendMode(app.texResult, SDL_BLENDMODE_BLEND);
    app.px = std::clamp(app.px, 0, w - 1);
    app.py = std::clamp(app.py, 0, h - 1);
    return true;
}

// ---- Chips ---------------------------------------------------------------------------------------------------------

std::string label(const App& app, Action a) {
    switch (a) {
        case Action::DamageNone: return "1 intact";
        case Action::DamageSignature: return "2 bad signature";
        case Action::DamageCrc: return "3 bad CRC";
        case Action::DamageCutOff: return "4 cut off";
        case Action::Nearest: return "N nearest";
        case Action::Linear: return "L linear";
        case Action::Zoom1: return "1x";
        case Action::Zoom2: return "2x";
        case Action::Zoom4: return "4x";
        case Action::FlipX: return "H flip x";
        case Action::FlipY: return "V flip y";
        case Action::Color: return std::string("C ") + kMods[static_cast<std::size_t>(app.mod)].first;
        case Action::Alpha: return "A " + std::to_string(kAlphas[static_cast<std::size_t>(app.alpha)] * 100 / 255) + "%";
        case Action::OpNone: return "0 off";
        case Action::OpGray: return "G grayscale";
        case Action::OpInvert: return "I invert";
        case Action::OpTint: return "T tint";
        case Action::Pad: return "P pad rows";
        case Action::Bug: return "B width*4 bug";
    }
    return {};
}

bool active(const App& app, Action a) {
    switch (a) {
        case Action::DamageNone: return app.damage == imgl::Damage::None;
        case Action::DamageSignature: return app.damage == imgl::Damage::Signature;
        case Action::DamageCrc: return app.damage == imgl::Damage::Crc;
        case Action::DamageCutOff: return app.damage == imgl::Damage::CutOff;
        case Action::Nearest: return app.scale == SDL_SCALEMODE_NEAREST;
        case Action::Linear: return app.scale == SDL_SCALEMODE_LINEAR;
        case Action::Zoom1: return app.zoomIndex == 0;
        case Action::Zoom2: return app.zoomIndex == 1;
        case Action::Zoom4: return app.zoomIndex == 2;
        case Action::FlipX: return app.flipX;
        case Action::FlipY: return app.flipY;
        case Action::Color: return app.mod != 0;
        case Action::Alpha: return app.alpha != 0;
        case Action::OpNone: return app.op == imgl::Op::None;
        case Action::OpGray: return app.op == imgl::Op::Grayscale;
        case Action::OpInvert: return app.op == imgl::Op::Invert;
        case Action::OpTint: return app.op == imgl::Op::Tint;
        case Action::Pad: return app.pad;
        case Action::Bug: return app.bug;
    }
    return false;
}

/// Returns false when the rebuild hit an SDL error.
bool apply(App& app, Action a) {
    bool again = true;
    switch (a) {
        case Action::DamageNone: app.damage = imgl::Damage::None; break;
        case Action::DamageSignature: app.damage = imgl::Damage::Signature; break;
        case Action::DamageCrc: app.damage = imgl::Damage::Crc; break;
        case Action::DamageCutOff: app.damage = imgl::Damage::CutOff; break;
        case Action::OpNone: app.op = imgl::Op::None; break;
        case Action::OpGray: app.op = imgl::Op::Grayscale; break;
        case Action::OpInvert: app.op = imgl::Op::Invert; break;
        case Action::OpTint: app.op = imgl::Op::Tint; break;
        case Action::Pad: app.pad = !app.pad; break;
        case Action::Bug: app.bug = !app.bug; break;
        default: again = false; break;
    }
    switch (a) {  // settings that only change how the textures are drawn
        case Action::Nearest: app.scale = SDL_SCALEMODE_NEAREST; break;
        case Action::Linear: app.scale = SDL_SCALEMODE_LINEAR; break;
        case Action::Zoom1: app.zoomIndex = 0; break;
        case Action::Zoom2: app.zoomIndex = 1; break;
        case Action::Zoom4: app.zoomIndex = 2; break;
        case Action::FlipX: app.flipX = !app.flipX; break;
        case Action::FlipY: app.flipY = !app.flipY; break;
        case Action::Color: app.mod = (app.mod + 1) % static_cast<int>(kMods.size()); break;
        case Action::Alpha: app.alpha = (app.alpha + 1) % static_cast<int>(kAlphas.size()); break;
        default: break;
    }
    return again ? rebuild(app) : true;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* sizing;  // the longest label this chip can show, so the chip does not change its width
        float gapBefore;
        int row;
    };
    const std::array<Def, 19> defs{{
        {Action::DamageNone, "1 intact", 0.0F, 0},
        {Action::DamageSignature, "2 bad signature", 0.0F, 0},
        {Action::DamageCrc, "3 bad CRC", 0.0F, 0},
        {Action::DamageCutOff, "4 cut off", 0.0F, 0},
        {Action::Nearest, "N nearest", 0.0F, 1},
        {Action::Linear, "L linear", 0.0F, 1},
        {Action::Zoom1, "1x", 12.0F, 1},
        {Action::Zoom2, "2x", 0.0F, 1},
        {Action::Zoom4, "4x", 0.0F, 1},
        {Action::FlipX, "H flip x", 12.0F, 1},
        {Action::FlipY, "V flip y", 0.0F, 1},
        {Action::Color, "C orange", 12.0F, 1},
        {Action::Alpha, "A 100%", 0.0F, 1},
        {Action::OpNone, "0 off", 0.0F, 2},
        {Action::OpGray, "G grayscale", 0.0F, 2},
        {Action::OpInvert, "I invert", 0.0F, 2},
        {Action::OpTint, "T tint", 0.0F, 2},
        {Action::Pad, "P pad rows", 16.0F, 2},
        {Action::Bug, "B width*4 bug", 0.0F, 2},
    }};
    int lastRow = -1;
    float x = 0.0F;
    for (const Def& d : defs) {
        if (d.row != lastRow) {
            lastRow = d.row;
            x = 112.0F;
        } else {
            x += 8.0F + d.gapBefore;
        }
        const float w = app.hud.width(d.sizing, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({d.action, SDL_FRect{x, kChipRows[static_cast<std::size_t>(d.row)], w, kChipH}});
        x += w;
    }
}

// ---- Drawing helpers -----------------------------------------------------------------------------------------------

void put(App& app, float x, float y, const std::string& s, gd::Color c = gd::kInk, gd::Hud::Style style = gd::Hud::Style::Regular) { app.hud.text(app.renderer, x, y, s, c, style); }

void putRight(App& app, float right, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) { put(app, right - app.hud.width(s, style), y, s, c, style); }

void panel(App& app, float x, float y, float w, float h, const char* title, const std::string& right = {}, gd::Color rightColor = gd::kMuted) {
    gd::card(app.renderer, x, y, w, h, gd::kWhite, gd::kInk);
    put(app, x + 12.0F, y + 6.0F, title, gd::kTealDark, gd::Hud::Style::Bold);
    if (!right.empty()) putRight(app, x + w - 12.0F, y + 6.0F, right, rightColor);
}

std::string fmt(const char* format, auto... args) {
    char buf[96];
    std::snprintf(buf, sizeof buf, format, args...);
    return buf;
}

void checker(SDL_Renderer* r, float x, float y, float size, float cell) {
    gd::fillRect(r, x, y, size, size, gd::kWhite);
    const int n = static_cast<int>(size / cell);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            if ((i + j) % 2 == 1) gd::fillRect(r, x + static_cast<float>(i) * cell, y + static_cast<float>(j) * cell, cell, cell, gd::kGrid);
        }
    }
}

SDL_FRect boxRect(float col) { return SDL_FRect{col + kBoxInset, kBoxTop, kBox, kBox}; }

/// Where the image sits inside its box: centered, zoom times its size.
SDL_FRect imageRect(const App& app, float col) {
    const SDL_FRect box = boxRect(col);
    const float w = static_cast<float>(app.base->w * zoom(app));
    const float h = static_cast<float>(app.base->h * zoom(app));
    return SDL_FRect{box.x + (box.w - w) / 2.0F, box.y + (box.h - h) / 2.0F, w, h};
}

/// The image pixel under a screen point (after zoom and flips), if any.
std::optional<std::pair<int, int>> pixelAt(const App& app, float col, float mx, float my) {
    if (app.base == nullptr) return std::nullopt;
    const SDL_FRect r = imageRect(app, col);
    const SDL_FPoint p{mx, my};
    if (!SDL_PointInRectFloat(&p, &r)) return std::nullopt;
    int x = static_cast<int>((mx - r.x) / static_cast<float>(zoom(app)));
    int y = static_cast<int>((my - r.y) / static_cast<float>(zoom(app)));
    x = std::clamp(x, 0, app.base->w - 1);
    y = std::clamp(y, 0, app.base->h - 1);
    return std::make_pair(app.flipX ? app.base->w - 1 - x : x, app.flipY ? app.base->h - 1 - y : y);
}

void drawImage(App& app, SDL_Texture* tex, float col, bool modulate) {
    SDL_Renderer* r = app.renderer;
    const SDL_FRect box = boxRect(col);
    checker(r, box.x, box.y, kBox, 16.0F);
    if (tex != nullptr && app.base != nullptr) {
        const SDL_FRect dst = imageRect(app, col);
        const gd::Color m = kMods[static_cast<std::size_t>(app.mod)].second;
        SDL_SetTextureScaleMode(tex, app.scale);
        SDL_SetTextureColorMod(tex, modulate ? m.r : 255, modulate ? m.g : 255, modulate ? m.b : 255);
        SDL_SetTextureAlphaMod(tex, modulate ? static_cast<Uint8>(kAlphas[static_cast<std::size_t>(app.alpha)]) : 255);
        const int flip = (app.flipX ? SDL_FLIP_HORIZONTAL : 0) | (app.flipY ? SDL_FLIP_VERTICAL : 0);
        SDL_RenderTextureRotated(r, tex, nullptr, &dst, 0.0, nullptr, static_cast<SDL_FlipMode>(flip));

        // Marker on the inspected pixel.
        const float z = static_cast<float>(zoom(app));
        const float ux = static_cast<float>(app.flipX ? app.base->w - 1 - app.px : app.px);
        const float uy = static_cast<float>(app.flipY ? app.base->h - 1 - app.py : app.py);
        const float cx = dst.x + ux * z + z / 2.0F;
        const float cy = dst.y + uy * z + z / 2.0F;
        const float half = std::max(z / 2.0F + 1.0F, 5.0F);
        gd::strokeRect(r, cx - half - 1.0F, cy - half - 1.0F, 2.0F * half + 2.0F, 2.0F * half + 2.0F, gd::kInk);
        gd::strokeRect(r, cx - half, cy - half, 2.0F * half, 2.0F * half, gd::kOrange);
    }
    gd::strokeRect(r, box.x - 1.0F, box.y - 1.0F, kBox + 2.0F, kBox + 2.0F, gd::kInk);
}

// ---- Panels --------------------------------------------------------------------------------------------------------

void drawFilePanel(App& app) {
    const float x = kColA + 12.0F;
    const float y = kPanelTop;
    panel(app, kColA, y, kColW, kPanelHalf, "1  FILE BYTES", fmt("%zu B", app.file.size()));

    bool sigOk = true;
    for (std::size_t i = 0; i < imgl::kSignature.size(); ++i) {
        const bool have = i < app.file.size();
        const bool ok = have && app.file[i] == imgl::kSignature[i];
        sigOk = sigOk && ok;
        put(app, x + static_cast<float>(i) * 30.0F, y + 30.0F, have ? fmt("%02X", app.file[i]) : "..", ok ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    }
    putRight(app, kColA + kColW - 12.0F, y + 30.0F, sigOk ? "ok" : "BAD", sigOk ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);

    put(app, x, y + 52.0F, "type     len  crc", gd::kMuted);
    const auto& chunks = app.parsed.chunks;
    for (std::size_t i = 0; i < chunks.size() && i < 3; ++i) {
        const imgl::Chunk& c = chunks[i];
        const float ry = y + 74.0F + 22.0F * static_cast<float>(i);
        put(app, x, ry, fmt("%.4s %6u  %08X", c.type.data(), c.length, c.crcStored));
        putRight(app, kColA + kColW - 12.0F, ry, c.crcOk() ? "ok" : "BAD", c.crcOk() ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    }
    if (chunks.size() > 3) put(app, x, y + 118.0F, fmt("... +%zu more", chunks.size() - 3), gd::kMuted);

    if (app.parsed.header) {
        const imgl::Header& h = *app.parsed.header;
        put(app, x, y + 140.0F, fmt("%ux%u  %u-bit  %s", h.width, h.height, h.bitDepth, imgl::colorTypeName(h.colorType)), gd::kInk, gd::Hud::Style::Bold);
    } else {
        put(app, x, y + 140.0F, "no IHDR", gd::kOrange, gd::Hud::Style::Bold);
    }

    std::string status;
    gd::Color color = gd::kOrange;
    switch (app.parsed.error) {
        case imgl::Error::BadSignature: status = "bad signature"; break;
        case imgl::Error::Truncated: status = fmt("cut off at byte %zu", app.parsed.errorOffset); break;
        case imgl::Error::NoHeader: status = "first chunk is no IHDR"; break;
        case imgl::Error::None:
            if (app.parsed.allCrcOk()) {
                status = "all CRCs ok";
                color = gd::kTeal;
            } else {
                for (const imgl::Chunk& c : chunks) {
                    if (!c.crcOk()) {
                        status = fmt("%.4s CRC should be %08X", c.type.data(), c.crcComputed);
                        break;
                    }
                }
            }
            break;
    }
    put(app, x, y + 162.0F, status, color);
}

void drawSurfacePanel(App& app) {
    const float x = kColA + 12.0F;
    const float y = kPanelTop + kPanelHalf + kPanelGap;
    const bool ok = app.work != nullptr;
    panel(app, kColA, y, kColW, kPanelHalf, "2  SURFACE (RAM)", ok ? "load ok" : "FAILED", ok ? gd::kTeal : gd::kOrange);
    if (!ok) {
        put(app, x, y + 30.0F, "SDL_LoadPNG_IO failed:", gd::kOrange, gd::Hud::Style::Bold);
        std::string rest = app.loadError;
        for (int line = 0; line < 4 && !rest.empty(); ++line) {
            std::size_t cut = std::min<std::size_t>(rest.size(), 27);
            if (cut < rest.size()) {
                const std::size_t space = rest.rfind(' ', cut);
                if (space != std::string::npos && space > 0) cut = space;
            }
            put(app, x, y + 52.0F + 22.0F * static_cast<float>(line), rest.substr(0, cut), gd::kInk);
            rest = rest.substr(std::min(rest.size(), cut + (rest[cut] == ' ' ? 1 : 0)));
        }
        return;
    }
    const SDL_Surface* s = app.work;
    put(app, x, y + 28.0F, SDL_GetPixelFormatName(s->format), gd::kInk, gd::Hud::Style::Bold);
    const bool padded = s->pitch != s->w * 4;
    put(app, x, y + 50.0F, fmt("%dx%d  4 B/px  pitch ", s->w, s->h), gd::kInk);
    put(app, x + app.hud.width(fmt("%dx%d  4 B/px  pitch ", s->w, s->h)), y + 50.0F, fmt("%d", s->pitch), padded ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
    put(app, x, y + 72.0F, "memory order " + app.order + (app.converted ? " (converted)" : ""), gd::kInk);

    // One row of memory: the pixels, then the padding the pitch leaves at the end.
    const float barW = kColW - 24.0F;
    const float pixelsW = barW * static_cast<float>(s->w * 4) / static_cast<float>(s->pitch);
    gd::fillRect(app.renderer, x, y + 98.0F, pixelsW, 12.0F, gd::kTeal);
    if (padded) gd::fillRect(app.renderer, x + pixelsW, y + 98.0F, barW - pixelsW, 12.0F, gd::kOrange);
    gd::strokeRect(app.renderer, x, y + 98.0F, barW, 12.0F, gd::kInk);

    const int off = static_cast<int>(imgl::pixelOffset(app.px, app.py, s->pitch, 4));
    put(app, x, y + 118.0F, fmt("(%d,%d) %d*%d+%d*4=%d", app.px, app.py, app.py, s->pitch, app.px, off), gd::kInk);
    const auto* bytes = static_cast<const Uint8*>(s->pixels) + off;
    float bx = x;
    for (int i = 0; i < 4; ++i) {
        const std::string t = fmt("%c %02X", app.order[static_cast<std::size_t>(i)], bytes[i]);
        put(app, bx, y + 140.0F, t, gd::kInk, gd::Hud::Style::Bold);
        bx += app.hud.width(t, gd::Hud::Style::Bold) + 12.0F;
    }
    Uint8 r = 0;
    Uint8 g = 0;
    Uint8 b = 0;
    Uint8 a = 0;
    SDL_ReadSurfacePixel(app.work, app.px, app.py, &r, &g, &b, &a);
    put(app, x, y + 160.0F, fmt("SDL: %d %d %d %d", r, g, b, a), gd::kMuted);
    checker(app.renderer, kColA + kColW - 12.0F - 24.0F, y + 150.0F, 24.0F, 8.0F);
    gd::fillRect(app.renderer, kColA + kColW - 12.0F - 24.0F, y + 150.0F, 24.0F, 24.0F, gd::Color{r, g, b, a});
    gd::strokeRect(app.renderer, kColA + kColW - 12.0F - 24.0F, y + 150.0F, 24.0F, 24.0F, gd::kInk);
}

void drawTexturePanel(App& app) {
    const float x = kColB + 12.0F;
    panel(app, kColB, kPanelTop, kColW, kPanelFull, "3  TEXTURE (GPU)");
    drawImage(app, app.texOriginal, kColB, true);
    if (app.base == nullptr) {
        gd::fillRect(app.renderer, boxRect(kColB).x + 8.0F, kBoxTop + 108.0F, kBox - 16.0F, 32.0F, gd::kWhite);
        put(app, boxRect(kColB).x + 18.0F, kBoxTop + 114.0F, "no surface, no texture", gd::kOrange, gd::Hud::Style::Bold);
        return;
    }
    const char* scaleName = app.scale == SDL_SCALEMODE_NEAREST ? "nearest" : "linear";
    const char* flipName = app.flipX && app.flipY ? "x+y" : app.flipX ? "x" : app.flipY ? "y" : "none";
    const float ly = kBoxTop + kBox + 8.0F;
    put(app, x, ly, fmt("scale %s  flip %s", scaleName, flipName), gd::kInk);
    put(app, x, ly + 22.0F, fmt("%dx%d x %d = %dx%d", app.base->w, app.base->h, zoom(app), app.base->w * zoom(app), app.base->h * zoom(app)), gd::kInk);
    const gd::Color m = kMods[static_cast<std::size_t>(app.mod)].second;
    put(app, x, ly + 44.0F, fmt("mod %d %d %d  a %d", m.r, m.g, m.b, kAlphas[static_cast<std::size_t>(app.alpha)]), gd::kInk);
}

void drawCpuPanel(App& app) {
    const float x = kColC + 12.0F;
    panel(app, kColC, kPanelTop, kColW, kPanelFull, "4  CPU PIXEL LOOP");
    drawImage(app, app.texResult, kColC, false);
    if (app.base == nullptr) {
        gd::fillRect(app.renderer, boxRect(kColC).x + 8.0F, kBoxTop + 108.0F, kBox - 16.0F, 32.0F, gd::kWhite);
        put(app, boxRect(kColC).x + 18.0F, kBoxTop + 114.0F, "nothing to loop over", gd::kOrange, gd::Hud::Style::Bold);
        return;
    }
    const char* names[] = {"off, plain copy", "grayscale", "invert", "tint"};
    const float ly = kBoxTop + kBox + 8.0F;
    put(app, x, ly, std::string("op: ") + names[static_cast<int>(app.op)], gd::kInk);
    if (app.usPerRun >= 0.0) {
        put(app, x, ly + 22.0F, fmt("%d px: %.1f us (avg %d)", app.work->w * app.work->h, app.usPerRun, kRuns), gd::kInk);
    } else {
        put(app, x, ly + 22.0F, "no loop, new texture only", gd::kMuted);
    }
    const int tight = app.work->w * 4;
    if (app.bug && app.work->pitch != tight) {
        put(app, x, ly + 44.0F, fmt("stride w*4 = %d  BUG", tight), gd::kOrange, gd::Hud::Style::Bold);
    } else if (app.bug) {
        put(app, x, ly + 44.0F, "w*4 = pitch, bug hidden", gd::kMuted);
    } else {
        put(app, x, ly + 44.0F, fmt("stride pitch = %d", app.work->pitch), gd::kTealDark);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ IMAGE LOADING ]", "click an image: inspect a pixel, wheel: zoom");
    drawFilePanel(app);
    drawSurfacePanel(app);
    drawTexturePanel(app);
    drawCpuPanel(app);

    const std::array<const char*, 3> rowNames{"file:", "texture:", "cpu:"};
    for (std::size_t i = 0; i < rowNames.size(); ++i) put(app, 20.0F, kChipRows[i] + 4.0F, rowNames[i], gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color fill = !on ? gd::kWhite : c.action == Action::Bug ? gd::kOrange : gd::kTeal;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        const std::string text = label(app, c.action);
        put(app, c.rect.x + (c.rect.w - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, text, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

bool selectPixelAt(App& app, float mx, float my) {
    for (const float col : {kColB, kColC}) {
        if (const auto p = pixelAt(app, col, mx, my)) {
            app.px = p->first;
            app.py = p->second;
            return true;
        }
    }
    return false;
}

void stepZoom(App& app, int dir) { app.zoomIndex = std::clamp(app.zoomIndex + dir, 0, static_cast<int>(kZooms.size()) - 1); }

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) {
            app->maxFrames = std::atol(argv[i + 1]);
        }
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("image-loading", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    app->original.assign(std::begin(asset::kGemPng), std::end(asset::kGemPng));
    layoutChips(*app);
    if (!rebuild(*app) || app->work == nullptr) {
        SDL_Log("the embedded PNG did not load: %s", app->loadError.c_str());
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect) && !apply(app, c.action)) return SDL_APP_FAILURE;
            }
            app.picking = selectPixelAt(app, p.x, p.y);
            break;
        }
        case SDL_EVENT_MOUSE_MOTION:
            if (app.picking) selectPixelAt(app, event->motion.x, event->motion.y);
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP: app.picking = false; break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (event->wheel.y != 0.0F) stepZoom(app, event->wheel.y > 0.0F ? 1 : -1);
            break;
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.repeat) break;
            std::optional<Action> a;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: a = Action::DamageNone; break;
                case SDLK_2: a = Action::DamageSignature; break;
                case SDLK_3: a = Action::DamageCrc; break;
                case SDLK_4: a = Action::DamageCutOff; break;
                case SDLK_N: a = Action::Nearest; break;
                case SDLK_L: a = Action::Linear; break;
                case SDLK_H: a = Action::FlipX; break;
                case SDLK_V: a = Action::FlipY; break;
                case SDLK_C: a = Action::Color; break;
                case SDLK_A: a = Action::Alpha; break;
                case SDLK_0: a = Action::OpNone; break;
                case SDLK_G: a = Action::OpGray; break;
                case SDLK_I: a = Action::OpInvert; break;
                case SDLK_T: a = Action::OpTint; break;
                case SDLK_P: a = Action::Pad; break;
                case SDLK_B: a = Action::Bug; break;
                case SDLK_EQUALS:
                case SDLK_PLUS:
                case SDLK_KP_PLUS: stepZoom(app, 1); break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS: stepZoom(app, -1); break;
                default: break;
            }
            if (a && !apply(app, *a)) return SDL_APP_FAILURE;
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        release(*app);
        app->hud.destroy();
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
