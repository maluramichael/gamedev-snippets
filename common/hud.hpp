// Shared look of all demos: JetBrains Mono text (same font as malura.de), teal and orange accents on a
// paper background with a fine grid. Header only, depends on SDL3 only.
//
//   gd::Hud hud;                       // once, after the renderer exists
//   hud.load(renderer);
//   gd::backdrop(renderer, w, h);      // every frame
//   hud.text(renderer, x, y, "hello", gd::kInk);
#pragma once

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <vector>

#include "font_atlas.hpp"

namespace gd {

/// How the 960x540 canvas maps to the drawing buffer. In the browser the page CSS already forces the canvas to 16:9, but the buffer
/// size is rounded to whole pixels (an iframe 403.875 px high is 403 px in its viewport), so a letterboxed canvas would leave a
/// light 1 px strip at the sides. Stretching changes the aspect by less than a tenth of a percent there. Natively the window can
/// be any shape, so it stays letterboxed.
#ifdef __EMSCRIPTEN__
inline constexpr SDL_RendererLogicalPresentation kLogicalPresentation = SDL_LOGICAL_PRESENTATION_STRETCH;
#else
inline constexpr SDL_RendererLogicalPresentation kLogicalPresentation = SDL_LOGICAL_PRESENTATION_LETTERBOX;
#endif

struct Color {
    Uint8 r, g, b, a = 255;
};

inline constexpr Color kTeal{13, 148, 136};
inline constexpr Color kTealDark{15, 118, 110};
inline constexpr Color kTealSoft{204, 251, 241};
inline constexpr Color kOrange{234, 88, 12};
inline constexpr Color kInk{17, 24, 39};
inline constexpr Color kMuted{107, 114, 128};
inline constexpr Color kPaper{250, 250, 249};
inline constexpr Color kGrid{231, 229, 228};
inline constexpr Color kWhite{255, 255, 255};

inline void setColor(SDL_Renderer* r, Color c) { SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a); }

inline SDL_FColor fcolor(Color c, float alpha = 1.0F) {
    return SDL_FColor{c.r / 255.0F, c.g / 255.0F, c.b / 255.0F, c.a / 255.0F * alpha};
}

/// Paper with a 30 px grid, the common background of every demo.
inline void backdrop(SDL_Renderer* r, int w, int h) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    setColor(r, kPaper);
    SDL_RenderClear(r);
    setColor(r, kGrid);
    for (int x = 0; x < w; x += 30) SDL_RenderLine(r, static_cast<float>(x), 0.0F, static_cast<float>(x), static_cast<float>(h));
    for (int y = 0; y < h; y += 30) SDL_RenderLine(r, 0.0F, static_cast<float>(y), static_cast<float>(w), static_cast<float>(y));
}

inline void fillRect(SDL_Renderer* r, float x, float y, float w, float h, Color c) {
    setColor(r, c);
    const SDL_FRect rect{x, y, w, h};
    SDL_RenderFillRect(r, &rect);
}

inline void strokeRect(SDL_Renderer* r, float x, float y, float w, float h, Color c) {
    setColor(r, c);
    const SDL_FRect rect{x, y, w, h};
    SDL_RenderRect(r, &rect);
}

/// Rectangle with a hard offset shadow, the 90s button look malura.de uses.
inline void card(SDL_Renderer* r, float x, float y, float w, float h, Color fill, Color edge) {
    fillRect(r, x + 4.0F, y + 4.0F, w, h, Color{17, 24, 39, 40});
    fillRect(r, x, y, w, h, fill);
    strokeRect(r, x, y, w, h, edge);
}

/// Vertical gradient from `top` to `bottom`.
inline void gradient(SDL_Renderer* r, float x, float y, float w, float h, Color top, Color bottom) {
    const SDL_FColor a = fcolor(top);
    const SDL_FColor b = fcolor(bottom);
    const SDL_Vertex v[4] = {{{x, y}, a, {0, 0}}, {{x + w, y}, a, {1, 0}}, {{x + w, y + h}, b, {1, 1}}, {{x, y + h}, b, {0, 1}}};
    const int idx[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
}

/// Filled circle as a triangle fan; `edge` fades the rim for a soft glow when its alpha is lower than `center`.
inline void disc(SDL_Renderer* r, float cx, float cy, float radius, Color center, Color edge, int segments = 40) {
    std::vector<SDL_Vertex> v;
    std::vector<int> idx;
    v.push_back({{cx, cy}, fcolor(center), {0, 0}});
    for (int i = 0; i <= segments; ++i) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(segments);
        v.push_back({{cx + std::cos(a) * radius, cy + std::sin(a) * radius}, fcolor(edge), {0, 0}});
        if (i > 0) {
            idx.insert(idx.end(), {0, i, i + 1});
        }
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

inline void disc(SDL_Renderer* r, float cx, float cy, float radius, Color c, int segments = 40) { disc(r, cx, cy, radius, c, c, segments); }

inline void ring(SDL_Renderer* r, float cx, float cy, float radius, Color c, int segments = 48) {
    setColor(r, c);
    for (int i = 0; i < segments; ++i) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(segments);
        const float b = 6.2831853F * static_cast<float>(i + 1) / static_cast<float>(segments);
        SDL_RenderLine(r, cx + std::cos(a) * radius, cy + std::sin(a) * radius, cx + std::cos(b) * radius, cy + std::sin(b) * radius);
    }
}

class Hud {
public:
    bool load(SDL_Renderer* renderer) {
        regular_ = make(renderer, font::kRegular16Png, sizeof font::kRegular16Png, font::kRegular16CellW, font::kRegular16CellH);
        bold_ = make(renderer, font::kBold16Png, sizeof font::kBold16Png, font::kBold16CellW, font::kBold16CellH);
        large_ = make(renderer, font::kBold32Png, sizeof font::kBold32Png, font::kBold32CellW, font::kBold32CellH);
        return regular_.texture != nullptr && bold_.texture != nullptr && large_.texture != nullptr;
    }

    void destroy() {
        for (Face* face : {&regular_, &bold_, &large_}) {
            SDL_DestroyTexture(face->texture);
            face->texture = nullptr;
        }
    }

    enum class Style { Regular, Bold, Large };

    float width(std::string_view s, Style style = Style::Regular) const {
        const Face& face = pick(style);
        float n = 0;
        for (std::size_t i = 0; i < s.size(); i += length(s[i])) ++n;
        return n * static_cast<float>(face.cellW);
    }

    float lineHeight(Style style = Style::Regular) const { return static_cast<float>(pick(style).cellH); }

    void text(SDL_Renderer* r, float x, float y, std::string_view s, Color c, Style style = Style::Regular) const {
        const Face& face = pick(style);
        if (face.texture == nullptr) return;
        SDL_SetTextureColorMod(face.texture, c.r, c.g, c.b);
        SDL_SetTextureAlphaMod(face.texture, c.a);
        for (std::size_t i = 0; i < s.size(); i += length(s[i])) {
            const int index = glyphIndex(s, i);
            if (index >= 0) {
                const SDL_FRect src{static_cast<float>((index % font::kColumns) * face.cellW), static_cast<float>((index / font::kColumns) * face.cellH),
                                    static_cast<float>(face.cellW), static_cast<float>(face.cellH)};
                const SDL_FRect dst{std::round(x), std::round(y), static_cast<float>(face.cellW), static_cast<float>(face.cellH)};
                SDL_RenderTexture(r, face.texture, &src, &dst);
            }
            x += static_cast<float>(face.cellW);
        }
    }

    /// Teal strip with a bold title in brackets, like the section headings on malura.de.
    void titleBar(SDL_Renderer* r, int w, std::string_view title, std::string_view right = {}) const {
        fillRect(r, 0, 0, static_cast<float>(w), 30.0F, kTeal);
        text(r, 12.0F, 7.0F, title, kWhite, Style::Bold);
        if (!right.empty()) text(r, static_cast<float>(w) - width(right) - 12.0F, 7.0F, right, kTealSoft);
    }

private:
    struct Face {
        SDL_Texture* texture = nullptr;
        int cellW = 0;
        int cellH = 0;
    };

    Face regular_, bold_, large_;

    const Face& pick(Style style) const { return style == Style::Regular ? regular_ : style == Style::Bold ? bold_ : large_; }

    static Face make(SDL_Renderer* renderer, const unsigned char* png, std::size_t size, int cellW, int cellH) {
        SDL_IOStream* io = SDL_IOFromConstMem(png, size);
        SDL_Surface* surface = io != nullptr ? SDL_LoadPNG_IO(io, true) : nullptr;
        if (surface == nullptr) return {};
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_DestroySurface(surface);
        if (texture != nullptr) {
            SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);
            SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
        }
        return Face{texture, cellW, cellH};
    }

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
        const char32_t cp = decode(s, i);
        const auto it = std::find(charset.begin(), charset.end(), cp);
        return it == charset.end() ? -1 : static_cast<int>(it - charset.begin());
    }
};

}  // namespace gd
