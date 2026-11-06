// The testable part of the image-loading demo, no SDL: CRC32, a PNG chunk reader, pixel offsets with a pitch,
// the byte order of a 32-bit pixel format and the CPU pixel loop (grayscale, invert, tint).
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace imgl {

inline constexpr std::array<std::uint8_t, 8> kSignature{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

/// CRC-32 as PNG (and zlib) use it: reflected, polynomial 0xEDB88320, start and end value all ones.
inline std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    static const std::array<std::uint32_t, 256> table = [] {
        std::array<std::uint32_t, 256> t{};
        for (std::uint32_t n = 0; n < 256; ++n) {
            std::uint32_t c = n;
            for (int k = 0; k < 8; ++k) c = (c & 1u) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[n] = c;
        }
        return t;
    }();
    std::uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) c = table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

/// PNG stores every number big endian: the most significant byte comes first.
inline std::uint32_t readBe32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) | (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}

inline bool hasSignature(std::span<const std::uint8_t> file) { return file.size() >= kSignature.size() && std::equal(kSignature.begin(), kSignature.end(), file.begin()); }

struct Chunk {
    std::array<char, 4> type{};
    std::uint32_t length = 0;  // data bytes, without the 12 bytes of length, type and CRC
    std::size_t offset = 0;    // where the length field starts in the file
    std::uint32_t crcStored = 0;
    std::uint32_t crcComputed = 0;
    bool crcOk() const { return crcStored == crcComputed; }
};

struct Header {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint8_t bitDepth = 0;
    std::uint8_t colorType = 0;
    std::uint8_t compression = 0;
    std::uint8_t filter = 0;
    std::uint8_t interlace = 0;
};

enum class Error { None, BadSignature, Truncated, NoHeader };

struct Parsed {
    Error error = Error::None;
    std::size_t errorOffset = 0;  // Truncated: where the chunk that does not fit (or the missing IEND) starts
    std::vector<Chunk> chunks;    // every complete chunk, in file order
    std::optional<Header> header;
    bool allCrcOk() const {
        return std::all_of(chunks.begin(), chunks.end(), [](const Chunk& c) { return c.crcOk(); });
    }
};

/// Reads the chunk list of a PNG file without decompressing anything. A wrong CRC is reported per chunk and does
/// not stop the reader, a chunk that does not fit into the file does (Truncated).
inline Parsed parse(std::span<const std::uint8_t> file) {
    Parsed out;
    if (file.size() < kSignature.size()) {
        out.error = Error::Truncated;
        out.errorOffset = file.size();
        return out;
    }
    if (!hasSignature(file)) {
        out.error = Error::BadSignature;
        return out;
    }
    std::size_t offset = kSignature.size();
    bool sawIend = false;
    while (offset < file.size()) {
        const std::size_t left = file.size() - offset;
        // Length, type and CRC take 12 bytes. The comparison runs on size_t, so a length of 4 GB cannot wrap around.
        if (left < 12 || readBe32(&file[offset]) > left - 12) {
            out.error = Error::Truncated;
            out.errorOffset = offset;
            break;
        }
        Chunk c;
        c.length = readBe32(&file[offset]);
        std::memcpy(c.type.data(), &file[offset + 4], 4);
        c.offset = offset;
        c.crcStored = readBe32(&file[offset + 8 + c.length]);
        c.crcComputed = crc32(&file[offset + 4], 4 + static_cast<std::size_t>(c.length));  // the CRC covers type and data
        out.chunks.push_back(c);
        offset += 12 + static_cast<std::size_t>(c.length);
        if (std::memcmp(c.type.data(), "IEND", 4) == 0) {
            sawIend = true;
            break;
        }
    }
    if (out.error == Error::None && !sawIend) {
        out.error = Error::Truncated;
        out.errorOffset = offset;
    }
    if (!out.chunks.empty() && std::memcmp(out.chunks.front().type.data(), "IHDR", 4) == 0 && out.chunks.front().length == 13) {
        const std::uint8_t* d = &file[out.chunks.front().offset + 8];
        out.header = Header{readBe32(d), readBe32(d + 4), d[8], d[9], d[10], d[11], d[12]};
    }
    if (out.error == Error::None && !out.header) out.error = Error::NoHeader;
    return out;
}

inline const char* colorTypeName(int colorType) {
    switch (colorType) {
        case 0: return "gray";
        case 2: return "RGB";
        case 3: return "palette";
        case 4: return "gray+alpha";
        case 6: return "RGBA";
        default: return "invalid";
    }
}

/// Three ways to break a file for the demo: a wrong signature byte, a wrong CRC in the first IDAT, a file that ends in the middle of it.
enum class Damage { None, Signature, Crc, CutOff };

inline std::vector<std::uint8_t> damaged(std::span<const std::uint8_t> file, Damage how) {
    std::vector<std::uint8_t> out(file.begin(), file.end());
    if (how == Damage::None) return out;
    if (how == Damage::Signature) {
        if (out.size() > 1) out[1] ^= 0x01;  // 'P' becomes 'Q'
        return out;
    }
    for (const Chunk& c : parse(file).chunks) {
        if (std::memcmp(c.type.data(), "IDAT", 4) != 0) continue;
        if (how == Damage::Crc) {
            out[c.offset + 8 + c.length + 3] ^= 0x01;  // last byte of the stored CRC, the data stays valid
        } else {
            out.resize(c.offset + 8 + c.length / 2);
        }
        break;
    }
    return out;
}

/// Where pixel (x, y) starts in memory. Rows are `pitch` bytes apart, which may be more than width * bytesPerPixel.
inline std::size_t pixelOffset(int x, int y, int pitch, int bytesPerPixel) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch) + static_cast<std::size_t>(x) * static_cast<std::size_t>(bytesPerPixel);
}

/// For each channel the index of its byte inside a 32-bit pixel as it lies in memory, -1 if the format has no such channel.
struct ChannelBytes {
    int r = 0;
    int g = 1;
    int b = 2;
    int a = 3;
};

inline int byteIndex(std::uint32_t mask, bool littleEndian) {
    if (mask == 0) return -1;
    const int byte = std::countr_zero(mask) / 8;
    return littleEndian ? byte : 3 - byte;
}

/// SDL describes a packed format with masks over the 32-bit value. What you see in memory depends on the byte order of the machine.
inline ChannelBytes channelBytes(std::uint32_t rmask, std::uint32_t gmask, std::uint32_t bmask, std::uint32_t amask, bool littleEndian) {
    return ChannelBytes{byteIndex(rmask, littleEndian), byteIndex(gmask, littleEndian), byteIndex(bmask, littleEndian), byteIndex(amask, littleEndian)};
}

/// "RGBA", "BGRA", "ARGB", a dot marks an unused byte.
inline std::string memoryOrder(ChannelBytes c) {
    std::string s(4, '.');
    const std::array<std::pair<int, char>, 4> map{{{c.r, 'R'}, {c.g, 'G'}, {c.b, 'B'}, {c.a, 'A'}}};
    for (const auto& [index, letter] : map) {
        if (index >= 0 && index < 4) s[static_cast<std::size_t>(index)] = letter;
    }
    return s;
}

/// True for a format with four 8-bit channels (or three plus padding) that fill the 32 bits without overlap.
inline bool is8888(std::uint32_t rmask, std::uint32_t gmask, std::uint32_t bmask, std::uint32_t amask) {
    const bool bytes = std::popcount(rmask) == 8 && std::popcount(gmask) == 8 && std::popcount(bmask) == 8 && (std::popcount(amask) == 8 || amask == 0);
    return bytes && (rmask & gmask) == 0 && (rmask & bmask) == 0 && (gmask & bmask) == 0 && ((rmask | gmask | bmask) & amask) == 0;
}

/// Gray from RGB with the Rec. 601 weights 0.299, 0.587, 0.114 as integers over 256 (77 + 150 + 29 = 256).
inline std::uint8_t luma(std::uint8_t r, std::uint8_t g, std::uint8_t b) { return static_cast<std::uint8_t>((77 * r + 150 * g + 29 * b) >> 8); }

enum class Op { None, Grayscale, Invert, Tint };

struct Rgb {
    std::uint8_t r, g, b;
};
inline constexpr Rgb kTintColor{234, 88, 12};  // the orange of the demo, tint = average with this

/// Runs `op` over every pixel of a 32-bit image, reading `src` and writing `dst` (alpha is copied as it is, the rest of dst is not
/// touched). Rows are `pitch` bytes apart. With `widthTimesFour` the loop steps rows by width * 4 instead: the bug the demo shows,
/// harmless as long as pitch == width * 4 and wrong as soon as the rows are padded.
inline void process(const std::uint8_t* src, std::uint8_t* dst, int width, int height, int pitch, Op op, ChannelBytes ch, bool widthTimesFour) {
    if (op == Op::None) return;
    const auto r = static_cast<std::size_t>(ch.r);
    const auto g = static_cast<std::size_t>(ch.g);
    const auto b = static_cast<std::size_t>(ch.b);
    for (int y = 0; y < height; ++y) {
        const std::size_t row = widthTimesFour ? static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 4 : static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch);
        for (int x = 0; x < width; ++x) {
            const std::size_t o = row + static_cast<std::size_t>(x) * 4;
            const std::uint8_t pr = src[o + r];
            const std::uint8_t pg = src[o + g];
            const std::uint8_t pb = src[o + b];
            switch (op) {
                case Op::Grayscale: dst[o + r] = dst[o + g] = dst[o + b] = luma(pr, pg, pb); break;
                case Op::Invert:
                    dst[o + r] = static_cast<std::uint8_t>(255 - pr);
                    dst[o + g] = static_cast<std::uint8_t>(255 - pg);
                    dst[o + b] = static_cast<std::uint8_t>(255 - pb);
                    break;
                case Op::Tint:
                    dst[o + r] = static_cast<std::uint8_t>((pr + kTintColor.r) / 2);
                    dst[o + g] = static_cast<std::uint8_t>((pg + kTintColor.g) / 2);
                    dst[o + b] = static_cast<std::uint8_t>((pb + kTintColor.b) / 2);
                    break;
                case Op::None: break;
            }
        }
    }
}

/// Copies `rows` rows of `rowBytes` pixel bytes into a buffer whose rows are `dstPitch` bytes apart, the gap filled with `fill`.
inline std::vector<std::uint8_t> copyWithPitch(const std::uint8_t* src, int rowBytes, int rows, int srcPitch, int dstPitch, std::uint8_t fill) {
    std::vector<std::uint8_t> out(static_cast<std::size_t>(dstPitch) * static_cast<std::size_t>(rows), fill);
    for (int y = 0; y < rows; ++y) {
        std::memcpy(&out[static_cast<std::size_t>(y) * static_cast<std::size_t>(dstPitch)], src + static_cast<std::size_t>(y) * static_cast<std::size_t>(srcPitch), static_cast<std::size_t>(rowBytes));
    }
    return out;
}

}  // namespace imgl
