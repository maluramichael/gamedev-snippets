// Tests for the image-loading logic: CRC32, the PNG chunk parser, pixel offsets, byte order and the CPU pixel loop.
// Plain asserts on hand-computed cases, no framework, no SDL. The reference CRCs and the 71 byte PNG below were
// produced independently with Python's zlib, they are not computed by the code under test.
#undef NDEBUG
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "asset.hpp"
#include "logic.hpp"

using namespace imgl;

namespace {

// A 2x1 RGBA image (red, then half transparent blue) written with zlib: signature, IHDR, IDAT, IEND.
const std::vector<std::uint8_t> kTiny{
    0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,                                                  // signature, bytes 0..7
    0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,  // IHDR at 8: 2x1
    0x08, 0x06, 0x00, 0x00, 0x00, 0xF4, 0x22, 0x7F, 0x8A,                                            // 8 bit, RGBA, crc F4227F8A
    0x00, 0x00, 0x00, 0x0E, 0x49, 0x44, 0x41, 0x54, 0x78, 0xDA, 0x63, 0xF8, 0xCF, 0xC0, 0x00, 0x42,  // IDAT at 33: 14 bytes
    0x0D, 0x00, 0x0F, 0x7A, 0x03, 0x7E, 0x6A, 0x81, 0x31, 0xE1,                                      // crc 6A8131E1
    0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82};                          // IEND at 59

std::vector<std::uint8_t> embedded() { return {std::begin(asset::kGemPng), std::end(asset::kGemPng)}; }

bool same(const Chunk& c, const char* type) { return std::memcmp(c.type.data(), type, 4) == 0; }

void crc32KnownValues() {
    const char* check = "123456789";  // the standard check value of CRC-32 (zlib, PNG)
    assert(crc32(reinterpret_cast<const std::uint8_t*>(check), 9) == 0xCBF43926u);
    assert(crc32(nullptr, 0) == 0u);
    // The CRC of a PNG chunk covers type and data, IEND has no data: the famous AE426082.
    const std::array<std::uint8_t, 4> iend{'I', 'E', 'N', 'D'};
    assert(crc32(iend.data(), iend.size()) == 0xAE426082u);
    // A single zero byte, computed with zlib.
    const std::uint8_t zero = 0;
    assert(crc32(&zero, 1) == 0xD202EF8Du);
}

void readsBigEndian() {
    const std::array<std::uint8_t, 4> b{0x00, 0x00, 0x01, 0x02};
    assert(readBe32(b.data()) == 258u);
    const std::array<std::uint8_t, 4> c{0xFF, 0x00, 0x00, 0x01};
    assert(readBe32(c.data()) == 0xFF000001u);
}

void parsesTheTinyPng() {
    const Parsed p = parse(kTiny);
    assert(p.error == Error::None);
    assert(p.chunks.size() == 3);
    assert(same(p.chunks[0], "IHDR") && p.chunks[0].length == 13 && p.chunks[0].offset == 8);
    assert(same(p.chunks[1], "IDAT") && p.chunks[1].length == 14 && p.chunks[1].offset == 33);
    assert(same(p.chunks[2], "IEND") && p.chunks[2].length == 0 && p.chunks[2].offset == 59);
    assert(p.chunks[0].crcStored == 0xF4227F8Au && p.chunks[0].crcComputed == 0xF4227F8Au);
    assert(p.chunks[1].crcStored == 0x6A8131E1u && p.chunks[1].crcComputed == 0x6A8131E1u);
    assert(p.chunks[2].crcStored == 0xAE426082u);
    assert(p.allCrcOk());
    assert(p.header.has_value());
    assert(p.header->width == 2 && p.header->height == 1);
    assert(p.header->bitDepth == 8 && p.header->colorType == 6 && p.header->interlace == 0);
    // Every chunk takes 12 bytes plus its data, the signature 8: that has to add up to the file size.
    std::size_t total = 8;
    for (const Chunk& c : p.chunks) total += 12 + c.length;
    assert(total == kTiny.size());
}

void parsesTheEmbeddedFile() {
    const std::vector<std::uint8_t> file = embedded();
    const Parsed p = parse(file);
    assert(p.error == Error::None);
    assert(p.allCrcOk());
    assert(p.header.has_value());
    assert(p.header->width == 64 && p.header->height == 64);
    assert(p.header->bitDepth == 8 && p.header->colorType == 6 && p.header->interlace == 0);
    assert(p.chunks.size() >= 3);
    assert(same(p.chunks.front(), "IHDR") && same(p.chunks.back(), "IEND"));
    assert(p.chunks.back().crcStored == 0xAE426082u);
    std::size_t total = 8;
    std::size_t idat = 0;
    for (const Chunk& c : p.chunks) {
        total += 12 + c.length;
        if (same(c, "IDAT")) idat += c.length;
    }
    assert(total == file.size());
    assert(idat > 0);
}

void wrongSignature() {
    std::vector<std::uint8_t> file = kTiny;
    file[1] = 'Q';
    const Parsed p = parse(file);
    assert(p.error == Error::BadSignature);
    assert(p.chunks.empty());
    assert(!p.header.has_value());
    assert(!hasSignature(file));
    assert(hasSignature(kTiny));
    // The old-school transfer damage: a line ending converted (0D 0A became 0A) shifts everything.
    std::vector<std::uint8_t> crlf = kTiny;
    crlf.erase(crlf.begin() + 4);
    assert(parse(crlf).error == Error::BadSignature);
}

void truncatedFiles() {
    // Fewer than eight bytes cannot even hold the signature.
    assert(parse(std::span<const std::uint8_t>(kTiny.data(), 0)).error == Error::Truncated);
    assert(parse(std::span<const std::uint8_t>(kTiny.data(), 5)).error == Error::Truncated);
    // Signature only: no chunk at all, and no IEND.
    Parsed p = parse(std::span<const std::uint8_t>(kTiny.data(), 8));
    assert(p.error == Error::Truncated && p.chunks.empty());
    // Cut inside the IHDR data.
    p = parse(std::span<const std::uint8_t>(kTiny.data(), 20));
    assert(p.error == Error::Truncated && p.chunks.empty());
    assert(p.errorOffset == 8);
    // Cut inside the IDAT data: IHDR was fine and is kept.
    p = parse(std::span<const std::uint8_t>(kTiny.data(), 45));
    assert(p.error == Error::Truncated && p.chunks.size() == 1);
    assert(p.errorOffset == 33);
    assert(p.header.has_value());
    // Cut one byte short of the IDAT's CRC.
    p = parse(std::span<const std::uint8_t>(kTiny.data(), 58));
    assert(p.error == Error::Truncated && p.chunks.size() == 1);
    // Cut exactly after the IDAT: every chunk is intact, but IEND is missing.
    p = parse(std::span<const std::uint8_t>(kTiny.data(), 59));
    assert(p.error == Error::Truncated && p.chunks.size() == 2 && p.allCrcOk());
    assert(p.errorOffset == 59);
    // The same for the real file, cut in the middle of its IDAT.
    const std::vector<std::uint8_t> file = embedded();
    p = parse(std::span<const std::uint8_t>(file.data(), file.size() / 2));
    assert(p.error == Error::Truncated);
}

void hugeLengthDoesNotOverflow() {
    std::vector<std::uint8_t> file = kTiny;
    file[33] = 0xFF;  // IDAT length 0xFF00000E: far beyond the file, must not wrap around on 32 bit
    const Parsed p = parse(file);
    assert(p.error == Error::Truncated);
    assert(p.chunks.size() == 1);
}

void badCrcIsReportedPerChunk() {
    std::vector<std::uint8_t> file = kTiny;
    file[58] ^= 0x01;  // last byte of the IDAT's stored CRC
    Parsed p = parse(file);
    assert(p.error == Error::None);  // structurally fine, the parser keeps going
    assert(p.chunks.size() == 3);
    assert(p.chunks[0].crcOk());
    assert(!p.chunks[1].crcOk());
    assert(p.chunks[1].crcStored == 0x6A8131E0u && p.chunks[1].crcComputed == 0x6A8131E1u);
    assert(p.chunks[2].crcOk());
    assert(!p.allCrcOk());
    // A damaged data byte shows up the same way: the stored CRC no longer matches the data.
    file = kTiny;
    file[41 + 3] ^= 0x10;
    p = parse(file);
    assert(p.chunks.size() == 3 && !p.chunks[1].crcOk() && p.chunks[0].crcOk());
}

void headerMustComeFirst() {
    // IDAT first, IHDR second: structurally readable, but there is no header.
    std::vector<std::uint8_t> file(kTiny.begin(), kTiny.begin() + 8);
    file.insert(file.end(), kTiny.begin() + 33, kTiny.begin() + 59);  // the IDAT chunk
    file.insert(file.end(), kTiny.begin() + 8, kTiny.begin() + 33);   // the IHDR chunk
    file.insert(file.end(), kTiny.begin() + 59, kTiny.end());         // IEND
    const Parsed p = parse(file);
    assert(p.error == Error::NoHeader);
    assert(!p.header.has_value());
    // An IHDR that is not 13 bytes long is no header either.
    file = kTiny;
    file[11] = 0x0C;
    assert(!parse(file).header.has_value());
}

void colorTypeNames() {
    assert(std::string(colorTypeName(0)) == "gray");
    assert(std::string(colorTypeName(2)) == "RGB");
    assert(std::string(colorTypeName(3)) == "palette");
    assert(std::string(colorTypeName(4)) == "gray+alpha");
    assert(std::string(colorTypeName(6)) == "RGBA");
    assert(std::string(colorTypeName(5)) == "invalid");
}

void damageHelper() {
    const std::vector<std::uint8_t> file = embedded();
    assert(damaged(file, Damage::None) == file);

    const std::vector<std::uint8_t> sig = damaged(file, Damage::Signature);
    assert(sig.size() == file.size() && parse(sig).error == Error::BadSignature);

    const std::vector<std::uint8_t> crc = damaged(file, Damage::Crc);
    const Parsed pc = parse(crc);
    assert(crc.size() == file.size() && pc.error == Error::None && !pc.allCrcOk());
    int bad = 0;
    for (const Chunk& c : pc.chunks) bad += c.crcOk() ? 0 : 1;
    assert(bad == 1);  // exactly one chunk, the first IDAT

    const std::vector<std::uint8_t> cut = damaged(file, Damage::CutOff);
    assert(cut.size() < file.size() && parse(cut).error == Error::Truncated);
}

void pixelOffsetUsesThePitch() {
    // Tight rows: pitch == width * 4.
    assert(pixelOffset(0, 0, 256, 4) == 0);
    assert(pixelOffset(1, 0, 256, 4) == 4);
    assert(pixelOffset(0, 1, 256, 4) == 256);
    assert(pixelOffset(63, 63, 256, 4) == 63u * 256 + 63u * 4);
    // Padded rows: a row is 280 bytes although the pixels take 256. Pixel (3,2) is NOT at 2*256 + 3*4.
    assert(pixelOffset(3, 2, 280, 4) == 2u * 280 + 12);
    assert(pixelOffset(3, 2, 280, 4) != pixelOffset(3, 2, 256, 4));
    // Three bytes per pixel (RGB24) with 4 byte aligned rows: width 5 -> 15 bytes of pixels, pitch 16.
    assert(pixelOffset(4, 1, 16, 3) == 16u + 12);
}

void byteOrder() {
    // SDL_PIXELFORMAT_RGBA32 on a little endian machine is ABGR8888: the masks look reversed, the memory is R G B A.
    ChannelBytes abgr = channelBytes(0x000000FFu, 0x0000FF00u, 0x00FF0000u, 0xFF000000u, true);
    assert(abgr.r == 0 && abgr.g == 1 && abgr.b == 2 && abgr.a == 3);
    assert(memoryOrder(abgr) == "RGBA");
    // ARGB8888 (the format of many windows and of SDL_PIXELFORMAT_ARGB8888) is B G R A in memory on little endian.
    ChannelBytes argb = channelBytes(0x00FF0000u, 0x0000FF00u, 0x000000FFu, 0xFF000000u, true);
    assert(argb.r == 2 && argb.g == 1 && argb.b == 0 && argb.a == 3);
    assert(memoryOrder(argb) == "BGRA");
    // The same ARGB8888 masks on a big endian machine put A first.
    ChannelBytes be = channelBytes(0x00FF0000u, 0x0000FF00u, 0x000000FFu, 0xFF000000u, false);
    assert(be.a == 0 && be.r == 1 && be.g == 2 && be.b == 3);
    assert(memoryOrder(be) == "ARGB");
    // No alpha channel (XRGB8888): the fourth byte is padding.
    ChannelBytes xrgb = channelBytes(0x00FF0000u, 0x0000FF00u, 0x000000FFu, 0u, true);
    assert(xrgb.a == -1 && memoryOrder(xrgb) == "BGR.");
    assert(is8888(0x000000FFu, 0x0000FF00u, 0x00FF0000u, 0xFF000000u));
    assert(!is8888(0x3FF00000u, 0x000FFC00u, 0x000003FFu, 0xC0000000u));  // 2:10:10:10
    assert(!is8888(0xF800u, 0x07E0u, 0x001Fu, 0u));                       // RGB565
}

void grayscaleFormula() {
    // Rec. 601 weights 0.299 / 0.587 / 0.114 as integers over 256: 77 + 150 + 29 = 256, so white stays white.
    assert(luma(255, 0, 0) == 76);   // 77 * 255 / 256 = 76.7
    assert(luma(0, 255, 0) == 149);  // 150 * 255 / 256 = 149.4
    assert(luma(0, 0, 255) == 28);   // 29 * 255 / 256 = 28.9
    assert(luma(255, 255, 255) == 255);
    assert(luma(0, 0, 0) == 0);
    assert(luma(100, 100, 100) == 100);  // gray stays gray, no rounding drift
    assert(luma(13, 148, 136) == (77 * 13 + 150 * 148 + 29 * 136) / 256);
}

// A 3x2 RGBA image with the given pitch. Padding bytes are 0xCD so a stray write is visible.
std::vector<std::uint8_t> image3x2(int pitch) {
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(pitch) * 2, 0xCD);
    const std::array<std::array<std::uint8_t, 4>, 6> px{{{255, 0, 0, 255}, {0, 255, 0, 255}, {0, 0, 255, 255}, {10, 20, 30, 40}, {200, 100, 50, 0}, {255, 255, 255, 128}}};
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 3; ++x) {
            std::memcpy(&buf[pixelOffset(x, y, pitch, 4)], px[static_cast<std::size_t>(y * 3 + x)].data(), 4);
        }
    }
    return buf;
}

void grayscaleLoop() {
    const ChannelBytes rgba{0, 1, 2, 3};
    const std::vector<std::uint8_t> src = image3x2(12);
    std::vector<std::uint8_t> dst = src;
    process(src.data(), dst.data(), 3, 2, 12, Op::Grayscale, rgba, false);
    assert(dst[0] == 76 && dst[1] == 76 && dst[2] == 76 && dst[3] == 255);  // red keeps its alpha
    assert(dst[4] == 149 && dst[8] == 28);
    const std::size_t p3 = pixelOffset(0, 1, 12, 4);
    assert(dst[p3] == luma(10, 20, 30) && dst[p3 + 3] == 40);  // alpha is never touched
    assert(dst[pixelOffset(1, 1, 12, 4) + 3] == 0);
}

void grayscaleRespectsChannelOrder() {
    // The same red pixel stored as B G R A: the loop must still find the red byte at index 2.
    const ChannelBytes bgra{2, 1, 0, 3};
    const std::vector<std::uint8_t> src{0, 0, 255, 255};
    std::vector<std::uint8_t> dst = src;
    process(src.data(), dst.data(), 1, 1, 4, Op::Grayscale, bgra, false);
    assert(dst[0] == 76 && dst[1] == 76 && dst[2] == 76 && dst[3] == 255);
}

void invertAndTint() {
    const ChannelBytes rgba{0, 1, 2, 3};
    const std::vector<std::uint8_t> src{255, 0, 100, 77};
    std::vector<std::uint8_t> dst = src;
    process(src.data(), dst.data(), 1, 1, 4, Op::Invert, rgba, false);
    assert(dst[0] == 0 && dst[1] == 255 && dst[2] == 155 && dst[3] == 77);
    // Tint is the average with the tint color (234, 88, 12), integer division.
    process(src.data(), dst.data(), 1, 1, 4, Op::Tint, rgba, false);
    assert(dst[0] == (255 + 234) / 2 && dst[1] == (0 + 88) / 2 && dst[2] == (100 + 12) / 2 && dst[3] == 77);
    dst = src;
    process(src.data(), dst.data(), 1, 1, 4, Op::None, rgba, false);
    assert(dst == src);  // None leaves the destination alone
}

void loopHonoursPitch() {
    const ChannelBytes rgba{0, 1, 2, 3};
    const int pitch = 16;  // 3 pixels = 12 bytes, 4 bytes of padding per row
    const std::vector<std::uint8_t> src = image3x2(pitch);
    std::vector<std::uint8_t> dst = src;
    process(src.data(), dst.data(), 3, 2, pitch, Op::Invert, rgba, false);
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 3; ++x) {
            const std::size_t o = pixelOffset(x, y, pitch, 4);
            assert(dst[o] == 255 - src[o] && dst[o + 1] == 255 - src[o + 1] && dst[o + 2] == 255 - src[o + 2] && dst[o + 3] == src[o + 3]);
        }
        for (int k = 12; k < 16; ++k) assert(dst[static_cast<std::size_t>(y * pitch + k)] == 0xCD);  // padding untouched
    }
}

void widthTimesFourBug() {
    const ChannelBytes rgba{0, 1, 2, 3};
    const int pitch = 16;
    const std::vector<std::uint8_t> src = image3x2(pitch);
    std::vector<std::uint8_t> dst = src;
    process(src.data(), dst.data(), 3, 2, pitch, Op::Invert, rgba, true);
    // Row 0 is fine. Row 1 really starts at y * pitch = 16, but the loop starts it at y * width * 4 = 12: it inverts the
    // padding of row 0 (4 bytes of 0xCD) and then everything one pixel too early ...
    assert(dst[12] == 255 - 0xCD);
    // ... so the last real pixel of row 1 (at 24) is never reached and keeps its old colors.
    assert(dst[24] == src[24] && src[24] != 255 - src[24]);
    // With tight rows the bug is invisible: both loops give the same bytes.
    const std::vector<std::uint8_t> tight = image3x2(12);
    std::vector<std::uint8_t> a = tight;
    std::vector<std::uint8_t> b = tight;
    process(tight.data(), a.data(), 3, 2, 12, Op::Grayscale, rgba, false);
    process(tight.data(), b.data(), 3, 2, 12, Op::Grayscale, rgba, true);
    assert(a == b);
}

void copyWithPitchKeepsPixels() {
    const std::vector<std::uint8_t> tight = image3x2(12);
    const std::vector<std::uint8_t> padded = copyWithPitch(tight.data(), 3 * 4, 2, 12, 24, 0xAB);
    assert(padded.size() == 48);
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 3; ++x) {
            const std::size_t a = pixelOffset(x, y, 12, 4);
            const std::size_t b = pixelOffset(x, y, 24, 4);
            assert(std::memcmp(&tight[a], &padded[b], 4) == 0);
        }
        assert(padded[static_cast<std::size_t>(y * 24 + 12)] == 0xAB && padded[static_cast<std::size_t>(y * 24 + 23)] == 0xAB);
    }
    // Copying back down to tight rows gives the original again.
    assert(copyWithPitch(padded.data(), 12, 2, 24, 12, 0) == tight);
}

}  // namespace

int main() {
    crc32KnownValues();
    readsBigEndian();
    parsesTheTinyPng();
    parsesTheEmbeddedFile();
    wrongSignature();
    truncatedFiles();
    hugeLengthDoesNotOverflow();
    badCrcIsReportedPerChunk();
    headerMustComeFirst();
    colorTypeNames();
    damageHelper();
    pixelOffsetUsesThePitch();
    byteOrder();
    grayscaleFormula();
    grayscaleLoop();
    grayscaleRespectsChannelOrder();
    invertAndTint();
    loopHonoursPitch();
    widthTimesFourBug();
    copyWithPitchKeepsPixels();
    std::puts("image-loading logic: all asserts passed");
    return 0;
}
