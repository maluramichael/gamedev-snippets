// The presets of the demo: a clock, a diode, an OR gate, an XOR gate and a half adder, as ASCII art (see logic.hpp for the
// legend). The geometry is found and checked by simulation, test.cpp holds the truth tables. Nothing here is special to
// the engine: the gates are wire cells and nothing else.
//
// How the gates work, in one paragraph: a wire fires on 1 or 2 head neighbours and stays dark on 3 or more. The XOR
// gate lets both inputs meet at cells that then see too many heads, so only a single electron gets through. A diode is
// a wire with a hole that a pair of side cells bridges, and an electron coming from the wrong side arrives there as a
// crowd. The half adder gets its carry from (A or B) xor SUM: with both inputs on, the OR gives 1 and the SUM gives 0.
#pragma once

#include <cstddef>
#include <vector>

#include "logic.hpp"

namespace ww {

inline unsigned expectDiode(unsigned in) { return in & 1U; }  // bit 0: FWD wire passes, bit 1: REV wire stays dark
inline unsigned expectOr(unsigned in) { return in != 0U ? 1U : 0U; }
inline unsigned expectXor(unsigned in) { return ((in & 1U) ^ ((in >> 1U) & 1U)); }
inline unsigned expectHalfAdder(unsigned in) {
    const unsigned a = in & 1U;
    const unsigned b = (in >> 1U) & 1U;
    return (a ^ b) | ((a & b) << 1U);  // bit 0 SUM, bit 1 CARRY
}

inline const std::vector<Circuit>& circuits() {
    static const std::vector<Circuit> all = [] {
        std::vector<Circuit> v;

        {
            Circuit c;
            c.id = "clock";
            c.label = "Clock";
            c.title = "CLOCK";
            c.notes = {"One electron runs", "round the ring, the", "stub taps it"};
            c.art = {
                "     1      ",
                "     #      ",
                "     #      ",
                "     #      ",
                "     #      ",
                "     #      ",
                "  ########  ",
                " #        # ",
                " #        # ",
                " #        # ",
                "  #####HT#  ",
            };
            c.outputNames = {"OUT"};
            v.push_back(c);
        }
        {
            Circuit c;
            c.id = "diode";
            c.label = "Diode";
            c.title = "DIODE";
            c.notes = {"A feeds both wires.", "The lower diode is", "turned around."};
            c.art = {
                "                  ##",
                "a################## ################1",
                "                  ##",
                "",
                "",
                "                  ##",
                "c################# #################2",
                "                  ##",
            };
            c.inputs = {{"A", "ac"}};
            c.outputNames = {"FWD", "REV"};
            c.expect = expectDiode;
            c.period = 48;
            c.startBits = 1;
            v.push_back(c);
        }
        {
            Circuit c;
            c.id = "or";
            c.label = "OR";
            c.title = "OR GATE";
            c.notes = {"Click a row or use", "keys A and B"};
            c.art = {
                "      ##",
                "a###### ###",
                "      ##   #",
                "            #",
                "             #################1",
                "            #",
                "      ##   #",
                "b###### ###",
                "      ##",
            };
            c.inputs = {{"A", "a"}, {"B", "b"}};
            c.outputNames = {"OR"};
            c.expect = expectOr;
            c.period = 44;
            c.startBits = 1;
            v.push_back(c);
        }
        {
            Circuit c;
            c.id = "xor";
            c.label = "XOR";
            c.title = "XOR GATE";
            c.notes = {"Click a row or use", "keys A and B"};
            c.art = {
                " a############ ###",
                "             ##   ################1",
                " b############",
            };
            c.inputs = {{"A", "a"}, {"B", "b"}};
            c.outputNames = {"XOR"};
            c.expect = expectXor;
            c.period = 48;
            c.startBits = 1;
            v.push_back(c);
        }
        {
            Circuit c;
            c.id = "halfadder";
            c.label = "Half adder";
            c.title = "HALF ADDER";
            c.notes = {"SUM = A xor B", "CARRY = (A or B)", "xor SUM"};
            c.art = {
                "  a####### ###",
                "         ##   ######################1",
                "  b#######        #",
                "                  #",
                "                  # ##",
                "                  ### ###### ###",
                "                    ##     ##   ####2",
                "      ##               #####",
                "c###### ###           #",
                "      ##   #         #",
                "            #       #",
                "             #######",
                "            #",
                "      ##   #",
                "d###### ###",
                "      ##",
            };
            c.inputs = {{"A", "ac"}, {"B", "bd"}};
            c.outputNames = {"SUM", "CARRY"};
            c.expect = expectHalfAdder;
            c.period = 44;
            c.startBits = 3;
            v.push_back(c);
        }
        return v;
    }();
    return all;
}

inline constexpr std::size_t kStartPreset = 4;  // the half adder

}  // namespace ww
