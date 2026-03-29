// =============================================================================
//  Exercise 01 — Parts Counter
// =============================================================================
//
//  Virtual hardware:
//    SW 0        →  io.digital_read(0)        Inductive sensor input
//    Display     →  io.write_reg(6, …)        LCD debug (see README for format)
//                   io.write_reg(7, …)
//
//  Goal:
//    Count every part that passes the sensor and show the total on the display.
//
//  Read README.md before starting.
//
//  ------------------ BRIEF ------------------------------------------
//  By EngJordanFloriano
//
// I choose a polling-based solution with a controlled time window because,
// for this problem, the priority is not only detect an edge, but also
// interpret in the correct and robust way the full part passing cycle.
//
// By sampling the sensor every 1 ms, I keep enough sensitivity to not lose
// short transitions, and at the same time I can apply a simple and reliable
// state logic to guarantee only one count per part, even in cases of
// prolonged signal, noise, or parts passing very close one to another.
//
// The use of PIECE_GAP_MS is a practical way to separate consecutive events,
// while MAX_PIECE_MS adds one more protection layer against field problems,
// like a sensor stuck in high level.
//
// An ISR would also be a technically valid alternative, and in many systems
// it is a very good choice, but here I preferred an approach more predictable,
// easier to validate and maintain, and fully enough for the functional
// requirement without adding unnecessary complexity.
// =============================================================================
#include <trac_fw_io.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{
    // --Constants
    constexpr uint8_t SENSOR_PIN = 0;

    constexpr uint32_t SAMPLE_MS = 1;       // keep 1 ms to no lose short transition
    constexpr uint32_t PIECE_GAP_MS = 70;   // If use a higher value, will not be able to detect two parts that are close together
    constexpr uint32_t MAX_PIECE_MS = 2500; // protection if signal get stuck high
    constexpr uint32_t MIN_PIECE_MS = 5; 

    // ---LCD
    static void lcdWriteU32(trac_fw_io_t &io, int regLo, int regHi, uint32_t value)
    {
        char bufDisplay[9] = {};
        std::snprintf(bufDisplay, sizeof(bufDisplay), "%8u", value);

        uint32_t a = 0;
        uint32_t b = 0;

        std::memcpy(&a, bufDisplay + 0, 4);
        std::memcpy(&b, bufDisplay + 4, 4);

        io.write_reg(regLo, a);
        io.write_reg(regHi, b);
    }

    // I always use this method to avoid overflow
    static bool timePassed(uint32_t now, uint32_t target)
    {
        return static_cast<int32_t>(now - target) >= 0;
    }

} // namespace

int main()
{
    trac_fw_io_t io;

    uint32_t count = 0;

    lcdWriteU32(io, 6, 7, count);

    bool candidate_piece = false;

    uint32_t nextSample = io.millis();
    uint32_t pieceStart = 0;
    uint32_t lastHigh = 0;

    while (true)
    {
        uint32_t now = io.millis();

        if (timePassed(now, nextSample))
        {
            do
            {
                nextSample += SAMPLE_MS;
            } while (timePassed(now, nextSample));

            bool sensor = io.digital_read(SENSOR_PIN);

            if (!candidate_piece)
            {
                if (sensor)
                {
                    candidate_piece = true;
                    pieceStart = now;
                    lastHigh = now;
                }
            }
            else
            {
                if (sensor)
                {
                    lastHigh = now;
                }

                bool finishedByGap = (!sensor && (now - lastHigh) >= PIECE_GAP_MS);
                bool finishedByTime = ((now - pieceStart) >= MAX_PIECE_MS);

                if ( finishedByGap || finishedByTime)
                {
                    uint32_t activeTime = lastHigh - pieceStart;

                    if ( activeTime >= MIN_PIECE_MS)
                    {
                        count++;
                        lcdWriteU32(io, 6, 7, count);
                    }

                    candidate_piece = false;
                }
            }
        }
        io.delay(1);
    }

    return 0;
}