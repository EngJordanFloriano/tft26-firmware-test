// =============================================================================
//  Challenge 02 — Frequency Estimator
// =============================================================================
//
//  Virtual hardware:
//    ADC Ch 0  →  io.analog_read(0)      Process sensor signal (0–4095)
//    OUT reg 3 →  io.write_reg(3, …)     Frequency estimate in centiHz
//                                        e.g. write_reg(3, 4733) = 47.33 Hz
//
//  Goal:
//    Measure the frequency of the signal on ADC channel 0 and publish your
//    estimate continuously via register 3.
//
//  Read README.md before starting.
//
//  ------------------ BRIEF ------------------------------------------
//  By EngJordanFloriano
//
// I chose a light and practical approach because, in embedded systems,
// the goal is not to use the most complex filter, but to get a stable and
// reliable measurement with low processing cost. The raw signal is smoothed
// with a simple filter, while the signal center moves slower to follow offset
// changes without reacting too much to instant oscillations. On top of that,
// an adaptive hysteresis in Schmitt trigger style helps to avoid false edge
// detection caused by noise close to the threshold. The frequency is also
// calculated from multiple periods instead of only one, which makes the
// result more stable and less sensitive to jitter or transient spikes.
// I also used shifts on purpose, because they are a very efficient way to do
// this kind of filtering and adjustment in embedded firmware, keeping the
// code simple, fast, and deterministic.
// =============================================================================
#include <trac_fw_io.hpp>
#include <array>
#include <cstdint>
#include <cmath>

namespace
{
// -- Constants
constexpr uint8_t ADC_CH = 0;
constexpr uint8_t OUT_REG = 3;

constexpr uint32_t SAMPLE_MS = 1;   // 1 kHz

// shift to optimize embedded processing
constexpr int FILTER_SHIFT = 3;       // simple filter, 1/8
constexpr int CENTER_SHIFT = 6;     // Center moves more slowly to avoid instantaneous oscillations.

//A good relationship to avoid noise
constexpr int MIN_HYST = 45;
constexpr int MAX_HYST = 500;

//Range to reject transients
constexpr uint32_t MIN_PERIORD_MS = 4;     // avoid crazy glitch
constexpr uint32_t MAX_PERIOD_MS = 1000;  // 1 Hz, already very low

// -- Utils

//I always use this method to avoid overflow
static bool timePassed(uint32_t now, uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

// --Freq calc
struct FreqCalc
{
    std::array<uint32_t, 8> buf{};
    int n = 0;
    int wr = 0;

    uint32_t lastEdge = 0;
    bool hasLast = false;

    uint32_t lastFreq = 0;

    void addPeriod(uint32_t dt)
    {
        buf[wr] = dt;
        wr++;

        if (wr >= static_cast<int>(buf.size())) {
            wr = 0;
        }

        if (n < static_cast<int>(buf.size())) {
            n++;
        }
    }

    bool getFreq(uint32_t& out)
    {
        if (n < 2) {
            return false; // still low info
        }

        uint32_t sum = 0;
        for (int i = 0; i < n; i++) {
            sum += buf[i];
        }

        if (sum == 0) {
            return false;
        }

        // freq = n periods / sum_of_periods
        // Better than a simple average because it uses the total time across multiple cycles,
        // making the estimate more stable and less sensitive to noise and jitter.


        // in centiHz => *100000
        // This cast is necessary to prevent overflow when multiplying by 100000
        out = static_cast<uint32_t>(((static_cast<uint64_t>(n) * 100000ULL) + (sum / 2)) / sum);
        return true;
    }

    bool onEdge(uint32_t now, uint32_t& out)
    {
        if (!hasLast) {
            hasLast = true;
            lastEdge = now;
            return false;
        }

        uint32_t dt = now - lastEdge;

        // reject some dumb transient
        if (dt < MIN_PERIORD_MS || dt > MAX_PERIOD_MS) {
            return false;
        }

        // use previous freq to not accept too crazy jump
        if (lastFreq > 0) {
            uint32_t prevPeriod = 100000U / lastFreq;
            if (prevPeriod > 0) {
                if (dt < (prevPeriod / 2) || dt > (prevPeriod * 2 + 2)) {
                    // let it change, but not in one shot for random spike
                    return false;
                }
            }
        }

        lastEdge = now;
        addPeriod(dt);

        if (!getFreq(out)) {
            return false;
        }

        lastFreq = out;
        return true;
    }
};

} // namespace

int main()
{
    trac_fw_io_t io;

    uint32_t next = io.millis();

    int filt = 1000;
    int center = 1000;
    int amp = 180; // simple initial guess

    bool started = false;
    bool high = false;

    FreqCalc calc;
    uint32_t published = 0;

    io.write_reg(OUT_REG, 0);

    while (true) {
        uint32_t now = io.millis();

        if (timePassed(now, next)) {
            do {
                next += SAMPLE_MS; //to avoid drift
            } while (timePassed(now, next));

            int raw = static_cast<int>(io.analog_read(ADC_CH));

            if (!started) {
                //fill the initial values
                filt = raw;
                center = raw;
                started = true;
            }

            // simple filter but works ok
            // This is equivalent to -> filt = filt + (raw - filt)/8
            filt += (raw - filt) >> FILTER_SHIFT;

            // signal center moving slower
            center += (filt - center) >> CENTER_SHIFT;

            // Track signal amplitude to keep the hysteresis adaptive.
            int dev = filt - center;
            int absDev = std::abs(dev);
            amp += (absDev - amp) >> 4; // simple damping

            int hyst = amp / 2;
            if (hyst < MIN_HYST) {
                hyst = MIN_HYST;
            }
            if (hyst > MAX_HYST) {
                hyst = MAX_HYST;
            }



            // Simple classic Schmitt trigger to avoid noise around the crossing point.
            int hi = center + hyst;
            int lo = center - hyst;

            bool rising = false;

            if (!high) {
                if (filt >= hi) {
                    high = true;
                    rising = true;
                }
            } else {
                if (filt <= lo) {
                    high = false;
                }
            }

            if (rising) {
                uint32_t freq100 = 0;
                if (calc.onEdge(now, freq100)) {

                    //To avoid excessive changes at regs
                    if (std::abs(static_cast<int>(freq100) - static_cast<int>(published)) >= 3) { 
                        published = freq100;
                        io.write_reg(OUT_REG, published);
                    }
                }
            }
        }

        io.delay(1);
    }
}