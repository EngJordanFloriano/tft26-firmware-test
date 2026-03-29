// =============================================================================
//  Exercise 03 — I2C Sensors (Bit-bang)
// =============================================================================
//
//  Virtual hardware:
//    P8 (SCL)  →  io.digital_write(8, …) / io.digital_read(8)
//    P9 (SDA)  →  io.digital_write(9, …) / io.digital_read(9)
//
//  PART 1 — TMP64 temperature sensor at I2C address 0x48
//    Register 0x0F  WHO_AM_I   — 1 byte  (expected: 0xA5)
//    Register 0x00  TEMP_RAW   — 4 bytes, big-endian int32_t, milli-Celsius
//
//  PART 2 — Unknown humidity sensor (same register layout, address unknown)
//    Register 0x0F  WHO_AM_I   — 1 byte
//    Register 0x00  HUM_RAW    — 4 bytes, big-endian int32_t, milli-percent
//
//  Goal (Part 1):
//    1. Implement an I2C master via bit-bang on P8/P9.
//    2. Read WHO_AM_I from TMP64 and confirm the sensor is present.
//    3. Read TEMP_RAW in a loop and print the temperature in °C every second.
//    4. Update display registers 6–7 with the formatted temperature string.
//
//  Goal (Part 2):
//    5. Scan the I2C bus (addresses 0x08–0x77) and print every responding address.
//    6. For each unknown device found, read its WHO_AM_I and print it.
//    7. Add the humidity sensor to the 1 Hz loop: read HUM_RAW and print %RH.
//
//  Read README.md before starting.
//
//  ------------------ BRIEF ------------------------------------------
//  By EngJordanFloriano
//
// In this code, the main points of the solution were the manual
// implementation of the I2C protocol by bit-bang and the clear separation
// between sensor discovery, reading, validation and display. As the problem
// requires access to one known sensor and another one with unknown address,
// the technique used was first validate the TMP64 by WHO_AM_I at the fixed
// address and then do a full bus scan, checking ACK on each address and
// reading WHO_AM_I from the devices found to identify the humidity sensor.
// The I2C communication was implemented with explicit control of start,
// repeated start, stop, bit write, bit read and ACK/NACK handling, which
// gives full control of the transaction and makes debug easier. Another
// important point was validating the measurements before publish: temperature
// and humidity are only accepted if they are inside the expected ranges,
// avoiding read failure, noise or even a wrong device generating invalid
// values on the display. The scan ambiguity was also handled: if more than
// one unknown candidate is found, the code does not assume one by itself and
// signals error, making the behavior safer and more consistent with the
// problem.
// =============================================================================
#include <trac_fw_io.hpp>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{
// -- Constants
constexpr uint8_t SCL_PIN = 8;
constexpr uint8_t SDA_PIN = 9;

constexpr uint8_t TEMP_ADDR = 0x48;
constexpr uint8_t REG_WHO = 0x0F;
constexpr uint8_t REG_DATA = 0x00;

constexpr uint8_t TEMP_WHO_OK = 0xA5;

constexpr uint8_t SCAN_START = 0x08;
constexpr uint8_t SCAN_END = 0x77;

constexpr uint32_t LOOP_MS = 1000;

// datasheet range - TMP64: -40.000 C to +125.000 C
constexpr int32_t TEMP_MIN = -40000;
constexpr int32_t TEMP_MAX = 125000;

// datasheet range - HMD10: 0.000 %RH to 100.000 %RH
constexpr int32_t HUM_MIN = 0;
constexpr int32_t HUM_MAX = 100000;

// --Structs
struct HumiditySensor
{
    bool found = false;
    uint8_t addr = 0;
    uint8_t who = 0;
};

// -- Utils
//I always use this method to avoid overflow
static bool timePassed(uint32_t now, uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

static int32_t bigEndianToI32(const std::array<uint8_t, 4>& bytes)
{
    uint32_t raw = 0;
    raw |= (static_cast<uint32_t>(bytes[0]) << 24);
    raw |= (static_cast<uint32_t>(bytes[1]) << 16);
    raw |= (static_cast<uint32_t>(bytes[2]) << 8);
    raw |= (static_cast<uint32_t>(bytes[3]) << 0);
    return static_cast<int32_t>(raw);
}

// -- I2C bit-bang
class BitBangI2C
{
public:
    explicit BitBangI2C(trac_fw_io_t& io) : io(io) {}

    void begin()
    {
        io.set_pullup(SCL_PIN, true);
        io.set_pullup(SDA_PIN, true);

        io.digital_write(SCL_PIN, 1);
        io.digital_write(SDA_PIN, 1);
    }

    // check wich address respond with ACK, after this try read who am i
    bool checkResponseAck(uint8_t addr)
    {
        if (!busIdle()) {
            return false;
        }

        start();
        bool ack = writeByte(static_cast<uint8_t>((addr << 1) | 0));
        stop();
        return ack;
    }

    bool readReg(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n)
    {
        if (out == nullptr || n == 0) {
            return false;
        }

        if (!busIdle()) {
            return false;
        }

        start();

        if (!writeByte(static_cast<uint8_t>((addr << 1) | 0))) {
            stop();
            return false;
        }

        if (!writeByte(reg)) {
            stop();
            return false;
        }

        // repeated start
        start();

        if (!writeByte(static_cast<uint8_t>((addr << 1) | 1))) {
            stop();
            return false;
        }

        for (uint8_t i = 0; i < n; i++) {
            bool ack = (i < static_cast<uint8_t>(n - 1));
            if (!readByte(out[i], ack)) {
                stop();
                return false;
            }
        }

        stop();
        return true;
    }

private:
    trac_fw_io_t& io;

    bool busIdle()
    {
        io.digital_write(SDA_PIN, 1);
        io.digital_write(SCL_PIN, 1);
        return io.digital_read(SDA_PIN) && io.digital_read(SCL_PIN);
    }

    void sclLow() { io.digital_write(SCL_PIN, 0); }
    void sclHigh() { io.digital_write(SCL_PIN, 1); }
    void sdaLow() { io.digital_write(SDA_PIN, 0); }
    void sdaHigh() { io.digital_write(SDA_PIN, 1); }

    bool sdaRead()
    {
        return io.digital_read(SDA_PIN);
    }

    void start()
    {
        sdaHigh();
        sclHigh();
        sdaLow();
        sclLow();
    }

    void stop()
    {
        sdaLow();
        sclHigh();
        sdaHigh();
    }

    void writeBit(bool bit)
    {
        if (bit) {
            sdaHigh();
        } else {
            sdaLow();
        }

        sclHigh();
        sclLow();
    }

    bool readBit(bool& bit)
    {
        sdaHigh();
        sclHigh();
        bit = sdaRead();
        sclLow();
        return true;
    }

    bool writeByte(uint8_t value)
    {
        uint8_t mask = 0x80;

        while (mask != 0) {
            if ((value & mask) != 0) {
                writeBit(true);
            } else {
                writeBit(false);
            }
            mask >>= 1;
        }

        bool ack = true;
        if (!readBit(ack)) {
            return false;
        }

        return !ack;
    }

    bool readByte(uint8_t& value, bool ack)
    {
        value = 0;
        uint8_t mask = 0x80;

        while (mask != 0) {
            bool bit = false;
            if (!readBit(bit)) {
                return false;
            }

            if (bit) {
                value |= mask;
            }

            mask >>= 1;
        }

        // ACK for intermediate bytes, NACK for last byte
        if (ack) {
            sdaLow();
        } else {
            sdaHigh();
        }

        sclHigh();
        sclLow();
        sdaHigh();

        return true;
    }
};

// -- LCD
static void lcdWriteLine(trac_fw_io_t& io, int regLo, int regHi, const char* buf)
{
    uint32_t a = 0;
    uint32_t b = 0;

    std::memcpy(&a, buf + 0, 4);
    std::memcpy(&b, buf + 4, 4);

    io.write_reg(regLo, a);
    io.write_reg(regHi, b);
}

//Format temperature for the LCD display registers.
static void lcdShowTemp(trac_fw_io_t& io, float value)
{
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%7.3fC", value);
    lcdWriteLine(io, 6, 7, buf);
}
//Format humidity for the LCD display registers.
static void lcdShowHum(trac_fw_io_t& io, float value)
{
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%7.3f%%", value);
    lcdWriteLine(io, 4, 5, buf);
}

// Show ERR on the lines  when the sensor read is invalid.
static void lcdShowTempErr(trac_fw_io_t& io)
{
    char buf[8] = {' ', ' ', ' ', ' ', ' ', 'E', 'R', 'R'};
    lcdWriteLine(io, 6, 7, buf);
}

static void lcdShowHumErr(trac_fw_io_t& io)
{
    char buf[8] = {' ', ' ', ' ', ' ', ' ', 'E', 'R', 'R'};
    lcdWriteLine(io, 4, 5, buf);
}

// --- Sensor utils
static bool readWho(BitBangI2C& i2c, uint8_t addr, uint8_t& who)
{
    return i2c.readReg(addr, REG_WHO, &who, 1);
}

static bool read32(BitBangI2C& i2c, uint8_t addr, int32_t& out)
{
    std::array<uint8_t, 4> raw{};
    if (!i2c.readReg(addr, REG_DATA, raw.data(), static_cast<uint8_t>(raw.size()))) {
        return false;
    }

    out = bigEndianToI32(raw);
    return true;
}

static HumiditySensor scanHumiditySensors(BitBangI2C& i2c)
{
    HumiditySensor dev{};

    std::printf("I2C scan (%02X-%02X)\n", SCAN_START, SCAN_END);

    int nonTmpCount = 0;
    uint8_t lastAddr = 0;
    uint8_t lastWho = 0;

    for (uint8_t addr = SCAN_START; addr <= SCAN_END; ++addr) {
        if (!i2c.checkResponseAck(addr)) {
            continue;
        }

        uint8_t who = 0;
        bool whoOk = readWho(i2c, addr, who);

        if (whoOk) {
            std::printf("  ACK at 0x%02X WHO_AM_I=0x%02X\n", addr, who);
        } else {
            std::printf("  ACK at 0x%02X WHO_AM_I read failed\n", addr);
        }

        if (addr != TEMP_ADDR) {
            nonTmpCount++;
            lastAddr = addr;
            lastWho = who;
        }
    }


    //According to the challenge statement, there will only be two sensors,
    // one for temperature and one for humidity, with an unknown address.
    // Even so, I will continue scanning and if there is more than one, 
    //I will treat it as an error, since I don't have information on how to handle these cases.  

    if (nonTmpCount == 1) {
        dev.found = true;
        dev.addr = lastAddr;
        dev.who = lastWho;
    } else if (nonTmpCount == 0) {
        std::printf("humidity sensor not found\n");
    } else {
        std::printf("humidity sensor ambiguous: %d candidates\n", nonTmpCount);
    }

    return dev;
}

// -- Aplication layer
static void publishTemp(trac_fw_io_t& io, BitBangI2C& i2c)
{
    int32_t temp = 0;
    if (!read32(i2c, TEMP_ADDR, temp)) {
        std::printf("TMP64 temperature: read failed\n");
        lcdShowTempErr(io);
        return;
    }

    // Validate data before print and updating the display
    if (temp < TEMP_MIN || temp > TEMP_MAX) {
        std::printf("TMP64 temperature: out off range raw=%ld mC\n", static_cast<long>(temp));
        lcdShowTempErr(io);
        return;
    }

    float tempC = static_cast<float>(temp) / 1000.0f;
    std::printf("TMP64 temperature:\t %8.3f C\n", tempC);
    lcdShowTemp(io, tempC);
}

static void publishHum(trac_fw_io_t& io, BitBangI2C& i2c, const HumiditySensor& humSensor)
{
    if (!humSensor.found) {
        lcdShowHumErr(io);
        return;
    }

    int32_t humidity = 0;
    if (!read32(i2c, humSensor.addr, humidity)) {
        std::printf("HMD10 humidity @0x%02X: read failed\n", humSensor.addr);
        lcdShowHumErr(io);
        return;
    }

        // Validate data before print and updating the display
    if (humidity < HUM_MIN || humidity > HUM_MAX) {
        std::printf("HMD10 humidity @0x%02X: out of range raw=%ld m%%RH\n",
                    humSensor.addr, static_cast<long>(humidity));
        lcdShowHumErr(io);
        return;
    }

    float humValue = static_cast<float>(humidity) / 1000.0f;
    std::printf("HMD10 humidity @0x%02X:\t %7.3f %%RH\n\r\n", humSensor.addr, humValue);
    lcdShowHum(io, humValue);
}

} // namespace

int main()
{
    trac_fw_io_t io;
    BitBangI2C i2c(io);
    i2c.begin();

    uint8_t who = 0;
    if (readWho(i2c, TEMP_ADDR, who)) {
        std::printf("TMP64 WHO_AM_I: 0x%02X (%s)\n",
                    who,
                    (who == TEMP_WHO_OK) ? "OK" : "UNEXPECTED");
    } else {
        std::printf("TMP64 WHO_AM_I: read failed\n");
        lcdShowTempErr(io);
    }

    HumiditySensor humSensor = scanHumiditySensors(i2c);

    // If only one humidity sensor was found on the bus, use it in the rest of the code
    if (humSensor.found) {
        std::printf("Humidity sensor selected at 0x%02X, WHO_AM_I=0x%02X\n\r\n",
                    humSensor.addr, humSensor.who);
    } else {
        //If no sensor is found, or if more than one is found, an error will be displayed
        lcdShowHumErr(io);
    }

    uint32_t next = io.millis();
    while (true) {
        uint32_t now = io.millis();

        if (timePassed(now, next)) {
            do {
                next += LOOP_MS; //to avoid drift
            } while (timePassed(now, next));

            publishTemp(io, i2c);
            publishHum(io, i2c, humSensor);
        }

        io.delay(1);
    }
}