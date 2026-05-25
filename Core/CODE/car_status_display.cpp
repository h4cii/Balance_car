#include "car_status_display.hpp"

#include <stdio.h>

#include "car_control.hpp"

namespace car {

StatusDisplay status_display;

void StatusDisplay::init()
{
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(20U);

    cmd(0xAEU);
    cmd(0x20U);
    cmd(0x02U);
    cmd(0xB0U);
    cmd(0xC8U);
    cmd(0x00U);
    cmd(0x10U);
    cmd(0x40U);
    cmd(0x81U);
    cmd(0x7FU);
    cmd(0xA1U);
    cmd(0xA6U);
    cmd(0xA8U);
    cmd(0x3FU);
    cmd(0xA4U);
    cmd(0xD3U);
    cmd(0x00U);
    cmd(0xD5U);
    cmd(0x80U);
    cmd(0xD9U);
    cmd(0xF1U);
    cmd(0xDAU);
    cmd(0x12U);
    cmd(0xDBU);
    cmd(0x40U);
    cmd(0x8DU);
    cmd(0x14U);
    cmd(0xAFU);

    clear();
    ready_ = true;
}

void StatusDisplay::update()
{
    char line[24];
    char value[12];
    const uint32_t now = HAL_GetTick();

    if (!ready_ || ((now - last_ms_) < 200U)) {
        return;
    }
    last_ms_ = now;

    const BalanceState s = controller.state();

    snprintf(line, sizeof(line), "B570 LQR %s", s.enabled ? "RUN" : "STOP");
    writeText(0U, line);

    formatTenths(value, sizeof(value), (long)(s.pitch_deg * 10.0f));
    snprintf(line, sizeof(line), "ANG %s DEG", value);
    writeText(2U, line);

    snprintf(line, sizeof(line), "BAT %u.%02uV",
             (unsigned)(s.battery_centivolts / 100U),
             (unsigned)(s.battery_centivolts % 100U));
    writeText(4U, line);

    snprintf(line, sizeof(line), "PWM %d %d", (int)s.pwm_left, (int)s.pwm_right);
    writeText(6U, line);
}

void StatusDisplay::scl(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, state);
}

void StatusDisplay::sda(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, state);
}

void StatusDisplay::dc(GPIO_PinState state)
{
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, state);
}

void StatusDisplay::writeByte(uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        scl(GPIO_PIN_RESET);
        sda((value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        value <<= 1U;
        scl(GPIO_PIN_SET);
    }
    scl(GPIO_PIN_RESET);
}

void StatusDisplay::cmd(uint8_t command)
{
    dc(GPIO_PIN_RESET);
    writeByte(command);
}

void StatusDisplay::data(uint8_t value)
{
    dc(GPIO_PIN_SET);
    writeByte(value);
}

void StatusDisplay::setPos(uint8_t page, uint8_t col)
{
    cmd((uint8_t)(0xB0U + page));
    cmd((uint8_t)(0x00U + (col & 0x0FU)));
    cmd((uint8_t)(0x10U + ((col >> 4U) & 0x0FU)));
}

void StatusDisplay::clear()
{
    for (uint8_t page = 0U; page < 8U; ++page) {
        setPos(page, 0U);
        for (uint8_t col = 0U; col < 128U; ++col) {
            data(0x00U);
        }
    }
}

const uint8_t *StatusDisplay::glyphFor(char c)
{
    static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t zero[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t one[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t two[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const uint8_t three[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const uint8_t four[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t five[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const uint8_t six[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const uint8_t seven[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t eight[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t nine[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
    static const uint8_t a[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t b[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t d[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t e[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t f[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
    static const uint8_t g[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
    static const uint8_t i[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t k[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const uint8_t l[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t m[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const uint8_t n[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
    static const uint8_t o[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t p[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E};
    static const uint8_t r[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t s[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t t[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t u[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t v[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const uint8_t w[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F};
    static const uint8_t y[5] = {0x07, 0x08, 0x70, 0x08, 0x07};

    if ((c >= 'a') && (c <= 'z')) {
        c = (char)(c - ('a' - 'A'));
    }

    switch (c) {
    case ' ': return blank;
    case '-': return dash;
    case '.': return dot;
    case ':': return colon;
    case '0': return zero;
    case '1': return one;
    case '2': return two;
    case '3': return three;
    case '4': return four;
    case '5': return five;
    case '6': return six;
    case '7': return seven;
    case '8': return eight;
    case '9': return nine;
    case 'A': return a;
    case 'B': return b;
    case 'D': return d;
    case 'E': return e;
    case 'F': return f;
    case 'G': return g;
    case 'I': return i;
    case 'K': return k;
    case 'L': return l;
    case 'M': return m;
    case 'N': return n;
    case 'O': return o;
    case 'P': return p;
    case 'Q': return q;
    case 'R': return r;
    case 'S': return s;
    case 'T': return t;
    case 'U': return u;
    case 'V': return v;
    case 'W': return w;
    case 'Y': return y;
    default: return blank;
    }
}

void StatusDisplay::writeText(uint8_t page, const char *text)
{
    uint8_t col = 0U;

    setPos(page, 0U);
    while ((*text != '\0') && (col < 126U)) {
        const uint8_t *glyph = glyphFor(*text++);
        for (uint8_t i = 0U; i < 5U; ++i) {
            data(glyph[i]);
        }
        data(0x00U);
        col = (uint8_t)(col + 6U);
    }
    while (col < 128U) {
        data(0x00U);
        col++;
    }
}

void StatusDisplay::formatTenths(char *buf, unsigned int len, long value10)
{
    const char *sign = "";

    if (value10 < 0) {
        sign = "-";
        value10 = -value10;
    }
    const long whole = value10 / 10L;
    const long frac = value10 % 10L;
    snprintf(buf, len, "%s%ld.%ld", sign, whole, frac);
}

}  // namespace car
