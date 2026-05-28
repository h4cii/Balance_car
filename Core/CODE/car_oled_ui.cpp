#include "car_oled_ui.hpp"

#include "main.h"
#include "oled.h"

namespace car {

namespace {

uint16_t abs16(int16_t value)
{
    if (value >= 0) {
        return (uint16_t)value;
    }
    return (uint16_t)(-(int32_t)value);
}

void showSign(u8 x, u8 y, int16_t value)
{
    OLED_ShowString(x, y, (value < 0) ? "-" : "+");
}

void showSignedNumber(u8 sign_x, u8 number_x, u8 y, int16_t value, u8 digits)
{
    showSign(sign_x, y, value);
    OLED_ShowNumber(number_x, y, abs16(value), digits, 12);
}

void showPitch(u8 y, int16_t pitch_deg_x10)
{
    uint16_t value = abs16(pitch_deg_x10);
    if (value > 9999U) {
        value = 9999U;
    }
    showSign(48, y, pitch_deg_x10);
    OLED_ShowNumber(56, y, value / 10U, 3, 12);
    OLED_ShowString(80, y, ".");
    OLED_ShowNumber(88, y, value % 10U, 1, 12);
}

uint8_t batteryPercent(uint16_t centivolts)
{
    if (centivolts == 0U) {
        return 0U;
    }
    if (centivolts <= 1000U) {
        return 5U;
    }
    if (centivolts >= 1260U) {
        return 100U;
    }
    return (uint8_t)(((uint32_t)(centivolts - 1000U) * 95U) / 260U + 5U);
}

void showVoltage(u8 y, uint16_t centivolts)
{
    if (centivolts > 9999U) {
        centivolts = 9999U;
    }
    OLED_ShowString(0, y, "V");
    OLED_ShowString(30, y, ".");
    OLED_ShowString(64, y, "V");
    OLED_ShowNumber(19, y, centivolts / 100U, 2, 12);
    OLED_ShowNumber(42, y, (centivolts / 10U) % 10U, 1, 12);
    OLED_ShowNumber(50, y, centivolts % 10U, 1, 12);
}

void showRunState(u8 x, u8 y, bool enabled)
{
    OLED_ShowString(x, y, enabled ? "ON " : "OFF");
}

void showMpuState(u8 x, u8 y, bool ok)
{
    OLED_ShowString(x, y, ok ? "MPU" : "---");
}

void showSpinner(u8 x, u8 y, uint16_t frame)
{
    switch ((frame / 2U) & 0x03U) {
    case 0:
        OLED_ShowString(x, y, "-");
        break;
    case 1:
        OLED_ShowString(x, y, "\\");
        break;
    case 2:
        OLED_ShowString(x, y, "|");
        break;
    default:
        OLED_ShowString(x, y, "/");
        break;
    }
}

void showDots(u8 x, u8 y, uint16_t frame)
{
    switch ((frame / 4U) & 0x03U) {
    case 0:
        OLED_ShowString(x, y, "   ");
        break;
    case 1:
        OLED_ShowString(x, y, ".  ");
        break;
    case 2:
        OLED_ShowString(x, y, ".. ");
        break;
    default:
        OLED_ShowString(x, y, "...");
        break;
    }
}

void drawPixel(int x, int y, bool on = true)
{
    if ((x < 0) || (x > 127) || (y < 0) || (y > 63)) {
        return;
    }
    OLED_DrawPoint((u8)x, (u8)y, on ? 1U : 0U);
}

void drawHLine(int x, int y, int width)
{
    for (int i = 0; i < width; ++i) {
        drawPixel(x + i, y);
    }
}

void drawVLine(int x, int y, int height)
{
    for (int i = 0; i < height; ++i) {
        drawPixel(x, y + i);
    }
}

void drawRect(int x, int y, int width, int height)
{
    drawHLine(x, y, width);
    drawHLine(x, y + height - 1, width);
    drawVLine(x, y, height);
    drawVLine(x + width - 1, y, height);
}

void fillRect(int x, int y, int width, int height)
{
    for (int row = 0; row < height; ++row) {
        drawHLine(x, y + row, width);
    }
}

void drawBatteryIcon(int x, int y, uint8_t percent, uint16_t frame)
{
    drawRect(x, y, 22, 10);
    drawVLine(x + 22, y + 3, 4);

    int filled = (18 * percent) / 100;
    if ((percent == 0U) && ((frame & 0x08U) != 0U)) {
        filled = 3;
    }
    if (filled > 0) {
        fillRect(x + 2, y + 2, filled, 6);
    }
}

void drawAttitudeIcon(int x, int y, int16_t pitch_deg_x10, uint16_t frame)
{
    drawRect(x - 14, y - 10, 28, 20);
    drawHLine(x - 10, y, 21);
    drawVLine(x, y - 7, 15);

    int16_t limited = pitch_deg_x10;
    if (limited > 300) {
        limited = 300;
    } else if (limited < -300) {
        limited = -300;
    }

    const int offset = limited / 60;
    drawHLine(x - 8, y - offset, 17);
    drawPixel(x - 9, y - offset - 1);
    drawPixel(x + 9, y - offset + 1);

    const int dot_x = x - 10 + (int)((frame / 2U) % 21U);
    drawPixel(dot_x, y + 7);
}

void drawPulse(int x, int y, uint16_t frame)
{
    static const int8_t shape[16] = {0, 0, -2, 0, 3, -5, 3, 0, 0, 0, -3, 0, 2, 0, 0, 0};
    for (int i = 0; i < 28; ++i) {
        const int idx = (i + (int)(frame / 2U)) & 0x0F;
        drawPixel(x + i, y + shape[idx]);
    }
}

}  // namespace

OledUiDemo oled_ui_demo;

void OledUiDemo::init()
{
    OLED_Init();
    render({});
}

void OledUiDemo::update(const OledUiData &data)
{
    frame_++;
    render(data);
}

void OledUiDemo::render(const OledUiData &data)
{
    OLED_Clear_Gram();

    drawRect(0, 0, 128, 64);
    drawHLine(1, 14, 126);
    drawHLine(1, 40, 126);
    drawVLine(82, 15, 25);

    OLED_ShowString(4, 2, "B570");
    showMpuState(44, 2, data.mpu_ok);
    showRunState(76, 2, data.enabled);
    showSpinner(116, 2, frame_);

    OLED_ShowString(4, 17, "ANG");
    showPitch(17, data.pitch_deg_x10);
    drawAttitudeIcon(112, 27, data.pitch_deg_x10, frame_);

    OLED_ShowString(4, 29, "STA");
    OLED_ShowString(32, 29, data.enabled ? "RUN " : "STOP");
    showDots(68, 29, frame_);
    // drawPulse(94, 34, frame_);

    OLED_ShowString(4, 42, "L");
    showSignedNumber(14, 22, 42, data.pwm_left, 4);
    OLED_ShowString(64, 42, "R");
    showSignedNumber(74, 82, 42, data.pwm_right, 4);

    showVoltage(52, data.battery_centivolts);
    drawBatteryIcon(76, 53, batteryPercent(data.battery_centivolts), frame_);
    showRunState(104, 52, data.enabled);

    OLED_Refresh_Gram();
}

}  // namespace car
