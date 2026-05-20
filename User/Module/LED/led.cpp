#include "led.hpp"

static void LED_set_color(enum LED_COLOR color)
{
    switch (color)
    {
        case LED_RED:
            WS2812_Ctrl(7, 0, 0);
            break;
        case LED_GREEN:
            WS2812_Ctrl(0, 7, 0);
            break;
        case LED_BLUE:
            WS2812_Ctrl(0, 0, 7);
            break;
        case LED_WHITE:
            WS2812_Ctrl(7, 7, 7);
            break;
    }
}

void LED_ALL_ON()
{
    WS2812_Ctrl(7,7,7);
}

void LED_ALL_OFF()
{
    WS2812_Ctrl(0, 0, 0);
}

void LED_blink(enum LED_COLOR color)
{
    static uint32_t flash_count;
    flash_count ++;

    if (flash_count <= 250)
    {
        WS2812_Ctrl(0, 0, 0); // 关闭LED灯
    }
    else if (flash_count <= 500)
    {
        LED_set_color(color);
    }

    if (flash_count >= 500)
    {
        flash_count = 0;
    }
}

void LED_blink_alternate(enum LED_COLOR color_a, enum LED_COLOR color_b)
{
    static uint32_t flash_count;
    flash_count++;

    if (flash_count <= 250)
    {
        LED_set_color(color_a);
    }
    else if (flash_count <= 500)
    {
        LED_set_color(color_b);
    }

    if (flash_count >= 500)
    {
        flash_count = 0;
    }
}

void LED_blink_alternate(enum LED_COLOR color_a, enum LED_COLOR color_b, enum LED_COLOR color_c)
{
    static uint32_t flash_count;
    flash_count++;

    if (flash_count <= 250)
    {
        LED_set_color(color_a);
    }
    else if (flash_count <= 500)
    {
        LED_set_color(color_b);
    }
    else if (flash_count <= 750)
    {
        LED_set_color(color_c);
    }

    if (flash_count >= 750)
    {
        flash_count = 0;
    }
}
