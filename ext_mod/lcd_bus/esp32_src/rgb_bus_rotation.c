// Copyright (c) 2024 - 2025 Kevin G. Schlosser


#include "soc/soc_caps.h"

#if SOC_LCD_RGB_SUPPORTED
    // micropython includes
    #include "py/obj.h"
    #include "py/runtime.h"
    #include "py/gc.h"
    #include "py/stackctrl.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
    #include "freertos/event_groups.h"
    #include "freertos/idf_additions.h"
    #include "rom/ets_sys.h"
    #include "esp_system.h"
    #include "esp_cpu.h"

    #include "esp_lcd_panel_ops.h"

    #include "rgb_bus.h"
    #include "rgb565_dither.h"

    #include <string.h>

    #define RGB_BIT_0 (1 << 0)


    void rgb_bus_event_init(rgb_bus_event_t *event)
    {
        event->handle = xEventGroupCreateStatic(&event->buffer);
    }


    void rgb_bus_event_delete(rgb_bus_event_t *event)
    {
        xEventGroupSetBits(event->handle, RGB_BIT_0);
        vEventGroupDelete(event->handle);

    }

    void rgb_bus_event_wait(rgb_bus_event_t *event)
    {
        xEventGroupWaitBits(event->handle, RGB_BIT_0, pdFALSE, pdTRUE, portMAX_DELAY);
    }


    bool rgb_bus_event_isset(rgb_bus_event_t *event)
    {
        return (bool)(xEventGroupGetBits(event->handle) & RGB_BIT_0);
    }


    bool rgb_bus_event_isset_from_isr(rgb_bus_event_t *event)
    {
        return (bool)(xEventGroupGetBitsFromISR(event->handle) & RGB_BIT_0);
    }


    void rgb_bus_event_set(rgb_bus_event_t *event)
    {
        xEventGroupSetBits(event->handle, RGB_BIT_0);
    }


    void rgb_bus_event_clear(rgb_bus_event_t *event)
    {
        xEventGroupClearBits(event->handle, RGB_BIT_0);
    }


    void rgb_bus_event_clear_from_isr(rgb_bus_event_t *event)
    {
        xEventGroupClearBitsFromISR(event->handle, RGB_BIT_0);
    }


    void rgb_bus_event_set_from_isr(rgb_bus_event_t *event)
    {
        xEventGroupSetBitsFromISR(event->handle, RGB_BIT_0, pdFALSE);
    }


    int rgb_bus_lock_acquire(rgb_bus_lock_t *lock, int32_t wait_ms)
    {
        return pdTRUE == xSemaphoreTake(lock->handle, wait_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS((uint16_t)wait_ms));
    }


    void rgb_bus_lock_release(rgb_bus_lock_t *lock)
    {
        xSemaphoreGive(lock->handle);
    }


    void rgb_bus_lock_release_from_isr(rgb_bus_lock_t *lock)
    {
        xSemaphoreGiveFromISR(lock->handle, pdFALSE);
    }


    void rgb_bus_lock_init(rgb_bus_lock_t *lock)
    {
        lock->handle = xSemaphoreCreateBinaryStatic(&lock->buffer);
        xSemaphoreGive(lock->handle);
    }


    void rgb_bus_lock_delete(rgb_bus_lock_t *lock)
    {
        vSemaphoreDelete(lock->handle);
    }


    #define RGB_BUS_ROTATION_0    (0)
    #define RGB_BUS_ROTATION_90   (1)
    #define RGB_BUS_ROTATION_180  (2)
    #define RGB_BUS_ROTATION_270  (3)


    static void rotate0(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint8_t bytes_per_pixel, uint8_t rgb565_dither);

    static void rotate_8bpp(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint8_t rotate);

    static void rotate_16bpp(uint16_t *src, uint16_t *dst, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint8_t rotate, uint8_t rgb565_dither);

    static void rotate_24bpp(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint8_t rotate);

    static void rotate_32bpp(uint32_t *src, uint32_t *dst, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint8_t rotate);


    static void copy_pixels(void *dst, void *src, uint32_t x_start, uint32_t y_start,
                        uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                        uint32_t bytes_per_pixel, uint8_t rotate, uint8_t rgb565_dither);


    static bool rgb_bus_trans_done_cb(esp_lcd_panel_handle_t panel,
                                    const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
    {
        LCD_UNUSED(panel);
        LCD_UNUSED(edata);
        mp_lcd_rgb_bus_obj_t *self = (mp_lcd_rgb_bus_obj_t *)user_ctx;

        if (!rgb_bus_event_isset_from_isr(&self->swap_bufs)) {
            uint8_t *idle_fb = self->idle_fb;
            self->idle_fb = self->active_fb;
            self->active_fb = idle_fb;
            rgb_bus_event_set_from_isr(&self->swap_bufs);
        }

        return false;
    }


    // With bounce buffers, the driver reads a frame buffer only to refill
    // them, and takes the buffer last passed to esp_lcd_panel_draw_bitmap when
    // the refill wraps to the next frame. This callback runs in that same ISR,
    // right after the switch: once it has fired, the other buffer is no longer
    // read. Direct mode waits for it before LVGL may draw again.
    static bool rgb_bus_frame_done_cb(esp_lcd_panel_handle_t panel,
                                    const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
    {
        LCD_UNUSED(panel);
        LCD_UNUSED(edata);
        mp_lcd_rgb_bus_obj_t *self = (mp_lcd_rgb_bus_obj_t *)user_ctx;

        if (!self->frame_wait) return false;

        self->frame_wait = 0;
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(self->frame_done.handle, &woken);
        return woken == pdTRUE;
    }


    void rgb_bus_copy_task(void *self_in) {
        LCD_DEBUG_PRINT("rgb_bus_copy_task - STARTED\n")

        mp_lcd_rgb_bus_obj_t *self = (mp_lcd_rgb_bus_obj_t *)self_in;

        esp_lcd_rgb_panel_event_callbacks_t callbacks = {
            .on_vsync = rgb_bus_trans_done_cb,
            .on_frame_buf_complete = rgb_bus_frame_done_cb
        };

        self->init_err = esp_lcd_new_rgb_panel(&self->panel_io_config, &self->panel_handle);
        if (self->init_err != 0) {
            self->init_err_msg = MP_ERROR_TEXT("%d(esp_lcd_new_rgb_panel)");
            rgb_bus_lock_release(&self->init_lock);
            vTaskDelete(NULL);
            return;
        }

        self->init_err = esp_lcd_rgb_panel_register_event_callbacks(self->panel_handle, &callbacks, self);
        if (self->init_err != 0) {
            self->init_err_msg = MP_ERROR_TEXT("%d(esp_lcd_rgb_panel_register_event_callbacks)");
            rgb_bus_lock_release(&self->init_lock);
            vTaskDelete(NULL);
            return;
        }

        self->init_err = esp_lcd_panel_reset(self->panel_handle);
        if (self->init_err != 0) {
            self->init_err_msg = MP_ERROR_TEXT("%d(esp_lcd_panel_reset)");
            rgb_bus_lock_release(&self->init_lock);
            vTaskDelete(NULL);
            return;
        }

        self->init_err = esp_lcd_panel_init(self->panel_handle);
        if (self->init_err != 0) {
            self->init_err_msg = MP_ERROR_TEXT("%d(esp_lcd_panel_init)");
            rgb_bus_lock_release(&self->init_lock);
            vTaskDelete(NULL);
            return;
        }

        void *frame_buffer0 = NULL;
        void *frame_buffer1 = NULL;
        self->init_err = esp_lcd_rgb_panel_get_frame_buffer(
            self->panel_handle, 2, &frame_buffer0, &frame_buffer1);

        if (self->init_err != 0) {
            self->init_err_msg = MP_ERROR_TEXT("%d(esp_lcd_rgb_panel_get_frame_buffer)");
            rgb_bus_lock_release(&self->init_lock);
            vTaskDelete(NULL);
            return;
        }

        self->active_fb = (uint8_t *)frame_buffer0;
        self->idle_fb = (uint8_t *)frame_buffer1;

        uint8_t *idle_fb;
        bool last_update;

        uint8_t bytes_per_pixel = self->bytes_per_pixel;
        rgb_bus_lock_acquire(&self->copy_lock, -1);

        self->init_err = LCD_OK;
        rgb_bus_lock_release(&self->init_lock);

        bool exit = rgb_bus_event_isset(&self->copy_task_exit);
        while (!exit) {
            rgb_bus_lock_acquire(&self->copy_lock, -1);

            if (self->partial_buf == NULL) break;
            // Set before tx_color_lock is released below, so once tx_color
            // has returned, copy_busy tells whether the frame is still being
            // shown and resynced.
            self->copy_busy = 1;
            last_update = self->last_update;

            idle_fb = self->idle_fb;

            copy_pixels(
                (void *)idle_fb, (void *)self->partial_buf,
                self->x_start, self->y_start,
                self->x_end, self->y_end,
                self->width, self->height,
                bytes_per_pixel, self->rotation, self->rgb565_dither);

            rgb_bus_lock_release(&self->tx_color_lock);

            if (last_update) {
                mp_lcd_err_t ret = esp_lcd_panel_draw_bitmap(
                    self->panel_handle,
                    0,
                    0,
                    self->width,
                    self->height,
                    idle_fb
                );

                if (ret != 0) {
                    mp_printf(&mp_plat_print, "esp_lcd_panel_draw_bitmap error (%d)\n", ret);
                } else {
                    rgb_bus_event_clear(&self->swap_bufs);
                    rgb_bus_event_wait(&self->swap_bufs);
                    memcpy(self->idle_fb, self->active_fb, self->width * self->height * bytes_per_pixel);
                }
            }

            self->copy_busy = 0;
            exit = rgb_bus_event_isset(&self->copy_task_exit);
        }

        LCD_DEBUG_PRINT("rgb_bus_copy_task - STOPPED\n")
        vTaskDelete(NULL);
    }


    __attribute__((always_inline))
    static inline void copy_8bpp(uint8_t *from, uint8_t *to)
    {
        *to++ = *from++;
    }

    __attribute__((always_inline))
    static inline void copy_16bpp(uint16_t *from, uint16_t *to)
    {
        *to++ = *from++;
    }

    __attribute__((always_inline))
    static inline void copy_24bpp(uint8_t *from, uint8_t *to)
    {
        *to++ = *from++;
        *to++ = *from++;
        *to++ = *from++;
    }

    __attribute__((always_inline))
    static inline void copy_32bpp(uint32_t *from, uint32_t *to)
    {
        *to++ = *from++;
    }

    void copy_pixels(void *dst, void *src, uint32_t x_start, uint32_t y_start,
            uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
            uint32_t bytes_per_pixel, uint8_t rotate, uint8_t rgb565_dither)
    {
        if (rotate == RGB_BUS_ROTATION_0) {
            rotate0(src, dst, MIN(x_start, dst_width), MIN(y_start, dst_height),
                    MIN(x_end, dst_width), MIN(y_end, dst_height),
                    dst_width, dst_height, bytes_per_pixel, rgb565_dither);
        } else {
            y_end += 1; // removes black lines between blocks
            if (rotate == RGB_BUS_ROTATION_90 || rotate == RGB_BUS_ROTATION_270) {
                x_start = MIN(x_start, dst_height);
                x_end = MIN(x_end, dst_height);
                y_start = MIN(y_start, dst_width);
                y_end = MIN(y_end, dst_width);
            } else {
                x_start = MIN(x_start, dst_width);
                x_end = MIN(x_end, dst_width);
                y_start = MIN(y_start, dst_height);
                y_end = MIN(y_end, dst_height);
            }

            if (bytes_per_pixel == 1) {
                rotate_8bpp(src, dst, x_start, y_start, x_end, y_end, dst_width, dst_height, rotate);
            } else if (bytes_per_pixel == 2) {
                rotate_16bpp(src, dst, x_start, y_start, x_end, y_end, dst_width, dst_height, rotate, rgb565_dither);
            } else if (bytes_per_pixel == 3) {
                rotate_24bpp(src, dst, x_start, y_start, x_end, y_end, dst_width, dst_height, rotate);
            } else if (bytes_per_pixel == 4) {
                rotate_32bpp(src, dst, x_start, y_start, x_end, y_end, dst_width, dst_height, rotate);
            }
        }
    }


    void rotate0(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                uint32_t x_end, uint32_t y_end, uint32_t dst_width,
                uint32_t dst_height, uint8_t bytes_per_pixel, uint8_t rgb565_dither)
    {
        dst += ((y_start * dst_width + x_start) * bytes_per_pixel);
        if(x_start == 0 && x_end == (dst_width - 1) && !rgb565_dither) {
            memcpy(dst, src, dst_width * (y_end - y_start + 1) * bytes_per_pixel);
        } else {
            uint32_t src_bytes_per_line = (x_end - x_start + 1) * bytes_per_pixel;
            uint32_t dst_bytes_per_line = dst_width * bytes_per_pixel;

            if (rgb565_dither) {
                for(uint32_t y = y_start; y <= y_end; y++) {
                    for (uint32_t x=0;x<x_end;x++) {
                        rgb565_dither_pixel(CALC_THRESHOLD(x, y), (uint16_t *)(src) + x);
                        copy_16bpp((uint16_t *)(src) + x, (uint16_t *)(dst) + x);
                    }
                    dst += dst_bytes_per_line;
                    src += src_bytes_per_line;
                }
            } else {
                for(uint32_t y = y_start; y <= y_end; y++) {
                    memcpy(dst, src, src_bytes_per_line);
                    dst += dst_bytes_per_line;
                    src += src_bytes_per_line;
                }
            }
        }
    }

    void rotate_8bpp(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                    uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                    uint8_t rotate)
    {
        uint32_t i;
        uint32_t j;

        uint32_t src_bytes_per_line = x_end - x_start + 1;
        uint32_t offset = y_start * src_bytes_per_line + x_start;

        switch (rotate) {
            case RGB_BUS_ROTATION_90:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x - offset;
                        j = (dst_height - 1 - x) * dst_width + y;
                        copy_8bpp(src + i, dst + j);
                    }
                }
                break;

            // MIRROR_X MIRROR_Y
            case RGB_BUS_ROTATION_180:
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);

                for (uint32_t y = y_start; y < y_end; y++) {
                    i = (dst_height - 1 - y) * dst_width + (dst_width - 1 - x_start);
                    for (uint32_t x = x_start; x < x_end; x++) {
                        copy_8bpp(src, dst + i);
                        src++;
                        i--;
                    }
                    src++;
                }
                break;

            // SWAP_XY   MIRROR_X
            case RGB_BUS_ROTATION_270:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x - offset;
                        j = x * dst_width + dst_width - 1 - y;
                        copy_8bpp(src + i, dst + j);
                    }
                }
                break;

            default:
                LCD_UNUSED(i);
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);
                break;
        }

    }


    void rotate_16bpp(uint16_t *src, uint16_t *dst, uint32_t x_start, uint32_t y_start,
                    uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                    uint8_t rotate, uint8_t rgb565_dither)
    {
        uint32_t i;
        uint32_t j;

        uint32_t src_bytes_per_line = x_end - x_start + 1;
        uint32_t offset = y_start * src_bytes_per_line + x_start;

        if (rgb565_dither) {
            switch (rotate) {
                case RGB_BUS_ROTATION_90:
                    for (uint32_t y = y_start; y < y_end; y++) {
                        for (uint32_t x = x_start; x < x_end; x++) {
                            i = y * src_bytes_per_line + x - offset;
                            j = (dst_height - 1 - x) * dst_width + y;
                            rgb565_dither_pixel(CALC_THRESHOLD(x, y), src + i);
                            copy_16bpp(src + i, dst + j);
                        }
                    }
                    break;

                // MIRROR_X MIRROR_Y
                case RGB_BUS_ROTATION_180:
                    LCD_UNUSED(j);
                    LCD_UNUSED(src_bytes_per_line);
                    LCD_UNUSED(offset);

                    for (uint32_t y = y_start; y < y_end; y++) {
                        i = (dst_height - 1 - y) * dst_width + (dst_width - 1 - x_start);
                        for (uint32_t x = x_start; x < x_end; x++) {
                            rgb565_dither_pixel(CALC_THRESHOLD(x, y), src);
                            copy_16bpp(src, dst + i);
                            src++;
                            i--;
                        }
                        src++;
                    }
                    break;

                // SWAP_XY   MIRROR_X
                case RGB_BUS_ROTATION_270:
                    for (uint32_t y = y_start; y < y_end; y++) {
                        for (uint32_t x = x_start; x < x_end; x++) {
                            i = y * src_bytes_per_line + x - offset;
                            j = (x * dst_width + dst_width - 1 - y);
                            rgb565_dither_pixel(CALC_THRESHOLD(x, y), src + i);
                            copy_16bpp(src + i, dst + j);
                        }
                    }
                    break;

                default:
                    LCD_UNUSED(i);
                    LCD_UNUSED(j);
                    LCD_UNUSED(src_bytes_per_line);
                    LCD_UNUSED(offset);
                    break;
            }
        } else {
            switch (rotate) {
                case RGB_BUS_ROTATION_90:
                    for (uint32_t y = y_start; y < y_end; y++) {
                        for (uint32_t x = x_start; x < x_end; x++) {
                            i = y * src_bytes_per_line + x - offset;
                            j = (dst_height - 1 - x) * dst_width + y;
                            copy_16bpp(src + i, dst + j);
                        }
                    }
                    break;

                // MIRROR_X MIRROR_Y
                case RGB_BUS_ROTATION_180:
                    LCD_UNUSED(j);
                    LCD_UNUSED(src_bytes_per_line);
                    LCD_UNUSED(offset);

                    for (uint32_t y = y_start; y < y_end; y++) {
                        i = (dst_height - 1 - y) * dst_width + (dst_width - 1 - x_start);
                        for (uint32_t x = x_start; x < x_end; x++) {
                            copy_16bpp(src, dst + i);
                            src++;
                            i--;
                        }
                        src++;
                }
                    break;

                // SWAP_XY   MIRROR_X
                case RGB_BUS_ROTATION_270:
                    for (uint32_t y = y_start; y < y_end; y++) {
                        for (uint32_t x = x_start; x < x_end; x++) {
                            i = y * src_bytes_per_line + x - offset;
                            j = (x * dst_width + dst_width - 1 - y);
                            copy_16bpp(src + i, dst + j);
                        }
                    }
                    break;

                default:
                    LCD_UNUSED(i);
                    LCD_UNUSED(j);
                    LCD_UNUSED(src_bytes_per_line);
                    LCD_UNUSED(offset);
                    break;
            }
        }
    }


    void rotate_24bpp(uint8_t *src, uint8_t *dst, uint32_t x_start, uint32_t y_start,
                    uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                    uint8_t rotate)
    {
        uint32_t i;
        uint32_t j;

        uint32_t src_bytes_per_line = (x_end - x_start + 1) * 3;
        uint32_t offset = y_start * src_bytes_per_line + x_start * 3;

        switch (rotate) {

            case RGB_BUS_ROTATION_90:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x * 3 - offset;
                        j = ((dst_height - 1 - x) * dst_width + y) * 3;
                        copy_24bpp(src + i, dst + j);
                    }
                }
                break;

            // MIRROR_X MIRROR_Y
            case RGB_BUS_ROTATION_180:
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);

                for (int y = y_start; y < y_end; y++) {
                    i = ((dst_height - 1 - y) * dst_width + (dst_width - 1 - x_start)) * 3;
                    for (size_t x = x_start; x < x_end; x++) {
                        copy_24bpp(src, dst + i);
                        src += 3;
                        i -= 3;
                    }
                    src++;
                }
                break;

            // SWAP_XY   MIRROR_X
            case RGB_BUS_ROTATION_270:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x * 3 - offset;
                        j = (x * dst_width + dst_width - 1 - y) * 3;
                        copy_24bpp(src + i, dst + j);
                    }
                }
                break;

            default:
                LCD_UNUSED(i);
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);
                break;
        }
    }


    void rotate_32bpp(uint32_t *src, uint32_t *dst, uint32_t x_start, uint32_t y_start,
                    uint32_t x_end, uint32_t y_end, uint32_t dst_width, uint32_t dst_height,
                    uint8_t rotate)
    {
        uint32_t i;
        uint32_t j;

        uint32_t src_bytes_per_line = x_end - x_start + 1;
        uint32_t offset = y_start * src_bytes_per_line + x_start;

        switch (rotate) {
            case RGB_BUS_ROTATION_90:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x - offset;
                        j = (dst_height - 1 - x) * dst_width + y;
                        copy_32bpp(src + i, dst + j);
                    }
                }
                break;

            // MIRROR_X MIRROR_Y
            case RGB_BUS_ROTATION_180:
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);

                for (uint32_t y = y_start; y < y_end; y++) {
                    i = (dst_height - 1 - y) * dst_width + (dst_width - 1 - x_start);
                    for (uint32_t x = x_start; x < x_end; x++) {
                        copy_32bpp(src, dst + i);
                        src++;
                        i--;
                    }
                    src++;
                }
                break;

            // SWAP_XY   MIRROR_X
            case RGB_BUS_ROTATION_270:
                for (uint32_t y = y_start; y < y_end; y++) {
                    for (uint32_t x = x_start; x < x_end; x++) {
                        i = y * src_bytes_per_line + x - offset;
                        j = x * dst_width + dst_width - 1 - y;
                        copy_32bpp(src + i, dst + j);
                    }
                }
                break;

            default:
                LCD_UNUSED(i);
                LCD_UNUSED(j);
                LCD_UNUSED(src_bytes_per_line);
                LCD_UNUSED(offset);
                break;
        }
    }

#endif
