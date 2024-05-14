/**
 * Display class for generic e-Paper driven by EPDiy class
*/
#ifndef EPDIY_H
#define EPDIY_H

#define EPDIY_COLUMNS          (LV_HOR_RES_MAX / 8)

#ifdef __cplusplus
extern "C" {
#endif


#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
 #include "lvgl/lvgl.h"
#endif
#include "sdkconfig.h"

/* Configure your display */
void epdiy_init(void);

/* LVGL callbacks */
void epdiy_flush(lv_display_t *drv, const lv_area_t *area, uint8_t * color_map);

void epdiy_release_cb(lv_event_t * e);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* EPDIY_H */