#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "epdiy_epaper.h"
#include "epdiy.h"
#include "string.h"

EpdiyHighlevelState hl;
uint16_t flushcalls = 0;
uint8_t *framebuffer;
uint8_t temperature = 25;
bool init = true;
// MODE_DU: Fast monochrome | MODE_GC16 slow with 16 grayscales
// MODE_PACKING_8PPB -> Monochrome mode. Still do not get how it works, added a Discussion in epdiy
// LINK: https://github.com/vroland/epdiy/discussions/312
enum EpdDrawMode updateMode = MODE_DU;

/* Display initialization routine */
void epdiy_init(void)
{
  epd_init(&epd_board_v7, &ED060XC3, EPD_LUT_64K);
  // Set VCOM for boards that allow to set this in software (in mV).
  // This will print an error if unsupported. In this case,
  // set VCOM using the hardware potentiometer and delete this line.
  epd_set_vcom(1760);
  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  framebuffer = epd_hl_get_framebuffer(&hl);

  // Declaring our own 8BPP buffer: Does not work correctly
  //framebuffer = (uint8_t *)heap_caps_malloc(epd_height()*epd_width()/2, MALLOC_CAP_SPIRAM);
   
  epd_poweron();
  // Clear all display in initialization to remove any ghosts
  epd_fullclear(&hl, temperature);
}

// Larry Kaleido take that used to work correctly for RGB233, now in RGB332
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t *image_data) {
  assert(framebuffer != NULL);
  int x, xx = image_area.x;
  int y, yy = image_area.y;
  int w = image_area.width;
  int h = image_area.height;
  const uint8_t ucCLRMask = 0xc0;
  uint8_t uc, *s, *d;
// source data is one byte per pixel (RGB233)
   for (y=yy; y<(yy + h); y++) {
       int i = (xx+y) % 3; // which color filter is in use?
       s = (uint8_t *)&image_data[(y-yy) * w];
       d = &framebuffer[(y * epd_width() / 2) + (xx / 2)];
       x = xx;
       if (x & 1) {
          uc = d[0] & 0xf0; // special case for odd starting pixel
          if (s[0] & ucCLRMask)
              uc |= 0xf;
          s++;
          *d++ = uc;
          x++;
          if (i >= 3) i-=3;
       }
       for (; x<(xx + w); x+=2) { // work 2 pixels at a time
            uc = 0;
            if (s[0] & ucCLRMask) uc |= 0xf;

            if (s[1] & ucCLRMask) uc |= 0xf0;
 
            *d++ = uc;
            s += 2;
       } // for x
   } // for y
   }

/* This would be the IDEAL way to handle it, just memcpy line per line in epdiy buffer */
void buf_copy_to_framebuffer_directCC(EpdRect image_area, const uint8_t *image_data)
{
  uint16_t data_idx = 0;
  // Copy entire lines in epdiy buffer
  uint8_t *buf_ptr;
  for (uint16_t y = image_area.y; y < image_area.height+image_area.y; y++) {
    #ifdef DEBUG_IMG_DATA
    //Byte to bit conversion seems to be OK:
    printf("%x %d%d%d%d%d%d%d%d\n",image_data[i],bit[7],bit[6],bit[5],bit[4],bit[3],bit[2],bit[1],bit[0]);
    #endif
    buf_ptr = &framebuffer[y * epd_width() / 2 + image_area.x / 2];
    memcpy(buf_ptr, &image_data[data_idx],image_area.width/2);

    data_idx = y * (image_area.width/2);
  }
  // Debug dump a line in the image buffer
  //ESP_LOG_BUFFER_HEXDUMP("BUF", image_data, image_area.width/8, ESP_LOG_INFO);
}

/* Required by LVGL. Sends the color_map to the screen with a partial update  */
void epdiy_flush(lv_display_t *drv, const lv_area_t *area, uint8_t * color_map)
{
  static int x1 = 65535, y1 = 65535, x2 = -1, y2 = -1;
  ++flushcalls;
  uint16_t w = lv_area_get_width(area);
  uint16_t h = lv_area_get_height(area);

  EpdRect update_area = {
      .x = (uint16_t)area->x1,
      .y = (uint16_t)area->y1,
      .width = w,
      .height = h};

  // capture the upper left and lower right corners
  if (area->x1 < x1)
    x1 = area->x1;
  if (area->y1 < y1)
    y1 = area->y1;
  if (area->x2 > x2)
    x2 = area->x2;
  if (area->y2 > y2)
    y2 = area->y2;

  // This is the slower version that works good without leaving any white line
  buf_copy_to_framebuffer(update_area, color_map);

  if (lv_disp_flush_is_last(drv))
  { // only send to e-paper when complete
    update_area.x = x1;
    update_area.y = y1;
    update_area.width = (x2 - x1) + 1;
    update_area.height = (y2 - y1) + 1;
    //epd_poweron();
    epd_hl_update_area(&hl, updateMode, temperature, update_area); // update_area
    printf("update_area\n");
    //epd_poweroff();
    x1 = y1 = 65535;
    x2 = y2 = -1; // reset update boundary
  }
  printf("epdiy_flush %d x:%d y:%d w:%d h:%d\n", flushcalls,(uint16_t)area->x1,(uint16_t)area->y1,w,h);
  /* Inform the graphics library that you are ready with the flushing */
  lv_disp_flush_ready(drv);
}

void epdiy_release_cb(lv_event_t * e) {
  printf("epdiy_release_cb\n");
}
