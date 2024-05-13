#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "epdiy_epaper.h"
#include "epdiy.h"

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
  epd_poweron();
  // Clear all display in initialization to remove any ghosts
  epd_fullclear(&hl, temperature);
}

// Larry Kaleido take that used to work correctly for RGB232
void buf_copy_to_framebuffer_Kaleido(EpdRect image_area, const uint8_t *image_data) {
  assert(framebuffer != NULL);
  int x, xx = image_area.x;
  int y, yy = image_area.y;
  int w = image_area.width;
  int h = image_area.height;
  const uint8_t ucCLRMask[3] = {0xc0,0x38,0x7};
  uint8_t uc, *s, *d;
// source data is one byte per pixel (RGB233)
   for (y=yy; y<(yy + h); y++) {
       int i = (xx+y) % 3; // which color filter is in use?
       s = (uint8_t *)&image_data[(y-yy) * w];
       d = &framebuffer[(y * epd_width() / 2) + (xx / 2)];
       x = xx;
       if (x & 1) {
          uc = d[0] & 0xf0; // special case for odd starting pixel
          if (s[0] & ucCLRMask[i])
              uc |= 0xf;
          i++;
          s++;
          *d++ = uc;
          x++;
          if (i >= 3) i-=3;
       }
       for (; x<(xx + w); x+=2) { // work 2 pixels at a time
            uc = 0;
            if (s[0] & ucCLRMask[i]) uc |= 0xf;
            i++; if (i >= 3) i -= 3;
            if (s[1] & ucCLRMask[i]) uc |= 0xf0;
            i++; if (i >= 3) i -= 3;
            *d++ = uc;
            s += 2;
       } // for x
   } // for y
}

/* A copy from epd_copy_to_framebuffer with temporary lenght prediction */
  // Best though: Use L8 and also epdiy MODE_PACKING_8PPB 1bpp buffer (Both 8 pixels in a byte)
  // LV_COLOR_FORMAT_L8 full monochrome 1bpp, need to take that and convert it to epdiy framebuffer
  // ^ BUT... Could not make work L8 mode :(
void buf_copy_to_framebuffer(EpdRect image_area, const uint8_t *image_data)
{
  // This is mostly crap. A better start is Larry Kaleido version
  uint16_t data_idx = 0;
  for (uint16_t y = image_area.y; y < image_area.height+image_area.y; y++) {
    for (uint16_t x = image_area.x; x < image_area.width+image_area.x/8; x++)
    {
      // Extracts bit's from a supposedly 8BPP image
      // with the mission to fit in a 4BPP epdiy framebuffer
      uint8_t bit[8];
      bit[0] = ((image_data[data_idx]>>0)&1); // pixel 0
      bit[1] = ((image_data[data_idx]>>1)&1); // pixel 1 -> 1st framebuffer(FB) pos
      bit[2] = ((image_data[data_idx]>>2)&1); // 0
      bit[3] = ((image_data[data_idx]>>3)&1); // 1 2nd FB pos
      bit[4] = ((image_data[data_idx]>>4)&1); // 0
      bit[5] = ((image_data[data_idx]>>5)&1); // 1 3rd FB pos
      bit[6] = ((image_data[data_idx]>>6)&1); // 0
      bit[7] = ((image_data[data_idx]>>7)&1); // 1 4th FB pos
      data_idx++;
      //printf("%x ", image_data[i]); // Byte to bit conversion seems to be OK:
      //printf("%d%d%d%d%d%d%d%d\n",bit[7],bit[6],bit[5],bit[4],bit[3],bit[2],bit[1],bit[0]);
      
      /**
       * @brief What I don't think is OK that even at the beginning where image_data is 
       * supposed to be all white, there are different black points, why?
       * I don't get it.
       */
      for (uint8_t fp=1; fp<=8; fp++) {
        // In epdiy 255 makes a pixel full white, 0 full black.
        // This is of course crap, but you get the idea:
        uint8_t color = (bit[fp-1]==0) ? 0 : 255;

        // This should actually fill all updated parts with a black pixel changing color to 0
        epd_draw_pixel((x*8)+fp, y, color, framebuffer);
      }
    }
  }
  //printf("\nTotal image_data bytes: %d\n", data_idx);
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

/*
 * Called for each pixel. Designed with the idea to fill the buffer directly, not to set each pixel, see LVGL Forum (buf_area_to_framebuffer)
 * Setting buf here comes as color_map in epdiy_flush
 */
void epdiy_set_px_cb(lv_display_t *disp_drv, uint8_t *buf,
                     lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                     lv_color_t color, lv_opa_t opa)
{
  // Test using RGB232. NOT used in v9
  /*
  int16_t epd_color = 255;
  if ((int16_t)color.full < 254)
  {
    epd_color = (updateMode == MODE_DU) ? 0 : (int16_t)color.full / 3;
  }

  // Instead of using epd_draw_pixel: Set pixel directly in *buf that comes afterwards in flush as *color_map
  uint16_t idx = (int16_t)y * buf_w / 2 + (int16_t)x / 2;
  if (x % 2)
  {
    buf[idx] = (buf[idx] & 0x0F) | (epd_color & 0xF0);
  }
  else
  {
    buf[idx] = (buf[idx] & 0xF0) | (epd_color >> 4);
  } 
  */
}
