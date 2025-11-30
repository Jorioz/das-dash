#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include "HWCDC.h"
#include "pin_config.h"
#include "image.h"
#include "ui/ui.h"
#include <esp_now.h>
#include <WiFi.h>

// ESP NOW
uint8_t gateway_mac[] = { 0x6C, 0xC8, 0x40, 0x59, 0x4D, 0x48 };

typedef struct {
  int mesh_id;
  unsigned long can_id;
  byte len;
  uint8_t d[8];
} esp_now_frame_t;


void update_battery_ui(float percentage);
void animate_arc_to(lv_obj_t *arc, int new_value);
unsigned long lastBatteryUpdate = 0;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
                                    1 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);


std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
                                                         TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  CST816T->IIC_Interrupt_Flag = true;
}

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

uint32_t screenWidth;
uint32_t screenHeight;

static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf[screenWidth * screenHeight / 10];


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static uint8_t count = 0;
void example_increase_reboot(void *arg) {
  count++;
  if (count == 30) {
    esp_restart();
  }
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  // Only read if interrupt flag is set (touch event)
  if (!CST816T->IIC_Interrupt_Flag) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  int32_t touchX = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  // Suppress errors if no valid touch response
  if (touchX < 0 || touchY < 0) {
    data->state = LV_INDEV_STATE_REL;
    CST816T->IIC_Interrupt_Flag = false;
    return;
  }

  int32_t mappedX = touchY;
  int32_t mappedY = 240 - touchX;

  CST816T->IIC_Interrupt_Flag = false;
  data->state = LV_INDEV_STATE_PR;

  /* Set the coordinates with some debounce */
  if (mappedX >= 0 && mappedY >= 0) {
    data->point.x = mappedX;
    data->point.y = mappedY;
    //USBSerial.printf("Data x: %d, Data y: %d\n", mappedX, mappedY);
  }
}

unsigned long lastSendTime = 0;
void battery_charge_request() {
  esp_now_frame_t frame;
  frame.mesh_id = 101;
  frame.can_id = 0x765; // Req to get battery %
  frame.len = 8;
  uint8_t data[8] = {0x03, 0x22, 0x1D, 0xD0, 0x00, 0x00, 0x00, 0x00};
  memcpy(frame.d, data, 8);
  esp_now_send(gateway_mac, (uint8_t*)&frame, sizeof(frame));
  USBSerial.print("Sent Request to Gateway\n");
}

// Function to handle battery charge response frame from gateway
void battery_charge_response(const esp_now_frame_t* frame) {
  lastBatteryUpdate = millis();
  USBSerial.println("Received battery charge response from gateway:");
  USBSerial.print("mesh_id: "); USBSerial.println(frame->mesh_id);
  USBSerial.print("can_id: "); USBSerial.println(frame->can_id, HEX);
  USBSerial.print("len: "); USBSerial.println(frame->len);
  USBSerial.print("data: ");
  for (int i = 0; i < frame->len; i++) {
    USBSerial.print(frame->d[i], HEX); USBSerial.print(" ");
  }
  USBSerial.println();
  // Parse Percentage
  float battery_percent = frame->d[4] / 2.0f;
  update_battery_ui(battery_percent);
  
}

void animate_arc_to(lv_obj_t *arc, int new_value) {
    int current = lv_arc_get_value(arc);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_values(&a, current, new_value);
    lv_anim_set_time(&a, 250);
    lv_anim_set_exec_cb(&a, [](void *obj, int32_t v) {
        lv_arc_set_value((lv_obj_t *)obj, v);
    });
    lv_anim_start(&a);
}

void update_battery_ui(float percentage){
    if (ui_battery_percentage){
        int rounded = (int)roundf(percentage);
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", rounded);
        lv_label_set_text(ui_battery_percentage, buf);
    }
    if (ui_battery_level_arc) {
        animate_arc_to(ui_battery_level_arc, (int)percentage);
    }
}
// ESP-NOW receive callback
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(esp_now_frame_t)) {
    const esp_now_frame_t* frame = (const esp_now_frame_t*)incomingData;
    battery_charge_response(frame);
  } else {
    USBSerial.println("Received ESP-NOW data with unexpected length.");
  }
}

void setup() {
  USBSerial.begin(115200); /* prepare for possible serial debug */

  while (CST816T->begin() == false) {
    USBSerial.println("CST816T initialization fail");
    delay(2000);
  }
  USBSerial.println("CST816T initialization successfully");

  CST816T->IIC_Write_Device_State(CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                  CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

  gfx->begin();
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  lv_init();

  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  USBSerial.println(LVGL_Arduino);
  USBSerial.println("I am LVGL_Arduino");



#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * screenHeight / 4);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello Ardino and LVGL!");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  const esp_timer_create_args_t reboot_timer_args = {
    .callback = &example_increase_reboot,
    .name = "reboot"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
  USBSerial.println("Setup done");
  ui_init();


  // General Setup
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gateway_mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  if (millis() - lastSendTime >= 1000) {
    battery_charge_request();
    lastSendTime = millis();
  }

    if (millis() - lastBatteryUpdate > 5000) {
    if (ui_battery_percentage) lv_label_set_text(ui_battery_percentage, "---");
    if (ui_battery_level_arc) animate_arc_to(ui_battery_level_arc, 0);
  }
  delay(5);

}
