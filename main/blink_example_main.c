/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h" 
#include "esp_timer.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
// #include "deep_sleep.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#define OP_MODE_BLINK_1 0
#define OP_MODE_BLINK_2 1
#define OP_MODE_CLEAR   2
#define OP_MODE_IDLE    3
#define OP_MODE_FLASH   4
#define OP_MODE_SLEEP   5
static uint8_t op_mode = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

// FIXME: externalize. on esp32c3 this is "D0" (really!)
#define CONFIG_BUTTON_SLEEP_PIN 2

#define GPIO_INPUT_BTN      CONFIG_BUTTON_PIN
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_BTN)
#define GPIO_SLEEP_BTN      CONFIG_BUTTON_SLEEP_PIN
#define GPIO_SLEEP_BTN_SEL  (1ULL<<GPIO_SLEEP_BTN)

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;
static int64_t last_press = -1;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int level = gpio_get_level(io_num);
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            
            if (level == 0) {
                // button pressed
                last_press = esp_timer_get_time();
            } else {
                // button released
                int64_t diff = esp_timer_get_time() - last_press;
                last_press = -1;

                if (diff > 1000000) { // 1 second = 1.000.000 microseconds
                    // SLEEP
                    op_mode = OP_MODE_SLEEP;
                } else {
                    op_mode = (op_mode+1) % 2;
                }


                printf("  >>> diff: %lld\n", diff);
                printf("  >>> New mode: %d\n", op_mode);
            }
            
        }
    }
}

static int NUM_LEDS = 16;
static int idx = 0;
static int patterns[16][3] = {
    { 0,  0,  32},
    { 0,  32,  0},
    { 0,  0,  0},
    { 0,  0,  0},

    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
    
    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
    
    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
    { 0,  0,  0},
};
static int red_hue = 0;
static int red_hue_direction = 0; // 0=up, 1=down
static int red_hue_lower = 0;
static int red_hue_upper = 60;
static int blue_hue = 0;
static int blue_hue_direction = 0; // 0=up, 1=down
static int blue_hue_lower = 0;
static int blue_hue_upper = 60;
static void adapt_hue() {
    if (red_hue_direction == 0) {
        red_hue++;
    } else {
        red_hue--;
    }

    if (red_hue < red_hue_lower) {
        red_hue_direction = 0;  // now: UP!
    }
    if (red_hue > red_hue_upper) {
        red_hue_direction = 1;  // now: DOWN!
    }
    //printf("  >>> RED: %d\n", red_hue);

    // TODO
    blue_hue = red_hue;
}

static void blink_led2(void)
{
    int step = 4;   // should be devidable by 2
    int i,j;
    for (i = 0; i < NUM_LEDS; i+=step) {
        for (j=0; j<step; j++) {
            int led_idx = i + j;
            int pat_idx = (i + j + (idx*step))%(NUM_LEDS);
            int *pat = &patterns[pat_idx];
            //led_strip_set_pixel(led_strip, led_idx, pat[0],  pat[1], pat[2]);
            
            int r = pat[0];
            int g = pat[1];
            int b = pat[2];
            if (led_idx < 8 && led_idx >= 0) {
                r += red_hue;
            }
            if (led_idx >= 8 && led_idx < NUM_LEDS) {
                b += blue_hue;
            }

            led_strip_set_pixel(led_strip, led_idx, b,  g, r);
            
            //printf("LED[%d] with PAT[%d] -- %d, %d, %d\n", led_idx, pat_idx, pat[0],  pat[1], pat[2]);
        }

        //printf("LED[%d] %d, %d, %d\n", i, pat[0],  pat[1], pat[2]);

        //vTaskDelay(63 / portTICK_PERIOD_MS);
    }
    
    idx++;
    adapt_hue();
    
    led_strip_refresh(led_strip);
}

static int flash = 0;
static void flash_led(void)
{
    if (flash == 0) {
        led_strip_clear(led_strip);
        flash = 1;
    } else {
        for (int i = 0; i < NUM_LEDS; i++) {
            led_strip_set_pixel(led_strip,  i,  75,  75, 75);
        }
        flash = 0;
    }
    led_strip_refresh(led_strip);
}



static int leds_upper[3] = { 0, 0, 0 };
static int leds_lower[3] = { 0, 0, 0 };

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {      
        //
        led_strip_set_pixel(led_strip,  0,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  1,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  2,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  3,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        //
        led_strip_set_pixel(led_strip,  4,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  5,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  6,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        led_strip_set_pixel(led_strip,  7,  leds_lower[0],  leds_lower[1], leds_lower[2]);
        //
        led_strip_set_pixel(led_strip,  8,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip,  9,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip, 10,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip, 11,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        //
        led_strip_set_pixel(led_strip, 12,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip, 13,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip, 14,  leds_upper[0],  leds_upper[1], leds_upper[2]);
        led_strip_set_pixel(led_strip, 15,  leds_upper[0],  leds_upper[1], leds_upper[2]);

        //
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}



static void init_sleep() {
    printf("entering sleep ...\n");
    
    // esp_deep_sleep_enable_gpio_wakeup(GPIO_SLEEP_BTN, ESP_GPIO_WAKEUP_GPIO_LOW);
    // esp_deep_sleep_enable_gpio_wakeup(GPIO_SLEEP_BTN, ESP_GPIO_WAKEUP_GPIO_HIGH);
    // esp_deep_sleep_start();
    
    // esp_sleep_enable_gpio_wakeup();
    // esp_light_sleep_start();

    esp_deep_sleep_enable_gpio_wakeup((1ULL << GPIO_SLEEP_BTN), ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();

    
    printf("...returned!\n"); // wont ever be displayed ??
}





static void configure_btn(void) 
{
    gpio_config_t io_conf = {};

    // mode button
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    // sleep button
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_SLEEP_BTN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    //change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_BTN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_SLEEP_BTN, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BTN, gpio_isr_handler, (void*) GPIO_INPUT_BTN);
    gpio_isr_handler_add(GPIO_SLEEP_BTN, gpio_isr_handler, (void*) GPIO_SLEEP_BTN);


}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 16, // 4 PCBs with 4 LEDs each.
        .led_model = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

static bool up = true;
static int val = 0;



void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_btn();

    while (1) {
        // check for button long press
        if (last_press != -1) {
            int64_t diff = esp_timer_get_time() - last_press;
            if (diff > 1000000) {
                printf("  >>> reached 1 second!\n");
                op_mode = OP_MODE_FLASH;
                last_press = -1;// good, bad, optional?
            }
        }

        switch (op_mode) {
            case OP_MODE_BLINK_1:
                blink_led();
                s_led_state = true;

                if (up) {
                    if (val < 150) {
                        val = val + 5;
                    } else {
                        up = !up;
                    }
                } else {
                    if (val > 20) {
                        val = val - 5;
                    } else {
                        up = !up;
                    }
                }
                
                // upper red
                leds_upper[0] = val;
                leds_lower[2] = 150 - val;
                break;
            case OP_MODE_BLINK_2:
                blink_led2();
                break;
            case OP_MODE_CLEAR:
                 led_strip_clear(led_strip);
                 led_strip_refresh(led_strip);
                 // send off to idle
                 op_mode = OP_MODE_IDLE;
                 break;
            case OP_MODE_IDLE:
                // idle.
                break;
            case OP_MODE_FLASH:
                flash_led();
                break;
            case OP_MODE_SLEEP:
                led_strip_clear(led_strip);
                led_strip_refresh(led_strip);
                init_sleep();
                break;
            default:
                // nothing.
                break;
        }
        //vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

