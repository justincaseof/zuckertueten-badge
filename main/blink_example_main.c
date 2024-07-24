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
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

static uint8_t op_mode = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;


#define GPIO_INPUT_BTN      CONFIG_BUTTON_PIN
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_BTN)
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

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
            
            // only act on falling edges
            if (level == 1) {
                op_mode = !op_mode;
            }
        }
    }
}

// static int idx = 0;
// static int num_leds = 16;
// static int patterns[4][3] = {
//     { 0,  0, 16},
//     { 0, 16,  0},
//     {16,  0,  0},
//     {16, 16, 16}
// };
// static void blink_led2(void)
// {
//     int i;
//     for (i = num_leds; i > 0; i--) {
//         int _idx = ((i-1)+idx) % num_leds;
//         int pat[] = patterns[_idx%4];
//         led_strip_set_pixel(led_strip, _idx, pat[0],  pat[1], pat[2]);

//         vTaskDelay(63 / portTICK_PERIOD_MS);
//     }
//     idx++;
// }

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        // /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        // led_strip_set_pixel(led_strip,  0,  0,  0, 16);
        // led_strip_set_pixel(led_strip,  1,  0, 16,  0);
        // led_strip_set_pixel(led_strip,  2, 16,  0,  0);
        // led_strip_set_pixel(led_strip,  3, 16, 16, 16);
        // //
        // led_strip_set_pixel(led_strip,  4,  0,  0, 16);
        // led_strip_set_pixel(led_strip,  5,  0, 16,  0);
        // led_strip_set_pixel(led_strip,  6, 16,  0,  0);
        // led_strip_set_pixel(led_strip,  7, 16, 16, 16);
        // //
        // led_strip_set_pixel(led_strip,  8,  0,  0, 16);
        // led_strip_set_pixel(led_strip,  9,  0, 16,  0);
        // led_strip_set_pixel(led_strip, 10, 16,  0,  0);
        // led_strip_set_pixel(led_strip, 11, 16, 16, 16);
        // //
        // led_strip_set_pixel(led_strip, 12,  0,  0, 16);
        // led_strip_set_pixel(led_strip, 13,  0, 16,  0);
        // led_strip_set_pixel(led_strip, 14, 16,  0,  0);
        // led_strip_set_pixel(led_strip, 15, 16, 16, 16);

        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        
        led_strip_set_pixel(led_strip,  0,  0,  0, 16);
        led_strip_set_pixel(led_strip,  1,  0,  0, 16);
        led_strip_set_pixel(led_strip,  2,  0,  0, 16);
        led_strip_set_pixel(led_strip,  3,  0,  0, 16);
        //
        led_strip_set_pixel(led_strip,  4,  0,  0, 16);
        led_strip_set_pixel(led_strip,  5,  0,  0, 16);
        led_strip_set_pixel(led_strip,  6,  0,  0, 16);
        led_strip_set_pixel(led_strip,  7,  0,  0, 16);
        //
        led_strip_set_pixel(led_strip,  8,  16,  0,  0);
        led_strip_set_pixel(led_strip,  9,  16,  0,  0);
        led_strip_set_pixel(led_strip, 10,  16,  0,  0);
        led_strip_set_pixel(led_strip, 11,  16,  0,  0);
        //
        led_strip_set_pixel(led_strip, 12,  16,  0,  0);
        led_strip_set_pixel(led_strip, 13,  16,  0,  0);
        led_strip_set_pixel(led_strip, 14,  16,  0,  0);
        led_strip_set_pixel(led_strip, 15,  16,  0,  0);



        //
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_btn(void) 
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_BTN, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BTN, gpio_isr_handler, (void*) GPIO_INPUT_BTN);


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

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_btn();

    while (1) {
        if (op_mode) {
            ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
            blink_led();
            // blink_led2();
            /* Toggle the LED state */
            s_led_state = !s_led_state;
        }
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
