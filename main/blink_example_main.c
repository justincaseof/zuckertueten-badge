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
#define OP_MODE_CLEAR 2
#define OP_MODE_IDLE 3
#define OP_MODE_FLASH 4
#define OP_MODE_SLEEP 5
static uint8_t op_mode = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

#define GPIO_INPUT_BTN CONFIG_BUTTON_PIN
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_BTN)
#define GPIO_SLEEP_BTN CONFIG_BUTTON_SLEEP_PIN
#define GPIO_SLEEP_BTN_SEL (1ULL << GPIO_SLEEP_BTN)

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;
static int64_t last_press = -1;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            int level = gpio_get_level(io_num);
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));

            if (level == 0)
            {
                // button pressed
                last_press = esp_timer_get_time();
            }
            else
            {
                if (last_press != -1)
                {
                    // button released
                    int64_t diff = esp_timer_get_time() - last_press;
                    last_press = -1;

                    if (diff > 1000000)
                    { // 1 second = 1.000.000 microseconds
                        // SLEEP
                        op_mode = OP_MODE_SLEEP;
                    }
                    else
                    {
                        op_mode = (op_mode + 1) % 2;
                    }

                    printf("  >>> diff: %lld\n", diff);
                    printf("  >>> New mode: %d\n", op_mode);
                }
                else
                {
                    printf("  >>> !!! prevented pin banging fail\n");
                }
            }
        }
    }
}

const int NUM_LEDS = 4;
const int RGB_IDX_RED = 0;
const int RGB_IDX_GREEN = 1;
const int RGB_IDX_BLUE = 2;

static int mode_2_idx = 0;
static uint32_t mode_2_patterns[4][3] = {
    {0, 0, 32},
    {0, 32, 0},
    {32, 0, 0},
    {32, 32, 32},
};

static int change_delay = 0;
static void blink_led2(void)
{
    if (change_delay % 128 == 0) {
        // FLASH !
        led_strip_set_pixel(led_strip, 0, 200, 200, 200);
        led_strip_set_pixel(led_strip, 1, 200, 200, 200);
        led_strip_set_pixel(led_strip, 2, 200, 200, 200);
        led_strip_set_pixel(led_strip, 3, 200, 200, 200);
    }
    else {
        // ROTATE
        int led_idx = mode_2_idx % NUM_LEDS;
        int led_idx_0 = (led_idx + 0) % NUM_LEDS;
        int led_idx_1 = (led_idx + 1) % NUM_LEDS;
        int led_idx_2 = (led_idx + 2) % NUM_LEDS;
        int led_idx_3 = (led_idx + 3) % NUM_LEDS;
        led_strip_set_pixel(led_strip, 0, mode_2_patterns[led_idx_0][RGB_IDX_RED], mode_2_patterns[led_idx_0][RGB_IDX_GREEN], mode_2_patterns[led_idx_0][RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 1, mode_2_patterns[led_idx_1][RGB_IDX_RED], mode_2_patterns[led_idx_1][RGB_IDX_GREEN], mode_2_patterns[led_idx_1][RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 2, mode_2_patterns[led_idx_2][RGB_IDX_RED], mode_2_patterns[led_idx_2][RGB_IDX_GREEN], mode_2_patterns[led_idx_2][RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 3, mode_2_patterns[led_idx_3][RGB_IDX_RED], mode_2_patterns[led_idx_3][RGB_IDX_GREEN], mode_2_patterns[led_idx_3][RGB_IDX_BLUE]);
    }


    if (change_delay++ % 2 == 0) {
        mode_2_idx++;
    }
    // refresh leds
    led_strip_refresh(led_strip);
}

static int mode_2_flash_onoff = 0;
static void blink_led_mode_3_flash_led(void)
{
    if (mode_2_flash_onoff == 0)
    {
        led_strip_clear(led_strip);
        mode_2_flash_onoff = 1;
    }
    else
    {
        for (int i = 0; i < NUM_LEDS; i++)
        {
            led_strip_set_pixel(led_strip, i, 75, 75, 75);
        }
        mode_2_flash_onoff = 0;
    }
    led_strip_refresh(led_strip);
}

static int leds_upper[3] = {0, 0, 0};
static int leds_lower[3] = {0, 0, 0};

static void blink_led_mode_1(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        //
        led_strip_set_pixel(led_strip, 0, leds_lower[RGB_IDX_RED], leds_lower[RGB_IDX_GREEN], leds_lower[RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 1, leds_lower[RGB_IDX_RED], leds_lower[RGB_IDX_GREEN], leds_lower[RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 2, leds_upper[RGB_IDX_RED], leds_upper[RGB_IDX_GREEN], leds_upper[RGB_IDX_BLUE]);
        led_strip_set_pixel(led_strip, 3, leds_upper[RGB_IDX_RED], leds_upper[RGB_IDX_GREEN], leds_upper[RGB_IDX_BLUE]);

        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void init_sleep()
{
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

    // change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_BTN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_SLEEP_BTN, GPIO_INTR_ANYEDGE);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BTN, gpio_isr_handler, (void *)GPIO_INPUT_BTN);
    gpio_isr_handler_add(GPIO_SLEEP_BTN, gpio_isr_handler, (void *)GPIO_SLEEP_BTN);
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

static bool mode_1_up = true;
static int mode_1_val_max = 200;
static int mode_1_val_min = 20;
static int mode_1_val = 0;
static int mode_1_val_step = 5;

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_btn();

    while (1)
    {
        // check for button long press
        if (last_press != -1)
        {
            int64_t diff = esp_timer_get_time() - last_press;
            if (diff > 1000000)
            {
                printf("  *** reached 1 second!\n");
                op_mode = OP_MODE_FLASH;
            }
        }

        switch (op_mode)
        {
        case OP_MODE_BLINK_1:
            blink_led_mode_1();
            s_led_state = true;

            if (mode_1_up)
            {
                if (mode_1_val < mode_1_val_max)
                {
                    mode_1_val = mode_1_val + mode_1_val_step;
                }
                else
                {
                    mode_1_up = !mode_1_up;
                }
            }
            else
            {
                if (mode_1_val > mode_1_val_min)
                {
                    mode_1_val = mode_1_val - mode_1_val_step;
                }
                else
                {
                    mode_1_up = !mode_1_up;
                }
            }

            // upper: red
            leds_upper[RGB_IDX_RED] = mode_1_val + 50;
            leds_upper[RGB_IDX_GREEN] = mode_1_val / 10;
            leds_upper[RGB_IDX_BLUE] = 0;
            // lower: green
            leds_lower[RGB_IDX_RED] = 0;
            leds_lower[RGB_IDX_GREEN] = mode_1_val_max - mode_1_val + 20;
            leds_lower[RGB_IDX_BLUE] = 0;
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
            blink_led_mode_3_flash_led();
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
        // vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
