#define DT_DRV_COMPAT zmk_behavior_led

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 直接从设备树获取 Spec
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_INST_GET(0, gpios);

static int behavior_led_init(const struct device *dev) {
    if (!gpio_is_ready_dt(&led_gpio)) {
        return -ENODEV;
    }
    // 初始化为不激活状态（灭）
    return gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
}

static int on_led_binding_pressed(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event) {
    // 强制拉高/拉低（根据你 LED 的极性尝试 1 或 0）
    gpio_pin_set_dt(&led_gpio, 1); 
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_led_driver_api = {
    .binding_pressed = on_led_binding_pressed,
};

BEHAVIOR_DT_INST_DEFINE(0, behavior_led_init, NULL, NULL, NULL, 
                       POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, 
                       &behavior_led_driver_api);