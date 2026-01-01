#define DT_DRV_COMPAT xinwang_xw12a

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <dt-bindings/zmk/keys.h>

struct xw12a_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct xw12a_data {
    struct gpio_callback gpio_cb;
    struct k_work work;
    const struct device *dev;
    // 增加状态记录
    bool last_pad0_state;
    bool last_pad1_state;
};

static void xw12a_work_handler(struct k_work *work)
{
    struct xw12a_data *data = CONTAINER_OF(work, struct xw12a_data, work);
    const struct xw12a_config *config = data->dev->config;
    uint8_t buf[2];

    if (i2c_read_dt(&config->i2c, buf, sizeof(buf)) != 0) {
        return;
    }

    uint16_t raw_val = (buf[0] << 8) | buf[1];
    
    // 解析状态：Pad0 是 Bit 15, Pad1 是 Bit 14
    bool pad0_pressed = !(raw_val & BIT(15));
    bool pad1_pressed = !(raw_val & BIT(14));

    // --- Pad0 逻辑 (输出 A) ---
    if (data->last_pad0_state && !pad0_pressed) {
        raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(A, true, k_uptime_get()));
        k_msleep(10);
        raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(A, false, k_uptime_get()));
    }
    data->last_pad0_state = pad0_pressed;

    // --- Pad1 逻辑 (输出 B) ---
    if (data->last_pad1_state && !pad1_pressed) {
        raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(B, true, k_uptime_get()));
        k_msleep(10);
        raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(B, false, k_uptime_get()));
    }
    data->last_pad1_state = pad1_pressed;

    // 扫动补丁：若仍有键被按住，继续处理
    if (raw_val != 0xFFFF) {
        k_work_submit(&data->work);
    }
}

static void xw12a_gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct xw12a_data *data = CONTAINER_OF(cb, struct xw12a_data, gpio_cb);
    k_work_submit(&data->work);
}

static int xw12a_init(const struct device *dev)
{
    const struct xw12a_config *config = dev->config;
    struct xw12a_data *data = dev->data;
    data->dev = dev;

    if (!device_is_ready(config->i2c.bus) || !device_is_ready(config->int_gpio.port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_BOTH);

    k_work_init(&data->work, xw12a_work_handler);
    gpio_init_callback(&data->gpio_cb, xw12a_gpio_callback, BIT(config->int_gpio.pin));
    gpio_add_callback(config->int_gpio.port, &data->gpio_cb);

    // 初始状态同步
    uint8_t init_buf[2];
    if (i2c_read_dt(&config->i2c, init_buf, sizeof(init_buf)) == 0) {
        uint16_t val = (init_buf[0] << 8 | init_buf[1]);
        data->last_pad0_state = !(val & BIT(15));
        data->last_pad1_state = !(val & BIT(14));
    }

    return 0;
}

static struct xw12a_data xw12a_data_0;
static const struct xw12a_config xw12a_config_0 = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

DEVICE_DT_INST_DEFINE(0, xw12a_init, NULL, &xw12a_data_0, &xw12a_config_0, POST_KERNEL, 99, NULL);