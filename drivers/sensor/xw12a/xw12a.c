#define DT_DRV_COMPAT xinwang_xw12a

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <dt-bindings/zmk/keys.h>

LOG_MODULE_REGISTER(XW12A, CONFIG_SENSOR_LOG_LEVEL);

struct xw12a_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
};

struct xw12a_data {
    struct gpio_callback gpio_cb;
    struct k_work work;
    const struct device *dev;
    bool last_pad0_state;
};

static void xw12a_work_handler(struct k_work *work)
{
    struct xw12a_data *data = CONTAINER_OF(work, struct xw12a_data, work);
    const struct xw12a_config *config = data->dev->config;

    uint8_t buf[2];

    // XW12A 地址已经在 config 中 (0x40)
    // 直接读取 2 字节数据
    if (i2c_read_dt(&config->i2c, buf, sizeof(buf)) != 0) {
        return;
    }

    /* * 根据 8051 示例：
     * PAD0 按下时返回 0x7FFF (0111 1111 1111 1111)
     * 正常状态是 0xFFFF (1111 1111 1111 1111)
     * 大端序或小端序可能影响字节顺序，通常 I2C 高位在前
     */
    uint16_t raw_val = (buf[0] << 8) | buf[1];
    
    // 如果 Bit 15 是 0，表示 PAD0 被按下
    bool pad0_pressed = !(raw_val & BIT(15));

    if (pad0_pressed == data->last_pad0_state) {
        return;
    }

    data->last_pad0_state = pad0_pressed;

    // 发送 A 键
    raise_zmk_keycode_state_changed(
        zmk_keycode_state_changed_from_encoded(A, pad0_pressed, k_uptime_get())
    );
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
    data->last_pad0_state = false;

    if (!device_is_ready(config->i2c.bus) || !device_is_ready(config->int_gpio.port)) {
        return -ENODEV;
    }

    // 配置引脚：XW12A 的 INT 通常需要上拉
    gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    k_work_init(&data->work, xw12a_work_handler);
    gpio_init_callback(&data->gpio_cb, xw12a_gpio_callback, BIT(config->int_gpio.pin));
    gpio_add_callback(config->int_gpio.port, &data->gpio_cb);

    // 【重要】自检测试：开机直接发一个 A 验证链路
    // 放在这里确保 APPLICATION 阶段能被 ZMK 捕获
    k_msleep(5000); 
    raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(A, true, k_uptime_get()));
    k_msleep(100);
    raise_zmk_keycode_state_changed(zmk_keycode_state_changed_from_encoded(A, false, k_uptime_get()));

    return 0;
}

static struct xw12a_data xw12a_data_0;

static const struct xw12a_config xw12a_config_0 = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET(0, int_gpios),
};

// 使用 APPLICATION 级别确保 ZMK 已就绪
DEVICE_DT_INST_DEFINE(0, xw12a_init, NULL, &xw12a_data_0, &xw12a_config_0, POST_KERNEL, 99, NULL);