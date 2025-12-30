#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>

#if IS_ENABLED(CONFIG_BATTERY_ALERT)

/* --- 自定义配置区 --- */
#define BATTERY_ALERT_CYCLE_COUNT    10     
#define TIME_LOW_MS                  1000  
#define TIME_DISCONNECT_MS           1000  
#define ALERT_PIN                    31    
/* -------------------- */

static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

// 定义工作队列任务
static struct k_work_delayable alert_work;

void perform_battery_alert_action(void) {
    if (!device_is_ready(gpio0_dev)) return;

    for (int i = 0; i < BATTERY_ALERT_CYCLE_COUNT; i++) {
        // 1. 拉低
        gpio_pin_configure(gpio0_dev, ALERT_PIN, GPIO_OUTPUT_LOW);
        k_msleep(TIME_LOW_MS);

        // 2. 断开 (高阻输入)
        gpio_pin_configure(gpio0_dev, ALERT_PIN, GPIO_INPUT);
        k_msleep(TIME_DISCONNECT_MS);
    }
}

// 工作队列的回调函数
static void alert_work_handler(struct k_work *work) {
    // 强制触发一次电池电量更新逻辑（可选，取决于具体驱动）
    // 获取当前电量百分比
    uint8_t level = zmk_battery_state_of_charge();
    
    // 调试建议：如果电量检测始终不准，可以暂时注释掉判断条件来验证引脚是否工作
    if (level > 0 && level < CONFIG_BATTERY_LEVEL) {
        perform_battery_alert_action();
    }
}

static int battery_alert_init(void) {
    // 初始化延迟工作项
    k_work_init_delayable(&alert_work, alert_work_handler);
    
    // 在系统启动 5 秒后执行检测
    // 这能确保 ADC 采样逻辑已经完全跑通并有了第一次数据
    k_work_schedule(&alert_work, K_SECONDS(5));
    
    return 0;
}

SYS_INIT(battery_alert_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif