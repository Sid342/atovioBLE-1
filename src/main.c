#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/lbs.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(pwm_led_control, LOG_LEVEL_INF);

/* --- Devicetree aliases --------------------------------------- */
#define BTN_NODE                 DT_ALIAS(sw0)
#define PWM_NODE0                DT_ALIAS(pwm_led0)  // Blue LED
#define PWM_NODE1                DT_ALIAS(pwm_led1)  // Green LED
#define PWM_NODE2                DT_ALIAS(pwm_led2)  // Red LED
#define CHARGER_NODE             DT_ALIAS(charging_stat)
#define IONIZER_NODE             DT_ALIAS(ionz) // Ionizer PWM

#define ADC_NODE                DT_NODELABEL(adc)   // ADC node
#define ADC_RESOLUTION          12
#define ADC_GAIN                ADC_GAIN_1_6
#define ADC_REFERENCE           ADC_REF_INTERNAL
#define ADC_CHANNEL_ID          4
#define ADC_BUFFER_SIZE         1

#define NUM_SAMPLES             30
#define TIME_IN_MIN             60000  // 1 minute in milliseconds
#define NUM_PWM                 (ARRAY_SIZE(pwm_leds))
#define DEBOUNCE_MS             50
#define SLOW_MODE_TOGGLE_TIME   K_SECONDS(1)

#define VOLTAGE_SHUTDOWN_MV     3100  // 3.1V shutdown threshold
#define VTG_THRESH_TOSAVETIME   3200  // 3.2V
#define BTRY_LOW_PCT            20           // 20% battery considered low
#define DFU_FLAG_ID             10  // Unique ID for our DFU flag
#define BUTTON_HOLD_DISCONNECT  3000

/* NVS Configuration */
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET    FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION           storage_partition
#define MODE_TIME_OFF_ID        1
#define MODE_TIME_NORMAL_ID     2
#define MODE_TIME_TURBO_ID      3
#define MAGIC_ID                4
#define MAGIC_VALUE             0x55AA55AA

/* NVS Persistent Mode Time Configuration */
static struct nvs_fs fs;
static uint32_t magic_check = 0;

/* PWM descriptors for channels 0-2 ----------------------------- */
static const struct pwm_dt_spec pwm_leds[] = {
    PWM_DT_SPEC_GET(PWM_NODE0),  // Blue LED 
    PWM_DT_SPEC_GET(PWM_NODE1),  // Green LED 
    PWM_DT_SPEC_GET(PWM_NODE2),  // Red LED
};

static const struct gpio_dt_spec button         = GPIO_DT_SPEC_GET(BTN_NODE, gpios);
static const struct gpio_dt_spec charger_gpio   = GPIO_DT_SPEC_GET(CHARGER_NODE, gpios);
static const struct gpio_dt_spec ionizer        = GPIO_DT_SPEC_GET(IONIZER_NODE, gpios);

/* ADC variables */
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buffer[ADC_BUFFER_SIZE];

static struct adc_channel_cfg adc_channel_cfg = {
    .gain               = ADC_GAIN,
    .reference          = ADC_REFERENCE,
    .acquisition_time   = ADC_ACQ_TIME_DEFAULT,
    .channel_id         = ADC_CHANNEL_ID,
    .input_positive     = SAADC_CH_PSELP_PSELP_AnalogInput4,  // AIN4 = GPIO 28
};

static struct adc_sequence sequence = {
    .channels           = BIT(ADC_CHANNEL_ID),
    .buffer             = adc_buffer,
    .buffer_size        = sizeof(adc_buffer),
    .resolution         = ADC_RESOLUTION,
};

/* --- Breathing parameters ------------------------------------- */
static const uint8_t max_duty[] = {100, 100, 100}; // Max brightness for all LEDs
static const uint32_t PWM_PERIOD_US = 20000U;   // 20 ms = 50 Hz
static const uint8_t STEP = 2;                 // Duty cycle step
static uint8_t breath_count = 0;  // For FAST mode counting

/* Runtime state */
static uint8_t active_ch = 0;     // Active LED channel (0=Blue, 1=Green, 2=Red)
static uint8_t current_duty = 0;  // Use uint8_t for duty cycle
static bool rising = true;
static bool last_charging_state = false;

/* --- Debounce setup ------------------------------------------- */
static struct gpio_callback btn_cb;
static struct k_work_delayable db_work;
static bool last_level, buttonPressed;

static struct k_work_delayable slow_mode_toggle_work;
static bool slow_mode_state = false;
static struct k_work_delayable button_hold_work;
static bool button_held = false;

/* Mode enum */
enum { modeOff = 0, modeNormal, modeTurbo, MODE_COUNT };
static volatile uint8_t mode = modeOff;

K_SEM_DEFINE(mode_change_sem, 0, 1);  // Initial count 0, max count 1

/* Mode time tracking */
static uint32_t mode_time[MODE_COUNT] = {0, 0, 0}; // Time spent in each mode
static uint32_t mode_start_time = 0; // Start time for the current mode

/* Bluetooth variables */
static struct bt_conn *current_conn = NULL;
static bool advertising_active = false;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/* Define 128-bit UUIDs for custom characteristics */
#define BT_UUID_MODE_VAL                BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_BATTERY_VOLTAGE_VAL     BT_UUID_128_ENCODE(0x87654321, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_BATTERY_PERCENTAGE_VAL  BT_UUID_128_ENCODE(0xabcdef01, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_CHARGING_STATUS_VAL     BT_UUID_128_ENCODE(0xabcdef12, 0x3456, 0x7890, 0xabcd, 0xef1234567890)
#define BT_UUID_MODE_TIME_VAL           BT_UUID_128_ENCODE(0xa7654321, 0x4321, 0x6789, 0x4321, 0xabcdef012345)
#define BT_UUID_MODE_RESET_VAL          BT_UUID_128_ENCODE(0x13579bdf, 0x2468, 0xace0, 0x1357, 0x9bdf2468ace0)

/* Instantiate characteristic UUID objects from the 128-bit values */
static struct bt_uuid_128 modeUuid           = BT_UUID_INIT_128(BT_UUID_MODE_VAL);
static struct bt_uuid_128 battVoltageUuid    = BT_UUID_INIT_128(BT_UUID_BATTERY_VOLTAGE_VAL);
static struct bt_uuid_128 battPercentUuid    = BT_UUID_INIT_128(BT_UUID_BATTERY_PERCENTAGE_VAL);
static struct bt_uuid_128 chargingStatusUuid = BT_UUID_INIT_128(BT_UUID_CHARGING_STATUS_VAL);
static struct bt_uuid_128 modeTimeUuid       = BT_UUID_INIT_128(BT_UUID_MODE_TIME_VAL);
static struct bt_uuid_128 modeResetUuid      = BT_UUID_INIT_128(BT_UUID_MODE_RESET_VAL);

static int battery_percent = 0;
static uint8_t mode_string[8] = "OFF"; // Initial mode string
static uint8_t batt_voltage_string[20] = "0.00V";
static uint8_t batt_percent_string[20] = "0%";
static uint8_t charging_status_string[15] = "Not Charging";

// Add this at the top before any usage:
static void set_pwm(uint8_t ch, uint32_t duty_pct);
static int  read_battery_voltage_mv(void);
static void notify_mode(void);
static void update_active_led(void);
static void blink_red_led_twice(void);
static int  save_mode_times(void);
static void set_ionizer(bool on);
static void blink_all_leds(uint8_t count, uint32_t on_time_ms, uint32_t off_time_ms);

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = conn;
    LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    current_conn = NULL;
    blink_all_leds(2, 500, 100); // Blink all LEDs 2 times when disconnected
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static ssize_t read_mode_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, mode_string, strlen((char *)mode_string));
}

static ssize_t read_battery_voltage_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, batt_voltage_string, strlen((char *)batt_voltage_string));
}

static ssize_t read_battery_percentage_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, batt_percent_string, strlen((char *)batt_percent_string));
}

static ssize_t read_charging_status_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, charging_status_string, strlen((char *)charging_status_string));
}

static ssize_t read_mode_time_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    // Format the mode time into a string
    char time_string[32];
    snprintf(time_string, sizeof(time_string), "O: %u,\tS: %u,\tF: %u",
             mode_time[modeOff]     / TIME_IN_MIN, 
             mode_time[modeNormal]  / TIME_IN_MIN, 
             mode_time[modeTurbo]   / TIME_IN_MIN);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, time_string, strlen(time_string));
}

static ssize_t write_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len < 1) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

   // Check if battery is critical
    if (read_battery_voltage_mv() < VOLTAGE_SHUTDOWN_MV) {
        blink_red_led_twice();
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    char c = *((char *)buf);
    uint8_t new_mode = mode;

    switch (c) {
        case 'O': case 'o': new_mode = 0; break; // OFF
        case 'N': case 'n': new_mode = 1; break; // Normal
        case 'T': case 't': new_mode = 2; break; // Turbo
        default:
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    // PREVENT TURBO MODE IF BATTERY IS LOW AND NOT CHARGING
    if (new_mode == modeTurbo && battery_percent < BTRY_LOW_PCT && !last_charging_state) {
        LOG_INF("Cannot switch to Turbo mode: battery too low (%d%%)", battery_percent);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (new_mode != mode) {
        mode = new_mode;
        switch (mode) {
            case 0: strcpy((char *)mode_string, "OFF"); break;
            case 1: strcpy((char *)mode_string, "Normal"); break;
            case 2: strcpy((char *)mode_string, "Turbo"); break;
        }
        update_active_led();
        printk("Mode changed via BLE to: %s\n", mode_string);
        
        notify_mode();
        k_sem_give(&mode_change_sem);
    }
    return len;
}

static ssize_t write_mode_time_attr(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);    }

    char c = *((char *)buf);
    if (c == '1') {
        // Reset all timers
        memset(mode_time, 0, sizeof(mode_time));
        mode_start_time = k_uptime_get_32();
        printk("All timers reset to zero!\n");
        
        // Optionally notify if you want to confirm the reset
        char reset_msg[] = "Timers reset";
        bt_gatt_notify(NULL, attr, reset_msg, sizeof(reset_msg));
    } else {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    return len;
}

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(mode_svc,
    BT_GATT_PRIMARY_SERVICE(&modeUuid),

    /* Mode: READ/WRITE/NOTIFY (strings provided by owner) */
    BT_GATT_CHARACTERISTIC(&modeUuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_mode_attr, write_mode, NULL),
    /* Charging status: READ/NOTIFY */
    BT_GATT_CHARACTERISTIC(&chargingStatusUuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_charging_status_attr, NULL, NULL),
    /* Battery voltage: READ/NOTIFY */
    BT_GATT_CHARACTERISTIC(&battVoltageUuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_battery_voltage_attr, NULL, NULL),
    /* Battery percentage: READ/NOTIFY */
    BT_GATT_CHARACTERISTIC(&battPercentUuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_battery_percentage_attr, NULL, NULL),
    /* Mode time: READ only */
    BT_GATT_CHARACTERISTIC(&modeTimeUuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_mode_time_attr, NULL, NULL),
    /* Mode time reset: WRITE only */
    BT_GATT_CHARACTERISTIC(&modeResetUuid.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, write_mode_time_attr, NULL),
);
        
static void auto_shutdown_if_critical_battery(void)
{
    if (read_battery_voltage_mv() < VOLTAGE_SHUTDOWN_MV && mode != modeOff) {
        // Save current mode times before shutdown
        save_mode_times();
        
        // Switch to OFF mode
        mode = modeOff;
        strcpy((char *)mode_string, "OFF");
        
        // Turn off everything
        set_ionizer(false);
        set_pwm(0, 0);
        set_pwm(1, 0);
        set_pwm(2, 0);
        
        LOG_INF("Auto-shutdown: Battery critical (%.2fV < 3.1V)", read_battery_voltage_mv()/1000.0);
        notify_mode();
        mode_start_time = k_uptime_get_32();
    }
}

static void blink_all_leds(uint8_t count, uint32_t on_time_ms, uint32_t off_time_ms)
{
    // Save current mode and temporarily disable normal LED control
    uint8_t original_mode = mode;
    mode = 99; // Set to invalid mode to prevent interference from duty_thread
    
    for (int i = 0; i < count; i++) {
        // Turn all LEDs ON simultaneously
        set_pwm(0, 100); // Blue LED - full brightness
        set_pwm(1, 100); // Green LED - full brightness
        set_pwm(2, 100); // Red LED - full brightness
        k_sleep(K_MSEC(on_time_ms));
        
        // Turn all LEDs OFF simultaneously
        set_pwm(0, 0);
        set_pwm(1, 0);
        set_pwm(2, 0);
        k_sleep(K_MSEC(off_time_ms));
    }
    
    // Restore original mode
    mode = original_mode;
}

static void blink_red_led_twice(void)
{
    if (read_battery_voltage_mv() < VOLTAGE_SHUTDOWN_MV) {
        uint8_t original_mode = mode;  // Save current mode
        
        // Temporarily set to a special mode to prevent interference
        mode = 99;  // Use an invalid mode number
        
        // Blink red LED twice
        for (int i = 0; i < 2; i++) {
            set_pwm(2, 100);  // Red LED full brightness
            k_sleep(K_MSEC(500));
            set_pwm(2, 0);    // Red LED off
            k_sleep(K_MSEC(500));
        }
        
        mode = original_mode;  // Restore original mode
    }
}

int voltage_to_percent(int mv) {
    if (mv >= 4200) return 100;
    if (mv <= 3000) return 0;

// Linear mapping between 3.3V (0%) and 4.2V (100%)
return (mv - 3000) * 100 / (4200 - 3000);
}

static void notify_mode(void){
    bt_gatt_notify(NULL, &mode_svc.attrs[1], mode_string, strlen((char *)mode_string));
}

static void notify_battery_voltage(void) {
    bt_gatt_notify(NULL, &mode_svc.attrs[2], batt_voltage_string, strlen((char *)batt_voltage_string));
}

static void notify_battery_percent(void) {
    bt_gatt_notify(NULL, &mode_svc.attrs[3], batt_percent_string, strlen((char *)batt_percent_string));
}

static void notify_charging_status(void) {
    bt_gatt_notify(NULL, &mode_svc.attrs[4], charging_status_string, strlen((char *)charging_status_string));
}

static void set_ionizer(bool on) {
    gpio_pin_set_dt(&ionizer, on ? 1 : 0);
    LOG_INF("Ionizer turned %s", on ? "ON" : "OFF");
}

/* --- PWM helper ----------------------------------------------- */
static void set_pwm(uint8_t ch, uint32_t duty_pct)
{
    uint32_t pulse = (PWM_PERIOD_US * duty_pct) / 100;
    pwm_set_dt(&pwm_leds[ch], PWM_PERIOD_US, pulse);
}

/* --- Work Handlers ------------------------------------------- */
static void slow_mode_toggle_handler(struct k_work *work) {
    /* only generate ticks while in Normal mode */
    if (mode == modeNormal) {
        slow_mode_state = !slow_mode_state;
        
       if (slow_mode_state) {   set_ionizer(true);  LOG_INF("SLOW mode: ON for 1 second"); }
       else {                   set_ionizer(false); LOG_INF("SLOW mode: OFF for 1 second"); }
        // Reschedule the toggle next 1-second tick
        k_work_schedule(&slow_mode_toggle_work, SLOW_MODE_TOGGLE_TIME);
    }
}

/* NVS Timer Functions */
static int init_nvs(void)
{
    int rc;
    fs.flash_device = NVS_PARTITION_DEVICE;

    if (!device_is_ready(fs.flash_device)) {
        printk("Flash device not ready\n");
        return -ENODEV;
    }

    fs.offset = NVS_PARTITION_OFFSET;
    fs.sector_size = 4096; /* 4KB pages typical for nRF52 */
    fs.sector_count = 2U;  /* Use 2 sectors (8KB) for NVS */
    
    rc = nvs_mount(&fs);
    if (rc) {
        printk("NVS mount failed: %d\n", rc);
        return rc;
    }    
    printk("NVS mounted successfully\n");
    return 0;
}        

static int save_mode_times(void)
{
    int rc;
    
    // Save each mode time
    rc = nvs_write(&fs, MODE_TIME_OFF_ID, &mode_time[modeOff], sizeof(mode_time[modeOff]));
    if (rc < 0) {
        printk("Error saving OFF mode time: %d\n", rc);
        return rc;
    }
    
    rc = nvs_write(&fs, MODE_TIME_NORMAL_ID, &mode_time[modeNormal], sizeof(mode_time[modeNormal]));
    if (rc < 0) {
        printk("Error saving SLOW mode time: %d\n", rc);
        return rc;
    }
    
    rc = nvs_write(&fs, MODE_TIME_TURBO_ID, &mode_time[modeTurbo], sizeof(mode_time[modeTurbo]));
    if (rc < 0) {
        printk("Error saving FAST mode time: %d\n", rc);
        return rc;
    }
    
    // Save magic value to mark valid data
    rc = nvs_write(&fs, MAGIC_ID, &magic_check, sizeof(magic_check));
    if (rc < 0) {
        printk("Error saving magic value: %d\n", rc);
    }
    
    printk("Mode times saved: OFF=%u, SLOW=%u, FAST=%u ms\n",
           mode_time[modeOff], mode_time[modeNormal], mode_time[modeTurbo]);
    
    return rc;
}

static int load_mode_times(void) 
{
    int rc;
    
    // Check if we have valid stored data
    rc = nvs_read(&fs, MAGIC_ID, &magic_check, sizeof(magic_check));
    if (rc > 0 && magic_check == MAGIC_VALUE) {
        // Read mode times
        nvs_read(&fs, MODE_TIME_OFF_ID, &mode_time[modeOff], sizeof(mode_time[modeOff]));
        nvs_read(&fs, MODE_TIME_NORMAL_ID, &mode_time[modeNormal], sizeof(mode_time[modeNormal]));
        nvs_read(&fs, MODE_TIME_TURBO_ID, &mode_time[modeTurbo], sizeof(mode_time[modeTurbo]));
        
        printk("Loaded mode times from NVS: OFF=%u, SLOW=%u, FAST=%u ms\n",
               mode_time[modeOff], mode_time[modeNormal], mode_time[modeTurbo]);

    } else {   

        // First boot, initialize
        printk("First boot, initializing mode times...\n");
        magic_check = MAGIC_VALUE;
        nvs_write(&fs, MAGIC_ID, &magic_check, sizeof(magic_check));
        
        // Initialize mode times to zero
        memset(mode_time, 0, sizeof(mode_time));
        save_mode_times();
    }    
    
    return 0;
}

/* --- Apply brightness to active LED only ---------------------- */
static void apply_pwm(void)
{
    for (uint8_t i = 0; i < NUM_PWM; i++) {
        set_pwm(i, (i == active_ch) ? current_duty : 0);
    }
}

/* --- Check charging status and update active LED -------------- */
static void update_active_led(void)
{
    bool charging = (gpio_pin_get_dt(&charger_gpio) == 1);
    static int last_battery_percent = -1; // Track previous battery percentage
    
    // Check if charging state OR battery percentage changed
    if (charging != last_charging_state || battery_percent != last_battery_percent) {
        last_charging_state = charging;
        last_battery_percent = battery_percent; // Store current battery percentage

        if (charging) {
            active_ch = 1;  // Green LED when charging
            current_duty = 0;
            rising = true;
            breath_count = 0;
        } else {
            // If not charging, check battery percentage
            if (battery_percent < BTRY_LOW_PCT) {
                active_ch = 2;  // Red LED when battery is low
            } else {
                active_ch = 0;  // Blue LED when battery is not low
            }
            current_duty = 0; // Reset duty cycle
            rising = true; // Start breathing
        }
        
        LOG_INF("LED updated: Charging=%d, Battery=%d%%, Active LED=%s",
               charging, battery_percent, 
               (active_ch == 0) ? "Blue" : (active_ch == 1) ? "Green" : "Red");
    }
}

/* --- ADC reading function ------------------------------------- */
int read_battery_voltage_mv(void)
{
    int32_t sum = 0;
    int err;
    // Configure the ADC channel
    err = adc_channel_setup(adc_dev, &adc_channel_cfg);
    if (err) {
        LOG_ERR("Failed to configure ADC channel: %d", err);
        return err;
    }

    for (int i = 0; i < NUM_SAMPLES; i++) {
        err = adc_read(adc_dev, &sequence);
       
        int16_t raw = adc_buffer[0];
        // Convert to millivolts for each sample:
        int32_t mv = (raw * 3600 * 6) / 4095;
        // Multiply by 2 if using voltage divider (e.g. 1:2 ratio)
        mv *= 2;
        mv = mv * 0.166; // Calculated factor

        sum += mv;
        k_sleep(K_MSEC(1)); // Add a small delay between samples (optional)
    }
    // Calculate the average
    int32_t average_mv = sum / NUM_SAMPLES;
    return average_mv;
}

static void button_hold_handler(struct k_work *work)
{
    if (button_held && current_conn != NULL) {
        // First call: disconnect
        LOG_INF("Button held for 3 seconds - disconnecting");
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        
        // Schedule advertising restart after disconnection
        k_work_schedule(&button_hold_work, K_MSEC(1000));
        
    } else if (work != NULL) {
        // Second call: restart advertising (after disconnection)
        if (current_conn == NULL && mode != modeOff && !advertising_active) {
            int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
            if (err) {
                LOG_ERR("Advertising failed to restart: %d", err);
            } else {
                advertising_active = true;
                LOG_INF("Advertising restarted after manual disconnect");
            }
        }
    }
    
    button_held = false; // Reset for next press
}

/* --- Debounce logic (for button) ------------------------------ */
static void debounce_work(struct k_work *work)
{
     // Cancel any pending slow mode toggle
    k_work_cancel_delayable(&slow_mode_toggle_work);

    bool level = gpio_pin_get_dt(&button);

    if (level == last_level) {
        if (!buttonPressed && level) {
            buttonPressed = true;              /* press */

            // Start button hold timer
            button_held = true;
            k_work_schedule(&button_hold_work, K_MSEC(BUTTON_HOLD_DISCONNECT));
            LOG_INF("Button pressed");

        } else if (buttonPressed && !level) {
            buttonPressed = false;           /* release */

            button_held = false; // Button released before hold time
            k_work_cancel_delayable(&button_hold_work);
            // Check if battery is critical - blink red and prevent mode change

    if (read_battery_voltage_mv() < VOLTAGE_SHUTDOWN_MV) {
        blink_red_led_twice();
        LOG_INF("Battery critical - cannot change mode");
        return;
    }

            // NO TURBO mode when low battery
            if (mode == modeNormal && battery_percent < BTRY_LOW_PCT && !last_charging_state) {
                mode = modeOff;
                strcpy((char *)mode_string, "OFF");

            } else {
                mode = (mode + 1) % MODE_COUNT;
                current_duty = 0;
                rising = true;
                breath_count = 0;
                switch (mode) {
                    case modeOff:   strcpy((char *)mode_string, "OFF"); 
                                    set_ionizer(false);
                                    break;
                       
                    case modeNormal: strcpy((char *)mode_string, "Normal"); 
                                     set_ionizer(true);
                                     k_work_schedule(&slow_mode_toggle_work, SLOW_MODE_TOGGLE_TIME);
                                     break;

                    case modeTurbo: strcpy((char *)mode_string, "Turbo"); 
                                    set_ionizer(true);
                                    break;
                }
            }

            // FIX: FORCE UPDATE ACTIVE LED ON MODE CHANGE
            update_active_led();

            notify_mode(); // Notify Bluetooth clients of mode change
            mode_start_time = k_uptime_get_32();
        }
    }
}

static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    last_level = gpio_pin_get_dt(&button);
    k_work_reschedule(&db_work, K_MSEC(DEBOUNCE_MS));
}

/* --- Init helpers --------------------------------------------- */
static int pwm_init_all(void)
{
    for (uint8_t i = 0; i < NUM_PWM; i++) {
        if (!pwm_is_ready_dt(&pwm_leds[i])) {
            return -ENODEV;
        }
        set_pwm(i, 0);
    }
    return 0;
}

static int button_init(void)
{
    if (!gpio_is_ready_dt(&button)) return -ENODEV;
    gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &btn_cb);
    k_work_init_delayable(&db_work, debounce_work);
    return 0;
}

static int charger_init(void)
{
    if (!gpio_is_ready_dt(&charger_gpio)) { return -ENODEV; }
    gpio_pin_configure_dt(&charger_gpio, GPIO_INPUT);
    return 0;
}

static int ionizer_init(void) {
    if (!gpio_is_ready_dt(&ionizer)) return -ENODEV;
    return gpio_pin_configure_dt(&ionizer, GPIO_OUTPUT_INACTIVE);
}

bool led_breath_step(uint8_t ch, uint8_t max_duty, bool *rising, uint8_t *duty) {
    if (*rising) {
        *duty += STEP;
            if (*duty >= max_duty) {
            *duty = max_duty;
            *rising = false;
        }
    } else {
        if (*duty > STEP) {
            *duty -= STEP;
        } else {
            *duty = 0;
            *rising = true;
            apply_pwm(); // Apply final 0 before switching
            return true;  // breath complete
        }
    }
    active_ch = ch;      // Set which LED to apply PWM
    apply_pwm();         // Apply updated PWM
    return false;        // breath not complete
}

static void duty_thread(void)
{
    while (1) {
        update_active_led();
        
          // Skip LED control if mode is invalid (during blinking)
        if (mode >= MODE_COUNT) {  // MODE_COUNT is 3, so 99 is invalid
            k_sleep(K_MSEC(100));
            continue;
        }

        // Update mode time
        if (mode_start_time != 0) {
            uint32_t elapsed_time = k_uptime_get_32() - mode_start_time;
            mode_time[mode] += elapsed_time; // Accumulate time for the current mode
            mode_start_time = k_uptime_get_32(); // Reset start time for the next interval
        }

        // NOT charging
        if (!last_charging_state) {
            if (mode == modeOff) {
                set_pwm(0, 0);
                set_pwm(1, 0);
                set_pwm(2, 0); // Turn off all LEDs
                k_sleep(K_MSEC(100));
                continue;
            }

            uint32_t interval = (mode == modeNormal) ? 42 : 8;
            k_sleep(K_MSEC(interval));

            // Check battery percentage and breathe red and blue LEDs if below 20%
            if (battery_percent < BTRY_LOW_PCT) {
                // Prioritize breathing red LED first
                if (active_ch == 2) { // Red is active
                    if (led_breath_step(2, max_duty[2], &rising, &current_duty)) {
                        active_ch = 0;  // Switch to blue
                        rising = true;
                        current_duty = 0;
                    }
                } else { // Blue is active
                    if (led_breath_step(0, max_duty[0], &rising, &current_duty)) {
                        active_ch = 2;  // Switch to red
                        rising = true;
                        current_duty = 0;
                    }
                }
                continue;
            }

            // Normal breathing for active LED
            led_breath_step(active_ch, max_duty[active_ch], &rising, &current_duty);
            continue;
        }

        // Charging behavior
        switch (mode) {
            case modeOff:
                k_sleep(K_MSEC(42));
                led_breath_step(1, max_duty[1], &rising, &current_duty); // Green LED
                break;

            case modeNormal:
                k_sleep(K_MSEC(42));
                if (led_breath_step(active_ch, max_duty[active_ch], &rising, &current_duty)) {
                    active_ch = (active_ch == 0) ? 1 : 0;  // Toggle LED
                }
                break;

            case modeTurbo:
                if (active_ch == 1) {  // Green LED
                    k_sleep(K_MSEC(42));
                    if (led_breath_step(1, max_duty[1], &rising, &current_duty)) {
                        active_ch = 0; // Switch to blue LED
                        breath_count = 0;
                        rising = true;
                        current_duty = 0;
                    }
                } else {  // Blue LED
                    k_sleep(K_MSEC(8));
                    if (led_breath_step(0, max_duty[0], &rising, &current_duty)) {
                        breath_count++;
                        if (breath_count >= 3) {
                            active_ch = 1; // Switch to green LED
                            rising = true;
                            current_duty = 0;
                        }
                    }
                }
                break;

            default:
                break;
        }
    }
}

/* --- Main periodic handlers ---------------------------------- */
static void handle_battery_monitoring(void)
{
    static uint32_t last_battery_check = 0;
    static bool low_voltage_detected = false;

        // Read battery voltage and check if below threshold (every 10 seconds)
        if (k_uptime_get_32() - last_battery_check >= 10000) {
            int mv = read_battery_voltage_mv();
            if (mv >= 0) {
                battery_percent = voltage_to_percent(mv);

                // AUTO SHUTDOWN CHECK - ADD THIS
                auto_shutdown_if_critical_battery();
                update_active_led();

                // Update BLE characteristics
                snprintf((char *)batt_voltage_string, sizeof(batt_voltage_string),
                         "%d.%02dV", mv / 1000, (mv % 1000) / 10);
                snprintf((char *)batt_percent_string, sizeof(batt_percent_string),
                         "%d%%", battery_percent);
                
        // Check if voltage is below 3.2V and save mode times
        // In the battery check section:
        if (mv < VTG_THRESH_TOSAVETIME) {
                 if (!low_voltage_detected) {
                                low_voltage_detected = true;

                        printk("Low voltage detected (%.2fV < 3.2V), saving mode times to NVS\n", mv/1000.0);
                        save_mode_times(); // <-- ONLY SAVE ONCE!
                        update_active_led(); 
                        }
                } else if (mv >= VTG_THRESH_TOSAVETIME) {
                    low_voltage_detected = false;  // 
                }
            }
            last_battery_check = k_uptime_get_32();
        }
}
static void handle_mode_time_tracking(void)
{
    if (mode_start_time != 0) {
        uint32_t elapsed_time = k_uptime_get_32() - mode_start_time;
        mode_time[mode] += elapsed_time;
        mode_start_time = k_uptime_get_32();
    }
}
static void handle_ionizer_control(void)
{
    switch (mode) {
        case modeOff:
            set_ionizer(false);
            break;
        case modeNormal:
        case modeTurbo:
            set_ionizer(true);
            break;
    }
}
static void handle_advertising_management(void)
{
    int err;
    
    if (mode == modeOff && advertising_active) {
        bt_le_adv_stop();
        advertising_active = false;
        printk("Advertising stopped for sleep mode\n");
    } else if (mode != modeOff && !advertising_active && current_conn == NULL) {
        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            LOG_ERR("Advertising restart failed: %d", err);
        } else {
            advertising_active = true;
            printk("Advertising restarted\n");
        }
    }
}
static void handle_charging_status(void)
{
    static bool last_charge_state = false;
    bool current_charge_state = (gpio_pin_get_dt(&charger_gpio) == 1);
    
    // Only update if changed
    if (current_charge_state != last_charge_state) {
        last_charge_state = current_charge_state;
        strcpy((char *)charging_status_string, current_charge_state ? "Charging" : "Not Charging");
        notify_charging_status();
    }
}
static void handle_ble_notifications(void)
{
    static uint32_t last_notification = 0;
    
    // Send notifications every 2 seconds instead of every loop iteration
    if (k_uptime_get_32() - last_notification >= 2000) {
        notify_battery_voltage();
        notify_battery_percent();
        last_notification = k_uptime_get_32();
    }
}

/* ---------- Optional: one place to initialize everything ---------- */
static int appInit(void)
{
    // Hardware initialization
    if (pwm_init_all() != 0 || button_init() != 0 || charger_init() != 0 || ionizer_init() != 0) {
    return -1;
    }
    
    // Initial LED selection based on charging status
    last_charging_state = (gpio_pin_get_dt(&charger_gpio) == 1);
    active_ch = 0; // Always start with Blue LED
    apply_pwm();

    // Initialize NVS
    if (init_nvs() != 0) {
        printk("NVS initialization failed\n");
        return -1;
    }
    // Load mode times from flash
    if (load_mode_times() != 0) {
        printk("Failed to load mode times\n");
        return -1;
    }

    LOG_INF("hardware initialized");
    return 0;
}

/* ───────────────────── Bring-up helpers ───────────────────── */
static void startAppWorkers(void)
{
   
    // After your other initializations
    set_ionizer(false);
    k_work_init_delayable(&button_hold_work, button_hold_handler);
    k_work_init_delayable(&slow_mode_toggle_work, slow_mode_toggle_handler);
    // k_work_init_delayable(&batteryWork,    batteryWorkHandler);

    /* start periodic battery sampling; cadence ~2s */
    // k_work_schedule(&batteryWork, K_MSEC(20));
}

K_THREAD_DEFINE(breathe_tid, 1024, duty_thread, NULL, NULL, NULL, 7, 0, 0);

/* --- Main entry point ----------------------------------------- */
int main(void)
{
    printk("Starting BLE Peripheral with Mode Time Storage\n");
    
// 1. Hardware first
    if (appInit() != 0) return -1;
// 2. Bluetooth stack second
    if (bt_enable(NULL)) return -1;
// 3. Bluetooth services third
    if (bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd))) return -1;
    advertising_active = true;

    printk("Advertising successfully started\n");
  
   /* Kick background workers */
    startAppWorkers();

    while (1) {
        handle_battery_monitoring();
        handle_mode_time_tracking();
        handle_ionizer_control();
        handle_advertising_management();
        handle_charging_status();
        handle_ble_notifications();

        // Calculate precise sleep time
        // uint32_t loop_time = k_uptime_get_32() - loop_start_time;
        // if (loop_time < 1000) {
        //     k_sleep(K_MSEC(1000 - loop_time));
        // }
         k_sleep(K_SECONDS(1));
    }
}