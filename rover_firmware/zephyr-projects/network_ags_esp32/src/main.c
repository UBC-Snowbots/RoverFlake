#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <stdio.h>
// #include <sys/printk.h>

#include <string.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

//stepper
#define DIR_PIN  27
#define PUL_PIN  14

#define GEAR_RATIO 47.0
//#define MICROSTEP  400
#define STEPS_PER_REVOLUTION 200 //INCLUDES MICROSTEP Generally, stepper motors have 200 steps per revolution
#define FULL_STEP_SEQUENCE ((int)(STEPS_PER_REVOLUTION * GEAR_RATIO))


/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_CONFIGURATION_DEFINE(config_1,
			  USB_SCD_SELF_POWERED,
			  200);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "UBC ROVER");
USBD_DESC_PRODUCT_DEFINE(sample_product, "Z USBD CDC ACM");
USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn, "0123456789ABCDEF");

USBD_DEVICE_DEFINE(sample_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0001);




static int enable_usb_device_next(void)
{
	int err;

	err = usbd_add_descriptor(&sample_usbd, &sample_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
	if (err) {
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_product);
	if (err) {
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_sn);
	if (err) {
		LOG_ERR("Failed to initialize SN descriptor (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&sample_usbd, &config_1);
	if (err) {
		LOG_ERR("Failed to add configuration (%d)", err);
		return err;
	}

	err = usbd_register_class(&sample_usbd, "cdc_acm_0", 1);
	if (err) {
		LOG_ERR("Failed to register CDC ACM class (%d)", err);
		return err;
	}

	err = usbd_init(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to initialize device support");
		return err;
	}

	err = usbd_enable(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */


static struct k_timer stepper_timer;

//this stepper controls the yaw of the stepper, following right hand rule (your thumb would be the antenna dish)
const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

static int steps_remaining = 0;

static int des_pos = 0;
static int pos = 0;
static bool dir = 0;

static bool moving = 0;

/* receive buffer used in UART ISR callback */

static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
    //character, ASICI
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

/*
    step steppers

*/
static void stepper_timer_callback(struct k_timer *timer_id) {
    static bool pulse_state = false;
    moving = true;
    if (steps_remaining > 0) {
        if (pulse_state) {
            // End of pulse, prepare for the next one
            gpio_pin_set(gpio_dev, PUL_PIN, 0);
            steps_remaining--;    
              if(des_pos > pos){
pos++;    } else{
        pos --;
    }      
            printk("moving from %d, approaching %d\n", pos, des_pos);
  
        } else {
            // Start of pulse
            gpio_pin_set(gpio_dev, PUL_PIN, 1);
        }

        // Toggle pulse state
        pulse_state = !pulse_state;

        // Reset the timer
        k_timer_start(&stepper_timer, K_MSEC(1), K_NO_WAIT);
    } else {
        // No steps remaining, stop the timer
        k_timer_stop(&stepper_timer);
        moving = false;
        printk("destination approached");

    }
}

int calc_steps_remaining(){
    int displacement = des_pos - pos;
    dir = (des_pos > pos);
//k_sleep(K_MSEC(5));
    return abs(displacement);

}

void step() {


if(!moving){
    k_timer_start(&stepper_timer, K_NO_WAIT, K_NO_WAIT);
}
    // Set the direction
    steps_remaining = calc_steps_remaining();

    gpio_pin_set(gpio_dev, DIR_PIN, dir);
    // Pulse the PUL pin for the given number of steps

}

int main(void)
{
    enable_usb_device_next();

     if (!gpio_dev) {
        printk("Cannot find GPIO device!\n");
        return 0;
    }
    int hi;
    hi = gpio_pin_configure(gpio_dev, PUL_PIN, GPIO_OUTPUT);
	hi = hi ? hi : gpio_pin_configure(gpio_dev, DIR_PIN, GPIO_OUTPUT);

	if (hi) {
		printk("Error %d: failed to configure GPIO pins\n", hi);
		return 0;
	}
    k_timer_init(&stepper_timer, stepper_timer_callback, NULL);




	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	print_uart("Network AGS System Online.\r\n");

	/* indefinitely wait for input from the user */
    while(true){

    
    if (k_msgq_get(&uart_msgq, &tx_buf, K_NO_WAIT) == 0) {
       
        
        
        float angle = strtof(tx_buf, NULL);
    if (angle < 0.0f || angle > 360.0f) {
        print_uart("Angle out of range.\r\n");
    } else {
        des_pos = (int)(angle / 360.0 * FULL_STEP_SEQUENCE);
        step();
    }

		print_uart("Echo: ");
		print_uart(tx_buf);
		print_uart("\r\n");

        if(strcmp((char*)tx_buf, "pos") == 0){
            printk("Current Position: %d", pos);
         }


        //des_pos = atoi(tx_buf);

       

        // if(strcmp((char*)tx_buf, "R") == 0){
        // step(true, FULL_STEP_SEQUENCE);  // Full rotation clockwise
        // printk("Moving CounterClockwise\n");

        // }

        step();


	}


    

    }
	
	return 0;
}


