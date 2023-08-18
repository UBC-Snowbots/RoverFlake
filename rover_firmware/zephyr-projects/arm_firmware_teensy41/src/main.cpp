

#include <../include/armFirmware.h>




//stepper
// #define DIR_PIN  27
// #define PUL_PIN  14

#define GEAR_RATIO 47.0
//#define MICROSTEP  400
#define STEPS_PER_REVOLUTION 200 //INCLUDES MICROSTEP Generally, stepper motors have 200 steps per revolution
#define FULL_STEP_SEQUENCE ((int)(STEPS_PER_REVOLUTION * GEAR_RATIO))


/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


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


int calc_steps_remaining(){
    int displacement = des_pos - pos;
    dir = (des_pos > pos);
//k_sleep(K_MSEC(5));
    return abs(displacement);

}

// void step() {


// if(!moving){
//     k_timer_start(&stepper_timer, K_NO_WAIT, K_NO_WAIT);
// }
//     // Set the direction
//     steps_remaining = calc_steps_remaining();

//     gpio_pin_set(gpio_dev, DIR_PIN, dir);
//     // Pulse the PUL pin for the given number of steps

// }

int main(void)
{

    if (!gpio_dev) {
        printk("Cannot find GPIO device!\n");
        return 0;
    }

Axis axes[NUM_AXES];

//initiate axes
for (int i = 0; i < NUM_AXES; i++){

    axes[i].STEP_PIN = stepPins[i];
    axes[i].DIR_PIN = dirPins[i];
    axes[i].ENC_PIN_A = dirPins[i];
    axes[i].ENC_PIN_B = dirPins[i];
    axes[i].PPR = dirPins[i];
    axes[i].REDUCTION = dirPins[i];
    axes[i].steps_remaining = 0;
    axes[i].angle = 0.0;
    axes[i].attach(); //must be called after pins declared


}
   

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
        //step();
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

       /// step();


	}


    

    }
	
	return 0;
}


