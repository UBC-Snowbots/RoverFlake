#include <armFirmware.h>

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

// #define RING_BUF_SIZE 2048
uint8_t ring_buffer[RING_BUF_SIZE];

Axis axes[NUM_AXES];
// struct k_timer Axis::stepper_timer; -- still want to figure out exactly why this broke / fixed my code

// planning to switch this to a thread
struct k_timer comm_timer;

// timers
struct k_timer home_timer;
struct k_timer pingStepPosition_timer;
struct k_timer pingAnglePosition_timer;
struct k_timer stepAll_timer;


// struct k_msgq axes_msgqs[NUM_AXES];

bool homing_complete = false;

// K_MSGQ_DEFINE(axes_data_queue[0], sizeof(struct AxisData), 2, 4);
// K_MSGQ_DEFINE(axes_data_queue[1], sizeof(struct AxisData), 2, 4);
// K_MSGQ_DEFINE(axes_data_queue[2], sizeof(struct AxisData), 2, 4);
// K_MSGQ_DEFINE(axes_data_queue[3], sizeof(struct AxisData), 2, 4);
// K_MSGQ_DEFINE(axes_data_queue[4], sizeof(struct AxisData), 2, 4);
// K_MSGQ_DEFINE(axes_data_queue[5], sizeof(struct AxisData), 2, 4);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_CONFIGURATION_DEFINE(config_1,
						  USB_SCD_SELF_POWERED,
						  200);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "UBC ROVER");
USBD_DESC_PRODUCT_DEFINE(sample_product, "ARM MCU");
USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn, "0123456789ABCDEF");

USBD_DEVICE_DEFINE(sample_usbd,
				   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
				   0x2fe3, 0x0001);

static int enable_usb_device_next(void)
{
	int err;

	err = usbd_add_descriptor(&sample_usbd, &sample_lang);
	if (err)
	{
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
	if (err)
	{
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_product);
	if (err)
	{
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_sn);
	if (err)
	{
		LOG_ERR("Failed to initialize SN descriptor (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&sample_usbd, &config_1);
	if (err)
	{
		LOG_ERR("Failed to add configuration (%d)", err);
		return err;
	}

	err = usbd_register_class(&sample_usbd, "cdc_acm_0", 1);
	if (err)
	{
		LOG_ERR("Failed to register CDC ACM class (%d)", err);
		return err;
	}

	err = usbd_init(&sample_usbd);
	if (err)
	{
		LOG_ERR("Failed to initialize device support");
		return err;
	}

	err = usbd_enable(&sample_usbd);
	if (err)
	{
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */

static void interrupt_handler(const struct device *dev, void *user_data)
{

	ARG_UNUSED(user_data);
	static uint8_t cmd_buffer[RX_BUF_SIZE] = "";
	static size_t cmd_index = 0;
	uint8_t rx_buffer[RX_BUF_SIZE] = "";
	int recv_len = 0;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev))
	{
		if (uart_irq_rx_ready(dev))
		{	
			//k_timer_stop(&pingAnglePosition_timer); 

			
			recv_len = uart_fifo_read(dev, rx_buffer, sizeof(rx_buffer));
			//  if (recv_len > RX_BUF_SIZE) {
    		// // // Log an error or take corrective action
			// //sendMsg("Uart Parse Error/Overflow, rejecting message\n");
			// return;
			// }
			for (int i = 0; i < recv_len; i++)
			{
				if (cmd_index >= RX_BUF_SIZE - 1) {
                	cmd_index = 0; // Reset for next command
                //log an error or warning here
            }
				if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r' || rx_buffer[i] == '\0' ) //big error, forgot to put back in the '\r' and '\0', using minicom was all messed as I had '\n' as an endchar disabled??
				{
					cmd_buffer[cmd_index] = '\0'; // Null-terminate
					//double check it wont break everything
					if (cmd_index < RX_BUF_SIZE) {
                        parseCmd(cmd_buffer);
                    }
					cmd_index = 0; // Reset for next command
					i = recv_len;

				}
				else
				{
					cmd_buffer[cmd_index++] = rx_buffer[i];
					//cmd_index++;
				}
			}
			

		} 

		if (uart_irq_tx_ready(dev))
		{
			  if (uart_irq_tx_complete(dev)) {
			uint8_t buffer[TX_BUF_SIZE];
			int rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (rb_len > 0)
			{
				uart_fifo_fill(dev, buffer, rb_len);
			}
			else
			{
				uart_irq_tx_disable(dev);
			}
			  }
		}
		
	}

}
// Planning to sepperate steppers and uart into 2 different threads
// void uart_thread(void *arg1, void *arg2, void *arg3) {
//     // UART initialization code here

//     while (true) {
//         // Handle UART reception, parsing commands, etc.

//         // If a message related to an axis is received, enqueue it to the message queue
//         struct AxisData axis_data;
//         // Populate axis_data with the relevant information
//         k_msgq_put(&axis_data_queue, &axis_data, K_FOREVER);

//         // Handle UART transmission, sending responses, etc.
//     }
// }

void sendMsg(const char tx_msg[TX_BUF_SIZE])
{

	ring_buf_put(&ringbuf, (uint8_t *)tx_msg, strlen(tx_msg));
	uart_irq_tx_enable(dev);

}

void tx_enable_callback(struct k_timer *timer_id)
{
}

void pingStepPosition_timer_callback(struct k_timer *timer_id)
{

	char msg[TX_BUF_SIZE];
	sprintf(msg, "$my_stepP(%i, %i, %i, %i, %i, %i)\n\r\0", axes[0].step_pos, axes[1].step_pos, axes[2].step_pos, axes[3].step_pos, axes[4].step_pos, axes[5].step_pos);
	ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
	uart_irq_tx_enable(dev);
}

void pingAnglePosition_timer_callback(struct k_timer *timer_id)
{


	updateAngles();

char msg[TX_BUF_SIZE];
	sprintf(msg, "$my_angleP(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n\r\0", axes[0].angle_pos, axes[1].angle_pos, axes[2].angle_pos, axes[3].angle_pos, axes[4].angle_pos, axes[5].angle_pos);
	ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
	uart_irq_tx_enable(dev);
}

void updateAngles()
{
	for (int i = 0; i < NUM_AXES; i++)
	{

		axes[i].angle_pos = (float)(axes[i].step_pos / ppr[i] / red[i] * 360.00);
	}
}


    //P(120.00, 20.00, 10.00, 90.0, 70.30, 70.51)

// void limit_switch_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     	     char stat[TX_BUF_SIZE];
//            sprintf(stat, "Limit Switch for an axis trigged \n\r\0");
//             sendMsg(stat);
// 			k_sleep(K_MSEC(1000));
// 	for (int i = 0; i < NUM_AXES; i++) {
//      int value = get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]);
//         if (value > 0) { // assuming active high
//             char msg[TX_BUF_SIZE];
//            sprintf(msg, "Limit Switch for Axis %d, activated \n\r\0", axes[i].index);
//             sendMsg(msg);
//             // You can also add any additional handling for this specific axis here
//         }
//     }

// }

void parseCmd(uint8_t cmd[RX_BUF_SIZE])
{
		//k_timer_start(&pingAnglePosition_timer, K_NO_WAIT, K_MSEC(50)); 

		//sendMsg("Command Parsing \n");


	if (cmd[0] == '$' && cmd[strlen((char*)cmd) - 1] == ')') {
    // Parse the command

	int cmd_type = 0;

	//if(s)

	switch (cmd[1])
	{
	case HOME_CHAR:
		cmd_type = HOME; //example command buffer:   $h()
		parseHomeCmd(cmd);
		break;
	case 'P':
		cmd_type = ABSOLUTE_TARGET_POSITION;
		parseAbsoluteTargetPositionCmd(cmd); //example command buffer:   $P(0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f)
		break;
	case 'i':
		cmd_type = INCREMENTAL_TARGET_POSITION;
		parseIncrementalTargetPositionCmd(cmd); //example command buffer:   $p(0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f)
		break;
	case 'S':
		//cmd_type = INCREMENTAL_TARGET_POSITION;
		parseSettingCmd(cmd); //example command buffer:   $p(0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f)
		break;
	case 't':
		cmd_type = TEST_LIMITS;
		testLimits();
		break;
	case ANGLE_CALLBACK_CHAR:
		cmd_type = TEST_LIMITS;
		testLimits();
		break;
	case 'w':
		// debug cmd
		// stepAxis(arpo);
		sendMsg("Up\n");
		axes[0].step_des_pos = axes[0].step_pos + 50;
		break;
	case 'r':
		sendMsg("Down\n");

		// stepAxis(arpo);
		axes[0].step_des_pos = axes[0].step_pos - 50;

		break;
	default:
		sendMsg("Error: Unknown Command, Input Rejected\n");

		break; // Handle unknown command
	}
   
}else{
	sendMsg("Error: Invalid Command, Input Rejected\n");
	//char cmdrelay = (char *)cmd;
	//sendMsg(cmdrelay);

}
}
void parseSettingCmd(uint8_t cmd[RX_BUF_SIZE])
{
	//k_timer_stop(&pingAnglePosition_timer);
	// $Pv(1100, 11990, 110, 1130, 710, 470)

	char *start_ptr = (char *)cmd + 4; // Skip '$Sx('
	char *end_ptr;
	int i = 0;


	switch (cmd[2])
	{
	case 'v': // Set velocity setting
		sendMsg("Success: Velocity Settings Command, Setting Parsing\n");

		break;
	
	default:
		sendMsg("Error: Unkown Settings Command, Rejected\n");

		break;
	}


if(cmd[2] == 'v'){


	while (i < NUM_AXES && (end_ptr = strchr(start_ptr, ',')) != NULL)
	{
		char speed_str[10]; // Assume that the float will not exceed 9 characters
		strncpy(speed_str, start_ptr, end_ptr - start_ptr);
		speed_str[end_ptr - start_ptr] = '\0'; // Null-terminate
		axes[i].current_speed = atoi(speed_str);

		start_ptr = end_ptr + 1; // Move to the character after the comma
		i++;
	}

	// Last float (after the last comma and before the closing parenthesis)
	if (i < NUM_AXES)
	{
		end_ptr = strchr(start_ptr, ')');
		if (end_ptr)
		{
			char speed_str[10];
			strncpy(speed_str, start_ptr, end_ptr - start_ptr);
			speed_str[end_ptr - start_ptr] = '\0';
			axes[i].current_speed = atoi(speed_str);
			i++;
		}
	}
	if (i == NUM_AXES && arm_inited)
	{
		// All axes angles are in axes[i].des_angle_pos
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Speed Setting Update Accepted: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
	}
	else
	{
		// Handle the error if not all six angles are parsed
		// ...

		// Error handling: could not parse all 6 angles, or message is messed up.
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Speed Setting Update Rejected, incorrect syntax  or arm not init: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
		//k_timer_start(&pingAnglePosition_timer, K_MSEC(50), K_NO_WAIT);

		return;
	}

}

	// for (int i = 0; i < NUM_AXES; i++)
	// {
	// 	axes[i].step_des_pos = axes[i].des_angle_pos * red[i] * ppr[i] / 360.0;
	// 	//k_timer_start(&pingAnglePosition_timer, K_MSEC(POSITION_PING_MS_INTERVAL), K_NO_WAIT);
	// }
}

void testLimits()
{

	k_timer_start(&comm_timer, K_MSEC(100), K_NO_WAIT);
}
void comm_timer_callback(struct k_timer *timer_id)
{
	//k_timer_stop(&pingAnglePosition_timer);
	char msg[TX_BUF_SIZE];
	sprintf(msg, "Limit Switch Testing for Axes begin. Waiting for switch to activate... \n\r\0");
	ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
	uart_irq_tx_enable(dev);
	for (int i = 0; i < NUM_AXES; i++)
	{
		char tmpmsg[TX_BUF_SIZE];

		sprintf(tmpmsg, "Limit Switch %d, is %d.  \n\r\0", i + 1, get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]));

		sendMsg(tmpmsg);
	}

}

void home_timer_callback(struct k_timer *timer_id)
{
	//k_timer_stop(&pingAnglePosition_timer);


	homing_complete = true;
	for (int i = 0; i < NUM_AXES; i++)
	{
		if (!axes[i].homed)
		{
			homing_complete = false;
			break;
		}
	}


	if (!homing_complete)
	{
		arm_homing = true;
		for (int i = 0; i < NUM_AXES; i++)
		{
			if (!get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]) && axes[i].homed == false)
			{
				axes[i].homing = true;
				axes[i].current_speed = axes[i].home_speed;
				if (axes[i].home_dir)
				{
					axes[i].step_des_pos = axes[i].step_pos + 20;
				}
				else
				{
					axes[i].step_des_pos = axes[i].step_pos - 20;
				}
			}
			else
			{
				if (axes[i].homed == false)
				{

					axes[i].homed = true;
					axes[i].homing = false;
					axes[i].current_speed = axes[i].max_speed;
					axes[i].step_pos = 0;
					axes[i].step_des_pos = 0;
					char msg[TX_BUF_SIZE];
					sprintf(msg, "Axis %d is home. \n", i + 1);

					ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
					uart_irq_tx_enable(dev);
				}
			}
		}
	}
	else
	{
		if(arm_inited){
		sendMsg("Arm Ready\n");
		arm_inited = false;
		arm_homing = false;


		}else{
		// all axes homed, stop timer
		k_timer_stop(&home_timer);
		arm_inited = true;
		goto_preset(DEFAULT_POSITION);
		//k_sleep(K_MSEC(4000));
		sendMsg("Homing complete, returning to default position\n");
		k_timer_start(&home_timer, K_MSEC(7000), K_NO_WAIT);
		}
	}


}

void goto_preset(int position)
{

	switch (position)
	{
	case DEFAULT_POSITION:
		for (int i = 0; i < NUM_AXES; i++)
		{
			axes[i].step_des_pos = axes[i].preset_step_pos[DEFAULT_POSITION];
		}
		break;

	default:
		break;
	}

	// for (int i = 0; i < NUM_AXES; i++){
	// 	while(axes[i].steps_remaining > 0){
	// 		//k_sleep(K_MSE);
	// 	}
	// }
}

void parseAbsoluteTargetPositionCmd(uint8_t cmd[RX_BUF_SIZE])
{
	//k_timer_stop(&pingAnglePosition_timer);
	// P(11.0, 11.0, 11.0, 11.0, 11.0, 11.0)

	char *start_ptr = (char *)cmd + 3; // Skip '$P('
	char *end_ptr;
	int i = 0;

	while (i < NUM_AXES && (end_ptr = strchr(start_ptr, ',')) != NULL)
	{
		char angle_str[10]; // Assume that the float will not exceed 9 characters
		strncpy(angle_str, start_ptr, end_ptr - start_ptr);
		angle_str[end_ptr - start_ptr] = '\n'; // Null-terminate
		axes[i].des_angle_pos = atof(angle_str);

		start_ptr = end_ptr + 1; // Move to the character after the comma
		i++;
	}

	// Last float (after the last comma and before the closing parenthesis)
	if (i < NUM_AXES)
	{
		end_ptr = strchr(start_ptr, ')');
		if (end_ptr)
		{
			char angle_str[10];
			strncpy(angle_str, start_ptr, end_ptr - start_ptr);
			angle_str[end_ptr - start_ptr] = '\n';
			axes[i].des_angle_pos = atof(angle_str);
			i++;
		}
	}
	if (i == NUM_AXES && !arm_homing)
	{
		// All axes angles are in axes[i].des_angle_pos
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Absolute Target Position Command Accepted: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
	}
	else
	{
		// Handle the error if not all six angles are parsed
		// ...

		// Error handling: could not parse all 6 angles, or message is messed up.
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Absolute Target Position Command Rejected, incorrect syntax or arm not homed: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
		//k_timer_start(&pingAnglePosition_timer, K_MSEC(50), K_NO_WAIT);

		return;
	}

	for (int i = 0; i < NUM_AXES; i++)
	{
		axes[i].step_des_pos = axes[i].des_angle_pos * red[i] * ppr[i] / 360.0;
	}

}
void parseIncrementalTargetPositionCmd(uint8_t cmd[RX_BUF_SIZE])
{
	//k_timer_stop(&pingAnglePosition_timer);
	// P(11.0, 11.0, 11.0, 11.0, 11.0, 11.0)

	char *start_ptr = (char *)cmd + 3; // Skip '$P('
	char *end_ptr;
	int i = 0;

	while (i < NUM_AXES && (end_ptr = strchr(start_ptr, ',')) != NULL)
	{
		char angle_str[10]; // Assume that the float will not exceed 9 characters
		strncpy(angle_str, start_ptr, end_ptr - start_ptr);
		angle_str[end_ptr - start_ptr] = '\0'; // Null-terminate
			float angle = atof(angle_str);
			//if(angle > 0){
			axes[i].step_des_pos = axes[i].step_pos + (int)angle;
			//} else if(atof(angle_str) < 0){
			//axes[i].step_des_pos = axes[i].step_pos - (int)angle;

			//} else{
			//axes[i].step_des_pos = axes[i].step_pos;

			
			//}
		start_ptr = end_ptr + 1; // Move to the character after the comma
		i++;
	}

	// Last float (after the last comma and before the closing parenthesis)
	if (i < NUM_AXES)
	{
		end_ptr = strchr(start_ptr, ')');
		if (end_ptr)
		{
			char angle_str[10];
			strncpy(angle_str, start_ptr, end_ptr - start_ptr);
			angle_str[end_ptr - start_ptr] = '\0';
			float angle = atof(angle_str);
			//if(angle > 0){
			axes[i].step_des_pos = axes[i].step_pos + (int)angle;
			// } else if(atof(angle_str) < 0){
			// axes[i].step_des_pos = axes[i].step_pos - (int)angle;

			// } else{
			// axes[i].step_des_pos = axes[i].step_pos;

			// }
			
			
			i++;
		}
	}
	if (i == NUM_AXES && arm_inited)
	{
		// All axes angles are in axes[i].des_angle_pos
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Incremental Target Position Command Accepted: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
	}
	else
	{
		// Handle the error if not all six angles are parsed
		// ...

		// Error handling: could not parse all 6 angles, or message is messed up.
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Incremental Target Position Command Rejected, incorrect syntax or arm not homed: \n");

		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));
		uart_irq_tx_enable(dev);
		ring_buf_put(&ringbuf, cmd, RX_BUF_SIZE);
		uart_irq_tx_enable(dev);
		//k_timer_start(&pingAnglePosition_timer, K_MSEC(50), K_NO_WAIT);

		return;
	}

	// for (int i = 0; i < NUM_AXES; i++)
	// {
	// 	axes[i].step_des_pos = axes[i].des_angle_pos * red[i] * ppr[i] / 360.0;
	// 	//k_timer_start(&pingAnglePosition_timer, K_MSEC(POSITION_PING_MS_INTERVAL), K_NO_WAIT);
	// }
}

long angle_to_steps(float angle, int axis_index)
{

	return 100;
}
void parseHomeCmd(uint8_t homeCmd[RX_BUF_SIZE])
{
	bool switch_health = 1;
	// switch (homeCmd[1])
	// {
	// case 'A':
	// 	break;

	// default:

	// 	break;
	// }
	if (!arm_homing)
	{

		for (int i = 0; i < NUM_AXES; i++)
		{
			if (get_gpio(axes[i].LIMIT_PIN[0], axes[i].LIMIT_PIN[1]))
			{

				switch_health = 0;
			}
			else
			{
				axes[i].homed = 0;

				axes[i].step_pos = axes[i].max_step_pos;
			}
		}

		if (switch_health)
		{
			char msg[TX_BUF_SIZE];
			sendMsg("Homing Sequence Start\n");

			k_timer_start(&home_timer, K_NO_WAIT, K_MSEC(20));
		}
		else
		{
			char msg[TX_BUF_SIZE];
			sprintf(msg, "Homing Request Denied, Limit switch %d is pressed or broken\n");
			ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));

			uart_irq_tx_enable(dev);
		}

		// homeAllAxes();
	}
	else
	{
		char msg[TX_BUF_SIZE];
		sprintf(msg, "Homing Sequence In progress, request ignored\n");
		ring_buf_put(&ringbuf, (uint8_t *)msg, strlen(msg));

		uart_irq_tx_enable(dev);
	}
}

void homeAllAxes()
{
}

// void stepTo(){

//  }

void stepAll_timer_callback(struct k_timer *timer_id)
{
	for (int i = 0; i < NUM_AXES; i++)
	{
		stepAxis(i);
	}
}

void stepAxis(int axis)
{
	bool dir_signal;

	if (axes[axis].homing)
	{
		if (axes[axis].home_dir)
		{
			dir_signal = 1;
		}
		else
		{
			dir_signal = 0;
		}
	}
	else
	{
		if (axes[axis].step_des_pos > axes[axis].step_pos)
		{
			dir_signal = axes[axis].dir;
		}
		else
		{
			dir_signal = !axes[axis].dir;
		}
		if (axes[axis].step_des_pos > axes[axis].max_step_pos)
		{
			axes[axis].step_des_pos = axes[axis].max_step_pos;
		}
		if (axes[axis].step_des_pos < axes[axis].min_step_pos)
		{
			axes[axis].step_des_pos = axes[axis].min_step_pos;
		}
	}

	set_gpio(axes[axis].DIR_PIN[0], axes[axis].DIR_PIN[1], dir_signal);

	axes[axis].steps_remaining = abs(axes[axis].step_des_pos - axes[axis].step_pos);


	if (!axes[axis].moving)
	{
		k_timer_start(&axes[axis].stepper_timer, K_NO_WAIT, K_NO_WAIT);
	}

	

	// axes[axis].step_des_pos += steps;
	// axes[axis].steps_remaining = 100;//abs(axes[axis].step_pos - axes[axis].step_des_pos);

	// Code to move the motor steps
}

void set_gpio(int dev, int pin, int value)
{
	const struct device *tempdev;
	switch (dev)
	{
	case 1:
		tempdev = gpio1_dev;
		break;

	case 2:
		tempdev = gpio2_dev;

		break;
	case 3:
		tempdev = gpio3_dev;
		break;
	case 4:
		tempdev = gpio4_dev;
		break;
	default:
		break;
	}

	gpio_pin_set(tempdev, pin, value);
}

int get_gpio(int dev, int pin)
{
	const struct device *tempdev;
	switch (dev)
	{
	case 1:
		tempdev = gpio1_dev;
		break;

	case 2:
		tempdev = gpio2_dev;

		break;
	case 3:
		tempdev = gpio3_dev;
		break;
	case 4:
		tempdev = gpio4_dev;
		break;
	default:
		break;
	}

	return gpio_pin_get(tempdev, pin);
}

void stepper_timer_callback(struct k_timer *timer_id)
{
	for (int i = 0; i < NUM_AXES; i++)
	{
		if (timer_id == &axes[i].stepper_timer)
		{
			axes[i].moving = true;
			if (axes[i].steps_remaining > 0)
			{
				if (axes[i].pulse_state)
				{
					// End of pulse, prepare for the next one
					set_gpio(axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], 0);
					axes[i].steps_remaining--;
					if (axes[i].step_des_pos > axes[i].step_pos)
					{
						axes[i].step_pos++;
					}
					else
					{
						axes[i].step_pos--;
					}
				}
				else
				{
					// Start of pulse
					set_gpio(axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], 1);
				}

				// Toggle pulse state
				axes[i].pulse_state = !axes[i].pulse_state;

				// Reset the timer
				k_timer_start(&axes[i].stepper_timer, K_USEC(axes[i].current_speed), K_NO_WAIT);
			}
			else
			{
				// No steps remaining, stop the timer
				axes[i].moving = false;
				k_timer_stop(&axes[i].stepper_timer);
			}
		}
	}
}

int main(void)
{
	uint32_t baudrate, dtr = 0U;
	int ret;
	gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));
	gpio3_dev = DEVICE_DT_GET(DT_NODELABEL(gpio3));
	gpio4_dev = DEVICE_DT_GET(DT_NODELABEL(gpio4));
	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev))
	{
		// LOG_ERR("CDC ACM device not ready");
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	ret = enable_usb_device_next();
#else
	ret = usb_enable(NULL);
#endif

	if (ret != 0)
	{
		// LOG_ERR("Failed to enable USB");
		return 0;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	// LOG_INF("Wait for DTR");

	while (true)
	{
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr)
		{
			break;
		}
		else
		{
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}

	// LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	// ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	if (ret)
	{
		// LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	// ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
	// if (ret)
	// {
	// 	// LOG_WRN("Failed to set DSR, ret code %d", ret);
	// }

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
	{
		// LOG_WRN("Failed to get baudrate, ret code %d", ret);
	}
	else
	{
		// LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	k_sleep(K_MSEC(50));

	// //initiate axes, im sure there is a better way to orginize all this data

	axes[0].max_speed = 500;
	axes[1].max_speed = 800;
	axes[2].max_speed = 700;
	axes[3].max_speed = 700;
	axes[4].max_speed = 600;
	axes[5].max_speed = 5800;

	for (int i = 0; i < NUM_AXES; i++)
	{
		axes[i].index = i;

		axes[i].STEP_PIN[0] = stepGPIO_PIN[i][0];
		axes[i].STEP_PIN[1] = stepGPIO_PIN[i][1];
		axes[i].DIR_PIN[0] = dirGPIO_PIN[i][0];
		axes[i].DIR_PIN[1] = dirGPIO_PIN[i][1];

		axes[i].ENC_PIN_A = encPinA[i];
		axes[i].ENC_PIN_B = encPinB[i];

		axes[i].LIMIT_PIN[0] = limGPIO_PIN[i][0];
		axes[i].LIMIT_PIN[1] = limGPIO_PIN[i][1];

		axes[i].current_speed = axes[i].max_speed;

		axes[i].PPR = ppr[i];
		axes[i].REDUCTION = red[i];
		axes[i].steps_remaining = 0;
		axes[i].steps_remaining = 0;
		axes[i].pulse_state = false;
		axes[i].moving = false;
		axes[i].macrostep = 15;
		axes[i].dir = 1;
		axes[i].homing = 0;
		axes[i].attach(); // must be called after pins declared

		k_sleep(K_MSEC(50));

		char axmsg[TX_BUF_SIZE];

		sprintf(axmsg, "Axis %d attached at STEP: %d.%d, DIR: %d.%d, \n", i + 1, axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], axes[i].DIR_PIN[0], axes[i].DIR_PIN[1]);

		// memcpy((uint8_t)axmsg, tx_bufferr, 64);

		ring_buf_put(&ringbuf, (uint8_t *)axmsg, strlen(axmsg));

		uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending
		k_sleep(K_MSEC(50));
		k_timer_init(&axes[i].stepper_timer, stepper_timer_callback, NULL); // pass user data to callback

		// char buffer[100];
		//  sprintf(buffer, "Axis %d attached at STEP: %d, DIR: %d\n", i, axes[i].STEP_PIN, axes[i].DIR_PIN);
		//   print_uart(buffer);
	}

	k_timer_init(&comm_timer, comm_timer_callback, NULL);							// pass user data to callback
	k_timer_init(&home_timer, home_timer_callback, NULL);							// pass user data to callback
	k_timer_init(&stepAll_timer, stepAll_timer_callback, NULL);						// pass user data to callback
	k_timer_init(&pingAnglePosition_timer, pingAnglePosition_timer_callback, NULL); // pass user data to callback
	// k_timer_init(&pingPosition_timer, pingPosition_timer_callback, NULL); // pass user data to callback

	// can be cleaned up, but for now I'm leaving it like this
	axes[0].home_speed = 500;
	axes[1].home_speed = 800;
	axes[2].home_speed = 700;
	axes[3].home_speed = 700;
	axes[4].home_speed = 700;
	axes[5].home_speed = 7000;
	// 1 or 0 for dir setting
	axes[0].home_dir = 0;
	axes[1].home_dir = 1;
	axes[2].home_dir = 1;
	axes[3].home_dir = 0;
	axes[4].home_dir = 0;
	axes[5].home_dir = 1;

	axes[0].max_step_pos = 9000 - POSITION_STEP_LIMIT_THRESHOLD;
	axes[1].max_step_pos = 5000 - POSITION_STEP_LIMIT_THRESHOLD;
	axes[2].max_step_pos = 4000 - POSITION_STEP_LIMIT_THRESHOLD;
	axes[3].max_step_pos = 5000 - POSITION_STEP_LIMIT_THRESHOLD;
	axes[4].max_step_pos = 7000 - POSITION_STEP_LIMIT_THRESHOLD;
	axes[5].max_step_pos = 4000 - POSITION_STEP_LIMIT_THRESHOLD;

	for (int i = 0; i < NUM_AXES; i++)
	{
		axes[i].dir = !axes[i].home_dir;
	}

	axes[0].preset_step_pos[DEFAULT_POSITION] = 5000;
	axes[1].preset_step_pos[DEFAULT_POSITION] = 3500;
	axes[2].preset_step_pos[DEFAULT_POSITION] = 1500;
	axes[3].preset_step_pos[DEFAULT_POSITION] = 5100;
	axes[4].preset_step_pos[DEFAULT_POSITION] = 5000;
	axes[5].preset_step_pos[DEFAULT_POSITION] = 800;

	k_timer_start(&stepAll_timer, K_NO_WAIT, K_USEC(2000)); // should be 5-10 times slower than the slowest stepper -- nevermind
	//k_timer_start(&pingAnglePosition_timer, K_NO_WAIT, K_MSEC(25)); 

	//sendMsg("Arm Ready \n");

	// while (true) {
	// 	//check serial for msgs
	//     // uint8_t rx_buffer[64];
	//     // int rb_len = ring_buf_get(&ringbuf, rx_buffer, sizeof(rx_buffer));
	//     // if (rb_len) {
	//     //     // Process the received data here
	// 	// 	// uint8_t data_to_send[] = "MSG RECIEVED";
	// 	// 	// ring_buf_put(&ringbuf, data_to_send, sizeof(data_to_send) - 1);
	// 	// 	// uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending

	//     //     // buffer contains rb_len bytes of data
	//     // }
	//     // Sleep or yield if needed
	// 		//k_sleep(K_MSEC(500));

	// 		//stepAxis(0);

	// }

	return 0;
}
