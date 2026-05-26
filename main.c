#include "platform.h"
#include "uart.h"
#include "queue.h"
#include "timer.h"
#include "gpio.h"
#include "leds.h"
#include "bmp280.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Define pins for external hardware
#define EXT_BTN	PA_1		// external button
#define EXT_LED	PA_2		// external led

// define modes
#define ACTIVE_MODE	0
#define ECO_MODE		1

// Global variables
Queue rx_queue;
uint32_t system_ticks = 0;
uint32_t initial_pressure_P0 = 101325;


// System States
#define ACTIVE_MODE 0
#define ECO_MODE 		1
uint8_t current_mode = ACTIVE_MODE;

// ISR Flags
bool toggle_mode_flag = false;
bool update_p0_flag = false;
bool timer_tick = false;

// Flags
bool iir_filter_on = false;
bool alarm_active = false;

// Cyclic table of 10 values
uint32_t pressure_history[10] = {0};
uint8_t history_idx = 0;

// ---- Interrupts ---
// uart
void uart_rx_isr(uint8_t rx){
	if (rx >= 0x0 && rx <= 0x7F) queue_enqueue(&rx_queue, rx);
}

// timer
void systick_isr_callback(void){
	timer_tick = true;
}

// gpio buttons
void my_gpio_isr_callback(int pin_index){
	if(pin_index == GET_PIN_INDEX(P_SW)){ // user button
		update_p0_flag = true;
	} else if (pin_index == GET_PIN_INDEX(EXT_BTN)){ // external button
		toggle_mode_flag = true;
	}
}

int main(){
	char print_buff[128];
	uint8_t rx_char;
	int32_t temp;
	uint32_t press;
	float relative_altitude = 0.0;
	uint32_t last_led_tick = 0, last_sensor_tick = 0;
	bool led_state = false;
	
	// initializations
	queue_init(&rx_queue, 128);
	
	// uart
	uart_init(115200);
	uart_set_rx_callback(uart_rx_isr);
	uart_enable();
	
	// User (onboard) led
	leds_init();
	
	// bmp280
	bmp280_init();
	
	// User(onboard) button
	gpio_set_mode(P_SW, Input);
	gpio_set_trigger(P_SW, Falling);
	gpio_set_callback(P_SW, my_gpio_isr_callback);
	
	// External button
	gpio_set_mode(EXT_BTN, Input);
	gpio_set_trigger(EXT_BTN, Falling);
	gpio_set_callback(EXT_BTN, my_gpio_isr_callback);
	
	// External Led
	gpio_set_mode(EXT_LED, Output);
	gpio_set(EXT_LED, 0);
	
	// Timer
	timer_init(250000); // 250ms
	timer_set_callback(systick_isr_callback);
	timer_enable();
	
	__enable_irq();
	
	while(1){		
		// timer handling
		if(timer_tick){
			timer_tick = false;
			system_ticks++;
		}
		
		uint32_t current_ticks = system_ticks;
		uint32_t led_int = (current_mode == ACTIVE_MODE) ? 1 : 8; 	// 250ms vs 2sec
		uint32_t sen_int = (current_mode == ACTIVE_MODE) ? 4 : 20;	// 1sec vs 5sec
		
		// uart handling
		while (queue_dequeue(&rx_queue, &rx_char)){
			if(rx_char == 'f'){
				iir_filter_on = !iir_filter_on;
				bmp280_set_filter(iir_filter_on ? BMP280_FILTER_16 : BMP280_FILTER_OFF);
				uart_print(iir_filter_on ? "Filter: ON\r\n" : "Filter: OFF\r\n");
			} else if (rx_char == 'c'){
				alarm_active = false;
				uart_print("Alarm Cleared\r\n");
			} else if (rx_char == 's'){
				// general status
				sprintf(print_buff, "Mode: %s, Filter: %s, P0: %d.%02d hPa\r\n", 
				current_mode ? "ECO" : "ACTIVE", iir_filter_on ? "ON" : "OFF", initial_pressure_P0/100, initial_pressure_P0%100);
				uart_print(print_buff);
				
				// print memory
				uart_print("--- Last 10 measurements (Newest to Oldest) ---\r\n");
				for(int i = 1; i <= 10; i++){
					// calculate inverse idx
					int idx = (history_idx - i + 10) % 10;
					
					if(pressure_history[idx] != 0){
						sprintf(print_buff, " > %d.%02d hPa\r\n", pressure_history[idx] / 100, pressure_history[idx]%100);
						uart_print(print_buff);
					}
				}
				uart_print("-----------------------------------------------\r\n");
			}
		}
		
		// external button (toggle mode)
		if(toggle_mode_flag){
			toggle_mode_flag = false;		
			current_mode = (current_mode == ACTIVE_MODE) ? ECO_MODE : ACTIVE_MODE;
			uart_print(current_mode == ACTIVE_MODE ? "Mode: ACTIVE\r\n" : "Mode: ECO\r\n");
		}
		
		// external led (alarm)
		if(alarm_active){
			gpio_set(EXT_LED, 1);
		} else {
			gpio_set(EXT_LED, 0);
		}
		
		// user(onboard) button (update P0)
		if(update_p0_flag){
			update_p0_flag = false;
			bmp280_trigger_forced_measurement();
			// maybe delay needed here
			bmp280_read_measurements(&temp, &press);
			initial_pressure_P0 = press;
			sprintf(print_buff,"New ground pressure set! P0: %d.%02d hPa\r\n", initial_pressure_P0/100, initial_pressure_P0%100);
			uart_print(print_buff);
		}
		
		// Onboard LED handling (Blink depending on Mode)
		if ((current_ticks - last_led_tick) >= led_int){
			last_led_tick = current_ticks;
			led_state = !led_state;
			leds_set(led_state, 0, 0);
		}
			
		// Sensor handling
		if ((current_ticks - last_sensor_tick) >= sen_int){
			last_sensor_tick = current_ticks;
			bmp280_trigger_forced_measurement();
			// maybe delay needed here
			bmp280_read_measurements(&temp, &press);
			relative_altitude = 44330 * (1.0 - pow(((float)press/(float)initial_pressure_P0), (1.0/5.255)));
			
			// Alarm logic
			bool condition1 = (temp > 3500);
			bool condition2 = (initial_pressure_P0 > press) && ((initial_pressure_P0 - press) > 1000);
			bool condition3 = false;
			if(current_mode == ACTIVE_MODE){
				uint32_t past_press = pressure_history[history_idx];
				if(past_press != 0 && past_press > press && (past_press - press) > 500) condition3 = true;
			} else{
				int past_idx = (history_idx + 8) % 10;
				uint32_t past_press = pressure_history[past_idx];
				if(past_press != 0 && past_press > press && (past_press - press) > 500) condition3 = true;
			}
			if(condition1 || condition2 || condition3) alarm_active = true;
			
			if(alarm_active){		// print alarm message
				uart_print("[ALERT] Extreme Conditions Detected!\r\n");
			} else {						// print measurements
				char sign = (relative_altitude < 0) ? '-' : ' ';
				int alt_int = (int)fabs(relative_altitude);
				int alt_frac = (int)(fabs(relative_altitude - alt_int) * 100);
				
				sprintf(print_buff, "Temp: %d.%02d C | Press: %d hPa | Alt: %c%d.%02d m\r\n", temp/100, temp % 100, press / 100, sign, alt_int, alt_frac);
				uart_print(print_buff);
			}
				
			
			pressure_history[history_idx] = press;
			history_idx = (history_idx + 1) % 10;		// mod 10 so it turn numbers >= 10 to 1-9
		}
		
		__WFI();
	}
}
			
	