#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "fsm.h"

#define GPIO_D4 2
#define GPIO_D8 15
#define GPIO_D3 0
#define PERIOD_TICK 100/portTICK_RATE_MS
#define REBOUND_TICK 100/portTICK_RATE_MS

int done0;

void task_blink(void* ignore);
int button_pressed(fsm_t* this);
int button_not_pressed(fsm_t* this);
void led_on(fsm_t* this);
void led_off(fsm_t* this);
void isr_gpio(void* arg);

enum fsm_state {
  LED_ON,
  LED_OFF,
};


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void task_blink(void* ignore)
{
    while(true) {
      GPIO_OUTPUT_SET(GPIO_D4, 1);
        vTaskDelay(500/portTICK_RATE_MS);
        GPIO_OUTPUT_SET(GPIO_D4, 0);
        vTaskDelay(500/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

int button_pressed(fsm_t* this) {
  return (GPIO_INPUT_GET(GPIO_D8)) /*|| (!GPIO_INPUT_GET(GPIO_D3))*/;
}

// int button_not_pressed(fsm_t* this) {
//   return (!GPIO_INPUT_GET(GPIO_D0));
// }

void led_on(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,0);
}

void led_off(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,1);
}

static fsm_trans_t interruptor[] = {
  { LED_OFF, button_pressed, LED_ON, led_on },
  { LED_ON, button_pressed, LED_OFF, led_off },
  {-1, NULL, -1, NULL},
};

void isr_gpio(void* arg) {
  static portTickType xLastISRTick0 = 0;
  uint32 status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  portTickType now = xTaskGetTickCount();
  if(status & BIT(0)) {
    if(now > xLastISRTick0){
      xLastISRTick0 = now + REBOUND_TICK;
      done0 = 1;
    }
  }
}

void inter(void* ignore) {
  gpio_intr_handler_register(isr_gpio, NULL);
   fsm_t* fsm = fsm_new(interruptor);
   led_off(fsm);
    portTickType xLastWakeTime;
   while(true){
      xLastWakeTime = xTaskGetTickCount();
     fsm_fire(fsm);
      vTaskDelayUntil(&xLastWakeTime, PERIOD_TICK);
   }
 }

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
  // Config pin as GPIO12
 PIN_FUNC_SELECT (PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO2);
 PIN_FUNC_SELECT(GPIO_PIN_REG_15, FUNC_GPIO15);
 // PIN_FUNC_SELECT(GPIO_PIN_REG_0, FUNC_GPIO0);


    xTaskCreate(&inter, "startup", 2048, NULL, 1, NULL);
}
