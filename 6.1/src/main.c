#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "fsm.h"

#define GPIO_D4 2
#define GPIO_D8 15
#define GPIO_D3 0
#define PERIOD_TICK 100/portTICK_RATE_MS
#define REBOUND_TICK 100/portTICK_RATE_MS
#define ETS_GPIO_INTR_ENABLE() \
  _xt_isr_unmask(1 << ETS_GPIO_INUM)
#define ETS_GPIO_INTR_DISABLE() \
  _xt_isr_,ask(1 << ETS_GPIO_INUM)

int done0;
int active = false;

void task_blink(void* ignore);
int button_pressed(fsm_t* this);
int button_unpressed(fsm_t* this);
int sensor_detected(fsm_t* this);
int sensor_undetected(fsm_t* this);
void led_on(fsm_t* this);
void led_off(fsm_t* this);
void alarm_on(fsm_t* this);
void alarm_off(fsm_t* this);
void isr_gpio(void* arg);

enum fsm_state {
  WAITING,
  ALARM_ACTIVE,
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
  GPIO_OUTPUT_SET(GPIO_D4, 1);
    vTaskDelay(50/portTICK_RATE_MS);
    GPIO_OUTPUT_SET(GPIO_D4, 0);
    vTaskDelay(50/portTICK_RATE_MS);
}

int button_pressed(fsm_t* this) {
  return (GPIO_INPUT_GET(GPIO_D8));
}

int button_unpressed(fsm_t* this) {
  return (!GPIO_INPUT_GET(GPIO_D8));
}

int sensor_detected(fsm_t* this) {
  return (!GPIO_INPUT_GET(GPIO_D3));
}

int sensor_undetected(fsm_t* this) {
  return (GPIO_INPUT_GET(GPIO_D3));
}

void led_on(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,0);
}

void led_off(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,1);
}

void alarm_on(fsm_t* this) {
  /*codigo de activacion de la alarma por control remoto
  */
}

void alarm_off(fsm_t* this) {
  /*codigo de desactivacion de la alarma por control remoto
  */
}

static fsm_trans_t interruptor[] = {
  { WAITING, button_pressed, ALARM_ACTIVE, alarm_on },
  { ALARM_ACTIVE, button_unpressed, WAITING, alarm_off },
  { ALARM_ACTIVE, sensor_detected, ALARM_ACTIVE, led_on },
  { ALARM_ACTIVE, sensor_undetected, ALARM_ACTIVE, led_off },
  {-1, NULL, -1, NULL},
};

void isr_gpio(void* arg) {
  static portTickType xLastISRTick0 = 0;
  uint32 gpio_mask = _xt_read_ints();
  uint32 status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status); //limpia la interrupcion status
  portTickType now = xTaskGetTickCount();
  if(status & BIT(0)) {
    if(now > xLastISRTick0){
      xLastISRTick0 = now + REBOUND_TICK;
      done0 = 1;
    }
  }
}

void inter(void* ignore) {
  /* configuración interrupciones
  */
  gpio_intr_handler_register((void*)isr_gpio, NULL);
  gpio_pin_intr_state_set(0, GPIO_PIN_INTR_NEGEDGE);
  gpio_pin_intr_state_set(15, GPIO_PIN_INTR_POSEDGE);
  ETS_GPIO_INTR_ENABLE();
  /*configuracion maquina de estados
  */
   fsm_t* fsm = fsm_new(interruptor);
   led_off(fsm);
  //  portTickType xLastWakeTime;
   while(true){
    //  xLastWakeTime = xTaskGetTickCount();
     fsm_fire(fsm);
    //  vTaskDelayUntil(&xLastWakeTime, PERIOD_TICK);
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
 PIN_FUNC_SELECT(GPIO_PIN_REG_0, FUNC_GPIO0);
 /*
 configuarcion interrupciones
 */
GPIO_ConfigTypeDef io_conf2;
io_conf2.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;
io_conf2.GPIO_Mode = GPIO_Mode_Input;
io_conf2.GPIO_Pin = BIT(0);
io_conf2.GPIO_Pullup = GPIO_PullUp_DIS;
gpio_config(&io_conf2);

GPIO_ConfigTypeDef io_conf15;
io_conf15.GPIO_IntrType = GPIO_PIN_INTR_POSEDGE;
io_conf15.GPIO_Mode = GPIO_Mode_Input;
io_conf15.GPIO_Pin = BIT(15);
io_conf15.GPIO_Pullup = GPIO_PullUp_DIS;
gpio_config(&io_conf15);

    xTaskCreate(&inter, "startup", 2048, NULL, 1, NULL);
}
