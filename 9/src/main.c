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

int active = false;

void task_blink(void* ignore);
int cod_correcto(fsm_t* this);
int cod_incorrecto(fsm_t* this);
int mirar_flag(fsm_t* this);
int timeout(fsm_t* this);
int sensor_detected(fsm_t* this);

static volatile portTickType xLastFlag;
volatile int flag;
volatile int sensor_detected_flag;
volatile int indice;
int cod_introducido[3];
int cod[3] = {1,2,3};

void led_on(fsm_t* this);
void update_code(fsm_t* this);
void next_index(fsm_t* this);
void alarm_on(fsm_t* this);
void alarm_off(fsm_t* this);
void isr_gpio(void* arg);

enum fsm_codigo {
  WAITING,
};

enum fsm_alarm {
  ALARM_ON,
  ALARM_OFF,
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

int mirar_flag(fsm_t* this) {
  if(flag) {
    GPIO_OUTPUT_SET(GPIO_D4,0);
  }else {
    GPIO_OUTPUT_SET(GPIO_D4,1);
  }
  return flag;
}

int cod_correcto(fsm_t* this) {
  if(!memcmp(cod, cod_introducido, 3)) {
    GPIO_OUTPUT_SET(GPIO_D4,0);
  }
  return (!memcmp(cod, cod_introducido, 3));
}

int cod_incorrecto(fsm_t* this) {
  if(memcmp(cod, cod_introducido, 3)) {
    GPIO_OUTPUT_SET(GPIO_D4,0);
  }
  return (memcmp(cod, cod_introducido, 3));
}

int sensor_detected(fsm_t* this) {
  return (sensor_detected_flag);
}

int timeout(fsm_t* this) {
  if(xLastFlag == 0){
    return 0;
  }
  if((xTaskGetTickCount() - xLastFlag) == 1000) {
    GPIO_OUTPUT_SET(GPIO_D4,0);
  }else {
    GPIO_OUTPUT_SET(GPIO_D4,1);
  }
  return((xTaskGetTickCount() - xLastFlag) == 1000);
}

void update_code(fsm_t* this) {
  xLastFlag = xTaskGetTickCount();
  cod_introducido[indice] ++;
  flag = 0;
}

void next_index(fsm_t* this) {
    indice++;
}

void led_on(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,0);
  sensor_detected_flag = 0;
}

void alarm_on(fsm_t* this) {
  memset(cod_introducido, 0, 3);
  active = true;
  indice = 0;
  flag = 0;
}

void alarm_off(fsm_t* this) {
  GPIO_OUTPUT_SET(GPIO_D4,1);
  memset(cod_introducido, 0, 3);
  active = false;
  indice = 0;
  flag = 0;
}

static fsm_trans_t interruptor[] = {
  { WAITING, cod_correcto, WAITING, alarm_on },
  { WAITING, mirar_flag, WAITING, update_code },
  { WAITING, timeout, WAITING, next_index },
  { WAITING, cod_incorrecto, WAITING, alarm_off },
  {-1, NULL, -1, NULL},
};

static fsm_trans_t alarm[] = {
  { ALARM_OFF, cod_correcto, ALARM_ON, alarm_on },
  { ALARM_ON, cod_correcto, ALARM_OFF, alarm_off },
  { ALARM_ON, sensor_detected, ALARM_ON, led_on },
  {-1, NULL, -1, NULL},
};

void isr_gpio(void* arg) {
  static portTickType xLastISRTick0 = 0;
  uint32 gpio_mask = _xt_read_ints();
  uint32 status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  portTickType now = xTaskGetTickCount();
  if(status & BIT(0)) {
    if(now > xLastISRTick0){
      xLastISRTick0 = now + REBOUND_TICK;
      flag = 1;
    }
  }
  if(status & BIT(15)) {
    if(now > xLastISRTick0){
      xLastISRTick0 = now + REBOUND_TICK;
      sensor_detected_flag = 1;
    }
  }
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status); //limpia la interrupcion status
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
   fsm_t* fsm_codigo_new = fsm_new(interruptor);
   fsm_t* fsm_alarm_new = fsm_new(alarm);
   memset(cod_introducido, 0, 3);
   mirar_flag(fsm_codigo_new);
   portTickType xLastWakeTime;
   while(true){
      xLastWakeTime = xTaskGetTickCount();
      fsm_fire(fsm_codigo_new);
      fsm_fire(fsm_alarm_new);
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
 PIN_FUNC_SELECT(GPIO_PIN_REG_0, FUNC_GPIO0);
 /*
 configuracion interrupciones
 */
/*
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
*/

    xTaskCreate(&inter, "startup", 2048, NULL, 1, NULL);
}
