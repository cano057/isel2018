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

enum fsm_state {
  WAITING,
  NUM_1,
  NUM_2,
  NUM_3,
  NUM_4,
  NUM_5,
  NUM_6,
  NUM_7,
  NUM_8,
  NUM_9,
  NUM_10,
  ALARM_ACTIVE,
  NUM_1_ALARM,
  NUM_2_ALARM,
  NUM_3_ALARM,
  NUM_4_ALARM,
  NUM_5_ALARM,
  NUM_6_ALARM,
  NUM_7_ALARM,
  NUM_8_ALARM,
  NUM_9_ALARM,
  NUM_10_ALARM,
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
  if(cod_introducido[indice] >= 10) {
    if (indice < 2) {
      indice ++;
    }
    else {
      return;
    }
  }
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
  { WAITING, cod_correcto, ALARM_ACTIVE, alarm_on },
  { WAITING, cod_incorrecto, WAITING, alarm_off },
  { WAITING, mirar_flag, NUM_1, update_code },
  { NUM_1, timeout, WAITING, next_index },
  { NUM_1, mirar_flag, NUM_2, update_code },
  { NUM_2, timeout, WAITING, next_index },
  { NUM_2, mirar_flag, NUM_3, update_code },
  { NUM_3, timeout, WAITING, next_index },
  { NUM_3, mirar_flag, NUM_4, update_code },
  { NUM_4, timeout, WAITING, next_index },
  { NUM_4, mirar_flag, NUM_5, update_code },
  { NUM_5, timeout, WAITING, next_index },
  { NUM_5, mirar_flag, NUM_6, update_code },
  { NUM_6, timeout, WAITING, next_index },
  { NUM_6, mirar_flag, NUM_7, update_code },
  { NUM_7, timeout, WAITING, next_index },
  { NUM_7, mirar_flag, NUM_8, update_code },
  { NUM_8, timeout, WAITING, next_index },
  { NUM_8, mirar_flag, NUM_9, update_code },
  { NUM_9, timeout, WAITING, next_index },
  { NUM_9, mirar_flag, NUM_10, update_code },
  { NUM_10, timeout, WAITING, next_index },
  { NUM_10, mirar_flag, NUM_1, update_code },
  { ALARM_ACTIVE, sensor_detected, ALARM_ACTIVE, led_on },
  { ALARM_ACTIVE, cod_correcto, WAITING, alarm_off },
  { ALARM_ACTIVE, cod_incorrecto, ALARM_ACTIVE, alarm_on },
  { ALARM_ACTIVE, mirar_flag, NUM_1_ALARM, update_code },
  { NUM_1_ALARM, timeout, WAITING, next_index },
  { NUM_1_ALARM, mirar_flag, NUM_2_ALARM, update_code },
  { NUM_2_ALARM, timeout, WAITING, next_index },
  { NUM_2_ALARM, mirar_flag, NUM_3_ALARM, update_code },
  { NUM_3_ALARM, timeout, WAITING, next_index },
  { NUM_3_ALARM, mirar_flag, NUM_4_ALARM, update_code },
  { NUM_4_ALARM, timeout, WAITING, next_index },
  { NUM_4_ALARM, mirar_flag, NUM_5_ALARM, update_code },
  { NUM_5_ALARM, timeout, WAITING, next_index },
  { NUM_5_ALARM, mirar_flag, NUM_6_ALARM, update_code },
  { NUM_6_ALARM, timeout, WAITING, next_index },
  { NUM_6_ALARM, mirar_flag, NUM_7_ALARM, update_code },
  { NUM_7_ALARM, timeout, WAITING, next_index },
  { NUM_7_ALARM, mirar_flag, NUM_8_ALARM, update_code },
  { NUM_8_ALARM, timeout, WAITING, next_index },
  { NUM_8_ALARM, mirar_flag, NUM_9_ALARM, update_code },
  { NUM_9_ALARM, timeout, WAITING, next_index },
  { NUM_9_ALARM, mirar_flag, NUM_10_ALARM, update_code },
  { NUM_10_ALARM, timeout, WAITING, next_index },
  { NUM_10_ALARM, mirar_flag, NUM_1_ALARM, update_code },
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
   fsm_t* fsm = fsm_new(interruptor);
   memset(cod_introducido, 0, 3);
   mirar_flag(fsm);
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
