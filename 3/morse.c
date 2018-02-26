#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"

#define GPIO_D4 2

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

const char* morse(const char c)
{
  static const char* morse_ch[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
    "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
    "..-", "...-", ".--", "-..-", "-.--", "--.."
  };

  return morse_ch[c - 'a'];
}

/**
 * Activa el LED durante un tiempo determinado.
 */
void enableLED(int time)
{
  GPIO_OUTPUT_SET(GPIO_D4, 0);
    vTaskDelay(time/portTICK_RATE_MS);
  GPIO_OUTPUT_SET(GPIO_D4, 1);
}

/**
 * Copia en buf la versión en Morse del mensaje str, con un límite de n
 * caracteres.
 * Retorna la longitud de la string final
 */
int str2morse (char *buf, int n, const char* str)
{
  int dataWritten = 0;

  while (*str)
  {
    char characterToParse = *str;
    if (characterToParse == ' ')
    {
      strcat(buf, "  ");
    } else {
      const char* morseCode = morse(characterToParse);

      char* letter = malloc(strlen(morseCode)+2);
      strcpy(letter, morseCode);
      strcat(letter, "  ");
      int length = strlen(letter);

      dataWritten += length;
      if (dataWritten < n)
      {
        strcat(buf, letter);
      } else {
        dataWritten -= length;
        break;
      }
    }
    ++str;
  }

  return dataWritten;
}

/**
 * Envía el mensaje msg ya codificado en morse, encendiendo y apagando el led.
 */
void morse_send (const char* msg)
{
  switch (*msg) {
    case '.':
      // Activa el LED durante 250ms
      enableLED(250);
      // Espera 250ms.
      vTaskDelay(250/portTICK_RATE_MS);
      break;
    case '-':
      // Activa el led durante 750ms
      enableLED(750);
      // Espera 250ms.
      vTaskDelay(250/portTICK_RATE_MS);
      break;
    case ' ':
      // Espera 250ms.
      vTaskDelay(250/portTICK_RATE_MS);
      break;
    case '\0':
      return;
  }
  morse_send (++msg );
}

void init(const char* str)
{
  int n = 500;
  char buff[n];

  str2morse(buff, n, str);
  morse_send(buff);

  vTaskDelete(NULL);
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
  // Config pin as GPIO2
  PIN_FUNC_SELECT (PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO2);
  xTaskCreate(&init, "startup", 2048, "hola mundo", 1, NULL);
}
