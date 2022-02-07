/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

//interrupt
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/timer.h"
//interrupt

//mqtt headers
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
//mqtt headers


#include "sdkconfig.h"
#include "abc.h"
#include "filter.h"
//#include "components/bib/include/abc.h"


static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */

//#define FDC1004_SENSOR_ADDR 0x50
#define FDC1004_SENSOR_REG_ADDR_MANUFACTURER 0xFE	// este registro deberia leerse: 0x5449

#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */























//_______________________________________ PARAMETROS GLOBALES MODICAR ACA:
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA:
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA:
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA:

#define NUMERO_DE_SENSOR 		2
#define ORDEN_APROXIMACION		3

#define ALARM_MS 			50
#define cantMedidas 		50
#define CAPDAC_MID_RANGE 	23
#define CAPDAC_MAX			31
#define CAPDAC_MIN			0


#define CAPDAC_DEFAULT		16


#define SAMPLE_RATE_DEFAULT 	CUATROCIENTAS_Ss //CUATROCIENTAS_Ss
#define N_MUESTRAS_MEDIA		8
#define DELAY_ENTREMUESTRAS_US	2707
















// COEFICIENTES SN1
#if NUMERO_DE_SENSOR == 1
	#define NUMERO_DE_SENSOR_STR	"SN1"			// usado para el topic de calibracion, si el topic "cal" recibe "SN1" significa que sensor 1 se pone en 0mm.
	#define SECOND_ORDER_COEF_A		0.27665
	#define SECOND_ORDER_COEF_B		-13.2921
	#define SECOND_ORDER_COEF_C		274.7861

	#define TERCER_ORDEN_COEF_A		0.0091781
	#define TERCER_ORDEN_COEF_B		-1.0918
	#define TERCER_ORDEN_COEF_C		53.4195
	#define TERCER_ORDEN_COEF_D		-786.6967

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif








// COEFICIENTES SN2
#if NUMERO_DE_SENSOR == 2
	#define NUMERO_DE_SENSOR_STR	"SN2"			// usado para el topic de calibracion, si el topic "cal" recibe "SN2" significa que sensor 2 se pone en 0mm.
	#define SECOND_ORDER_COEF_A		0.22605
	#define SECOND_ORDER_COEF_B		-10.2967
	#define SECOND_ORDER_COEF_C		256.2716

	#define TERCER_ORDEN_COEF_A		0.0034596
	#define TERCER_ORDEN_COEF_B		-0.31804
	#define TERCER_ORDEN_COEF_C		17.7812
	#define TERCER_ORDEN_COEF_D		-218.6046

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif




// COEFICIENTES SN3
#if NUMERO_DE_SENSOR == 3
	#define NUMERO_DE_SENSOR_STR	"SN3"			// usado para el topic de calibracion, si el topic "cal" recibe "SN3" significa que sensor 3 se pone en 0mm.
	#define SECOND_ORDER_COEF_A
	#define SECOND_ORDER_COEF_B
	#define SECOND_ORDER_COEF_C

	#define TERCER_ORDEN_COEF_A		1
	#define TERCER_ORDEN_COEF_B		1
	#define TERCER_ORDEN_COEF_C		1
	#define TERCER_ORDEN_COEF_D		1

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif
















// COEFICIENTES SN4
#if NUMERO_DE_SENSOR == 4
	#define NUMERO_DE_SENSOR_STR	"SN4"			// usado para el topic de calibracion, si el topic "cal" recibe "SN4" significa que sensor 4 se pone en 0mm.
	#define SECOND_ORDER_COEF_A 	0.21533
	#define SECOND_ORDER_COEF_B 	-9.4471
	#define SECOND_ORDER_COEF_C 	198.9251

	#define TERCER_ORDEN_COEF_A		0.00058628
	#define TERCER_ORDEN_COEF_B		0.1246
	#define TERCER_ORDEN_COEF_C		-4.8513
	#define TERCER_ORDEN_COEF_D		122.8554

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif
















// COEFICIENTES SN5
#if NUMERO_DE_SENSOR == 5
	#define NUMERO_DE_SENSOR_STR	"SN5"			// usado para el topic de calibracion, si el topic "cal" recibe "SN5" significa que sensor 5 se pone en 0mm.
	#define SECOND_ORDER_COEF_A 	0.19314
	#define SECOND_ORDER_COEF_B 	-6.9808
	#define SECOND_ORDER_COEF_C 	139.5849

	#define TERCER_ORDEN_COEF_A		0.0054548
	#define TERCER_ORDEN_COEF_B		-0.64696
	#define TERCER_ORDEN_COEF_C		35.3998
	#define TERCER_ORDEN_COEF_D		-559.7443

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif













// COEFICIENTES SN6
#if NUMERO_DE_SENSOR == 6
	#define NUMERO_DE_SENSOR_STR	"SN6"			// usado para el topic de calibracion, si el topic "cal" recibe "SN6" significa que sensor 6 se pone en 0mm.
	#define SECOND_ORDER_COEF_A 	0.21859
	#define SECOND_ORDER_COEF_B 	-9.7166
	#define SECOND_ORDER_COEF_C 	199.7951

	#define TERCER_ORDEN_COEF_A		-0.00034509
	#define TERCER_ORDEN_COEF_B		0.27244
	#define TERCER_ORDEN_COEF_C		-12.4694
	#define TERCER_ORDEN_COEF_D		245.8257

	#if ORDEN_APROXIMACION == 3
		#define COEF_A	TERCER_ORDEN_COEF_A
		#define COEF_B	TERCER_ORDEN_COEF_B
		#define COEF_C	TERCER_ORDEN_COEF_C
		#define COEF_D	TERCER_ORDEN_COEF_D
	#else
		#define COEF_A	SECOND_ORDER_COEF_A
		#define COEF_B	SECOND_ORDER_COEF_B
		#define COEF_C	SECOND_ORDER_COEF_C
		#define COEF_D	0
	#endif
#endif












#define UPPER_RANGE_CORRECTION_OFFSET 0 //1.91
#define LOWER_RANGE_CORRECTION_OFFSET_FALLING_LVL 0.5



//_______________________________________ PARAMETROS GLOBALES MODICAR ACA.
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA.
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA.
//_______________________________________ PARAMETROS GLOBALES MODICAR ACA.




#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (( 80*1000000 ) / TIMER_DIVIDER)  // convert counter value to seconds	//lo reemplace porque no me lo tomaba al define original.

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;

static xQueueHandle s_mqtt_packets_queue;


struct capacity_packets_struct{
	uint32_t packet_id;
	int heigh[50];
	uint8_t capdac;
};





/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
/*
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}
*/

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    if (!info->auto_reload) {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_milisec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_milisec * TIMER_SCALE / 1000);
    //timer_enable_intr(group, timer);


    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_milisec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_disable_intr(group, timer);		// inicia deshabilitado
    timer_start(group, timer);

}







SemaphoreHandle_t ultimaMedida_mux = NULL;

esp_mqtt_client_handle_t client;

uint8_t midiendo= 0;	// no necesitan semaforo, uno lee y el otro escribe y no intercambian roles (timer y gpio ISRs).
uint8_t parar= 0;		// no necesitan semaforo, uno lee y el otro escribe y no intercambian roles (timer y gpio ISRs).



#define GPIO_OUTPUT_IO_0    4
//#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ( 1ULL<<GPIO_OUTPUT_IO_0 )
#define GPIO_INPUT_IO_0     5
//#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ( 1ULL<<GPIO_INPUT_IO_0 )
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;





static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}





static void mqtt_send_packets_task(void* arg){


	char packetID_str[12];	//el 12avo elemento es la ',' que separa packetID y ensayoID.
	char ensayoID_str[11];
	char heigh_str[5];	//"+hhh\0"
	char dataToPublish[380];	// son [363] pero por si calcule algo mal, le doy 380.
	char topic[50];
	uint32_t ensayoID= -1;	// Cada ensayo consta de varios paquetes sincronizados. El primer paquete de cada ensayo tiene ID=0. Arranca en -1 para compensar primer incremento.

	sprintf(topic, "/topic/nivel/sensor_%d", NUMERO_DE_SENSOR);

	struct capacity_packets_struct packet;


	while(1){

		if( xQueueReceive(s_mqtt_packets_queue, &packet, 10000/portTICK_RATE_MS)== pdTRUE){



			strcpy(dataToPublish, "[");			// no me deja poner dentro de sprintf.

			if(packet.packet_id==0){
				ensayoID++;
			}

			sprintf(ensayoID_str, "%d", ensayoID);
			strcat(dataToPublish, ensayoID_str);

			sprintf(packetID_str, ",%d", packet.packet_id);
			strcat(dataToPublish, packetID_str);


			for(int i=0; i<cantMedidas; i++){
				strcat(dataToPublish, ",");
				sprintf(heigh_str, "%d", packet.heigh[i]);
				strcat(dataToPublish, heigh_str);
			}

			strcat(dataToPublish, "]");			// no me deja poner dentro de sprintf..
			//printf("####################\ndatos a publicar: %s\n######################\n", dataToPublish);
			esp_mqtt_client_publish(client, topic, dataToPublish, 0, 1, 0);
			//printf("publico<<<<\n");
			//printf("\n\npaquete: %d enviado\n", packet.packet_id);														//COMENTADO POR DEBUG, NUMERO DE PAQUETE ENVIADO





		}
	}


}






static void gpio_task(void* arg)								// VER DIAGRAMA DE FLUJO
{
    uint32_t io_num;
    uint8_t gpio_level;


    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {


        	gpio_level= gpio_get_level(io_num);
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_level);



            if(gpio_level){			//POSITIVE EDGE

            	if(midiendo){
            		parar= 0;
            	}else{

            		//no reconfiguro alarm time porque siempre uso 50ms tanto para antirebote como para enviar datos.
            		//caso que se necesite que sean tiempos distintos, agregar aca configuracion de alarm time.
            		parar= 0;
            		timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
            		timer_enable_intr(TIMER_GROUP_0, TIMER_0);

            	}



            }else{					//NEGATIVE EDGE

            	if(midiendo){
            		parar= 1;
            	}else{
            		timer_disable_intr(TIMER_GROUP_0, TIMER_0);
            	}



            }



        }
    }
}






SampleFilter signalFilter_struct;
float last_filtered_cap;
uint16_t mm_mediaMovil;
uint16_t mm_offset_cal= 0;			// el offset de nivel al calibrar.

static void timer_task(void* arg)							// VER DIAGRAMA DE FLUJO
{
	uint8_t syncTest= 0;
	example_timer_event_t evt;

	char topic[50];
	sprintf(topic, "/topic/nivel/sensor_%d", NUMERO_DE_SENSOR);

	uint8_t pararConfirmado= 0;
															//Para toma de medidas:
	int8_t capdac= 0;
	uint8_t sampleNumber= 0;
	uint32_t packetID= 0;
	//char packetID_str[11];
	//char capacidad_str[7];	//"ccc.cc\0"
	//char dataToPublish[352];						// el formato a enviar: "[ccc.cc,ccc.cc,...,...]\0" -> 50 medidas -> 349+3 = 352.


	//float cap_temp;
	float capacidad[50];


	//float desviacionAceptable= 0.2;
	//uint8_t cantMuestras= 5;
	//mean_reliability estructuraResultado;


	struct capacity_packets_struct packets;


	//static unsigned char rango= 'M';	// para debug. borrar despues
	//unsigned char rangoAnterior= 'M';
	//int8_t rangoLPF= 0;

															//Para toma de medidas.

	//float temp;				// temporal que uso para hacer una media movil


    for(;;) {

    	if(xQueueReceive(s_timer_queue, &evt, portMAX_DELAY)){





    		if(parar){
    			pararConfirmado++;
    		}else{
    			pararConfirmado= 0;
    		}

    		midiendo= 1;






    		//BLOQUE QUE TOMA UNA SOLA MEDIDA CON CAPDAC FIJO:
			//read_single_cap_pF(&capacidad[sampleNumber], medidaNIVEL);
			//capacidad[sampleNumber] = capacidad[sampleNumber]+3.125*CAPDAC_DEFAULT;
			//FIN BLOQUE QUE TOMA UNA SOLA MEDIDA CON CAPDAC FIJO







    		//BLOQUE MEDIA DE n MEDIDAS:
    		/*
    		capacidad[sampleNumber] = 0;
    		for (int i = 0; i < N_MUESTRAS_MEDIA; i++) {														//
    			read_single_cap_pF(&temp, medidaNIVEL);
    			capacidad[sampleNumber] += temp;
    			usleep(DELAY_ENTREMUESTRAS_US);								//11ms para 100Ss					//
    		}
    		capacidad[sampleNumber] /= N_MUESTRAS_MEDIA;														//
    		capacidad[sampleNumber] = capacidad[sampleNumber] + 3.125 * CAPDAC_DEFAULT;
			*/
    		//MEDIA DE n MEDIDAS




    		//BLOQUE QUE FILTRA CON FIR

    		//SampleFilter_put(&signalFilter_struct, capacidad[sampleNumber]);
    		//capacidad[sampleNumber]= SampleFilter_get(&signalFilter_struct);

    		//BLOQUE QUE FILTRA CON FIR






    		//PARA TENER UN MUESTREO UNIFORME Y DE 400Ss, SE MIDE Y SE FILTRA EN UNA TAREA DISTINTA.

    		xSemaphoreTake(ultimaMedida_mux, portMAX_DELAY);
    		capacidad[sampleNumber]= last_filtered_cap;
    		xSemaphoreGive(ultimaMedida_mux);

    		//FIN BLOQUE QUE LEE CAPACIDAD FILTRADA OBTENIDA DE OTRA TAREA.















			//BLOQUE QUE HACE MEDIA MOVIL



			//BLOQUE QUE HACE MEDIA MOVIL













    		// TOMA DE MEDIDA:


    		/* EN TODAS LAS MUESTRAS HACE AUTORANGING, ASI QUE REMPLAZO ESTE BLOQUE POR EL SIGUIENTE.__________________________________________________
    		if(sampleNumber==0){				// medida de nivel AUTORANGO la primer medida, despues medidas normales:
				capdac= read_autoranging_cap_pF(capacidad, medidaNIVEL);
				//printf("primer medida, autoranging\ncapacidad: %0.2f\ncapdac: %d\n", capacidad[0]+capdac*3.125, capdac);
    		}else{
    			//read_single_cap_pF(&capacidad[sampleNumber], medidaNIVEL);
    			read_processed_cap_pF(medidaNIVEL, desviacionAceptable, cantMuestras, &estructuraResultado);
    			capacidad[sampleNumber]= estructuraResultado.mean;
    		}
    		*///EN TODAS LAS MUESTRAS HACE AUTORANGING, ASI QUE REMPLAZO ESTE BLOQUE POR EL SIGUIENTE._________________________________________________





    		// ESTE BLOQUE HACE AUTORANGING EN TODAS LAS MEDIDAS (el anterior lo hacia solo en la muestra 0):____________________________
    		//capdac= read_autoranging_cap_pF(&capacidad[sampleNumber], medidaNIVEL);		//
    		// FIN DE BLOQUE AUTORANGING EN TODAS LAS MUESTRAS.__________________________________________________________________________



    		/*
    		//BLOQUE DE 3 RANGOS DE 28.125pF:______________________________________

    		capdac= CAPDAC_MID_RANGE;
    		MEASn_capdac_config(capdac, medidaNIVEL);
    		usleep(8000);		// si este delay es menor a 6ms no alcanza para configurar el offset, y usa el offset anterior al configurado en la linea anterior.
    		read_single_cap_pF(&capacidad[sampleNumber], medidaNIVEL);


    		if(capacidad[sampleNumber]<-12.1){

    			//printf("s: %d cambio al rango inferior vvvv     saturo con C: %f\n", sampleNumber, capacidad[sampleNumber]);		//debug

    			//debug:
    			if(rango!='L'){
    				rango='L';
    				printf("cambio de rango a L\n");
    			}
    			//fin debug.


    			capdac= CAPDAC_MID_RANGE-6;		// lo empaqueta abajo, hay que actualizarlo.						//cambie de 9 a 8 por debug volver a 9.
    			if(capdac<CAPDAC_MIN){
    				printf("CAPDAC MENOR AL MINIMO CAPDAC.<<<<<<<<<<<<<<<<<<\n");
    				capdac= CAPDAC_MIN;
    			}

    			//printf("saturo con: %.2f y capdac: %d\n", capacidad[sampleNumber], capdac+9);
    			MEASn_capdac_config(capdac, medidaNIVEL);
    			usleep(8000);	// si falla aumentar este delay														//alargue el delay por debug. volver a 8ms
    			read_single_cap_pF(&capacidad[sampleNumber], medidaNIVEL);

    			//if(rangoAnterior=='M')
    				capacidad[sampleNumber]= capacidad[sampleNumber]+3.125*capdac+LOWER_RANGE_CORRECTION_OFFSET_FALLING_LVL;
    			//else
    				//capacidad[sampleNumber]= capacidad[sampleNumber]+3.125*capdac+LOWER_RANGE_CORRECTION_OFFSET_RISING_LVL;

    			//rangoAnterior= 'L';

    			//printf("la nueva medida en L es: C: %f, capdac: %d _-_-_-_-_-_-_-_-_-_-_-_-\n", capacidad[sampleNumber], capdac);

    		}else if(12.1<capacidad[sampleNumber]){

    			//printf("s: %d cambio al rango superior ^^^^     saturo con C: %f\n", sampleNumber, capacidad[sampleNumber]);		//debug

    			//debug:
				if(rango!='H'){
					rango='H';

					printf("cambio de rango a H, cap saturo en C: %.2f con capdac: %d\n", capacidad[sampleNumber], capdac);
				}
				//fin debug.


				capdac= CAPDAC_MID_RANGE+6;		// lo empaqueta abajo, hay que actualizarlo.
				if( CAPDAC_MAX < capdac ){
					capdac= CAPDAC_MAX;
				}


				MEASn_capdac_config(capdac, medidaNIVEL);
				usleep(8000);	// si falla aumentar este delay
				read_single_cap_pF(&capacidad[sampleNumber], medidaNIVEL);
				capacidad[sampleNumber]= capacidad[sampleNumber]+3.125*capdac+UPPER_RANGE_CORRECTION_OFFSET;

				//printf("la nueva medida en H es: C: %.2f con capdac: %d _-_-_-_-_-_-_-_-_-_-_-_-\n", capacidad[sampleNumber], capdac);







    		}else{

    			capacidad[sampleNumber]= capacidad[sampleNumber]+3.125*capdac;

    			//printf("s: %d rango medio-----------------       C: %f\n", sampleNumber, capacidad[sampleNumber]);
    			//debug:
				if(rango!='M'){
					rango='M';
					printf("la nueva medida en M es: C: %.2f con capdac: %d _-_-_-_-_-_-_-_-_-_-_-_-\n", capacidad[sampleNumber], capdac);
				}

				//fin debug.
    		}
    		//printf("CAPACIDAD ABSOLUTA: %f##########################################\n", capacidad[sampleNumber]+capdac*3.125);
    		//FIN DE BLOQUE DE 3 RANGOS DE 28.125pF________________________________
    		*/


    		//printf("capdac: %d, Coffset: %f\n<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", capdac, capdac*3.125);


    		sampleNumber++;





    		if( (cantMedidas-1) <sampleNumber){


    			//debug para testear que tan bien se sincronizan.
				syncTest= !syncTest;
    			gpio_set_level(GPIO_OUTPUT_IO_0, syncTest);
				//debug para testear que tan bien se sincronizan.


    			//printf("\n\n");																										// COMENTADO PARA PROBAR EL FILTRO, DESCOMENTAR PARA IMPRIMIR EL PAQUETE.
    			for(int j=0; j<50; j++){
    				packets.heigh[j]= cap_to_mm(ORDEN_APROXIMACION, capacidad[j], COEF_A, COEF_B, COEF_C, COEF_D);

    				packets.heigh[j]-= mm_offset_cal;

    				//printf(",%.2f", capacidad[j]);																						// COMENTADO PARA PROBAR EL FILTRO, DESCOMENTAR PARA IMPRIMIR EL PAQUETE.
    				//ESP_LOGI("capacidad==>", ",%.2f", capacidad[j]);
    				ESP_LOGI("altura==>", ",%d", packets.heigh[j]);
    			}
    			ESP_LOGI("altura==>","\n\n\n\n_________________________________\n\n\n\n");															// COMENTADO PARA PROBAR EL FILTRO, DESCOMENTAR PARA IMPRIMIR EL PAQUETE.


    			//__________________queue__________________________________

				packets.packet_id= packetID;
				packets.capdac= capdac;

				xQueueSend(s_mqtt_packets_queue, &packets, 5/portTICK_RATE_MS);
				//__________________queue__________________________________



				packetID++;

/*
    			//ENVIAR. ESTA PARTE MUEVO COMPLETA A UNA TAREA NUEVA____________________________________________________________________________________
				strcpy(dataToPublish, "[");			// no me deja poner dentro de sprintf.

				sprintf(packetID_str, "%d", packetID++);
				strcat(dataToPublish, packetID_str);


				for(int i=0; i<sampleNumber; i++){
					strcat(dataToPublish, ",");
					sprintf(capacidad_str, "%.2f", capacidad[i]+3.125*capdac);
					strcat(dataToPublish, capacidad_str);
				}

				strcat(dataToPublish, "]");			// no me deja poner dentro de sprintf..
				//printf("####################\ndatos a publicar: %s\n######################\n", dataToPublish);
				//esp_mqtt_client_publish(client, topic, dataToPublish, 0, 1, 0);
				//printf("publico<<<<\n");
				//FIN ENVIAR. ESTA PARTE MUEVO COMPLETA A UNA TAREA NUEVA________________________________________________________________________________
*/
				sampleNumber= 0;

				if(1<pararConfirmado){
					midiendo= 0;
					packetID= 0;
					timer_disable_intr(TIMER_GROUP_0, TIMER_0);
				}

    		}




			//vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);




    		// FIN TOMA DE MEDIDAS.





    	}
    }
}







/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @note cannot use master read slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 * @note cannot use master write slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
















static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}














esp_mqtt_client_handle_t client;


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
	char msg[50];	//escribo la cadena para no contar y errarle
	char topicRecibido[50];
	char stringRecibida[50];

	sprintf(msg, "Sensor %d conectado", NUMERO_DE_SENSOR);

    //esp_mqtt_client_handle_t client = event->client;
	client = event->client;

    //int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_publish(client, "/topic/nivel/connected_sensors", msg, 0, 1, 0);

            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            esp_mqtt_client_subscribe(client, "/topic/nivel/cal", 0);					// se suscribe al topic de calibracion


            //msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);					// no necesita suscribirse.
            //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");					// no se suscribio, entonces no se des-suscribe.
            //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        //case MQTT_EVENT_SUBSCRIBED:
            //ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            //break;

        /*
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            */

        case MQTT_EVENT_PUBLISHED:
            //ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;


        case MQTT_EVENT_DATA:
            //ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);

            // ACA COMPARAR SI LO QUE RECIBIO ES == "SN4" (para sensor 4, para cada sensor) topic: "/topic/nivel/cal"
            if( (event->topic_len) < 40){
            	sprintf(topicRecibido, "%.*s", event->topic_len, event->topic);
            	if( !strcmp(topicRecibido, "/topic/nivel/cal") ){
            		if( (event->data_len) < 40 ){
            			sprintf(stringRecibida, "%.*s", event->data_len, event->data);
            			if( !strcmp(stringRecibida, NUMERO_DE_SENSOR_STR) ){
							mm_offset_cal= mm_mediaMovil;
            			}
            		}
            	}
            }

            break;


        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}






static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}









static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}











/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)				// la pase dentro de capacimeter_config()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          //!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here.
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}





static void vLevelMeasureTask(void *arg)
{

    //uint32_t task_idx = (uint32_t)arg;
    float cap;

    float mediaMovil;
    uint16_t altura_mediaMovil;
    float mediaMovil_sample[20]= {0};		// hago la media movil de la capacidad filtrada, para obtener un valor medio, se necesita para una mejor calibracion.
    uint8_t mediaMovil_ix= 0;


    while (1) {

    	read_single_cap_pF(&cap, medidaNIVEL);
    	cap= cap+3.125*CAPDAC_DEFAULT;

       SampleFilter_put(&signalFilter_struct, cap);

       xSemaphoreTake(ultimaMedida_mux, portMAX_DELAY);
       last_filtered_cap= (float)SampleFilter_get(&signalFilter_struct);
       xSemaphoreGive(ultimaMedida_mux);




       //La media movil es unicamente para la calibracion de 0mm. No tiene nada que ver con las medidas de capacidad ni con el filtro que esta en esta misma tarea.
       if(19<mediaMovil_ix)
    	   mediaMovil_ix= 0;

       mediaMovil_sample[mediaMovil_ix++]= last_filtered_cap;

       mediaMovil= 0;
       for(int n=0; n<20; n++){
    	   mediaMovil+= mediaMovil_sample[n];
       }
       mediaMovil/=20;

       altura_mediaMovil= cap_to_mm(ORDEN_APROXIMACION, mediaMovil, COEF_A, COEF_B, COEF_C, COEF_D);

       mm_mediaMovil= altura_mediaMovil;




        usleep(DELAY_ENTREMUESTRAS_US);
    }

    vSemaphoreDelete(ultimaMedida_mux);
    vTaskDelete(NULL);

}






void app_main(void)
{
    ultimaMedida_mux = xSemaphoreCreateMutex();

    SampleFilter_init(&signalFilter_struct);

    //timer
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, ALARM_MS);
    //example_tg_timer_init(TIMER_GROUP_1, TIMER_0, false, 5);
    //timer






    //mqtt
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(example_connect());	// This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
										// Read "Establishing Wi-Fi or Ethernet Connection" section in examples/protocols/README.md for more information about this function.

	mqtt_app_start();
    //mqtt


	/*
    ESP_ERROR_CHECK(i2c_master_init());
    capacimeter_init(I2C_MASTER_NUM, 0, CUATROCIENTAS_Ss);		// antes abria el puerto, ahora unicamente ejecuta la configuracion inicial (meas_conf y cap_config).
	usleep(10000);	//lo que sigue tarda, puede ser prescindible este delay.
	capacimeter_config(CUATROCIENTAS_Ss, medidaNIVEL);
	usleep(8000);												//afinar este delay


    //xTaskCreate(vLevelMeasureTask, "vLevelMeasureTask_0", 1024 * 8, (void *)0, 10, NULL);
    //xTaskCreate(vLevelMeasureTask, "vLevelMeasureTask_1", 1024 * 2, (void *)1, 10, NULL);
	*/



	ESP_ERROR_CHECK(i2c_master_init());
	capacimeter_init(I2C_MASTER_NUM, 0, SAMPLE_RATE_DEFAULT);// antes abria el puerto, ahora unicamente ejecuta la configuracion inicial (meas_conf y cap_config).
	usleep(10000);	//lo que sigue tarda, puede ser prescindible este delay.
	capacimeter_config(SAMPLE_RATE_DEFAULT, medidaNIVEL);
	usleep(8000);											//afinar este delay
	MEASn_capdac_config(CAPDAC_DEFAULT, medidaNIVEL);
	usleep(8000);// si este delay es menor a 6ms no alcanza para configurar el offset, y usa el offset anterior al configurado en la linea anterior.

	float descartada;
	read_single_cap_pF(&descartada, medidaNIVEL);	// descarto la primer muestra
	usleep(8000);
	xTaskCreate(vLevelMeasureTask, "vLevelMeasureTask_0", 1024 * 8, (void *)0, 10, NULL);
	//xTaskCreate(vLevelMeasureTask, "vLevelMeasureTask_1", 1024 * 2, (void *)1, 10, NULL);











    // salida para ver si se sincronizan correctamente varios ESP32
    gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);




	// pin como entrada, interrupcion:
	//interrupt of any edge
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	//bit mask of the pins, use GPIO5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en= 0;
	gpio_config(&io_conf);



	//create a queue to handle sensor packets
	s_mqtt_packets_queue= xQueueCreate(8, sizeof(struct capacity_packets_struct));
	//start mqtt from queue task
	xTaskCreate(mqtt_send_packets_task, "mqttFromQueue_task", 4096, NULL, 10, NULL);
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
	//start timer task
	xTaskCreate(timer_task, "timer_task", 4096, NULL, 10, NULL);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);



	timer_enable_intr(TIMER_GROUP_0, TIMER_0);


	/*
	float filter_out;
	float filter_in;

	SampleFilter filtroPrueba;
	SampleFilter_init(&filtroPrueba);
	*/

	while(1){


		//PRUEBA DE FILTRO:
		/*
		for(int m=0; m<400; m++){
			filter_in= sine_1hz[m] + 0.5*sine_2hz[m] + 0.25*sine_5hz[m] + 0.125*sine_10hz[m]+0.5*white_noise[m];
			SampleFilter_put(&filtroPrueba, filter_in);
			filter_out= (float)SampleFilter_get(&filtroPrueba);
			//printf("%.2f,", filter_output);
			ESP_LOGI("DEBUG..","%.2f,", filter_out);



		}
		//printf("\n\n");
		ESP_LOGI("DEBUG..", "\n\n\n\n\n\n\n\n============================================================================================================\n\n\n\n\n\n\n\n");
		*/
		//FIN PRUEBA DE FILTRO.




		vTaskDelay(500/portTICK_RATE_MS);

	}


}

















