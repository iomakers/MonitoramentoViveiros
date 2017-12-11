

/**
 * Automação de Horta usando ESP32
 * Autor: João Campos
 * Sensores Utilizados
 * 2 PT_100
 * 2 HC-SR04
 * 1 sensor de chuva
 * 1 DHT11
 * 2 Sensor de Vazao
 *
 * Tutoriais Utilizados
 * http://randomnerdtutorials.com/esp32-dht11-dht22-temperature-humidity-web-server-arduino-ide/
 * https://github.com/espressif/esp-idf/issues/199
 * https://github.com/adafruit/DHT-sensor-library/issues/62
 * https://www.arduinoecia.com.br/2014/06/sensor-de-chuva-arduino.html
 * https://github.com/Ebiroll/esp32_ultra/blob/master/main/main.cpp
 * http://labdegaragem.com/profiles/blogs/tutorial-como-utilizar-o-sensor-de-fluxo-de-agua
 */


#include <Modbus.h>
#include <ModbusIP_ESP32.h>
#include <sys/time.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


void floatToWordArray(float , word * );
static void ultraDistanceTask(void *inpar);
uint32_t get_usec();


float distance_1 = 0;


// Sensor usado é o DHT11
#define DHTTYPE DHT11
#define ECHO_PIN GPIO_NUM_4
#define TRIG_PIN GPIO_NUM_15

// DHT Sensor
const int DHTPin = 16;
int porta_temperatura_pt100_0 = 36;
int porta_temperatura_pt100_1 = 39;
int porta_ph_0 = 14;
int porta_ph_1 = 12;
int porta_chuva = 27;
int porta_bomba_0 = 9;
int porta_bomba_1 = 10;



// Inicializar DHT sensor.
DHT dht(DHTPin, DHTTYPE);

//ModbusIP object
ModbusIP mb;


//Declaração de INPUT registers
const int SENSOR_HUMIDADE_0 = 0;
const int SENSOR_HUMIDADE_1 = 1;
const int SENSOR_TEMPERATURA_DHT_0 = 2;
const int SENSOR_TEMPERATURA_DHT_1 = 3;
const int SENSOR_TEMPERATURA_PT100_0_0 = 4;
const int SENSOR_TEMPERATURA_PT100_0_1 = 5;
const int SENSOR_TEMPERATURA_PT100_1_0 = 6;
const int SENSOR_TEMPERATURA_PT100_1_1 = 7;
const int SENSOR_DISTANCE_0_0 = 8;
const int SENSOR_DISTANCE_0_1 = 9;
const int SENSOR_DISTANCE_1_0 = 10;
const int SENSOR_DISTANCE_1_1 = 11;
const int SENSOR_PH_0_0 = 12;
const int SENSOR_PH_0_1 = 13;
const int SENSOR_PH_1_0 = 14;
const int SENSOR_PH_1_1 = 15;
const int SENSOR_RAIN = 16;
const int SENSOR_VAZAO_1 = 17;
const int SENSOR_VAZAO_2 = 18;
//Declaração de COIL registers
const int COIL_0 = 0;
const int COIL_1 = 1;




volatile int pulsos_vazao = 0;

// Declarando variáveis
// Porta do sensor de humidade de solo 1

//int porta_humidade_solo_2 = A1;
//int porta_humidade_solo_3 = A2;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // It is safe to use digitalRead/Write here if you want to toggle an output
}


// Interrupt handler
void IRAM_ATTR gpio_isr_handler_up(void* arg)
{
  pulsos_vazao++;
  portYIELD_FROM_ISR();
}



void setup() {
  Serial.begin(115200);
    delay(10);
 // Criar semáforo para checar no loop
  timerSemaphore = xSemaphoreCreateBinary();
  //Config Modbus IP
    mb.config("ScadaBR", "ScadaBRunp");


     while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Declarar tipos de portas
  pinMode(porta_temperatura_pt100_0, INPUT);
  pinMode(porta_temperatura_pt100_1, INPUT);
  pinMode(porta_ph_0, INPUT);
  pinMode(porta_ph_1, INPUT);
  pinMode(porta_chuva, INPUT);
  pinMode(porta_bomba_0, OUTPUT);
  pinMode(porta_bomba_1, OUTPUT);


    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Colocar alarme para a função onTimer ser chamada a cada 10 s (valor em microsegundos).
  // Repetir alarme (=true)
  timerAlarmWrite(timer, 10000000, true);

  // Adicionar input registers
  mb.addIreg(SENSOR_HUMIDADE_0);
  mb.addIreg(SENSOR_HUMIDADE_1);
  mb.addIreg(SENSOR_TEMPERATURA_DHT_0);
  mb.addIreg(SENSOR_TEMPERATURA_DHT_0);
  mb.addIreg(SENSOR_TEMPERATURA_PT100_0_0);
  mb.addIreg(SENSOR_TEMPERATURA_PT100_0_1);
  mb.addIreg(SENSOR_TEMPERATURA_PT100_1_0);
  mb.addIreg(SENSOR_TEMPERATURA_PT100_1_1);

  mb.addIreg(SENSOR_DISTANCE_0_0);
  mb.addIreg(SENSOR_DISTANCE_0_1);
  mb.addIreg(SENSOR_DISTANCE_1_0);
  mb.addIreg(SENSOR_DISTANCE_1_1);

  mb.addIreg(SENSOR_PH_0_0 );
  mb.addIreg(SENSOR_PH_0_1);
  mb.addIreg(SENSOR_PH_1_0);
  mb.addIreg(SENSOR_PH_1_1);

  mb.addIreg(SENSOR_RAIN);

// Atuadores a relé
   mb.addCoil(COIL_0);
   mb.addCoil(COIL_1);

  // Iniciar alarme
  timerAlarmEnable(timer);

  xTaskCreatePinnedToCore(&ultraDistanceTask, "ultra", 4096, NULL, 20, NULL, 0);
   // setting up the input GPIO 17 for Rising edge interrupt
  gpio_set_direction(GPIO_NUM_17, GPIO_MODE_INPUT);
  gpio_set_intr_type(GPIO_NUM_17, GPIO_INTR_NEGEDGE);
  gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);
  gpio_intr_enable(GPIO_NUM_17);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(GPIO_NUM_17, gpio_isr_handler_up, (void*) GPIO_NUM_17);

}

void loop() {
  //Call once inside loop() - all magic here
   mb.task();
    digitalWrite(porta_bomba_0, mb.Coil(COIL_0));
     digitalWrite(porta_bomba_1, mb.Coil(COIL_1));

  // semáforo foi disparado pelo Hardware timer
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){

    float h = dht.readHumidity();
      word registro_humidade[2];
      floatToWordArray(h, registro_humidade);
      mb.Ireg(SENSOR_HUMIDADE_0, registro_humidade[0]);
      mb.Ireg(SENSOR_HUMIDADE_1, registro_humidade[1]);
      // Read temperature as Celsius (the default)
      float t = dht.readTemperature();
      word registro_temperatura[2];
      floatToWordArray(h, registro_temperatura);

      mb.Ireg(SENSOR_TEMPERATURA_DHT_0, registro_temperatura[0]);
      mb.Ireg(SENSOR_TEMPERATURA_DHT_0, registro_temperatura[1]);
      word registro[2];

      // Leitura dos dois pt100
     float pt100_0 = analogRead(porta_temperatura_pt100_0);

      floatToWordArray(pt100_0, registro);

      mb.Ireg(SENSOR_TEMPERATURA_PT100_0_0, registro[0]);
      mb.Ireg(SENSOR_TEMPERATURA_PT100_0_1, registro[1]);


     float pt100_1 = analogRead(porta_temperatura_pt100_1);
     floatToWordArray(pt100_1, registro);

      mb.Ireg(SENSOR_TEMPERATURA_PT100_1_0, registro[0]);
      mb.Ireg(SENSOR_TEMPERATURA_PT100_1_1, registro[1]);

     // Leitura dos dois sensores PH
     float ph_0 = analogRead(porta_ph_0);

      floatToWordArray(pt100_0, registro);

      mb.Ireg(SENSOR_PH_0_0, registro[0]);
      mb.Ireg(SENSOR_PH_0_1, registro[1]);


     float ph_1 = analogRead(porta_ph_1);
     floatToWordArray(pt100_0, registro);

      mb.Ireg(SENSOR_PH_1_0, registro[0]);
      mb.Ireg(SENSOR_PH_1_1, registro[1]);

      // Leitura dos sensor de Chuva
     int chuva = analogRead(porta_chuva);
      mb.Ireg(SENSOR_RAIN, chuva);







      pulsos_vazao = 0;


  }

  // put your main code here, to run repeatedly:

}

////
//// Toggle trig pin and wait for input on echo pin
////
static void ultraDistanceTask(void *inpar) {

    gpio_pad_select_gpio(TRIG_PIN);
    gpio_pad_select_gpio(ECHO_PIN);

    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    while(1) {
        // HC-SR04P
        gpio_set_level(TRIG_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(TRIG_PIN, 0);
        uint32_t startTime=get_usec();

        while (gpio_get_level(ECHO_PIN)==0 && get_usec()-startTime < 500*1000)
        {
            // Wait until echo goes high
        }

        startTime=get_usec();

        while (gpio_get_level(ECHO_PIN)==1 && get_usec()-startTime < 500*1000)
        {
            // Wait until echo goes low again
        }

        if (gpio_get_level(ECHO_PIN) == 0)
        {
            uint32_t diff = get_usec() - startTime; // Diff time in uSecs
            // Distance is TimeEchoInSeconds * SpeeOfSound / 2
            distance_1 = 340.29 * diff / (1000 * 1000 * 2); // Distance in meters
           // printf("Distance is %f cm\n", distance * 100);
        }
        else
        {
            // No value
            printf("Did not receive a response!\n");
        }
        // Delay and re run.
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}



void floatToWordArray(float number, word *reg){
 byte*  ArrayOfFourBytes;
 ArrayOfFourBytes = (byte*) &number;
 reg[0] = (ArrayOfFourBytes[1]<<8)| ArrayOfFourBytes[0];
 reg[1] = (ArrayOfFourBytes[3]<<8)| ArrayOfFourBytes[2];

}


// Similar to uint32_t system_get_time(void)
uint32_t get_usec() {

 struct timeval tv;

 //              struct timeval {
 //                time_t      tv_sec;     // seconds
 //                suseconds_t tv_usec;    // microseconds
 //              };

 gettimeofday(&tv,NULL);
 return (tv.tv_sec*1000000 + tv.tv_usec);

}
