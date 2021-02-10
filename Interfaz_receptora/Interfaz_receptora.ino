// Jhonny Lopez ostaiza
//Programa para el control de una perforadora hidraulica
//Receptor interfaz
//Librerias para el modulo nRF24L01
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
// Librerias para las pantallas LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd_velocidades(0x20, 16, 4); //I2C address para DIsplay de velocidades
LiquidCrystal_I2C lcd_presiones(0x27, 16, 4); //I2C address para DIsplay de presiones
//Caracteres especiales para la LCD
byte flechaArriba[] = {0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04};//Flecha arriba
byte flechaAbajo[] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x15, 0x0E, 0x04};//Flecha Abajo
byte flechaHorario[] = {0x01, 0x01, 0x05, 0x09, 0x1F, 0x08, 0x04, 0x00}; //Flecha horario
byte flechaAntihorario[] { 0x00, 0x04, 0x08, 0x1F, 0x09, 0x05, 0x01, 0x01}; //Flecha antihoraria
byte cara_error[] = {0x11, 0x0A, 0x11, 0x04, 0x00, 0x0E, 0x11, 0x00}; //Carra de error
byte angulo[] = {0x07, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00}; //SImbolo de angulo
const int IO2 = 2, IO3 = 3, IO4 = 4, IO5 = 5, IO6 = 6, IO7 = 7, IO8 = 8, IO9 = 9, IO10 = 10, IO11 = 11, IO12 = 12, IO13 = 13, IO14 = 14, IO15 = 15, IO16 = 16, IO17 = 17, IO18 = 18; //Pines de palanca
const int Led_horario = 22, Led_antihorario = 23, Led_subir = 24, Led_bajar = 25, Led_cilindroA = 26, Led_cilindroB = 27, Led_cilindroC = 28, Led_torre = 29; //Pines para leds indicadores
const int IO42 = 42, IO43 = 43, IO44 = 44, IO47 = 47; //Pines de final de carrera
const int IO48 = 48; //Pine de es stop
const int Solenoide_drill_A = 30, Solenoide_drill_B = 31, Solenoide_avance_A = 32, Solenoide_avance_B = 33, Solenoide_cilindroA_A = 34, Solenoide_cilindroA_B = 35,
          Solenoide_cilindroB_A = 36, Solenoide_cilindroB_B = 37, Solenoide_cilindroC_A = 38, Solenoide_cilindroC_B = 39, Solenoide_torre_A = 40, Solenoide_torre_B = 41; //Pines para Solenoides (Bobinas) de las valvulas direccionales
boolean Palanca1A, Palanca1B, Palanca2A, Palanca2B, Palanca3A, Palanca3B, Palanca4A, Palanca4B, Palanca5A, Palanca5B, Palanca6A, Palanca6B, Palanca7A, Palanca7B, Palanca8A, Palanca8B; //Estados de los palancas
boolean final_avance_inferior, final_avance_superior, final_torre_horizontal, final_torre_vertical;
unsigned  long tiempo_inicial = millis(), tiempo_inicial_envio = millis(); //tiempo de ejecucion inicial
long tiempo_pantalla_inicial = 2000; //Duracion de la pantalla de encendido
boolean ingreso = false, envio_horario = true, envio_antihorario = true, envio_arriba = true, envio_abajo = true, error_horario = true, error_antihorario = true,
        error_arriba = true, error_abajo = true, rotacion, error_angulo, error_angulo1 = true, stop_1 = true, presiones_1 = true, avance, imprimir_velocidad = true, imprimir_presiones = true, imprimir_stop; //
int stop, stop_in, stop_in_anterior;
int error_palancas = 0, error_presiones = 0, caso = 0;//variables de manejo de errores
int velocidad_rotacion, velocidad_rotacion_in, velocidad_rotacion_in_anterior, velocidad_avance, velocidad_avance_in, velocidad_avance_in_anterior; //Variables para medicion de velocidad
int presion_drill, presion_drill_in, presion_drill_in_anterior, presion_cilindros, presion_cilindros_in, presion_cilindros_in_anterior, presion_avance, presion_avance_in, presion_avance_in_anterior; //Variables para medicion de presion
int angulo_roll, angulo_roll_in, angulo_roll_in_anterior, angulo_pitch, angulo_pitch_in, angulo_pitch_in_anterior; //Variables para medicion de angulo de inclinacion
const int angulo_maximo = 5; //se define como angulo maximo una desviacion de 5 grados
const int presion_drill_minimo = 3000, presion_avance_minimo = 1000, presion_cilindros_minimo = 1000; //Presimones minimas de operacion
RF24 radio(45, 46);
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
typedef struct {
  bool P1A;
  bool P1B;
  bool P2A;
  bool P2B;
  bool P3A;
  bool P3B;
  bool P4A;
  bool P4B;
  bool P5A;
  bool P5B;
  bool P6A;
  bool P6B;
  bool P7A;
  bool P7B;
  bool P8A;
  bool P8B;
  bool S;
  bool O;
}
A_t;

typedef struct {
  int VG;
  int VA;
  int AR;
  int AP;
  int P1;
  int P2;
  int P3;
  bool ST;
}
B_t;

A_t  duino1;
B_t   duino2;
//========================================================================================================================================================================
// Variables inicializadas
void setup() {
  Serial.begin(57600);
  pines();
  radio.begin();
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);
}

void loop() {
  unsigned long tiempo = millis(); //Tiempo de ejecucion por ciclo
  EstadoDePines();//Se inician los pines
  velocidades(velocidad_rotacion_in, velocidad_avance_in);//Se lee y escala el valor de los encoders de velocidad
  presiones(presion_drill_in, presion_cilindros_in, presion_avance_in); //Se lee y escala el valor de los sensores de presion
  angulos(angulo_roll_in, angulo_pitch_in); //Se lee y escala el valor del senor MPU6050(inclinacion)
  //retroceso();
  if (ingreso) {
    //=====================================================================
    //Se actualizan los registros de las velocidades de los motores
    actualizar_velocidades();
    velocidades(velocidad_rotacion_in, velocidad_avance_in);
    //Se actualizan los registros de las presiones
    actualizar_presiones();
    presiones(presion_drill_in, presion_cilindros_in, presion_avance_in);
    error_presion();
    //Se actualizan los registros de inclinacion
    actualizar_angulos();
    angulos(angulo_roll_in, angulo_pitch_in);
    //Se verifica si hay error de inclinacion
    angulo_error();
    //Se envia un stop si se pulsa el boton de paro de emergencia
    stop_();
    //=====================================================================================
    duino2.VG = velocidad_rotacion;
    duino2.VA = velocidad_avance;
    duino2.AR = angulo_roll;
    duino2.AP = angulo_pitch;
    duino2.P1 = presion_drill;
    duino2.P2 = presion_avance;
    duino2.P3 = presion_cilindros;
    duino2.ST = imprimir_stop;
    radio.stopListening();
    bool enviado = radio.write( &duino2, sizeof(duino2) );
    radio.startListening();
    Serial.print(enviado);
    //=====================================================================================
    //Sentido horario
    if (Palanca1A or Palanca2A) {//Si se activa la(s) palanca(s) (enclavada o no enclavada )de giro horario
      //Se verifica si una de las palancas esta activada en el sentido de giro contrario (antihorario)
      if ((Palanca1B or Palanca2B) == false) {
        //Se envia a la pantalla un mensaje mostrando el sentido de giro(horario)
        rotacion = true;
        if (envio_horario) {
          error_palancas = 0;
          //Se activa el led indicando el sentido de giro horario
          digitalWrite(Led_horario, HIGH);
          digitalWrite(Solenoide_drill_A, HIGH);
          //Cambio por false para no enviar la pantalla en cada ciclo del lazo loop()
          envio_horario = false;
          error_antihorario = true;
          imprimir_velocidad = true;
        }
      }
      //Si se activan las palancas en los dos sentidos (horario y antihorario) se muestra mensaje de error
      else {
        if (error_antihorario) {
          //Se envia error 1 activacion de palanca antihorario
          error_palancas = 1;
          //Se activa la variable para mostrar la pantalla de sentido de giro horario luego de salir del error
          envio_horario = true;
          error_antihorario = false;
          imprimir_velocidad = true;
        }
      }
    }
    //Si no se activan las palancas en sentido horario se apaga la luz indicadora y se limpia la pantalla
    else {
      digitalWrite(Led_horario, LOW);
      digitalWrite(Solenoide_drill_A, LOW);
      //Se activa la variable para mostrar la pantalla de sentido de giro horario
      envio_horario = true;

    }

    //=====================================================================================
    // Sentido antihorario
    if (Palanca1B or Palanca2B) {//Si se activa la(s) palanca(s) (enclavada o no enclavada )de giro horario
      //Se verifica si una de las palancas esta activada en el sentido de giro contrario (antihorario)
      if ((Palanca1A or Palanca2A) == false) {
        //Se envia a la pantalla un mensaje mostrando el sentido de giro(horario)
        if (envio_antihorario) {
          error_palancas = 0;
          //Se activa el led indicando el sentido de giro horario
          digitalWrite(Led_antihorario, HIGH);
          digitalWrite(Solenoide_drill_B, HIGH);
          //Cambio por false para no enviar la pantalla en cada ciclo del lazo loop()
          envio_antihorario = false;
          error_horario = true;
          rotacion = false;
          imprimir_velocidad = true;
        }
      }
      //Si se activan las palancas en los dos sentidos (horario y antihorario) se muestra mensaje de error
      else {
        if (error_horario) {
          //Se envia error de activacion de palanca horario
          error_palancas = 1;
          //Se activa la variable para mostrar la pantalla de sentido de giro antihorario luego de salir del error
          envio_antihorario = true;
          error_horario = false;
          imprimir_velocidad = true;
        }
      }
    }
    //Si no se activan las palancas en sentido antihorario se apaga la luz indicadora y se limpia la pantalla
    else {
      digitalWrite(Led_antihorario, LOW);
      digitalWrite(Solenoide_drill_B, LOW);
      //Se activa la variable para mostrar la pantalla de sentido de giro antihorario
      envio_antihorario = true;
    }
    //=====================================================================================
    //Sentido hacia arriba
    if (Palanca3A or Palanca4A) {//Si se activa la(s) palanca(s) (enclavada o no enclavada )de avanace hacia arriba
      //Se verifica si una de las palancas esta activada en el sentido de avance contrario (abajo)
      if ((Palanca3B or Palanca4B) == false) {
        //Se envia a la pantalla un mensaje mostrando el sentido de avance hacia arriba
        avance = true;
        if (envio_arriba) {
          Serial.println("Posi");
          error_palancas = 0;
          //Se activa el led indicando el sentido de avance hacia arriba
          digitalWrite(Led_subir, HIGH);
          digitalWrite(Solenoide_avance_A, HIGH);
          //Cambio por false para no enviar la pantalla en cada ciclo del lazo loop()
          envio_arriba = false;
          error_abajo = true;
          imprimir_velocidad = true;
        }
      }
      //Si se activan las palancas en los dos sentidos (arriba y abajo) se muestra mensaje de error
      else {
        if (error_abajo) {
          //Se envia error de palancas de avance
          error_palancas = 2;
          //Se activa la variable para mostrar la pantalla de sentido de avance hacia arriiba luego de salir del error
          envio_arriba = true;
          error_abajo = false;
          imprimir_velocidad = true;
        }
      }
      if (final_avance_superior) {
        digitalWrite(Solenoide_avance_A, LOW);
      }
    }
    //Si no se activan las palancas en sentido de avance hacia arriba se apaga la luz indicadora
    else {
      Serial.println("sii");
      digitalWrite(Led_subir, LOW);
      digitalWrite(Solenoide_avance_A, LOW);
      //Se activa la variable para mostrar la pantalla de sentido de avance hacia arriba
      envio_arriba = true;
    }
    //=====================================================================================
    // Sentido abajo
    if (Palanca3B or Palanca4B) {//Si se activa la(s) palanca(s) (enclavada o no enclavada )de avance hacia abajo
      //Se verifica si una de las palancas esta activada en el sentido de avance hacia arriba
      if ((Palanca3A or Palanca4A) == false) {
        //Se envia a la pantalla un mensaje mostrando el sentido de giro(horario)
        if (envio_abajo) {
          error_palancas = 0;
          //Se activa el led indicando el sentido de giro horario
          digitalWrite(Led_bajar, HIGH);
          digitalWrite(Solenoide_avance_B, HIGH);
          //Cambio por false para no enviar la pantalla en cada ciclo del lazo loop()
          envio_abajo = false;
          error_arriba = true;
          avance = false;
          imprimir_velocidad = true;
          Serial.println("OK");
        }
      }
      //Si se activan las palancas en los dos sentidos (horario y antihorario) se muestra mensaje de error
      else {
        if (error_arriba) {
          //Se envia error de palancas de avance
          error_palancas = 2;
          //Se activa la variable para mostrar la pantalla de sentido de giro antihorario luego de salir del error
          envio_abajo = true;
          error_arriba = false;
          imprimir_velocidad = true;
        }
      }
      if (final_avance_inferior) {
        digitalWrite(Solenoide_avance_B, LOW);
      }
    }
    //Si no se activan las palancas en sentido antihorario se apaga la luz indicadora y se limpia la pantalla
    else {
      digitalWrite(Led_bajar, LOW);
      digitalWrite(Solenoide_avance_B, LOW);
      //Se activa la variable para mostrar la pantalla de sentido de giro antihorario
      envio_abajo = true;
    }
    //===================================================================================
    //Posicion de cilindro A hacia arriba
    if (Palanca5A) {
      digitalWrite(Solenoide_cilindroA_A, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroA_A, LOW);
    }
    //Posicion de cilindro A hacia abajo
    if (Palanca5B) {
      digitalWrite(Solenoide_cilindroA_B, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroA_B, LOW);
    }
    //===================================================================================
    //Posicion de cilindro B hacia arriba
    if (Palanca6A) {
      digitalWrite(Solenoide_cilindroB_A, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroB_A, LOW);
    }
    //Posicion de cilindro B hacia abajo
    if (Palanca6B) {
      digitalWrite(Solenoide_cilindroB_B, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroB_B, LOW);
    }
    //===================================================================================
    //Posicion de cilindro C hacia arriba
    if (Palanca7A) {
      digitalWrite(Solenoide_cilindroC_A, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroC_A, LOW);
    }
    //Posicion de cilindro C hacia abajo
    if (Palanca7B) {
      digitalWrite(Solenoide_cilindroC_B, HIGH);

    }
    else {
      digitalWrite(Solenoide_cilindroC_B, LOW);
    }
    //=====================================================================
    //Posicion de Torre vertical
    if (Palanca8A) {
      if (final_torre_vertical == HIGH) {
        digitalWrite(Solenoide_torre_A, LOW);
        digitalWrite(Led_torre, HIGH);
      }
      else {
        digitalWrite(Solenoide_torre_A, HIGH);
        digitalWrite(Led_torre, LOW);
      }
    }
    else {
      digitalWrite(Solenoide_torre_A, LOW);
    }
    //Posicion de torre horizontal
    if (Palanca8B) {
      if (final_torre_horizontal == HIGH) {
        digitalWrite(Solenoide_torre_B, LOW);
      }
      else {
        digitalWrite(Solenoide_torre_B, HIGH);

      }

    }
    else {
      digitalWrite(Solenoide_torre_B, LOW);
    }


    //    }
    //=====================================================================
    //Parada de emergencia


    if (imprimir_velocidad) {
      pantallas_velocidades(lcd_velocidades, caso, velocidad_rotacion, velocidad_avance, angulo_roll, angulo_pitch, error_palancas, rotacion, avance);
      imprimir_velocidad = false;

    }
    if (imprimir_presiones) {
      pantallas_presiones(lcd_presiones, caso, error_presiones, presion_drill, presion_cilindros, presion_avance);
      imprimir_presiones = false;
    }
    if (imprimir_stop) {
      pantalla_Stop(lcd_velocidades, stop);
      imprimir_stop = false;
    }

    if (error_angulo) {
      error_palancas = 3;
      pantallas_velocidades(lcd_velocidades, caso, velocidad_rotacion, velocidad_avance, angulo_roll, angulo_pitch, error_palancas, rotacion, avance);
      error_angulo = false;
    }
  }
  else {
    //*********************************************************************************
    lcd_velocidades.begin(20, 4); //Inicio de el visualizador LCD 16x4
    //Carapteres especiales para la pantalla LCD de velocidades
    lcd_velocidades.createChar(0, flechaArriba);
    lcd_velocidades.createChar(1, flechaAbajo);
    lcd_velocidades.createChar(2, flechaHorario);
    lcd_velocidades.createChar(3, flechaAntihorario);
    lcd_velocidades.createChar(4, cara_error);
    lcd_velocidades.createChar(5, angulo);
    //Pantalla de velocidades a mostrar al encendido
    lcd_velocidades.home();
    lcd_velocidades.backlight();//Endendido de luz
    pantallas_velocidades(lcd_velocidades, caso, velocidad_rotacion, velocidad_avance, angulo_roll, angulo_pitch, error_palancas, rotacion, avance);
    //**********************************************************************************
    lcd_presiones.begin(20, 4); //Inicio de el visualizador LCD 16x4
    //Carapteres especiales para la pantalla LCD de velocidades
    lcd_presiones.createChar(0, flechaArriba);
    lcd_presiones.createChar(1, flechaAbajo);
    lcd_presiones.createChar(2, flechaHorario);
    lcd_presiones.createChar(3, flechaAntihorario);
    lcd_presiones.createChar(4, cara_error);
    //Pantalla de velocidades a mostrar al encendido
    lcd_presiones.home();
    lcd_presiones.backlight();//Endendido de luz
    pantallas_presiones(lcd_presiones, caso, error_presiones, presion_drill, presion_cilindros, presion_avance);

    if (tiempo - tiempo_inicial > tiempo_pantalla_inicial) {
      tiempo_inicial = tiempo;
      caso = 1;
      ingreso = true;
    }
  }

}




void pines() {
  //Estableciendo pines de las palancas como entradas
  pinMode(IO2, INPUT);
  pinMode(IO3, INPUT);
  pinMode(IO4, INPUT);
  pinMode(IO5, INPUT);
  pinMode(IO6, INPUT);
  pinMode(IO7, INPUT);
  pinMode(IO8, INPUT);
  pinMode(IO9, INPUT);
  pinMode(IO10, INPUT);
  pinMode(IO11, INPUT);
  pinMode(IO12, INPUT);
  pinMode(IO13, INPUT);
  pinMode(IO14, INPUT);
  pinMode(IO15, INPUT);
  pinMode(IO16, INPUT);
  pinMode(IO17, INPUT);
  pinMode(IO18, INPUT);
  //Estableciendo pines de leds indicadores como salidas
  pinMode(Led_subir, OUTPUT);
  pinMode(Led_bajar, OUTPUT);
  pinMode(Led_horario, OUTPUT);
  pinMode(Led_antihorario, OUTPUT);
  pinMode(Led_cilindroA, OUTPUT);
  pinMode(Led_cilindroB, OUTPUT);
  pinMode(Led_cilindroC, OUTPUT);
  pinMode(Led_torre, OUTPUT);
  //Establecliendo pines de activacion de solenoides
  pinMode(Solenoide_drill_A, OUTPUT);
  pinMode(Solenoide_drill_B, OUTPUT);
  pinMode(Solenoide_avance_A, OUTPUT);
  pinMode(Solenoide_avance_B, OUTPUT);
  pinMode(Solenoide_cilindroA_A, OUTPUT);
  pinMode(Solenoide_cilindroA_B, OUTPUT);
  pinMode(Solenoide_cilindroB_A, OUTPUT);
  pinMode(Solenoide_cilindroB_B, OUTPUT);
  pinMode(Solenoide_cilindroC_A, OUTPUT);
  pinMode(Solenoide_cilindroC_B, OUTPUT);
  pinMode(Solenoide_torre_A, OUTPUT);
  pinMode(Solenoide_torre_B, OUTPUT);
  //Estableciendo pines de finales de carrera
  pinMode(IO42, INPUT);
  pinMode(IO43, INPUT);
  pinMode(IO44, INPUT);
  pinMode(IO47, INPUT);
  //Estableciendo pin para boton stop
  pinMode(IO48, INPUT);
}
//Metodo para leer los datos de los pines de entrada
void EstadoDePines() {
  Palanca1A = digitalRead(IO2);
  Palanca1B = digitalRead(IO3);
  Palanca2A = digitalRead(IO4);
  Palanca2B = digitalRead(IO5);
  Palanca3A = digitalRead(IO6);
  Palanca3B = digitalRead(IO7);
  Palanca4A = digitalRead(IO8);
  Palanca4B = digitalRead(IO9);
  Palanca5A = digitalRead(IO10);
  Palanca5B = digitalRead(IO11);
  Palanca6A = digitalRead(IO12);
  Palanca6B = digitalRead(IO13);
  Palanca7A = digitalRead(IO14);
  Palanca7B = digitalRead(IO15);
  Palanca8A = digitalRead(IO16);
  Palanca8B = digitalRead(IO17);
  velocidad_rotacion_in = analogRead(A0);
  velocidad_avance_in = analogRead(A1);
  angulo_roll_in = analogRead(A2);
  angulo_pitch_in = analogRead(A3);
  presion_drill_in = analogRead(A4);
  presion_cilindros_in = analogRead(A5);
  presion_avance_in = analogRead(A6);
  final_avance_inferior = digitalRead(IO42);
  final_avance_superior = digitalRead(IO43);
  final_torre_horizontal = digitalRead(IO44);
  final_torre_vertical = digitalRead(IO47);
  stop = digitalRead(IO48);
}
void velocidad_Drill() {

}
int velocidad_Avance() {
  int salida;
  return salida;
}
float angulo_YX() {
  float salida;
  return salida;
}
float angulo_YZ() {
  float salida;
  return salida;
}
void pantallas_velocidades(LiquidCrystal_I2C lcd, int caso, int velocidad_rotacion, int velocidad_avance, int angulo_roll, int angulo_pitch, int error, boolean rotacion, boolean avance) {
  switch (caso) {
    case 0:
      //Se valida el tiempo de muestra de la pantalla inicial, si es lo supera , se limpia la pantalla y se pasa al
      //algoritmo de control.
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("RF SWITCH");
      lcd.setCursor(6, 1);
      lcd.print("INTISA");
      lcd.setCursor(4, 2);
      lcd.print(" SIMCO 2800");
      lcd.setCursor(6, 3);
      lcd.print(" HS HT");
      break;
    case 1:
      switch (error) {
        case 0:
          lcd.clear();
          lcd.setCursor(6, 0);
          lcd.print("Sensores");
          lcd.setCursor(0, 1);
          lcd.print("Rotacion:");
          lcd.setCursor(9, 1);
          if (velocidad_rotacion > 0) {
            lcd.print(velocidad_rotacion);
            lcd.print(" ");
            lcd.write(2);
            lcd.print(" RPM");
          }
          else if (velocidad_rotacion < 0) {
            lcd.print(-1 * velocidad_rotacion);
            lcd.print(" ");
            lcd.write(3);
            lcd.print(" RPM");
          }
          else {
            lcd.print(velocidad_rotacion);
            lcd.print(" Stop");
          }
          lcd.setCursor(0, 2);
          lcd.print("Avance:");
          lcd.setCursor(9, 2);
          if (velocidad_avance > 0) {
            lcd.print(velocidad_avance);
            lcd.print(" ");
            lcd.write(0);
            lcd.print(" Ft/min");
          }
          else if (velocidad_avance < 0) {
            lcd.print(-1 * velocidad_avance);
            lcd.print(" ");
            lcd.write(1);
            lcd.print(" Ft/min");
          }
          else {
            lcd.print(velocidad_avance);
            lcd.print(" Stop");
          }
          lcd.setCursor(0, 3);
          lcd.print("Roll: ");
          lcd.print(angulo_roll);
          lcd.write(5);
          lcd.setCursor(10, 3);
          lcd.print("Pitch:");
          lcd.print(angulo_pitch);
          lcd.write(5);

          break;
        case 1:
          lcd.clear();
          lcd.setCursor(0, 0);
          //Se envia mensaje de error
          lcd.write(4);
          lcd.print("  ERROR DE GIRO  ");
          lcd.write(4);
          lcd.setCursor(2, 1);
          lcd.print(" Quitar palanca");
          if (rotacion) {
            lcd.print(" ");
            lcd.write(2);
          }
          else {
            lcd.print(" ");
            lcd.write(3);
          }
          break;
        case 2:
          lcd.clear();
          lcd.setCursor(0, 0);
          //Se envia mensaje de error
          lcd.print("   ERROR DE AVANCE");

          lcd.setCursor(0, 1);
          lcd.print("  Quitar palanca");
          if (avance) {
            lcd.print(" ");
            lcd.write(1);
          }
          else {
            lcd.print(" ");
            lcd.write(0);
          }
          lcd.setCursor(8, 2);

          lcd.write(4);
          lcd.write(4);
          lcd.write(4);
          lcd.write(4);
          break;
        case 3:
          lcd.clear();
          lcd.setCursor(0, 0);
          //Se envia mensaje de error
          lcd.print("   ERROR DE NIVEL");
          lcd.setCursor(0, 1);
          lcd.print("   Mover palancas");
          if (angulo_roll > 5 ) {
            lcd.setCursor(7, 2);
            lcd.print("Base");
            lcd.setCursor(0, 3);
            lcd.print("Roll: ");
            lcd.print(angulo_roll);
            lcd.write(5);
            lcd.setCursor(10, 3);
            lcd.print("Pitch:");
            lcd.print(angulo_pitch);
            lcd.write(5);
          }
          else if (angulo_roll < -5) {
            lcd.setCursor(7, 2);
            lcd.print("Base");
            lcd.setCursor(0, 3);
            lcd.print("Roll: ");
            lcd.print(angulo_roll);
            lcd.write(5);
            lcd.setCursor(10, 3);
            lcd.print("Pitch:");
            lcd.print(angulo_pitch);
            lcd.write(5);
          }
          if (angulo_pitch > 5 or angulo_pitch < -5) {
            lcd.setCursor(7, 2);
            lcd.print("Base");
            lcd.setCursor(0, 3);
            lcd.print("Roll: ");
            lcd.print(angulo_roll);
            lcd.write(5);
            lcd.setCursor(10, 3);
            lcd.print("Pitch:");
            lcd.print(angulo_pitch);
            lcd.write(5);
          }
          error_palancas = 0;
          break;

      }
      break;
    case 2:

      break;
  }

}

void pantallas_presiones(LiquidCrystal_I2C lcd, int caso, int error_presion, int presion_drill, int presion_cilindros, int presion_avance) {
  switch (caso) {
    case 0:
      //Se valida el tiempo de muestra de la pantalla inicial, si es lo supera , se limpia la pantalla y se pasa al
      //algoritmo de control.
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("Preforadora");
      lcd.setCursor(5, 1);
      lcd.print("Rotatoria");
      lcd.setCursor(4, 2);
      lcd.print("SIMCO 2800");
      lcd.setCursor(7, 3);
      lcd.print("HS HT");
      break;
    case 1:
      switch (error_presion) {
        case 0:
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("  PRESION");
          lcd.setCursor(0, 1);
          lcd.print("Drill: ");
          lcd.setCursor(11, 1);
          lcd.print(presion_drill);
          lcd.setCursor(16, 1);
          lcd.print("Psi");
          lcd.setCursor(0, 2);
          lcd.print("Cilindros: ");
          //lcd.print(11, 2);
          lcd.print(presion_cilindros);
          lcd.setCursor(16, 2);
          lcd.print("Psi");
          lcd.setCursor(0, 3);
          lcd.print("Avance: ");
          lcd.setCursor(11, 3);
          lcd.print(presion_avance);
          lcd.setCursor(16, 3);
          lcd.print("Psi");
          break;
        case 1:
          lcd.clear();
          lcd.setCursor(5, 0);
          //Se envia mensaje de error
          lcd.write(4);
          lcd.print(" ERROR ");
          lcd.write(4);
          lcd.setCursor(3, 1);
          lcd.print("PRESION BAJA");
          if (presion_drill <= presion_drill_minimo) {
            lcd.setCursor(0, 2);
            lcd.print("Drill: ");
            lcd.setCursor(11, 2);
            lcd.print(presion_drill);
            lcd.setCursor(16, 2);
            lcd.print("Psi");
          }
          else if (presion_avance <= presion_avance_minimo) {
            lcd.setCursor(0, 2);
            lcd.print("Avance: ");
            lcd.setCursor(11, 2);
            lcd.print(presion_avance);
            lcd.setCursor(16, 2);
            lcd.print("Psi");
          }
          else if (presion_cilindros <= presion_cilindros_minimo) {
            lcd.setCursor(0, 2);
            lcd.print("Cilindros: ");
            lcd.setCursor(11, 2);
            lcd.print(presion_cilindros);
            lcd.setCursor(16, 2);
            lcd.print("Psi");

          }
          break;

      }
      break;
    case 2:

      break;
  }

}
//===================================================================================================
void pantalla_Stop(LiquidCrystal_I2C lcd, boolean stop) {
  if (stop) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PARADA DE EMERGENCIA");
    lcd.setCursor(2 , 1);
    lcd.print("DETENIENDO MOTORES");
    lcd.setCursor(2, 2);
    lcd.print("RETORNAR PALANCAS");
    lcd.setCursor(7, 3);
    lcd.write(4);
    lcd.write(4);
    lcd.write(4);
    lcd.write(4);
    lcd.write(4);
    lcd.write(4);


  }


}
//===================================================================================================
//Conversion de las entradas analogicas de las velocidades
void velocidades(int velocidad_rotacion_in, int velocidad_avance_in) {
  float constante_velocidad_rotacion = 185.0 / 512.0; //Entrada analogica de 0-1023 de 0-512 velocidad de -185Rpm hasta 0 Rpm
  // de 512-1023 velocidad de 0 Rpm hasta 185 Rpm
  float constante_velocidad_avance = 100.0 / 512.0; //Entrada analogica de 0-1023 de 0-512 velocidad de -100ft/min hasta 0 ft/min
  // de 512-1023 velocidad de 0 ft/min hasta 100 ft/min
  if (velocidad_rotacion_in >= 512) {
    velocidad_rotacion = velocidad_rotacion_in - 512;
    velocidad_rotacion = velocidad_rotacion * constante_velocidad_rotacion;
  }
  else if (velocidad_rotacion_in < 512) {
    velocidad_rotacion = -185 + (velocidad_rotacion_in * constante_velocidad_rotacion);
  }
  if (velocidad_avance_in >= 512) {
    velocidad_avance = (velocidad_avance_in - 512) * constante_velocidad_avance;

  }
  else if (velocidad_avance_in < 512) {
    velocidad_avance = -100 + (velocidad_avance_in * constante_velocidad_avance);
  }
}
//===================================================================================================
//Conversion de las entradas analogicas de las presiones
void presiones(int presion_drill_in, int presion_cilindros_in, int presion_avance_in) {
  float constante_presion_drill = 5000.0 / 1023.0; //Entrada analogica de 0-1023  que representa presion de 0-5000 PSI
  float constante_presion_cilindros = 2500.0 / 1023.0; //Entrada analogica de 0-1023  que representa presion de 0-2500 PSI
  float constante_presion_avance = 2500.0 / 1023.0; //Entrada analogica de 0-1023  que representa presion de 0-2500 PSI
  presion_drill = presion_drill_in * constante_presion_drill;
  presion_cilindros = presion_cilindros_in * constante_presion_cilindros;
  presion_avance = presion_avance_in * constante_presion_avance;
}
//===================================================================================================
void angulos(int  angulo_roll_in, int angulo_pitch_in) {
  float constante_angulo_roll = 45.0 / 512.0; //Entrada analogica de 0-1023 de 0-512 velocidad de -45 grados hasta 0 grados
  // de 512-1023 velocidad de 0 grados hasta 45 grados
  float constante_angulo_pitch = 45.0 / 512.0; //Entrada analogica de 0-1023 de 0-512 velocidad de -45 grados hasta 0 grados
  // de 512-1023 velocidad de 0 grados hasta 45 grados
  if (angulo_roll_in >= 512) {
    angulo_roll = angulo_roll_in - 512;
    angulo_roll = angulo_roll * constante_angulo_roll;
  }
  else if (angulo_roll_in < 512) {
    angulo_roll = -45 + (angulo_roll_in * constante_angulo_roll);
  }
  if (angulo_pitch_in >= 512) {
    angulo_pitch = (angulo_pitch_in - 512) * constante_angulo_pitch;
  }
  else if (angulo_pitch_in < 512) {
    angulo_pitch = -45 + (angulo_pitch_in * constante_angulo_pitch);
  }
}
//===================================================================================================
//Actualizacion de las leccturas de las variables de velocidad
void actualizar_velocidades() {
  if (velocidad_rotacion_in_anterior == velocidad_rotacion_in) {
  }
  //Siempre que la velocidad de rotacion medida y la velocidad de rotracion anterior sean diferentes se actualiza
  else {
    velocidad_rotacion_in_anterior = velocidad_rotacion_in;
    imprimir_velocidad = true;
  }

  //Siempre que la velocidad de avance medida y la velocidad de avance anterior sean diferentes se actualiza
  if (velocidad_avance_in_anterior == velocidad_avance_in) {

  }
  else {
    velocidad_avance_in_anterior = velocidad_avance_in;
    imprimir_velocidad = true;
  }
}
//===================================================================================================
//Actualizacion de las lecturas de las variables de presion
void actualizar_presiones() {
  if (presion_drill_in_anterior == presion_drill_in) {
  }
  //Siempre que la presion de drill  medida y presion  drill medida anterior sean diferentes se actualiza
  else {
    presion_drill_in_anterior = presion_drill_in;
    imprimir_presiones = true;
  }

  if (presion_cilindros_in_anterior == presion_cilindros_in) {
  }
  //Siempre que la presion de cilindros medida y la presion de cilindros anterior sean diferentes se actualiza
  else {
    presion_cilindros_in_anterior = presion_cilindros_in;
    imprimir_presiones = true;
  }

  if (presion_avance_in_anterior == presion_avance_in) {
  }
  //Siempre que la presion de avance medida y la presion de avance anterior sean diferentes se actualiza
  else {
    presion_avance_in_anterior = presion_avance_in;
    imprimir_presiones = true;
  }
}
//===================================================================================================
//Actualizacion de las leccturas de las variables de inclinacion
void actualizar_angulos() {
  if (angulo_roll_in_anterior == angulo_roll_in) {
  }
  //Siempre que el angulo de roll medido y el angulo de roll anterior sean diferentes se actualiza
  else {
    angulo_roll_in_anterior = angulo_roll_in;
    imprimir_velocidad = true;
  }

  //Siempre que el angulo pitch medido y el angulo de pitch anterior sean diferentes se actualiza
  if (angulo_pitch_in_anterior == angulo_pitch_in) {

  }
  else {
    angulo_pitch_in_anterior = angulo_pitch_in;
    imprimir_velocidad = true;
  }
}


void angulo_error() {
  if ((angulo_roll < -1 * angulo_maximo or angulo_roll > angulo_maximo ) or (angulo_pitch < -1 * angulo_maximo or angulo_pitch > angulo_maximo)) {
    if (error_angulo1) {
      error_angulo = true;
      error_palancas = 3;
      imprimir_velocidad = true;
      error_angulo1 = false;
    }
    if (error_palancas != 3) {
      error_palancas = 3;
    }
  }
  else {
    error_angulo = false;
    if (error_angulo1 == false) {
      error_palancas = 0;
      imprimir_velocidad = true;
    }
    error_angulo1 = true;
  }
}
void error_presion() {
  if (presion_drill <= presion_drill_minimo or presion_avance <= presion_avance_minimo or presion_cilindros <= presion_cilindros_minimo) {
    if (presiones_1) {
      error_presiones = 1;
      imprimir_presiones = true;
      presiones_1 = false;
    }
  }
  else {
    if (presiones_1 == false) {
      error_presiones = 0;
      imprimir_presiones = true;
    }
    presiones_1 = true;
  }
}
void retroceso() {
  if (velocidad_rotacion<10 and velocidad_rotacion> -10) {
    if (Palanca1A or Palanca1B) {
      Serial.println("Ingreso");
      digitalWrite(Solenoide_avance_B, LOW);
      digitalWrite(Led_bajar, LOW);
      digitalWrite(Solenoide_avance_A, HIGH);
      digitalWrite(Led_subir, HIGH);
    }
  }

  else {
    digitalWrite(Solenoide_avance_A, LOW);
    digitalWrite(Led_subir, LOW);
    envio_abajo = true;
  }
}
void stop_() {
  if (stop) {
    if (stop_1) {
      digitalWrite(Solenoide_drill_A , LOW);
      digitalWrite(Solenoide_drill_B, LOW);
      digitalWrite(Solenoide_avance_A, LOW);
      digitalWrite(Solenoide_avance_B, LOW);
      digitalWrite(Solenoide_cilindroA_A, LOW);
      digitalWrite(Solenoide_cilindroA_B , LOW);
      digitalWrite(Solenoide_cilindroB_A, LOW);
      digitalWrite(Solenoide_cilindroB_B , LOW);
      digitalWrite(Solenoide_cilindroC_A, LOW);
      digitalWrite(Solenoide_cilindroC_B , LOW);
      digitalWrite(Solenoide_torre_A, LOW);
      digitalWrite(Solenoide_torre_B, LOW);
      stop_1 = false;
      imprimir_stop = true;
    }
  }
  else {
    if (stop_1 == false) {
      error_palancas = 0;
      imprimir_velocidad = true;
      imprimir_stop = false;
      stop_1 = true;
    }
  }
}
