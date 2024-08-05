//######################################################    INCLUDES     #####################################################//
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>

//######################################################    DEFINES     ######################################################//
#define pi 3.14159             

// Ángulos de giro límites para los servos                                        
#define ANG_MAX pi / 3
#define ANG_MIN -pi / 4       

// Servos en conexiones: 4, 8, 12 (sentido inverso de giro); 7, 11, 15 (sentido normal de giro)                                                 
#define SERVO1 0
#define SERVO2 1
#define SERVO3 2
#define SERVO4 3
#define SERVO5 4
#define SERVO6 5

//######################################################   OBJETOS    ########################################################//
BluetoothSerial SerialBT;
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x41);

//#####################################################    FUNCIONES     ######################################################//
void setServo(uint8_t n_servo, int angulo);
void stewart(float x, float y, float z, float alpha_grad, float theta_grad, float psi_grad);

//#############################################    VARIABLES DE LOS SERVOS   ##################################################//
const uint16_t pos0[6] = {120,85,105,105,120,140};                         
const uint16_t pos180[6] = {463,420,455,440,460,480};      

//######################################    VARIABLES DE LA GEOMETRIA DEL SISTEMA   #############################################//
// Distancias (metros)
const float r_b = 104.89 / 1000;                                                // distancia del centro de la base a los puntos de unión de la misma
const float r_p = 90.545 / 1000;                                                // distancia del centro de la plataforma a los puntos de unión de la misma
const float h_0 = 85.112 / 1000;                                                // altura inicial de la plataforma
const float l_var = 100.0 / 1000;                                               // longitud de las varilla
const float l_brz = 15.0 / 1000;                                                // longitud de los brazos de los servos
float Bx_b[6], By_b[6], Bz_b[6];                                                // vectores que almacenan las posiciones x,y,z de los puntos de unión de la base
float Px_p[6], Py_p[6], Pz_p[6];                                                // vectores que almacenan las posiciones x,y,z de los puntos de unión de la plataforma

// Ángulos de rotacion
const float angBase[6] = {64.6, 355.4, 304.6, 235.4, 184.6, 115.4};             // Rotación de los puntos de unión de la base
const float angPlat[6] = {40.6, 19.4, 280.6, 259.4, 160.6, 139.4};              // Rotación de los puntos de unión de la plataforma
const float beta[6] = {180.0, 240.0, 60.0, 120.0, 300.0, 0.0};                  // Rotación de los brazos de cada servo respecto del eje X

//########################################    VARIABLES DE LA CINEMATICA INVERSA   #############################################//
static float T[3];                                                              // vector T del modelamiento
static float R[3][3];                                                           // matriz de rotación
static float Q[3][6];                                                           // los 6 vector Q del modelamiento
static float L[3][6];                                                           // los 6 vectores L del modelamiento
static float longitud[6];                                                       // el modulo de las longitudes L
static float a[6], b[6], c[6], d[6];                                            // vectores que almacenan valores utilizados en el cálculo del ángulo de giro de cada servo
static float ang[6];                                                            // vector con los ángulos de giro para cada servo (RADIANES)
static float ang1, ang2, ang3, ang4, ang5, ang6;                                // Ángulo de giro de cada uno de los servos (SEXAGESIMALES)
float posx, posy, posz, girox, giroy, giroz;                                    // referencia de posición y giro deseada para la plataforma

//#########################################################    SETUP     ######################################################//
void setup(){
    Wire.begin();
    servos.begin();             
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(50);          
    Serial.begin(9600);
    SerialBT.begin(9600);

    // Colocamos los servos a una posición inicial (90 grados)
    setServo(SERVO1, 90);   delay(50);
    setServo(SERVO2, 90);   delay(50);
    setServo(SERVO3, 90);   delay(50);
    setServo(SERVO4, 90);   delay(50);
    setServo(SERVO5, 90);   delay(50);
    setServo(SERVO6, 90);   delay(1000);

    // Calculamos las posiciones X y Y de las uniones entre la plataforma y la base
    for (int j = 0; j <= 5; j++){
        Bx_b[j] = r_b * cos(angBase[j] * pi / 180);
        By_b[j] = r_b * sin(angBase[j] * pi / 180);
        Px_p[j] = r_p * cos(angPlat[j] * pi / 180);
        Py_p[j] = r_p * sin(angPlat[j] * pi / 180);
    }
}

//######################################################    LOOP     ######################################################//
void loop(){
    stewart(0,0,0,0,0,0);

    if (SerialBT.available() > 0){      

    }
}

//######################################################    ZONA DE FUNCIONES     ######################################################//
void setServo(uint8_t n_servo, int angulo) {
    int duty = map(angulo,0,180,pos0[n_servo], pos180[n_servo]);
    servos.setPWM(n_servo, 0, duty); 
}

void stewart(float x, float y, float z, float alpha_grad, float theta_grad, float psi_grad){
    // Valores de x,y,z al vector de traslación de la plataforma
    T[0] = x / 1000;
    T[1] = y / 1000;
    T[2] = z / 1000 + h_0;

    // Conversión de ángulos recibidos a radianes
    float alpha = alpha_grad * pi / 180;
    float theta = theta_grad * pi / 180;
    float psi = psi_grad * pi / 180;

    // Cálculo de la matriz de rotación
    R[0][0] = cos(alpha) * cos(theta);
    R[0][1] = -sin(alpha) * cos(psi) + cos(alpha) * sin(theta) * sin(psi);
    R[0][2] = sin(alpha) * sin(psi) + cos(alpha) * sin(theta) * cos(psi);
    R[1][0] = sin(alpha) * cos(theta);
    R[1][1] = cos(alpha) * cos(psi) + sin(alpha) * sin(theta) * sin(psi);
    R[1][2] = -cos(alpha) * sin(psi) + sin(alpha) * sin(theta) * cos(psi);
    R[2][0] = -sin(theta);
    R[2][1] = cos(theta) * sin(psi);
    R[2][2] = cos(theta) * cos(psi);

    // Cálculo de los vectores Q
    for (int j = 0; j <= 5; j++){
        Q[0][j] = T[0] + R[0][0] * Px_p[j] + R[0][1] * Py_p[j] + R[0][2] * Pz_p[j];
        Q[1][j] = T[1] + R[1][0] * Px_p[j] + R[1][1] * Py_p[j] + R[1][2] * Pz_p[j];
        Q[2][j] = T[2] + R[2][0] * Px_p[j] + R[2][1] * Py_p[j] + R[2][2] * Pz_p[j];
    }

    // Cálculo de la matriz L
    for (int j = 0; j <= 5; j++){
        L[0][j] = Q[0][j] - Bx_b[j];
        L[1][j] = Q[1][j] - By_b[j];
        L[2][j] = Q[2][j] - Bz_b[j];
    }

    // Cálculo del modulo de L
    for (int j = 0; j <= 5; j++){
        longitud[j] = sqrt(pow(L[0][j], 2) + pow(L[1][j], 2) + pow(L[2][j], 2));
    }

    // Ecuacion de la cinematica inversa para obtener los angulos del servo
    for (int j = 0; j <= 5; j++){
        a[j] = 2 * l_brz * L[2][j];
        b[j] = 2 * l_brz * (sin(beta[j] * pi / 180) * L[1][j] + cos(beta[j] * pi / 180) * L[0][j]);
        c[j] = pow(longitud[j], 2) - pow(l_var, 2) + pow(l_brz, 2);
        d[j] = constrain(c[j] / sqrt(pow(a[j], 2) + pow(b[j], 2)), -1, 1);

        ang[j] = constrain(asin(d[j]) - atan(b[j] / a[j]), ANG_MIN, ANG_MAX);
    }

    // Conversión del ángulo de giro a grados sexagesimales
    ang1 = ang[0] * 180 / pi;
    ang2 = ang[1] * 180 / pi;
    ang3 = ang[2] * 180 / pi;
    ang4 = ang[3] * 180 / pi;
    ang5 = ang[4] * 180 / pi;
    ang6 = ang[5] * 180 / pi;

    // Envía a los servos el ángulo deseado
    setServo(SERVO1, 90 - ang1);    delay(10);
    setServo(SERVO2, 90 + ang2);    delay(10);
    setServo(SERVO3, 90 - ang3);    delay(10);
    setServo(SERVO4, 90 + ang4);    delay(10);
    setServo(SERVO5, 90 - ang5);    delay(10);
    setServo(SERVO6, 90 + ang6);    delay(50);

    String angulos = String(90 - ang1) + " " + String(90 + ang2) + " " + String(90 - ang3) + " " + String(90 + ang4) + " " + String(90 - ang5) + " " + String(90 + ang6);
    Serial.println(angulos);
}