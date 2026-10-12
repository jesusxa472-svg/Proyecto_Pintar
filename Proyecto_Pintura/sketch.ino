#include <Arduino.h>

// Pines del motor X
const int STEP_X = 18;
const int DIR_X = 19;

// Pines del motor Y
const int STEP_Y = 21;
const int DIR_Y = 22;

// Sensor ultrasonico
const int TRIG_PIN = 5;
const int ECHO_PIN = 17;

// Botones
const int ESTOP_PIN = 23;
const int LIMIT_X_PIN = 16;
const int LIMIT_Y_PIN = 4;

// LED de pintura (Pistola)
const int GUN_PIN = 25;

// Velocidad (Microsegundos entre pasos)
const unsigned long STEP_INTERVAL_US = 600;

// Estados
enum RobotState {
  IDLE,
  SCANNING,
  WAITING_AI,
  PAINTING,
  EMERGENCY_STOP,
  LIMIT_FAULT
};

RobotState robotState = IDLE;

// Posiciones
long positionX = 0;
long positionY = 0;
long targetX = 0;
long targetY = 0;

// Temporizadores
unsigned long lastStepX = 0;
unsigned long lastStepY = 0;
unsigned long lastTelemetry = 0;
unsigned long lastStatus = 0;

// Estados de los pulsos
bool stepLevelX = false;
bool stepLevelY = false;
bool gunOn = false;
bool manualMove = false;

// Trayectoria de prueba
const int PATH_SIZE = 6;
const long pathX[PATH_SIZE] = {0, 200, 400, 400, 200, 0};
const long pathY[PATH_SIZE] = {0, 0, 0, 150, 150, 150};
int pathIndex = 0;

// Prototipos
float readDistance();
void printHelp();
void printStatus();
void processCommand(String command);
void updateMotors();
void setGun(bool enabled);
void stopMotors();
void beginPainting();

const char* stateName() {
  switch (robotState) {
    case IDLE: return "IDLE";
    case SCANNING: return "SCANNING";
    case WAITING_AI: return "WAITING_AI";
    case PAINTING: return "PAINTING";
    case EMERGENCY_STOP: return "EMERGENCY_STOP";
    case LIMIT_FAULT: return "LIMIT_FAULT";
    default: return "UNKNOWN";
  }
}

bool emergencyPressed() {
  return digitalRead(ESTOP_PIN) == LOW;
}

bool limitXPressed() {
  return digitalRead(LIMIT_X_PIN) == LOW;
}

bool limitYPressed() {
  return digitalRead(LIMIT_Y_PIN) == LOW;
}

void setGun(bool enabled) {
  gunOn = enabled;
  digitalWrite(GUN_PIN, enabled ? HIGH : LOW);
}

void stopMotors() {
  digitalWrite(STEP_X, LOW);
  digitalWrite(STEP_Y, LOW);
  stepLevelX = false;
  stepLevelY = false;
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }
  return duration * 0.0343 / 2.0;
}

void printHelp() {
  Serial.println();
  Serial.println("===== COMANDOS =====");
  Serial.println("HELP   - Ver comandos");
  Serial.println("STATUS - Ver estado");
  Serial.println("SCAN   - Medir distancia");
  Serial.println("PAINT  - Iniciar recorrido");
  Serial.println("STOP   - Detener movimiento");
  Serial.println("RESET  - Restablecer sistema");
  Serial.println("X+     - Mover eje X adelante");
  Serial.println("X-     - Mover eje X atras");
  Serial.println("Y+     - Mover eje Y adelante");
  Serial.println("Y-     - Mover eje Y atras");
  Serial.println("====================");
}

void printStatus() {
  Serial.println();
  Serial.print("Estado: ");
  Serial.println(stateName());
  Serial.print("Posicion X: ");
  Serial.println(positionX);
  Serial.print("Posicion Y: ");
  Serial.println(positionY);
  Serial.print("Pistola: ");
  Serial.println(gunOn ? "ENCENDIDA" : "APAGADA");
  Serial.print("Distancia: ");
  Serial.print(readDistance());
  Serial.println(" cm");
}

void enterEmergency() {
  setGun(false);
  stopMotors();
  robotState = EMERGENCY_STOP;
  Serial.println("ALERTA: PARADA DE EMERGENCIA");
  Serial.println("Libera el boton y escribe RESET.");
}

void enterLimitFault(const char* axis) {
  setGun(false);
  stopMotors();
  robotState = LIMIT_FAULT;
  Serial.print("FINAL DE CARRERA ACTIVADO: ");
  Serial.println(axis);
  Serial.println("Revisa el sistema y escribe RESET.");
}

void beginPainting() {
  if (robotState == EMERGENCY_STOP || robotState == LIMIT_FAULT) {
    Serial.println("Fallo activo. No se puede pintar.");
    return;
  }

  positionX = 0;
  positionY = 0;
  pathIndex = 0;
  manualMove = false;
  targetX = pathX[0];
  targetY = pathY[0];
  
  setGun(false);
  robotState = PAINTING;

  Serial.println("Iniciando recorrido de prueba.");
  Serial.println("Pendiente.");
}

void updateAxis(int stepPin, int dirPin, long &position, long target, unsigned long &lastStep, bool &stepLevel, bool limitPressed, const char* axisName) {
  if (position == target) {
    if (stepLevel) {
      digitalWrite(stepPin, LOW);
      stepLevel = false;
    }
    return;
  }

  if (limitPressed) {
    enterLimitFault(axisName);
    return;
  }

  int direction = target > position ? HIGH : LOW;
  digitalWrite(dirPin, direction);

  unsigned long now = micros();

  if (now - lastStep >= STEP_INTERVAL_US) {
    lastStep = now;
    if (!stepLevel) {
      digitalWrite(stepPin, HIGH);
      stepLevel = true;
    } else {
      digitalWrite(stepPin, LOW);
      stepLevel = false;
      if (direction == HIGH) {
        position++;
      } else {
        position--;
      }
    }
  }
}

void updateMotors() {
  if (robotState != PAINTING) return;

  updateAxis(STEP_X, DIR_X, positionX, targetX, lastStepX, stepLevelX, limitXPressed(), "X");
  if (robotState != PAINTING) return;

  updateAxis(STEP_Y, DIR_Y, positionY, targetY, lastStepY, stepLevelY, limitYPressed(), "Y");
  if (robotState != PAINTING) return;

  if (positionX == targetX && positionY == targetY) {
    if (manualMove) {
      manualMove = false;
      robotState = IDLE;
      setGun(false);
      stopMotors();
      Serial.println("Movimiento manual terminado.");
      return;
    }

    if (pathIndex == 0) {
      pathIndex = 1;
      targetX = pathX[pathIndex];
      targetY = pathY[pathIndex];
      setGun(true);
      Serial.println("Pintando: punto 2.");
      return;
    }

    if (pathIndex < PATH_SIZE - 1) {
      pathIndex++;
      targetX = pathX[pathIndex];
      targetY = pathY[pathIndex];

      if (pathIndex == 3) {
        setGun(false);
        Serial.println("Cambio de tramo.");
      } else {
        setGun(true);
        Serial.print("Pintando: punto ");
        Serial.println(pathIndex + 1);
      }
      return;
    }

    setGun(false);
    stopMotors();
    robotState = IDLE;
    Serial.println("Recorrido terminado.");
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) return;

  Serial.print("> ");
  Serial.println(command);

  if (command == "HELP") {
    printHelp();
    return;
  }
  if (command == "STATUS") {
    printStatus();
    return;
  }
  if (command == "STOP") {
    setGun(false);
    stopMotors();
    if (robotState != EMERGENCY_STOP && robotState != LIMIT_FAULT) {
      robotState = IDLE;
    }
    Serial.println("Movimiento detenido.");
    return;
  }
  if (command == "RESET") {
    if (emergencyPressed()) {
      Serial.println("Libera el boton rojo primero.");
      return;
    }
    if (limitXPressed() || limitYPressed()) {
      Serial.println("Libera los finales de carrera primero.");
      return;
    }
    setGun(false);
    stopMotors();
    robotState = IDLE;
    manualMove = false;
    Serial.println("Sistema restablecido.");
    return;
  }
  
  if (robotState == EMERGENCY_STOP || robotState == LIMIT_FAULT) {
    Serial.println("Sistema bloqueado. Corrige el fallo y usa RESET.");
    return;
  }

  if (command == "SCAN") {
    robotState = SCANNING;
    Serial.println("Midiendo distancia...");
    float distance = readDistance();
    if (distance < 0) {
      Serial.println("No se recibio una lectura valida.");
    } else {
      Serial.print("Distancia detectada: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
    robotState = WAITING_AI;
    Serial.println("Esperando trayectoria.");
    return;
  }

  if (command == "PAINT") {
    beginPainting();
    return;
  }

  if (command == "X+" || command == "X-" || command == "Y+" || command == "Y-") {
    setGun(false);
    manualMove = true;
    robotState = PAINTING;

    if (command == "X+") {
      targetX = positionX + 100;
      targetY = positionY;
    } else if (command == "X-") {
      targetX = positionX - 100;
      targetY = positionY;
    } else if (command == "Y+") {
      targetX = positionX;
      targetY = positionY + 100;
    } else {
      targetX = positionX;
      targetY = positionY - 100;
    }
    Serial.println("Movimiento manual iniciado.");
    return;
  }
  Serial.println("Comando desconocido. Escribe HELP.");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Y_PIN, INPUT_PULLUP);

  pinMode(GUN_PIN, OUTPUT);

  setGun(false);
  stopMotors();

  digitalWrite(DIR_X, LOW);
  digitalWrite(DIR_Y, LOW);

  robotState = IDLE;

  Serial.println();
  Serial.println("===============================");
  Serial.println("ROBOT DE PINTURA CON ESP32");
  Serial.println("PRUEBA SERIAL CORRECTA");
  Serial.println("Estado: IDLE");
  Serial.println("===============================");
  printHelp();
}

void loop() {
  if (emergencyPressed()) {
    if (robotState != EMERGENCY_STOP) {
      enterEmergency();
    }
  }

  if (robotState != EMERGENCY_STOP && robotState != LIMIT_FAULT) {
    updateMotors();
  }

  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  unsigned long now = millis();

  if (now - lastTelemetry >= 1000) {
    lastTelemetry = now;
    Serial.print("[ESTADO] ");
    Serial.print(stateName());
    Serial.print(" | X=");
    Serial.print(positionX);
    Serial.print(" | Y=");
    Serial.print(positionY);
    Serial.print(" | Pistola=");
    Serial.println(gunOn ? "ON" : "OFF");
  }

  if (now - lastStatus >= 5000 && robotState == IDLE) {
    lastStatus = now;
    Serial.println("Robot en espera. Envia SCAN o PAINT.");
  }
}