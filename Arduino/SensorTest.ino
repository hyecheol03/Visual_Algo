#define ROWS 6
#define COLS 6

int rowPins[ROWS] = {13, 12, 11, 10, 9, 8};  // 행 전원 핀
int colPins[COLS] = {7, 6, 5, 4, 3, 2};  // 열 센서 입력 핀

void setup() {
    Serial.begin(9600);
    Serial.println("=== Magnet Sensor Matrix Test Start ===");

    // 행 핀 출력 설정
    for (int r = 0; r < ROWS; r++) {
        pinMode(rowPins[r], OUTPUT);
        digitalWrite(rowPins[r], HIGH); // 기본은 OFF
    }

    // 열 핀 입력 설정
    for (int c = 0; c < COLS; c++) {
        pinMode(colPins[c], INPUT_PULLUP);
    }

    delay(500);
}

void loop() {
    int matrix[ROWS][COLS] = {0};

    // 각 행을 순차적으로 활성화
    for (int r = 0; r < ROWS; r++) {
        digitalWrite(rowPins[r], LOW);
        delay(50);

        for (int c = 0; c < COLS; c++) {
            int val = digitalRead(colPins[c]);
            matrix[r][c] = !val;  // LOW면 감지됨
        }

        digitalWrite(rowPins[r], HIGH);
    }

    // 결과 출력
    Serial.println("----- Sensor Matrix -----");
    for (int r = 0; r < ROWS; r++) {
        Serial.print("Row ");
        Serial.print(r);
        Serial.print(": ");
        for (int c = 0; c < COLS; c++) {
            Serial.print(matrix[r][c]);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println("--------------------------\n");

    delay(500);
}