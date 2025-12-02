// --------- 변수 선언 및 전처리 ---------
#include <Adafruit_NeoPixel.h>

#define ROWS 8
#define COLS 8

// 행(Row): D0~D7
int rowPins[ROWS] = {0, 1, 2, 3, 4, 5, 6, 7};
// 열(Col): D8~D13, A0~A1
int colPins[COLS] = {8, 9, 10, 11, 12, 13, A0, A1};

// LED 전처리
#define LED_PIN    6
#define LED_WIDTH  16
#define LED_HEIGHT 16
#define LED_COUNT  (LED_WIDTH * LED_HEIGHT)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Pixel 구조체
struct Pixel {
    uint16_t hue;
    uint8_t brightness;
};

// 내부 버퍼
Pixel ledBuffer[LED_HEIGHT][LED_WIDTH];
int magnetSensors[ROWS][COLS];

// --------- 헬퍼 함수 ---------
int xyToIndex(int r, int c) {
    if (r % 2 == 1)
        return r * LED_WIDTH + c;
    else
        return r * LED_WIDTH + (LED_WIDTH - 1 - c);
}

// --------- IO 구현부 ---------

// 모든 자석센서 갱신
int** magnetInput() {
    for (int r = 0; r < ROWS; r++) {
        digitalWrite(rowPins[r], LOW);
        delay(50);

        for (int c = 0; c < COLS; c++) {
            int val = digitalRead(colPins[c]);
            matrix[r][c] = !val;
        }

        digitalWrite(rowPins[r], HIGH);
    }

    return matrix;
}

// 특정 위치 센서값
int magnetInput(int r, int c) {
    return magnetSensors[r][c];
}

// LED 버퍼에 기록
void LEDBuffer(int r, int c, Pixel p) {
    if (r < LED_HEIGHT && c < LED_WIDTH)
        ledBuffer[r][c] = p;
}

// 버퍼의 내용 출력
void LEDOutput() {
    for (int r = 0; r < LED_HEIGHT; r++) {
        for (int c = 0; c < LED_WIDTH; c++) {
            Pixel p = ledBuffer[r][c];
            uint32_t color = strip.ColorHSV(p.hue, 255, p.brightness);
            strip.setPixelColor(xyToIndex(r, c), color);
        }
    }
    strip.show();

    // 필요 시 버퍼 초기화
    for (int r = 0; r < LED_HEIGHT; r++)
        for (int c = 0; c < LED_WIDTH; c++)
            ledBuffer[r][c] = {0, 0};
}

// --------- 사용자 정의 소프트웨어 (예시) ---------
void examplefunc() {
    int** matrix = magnetInput();
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (matrix[i][j])
                LEDBuffer(i, j, {(uint16_t)((i + j) * 1000), 64});
        }
    }
    LEDOutput();
}

// --------- main ---------
void setup() {
    strip.begin();
    strip.show();

    // 핀 초기화
    for (int r = 0; r < ROWS; r++) {
        pinMode(rowPins[r], OUTPUT);
        digitalWrite(rowPins[r], HIGH);
    }
    for (int c = 0; c < COLS; c++) {
        pinMode(colPins[c], INPUT_PULLUP);
    }
}

void loop() {
    examplefunc();
    delay(1000);
}
