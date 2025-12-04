// 통합 하드웨어 인터페이스 V2 - Arduino 단일 파일 템플릿
// territory_game_flip_v.cpp의 검증된 로직(상하 반전, 핀 매핑) 적용
// .ino 파일로 복사해서 바로 사용 가능

// 실행 명령어 형식: g++ -std=c++17 -DTARGET_PC "경로/파일명.cpp" -o "경로/파일명(.cpp 없음)"

#ifdef TARGET_PC
// ================= PC 빌드용 스텁 =================
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#define F(x) x
#define HEX 16
#define A2 0
#define INPUT_PULLUP 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0

// 시간 함수 모의
inline void delay(unsigned long ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
inline void delayMicroseconds(int us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}
unsigned long millis() {
  using namespace std::chrono;
  static auto start = steady_clock::now();
  auto now = steady_clock::now();
  return duration_cast<milliseconds>(now - start).count();
}

// GPIO 함수 모의
void pinMode(int pin, int mode) {}
void digitalWrite(int pin, int value) {}
int digitalRead(int pin) { return 1; } // 기본적으로 눌리지 않음(HIGH)

// Serial 모의 클래스 (통합)
struct SerialMock {
  void begin(unsigned long) {}

  void print(const char *s) { std::printf("%s", s); }
  void print(char c) { std::printf("%c", c); }
  void print(int v) { std::printf("%d", v); }
  void print(unsigned int v) { std::printf("%u", v); }
  void print(long v) { std::printf("%ld", v); }
  void print(unsigned long v) { std::printf("%lu", v); }
  void print(uint8_t v) { std::printf("%u", v); }

  void print(uint8_t v, int base) {
    if (base == HEX)
      std::printf("%X", v);
    else
      std::printf("%u", v);
  }

  void println() {
    std::printf("\n");
    std::fflush(stdout);
  }
  void println(const char *s) {
    std::printf("%s\n", s);
    std::fflush(stdout);
  }
  void println(char c) {
    std::printf("%c\n", c);
    std::fflush(stdout);
  }
  void println(int v) {
    std::printf("%d\n", v);
    std::fflush(stdout);
  }
  void println(unsigned int v) {
    std::printf("%u\n", v);
    std::fflush(stdout);
  }
  void println(long v) {
    std::printf("%ld\n", v);
    std::fflush(stdout);
  }
  void println(unsigned long v) {
    std::printf("%lu\n", v);
    std::fflush(stdout);
  }
  void println(uint8_t v) {
    std::printf("%u\n", v);
    std::fflush(stdout);
  }

  int available() { return 0; } // PC에서는 직접 입력 받음
  int read() { return -1; }

  // PC 입력 처리
  int parseInt() {
    char buffer[10];
    if (fgets(buffer, sizeof(buffer), stdin)) {
      return atoi(buffer);
    }
    return -1;
  }
} Serial;

// NeoPixel 모의 클래스
#define NEO_GRB 0
#define NEO_KHZ800 0

class Adafruit_NeoPixel {
public:
  Adafruit_NeoPixel(int n, int /*pin*/, int /*flags*/)
      : _n(n), _brightness(20) {
    _pixels = new uint32_t[_n];
    clear();
  }
  ~Adafruit_NeoPixel() { delete[] _pixels; }

  void begin() {}
  void show() {}
  void setBrightness(uint8_t b) { _brightness = b; }
  uint8_t getBrightness() const { return _brightness; }
  void clear() {
    for (int i = 0; i < _n; ++i)
      _pixels[i] = 0;
  }

  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
  }

  void setPixelColor(int i, uint32_t c) {
    if (i >= 0 && i < _n)
      _pixels[i] = c;
  }

  uint32_t getPixelColor(int i) const {
    if (i >= 0 && i < _n)
      return _pixels[i];
    return 0;
  }

private:
  int _n;
  uint8_t _brightness;
  uint32_t *_pixels;
};

#else
// ================= 실제 Arduino 빌드용 =================
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#endif

// ============================================================================
// 1. 하드웨어 설정 (Configuration)
// ============================================================================

// 자석 모듈 핀 설정 (6x6 그리드)
// Row output pins (6개): 13, 12, 11, 10, 9, 8
#define ROW_PIN_START 13
#define ROW_PIN_END 8
#define ROW_COUNT 6

// Column input pins (6개): 7, 6, 5, 4, 3, 2
#define COL_PIN_START 7
#define COL_PIN_END 2
#define COL_COUNT 6

// LED 설정
#define LED_PIN A2
#define LED_COUNT 256
#define W 16
#define H 16

// 전역 객체
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// 2. 자료형 및 전역 변수
// ============================================================================

enum InputMode {
  INPUT_MAGNETIC, // 자석 모듈 입력
  INPUT_SERIAL    // 시리얼 입력
};

// 내부 상태
static unsigned long lastInputTime = 0;
static const unsigned long DEBOUNCE_DELAY = 200;
static int lastNodeInput = -1;

InputMode currentInputMode = INPUT_SERIAL;

// 디스플레이 상태
int currentFrameNumber = 0;
uint8_t lastBrightness = 255;
float animationSpeed = 5.0; // 6-Queens용 속도 조절

// ============================================================================
// 3. 좌표 변환 함수 (핵심 로직)
// ============================================================================

// 자석 모듈 (행, 열) -> 노드 번호 (0~35)
int coordToNode(int row, int col) {
  if (row < 0 || row >= ROW_COUNT || col < 0 || col >= COL_COUNT) {
    return -1;
  }
  return row * COL_COUNT + col;
}

// 노드 번호 -> 자석 모듈 (행, 열)
void nodeToCoord(int node, int *row, int *col) {
  if (node < 0 || node >= ROW_COUNT * COL_COUNT) {
    *row = -1;
    *col = -1;
    return;
  }
  *row = node / COL_COUNT;
  *col = node % COL_COUNT;
}

// LED 좌표 (x, y) -> LED 인덱스 (0~255)
// [상하 반전 적용됨: territory_game_flip_v.cpp 로직]
int xyToIndex(int x, int y) {
  // y 좌표를 반전 (0 -> 15, 15 -> 0)
  int mirroredY = (H - 1) - y;

  if (mirroredY % 2 == 0)
    return mirroredY * W + x;
  else
    return mirroredY * W + (W - 1 - x);
}

// 노드 번호 -> LED 좌표 (x, y)
// 6x6 그리드가 16x16 LED 매트릭스 중앙에 오도록 매핑 (2칸 오프셋)
void nodeToLED(int node, int *ledX, int *ledY) {
  int row, col;
  nodeToCoord(node, &row, &col);

  // 16x16 화면 중앙에 12x12(6*2) 체스판을 놓기 위한 오프셋
  int offsetX = 2;
  int offsetY = 2;

  if (row >= 0 && col >= 0) {
    *ledX = col * 2 + offsetX;
    *ledY = row * 2 + offsetY;
  } else {
    *ledX = -1;
    *ledY = -1;
  }
}

// ============================================================================
// 4. 입력 함수
// ============================================================================

void inputInit() {
  // Row 핀 (13~8): OUTPUT으로 설정
  for (int pin = ROW_PIN_END; pin <= ROW_PIN_START; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH); // 기본값 HIGH
  }

  // Column 핀 (7~2): INPUT_PULLUP으로 설정
  for (int pin = COL_PIN_END; pin <= COL_PIN_START; pin++) {
    pinMode(pin, INPUT_PULLUP);
  }

  lastInputTime = 0;
  lastNodeInput = -1;
}

int readMagneticInput() {
#ifdef TARGET_PC
  return -1;
#else
  unsigned long currentTime = millis();

  if (currentTime - lastInputTime < DEBOUNCE_DELAY) {
    return -1;
  }

  for (int row = 0; row < ROW_COUNT; row++) {
    int rowPin = ROW_PIN_START - row;
    digitalWrite(rowPin, LOW);
    delayMicroseconds(10);

    for (int col = 0; col < COL_COUNT; col++) {
      int colPin = COL_PIN_START - col;
      if (digitalRead(colPin) == LOW) {
        int nodeNum = coordToNode(row, col);
        digitalWrite(rowPin, HIGH);
        for (int r = 0; r < ROW_COUNT; r++)
          digitalWrite(ROW_PIN_START - r, HIGH);

        if (nodeNum == lastNodeInput)
          return -1;

        lastInputTime = currentTime;
        lastNodeInput = nodeNum;
        return nodeNum;
      }
    }
    digitalWrite(rowPin, HIGH);
  }
  return -1;
#endif
}

int readSerialInput() {
#ifdef TARGET_PC
  char buffer[10];
  if (fgets(buffer, sizeof(buffer), stdin)) {
    int node = atoi(buffer);
    if (node >= 0 && node < ROW_COUNT * COL_COUNT) {
      if (node == lastNodeInput)
        return -1;
      lastNodeInput = node;
      return node;
    }
  }
  return -1;
#else
  if (Serial.available() > 0) {
    int node = Serial.parseInt();
    if (node >= 0 && node < ROW_COUNT * COL_COUNT) {
      while (Serial.available() > 0)
        Serial.read();
      if (node == lastNodeInput)
        return -1;
      lastNodeInput = node;
      return node;
    }
  }
  return -1;
#endif
}

int readInput(InputMode mode) {
  if (mode == INPUT_MAGNETIC)
    return readMagneticInput();
  else
    return readSerialInput();
}

void setInputMode(InputMode mode) {
  currentInputMode = mode;
  if (mode == INPUT_MAGNETIC)
    Serial.println(F("Input mode: Magnetic sensor"));
  else
    Serial.println(F("Input mode: Serial"));
}

// ============================================================================
// 5. 출력 함수 (LED & Serial)
// ============================================================================

void printHex(uint8_t value) {
  if (value < 16)
    Serial.print("0");
  Serial.print(value, HEX);
}

void serialPrintHeader() {
#if USE_SERIAL
  Serial.println(F("=== 6-Queens Backtracking ==="));
  Serial.print(F("SIZE:"));
  Serial.print(W);
  Serial.print(F("x"));
  Serial.println(H);
  Serial.print(F("BRIGHTNESS:"));
  Serial.println(strip.getBrightness());
  Serial.println(F("============================"));
#endif
}

void serialPrintBrightnessChange() {
#if USE_SERIAL
  uint8_t currentBrightness = strip.getBrightness();
  if (currentBrightness != lastBrightness) {
    Serial.print(F("BRIGHTNESS:"));
    Serial.println(currentBrightness);
    lastBrightness = currentBrightness;
  }
#endif
}

void serialPrintFrame() {
#if USE_SERIAL
  Serial.print(F("FRAME:"));
  Serial.println(currentFrameNumber);
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int ledIndex = xyToIndex(x, y);
      uint32_t color = strip.getPixelColor(ledIndex);
      uint8_t r = (color >> 16) & 0xFF;
      uint8_t g = (color >> 8) & 0xFF;
      uint8_t b = color & 0xFF;
      printHex(r);
      printHex(g);
      printHex(b);
      if (x < W - 1)
        Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println(F("---"));
  currentFrameNumber++;
#endif
}

void displayInit() {
#if USE_SERIAL
  Serial.begin(115200);
  delay(2000);
#endif
#ifndef TARGET_PC
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  strip.begin();
  strip.show();
  strip.setBrightness(20);
#else
  strip.begin();
  strip.setBrightness(20);
#endif

  serialPrintHeader();
  lastBrightness = strip.getBrightness();
}

void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  if (x < 0 || x >= W || y < 0 || y >= H)
    return;
  int index = xyToIndex(x, y);
  strip.setPixelColor(index, strip.Color(r, g, b));
}

void displayShow() {
  serialPrintBrightnessChange();
#ifndef TARGET_PC
  strip.show();
#endif
  serialPrintFrame();
}

void displayClear() { strip.clear(); }
void setBrightness(uint8_t level) { strip.setBrightness(level); }

void displayDelay(unsigned long ms) {
  if (animationSpeed <= 0.0)
    animationSpeed = 1.0;
  delay((unsigned long)(ms / animationSpeed));
}

// ============================================================================
// 6. 6-Queens 알고리즘 구현
// ============================================================================

#define N 6 // 6x6 체스판

int queens[N]; // 퀸의 위치 저장 (각 행에 하나씩, queens[row] = col)
int fixedRow = -1; // 시작점으로 고정된 행 (사용자가 자석을 놓은 행)
int solutionCount = 0;
int backtrackCount = 0;

// 2x2 픽셀 블록으로 체스판 칸 그리기 (중앙 정렬)
void drawSquare(int col, int row, uint8_t r, uint8_t g, uint8_t b) {
  int offsetX = 2; // 화면 중앙 정렬을 위한 오프셋
  int offsetY = 2;

  int baseX = col * 2 + offsetX;
  int baseY = row * 2 + offsetY;

  for (int dy = 0; dy < 2; dy++) {
    for (int dx = 0; dx < 2; dx++) {
      setPixel(baseX + dx, baseY + dy, r, g, b);
    }
  }
}

// 퀸이 공격할 수 있는지 확인 (상하좌우 및 대각선 모두 확인)
// 사실상 queens 배열 구조(index=row, value=col)상 같은 행(row) 공격은 불가능함.
// 따라서 기존 로직(같은 열, 대각선)만으로 충분함.
bool isUnderAttack(int testRow, int testCol) {
  // 모든 행을 검사 (고정된 퀸 포함)
  for (int r = 0; r < N; r++) {
    // 아직 퀸이 배치되지 않은 행은 건너뜀 (단, 현재 테스트 중인 행 제외)
    if (queens[r] == -1 || r == testRow) continue; 

    int c = queens[r]; // r행에 있는 퀸의 열 위치

    // 1. 같은 열(Column) 공격 확인
    if (c == testCol) return true;

    // 2. 대각선(Diagonal) 공격 확인
    // |r1 - r2| == |c1 - c2| 이면 대각선 상에 있음
    if (abs(r - testRow) == abs(c - testCol)) return true;
  }
  return false;
}

// 전체 체스판 그리기
void drawBoard(int currentRow, int tryCol, bool showAttack) {
  displayClear();

  for (int row = 0; row < N; row++) {
    for (int col = 0; col < N; col++) {
      uint8_t r = 0, g = 0, b = 0;
      bool isWhiteSquare = (row + col) % 2 == 0;

      // 배치된 퀸인가?
      bool hasQueen = false;
      // 현재 행보다 이전에 배치되었거나, 아니면 고정된 행(fixedRow)에 있는 경우
      if (queens[row] != -1 && (row < currentRow || row == fixedRow)) {
        if(queens[row] == col) hasQueen = true;
      }

      if (hasQueen) {
        if (row == fixedRow) {
            r = 0; g = 255; b = 255; // 고정된 시작 퀸 - 청록색 (Cyan)
        } else {
            r = 0; g = 0; b = 255;   // 배치된 퀸 - 파란색
        }
      } 
      else if (row == currentRow && col == tryCol) {
        // 현재 시도 중인 위치
        if (showAttack && isUnderAttack(currentRow, tryCol)) {
          r = 255; g = 0; b = 0; // 공격당함 - 빨간색
        } else {
          r = 255; g = 255; b = 0; // 시도 중 - 노란색
        }
      } 
      else if (row == currentRow && showAttack) {
        // 공격 범위 미리보기
        if (isUnderAttack(currentRow, col)) {
          r = 100; g = 0; b = 0; // 어두운 빨강
        } else {
          r = 0; g = 80; b = 0; // 어두운 초록
        }
      } 
      else {
        // 빈 체스판
        if (isWhiteSquare) {
          r = 80; g = 80; b = 80;
        } else {
          r = 20; g = 20; b = 20;
        }
      }
      drawSquare(col, row, r, g, b);
    }
  }
  displayShow();
}

// 해를 찾았을 때 강조
void showSolution() {
  displayClear();
  for (int row = 0; row < N; row++) {
    for (int col = 0; col < N; col++) {
      uint8_t r, g, b;
      bool isWhiteSquare = (row + col) % 2 == 0;

      if (queens[row] == col) {
        r = 0; g = 255; b = 0; // 퀸 - 초록색
      } else {
        if (isWhiteSquare) {
          r = 80; g = 80; b = 80;
        } else {
          r = 20; g = 20; b = 20;
        }
      }
      drawSquare(col, row, r, g, b);
    }
  }
  displayShow();
}

// 백트래킹 애니메이션
void showBacktrack(int row) {
  Serial.print(F("Backtracking from row "));
  Serial.println(row);
  backtrackCount++;

  for (int i = 0; i < 2; i++) {
    drawSquare(queens[row], row, 255, 120, 0); // 주황색 깜빡임
    displayShow();
    displayDelay(200);
    drawSquare(queens[row], row, 0, 0, 0);
    displayShow();
    displayDelay(200);
  }
  // 백트래킹 시 해당 위치 초기화
  queens[row] = -1;
}

// 백트래킹 알고리즘 (수정됨: 고정된 행 건너뛰기)
bool solveNQueens(int row) {
  // 모든 행을 다 채웠으면 성공 (N번째 행까지 도달)
  if (row == N) {
    solutionCount++;
    Serial.print(F("Solution #"));
    Serial.print(solutionCount);
    Serial.print(F(" found! (Backtracks: "));
    Serial.print(backtrackCount);
    Serial.println(F(")"));

    showSolution();
    displayDelay(3000);
    return true;
  }

  // 만약 현재 행이 사용자가 고정한 행(fixedRow)이라면,
  // 이미 퀸이 배치되어 있으므로 다음 행으로 바로 넘어갑니다.
  if (row == fixedRow) {
      return solveNQueens(row + 1);
  }

  // 현재 행의 각 열(Col)을 시도
  for (int col = 0; col < N; col++) {
    Serial.print(F("Trying row "));
    Serial.print(row);
    Serial.print(F(", col "));
    Serial.println(col);

    drawBoard(row, col, false);
    displayDelay(300);

    drawBoard(row, col, true);
    displayDelay(500);

    // 공격받지 않는 위치라면 퀸 배치
    if (!isUnderAttack(row, col)) {
      queens[row] = col;
      Serial.print(F("Placing queen at row "));
      Serial.print(row);
      Serial.print(F(", col "));
      Serial.println(col);

      drawBoard(row, -1, false); // 다음 행으로 넘어가기 전 현재 상태 표시
      displayDelay(800);

      // 다음 행으로 재귀 호출
      if (solveNQueens(row + 1)) {
        return true;
      }

      // 실패 시 백트래킹 (퀸 회수)
      showBacktrack(row);
    } else {
      Serial.println(F("Position under attack, trying next..."));
      displayDelay(300);
    }
  }
  return false;
}

// ============================================================================
// 7. 메인 Setup & Loop (수정됨)
// ============================================================================

void setup() {
  displayInit();
  inputInit(); // 자석 모듈 초기화
  setBrightness(15);

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   6-Queens Backtracking (Magnetic Start)"));
  Serial.println(F("========================================"));
  Serial.println(F(""));

// PC가 아니면 자석 입력 모드를 기본으로 사용
#ifdef TARGET_PC
  setInputMode(INPUT_SERIAL);
#else
  setInputMode(INPUT_MAGNETIC);
#endif

  // 초기화
  for (int i = 0; i < N; i++) {
    queens[i] = -1;
  }
  solutionCount = 0;
  backtrackCount = 0;
}

void loop() {
  // 1. 대기 상태: 자석 입력이 들어올 때까지 대기
  Serial.println(F("Waiting for magnetic input to start..."));
  
  // 빈 보드 표시 (대기 중임을 알림)
  displayClear();
  drawBoard(0, -1, false); 
  displayShow();

  int startNode = -1;
  while(startNode == -1) {
    // 정의된 모드에 따라 입력 받음 (아두이노는 Magnetic, PC는 Serial)
    startNode = readInput(currentInputMode);
    
    // 입력이 감지되면 피드백 (잠깐 깜빡임)
    if (startNode != -1) {
       int r, c;
       nodeToCoord(startNode, &r, &c);
       // 시각적 피드백: 선택된 칸을 초록색으로 표시
       drawSquare(c, r, 0, 255, 0); 
       displayShow();
       displayDelay(500);
    } else {
       delay(50); // polling delay
    }
  }

  // 2. 시작점 설정: 입력된 노드의 정확한 (Row, Col)을 첫 번째 퀸의 위치로 사용
  int startRow, startCol;
  nodeToCoord(startNode, &startRow, &startCol);

  Serial.print(F("Input received at ("));
  Serial.print(startRow); Serial.print(F(", ")); Serial.print(startCol);
  Serial.println(F(")"));
  
  // 고정된 행(fixedRow) 설정
  fixedRow = startRow;

  // 리셋
  for (int i = 0; i < N; i++) queens[i] = -1;
  solutionCount = 0;
  backtrackCount = 0;

  // 사용자가 선택한 위치에 퀸 고정
  queens[fixedRow] = startCol;

  // 3. 탐색 시작: Row 0부터 해결 시작 (solveNQueens 내부에서 fixedRow는 건너뜀)
  // 먼저 고정된 퀸을 화면에 그려줍니다.
  drawBoard(0, -1, false); 
  displayDelay(1000);

  if (!solveNQueens(0)) {
    Serial.println(F("No solution found from this start point!"));
    // 실패 시 고정된 퀸 위치에서 빨간색으로 깜빡임
    for(int i=0; i<3; i++) {
        drawSquare(startCol, startRow, 255, 0, 0);
        displayShow();
        displayDelay(300);
        drawSquare(startCol, startRow, 0, 0, 0);
        displayShow();
        displayDelay(300);
    }
  }

  Serial.print(F("Total solutions: "));
  Serial.println(solutionCount);
  Serial.print(F("Total backtracks: "));
  Serial.println(backtrackCount);

  displayDelay(5000);
  
  // 다음 루프를 위해 상태 초기화
  fixedRow = -1;
}

#ifdef TARGET_PC
int main() {
  setup();
  while (true) {
    loop();
  }
  return 0;
}
#endif