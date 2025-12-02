#include <Adafruit_NeoPixel.h>

//--------- 설정 및 전처리 ---------
#define ROWS 6
#define COLS 6

// [핀 설정]
#define LED_PIN    A2

// 행 (출력, LOW)
int rowPins[ROWS] = {13, 12, 11, 10, 9, 8};
// 열 (입력, PULL_INPUT)
int colPins[COLS] = {7, 6, 5, 4, 3, 2};

#define MAP_ROW_OFFSET 2 // 위아래 여백
#define MAP_COL_OFFSET 2 // 좌우 여백

// LED 스트립 설정 (16x16)
#define LED_WIDTH  16       
#define LED_HEIGHT 16       
#define LED_COUNT  (LED_WIDTH * LED_HEIGHT)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

struct Pixel {
    uint8_t hue;
    uint8_t brightness;
    bool isWhite;
};

// 전역 변수
int sensorData[ROWS][COLS];
int* sensorRowPtrs[ROWS]; 
bool visited[ROWS][COLS]; 
bool wallMap[ROWS][COLS]; 

// 방향
int dr[4] = {-1, 0, 1, 0};
int dc[4] = {0, 1, 0, -1};

//--------- 헬퍼 함수 ---------
int xyToIndex(int r, int c) {
    if (r < 0 || r >= LED_HEIGHT || c < 0 || c >= LED_WIDTH) return -1;
    if (r % 2 == 1) return r * LED_WIDTH + c;
    else            return r * LED_WIDTH + (LED_WIDTH - 1 - c);
}

//---------- 하드웨어 IO ----------
int** magnetInput() {
    static bool initialized = false;
    if (!initialized) {
        for (int i = 0; i < ROWS; i++) sensorRowPtrs[i] = sensorData[i];
        initialized = true;
    }

    for (int r = 0; r < ROWS; r++) {
        digitalWrite(rowPins[r], LOW); 
        delay(5);         
        for (int c = 0; c < COLS; c++) {
            sensorData[r][c] = !digitalRead(colPins[c]);
        }
        digitalWrite(rowPins[r], HIGH); 
    }
    return sensorRowPtrs;
}

//---------- 시각화 함수들 ----------
void drawNode(int r, int c, Pixel p) {
    int ledR = r * 2 + MAP_ROW_OFFSET;
    int ledC = c * 2 + MAP_COL_OFFSET;
    
    uint32_t color;
    if (p.isWhite) {
        color = strip.Color(255, 255, 255); 
    } else if (p.brightness == 0) {
        color = 0; 
    } else if (p.hue == 0 && p.brightness == 50) { 
        color = strip.Color(30, 30, 30); 
    } else {
        color = strip.ColorHSV((uint16_t)p.hue * 256, 255, p.brightness);
    }
    
    int idx = xyToIndex(ledR, ledC);
    if (idx != -1) strip.setPixelColor(idx, color);
}

void drawBridge(int r1, int c1, int r2, int c2, Pixel p) {
    int ledR1 = r1 * 2 + MAP_ROW_OFFSET;
    int ledC1 = c1 * 2 + MAP_COL_OFFSET;
    int ledR2 = r2 * 2 + MAP_ROW_OFFSET;
    int ledC2 = c2 * 2 + MAP_COL_OFFSET;
    
    int midR = (ledR1 + ledR2) / 2;
    int midC = (ledC1 + ledC2) / 2;
    
    uint32_t color;
    if (p.isWhite) {
        color = strip.Color(255, 255, 255); 
    } else if (p.brightness == 0) {
        color = 0;
    } else if (p.hue == 0 && p.brightness == 50) {
        color = strip.Color(30, 30, 30); 
    } else {
        color = strip.ColorHSV((uint16_t)p.hue * 256, 255, p.brightness);
    }

    int idx = xyToIndex(midR, midC);
    if (idx != -1) strip.setPixelColor(idx, color);
}

void drawAdvancedWalls() {
    Pixel wallPixel = {0, 255, true}; 
    uint32_t whiteColor = strip.Color(255, 255, 255);

    for(int r=0; r<ROWS; r++) {
        for(int c=0; c<COLS; c++) {
            if (wallMap[r][c]) {
                drawNode(r, c, wallPixel);

                if (c + 1 < COLS && wallMap[r][c+1]) drawBridge(r, c, r, c+1, wallPixel);
                if (r + 1 < ROWS && wallMap[r+1][c]) drawBridge(r, c, r+1, c, wallPixel);

                int ledR = r * 2 + MAP_ROW_OFFSET;
                int ledC = c * 2 + MAP_COL_OFFSET;

                if (c == 0) { int idx = xyToIndex(ledR, ledC - 1); if(idx!=-1) strip.setPixelColor(idx, whiteColor); }
                if (c == COLS - 1) { int idx = xyToIndex(ledR, ledC + 1); if(idx!=-1) strip.setPixelColor(idx, whiteColor); }
                if (r == 0) { int idx = xyToIndex(ledR - 1, ledC); if(idx!=-1) strip.setPixelColor(idx, whiteColor); }
                if (r == ROWS - 1) { int idx = xyToIndex(ledR + 1, ledC); if(idx!=-1) strip.setPixelColor(idx, whiteColor); }
            }
        }
    }
}

void drawBorder() {
    int rStart = MAP_ROW_OFFSET - 1;
    int rEnd   = MAP_ROW_OFFSET + (ROWS * 2) - 1;
    int cStart = MAP_COL_OFFSET - 1;
    int cEnd   = MAP_COL_OFFSET + (COLS * 2) - 1;
    
    uint32_t borderColor = strip.Color(255, 255, 255); 

    for (int c = cStart; c <= cEnd; c++) {
        int idx1 = xyToIndex(rStart, c); int idx2 = xyToIndex(rEnd, c);
        if (idx1 != -1) strip.setPixelColor(idx1, borderColor);
        if (idx2 != -1) strip.setPixelColor(idx2, borderColor);
    }
    for (int r = rStart; r <= rEnd; r++) {
        int idx1 = xyToIndex(r, cStart); int idx2 = xyToIndex(r, cEnd);
        if (idx1 != -1) strip.setPixelColor(idx1, borderColor);
        if (idx2 != -1) strip.setPixelColor(idx2, borderColor);
    }
}

void drawDecisionStack(int count, Pixel p, bool push) {
    int stackRow = 15;
    int idx = xyToIndex(stackRow, count);
    if (idx != -1) {
        if (push) {
            uint32_t color = strip.ColorHSV((uint16_t)p.hue * 256, 255, 200);
            strip.setPixelColor(idx, color);
        } else {
            strip.setPixelColor(idx, 0);
        }
        strip.show();
    }
}

//---------- DFS 알고리즘 ----------
bool dfsRecursive(int r, int c, int decisionDepth, uint8_t currentHue) {
    if (r < 0 || r >= ROWS || c < 0 || c >= COLS) return false;
    if (wallMap[r][c]) return false;
    if (visited[r][c]) return false;

    visited[r][c] = true;
    
    // 방문(전진): 현재 색상, 밝기 150
    drawNode(r, c, {currentHue, 150, false});
    strip.show();
    delay(400); 

    int validNeighbors = 0;
    for (int i = 0; i < 4; i++) {
        int nr = r + dr[i];
        int nc = c + dc[i];
        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && 
            !visited[nr][nc] && !wallMap[nr][nc]) {
            validNeighbors++;
        }
    }
    
    bool isJunction = (validNeighbors > 1);
    if (isJunction) drawDecisionStack(decisionDepth, {currentHue, 0, false}, true);

    int branchCount = 0;
    for (int i = 0; i < 4; i++) {
        int nextR = r + dr[i];
        int nextC = c + dc[i];
        
        if (nextR >= 0 && nextR < ROWS && nextC >= 0 && nextC < COLS && 
            !visited[nextR][nextC] && !wallMap[nextR][nextC]) {
            
            uint8_t nextHue = currentHue;
            if (isJunction && branchCount > 0) {
                nextHue = (currentHue + 40) % 255; 
            }
            
            // 탐색: 현재 색상, 밝기 150
            drawBridge(r, c, nextR, nextC, {nextHue, 150, false});
            strip.show();
            delay(200); 

            int nextDecisionDepth = isJunction ? decisionDepth + 1 : decisionDepth;
            
            if (dfsRecursive(nextR, nextC, nextDecisionDepth, nextHue)) return true; 
            
            // 백트래킹: 빨간색(Hue 0), 밝기 150
            drawBridge(r, c, nextR, nextC, {0, 150, false}); 
            strip.show();
            delay(200);
            
            branchCount++;
        }
    }

    if (isJunction) drawDecisionStack(decisionDepth, {0,0,false}, false);
    
    // 현재 노드에서 나갈 때: 빨간색(Hue 0), 밝기 150
    if (!(r == 0 && c == 0)) {
        drawNode(r, c, {0, 150, false}); 
        strip.show();
    }

    return false;
}

void startDFS() {
    // 맵 초기화, 벽 정보 누적
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            visited[i][j] = false; 
        }
    }
    
    wallMap[0][0] = 0; 
    
    strip.clear();
    drawBorder(); 
    drawAdvancedWalls();
    strip.show();
    
    dfsRecursive(0, 0, 0, 80); 
    
    delay(3000); 

    // 맵 초기화
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            wallMap[i][j] = false;
        }
    }
}

void playStartEffect() {
    for(int k=0; k<3; k++){
        drawNode(0, 0, {0, 255, true});
        strip.show();
        delay(150);
        drawNode(0, 0, {0, 0, false});
        strip.show();
        delay(150);
    }
}

void runEditorMode(int** input) {
    strip.clear();
    drawBorder();
    
    // 벽 누적
    for(int i=0; i<ROWS; i++) {
        for(int j=0; j<COLS; j++) {
            if (input[i][j] == 1) {
                wallMap[i][j] = true;
            }
        }
    }
            
    drawAdvancedWalls();
    strip.show();
}

void runMatrixSystem() {
    int** currentInput = magnetInput();

    // (0,0)에 자석이 감지되면 게임 시작
    if (currentInput[0][0] == 1) {
        playStartEffect();
        startDFS();
    } else {
        runEditorMode(currentInput);
    }
    delay(20); 
}

//--------- Main Loop ---------
void setup() {
    strip.begin();
    strip.show();
    strip.setBrightness(15); 

    for (int i = 0; i < ROWS; i++) {
        pinMode(rowPins[i], OUTPUT);
        digitalWrite(rowPins[i], HIGH); 
    }
    for (int i = 0; i < COLS; i++) {
        pinMode(colPins[i], INPUT_PULLUP);
    }
}

void loop() {
    runMatrixSystem();
}