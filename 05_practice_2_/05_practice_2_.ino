// LED 핀이 연결된 핀 번호를 7로 설정
const int LED_PIN = 7;

void setup() {
  // LED_PIN을 출력 모드로 설정
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // 처음 1초 동안 LED 켜기
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  // 다음 1초 동안 LED가 5회 깜빡이게 하기
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);  // LED 켜짐 (0.1초)
    digitalWrite(LED_PIN, LOW);
    delay(100);  // LED 꺼짐 (0.1초)
  }

  // LED 끄기
  digitalWrite(LED_PIN, LOW);

  // 무한 루프 상태로 종료
  while (true) {
    // 빈 루프로 무한 루프 유지
  }
}
