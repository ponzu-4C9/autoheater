class TimeProportionalPWM {
public:
  TimeProportionalPWM(uint8_t pin, unsigned long periodMs = 1000, bool activeHigh = true)
    : pin(pin), periodMs(periodMs), activeHigh(activeHigh), duty(0.0),
      lastState(false) {}

  void begin() {
    pinMode(pin, OUTPUT);
    // 初期は OFF
    digitalWrite(pin, activeHigh ? LOW : HIGH);
    lastState = false;
  }

  // duty: 0.0 .. 1.0
  void setDuty(double d) {
    if (isnan(d)) return;
    if (d < 0.0) d = 0.0;
    if (d > 1.0) d = 1.0;
    duty = d;
    // optional: update immediately on change if you want
    // update(millis());
  }

  double getDuty() const { return duty; }

  void setPeriod(unsigned long ms) {
    if (ms == 0) ms = 1;
    periodMs = ms;
  }

  unsigned long getPeriod() const { return periodMs; }

  // 呼び出し側は loop() の中で頻繁に update() を呼ぶ
  void update(unsigned long now = millis()) {
    // 早期リターン: duty が 0 または 1 のときは単純に OFF/ON
    if (duty <= 0.0) {
      applyState(false);
      return;
    }
    if (duty >= 1.0) {
      applyState(true);
      return;
    }

    // フェーズ計算（now % period）
    unsigned long phase = now % periodMs;
    unsigned long onTime = (unsigned long)(duty * periodMs);

    bool shouldBeOn = (phase < onTime);
    applyState(shouldBeOn);
  }

private:
  uint8_t pin;
  unsigned long periodMs;
  bool activeHigh;
  double duty;
  bool lastState;

  void applyState(bool on) {
    if (on == lastState) return; // 状態変化がなければ書き込みしない
    lastState = on;
    if (activeHigh) {
      digitalWrite(pin, on ? HIGH : LOW);
    } else {
      digitalWrite(pin, on ? LOW : HIGH);
    }
  }
};