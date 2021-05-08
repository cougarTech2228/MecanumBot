/**************************************************************
   setupButtonInterrupts()
 **************************************************************/
void setupButtonInterrupts(){
  pinMode(ENABLE_BOT_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENABLE_BOT_BUTTON_PIN), enableBotButtonPressed, FALLING);

  pinMode(CHILD_MODE_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CHILD_MODE_BUTTON_PIN), childModeButtonPressed, FALLING);
}

/**************************************************************
   enableBotButtonPressed()
 **************************************************************/
void enableBotButtonPressed() {

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // If interrupts come faster than BUTTON_DEBOUNCE_TIME_MS, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > BUTTON_DEBOUNCE_TIME_MS) {

    // Toggle the botEnabled flag
    botEnabled = !botEnabled;
  }

  last_interrupt_time = interrupt_time;
}

/**************************************************************
   childModeButtonPressed()
 **************************************************************/
void childModeButtonPressed() {
  Serial.println("Child mode pressed");
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // If interrupts come faster than BUTTON_DEBOUNCE_TIME_MS, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > BUTTON_DEBOUNCE_TIME_MS) {

    // Toggle the childModeEnabled flag
    childModeEnabled = !childModeEnabled;
  }

  last_interrupt_time = interrupt_time;
}
