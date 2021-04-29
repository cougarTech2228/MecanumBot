/**************************************************************
   setupLEDs()
 **************************************************************/
\void setupLEDs()
{
  pinMode(ENABLE_BOT_LED_PIN, OUTPUT);
  pinMode(CHILD_MODE_LED_PIN, OUTPUT);
  pinMode(FAILSAFE_LED_PIN, OUTPUT);
  ledReset();
}

/**************************************************************
   updateLEDs()
 **************************************************************/
void updateLEDs()
{
  if (childModeEnabled)
  {
    digitalWrite(CHILD_MODE_LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(CHILD_MODE_LED_PIN, LOW);
  }

  if (botEnabled)
  {
    digitalWrite(ENABLE_BOT_LED_PIN, HIGH);
  }
  else 
  {
    digitalWrite(ENABLE_BOT_LED_PIN, LOW);
  }
  if(failSafeEnabled)
  {
    digitalWrite(FAILSAFE_LED_PIN, HIGH);
  }
  else if(botEnabled)
  {
    digitalWrite(FAILSAFE_LED_PIN, LOW);
  }
}

/**************************************************************
   ledReset()
 **************************************************************/
void ledReset()
{
  digitalWrite(FAILSAFE_LED_PIN, HIGH);
  delay(500);
  digitalWrite(FAILSAFE_LED_PIN, LOW);
  delay(500);
  digitalWrite(FAILSAFE_LED_PIN, HIGH);
  delay(500);
  digitalWrite(FAILSAFE_LED_PIN, LOW);
  delay(500);
}
