#include <subsystems/LEDs.h>

LEDSubsystem::LEDSubsystem(){
    m_redPin.EnablePWM(0);
    m_greenPin.EnablePWM(0);
    m_bluePin.EnablePWM(0);
}
void LEDSubsystem::TurnOnLEDs(float red, float green, float blue) {
   m_redPin.UpdateDutyCycle(red);
   m_greenPin.UpdateDutyCycle(green);
   m_bluePin.UpdateDutyCycle(blue);
}
void LEDSubsystem::Periodic(){
    
}