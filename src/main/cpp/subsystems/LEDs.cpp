#include <subsystems/LEDs.h>
#include <frc/DigitalOutput.h>

LEDSubsystem::LEDSubsystem(){
    /*m_redPin.EnablePWM(0);
    m_greenPin.EnablePWM(0);
    m_bluePin.EnablePWM(0);*/
}
void LEDSubsystem::GO(float red, float green, float blue) {
    /*m_redPin.UpdateDutyCycle(red);
    m_greenPin.UpdateDutyCycle(green);
    m_bluePin.UpdateDutyCycle(blue);
    */
   m_redPin.SetPosition(red);
   m_greenPin.SetPosition(green);
   m_bluePin.SetPosition(blue);
}
void LEDSubsystem::Periodic(){
    
}