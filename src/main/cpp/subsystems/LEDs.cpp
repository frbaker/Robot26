#include <subsystems/LEDs.h>
#include <frc/DigitalOutput.h>

LEDSubsystem::LEDSubsystem(){
    m_redPin.EnablePWM(1);
    m_greenPin.EnablePWM(1);
    m_bluePin.EnablePWM(1);
}
void LEDSubsystem::GO(float red, float green, float blue) {
    m_redPin.UpdateDutyCycle(red);
    m_greenPin.UpdateDutyCycle(green);
    m_bluePin.UpdateDutyCycle(blue);

   
}
void LEDSubsystem::Periodic(){
    
}