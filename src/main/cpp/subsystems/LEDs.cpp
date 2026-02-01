#include <subsystems/LEDs.h>
#include <frc/DigitalOutput.h>

LEDSubsystem::LEDSubsystem(){

}
void LEDSubsystem::GO(bool red, bool green, bool blue) {
    m_redPin.Set(red);
    m_greenPin.Set(green);
    m_bluePin.Set(blue);
}
void LEDSubsystem::Periodic(){
    
}