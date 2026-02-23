#include <subsystems/LEDs.h>
#include <frc/DriverStation.h>


LEDSubsystem::LEDSubsystem(){
    m_redPin.SetPWMRate(150);
    m_greenPin.SetPWMRate(150);
    m_bluePin.SetPWMRate(150);

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
    if (frc::DriverStation::IsDisabled()) {
        // TODO: Using Set(0) but LEDs are configured for PWM
        // Should use UpdateDutyCycle(0) for consistency with TurnOnLEDs()
        m_redPin.Set(0);
        m_greenPin.Set(0);
        m_bluePin.Set(0);
    }
}