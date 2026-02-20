#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

ClimberSubsystem::ClimberSubsystem(){

}

void ClimberSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("climber encoder", m_climberEncoder.GetPosition());
}

void ClimberSubsystem::Run(){
    m_climberMotor.Set(0.6); 
}
//Hypothetical values, ofc needs tuning
void ClimberSubsystem::Reverse(){
    m_climberMotor.Set(-0.6);
}

void ClimberSubsystem::Stop(){
    m_climberMotor.Set(0);
}