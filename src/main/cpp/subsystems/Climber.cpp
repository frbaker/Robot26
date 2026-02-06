#include "subsystems/Climber.h"

ClimberSubsystem::ClimberSubsystem(){

}

void ClimberSubsystem::Periodic(){

}

void ClimberSubsystem::Run(){
    m_climberMotor.Set(0.4); 
}
//Hypothetical values, ofc needs tuning
void ClimberSubsystem::Reverse(){
    m_climberMotor.Set(-0.4);
}