#include "subsystems/Climber.h"

ClimberSubsystem::ClimberSubsystem(){

}

void ClimberSubsystem::Periodic(){

}

void ClimberSubsystem::Run(){
    m_climberMotor.Set(0.4);
}

void ClimberSubsystem::Reverse(){
    m_climberMotor.Set(-0.4);
}

void ClimberSubsystem::Stop(){
    m_climberMotor.Set(0);
}

void ClimberSubsystem::Climb(){
    Run();  // Alias for Run()
}