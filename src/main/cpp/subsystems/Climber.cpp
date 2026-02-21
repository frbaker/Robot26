#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

ClimberSubsystem::ClimberSubsystem(){
    m_climberConfig.closedLoop.Pid(0.05, 0, 0);

    m_climberMotor.Configure(m_climberConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}

void ClimberSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("climber encoder", m_climberEncoder.GetPosition());
}

void ClimberSubsystem::Run(){
    //m_climberMotor.Set(0.6);
    m_climberController.SetSetpoint(60, SparkLowLevel::ControlType::kPosition);
}
//Hypothetical values, ofc needs tuning
void ClimberSubsystem::Reverse(){
    //m_climberMotor.Set(-0.6);
    m_climberController.SetSetpoint(0, SparkLowLevel::ControlType::kPosition);
}

void ClimberSubsystem::ReverseBypass(){
    m_climberMotor.Set(-0.2);
}

void ClimberSubsystem::Stop(){
    m_climberMotor.Set(0);
}

void ClimberSubsystem::Climb(){
    Run();  // Alias for Run()
}

bool ClimberSubsystem::IsLimitSwitchTriggered(){
    // Note: Limit switches are typically normally-closed (NC), so Get() returns
    // false when pressed. Invert if your switch is normally-open (NO).
    return !m_limitSwitch.Get();
}
