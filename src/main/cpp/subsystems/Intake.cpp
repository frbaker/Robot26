#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem(){
    m_lifterConfig.closedLoop.Pid(0.1, 0, 0.01);
    m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}

void IntakeSubsystem::Periodic(){

}

void IntakeSubsystem::LowerLifter(){
    // No idea where the setpoint will be, setting 0 for now
    m_lifterController.SetSetpoint(0, SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::RaiseLifter(){
    // No idea where the setpoint will be, setting 0 for now
    m_lifterController.SetSetpoint(0, SparkLowLevel::ControlType::kPosition);
}