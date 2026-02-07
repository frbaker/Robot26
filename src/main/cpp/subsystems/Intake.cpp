#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem(){
    m_lifterConfig.closedLoop.Pid(0.1, 0, 0.01);
    m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}

void IntakeSubsystem::Periodic(){

}

void IntakeSubsystem::Run(){
    m_intakeMotor.Set(0.5); //Placeholder value
}

void IntakeSubsystem::Reverse(){
    m_intakeMotor.Set(-0.5); //Placeholder value
}

void IntakeSubsystem::LowerLifter(){
    m_lifterController.SetSetpoint(-0.64, SparkLowLevel::ControlType::kPosition);
}

void IntakeSubsystem::RaiseLifter(){
    m_lifterController.SetSetpoint(0, SparkLowLevel::ControlType::kPosition);
}

double IntakeSubsystem::GetLifterEncoderValue(){
    return m_lifterEncoder.GetPosition(); //Do not run above ~0.4?
}