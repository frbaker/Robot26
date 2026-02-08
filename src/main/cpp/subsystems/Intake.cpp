#include "subsystems/Intake.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem(){
    m_lifterConfig.closedLoop.Pid(0.1, 0, 0.01);
    m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}

void IntakeSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("Lifter Value", m_lifterEncoder.GetPosition());
}

void IntakeSubsystem::Run(){
    m_intakeMotor.Set(0.4); //Placeholder value
}

void IntakeSubsystem::Reverse(){
    m_intakeMotor.Set(-0.4); //Placeholder value
}

void IntakeSubsystem::Stop(){
    m_intakeMotor.Set(0.0);
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

void IntakeSubsystem::SetLifter(double value){
    m_lifterMotor.Set(value);
}