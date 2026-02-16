#include "subsystems/Intake.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem(){
    m_lifterConfig.SmartCurrentLimit(30); //Lifter problems = raise
    m_lifterConfig.OpenLoopRampRate(0.1); //Change prolly
    m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);

    m_intakeConfig.SmartCurrentLimit(40); //Intake issues? Didn't ask
    m_intakeConfig.OpenLoopRampRate(0.1);

    m_intakeMotor.Configure(m_intakeConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}
//A
void IntakeSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("Lifter Value", m_lifterEncoder.GetPosition());
}

void IntakeSubsystem::Run(){
    m_intakeMotor.Set(0.6); //Placeholder value
}

void IntakeSubsystem::Reverse(){
    m_intakeMotor.Set(-0.6); //Placeholder value
}

void IntakeSubsystem::Stop(){
    m_intakeMotor.Set(0.0);
    m_lifterMotor.Set(0.0);
}

void IntakeSubsystem::LowerLifter(){
    m_lifterMotor.Set(-0.1);
}

void IntakeSubsystem::RaiseLifter(){
    m_lifterMotor.Set(0.2);
}

double IntakeSubsystem::GetLifterEncoderValue(){
    return m_lifterEncoder.GetPosition(); //Do not run above a certain height?
}

void IntakeSubsystem::SetLifter(double value){
    m_lifterMotor.Set(value);
}