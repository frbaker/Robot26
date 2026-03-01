#include "subsystems/Intake.h"
#include "frc/smartdashboard/SmartDashboard.h"


IntakeSubsystem::IntakeSubsystem(){
    m_lifterConfig.SmartCurrentLimit(30); //Lifter problems = raise
    m_lifterConfig.OpenLoopRampRate(0.1); //Change prolly
    m_lifterConfig.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);

    m_intakeConfig.SmartCurrentLimit(40);
    m_intakeConfig.closedLoop.Pid(0.0001, 0, 0);
    // To get back to the 60% we had - use the calcs below
    // kV = 1 / (5676 / gear_ratio) — 5676 is the NEO free speed in RPM
    // No reduction: 0.000176
    // 2:1: 0.000352
    // 3:1: 0.000528
    // 4:1: 0.000704
    // 5:1: 0.000880
    m_intakeConfig.closedLoop.feedForward.kV(0.000528);

    m_intakeMotor.Configure(m_intakeConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}
//A
void IntakeSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("Lifter Value", m_lifterEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Intake RPM", m_intakeEncoder.GetVelocity());
}

void IntakeSubsystem::Run(){
    m_intakeController.SetSetpoint(3400, SparkLowLevel::ControlType::kVelocity);
}

void IntakeSubsystem::Reverse(){
    m_intakeController.SetSetpoint(-3400, SparkLowLevel::ControlType::kVelocity);
}

void IntakeSubsystem::Stop(){
    m_intakeMotor.Set(0.0);
    m_lifterMotor.Set(0.0);
}

void IntakeSubsystem::LowerLifter(){
    if (m_lifterInBrakeMode) {
        m_lifterConfig.SetIdleMode(SparkMaxConfig::IdleMode::kCoast);
        m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
        m_lifterInBrakeMode = false;
    }
    m_lifterMotor.Set(-0.1);
}

void IntakeSubsystem::RaiseLifter(){
    if (!m_lifterInBrakeMode) {
        m_lifterConfig.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
        m_lifterMotor.Configure(m_lifterConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
        m_lifterInBrakeMode = true;
    }
    m_lifterMotor.Set(0.2);
}

double IntakeSubsystem::GetLifterEncoderValue(){
    return m_lifterEncoder.GetPosition(); //Do not run above a certain height?
}

void IntakeSubsystem::SetLifter(double value){
    m_lifterMotor.Set(value);
}