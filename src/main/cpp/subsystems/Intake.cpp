#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem() {
    m_intakeMotor.Configure(m_intakeConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}

void IntakeSubsystem::Periodic() {
    // Nothing needed here for now
}

void IntakeSubsystem::Run() {
    m_intakeMotor.Set(0.8);  // Adjust speed as needed
}

void IntakeSubsystem::Reverse() {
    m_intakeMotor.Set(-0.8);
}

void IntakeSubsystem::Stop() {
    m_intakeMotor.Set(0);
}
