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

// TODO: Intake motor slows down dramatically when handling multiple game pieces
// Use velocity control instead of open-loop to maintain speed under load:
//   - Add PID config: m_intakeConfig.closedLoop.Pid(0.0001, 0, 0);
//   - Add feedforward: m_intakeConfig.closedLoop.feedForward.kV(0.000176);
//   - Create encoder: SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
//   - Create controller: SparkClosedLoopController m_intakeController = m_intakeMotor.GetClosedLoopController();
//   - Replace Set(0.6) with: m_intakeController.SetSetpoint(targetRPM, SparkLowLevel::ControlType::kVelocity);
// Velocity control will automatically increase power to maintain speed under load
// NOTE: When gear reduction is added, adjust kV accordingly:
//   kV = 1 / (5676 / gear_ratio)  -- e.g., 2:1 ratio = 1/2838 = 0.000352

/********************************************
 * kV here is the velocity feedforward constant — it's the amount of motor output (voltage or percent) needed per unit of velocity to maintain a given speed with no load.
In this context, it's the inverse of the motor's free speed adjusted for your gear ratio. The idea is:

A NEO's free speed is 5676 RPM
So to command 1 RPM, you need roughly 1/5676 of full output = ~0.000176
If you add a 2:1 gear reduction, the output shaft's free speed is now 2838 RPM, so kV becomes 1/2838 = ~0.000352

Why it matters: In a velocity PID loop, the feedforward term does most of the heavy lifting. Instead of waiting for the PID error to build up and react, kV gives the motor a baseline output that should get it close to the target speed immediately. The PID then only has to correct for small disturbances like game piece contact or friction — not do all the work from scratch.
Without a good kV, your PID has to work much harder, responds slower, and is more likely to overshoot or oscillate. With a good kV, the motor jumps to roughly the right speed immediately and PID just fine-tunes it.
So when we add gear reduction to our intake and collector, update the kV value to match or our feedforward will be off.
 * *****************************************
 */
void IntakeSubsystem::Run(){
    // TODO: Placeholder value - tune on the actual robot
    // Test with game pieces to find optimal speed:
    //   - Too slow = pieces don't feed reliably
    //   - Too fast = pieces bounce or jam
    // Move final value to Constants.h (e.g., IntakeConstants::kIntakeSpeed)
    m_intakeMotor.Set(0.6); //Placeholder value
}

void IntakeSubsystem::Reverse(){
    // TODO: Placeholder value - may need different speed than forward
    // Test ejecting jammed pieces to find optimal reverse speed
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

// TODO: Add position limits to prevent lifter over-travel
// Check encoder position in RaiseLifter()/LowerLifter() and stop motor at limits
// Or implement soft limits in the motor config (see Climber.cpp for example)
double IntakeSubsystem::GetLifterEncoderValue(){
    return m_lifterEncoder.GetPosition(); //Do not run above a certain height?
}

void IntakeSubsystem::SetLifter(double value){
    m_lifterMotor.Set(value);
}