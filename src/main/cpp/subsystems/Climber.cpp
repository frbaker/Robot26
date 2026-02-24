#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

ClimberSubsystem::ClimberSubsystem(){
    // TODO: Consider adding current limiting for the climber motor
    // Climbers can draw high current when stalling against hard stops
    // Example: m_climberConfig.SmartCurrentLimit(40);
    // unnecessary based on testing — it won't stall, and if it does,
    // then can just stop holding the button

    // TODO: Consider adding soft limits to prevent over-travel
    // Currently no protection if climber goes beyond expected range
    // To implement soft limits:
    //   1. Configure the limits on the config object:
    //      m_climberConfig.softLimit.ForwardSoftLimit(65);
    //      m_climberConfig.softLimit.ReverseSoftLimit(-1);
    //      m_climberConfig.softLimit.ForwardSoftLimitEnabled(true);
    //      m_climberConfig.softLimit.ReverseSoftLimitEnabled(true);
    //   2. These get applied when Configure() is called below
    // also unnecessary, pid loop will stop from going over these and 
    // will be alot more accurate than a soft limit

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

frc2::CommandPtr ClimberSubsystem::UpAuto(){
    return RunOnce([this] {
        m_climberController.SetSetpoint(60, SparkLowLevel::ControlType::kPosition);
    });
    
}

frc2::CommandPtr ClimberSubsystem::DownAuto(){
    return RunOnce([this]{
        m_climberController.SetSetpoint(-0.01, SparkLowLevel::ControlType::kPosition);
    });
    
}