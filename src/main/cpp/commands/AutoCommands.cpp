#include "commands/AutoCommands.h"

#include <frc2/command/Command.h>
#include <pathplanner/lib/auto/NamedCommands.h>


void AutoCommands::RegisterCommands(IntakeSubsystem* intake,
                                    ShooterSubsystem* shooter,
                                    TurretSubsystem* turret,
                                    CameraSubsystem* camera,
                                    ClimberSubsystem* climber){
    using namespace pathplanner;

    NamedCommands::registerCommand( "IntakeStart", frc2::cmd::RunOnce([intake] { intake->Run(); }, {intake}));
    // TODO: CRITICAL BUG - Line below overwrites "IntakeStart" with Stop()!
    // This should be "IntakeStop" not "IntakeStart"
    // As-is, "IntakeStart" will STOP the intake instead of starting it
    NamedCommands::registerCommand( "IntakeStart", frc2::cmd::RunOnce([intake] { intake->Stop(); }, {intake}));

    // TODO: Shooter runs once but needs time to reach target RPM
    // Consider using a command that waits for shooter to be ready before feeding
    // Or add a delay between "Shoot" and feeding game pieces
    NamedCommands::registerCommand("Shoot", frc2::cmd::RunOnce([shooter] {shooter->Shoot();},{shooter}));
    NamedCommands::registerCommand("StopShooting", frc2::cmd::RunOnce([shooter] {shooter->Stop();},{shooter}));

    // TODO: Consider adding collector commands for auto
    // NamedCommands::registerCommand("CollectorStart", ...);
    // NamedCommands::registerCommand("CollectorStop", ...);
    }
    

