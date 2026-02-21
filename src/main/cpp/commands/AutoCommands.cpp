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
      
    NamedCommands::registerCommand("Shoot", frc2::cmd::RunOnce([shooter] {shooter->Shoot();},{shooter}));
    NamedCommands::registerCommand("StopShooting", frc2::cmd::RunOnce([shooter] {shooter->Stop();},{shooter}));
    
    }
    

