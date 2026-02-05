#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/controller/PIDController.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        void Periodic() override;

        void PointAtAprilTag(double yaw);

        /**
         * Turn turret to an absolute angle (degrees).
         * 0 = facing backward (shooter direction), positive = counterclockwise
         * @param angleDegrees Target angle in degrees
         */
        void TurnToAngle(double angleDegrees);

        /**
         * Get current turret angle in degrees.
         * @return Current angle (0 = backward, positive = counterclockwise)
         */
        double GetAngleDegrees();

        /**
         * Reset encoder position to zero (call when turret is at home position).
         */
        void ResetEncoder();

    private:
        SparkMax m_turretMotor{TurretConstants::kTurretCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_encoder = m_turretMotor.GetEncoder();
        frc::PIDController anglePIDController{0.05, 0, 0.005};
        frc::PIDController positionPIDController{0.02, 0, 0.001};  // For absolute positioning
};