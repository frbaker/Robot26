#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        void Periodic() override;

        void PointAtAprilTag(double yaw);

        void SetSpeed(double value);

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
        SparkRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
        SparkMaxConfig m_turretConfig;
        frc::PIDController anglePIDController{0.00075, 0, 0};
        frc::PIDController positionPIDController{0.02, 0, 0.001};  // For absolute positioning

        frc::DigitalInput TurretLimitSwitch{0};
};
