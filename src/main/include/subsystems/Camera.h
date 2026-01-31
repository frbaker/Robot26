#pragma once
#include <frc2/command/SubsystemBase.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>


class CameraSubsystem : public frc2::SubsystemBase{   
    public:
        CameraSubsystem();
        void Periodic() override;
        nt::BooleanSubscriber detection;
        nt::IntegerSubscriber tagId;
        nt::DoubleSubscriber distance;
        nt::DoubleSubscriber yaw;
        //nt::DoubleSubscriber centerX;
        //nt::DoubleSubscriber centerY;
        nt::NetworkTableInstance inst;
        //nt::DoubleSubscriber height;
        std::shared_ptr<nt::NetworkTable> table;

        void PutStuffOnSmartDashboard();

    private:
        
};