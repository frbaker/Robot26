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
        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;
        int priorityTag = 0;

        void PutStuffOnSmartDashboard();

        void SetPriorityTag(int tag);

    private:
        
};