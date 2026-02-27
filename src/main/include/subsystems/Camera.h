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
        
        int priorityTag = 0;
        int priorityTag2 = 0;

        void PutStuffOnSmartDashboard();

        void SetPriorityTag(int tag);
        void SetPriorityTag2(int tag);

        bool GetDetection();
        int GetTagId();
        double GetDistance();
        double GetYaw();
        bool GetDetection2();
        int GetTagId2();
        double GetDistance2();
        double GetYaw2();
        

    private:
        nt::BooleanSubscriber detection;
        nt::IntegerSubscriber tagId;
        nt::DoubleSubscriber distance;
        nt::DoubleSubscriber yaw;
        nt::BooleanSubscriber detection2;
        nt::IntegerSubscriber tagId2;
        nt::DoubleSubscriber distance2;
        nt::DoubleSubscriber yaw2;
        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;
};