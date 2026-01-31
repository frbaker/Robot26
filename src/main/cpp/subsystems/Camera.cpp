#include <subsystems/Camera.h>
#include <frc/smartdashboard/SmartDashboard.h>

CameraSubsystem::CameraSubsystem(){
    inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable("aprilTags");
    tagId = table->GetIntegerTopic("tagId").Subscribe(0.0);
    detection = table->GetBooleanTopic("detection").Subscribe(false);
}

void CameraSubsystem::Periodic(){
    
}

void CameraSubsystem::PutStuffOnSmartDashboard(){
    frc::SmartDashboard::PutNumber("tagId", tagId.Get());
    frc::SmartDashboard::PutBoolean("detection", detection.Get());
    frc::SmartDashboard::PutNumber("distance", distance.Get());
    frc::SmartDashboard::PutNumber("yaw", yaw.Get());
}