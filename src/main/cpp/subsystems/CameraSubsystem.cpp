#include <subsystems/CameraSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

CameraSubsystem::CameraSubsystem(){
    inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable("aprilTags");
    centerX = table->GetDoubleTopic("centerX").Subscribe(0.0);
    centerY = table->GetDoubleTopic("centerY").Subscribe(0.0);
    tagId = table->GetIntegerTopic("tagId").Subscribe(0.0);
    detection = table->GetBooleanTopic("detection").Subscribe(false);
    height = table->GetDoubleTopic("height").Subscribe(0.0);
}

void CameraSubsystem::Periodic(){
    
}

void CameraSubsystem::PutStuffOnSmartDashboard(){
    frc::SmartDashboard::PutNumber("centerX", centerX.Get());
    frc::SmartDashboard::PutNumber("centerY", centerY.Get());
    frc::SmartDashboard::PutNumber("tagId", tagId.Get());
    frc::SmartDashboard::PutBoolean("detection", detection.Get());
    frc::SmartDashboard::PutNumber("height", height.Get());
}