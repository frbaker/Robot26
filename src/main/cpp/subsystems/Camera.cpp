#include <subsystems/Camera.h>
#include <frc/smartdashboard/SmartDashboard.h>

CameraSubsystem::CameraSubsystem(){
    inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable("aprilTags");
    tagId = table->GetIntegerTopic("tagId").Subscribe(0.0);
    detection = table->GetBooleanTopic("detection").Subscribe(false);
    yaw = table->GetDoubleTopic("yaw").Subscribe(0.0);
    distance = table->GetDoubleTopic("distance").Subscribe(0.0);
}

void CameraSubsystem::Periodic(){
    PutStuffOnSmartDashboard();
}

void CameraSubsystem::PutStuffOnSmartDashboard(){
    frc::SmartDashboard::PutNumber("tagId", tagId.Get());
    frc::SmartDashboard::PutBoolean("detection", detection.Get());
    frc::SmartDashboard::PutNumber("distance", distance.Get());
    frc::SmartDashboard::PutNumber("yaw", yaw.Get());
}

void CameraSubsystem::SetPriorityTag(int tag){
    table->PutNumber("priorityTag", tag);
    priorityTag = tag;
}

bool CameraSubsystem::GetDetection(){
    return detection.Get();
}

int CameraSubsystem::GetTagId(){
    return tagId.Get();
}

double CameraSubsystem::GetYaw(){
    return yaw.Get();
}

double CameraSubsystem::GetDistance(){ //distance in ft
    return distance.Get();
}
