#include <subsystems/Camera.h>
#include <frc/smartdashboard/SmartDashboard.h>

CameraSubsystem::CameraSubsystem(){
    inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable("aprilTags");

    tagId = table->GetIntegerTopic("tagId").Subscribe(0);
    detection = table->GetBooleanTopic("detection").Subscribe(false);
    yaw = table->GetDoubleTopic("yaw").Subscribe(0.0);
    distance = table->GetDoubleTopic("distance").Subscribe(0.0);

    tagId2 = table->GetIntegerTopic("tagId2").Subscribe(0);
    detection2 = table->GetBooleanTopic("detection2").Subscribe(false);
    yaw2 = table->GetDoubleTopic("yaw2").Subscribe(0.0);
    distance2 = table->GetDoubleTopic("distance2").Subscribe(0.0);
}

void CameraSubsystem::Periodic(){
    PutStuffOnSmartDashboard();
}

void CameraSubsystem::PutStuffOnSmartDashboard(){
    frc::SmartDashboard::PutNumber("tagId", tagId.Get());
    frc::SmartDashboard::PutBoolean("detection", detection.Get());
    frc::SmartDashboard::PutNumber("distance", distance.Get());
    frc::SmartDashboard::PutNumber("yaw", yaw.Get());

    frc::SmartDashboard::PutNumber("tagId2", tagId2.Get());
    frc::SmartDashboard::PutBoolean("detection2", detection2.Get());
    frc::SmartDashboard::PutNumber("distance2", distance2.Get());
    frc::SmartDashboard::PutNumber("yaw2", yaw2.Get());
}

void CameraSubsystem::SetPriorityTag(int tag){
    table->PutNumber("priorityTag", tag);
    priorityTag = tag;
}
void CameraSubsystem::SetPriorityTag2(int tag){
    table->PutNumber("priorityTag2", tag);
    priorityTag2 = tag;
}

bool CameraSubsystem::GetDetection(){
    int64_t timestamp = detection.GetAtomic().time;
    int64_t now = nt::Now();
    //If >500ms (timestamp is in microseconds, so 500,000)
    //the data is stale, return no detection
    if((now - timestamp) > 500000){ 
        return false;
    }
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

bool CameraSubsystem::GetDetection2(){
    int64_t timestamp = detection2.GetAtomic().time;
    int64_t now = nt::Now();
    if((now - timestamp) > 500000){
        return false;
    }
    return detection2.Get();
}

int CameraSubsystem::GetTagId2(){
    return tagId2.Get();
}

double CameraSubsystem::GetYaw2(){
    return yaw2.Get();
}

double CameraSubsystem::GetDistance2(){ //distance in ft
    return distance2.Get();
}
