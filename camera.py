#from networktables import NetworkTables
import ntcore
#from ntcore import NetworkTableInstance
from pupil_apriltags import Detector
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#cap = cv2.VideoCapture(-1)

if not cap.isOpened():
 exit()
 raise ValueError("NOOOOO")

cWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
centerX = cWidth // 2
centerY = cHeight // 2

#NetworkTables.initialize("10.32.67.2")

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("aprilTags")

detectPub = table.getBooleanTopic("detection").publish()
tagIdPub = table.getIntegerTopic("tagId").publish()
distancePub = table.getDoubleTopic("distance").publish()
yawPub = table.getDoubleTopic("yaw").publish()
priorityTagSub = table.getIntegerTopic("priorityTag").subscribe(0)

inst.startClient4("vision")
inst.setServerTeam(3267)


at_detector = Detector(
 families="tag36h11",
 nthreads=1,
 quad_decimate=1.0,
 quad_sigma=0.0,
 refine_edges=1,
 decode_sharpening=0.25,
 debug=0
)


def put_tag(index):
 global tags, centerX, detectPub, tagIdPub, yawPub, distancePub
 blY = tags[index].corners[0][1]
 tlY = tags[index].corners[3][1]
 height = tlY - blY
 yaw = centerX - tags[index].center[0]
 print(f"{yaw}")
 detectPub.set(True)
 tagIdPub.set(tags[index].tag_id)
 yawPub.set(yaw)
 distancePub.set(187/height)

while True:

 ret,frame = cap.read()

 if not ret:
  exit()
  raise ValueError("NOOOOOO 2")

 image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

 tags = at_detector.detect(image)
# print(tags)
# table = NetworkTables.getTable("aprilTags")
 priorityTag = priorityTagSub.get()
 priorityTagExists = False
 if priorityTag != 0:
  priorityTagExists = any([tag["tag_id"] == priorityTag for tag in tags])

 if priorityTagExists:
  pti = [i for i, tag in enumerate(tags) if tag["tag_id"] == priorityTag][0]
  put_tag(pti)
 else:
  if len(tags) > 0:
   put_tag(0)
  else:
   detectPub.set(False)
