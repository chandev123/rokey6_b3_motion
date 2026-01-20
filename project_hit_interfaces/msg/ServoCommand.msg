#ServoCommand.msg
#로봇의 미세 위치 조정을 위한 메시지

#6축 변위 - 단위:mm,degree
float64[6] delta_pose

#제어 모드(예: 1=힘제어, 2=나선탐색)
int8 mode