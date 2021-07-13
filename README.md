# etcWorkplace

## 사용자 설정 변수
- seperate_spr : 반구면을 n * n 개로 분할한다.
- Source_point : 센서의 위치
- camera_moving_mount : Source_point를 기준으로 하여 z축(높이)을 중심으로 원형으로 시점을 변화시키며 데이터를 저장하는데 이때 시점의 갯수
                        (1 입력 시, 설정한 Source_point에서 1회 센싱)

## 1) Input & Output, 기능
//Input data : mesh data(.stl)
  path : /파일위치/data

Output data_ : pointcloud data (.ply)
  path : /파일위치/data/ply

기능 - 사용자가 설정한 Source_point를 기준으로 하여 z축을 중심으로 회전하며 camera_moving_mount값 만큼 pointcloud를 반환한다.
    - seperate_spr를 조정하여 라이다 포인터의 밀도를 조절한다.

## 2) 원리 
  한점에서 구면까지 잇는 선분을 생성하고 선분과 모델링 데이터의 교점 중 첫번째 교점만을 pointcloud 데이터로 수집하였다.
  
  <img width="668" alt="스크린샷 2021-07-13 오후 5 56 46" src="https://user-images.githubusercontent.com/74070059/125461379-6eade629-af03-4062-8032-8b4010234caa.png">
  
  구면의 반지름은 모델링을 구성하고있는 pointcloud 데이터 중 설정한 센서 위치로 부터 가장 먼 point의 거리를 반지름으로 설정하였다.
  pycaster Library에 내장되어있는 fromSTL 기능으로 통해 모델링파일(.stl)을 읽고, castRay 기능으로 두 점 사이의 모델링파일의 교점을 검출하였다.
  castRay는 검출순서대로 list type의 결과값이 리턴되어 첫번째 값(가장 먼저 검출)만을 사용하여 pointcloud로 변환하였다.

  pycaster docs : https://pypi.org/project/pycaster/
  
  pycaster project link : https://bitbucket.org/somada141/pycaster/src/master/

## 3)연산시간 관련 개발
  구면 전체를 사용하지 않고, 센서위치에서 원점을 바라보는 방향의 반구면을 사용
  구면의 반지름을 계산할때 동적계획법(Dynamic programming)을 사용하여 빠른속도로 최대 반지름을 계산

## 4) 개선필요점
  * 더욱 빠른 연산을 위한 개선
  <img width="697" alt="스크린샷 2021-07-13 오후 10 38 36" src="https://user-images.githubusercontent.com/74070059/125461967-3b3200e5-f9d9-4279-8108-ec66c899e2cf.png">
  
  위 그래프로와 같이 라이다 포인트 검출 갯수에 비례하여 계산 소요시간이 증가하였다. 
  
  ![스크린샷 2021-07-13 오후 5 56 23](https://user-images.githubusercontent.com/74070059/125462055-1be1b6fe-d28b-4960-a503-b2ffb0912f3e.png)
  
  반구면 전체가 물체를 통과하지 않으므로 일정 효율을 넘지 못한다.
  
### 개발노트(개발순서)
  * _V0 pycaster python3 작동확인

  * _V1 평행한 두 면을 이루는 각 점들을 잇는 선분과 모델링 파일의 교점 검출

  * _V2 실제 Lidar는 면이 아닌 점에서 빛이 출발, 때문에 면에 포함된 점들과 면 밖의 한 점을 잇는 선분을 이용하여 교점검출

  * _V3 실제 Lidar는 빛은 평면에 일정한 간격이 아닌 구면에 일정한 각도로 빛이 방출되므로 구면을 이루는 점들과 센서위치의 한 점을 잇는 선분을 이용하여 교점을 검출
  
  * _V4 매번 시점을 변경하지않고, 원형으로 시점을 자동변경하는 기능을 추가
