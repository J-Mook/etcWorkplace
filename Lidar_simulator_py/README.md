# Lidar_simulator
<img width="1072" alt="스크린샷 2021-07-13 오후 11 05 25" src="https://user-images.githubusercontent.com/74070059/125466161-352dc433-338b-4b59-9a42-e25fcc479b32.png">

## 기본 사용자 설정 변수
- mode_select : lidar mode, pinking mode 설정
- Source_point : 센서의 위치
- Source_target : 센서가 바라보는 방향
- camera_moving_mount : Source_point를 기준으로 하여 z축(높이)을 중심으로 원형으로 시점을 변화시키며 데이터를 저장하는데 이때 시점의 갯수   
                        (1 입력 시, 설정한 Source_point에서 1회 센싱)

## Lidar Mode 사용자 설정 변수
- Resolution : 센서 fov안의 라이다포인트 갯수
- model_select : 사용할 라이다 종류 fov자동설정 (xs, s, m, l)
- noise_mode : 노이즈 사용 여부 (Ture, False)

## 1) Input & Output, 기능
Input data : mesh data(.stl)
  path : /파일위치/data

Output data_ : pointcloud data (.ply)
  path : /파일위치/data/ply

#### 기능
  - Source_point 에서 Source_target 방향으로 센서 위치
  - 사용자가 설정한 Source_point를 기준으로  하여 z축을 중심으로 회전하며 camera_moving_mount값 만큼 pointcloud를 반환한다.
  - Resolution 조정하여 라이다 포인터의 갯수를 조절한다.
  ![화면 기록 2021-07-13 오후 2 18 04](https://user-images.githubusercontent.com/74070059/125465590-1688c67c-b62b-4ad8-87e7-d58a898a9caf.gif)
  
  - 사용할 라이다의 종류를 선택하면 저장된 데이터셋에 의하여 자동으로 fov결정
  ##### 사용모델   
  * PhoXi 3D Scanner XS - https://www.photoneo.com/products/phoxi-scan-xs/
  * PhoXi 3D Scanner S - https://www.photoneo.com/products/phoxi-scan-s/
  * PhoXi 3D Scanner M - https://www.photoneo.com/products/phoxi-scan-m/
  * PhoXi 3D Scanner L - https://www.photoneo.com/products/phoxi-scan-l/

## 2) 원리 
  한점에서 구면까지 잇는 선분을 생성하고 선분과 모델링 데이터의 교점 중 첫번째 교점만을 pointcloud 데이터로 수집하였다.
  
  <img width="668" alt="스크린샷 2021-07-13 오후 5 56 46" src="https://user-images.githubusercontent.com/74070059/125461379-6eade629-af03-4062-8032-8b4010234caa.png">
  
  ~~구면의 반지름은 모델링을 구성하고있는 pointcloud 데이터 중 설정한 센서 위치로 부터 가장 먼 point의 거리를 반지름으로 설정하였다.~~
  구면의 반지름은 각 센서모델의 최대 fov거리로 설정
  pycaster Library에 내장되어있는 fromSTL 기능으로 통해 모델링파일(.stl)을 읽고, castRay 기능으로 두 점 사이의 모델링파일의 교점을 검출하였다.
  castRay는 검출순서대로 list type의 결과값이 리턴되어 첫번째 값(가장 먼저 검출)만을 사용하여 pointcloud로 변환하였다.

  pycaster docs : https://pypi.org/project/pycaster/
  
  pycaster project link : https://bitbucket.org/somada141/pycaster/src/master/
  
  #### * 노이즈
  
  실제 센서 노이즈에 가깝게 일반 랜덤함수가 아닌 가우시안 분포르 따르는 랜덤한 숫자를 각 모델별 오차범위 내의 오차를 생성하여 x, y, z요소별로 적용
  
  <img width="1405" alt="스크린샷 2021-07-15 오후 9 42 02" src="https://user-images.githubusercontent.com/74070059/125789802-c030b082-9818-40bd-bd64-769248dc357e.png">   
  (3σ = 모델별 최대 오차)

## 3) 연산시간 관련 개발
  + 구면 전체를 사용하지 않고, 센서위치에서 원점을 바라보는 방향의 반구면을 사용   
  + 구면의 반지름을 계산할때 동적계획법(Dynamic programming)을 사용하여 빠른속도로 최대 반지름을 계산   
  <pre>
  <code>
  for k in mesh:
    pnt2src_dist = math.dist(k,point)
    longest_distance = max(pnt2src_dist,longest_distance)
    </code>
</pre>
  
  + 노이즈 생성시 랜덤함수 적용방식으로 정규분포를 따르는 방식이 random.normalvariate 과 random.gauss 두 가지가 존재하는데 좀 더 연산시간이 빠른 random.gauss 를 적용하였다.   
  <p align="center">
  <img width="752" alt="스크린샷 2021-07-16 오전 9 56 53" src="https://user-images.githubusercontent.com/74070059/125875884-58144442-7e25-4678-a3b7-a974d1734481.png">   
  <p align="center">
  normalvariate random - gauss random 비교 그래프   
  </p>
  관련 Stack Overflow 토론 - https://stackoverflow.com/questions/27749133/what-is-the-difference-between-random-normalvariate-and-random-gauss-in-pyth

## 4) 개선필요점
  * 더욱 빠른 연산을 위한 개선   
  ![image](https://user-images.githubusercontent.com/74070059/125888709-21c29224-b7dc-48de-a9b4-4d99adcda9ad.png)

  위 그래프로와 같이 라이다 포인트와 비례하여 계산 소요시간이 증가하였다. (1000points ≈ 1sec)
  
  ![스크린샷 2021-07-13 오후 5 56 23](https://user-images.githubusercontent.com/74070059/125462055-1be1b6fe-d28b-4960-a503-b2ffb0912f3e.png)
  
  ~~실제 모든 라이다 포인트가 물체에 도달하지 않으므로 일정 효율을 넘지 못한다.~~(완료)   
  fov 설정 및 Resolution 설정으로 해결
  
  예) Vertical Angular Resolution = 1, Horizontal Angular Resolution = 0.2   
  <p align="center">
  <img width="500" alt="스크린샷 2021-07-16 오전 11 44 14" src="https://user-images.githubusercontent.com/74070059/125883855-eea83108-725f-4987-b4a4-e217e116a46f.png"><img width="500" alt="스크린샷 2021-07-16 오전 11 45 10" src="https://user-images.githubusercontent.com/74070059/125883794-1f69e932-ec9f-4b48-acf6-cdd1a400c7d1.png">   
   fov 적용 전  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  fov 적용 후     
  </p>
  
  * 실제 개선여부 확인결과)   
       
  <p align="center">

  ![image](https://user-images.githubusercontent.com/74070059/125897714-b40636a6-d137-4438-a04f-9d4b77cb83d0.png)
</p>


# &nbsp;   
# Pinking_simulator

<img width="500" alt="스크린샷 2021-07-16 오후 4 54 54" src="https://user-images.githubusercontent.com/74070059/125912789-af852684-3730-4973-ac4a-5e6edd05293a.png"> <img width="500" alt="스크린샷 2021-07-16 오후 4 52 53" src="https://user-images.githubusercontent.com/74070059/125912543-8b923e4d-b94f-4b2b-bdef-aa8b2164f897.png"> 

## Pinking Mode 사용자 설정 변수
(예정) gaussian_mode : 센서가 바라보는 방향을 중심으로 가우시안 필터를 거쳐 중심과 멀어질수록 필터링하는 모드   
(예정) gaussian_sigma : gaussian_mode 적용 시 사용되는 표준편차(σ)   

## 1) Input & Output, 기능
Input data : mesh data(.stl), (mesh data에서 뽑아낸) pointcloud data(.ply)   
  path : /파일위치/data

Output data_ : pointcloud data (.ply)   
  path : /파일위치/data/ply

## 2) 원리
  Source_point와 point cloud의 점을 연결하는 선분과 mesh와의 교점 갯수로 앞쪽 여부 판단   

## 3) 오차보정
  * pointcloud 생성시 발생한 것으로 보이는 오차로 인해 모든 포인트 클라우드가 매쉬의 surface위에 정확히 위치하지 않아 일부 pointcloud가 검출되지않는 문제 발생   
  <img width="1072" alt="스크린샷 2021-07-16 오후 4 14 35" src="https://user-images.githubusercontent.com/74070059/125909455-4ae96b04-21e9-444b-852e-b4832850f4f0.png">   
  
  (plate 형상이 재대로 검출되지 않았다)   

  오차를 보정하기위해 센서가 원점을 바라보는 방향의 벡터를 크기가 1인 단위벡터로 정규화하여 오차범위로 설정하고 해당 범위에 들어온 pointcloud data는 검출 된것으로 판별하였다. 오차허용 여부는 미확정이므로 사용자변수 추가 X -> 주석처리로 on/off   
  <pre>
  <code>
    mesh_error_correction = pSource / np.linalg.norm(pSource)
  </code>
  </pre>

  <img width="1072" alt="스크린샷 2021-07-16 오후 4 32 48" src="https://user-images.githubusercontent.com/74070059/125909869-91fdade8-edaf-4362-a102-b88caaafe0a8.png">   

  (오차가 보정되어 plate 형상이 재대로 검출 되었다.)   

## 4) 개선필요점

### 개발노트
  * _V0 pycaster python3 작동확인

  * _V1 평행한 두 면을 이루는 각 점들을 잇는 선분과 모델링 파일의 교점 검출

  * _V2 실제 Lidar는 면이 아닌 점에서 빛이 출발, 때문에 면에 포함된 점들과 면 밖의 한 점을 잇는 선분을 이용하여 교점검출

  * _V3 실제 Lidar는 빛은 평면에 일정한 간격이 아닌 구면에 일정한 각도로 빛이 방출되므로 구면을 이루는 점들과 센서위치의 한 점을 잇는 선분을 이용하여 교점을 검출
  
  * _V4 매번 시점을 변경하지않고, 원형으로 시점을 자동변경하는 기능을 추가

  * _V5(20210714) 센서의 방향설정 추가(기존 : 센서위치로 부터 원점방향, 개선 : 원점이 아닌 사용자 직접 입력좌표)   
    스캐너 모델 s, m ,l 제원 medels_data에 입력완료, 센싱 최대거리를 반구의 반경으로 채택, 센싱 최단거리 보다 거리 클때만 데이터 append 

  * _V6(20210715) 센서 z축 FOV세팅 제원에 따라 계산&변경, check_fov 함수생성, 이 함수로 fov여부 확인 예정
    일단 xy range는 1번폭 3번폭 잇는 선분각도로 결정 사이즈 M 이상모델부터 오차발생, 모델 xs제원 추가
    pycaster 라이브러리 속도개선 실패, 오차모드 생성

  * _V7(20210716) Resolution(points)에서 Angular_Resolution 으로 변경 -> Vertical Resolution, Horizontal Resolution으로 분할, 해당 변경사항에 맞춰 기능 테스트 완료, pinking mode 구현완료, 오차보정 완료 plate관련 이슈 해결