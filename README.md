# Preliminary Implementation of Grasping Operation by a Collaborative Robot Arm: Using a Ball as Example

* Title : Preliminary Implementation of Grasping Operation by a Collaborative Robot Arm: Using a Ball as Example
* Author : Wen-Chang Cheng,Chien-Hung Lin, Cheng-Yi Shi,Hung-Chou Hsiao,Chun-Lung Chang
* Conference : (TAAI 2021) Taiwanese Association for Artificial Intelligence

## Abstract
長照體系為目前重要的產業之一。隨著高齡化社會的逐步邁進，可觀察出全世界年長者人口逐年增加的跡象。因此對於未來的長照體系將會被受考驗，尤其於老人及病患照顧上更為明顯。而近年來隨著機器手臂快速蓬勃發展下，智慧化社會已經開始逐步成形，機器手臂不只應用於工業製造上，於照護部分也開始投入應用，其目的為減輕照護壓力與提升患者自主能力，同時提高患者的活動意願，為將來人力照護不足的窘境提供一個解決方案。本專題以此為主題，完成一套協作機器手臂抓取物件流程實作，硬體部分整合深度相機、協作機器手臂、麥克風以及人工智慧邊緣計算設備，軟體部分整合電腦視覺技術、深度學習以及機器人作業系統。完成系統實作後，經實驗證實，實作的系統能正確完成抓取。

## Requirements
* Robot Operation System (ROS_1)
* Python 2.7

## 於(程式)資歷夾中具以下兩種資料夾 ：
- launch
- ros_robotarm_objdetect
  
## 於(launch)資歷夾中具以下檔案 ：
- Arm_camera_start.launch ： 用來啟動各個軟、硬體的程式，其中硬體包括(協作型機器手臂、Intel realsense D435i)，軟體包括(Darknet_ROS)。

## 於(ros_robotarm_objdetect)資歷夾中具以下檔案 ：
* 程式 ：
- Multi-execute.sh				：啟動下列程式碼，此程式中具備兩種模式，(1)test為確認相機視角是否正確，(2)voice為本專題的主體使用模式，因此使用者須自行編輯程式碼來添加、去除註解字元(#)。
- pose_action_client_finger_cartesian.py	：驅動協作型機器手臂程式。
- voice_detect.py				：偵測語音程式。
- voice_object_detect.py			：座標轉換程式(語音辨識版本)。
- camera_tf_broadcaster.py			：定義相對座標程式。
- object_detect.py				：座標轉換程式(點擊螢幕版本)。

* 音檔 ：
- ball.mp3	：尋問使用者是否要進行夾取物件(球)。
- bottle.mp3	：尋問使用者是否要進行夾取物件(寶特瓶)。
- cup.mp3	：尋問使用者是否要進行夾取物件(杯子)。
- no.mp3	：回應取消物件夾取的語音。
- yes.mp3	：回應協作型機器手臂將要開始夾取的語音。
- ok.mp3	：回應[好的]。
  
## 操作流程：
1. 開啟兩個terminal，分別進入launch及ros_robotarm_objdetect資料夾，輸入以下指令：roslaunch Arm_camera_start.launch 及 source Multi-execute.sh。

2. 語音輸入的語音具備七項功能：球、杯子、瓶子、放開、抓住、放回去、離開。

3. 當使用者說出要夾取的物件名稱後，系統會再次詢問此物件，需再次回應語音：是、不是。
