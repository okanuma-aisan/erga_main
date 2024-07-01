std_msgs/Header header
float32 side_diff   #GMPSセンサ出力である磁気マーカからの横変位[m]
uint8 pole         #磁極 N=1 S=2
uint8 mm_kind    #not used マーカ種別
int32 mm_id       #対応付けを行った結果の磁気マーカ番号
int32 tag_id_table  #対応付けを行った結果のRFIDタグ番号
int32 tag_id_detected #not used 実際に検知したRFIDタグ番号
float32 x_marker    #磁気マーカ埋設座標X [m]
float32 y_marker    #磁気マーカ埋設座標Y [m]
