uint8 OK=0
uint8 ERR_INIT=1                     #ノード起動時はこの値になる
uint8 ERR_GMPSSTA_TIMEOUT=2          #GMPSSTAが一定時間受信しないS
uint8 ERR_NO_COMMUNICATION=97        #ボードPC-磁気センサモジュール間で通信不可
uint8 ERR_RECEIVE=99                 #USBデータ受信エラー
uint8 ERR_SENSOR_VALUE_SATURATION=95 #センサ値の飽和

std_msgs/Header header
uint8 error_number
