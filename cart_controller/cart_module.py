import serial
import time

class Control_Cart():
    '''
    Arduinoによるカート制御
    このクラスへの入力は、移動量のみを入力してください。
    例）move_val = 100　→　正方向へ10.0cm移動します
    To Arduino
    input:  move_val:  ロボットの移動量[cm]
            F:         Forward 正転　進行方向
            R:         Reverse 逆転  逆方向
    output:
            Finish_flag 正常終了 0
    '''
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        
    def move_cart(self, move_val):
        # 進行方向へ
        if move_val > 0:
            #formatは第2引数の値まで0埋め,move_val=10だったら0010　
            send_data = format(int(round(move_val, 1)*10), '04') 
            send_data = 'F' + send_data
        # 逆方向へ
        elif move_val < 0:
            send_data = format(int(round(move_val, 1)*10), '05')
            send_data = 'R' + send_data[1:]
        # 停止
        else:
            send_data = 'F0000'

        #なぜタイムスリープ入れないと動かないのか
        time.sleep(1)
        if send_data[-1] != ',':
            send_data = send_data + ','
        time.sleep(1)
        self.ser.write(send_data.encode(encoding='utf-8'))
        time.sleep(1)

        try:
            self.ser.timeout = 5 #(s)
            line = self.ser.readline()
            receive_data = line.strip().decode('UTF-8')    
            if receive_data !="":
                return receive_data
            else :
                return None

        except serial.serialutil.SerialTimeoutException:
            # タイムアウトエラーが発生した場合の処理
            print("タイムアウトエラー: データの受信がタイムアウトしました")
            return None


