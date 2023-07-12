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
        
    def Go(self, move_val):
        # 進行方向へ
        if move_val > 0:
            send_data = format(int(round(move_val, 1)*10), '04')
            send_data = 'F' + send_data
        # 逆方向へ
        elif move_val < 0:
            send_data = format(int(round(move_val, 1)*10), '05')
            send_data = 'R' + send_data[1:]
        # 停止
        else:
            send_data = 'F0000'
        time.sleep(1)
        send_data = self.check_data(send_data)
        time.sleep(1)
        self.ser.write(send_data.encode(encoding='utf-8'))
        time.sleep(1)
        receive_data = self.serial_data() #何か文字を受け取ったら終了
        # print(receive_data)
        return 0
    def check_data(self, send_data):
        if send_data[-1] != ',':
            send_data = send_data + ','
        return send_data
    def serial_data(self):
        line = self.ser.readline()
        line_disp = line.strip().decode('UTF-8')
        return line_disp


