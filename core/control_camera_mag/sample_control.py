import serial
import time
import os

# 倍率會乘100 ~0.7~4.5  70-450 
def dec_to_hex(dec):
    d = hex(dec)
    if len(d) > 4:
        test = d.replace("x", "")
    else:
        test = d.replace("x", "0")
    command = bytes.fromhex(f'AB11 {test} CD')
    return command

home_command = bytes.fromhex('ABE5 0000 CD')

if __name__ == "__main__":
    ser = serial.Serial('COM3', 9600, bytesize=8, stopbits=1, timeout=1)
    ser.write(home_command)
    time.sleep(3)

    last_modified = None

    while True:
        try:
            time.sleep(1)
            # 檢查檔案是否存在
            if not os.path.exists('trigger_file.txt'):
                continue

            # 檢查檔案是否被更新
            current_modified = os.path.getmtime('trigger_file.txt')
            if last_modified == current_modified:
                time.sleep(1)  # 等待一秒後再次檢查
                continue

            last_modified = current_modified

            with open('trigger_file.txt', 'r') as file:
                data = file.read()
                try:
                    rate = int(data)
                except ValueError:
                    continue  # 如果檔案中不是數字，則繼續等待

                if rate == -1:
                    break  # 如果檔案中是 -1，則停止迴圈

                if 70 <= rate <= 450:  # 確保倍率在合理範圍內
                    command = dec_to_hex(rate)
                    ser.write(command)
                    time.sleep(3)

        except Exception as e:
            print("發生錯誤:", e)
            break
        time.sleep(1)

    # 回到初始狀態
    ser.write(home_command)
    time.sleep(3)
    ser.close()
