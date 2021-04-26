import sys
import time
import traceback
from client import RobotClient


osXPath = '/dev/tty.usbmodem14122201'
linuxPath = '/dev/ttyACM0'
path = linuxPath

if __name__ == "__main__":
    client = RobotClient(path)
    client.init()
    res = client.stop()
    print(res)
    time.sleep(2)

    while True:
        try:
            time.sleep(2)
            res = client.status()
            print(res)
            res = client.raw(0, -100)
            print(res)
            time.sleep(0.5)
            res = client.stop()
            print(res)
        except KeyboardInterrupt:
            client.stop()
            time.sleep(5)
            sys.exit(0)
        except cobs.DecodeError as e:
            print(e)
        except:
            print('Unexpected error:', sys.exc_info()[0])
            traceback.print_exc()
            sys.exit(-1)
