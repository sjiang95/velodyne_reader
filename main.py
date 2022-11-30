import pycurl 
try: 
    from io import BytesIO 
except ImportError: 
    from StringIO import StringIO as BytesIO 
try: 
    from urllib.parse import urlencode 
except ImportError: 
    from urllib import urlencode 
import urllib3
import json 
import time 
import socket
import numpy as np
import cv2
import argparse
# https://github.com/valgur/velodyne_decoder
import velodyne_decoder as vd
from velodyne_decoder_pylib import *
import time
from datetime import datetime, timedelta, timezone
from tzlocal import get_localzone

LOOKUP_COS = np.empty(36000)
LOOKUP_SIN = np.empty(36000)
for i in range(36000):
    LOOKUP_COS[i] = np.cos(np.radians(i/100.0))
    LOOKUP_SIN[i] = np.sin(np.radians(i/100.0))


class Visualizer:
    def __init__(self):
        height = 800
        width = 1400
        self._X_MIN = 2
        self._Y_MIN = 2
        self._X_MAX = (width - 3)
        self._Y_MAX = (height - 3)

        self.__map = np.ones((height, width, 3), np.uint8) * 255
        self._map = np.copy(self.__map)
        self._prev_azimuth = 0
        self._count = 0
        #self._d_azimuth_history = []

    def show_map(self):
        img = self._map
        self._map = np.copy(self.__map)
        cv2.imshow('asdf', img)
        cv2.waitKey(1)
        
    def write_map(self):
        img = self._map
        self._map = np.copy(self.__map)
        cv2.imwrite('asdf.png', img)

    def parse_data_block(self, data_block):
        flag, azimuth = struct.unpack('HH', data_block[:4])

        # this puts the "seam" in the front
        #if azimuth < self._prev_azimuth:

        # this puts the "seam" in the back (by the cable)
        if self._prev_azimuth < 18000 and azimuth >= 18000:
            self.write_map()
            #print(self._count)
            self._count = 0

        #    d_azimuth = azimuth - self._prev_azimuth + 36000
        #else:
        #    d_azimuth = azimuth - self._prev_azimuth

        #self._d_azimuth_history.append(d_azimuth)

        self._prev_azimuth = azimuth

        dist, = struct.unpack('H', data_block[7:9])
        if dist == 0:
            return

        self._count += 1
        r = 0.002 * dist

        x = r * LOOKUP_SIN[azimuth]
        y = r * LOOKUP_COS[azimuth]

        x_px = 700 + int(np.round(x * 50))
        y_px = 400 - int(np.round(y * 50))
        if x_px > self._X_MIN and x_px < self._X_MAX and y_px > self._Y_MIN and y_px < self._Y_MAX:
            self._map[y_px, x_px] = (0,0,0)
            self._map[y_px+1, x_px] = (0,0,0)
            self._map[y_px, x_px+1] = (0,0,0)
            self._map[y_px+1, x_px+1] = (0,0,0)

class ld:
    """
    Class for velodyne lidars.
    """
    def __init__(self, model:str='',
                 lidarip:str='192.168.1.201', 
                 dataPort:int=2368, 
                 rpm:int=600,
                 localhost:str='',
                 as_pcl_structs:bool=False) -> None:
        self.lidarip=lidarip
        assert dataPort in range(65535),f"Arg 'dataPort' has to be in [0,65535], but got {dataPort}."
        self.dataPort=dataPort
        self.as_pcl_structs=as_pcl_structs
        self.model=model
        self.rpm=rpm
        self.localhost=localhost
        self.config=vd.Config(model=self.model, rpm=self.rpm)
        
        # communication
        self.sensor = pycurl.Curl()
        self.Base_URL = 'http://'+self.lidarip+'/cgi/'
        print(f"Base_URL:{self.Base_URL}")
        self.buffer = BytesIO()

    def sensor_do(self, url, pf, buf):
        self.sensor.setopt(self.sensor.URL, url) 
        self.sensor.setopt(self.sensor.POSTFIELDS, pf) 
        self.sensor.setopt(self.sensor.WRITEDATA, buf) 
        self.sensor.perform() 
        rcode = self.sensor.getinfo(self.sensor.RESPONSE_CODE) 
        success = rcode in range(200, 207) 
        print(f"{url} {pf}: {rcode} ({'OK' if success else 'ERROR'})") 
        return success
    
    def launch(self):
        print(f"Launch the devide {self.model} at {self.lidarip}:")
        rc = self.sensor_do(self.Base_URL+'reset', urlencode({'data':'reset_system'}), self.buffer) 
        if rc: 
            time.sleep(5) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'rpm':args.rpm}), self.buffer) 
        if rc: 
            time.sleep(10) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'laser':'on'}), self.buffer) 
            
        http = urllib3.PoolManager()
        response = http.request('GET',self.Base_URL+"status.json")
        if response: 
            status = json.loads(response.data) 
            print (f"Sensor laser is {status['laser']['state']}, motor rpm is {status['motor']['rpm']}")
    
    def stop(self):
        print(f"Stopping the devide {self.model} at {self.lidarip}:")
        rc = self.sensor_do(self.Base_URL+'setting', urlencode({'rpm':'0'}), self.buffer) 
        if rc: 
            time.sleep(2) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'laser':'off'}), self.buffer) 
        if rc: 
            time.sleep(10)
            http = urllib3.PoolManager()
            response = http.request('GET',self.Base_URL+"status.json")
            if response: 
                status = json.loads(response.data) 
                print (f"Sensor laser is {status['laser']['state']}, motor rpm is {status['motor']['rpm']}")
        self.sensor.close()

    def read_live_data(self):
        """
        https://github.com/valgur/velodyne_decoder/issues/4#issuecomment-1248660033
        """
        decoder = vd.StreamDecoder(self.config)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((self.localhost, self.dataPort))
        while True:
            data, address = s.recvfrom(vd.PACKET_SIZE * 2)
            recv_stamp = time.time()
            yield decoder.decode(recv_stamp, data, self.as_pcl_structs)
        
def main(args):
    myld=ld(args.model,args.ip_lidar,args.dataport,args.rpm)
    try:
        myld.launch()
        for Data in myld.read_live_data():
            if Data != None:
                utc_stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%f')
                stamp, points = Data
                print(utc_stamp, points.shape)
                print(f"points:{points}")
                break
    except KeyboardInterrupt as e:
        print(e)
    finally:
        myld.stop()
        
if __name__=="__main__":
    parser=argparse.ArgumentParser(description="read data directly from velodyne lidar")
    parser.add_argument('--model', default='VLP-16', type=str,metavar="MODEL", help="Model of the velodyne lidar you use.")
    parser.add_argument('--ip-lidar', default='192.168.1.201', type=str,metavar="Lidar_IP", help="IP addr of velodyne lidar. Default:'192.168.1.201'.")
    parser.add_argument('--ip-local', default='', type=str,metavar="localhost", help="IP addr of localhost, not the velodyne lidar. Default:''(listen to all).")
    parser.add_argument('--dataport', default=2368, type=int,metavar="PORT", help="Data port to be listened for UDP packages. Default: 2368.")
    parser.add_argument('--rpm', default=600, type=int,metavar="RPM", help="RPM of the velodyne lidar to be set.")
    args=parser.parse_args()
    main(args)