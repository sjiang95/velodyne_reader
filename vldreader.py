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
import argparse
from datetime import datetime, timezone
import os
import logging
import queue
import threading
from tqdm import tqdm
from scapy.all import Ether, IP, UDP
from scapy.utils import PcapWriter
from prettytable import PrettyTable

# velodyne
# https://github.com/valgur/velodyne_decoder
import velodyne_decoder as vd
from velodyne_decoder_pylib import *
supportModels=vd.Config.SUPPORTED_MODELS

class ld:
    """
    Class for velodyne lidars.
    
    Args:
        model: string
            Device model. Must be one of the outputs of this command `python -c 'import velodyne_decoder as vd;print(vd.Config.SUPPORTED_MODELS)'`.
        lidarip: string
            The lidar's ip address. Default: 192.168.1.201.
        rpm: int
            Desired RPM of the lidar. Default: 600.
        returnMode: string
            Desired return mode of the lidar. Default: dual.
        localhost: string
            Interface to listen for UDP data packets sent from lidar. Default: ''(all network interfaces).
        as_pcl_structs: bool
            To return arrays of structs instead of the default contiguous arrays, set `as_pcl_structs=True`.
        filePref: string
            Filename prefix of the output `.pcap` and `.log` files. Default: `<model>_<rpm><returnMode>_<UTCtimestamp>`.
    """
    def __init__(self, model:str='',
                 lidarip:str='192.168.1.201', 
                 dataPort:int=2368, 
                 rpm:int=600,
                 returnMode:str='dual',
                 localhost:str='',
                 as_pcl_structs:bool=False,
                 outputRoot:str='',
                 filePref:str=None,
                 logger:logging.Logger=None) -> None:
        assert model in supportModels,f"Unsupported model {model}, please choose from {supportModels}."
        # utils
        self.progressBar=None
        
        # velodyne lidar
        self.lidarip=lidarip
        assert dataPort in range(65535),f"Arg 'dataPort' has to be in [0,65535], but got {dataPort}."
        self.dataPort=dataPort
        self.as_pcl_structs=as_pcl_structs
        self.model=model
        self.rpm=rpm
        assert returnMode in ['strongest','last','dual'], f"returnMode must be one of ['strongest','last','dual'], but got {returnMode}"
        self.returnMode=returnMode.capitalize()
        self.config=vd.Config(model=self.model, rpm=self.rpm)
        self.decoder = vd.StreamDecoder(self.config)
        self.utc_time=datetime.now(timezone.utc)
        self.utcDate=self.utc_time.strftime('%Y%m%d')
        self.utcHMS=self.utc_time.strftime('%H%M%S')
        self.outputRoot=outputRoot
        self.filePref=filePref if filePref is not None else self.model+'_'+f"{self.rpm:04d}"+'rpm'+self.returnMode.capitalize()+'_'+self.utc_time.strftime('%Y%m%dT%H%M%S.%f')
        
        # logger
        self.logger=self.buildlogger() if logger is None else logger
        
        # communication
        self.localhost=localhost
        self.sensor = pycurl.Curl()
        self.Base_URL = 'http://'+self.lidarip+'/cgi/'
        self.logger.info(f"Base_URL:{self.Base_URL}")
        self.buffer = BytesIO()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.localhost, self.dataPort))
        
        # data container
        self.qStream=queue.Queue()
        
        # workflow flag
        self.stream2pcapFlag=False # use this flag to break loop in func stream2pcap()
        
        self.deviceSettings()
        
    def buildlogger(self):
        logger = logging.getLogger()
        logger.setLevel('DEBUG')
        BASIC_FORMAT = "[vld_reader] %(asctime)s.%(msecs)03d:%(levelname)s:%(message)s"
        DATE_FORMAT = '%Y-%m-%d %H:%M:%S'
        formatter = logging.Formatter(BASIC_FORMAT, DATE_FORMAT)
        terminalHandler = logging.StreamHandler() # handler outputs to console
        terminalHandler.setFormatter(formatter)
        terminalHandler.setLevel('INFO')
        logger.addHandler(terminalHandler)
        outdir=os.path.join(self.outputRoot,self.utcDate,self.utcHMS,"lidarlog")
        if not os.path.exists(outdir): os.makedirs(outdir)
        fileHandler = logging.FileHandler(os.path.join(outdir,self.filePref+'.log')) # handler write to .log file
        fileHandler.setFormatter(formatter)
        logger.addHandler(fileHandler)
        
        logger.info(f"Lidar logs will be written to {outdir}.")
        
        return logger


    def sensor_do(self, url, pf, buf):
        """
        From VLP-16 user mannual
        """
        self.sensor.setopt(self.sensor.URL, url) 
        self.sensor.setopt(self.sensor.POSTFIELDS, pf) 
        self.sensor.setopt(self.sensor.WRITEDATA, buf) 
        retriesLeft = 3
        delayBetweenRetries = 5 # seconds
        while retriesLeft > 0:
            try:
                self.sensor.perform() 
                rcode = self.sensor.getinfo(self.sensor.RESPONSE_CODE) 
                success = rcode in range(200, 207) 
                self.logger.info(f"{url} {pf}: {rcode} ({'OK' if success else 'ERROR'})")
                return success
            except Exception as e:
                self.logger.warning(e)
                for i in reversed(range(delayBetweenRetries)):
                    self.logger.info(f"retrying after {i+1}s.")
                    time.sleep(1)
                retriesLeft -= 1
        return False
    
    def deviceSettings(self):
        """
        Summarize lidar settings.
        """
        tb=PrettyTable(field_names=["key","value"])
        tb.add_rows(
            [
                ["model",f"{self.model}"],
                ["lidar's address",f"{self.lidarip}"],
                ["data port",f"{self.dataPort}"],
                ["rpm",f"{self.rpm}"],
                ["returnMode",f"{self.returnMode}"],
            ]
        )
        self.logger.info(f"Initialize lidar device with the following settings\n{tb}")
        
    def isAlive(self):
        """
        Return True if `laser is On` or `rpm!=0`.
        """
        http = urllib3.PoolManager()
        response = http.request('GET',self.Base_URL+"status.json")
        if response: 
            status = json.loads(response.data) 
            if status['laser']['state']=='On' or status['motor']['rpm']!=0:
                return True
            else:
                return False
            
    def initProgressBar(self, maxiters:int,desc:str=None):
        return tqdm(total=maxiters,desc=desc)
        
    def launch(self):
        self.logger.info(f"Launch the device {self.model} at {self.lidarip}:")
        rc = self.sensor_do(self.Base_URL+'reset', urlencode({'data':'reset_system'}), self.buffer) 
        if rc: 
            time.sleep(5) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'returns':self.returnMode}), self.buffer)
        if rc: 
            time.sleep(5) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'rpm':self.rpm}), self.buffer) 
        if rc: 
            time.sleep(10) 
            rc = self.sensor_do(self.Base_URL+'setting', urlencode({'laser':'on'}), self.buffer) 
            
        http = urllib3.PoolManager()
        response = http.request('GET',self.Base_URL+"status.json")
        if response: 
            status = json.loads(response.data) 
            self.logger.info(f"Sensor laser is {status['laser']['state']}, motor rpm is {status['motor']['rpm']}")
    
    def stop(self):
        self.logger.info(f"Stopping the device {self.model} at {self.lidarip}:")
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
                self.logger.info(f"Sensor laser is {status['laser']['state']}, motor rpm is {status['motor']['rpm']}")
        self.socket.close()
        self.sensor.close()

    def read_live_data(self):
        """
        https://github.com/valgur/velodyne_decoder/issues/4#issuecomment-1248660033
        """
        while True:
            data, address = self.socket.recvfrom(vd.PACKET_SIZE * 2)
            recv_stamp = time.time()
            yield self.decoder.decode(recv_stamp, data, self.as_pcl_structs)
            
    def _recvfrom(self):
        while not self.stream2pcapFlag:
            self.qStream.put(item=(time.time(), *self.socket.recvfrom(vd.PACKET_SIZE * 2))) # push tuple (timeStamp, data, addr) to the queue
        self.progressBar=self.initProgressBar(maxiters=self.qStream.qsize(),desc="Writing pcaps")
        self.stop()
            
    def stream2pcap(self, baseThread:threading.Thread=None, filename:str=None):
        assert baseThread is not None, f"Must specify baseThread (threading.Thread class) on which `q2pcap` is relied."
        if filename is None: 
            outdir=os.path.join(self.outputRoot,self.utcDate,self.utcHMS,"lidarout")
            if not os.path.exists(outdir): os.makedirs(outdir)
            filename =os.path.join(outdir,self.filePref+'.pcap')
        etherIPHead=(
                Ether(src='ff:ff:ff:ff:ff:ff',dst='ff:ff:ff:ff:ff:ff') # dst mac addr is broadcast with no doubt, but the src mac addr is also broadcast from the pcap file recorded by veloview. This would not affect the function of captured pcap file.
                / IP(src='192.168.0.200',dst='255.255.255.255') # src ip should be the ip of the lidar but is 192.168.0.200 in the pcap file recorded by veloview. This would not affect the function of captured pcap file.
                )
        pktWriter=PcapWriter(filename=filename,sync=True) # Make sure setting `sync=True` for steady file output. Otherwise the frames parsed by veloview would be chaotic.
        counter=0
        while True:
            if self.qStream.empty() and not baseThread.is_alive(): # exit thread
                break
            elif self.qStream.empty() and baseThread.is_alive(): # waiting for data
                # print(f"q is empty, continue.")
                continue
            else: # processing data
                oneTimestamp,oneData,oneAddress=self.qStream.get()
                if counter==0: # discard first (idx==0) sample due to its strange time difference to the 2nd sample
                    counter+=1
                    continue
                counter+=1
                packet = (
                    etherIPHead
                    / UDP(sport=oneAddress[1],dport=oneAddress[1],chksum=0) # checksum is set to 0 (ignore) according to the pcap file recorded by veloview.
                    / oneData # use operator / to append the recieved data at last
                    )
                packet.time=oneTimestamp
                if self.progressBar is not None: 
                    self.progressBar.update(1)
                else:
                    self.logger.info(f"Write pcap of timestamp {oneTimestamp} to file, {self.qStream.qsize()} samples wait to be written.")
                pktWriter.write(packet)
        pktWriter.close()
        if self.progressBar is not None: self.progressBar.close()
                
        
def main(args):
    myld=ld(args.model,args.ip_lidar,args.dataport,args.rpm,args.returnmode,outputRoot=args.outdir)
    utcDate=datetime.now(timezone.utc).strftime('%Y%m%d')
    utcHMS=datetime.now(timezone.utc).strftime('%H%M%S')
    outdir=os.path.join(args.outdir,utcDate,utcHMS,"lidarpcap")
    if not os.path.exists(outdir): os.makedirs(outdir)
    myld.logger.info(f"Outputs will be written to {outdir}.")
    myld.launch()
        
    if args.mode=='live':
        # live mode
        for Data in myld.read_live_data():
            if Data != None:
                utc_time=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%f')
                stamp, points = Data
                print(utc_time, points.shape)
                print(f"points:{points}")
                break
    elif args.mode=='pcap':
        # write mode
        utc_time=datetime.now(timezone.utc)
        filenamePrefix=args.model+'_'+str(args.rpm)+'rpm'+args.returnmode.capitalize()+'_'+utc_time.strftime('%Y%m%dT%H%M%S.%f')
    
        threadList=[]
        threadRecv=threading.Thread(target=myld._recvfrom,name='_recvfrom')
        threadList.append(threadRecv)
        pcapFilename=os.path.join(outdir,filenamePrefix+'.pcap')
        threadStream2pcap=threading.Thread(target=myld.stream2pcap,name='stream2pcap',args=(threadRecv,pcapFilename))
        threadList.append(threadStream2pcap)
        for oneThread in threadList:
            myld.logger.info(f"Starting thread:\t{oneThread.name}.")
            oneThread.start()
            
        try:
            time.sleep(60) # seconds
        except KeyboardInterrupt:
            myld.logger.info("User interruption.")
        finally:
            myld.stream2pcapFlag=True
            
        for oneThread in threadList:
            oneThread.join()
            myld.logger.info(f"Thread {oneThread.name} exited.")
    
    if myld.isAlive(): myld.stop()
                
        
if __name__=="__main__":
    parser=argparse.ArgumentParser(description="read data directly from velodyne lidar")
    parser.add_argument('--model', default='VLP-16', type=str,choices=supportModels,metavar="MODEL", help="Model of the velodyne lidar you use.")
    parser.add_argument('--ip-lidar', default='192.168.1.201', type=str,metavar="Lidar_IP", help="IP addr of velodyne lidar. Default:'192.168.1.201'.")
    parser.add_argument('--ip-local', default='', type=str,metavar="localhost", help="IP addr of localhost, not the velodyne lidar. Default:''(listen to all).")
    parser.add_argument('--dataport', default=2368, type=int,metavar="PORT", help="Data port to be listened for UDP packages. Default: 2368.")
    parser.add_argument('--rpm', default=1200, type=int,metavar="RPM", help="RPM of the velodyne lidar to be set.")
    parser.add_argument('--returnmode',default='dual',choices=['strongest','last','dual'],type=str,metavar="ReturnMode", help="pcap: read data and write to pcap file; live: read data in stream mode.")
    parser.add_argument('--mode',default='pcap',choices=['pcap','live'],type=str,metavar="MODE", help="pcap: read data and write to pcap file; live: read data in stream mode.")
    parser.add_argument('--outdir',default='out',type=str,metavar="DIR", help="output path.")
    args=parser.parse_args()
    main(args)