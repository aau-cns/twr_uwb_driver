#!/usr/bin/env python3
import rospy
import time
import sys
import os
from std_msgs.msg import String
from uwb_msgs.msg import TwoWayRangeStamped
import serial
import numpy as np
import logging
import queue
import threading
import serial
import signal


# ref: https://github.com/zidik/threaded_serial/blob/master/threaded_serial/threaded_serial.py

class ThreadedSerialManager:
    def __init__(self, connection: serial.Serial, callback=lambda x: None):
        self.connection = connection
        self.connection.timeout = 0.1
        self.callback = callback

        self._stop = False
        self._send_queue = queue.Queue()

        self._receiving_thread = threading.Thread(target=self._run_receiving_thread, name="ReceivingThread")
        self._sending_thread = threading.Thread(target=self._run_sending_thread, name="SendingThread")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        """
        Start managing serial port
        """
        logging.info("{} {} starting...".format(self.connection.name, type(self).__name__))
        if not self.connection.isOpen():
            logging.info("Connecting to {}".format(self.connection.name))
            self.connection.open()
            logging.info("Connected to {}".format(self.connection.name))
        self._receiving_thread.start()
        self._sending_thread.start()
        logging.info("{} {} started".format(self.connection.name, type(self).__name__))

    def stop(self, timeout=None, close=True):
        """
        Signal threads to stop
        :param timeout: thread join timeout
        :param close: close serial connection
        :return:
        """
        logging.info("{} {} stopping...".format(self.connection.name, type(self).__name__))
        self._stop = True
        if threading.current_thread() is not self._receiving_thread:
            self._receiving_thread.join(timeout)
        if threading.current_thread() is not self._sending_thread:
            self._sending_thread.join(timeout)
        if close:
            self.connection.close()
        logging.info("{} {} stopped".format(self.connection.name, type(self).__name__))

    def send(self, data):
        self._send_queue.put_nowait(data)

    def send_string(self, string, encoding="ascii"):
        self.send(string.encode(encoding))

    def _run_receiving_thread(self):
        """
        Thread that monitors serial port and calls callback function every time it receives a full line.
        """
        logging.debug("{} {} started". format(self.connection.name, threading.current_thread().name))
        try:
            raw_line = bytearray()
            while not self._stop:
                raw_line += self.connection.readline()
                if len(raw_line) < 1:
                    continue
                if raw_line[-1] != 10: # when not ending with "Line feed" (timeout occurred)
                    continue

                self.callback(raw_line)
                raw_line = bytearray()
        except Exception as e:
            logging.exception(e)
        self._stop = True  # Also bring down the other thread
        logging.debug("{} {} stopped". format(self.connection.name, threading.current_thread().name))

    def _run_sending_thread(self):
        """
        Thread that sends data that has been placed in _send_queue
        """
        logging.debug("{} {} started". format(self.connection.name, threading.current_thread().name))
        try:
            while not self._stop:
                try:
                    data = self._send_queue.get(timeout=0.1)
                except queue.Empty:
                    pass
                else:
                    self.connection.write(data)
        except Exception as e:
            print(type(data))
            logging.exception(e)
        self._stop = True  # Also bring down the other thread
        logging.debug("{} {} stopped". format(self.connection.name, threading.current_thread().name))



# class Sensor:
class SensorUWB:

    def __init__(self):
        # info
        self.firmware_version   = [1.0]
        self.hardware_version  = [1001]        

        # initialise ros node
        rospy.init_node("~", anonymous=False,log_level=rospy.INFO)

        #rospy.on_shutdown(self._signal_handler)
        
        # get params
        self.device_port    = rospy.get_param('~port', '/dev/ttyAMA0')
        self.baudrate       = int(rospy.get_param('~baudrate', '921600'))
        self.uwb_topic      = rospy.get_param('~topic_name', 'ranging')
        self.uwb_freq       = rospy.get_param('~rate_Hz', '1')
        self.device_type    = rospy.get_param('~device_type', 'A')
        self.anchor_ids     = rospy.get_param('~anchor_ids', [1, 2, 3, 4])
        self.tag_ids        = rospy.get_param('~tag_ids', [0])
        self.ID_offset      = int(rospy.get_param('~ID_offset', 0))
        self.verbose        = rospy.get_param('~verbose', False)


        if isinstance(self.anchor_ids, str):
            print("convert anchor_ids as string into int list...")
            self.anchor_ids = self.anchor_ids.replace('[', '')
            self.anchor_ids = self.anchor_ids.replace(']', '')
            ids_list        = self.anchor_ids.split(',') # split by comma
            self.anchor_ids = [int(x) for x in ids_list]
        if isinstance(self.tag_ids, str):
            print("convert tag_ids as string into int list...")
            self.tag_ids = self.tag_ids.replace('[', '')
            self.tag_ids = self.tag_ids.replace(']', '')
            ids_list        = self.tag_ids.split(',') # split by comma
            self.tag_ids = [int(x) for x in ids_list]



        rospy.loginfo("Node config:" +
            "\n\t device_port: "       + str(self.device_port) +
            "\n\t baudrate: "       + str(self.baudrate) +
            "\n\t uwb_topic: "          + str(self.uwb_topic) +
            "\n\t rate_Hz: "  + str(self.uwb_freq) +
            "\n\t device_type: "     + str(self.device_type) + 
            "\n\t anchor_ids: "     + str(self.anchor_ids) + 
            "\n\t tag_ids: "     + str(self.tag_ids) +
            "\n\t verbose: "     + str(self.verbose))

            
        # init topology, next version this information will be included in the UWB firmware
        #   "A": anchor
        #   "T": tag
        #   "U": unknown
        self.topology = {   'A': self.anchor_ids,
                            'T': self.tag_ids 
                        }
        # intialise serial communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=0)
            rospy.loginfo("Opening serial communication with UWB sensor")
            try:
                if self.serial.in_waiting:
                    self.serial.read_all()
            except:
                 rospy.loginfo("Opening serial communication with UWB sensor failed")
                 pass
        except:
            rospy.logerr("Can not open serial" + self.device_port)
            self.serial.close
            sys.exit
        rospy.loginfo("Serial communication with UWB sensor SUCCEED")

        # https://stackoverflow.com/a/46810180
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False
        self.threaded = ThreadedSerialManager(connection=self.serial, callback=self.data_received)

        
        # set frequency
        self.rate   = rospy.Rate(self.uwb_freq)
        
        # get only the most recent measurement
        self.pub    = rospy.Publisher(self.uwb_topic, TwoWayRangeStamped, queue_size=1)
        
        # initialise UWB message
        self.msg    = TwoWayRangeStamped()

    def _signal_handler(self):
        rospy.loginfo(" ... in signal handler")
        self.terminated = True

    def data_received(self, byte_arr):
        # decode bytes object to produce a string: https://stackoverflow.com/a/606199
        if len(byte_arr) > 1:
            try:
                line = byte_arr.decode("utf-8")
                line = line.replace('\r\n', '')
                line = line.replace('\n', '')
                self.distribute_data(line)
            except Exception as e:
                print('Error in decoding line' + str(byte_arr))
                print(str(e))
                pass

    def send_string(self, msg):
        self.threaded.send_string(msg)

    def distribute_data(self, msg):
        msg = msg.replace(' ', ',')
        msg = msg.replace(',,,', ',')
        msg = msg.replace(',,', ',')

        serial              = msg.split(',') # split by comma
        
        if(len(serial) == 4):
            self.msg.header.seq = int(serial[0])
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.frame_id = ""
            self.msg.UWB_ID1    = int(serial[1])
            self.msg.UWB_ID2    = int(serial[2])

            # check type of transmitter
            if int(self.msg.UWB_ID1) in self.topology.get('A'):
                self.msg.type_ID1    = ord('A')
                self.msg.header.frame_id = "A" + str(self.msg.UWB_ID1)
            elif int(self.msg.UWB_ID1) in self.topology.get('T'):
                self.msg.type_ID1    = ord('T')
                self.msg.header.frame_id = "T" + str(self.msg.UWB_ID1)
            else:
                self.msg.type_ID1    = ord('U')
                


            # check type of receiver
            if int(self.msg.UWB_ID2)  in self.topology.get('A'):
                self.msg.type_ID2    = ord('A')
            elif int(self.msg.UWB_ID2)   in self.topology.get('T'):
                self.msg.type_ID2    = ord('T')
            else:
                self.msg.type_ID2    = ord('U')

            # virtually increase the ID
            self.msg.UWB_ID1    = int(self.msg.UWB_ID1 + self.ID_offset)
            self.msg.UWB_ID2    = int(self.msg.UWB_ID2 + self.ID_offset)

            self.msg.LOS        = True
            self.msg.range_raw  = float(serial[3])
            self.msg.range_corr = np.uint32(0)
            
            if self.verbose:
                rospy.loginfo(self.msg)   
            self.pub.publish(self.msg)
        else:
            rospy.loginfo("Expected 4 tokens in msg (delimiter is [,]): " + str(msg))
            rospy.loginfo("- got following tokens: " + str(serial))  


    def start(self):
        self.threaded.start()

    def stop(self):
        self.threaded.stop()


    # get firmware and hardware version        
    def getVersion(self):
         output = "UWB FIRMWARE V" + str(self.firmware_version) + "UWB MODEL" + str(self.hardware_version)
         rospy.loginfo(output)

    def run(self):
        self.start()
        count = 0
        while not rospy.is_shutdown() and not self.terminated:
            self.rate.sleep()

        rospy.loginfo(" stop threaded serial...")  
        self.stop()
        rospy.loginfo("done!")  
            

if __name__ == '__main__':
    
    uwb = SensorUWB()
    uwb.getVersion()
    
    try:
        uwb.run()
    except rospy.ROSInterruptException:
        pass
