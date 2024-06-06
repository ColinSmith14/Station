import configparser
import random
import logging
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import RPi.GPIO as GPIO
import time
from datetime import datetime
import socket
from typing import List
from threading import Thread
from pymongo import MongoClient
from dataclasses import dataclass
import json
from .RFID import Scanner
import os

# !/usr/bin/env python

# being logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# reading station config for pin #'s
with open('station/config.json', 'r') as config_file:
    config = json.load(config_file)
    
# stores station info as described in mongodb
@dataclass
class StationStatus:
    """Holds all status of Station"""
    station_id: int = 0  # ID of Station
    last_rfid: int = 0  # Last scanned
    pickup_rfid: int = 0  # Will not be used
    dropoff_rfid: int = 0  # Will not be used
    last_pickup_time: datetime = datetime.now()  # When the most recent pickup has occured
    in_place: bool = False  # Rack in place
    enabled: bool = True  # Station is turned on or not
    rfid: int = 0  # The rfid that the station is at
    path: List[int] = None  # Also not sure what this is used for
    cycle_time: datetime = datetime.now()  # Not really sure what this is used for
    distance: int = 0  # Distance from "Home" to station?
    median_time: int = 500  # Time between racks
    assigned: int = 0  # Which AGV is assigned to this station
    in_progress: bool = False  # Will be unused
    pull = None  # Unused?
    name: str = ""  # Name of station
    allow_prod: bool = True  # If production is allowed to print another label

def get_db():
    """Get the database"""
    try:
        config = configparser.ConfigParser()
        config.read('/home/ubuntu/fisher_agc/AGCROS/.ini')
        db_uri = config.get('AGVP2', "MONGO_URI")
        db_ns  = config.get('AGVP2', "MONGO_NS")
        db = MongoClient(db_uri)[db_ns]
        return db
    except Exception as err:
        print("Error getting database: ".format(err))
        
def update_station(db, station):
    if not isinstance(db, MongoClient):
        raise ValueError("DB must be of type MongoClient")
    
    if not isinstance(station, StationStatus):
        raise ValueError("Station must be of type StationStatus")
    update_doc = {
        "station_id": station.station_id,
        "last_rfid": station.last_rfid,
        "pickup_rfid": station.pickup_rfid,
        "dropoff_rfid": station.dropoff_rfid,
        "last_pickup_time": station.last_pickup_time,
        "in_place": station.in_place,
        "enabled": station.enabled,
        "rfid": station.rfid,
        "path": station.path,
        "cycle_time": station.cycle_time,
        "distance": station.distance,
        "median_time": station.median_time,
        "assigned": station.assigned,
        "in_progress": station.in_progress,
        "pull": station.pull,
        "name": station.name,
        "allow_prod": station.allow_prod
    }
    try:
        return db.stations.update_one(
            {"_id": station.station_id},
            {"$set": update_doc},
            upsert=True
        )
    except Exception as err:
        print("Error: {0}".format(err))
        return False

def get_status(db, station):
    try:
        doc = db.stations.find_one(
            {"name": station.name}
        )
        station.station_id = doc["_id"]
        station.last_rfid = doc["last_rfid"]
        station.dropoff_rfid = doc["dropoff_rfid"]
        station.pickup_rfid = doc["pickup_rfid"]
        station.last_pickup_time = doc["last_pickup_time"]
        station.in_place = doc["in_place"]
        station.enabled = doc["enabled"]
        station.rfid = doc["rfid"]
        station.path = doc["path"]
        station.cycle_time = doc["cycle_time"]
        station.distance = doc["distance"]
        station.median_time = doc["median_time"]
        station.assigned = doc["assigned"]
        station.in_progress = doc["in_progress"]
        station.pull = doc["pull"]
        station.name = doc["name"]
        station.allow_prod = doc["allow_prod"]

    except Exception as err:
        print("Error: {0}".format(err))
        return None

def update_one_field(db, station : StationStatus, field: str):
    if not isinstance(db, MongoClient):
        raise ValueError("DB must be of type MongoClient")
    if not isinstance(station, StationStatus):
        raise ValueError("Station must be of type StationStatus")
    
    update_doc = {
        field: getattr(station, field)
    }
    try:
        return db.stations.update_one(
            {"name": station.name},
            {"$set": update_doc},
            upsert=True
        )
    except Exception as err:
        print("Error: {0}".format(err))
        return False

class Sensor:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        logging.info(f"Sensor initialized: {self.pin}")

    def read(self):
        try:
            logging.info(f"Reading sensor: {self.pin}")
            return GPIO.input(self.pin)
        except Exception as e:
            logging.error(f"Error reading sensor: {e}")
            return None

class LightStack:
    def __init__(self, config):
        self.config = config
        self.r = False
        self.g = False
        self.b = False
        self.count = 0
        GPIO.setup(self.config['light1'], GPIO.OUT)
        GPIO.setup(self.config['light2'], GPIO.OUT)
        GPIO.setup(self.config['light3'], GPIO.OUT)
        logging.info("Light stack initialized")

    def change_color(self, cart, docking, flash_red):
        if(cart and docking):
            self.blue()
        elif(cart and not docking):
            self.blue()
        elif(not cart and not docking):
            if(flash_red):
                self.flash_red()
            else:
                self.green()

        try:
            GPIO.output(self.config['light1'], self.r)
            GPIO.output(self.config['light2'], self.g)
            GPIO.output(self.config['light3'], self.b)
        except Exception as e:
            logger.error(f"Error changing light color: {e}")

    def blue(self):
        self.r = False
        self.g = False
        self.b = True

    def green(self):
        self.r = False
        self.g = True
        self.b = False
    
    def flash_red(self):
        if(self.count % 2 == 0):
            self.r = True # Flash red light during docking
            self.g = False
            self.b = False
        else:
            self.r = False
            self.g = False
            self.b = False
        self.count += 1
        
class Station:
    def __init__(self, config, name):
        self.station_status = StationStatus(name=name)
        self.db = get_db()
        self.config = config
        GPIO.setmode(GPIO.BOARD)
        self.light_stack = LightStack(self.config['GPIO_pins'])
        self.sensor1 = Sensor(self.config['GPIO_pins']['sensor1'])
        self.sensor2 = Sensor(self.config['GPIO_pins']['sensor2'])
        self.cart_in_place = False
        self.in_progress = False
        self.flash_red_timer = 0
        self.timer_duration = 10
        self.flash_red = True
        

    def check_sensors(self):
        try:
            sensor1_status = self.sensor1.read()
            sensor2_status = self.sensor2.read()
            if sensor1_status and sensor2_status:
                time.sleep(1)
                if self.sensor1.read() and self.sensor2.read():
                    return True
            return False
        except Exception as e:
            return None
    
    def update_cart_status(self):
        try:
            self.station_status.in_place = self.cart_in_place
            update_one_field(self.db, self.station_status, "in_place")
            logging.info(f"Cart status updated: {self.cart_in_place}")
        except Exception as e:
            logging.error(f"Error updating cart status: {e}")
        
    def fetch_in_progress(self):
        try:
            get_status(self.db, self.station_status)
            logging.info(f"Fetch in progress: {self.station_status.in_progress}")
            self.in_progress = self.station_status.in_progress
        
        except Exception as e:
            logging.error(f"Error fetching in progress: {e}")
            return None
    
    def run(self):
        while rclpy.ok():
            self.fetch_in_progress()
            self.cart_in_place = self.check_sensors()
            self.update_cart_status()
            if(not self.cart_in_place and not self.in_progress):
                if(self.flash_red_timer > 0):
                    self.flash_red = True
                    self.flash_red_timer -= 1
                else:
                    self.flash_red = False
            if(self.cart_in_place):
                self.flash_red_timer = self.timer_duration
            self.light_stack.change_color(self.cart_in_place, self.station_status.in_place, self.flash_red)
            time.sleep(1)
            logging.info("Docking status: {} Cart Status: {}".format(self.docking, self.cart_in_place))
    
    def main():
        rclpy.init()
        station_name = socket.gethostname().split('-', 1)[0]
        station = Station(config, station_name)
        station.run()
        rclpy.spin(station)
        rclpy.shutdown()
