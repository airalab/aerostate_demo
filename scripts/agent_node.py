#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
from robonomics_lighthouse.msg import Ask, Bid
from std_msgs.msg import String
from std_srvs.srv import Empty
try:
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('Check ' + link)) from e
from web3 import Web3, HTTPProvider
import requests
import threading
import json


class Agent:

    current_job = {'position': None, 'parameters': None}

    def __init__(self):
        rospy.init_node('agent')

        self.model = rospy.get_param('~model')
        self.token = rospy.get_param('~token')
        self.bid_lifetime = rospy.get_param('~bid_lifetime')
        self.web3 = Web3(HTTPProvider(rospy.get_param('~web3_http_provider')))
        self.apikey = rospy.get_param('~apikey')

        self.signing_bid = rospy.Publisher('lighthouse/infochan/signing/bid', Bid, queue_size=128)

        def on_incoming_ask(incoming_ask):
            if incoming_ask.model == self.model and incoming_ask.token == self.token:
                rospy.loginfo('Incoming ask for my model and token.')
                self.make_bid(incoming_ask)
            else:
                rospy.loginfo('Incoming ask not for my model, skip.')
        rospy.Subscriber('infochan/incoming/ask', Ask, on_incoming_ask)

        def on_position(msg):
            self.curent_job['position'] = msg.data
        rospy.Subscriber('~objective/position', String, on_position)

        def on_parameters(msg):
            self.curent_job['parameters'] = msg.data
        rospy.Subscriber('~objective/parameters', String, on_parameters)

        self.result_position = rospy.Publisher('~result/position', String, queue_size=10)
        self.result_values = rospy.Publisher('~result/values', String, queue_size=10)

        rospy.wait_for_service('liability/finish')
        self.finish = rospy.ServiceProxy('liability/finish', Empty)

        threading.Thread(target=self.process, daemon=True).start()

    def make_bid(self, incoming_ask):
        rospy.loginfo('Making bid...')

        bid = Bid()
        bid.model = self.model
        bid.objective = incoming_ask.objective
        bid.token = self.token
        bid.cost = incoming_ask.cost
        bid.lighthouseFee = 0
        bid.deadline = self.web3.eth.getBlock('latest').number + self.bid_lifetime
        self.signing_bid(bid)

    def process(self):
        while True:
            while not all(self.curent_job.values()):
                rospy.sleep(1)
            rospy.loginfo('Starting process: ' + str(self.curent_job))
            data = self.get_data(self.current_job['position'], self.current_job['parameters'])
            self.result_position.publish(String(data=self.current_job['position']))
            self.result_values.publish(String(data=str(data)))
            rospy.loginfo('Process complete: ' + str(self.curent_job))
            self.current_job = dict.fromkeys(self.curent_job, None)
            self.finish()

    def get_data(self, position, parameters):
        url = "https://api.aerostate.io/{pos}/{param}".format(pos=position, param=parameters)
        headers = {"Authorization": "basic " + self.apikey}
        response = requests.get(url, headers=headers)
        return response.content
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    Agent().spin()
