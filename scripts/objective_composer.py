#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rosbag
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('Check ' + link)) from e
from std_msgs.msg import String
import ipfsapi
from flask import Flask, request
from flask_restful import Resource, Api
from flask.json import jsonify
from flask_cors import CORS
from urllib.parse import urlparse
import tempfile
import shelve
import os


rospy.init_node('objective_composer')

ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

app = Flask(__name__)
CORS(app)
api = Api(app)

per_dir = rospy.get_param('~per_dir', os.path.dirname(os.path.realpath(__file__)))
with shelve.open(per_dir + '/storage.shelve') as db:
    if not 'composed' in db:
        db['composed'] = 0

tmp_dir = rospy.get_param('~tmp_dir', '/tmp/agent')
if not os.path.exists(tmp_dir):
    os.makedirs(tmp_dir)


class Stats(Resource):
    def get(self):
        with shelve.open(per_dir + '/storage.shelve') as db:
            rospy.loginfo('Stats request. Composed: %d' % db['composed'])
            return jsonify({'objectivesComposed': db['composed']})

class ComposeObjective(Resource):
    def post(self):
        """request example:
           $ curl -H "Content-type: application/json" \
                  -X POST http://127.0.0.1:3020/ \
                  -d '{"GPS":"49.368550,75.403660", "Params":"no,so2"}'
        """
        rospy.loginfo('Composer request for: ' + str(request.get_json()))
        gps = request.get_json()['GPS']
        params = request.get_json()['Params']
        bag_name = tmp_dir + '/objective-' + next(tempfile._get_candidate_names()) + '.bag'
        with rosbag.Bag(bag_name, 'w') as bag:
            bag.write('/agent/objective/position', String(data=gps))
            bag.write('/agent/objective/parameters', String(data=params))
        with shelve.open(per_dir + '/storage.shelve') as db:
            db['composed'] += 1
        objective_hash = ipfs.add(bag_name)['Hash']
        rospy.loginfo('Objective: ' + objective_hash)
        return jsonify({'objective': objective_hash})

api.add_resource(Stats, '/stats')
api.add_resource(ComposeObjective, '/get_objective')


server_address = urlparse(rospy.get_param('~server_address')).netloc.split(':')
app.run(host=server_address[0],
        port=int(server_address[1]),
        ssl_context=(rospy.get_param('~certfile'), rospy.get_param('~keyfile')))
rospy.spin()
