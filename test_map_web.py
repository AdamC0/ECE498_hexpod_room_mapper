"""ROS-independent integration tests for the embedded web server."""

import json
import socket
import threading
import unittest
import urllib.error
import urllib.parse
import urllib.request
from types import SimpleNamespace

import numpy as np

import map_web
from map_explore import FREE, OCCUPIED


class ObjectTrackerStub:
    min_observations = 3

    def snapshot(self, confirmed_only):
        return [{'id': 1, 'label': 'chair', 'name': 'Desk chair',
                 'x': 1.0, 'y': 0.5, 'count': 3}]


class FakeNode:
    def __init__(self):
        n = 10
        self.camera = None
        self.view = {}
        self.use_depth = False
        self.t_depth = 0.0
        self.floor_profile = None
        self.depth_health = 'waiting'
        self.paused = False
        self.explorer = SimpleNamespace(
            state=np.zeros((n, n), np.uint8), half=0.5, cell=0.1, n=n)
        self.pose = None
        self.map_lock = threading.Lock()
        self.path = []
        self.web_map_center = (0., 0.)
        self.body_radius = 0.25
        self.object_lock = threading.Lock()
        self.object_detections = []
        self.object_tracker = ObjectTrackerStub()
        self.commands = []

    def web_status(self):
        return 'test status'

    def web_battery(self):
        return {'available': True, 'percentage': 72.4, 'voltage': 11.77,
                'charging': False, 'age': 0.1, 'source': '/voltage'}

    def web_speed(self):
        return {'translation': 100.0, 'rotation': 100.0,
                'minimum': 50.0, 'maximum': 200.0}

    def web_request(self, command):
        self.commands.append(command)


class WebServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = FakeNode()
        cls.port = map_web.start(cls.node, 0)
        cls.base = 'http://127.0.0.1:%d' % cls.port

    def request(self, path, method='GET'):
        request = urllib.request.Request(self.base + path, method=method)
        return urllib.request.urlopen(request, timeout=2)

    def test_os_selected_port_and_page(self):
        self.assertGreater(self.port, 0)
        with self.request('/') as response:
            body = response.read().decode()
            self.assertEqual(response.status, 200)
            self.assertIn('JetHexa room mapper', body)
            self.assertIn("setTimeout(load,ms);load()", body)
            self.assertIn("if(wasDriving)drivePulse('stop')", body)
            self.assertNotIn('driveStop();driveCommand=command', body)
            self.assertIn("data-tab='objects'", body)
            self.assertIn("post('/navigate?'+q)", body)
            self.assertIn("CANCEL ROUTE", body)
            self.assertIn("id='center-map'", body)
            self.assertIn("id='zoom-in'", body)
            self.assertIn("id='battery'", body)
            self.assertIn("fetch('/battery.json')", body)
            self.assertIn("id='translation-speed'", body)
            self.assertIn("id='rotation-speed'", body)
            self.assertIn("fetch('/speed.json')", body)
            self.assertIn("max='200'", body)
            self.assertNotIn('Navigate to this map position?', body)

    def test_status_jpeg_and_object_endpoints(self):
        with self.request('/status') as response:
            self.assertEqual(response.read(), b'test status')
        for path in ('/cam.jpg', '/map.jpg'):
            with self.request(path) as response:
                self.assertEqual(response.headers.get_content_type(), 'image/jpeg')
                self.assertEqual(response.read()[:2], b'\xff\xd8')
        with self.request('/objects.json') as response:
            objects = json.loads(response.read().decode())
            self.assertEqual(objects[0]['name'], 'Desk chair')
            self.assertTrue(objects[0]['confirmed'])
        with self.request('/battery.json') as response:
            battery = json.loads(response.read().decode())
            self.assertEqual(battery['percentage'], 72.4)
            self.assertEqual(battery['voltage'], 11.77)
        with self.request('/speed.json') as response:
            speed = json.loads(response.read().decode())
            self.assertEqual(speed['translation'], 100.0)

    def test_controls_and_missing_route(self):
        for command, path in (('pause', '/pause'), ('cancel_route', '/cancel-route'),
                              ('resume', '/resume'),
                              ('clear_map', '/clear-map')):
            with self.request(path, 'POST') as response:
                self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-4:],
                         [('pause',), ('cancel_route',), ('resume',), ('clear_map',)])
        with self.assertRaises(urllib.error.HTTPError) as raised:
            self.request('/missing')
        self.assertEqual(raised.exception.code, 404)

    def test_manual_drive_endpoint_validates_commands(self):
        for command in ('forward', 'back', 'left', 'right', 'stop'):
            with self.request('/manual?command=' + command, 'POST') as response:
                self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-5:],
                         [('manual', command) for command in
                          ('forward', 'back', 'left', 'right', 'stop')])
        with self.assertRaises(urllib.error.HTTPError) as raised:
            self.request('/manual?command=launch', 'POST')
        self.assertEqual(raised.exception.code, 400)

    def test_speed_endpoint_validates_and_queues_both_sliders(self):
        with self.request('/speed?translation=125&rotation=80', 'POST') as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-1], ('speed', 125.0, 80.0))
        for path in ('/speed?translation=201&rotation=100',
                     '/speed?translation=nope&rotation=100'):
            with self.assertRaises(urllib.error.HTTPError) as raised:
                self.request(path, 'POST')
            self.assertEqual(raised.exception.code, 400)

    def test_navigation_and_object_management_endpoints(self):
        with self.request('/navigate?px=210&py=160', 'POST') as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-1], ('navigate', 1.0, 0.0))
        with self.request('/navigate?px=210&py=160&scale=100&dx=.5&dy=-.25', 'POST') as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-1], ('navigate', 1.0, -0.25))
        # A rendered viewport center is authoritative even if the robot moves
        # before the click reaches the server.
        self.node.pose = (8., 9., 0.)
        with self.request('/navigate?px=210&py=160&scale=100&cx=.5&cy=-.25', 'POST') as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-1], ('navigate', 1.0, -0.25))
        self.node.pose = None
        for action, expected in (
                ('navigate', ('navigate_object', 1)),
                ('remove', ('remove_object', 1))):
            with self.request('/object/%s?id=1' % action, 'POST') as response:
                self.assertEqual(response.status, 200)
            self.assertEqual(self.node.commands[-1], expected)
        name = urllib.parse.quote('Reading chair')
        with self.request('/object/rename?id=1&name=' + name, 'POST') as response:
            self.assertEqual(response.status, 200)
        self.assertEqual(self.node.commands[-1], ('rename_object', 1, 'Reading chair'))

    def test_map_view_tracks_robot_pose(self):
        self.node.pose = (1.25, -0.75, 0.)
        self.assertEqual(map_web.map_view_center(self.node), (1.25, -0.75))
        self.node.pose = None
        self.assertEqual(map_web.map_view_center(self.node), (0., 0.))

    def test_map_image_accepts_valid_view_and_rejects_invalid_zoom(self):
        with self.request('/map.jpg?scale=100&dx=.5&dy=-.25') as response:
            self.assertEqual(response.headers.get_content_type(), 'image/jpeg')
            self.assertEqual(float(response.headers['X-Map-Center-X']), .5)
            self.assertEqual(float(response.headers['X-Map-Center-Y']), -.25)
            self.assertEqual(float(response.headers['X-Map-Scale']), 100.)
        with self.assertRaises(urllib.error.HTTPError) as raised:
            self.request('/map.jpg?scale=500')
        self.assertEqual(raised.exception.code, 400)

    def test_depth_only_obstacles_use_distinct_map_color(self):
        n = FakeNode()
        n.explorer.state[5, 5] = OCCUPIED
        n.explorer.slam_state = np.full_like(n.explorer.state, FREE)
        depth_mask = np.zeros_like(n.explorer.state, dtype=bool)
        depth_mask[5, 5] = True
        n.explorer.depth_obstacle_mask = lambda: depth_mask
        image = map_web.draw_map(n)
        matches = np.all(image == map_web.DEPTH_OBSTACLE_COLOR, axis=2)
        self.assertTrue(matches.any())
        self.assertEqual(tuple(image[207, 207]), map_web.DEPTH_OBSTACLE_COLOR)

        n.explorer.slam_state[5, 5] = OCCUPIED
        image = map_web.draw_map(n)
        self.assertEqual(tuple(image[207, 207]), (0, 0, 0))

    def test_navigation_endpoints_reject_invalid_input(self):
        for path in ('/navigate?px=-1&py=20', '/navigate?px=nope&py=20',
                     '/object/navigate?id=0', '/object/rename?id=1&name='):
            with self.assertRaises(urllib.error.HTTPError) as raised:
                self.request(path, 'POST')
            self.assertEqual(raised.exception.code, 400)

    def test_busy_port_falls_back(self):
        with socket.socket() as occupied:
            occupied.bind(('127.0.0.1', 0))
            occupied.listen()
            busy_port = occupied.getsockname()[1]
            if busy_port == 65535:
                self.skipTest('no following port is available for this check')
            selected = map_web.start(FakeNode(), busy_port)
        self.assertGreater(selected, busy_port)
        self.assertLessEqual(selected, busy_port + 9)


if __name__ == '__main__':
    unittest.main()
