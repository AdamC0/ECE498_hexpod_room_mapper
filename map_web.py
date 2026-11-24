#!/usr/bin/env python3
"""The room-mapper live camera/map page.

It serves the live camera, SLAM map, status, object JSON, and the autonomous
mapper's pause/resume safety controls.
"""

import json
import math
import os
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from video_recording import VideoRecorder
from web_dashboard import PAGE



MAP_SIZE, MAP_SCALE = 420, 50.0
DEPTH_OBSTACLE_COLOR = (180, 0, 220)  # BGR: magenta in the encoded map
CONFIRMED_OBJECT_COLOR = (0, 165, 255)
PENDING_OBJECT_COLOR = (0, 205, 255)
CAMERA_RECOVERY_LOCK = threading.Lock()


def recover_camera():
    """Run the fixed RGB recovery helper and return its combined output."""
    if not CAMERA_RECOVERY_LOCK.acquire(False):
        return False, 'Camera recovery is already running', 409
    try:
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'recover_rgb_camera.sh')
        if not os.path.isfile(script):
            return False, 'Camera recovery script is missing', 500
        try:
            result = subprocess.run(
                ['/bin/bash', script], cwd=os.path.dirname(script),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, timeout=135)
        except subprocess.TimeoutExpired:
            return False, 'Camera recovery timed out after 135 seconds', 504
        output = (result.stdout or '').strip()
        if result.returncode:
            return False, output or 'Camera recovery script failed', 500
        return True, output or 'RGB recovery succeeded', 200
    finally:
        CAMERA_RECOVERY_LOCK.release()


def jpeg(img, quality=75):
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else b''


def camera_image(node):
    """Return a 640x480 RGB-camera image in OpenCV BGR order."""
    frame = node.camera
    if frame is None:
        return None, 640, 480
    source_h, source_w = frame.shape[:2]
    return cv2.resize(frame, (640, 480)).copy(), source_w, source_h


def fresh_detections(node):
    """Copy only detections that are still valid for the current camera view."""
    with node.object_lock:
        age = time.time() - getattr(node, 'object_detections_stamp', 0.0)
        fresh = 0.0 <= age <= getattr(node, 'object_detection_ttl', 1.0)
        return [dict(d) for d in node.object_detections] if fresh else []


def draw_detections(img, detections, source_w, source_h):
    """Draw YOLO boxes and labels on a camera image in place."""
    sx, sy = 640.0 / source_w, 480.0 / source_h
    for detection in detections:
        x1, y1, x2, y2 = detection['box']
        p1, p2 = (int(x1*sx), int(y1*sy)), (int(x2*sx), int(y2*sy))
        mapped = detection.get('map_point') is not None
        box_color = (60, 210, 60) if mapped else (0, 180, 255)
        cv2.rectangle(img, p1, p2, box_color, 2)
        text = '%s %.0f%%' % (detection['label'], 100*detection['confidence'])
        if detection.get('depth') is not None:
            text += ' %.1fm' % detection['depth']
        cv2.putText(img, text, (p1[0], max(42, p1[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, box_color, 2)
    return img


def draw_recording_camera(node):
    """Render raw RGB with YOLO annotations and no mapper/status overlays."""
    img, source_w, source_h = camera_image(node)
    if img is None:
        return np.zeros((480, 640, 3), np.uint8)
    return draw_detections(img, fresh_detections(node), source_w, source_h)


def draw_camera(node):
    img, source_w, source_h = camera_image(node)
    if img is None:
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(img, 'waiting for camera...', (170, 240), cv2.FONT_HERSHEY_SIMPLEX,
                    .8, (255,255,255), 2)
        return img
    v = node.view
    color = (40, 40, 235) if v.get('blocked') else (60, 200, 60)
    label = 'BLOCKED' if v.get('blocked') else 'clear'
    depth = v.get('depth', 99.0)
    depth_text = 'depth %.2f m (%s)' % (depth, v.get('depth_source', 'clear')) if depth < 99 else 'depth clear'
    if not node.use_depth:
        depth_text = 'depth disabled'
    elif time.time() - node.t_depth > 1.0:
        depth_text = 'depth stale / waiting'
    elif node.floor_profile is None:
        depth_text = 'learning floor'
    elif not node.depth_health.startswith('ready'):
        depth_text = 'depth unknown'
    cv2.putText(img, 'LiDAR %.2f m | %s - %s' % (v.get('front', 0), depth_text, label), (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    if node.paused:
        label = 'AUTO PAUSED'
        font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, .72, 2
        (text_w, text_h), baseline = cv2.getTextSize(label, font, scale, thickness)
        x, y = img.shape[1] - text_w - 18, 14 + text_h
        cv2.rectangle(img, (x - 8, 7), (img.shape[1] - 8, y + baseline + 7),
                      (28, 28, 155), -1)
        cv2.putText(img, label, (x, y), font, scale, (255,255,255), thickness,
                    cv2.LINE_AA)
    return draw_detections(img, fresh_detections(node), source_w, source_h)


def draw_depth_camera(node, near=0.2, far=4.0):
    """Render native collision depth with an on-image metre legend."""
    lock = getattr(node, 'data_lock', None)
    if lock is None:
        frame = getattr(node, 'depth', None)
        frame = None if frame is None else frame.copy()
    else:
        with lock:
            frame = getattr(node, 'depth', None)
            frame = None if frame is None else frame.copy()
    if frame is None:
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(img, 'waiting for native depth...', (135, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        return img

    valid = np.isfinite(frame) & (frame > 0.0)
    clipped = np.clip(np.where(valid, frame, far), near, far)
    scaled = np.clip((far - clipped) * 255.0 / (far - near), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    color[~valid] = 0
    img = cv2.resize(color, (640, 480), interpolation=cv2.INTER_NEAREST)

    return draw_depth_legend(img, near, far, 'native depth')


def draw_depth_legend(img, near, far, title):
    """Draw the shared depth colour scale into a 640x480 image."""
    # Near is red and far is blue. Invalid/no-return pixels stay black.
    y0, y1, x0, x1 = 70, 410, 568, 592
    values = np.linspace(255, 0, y1 - y0, dtype=np.uint8)[:, None]
    legend = cv2.applyColorMap(values, cv2.COLORMAP_JET)
    legend = np.repeat(legend, x1 - x0, axis=1)
    cv2.rectangle(img, (552, 42), (636, 438), (15, 15, 15), -1)
    img[y0:y1, x0:x1] = legend
    cv2.rectangle(img, (x0, y0), (x1, y1), (235, 235, 235), 1)
    for distance in (near, 1.0, 2.0, 3.0, far):
        fraction = (distance - near) / (far - near)
        y = int(round(y0 + fraction * (y1 - y0)))
        y = min(y1 - 1, max(y0, y))
        cv2.line(img, (x0 - 4, y), (x1 + 4, y), (255, 255, 255), 1)
        cv2.putText(img, ('%.1f' % distance), (596, min(430, y + 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 255), 1,
                    cv2.LINE_AA)
    cv2.putText(img, 'm', (604, 60), cv2.FONT_HERSHEY_SIMPLEX, .45,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(img, (560, 416), (572, 428), (0, 0, 0), -1)
    cv2.rectangle(img, (560, 416), (572, 428), (255, 255, 255), 1)
    cv2.putText(img, 'invalid', (577, 427), cv2.FONT_HERSHEY_SIMPLEX, .32,
                (255, 255, 255), 1, cv2.LINE_AA)
    status = ('%s | %.1f-%.1f m | black = invalid' % (title, near, far))
    cv2.rectangle(img, (0, 0), (430, 30), (15, 15, 15), -1)
    cv2.putText(img, status, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, .52,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def draw_depth_overlay(node, opacity=0.45, near=0.2, far=4.0):
    """Blend RGB-aligned CUDA registered depth over the live RGB image."""
    opacity = float(opacity)
    if not 0.0 <= opacity <= 1.0 or not math.isfinite(opacity):
        raise ValueError('Depth overlay opacity must be between zero and one')
    rgb, _, _ = camera_image(node)
    lock = getattr(node, 'data_lock', None)
    if lock is None:
        depth = getattr(node, 'registered_depth', None)
        depth = None if depth is None else depth.copy()
    else:
        with lock:
            depth = getattr(node, 'registered_depth', None)
            depth = None if depth is None else depth.copy()
    if rgb is None:
        rgb = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(rgb, 'waiting for RGB...', (190, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    if depth is None:
        cv2.putText(rgb, 'waiting for aligned depth...', (145, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 210, 255), 2)
        return draw_depth_legend(rgb, near, far, 'registered overlay')

    valid = np.isfinite(depth) & (depth > 0.0)
    clipped = np.clip(np.where(valid, depth, far), near, far)
    scaled = np.clip((far - clipped) * 255.0 / (far - near), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    color = cv2.resize(color, (640, 480), interpolation=cv2.INTER_NEAREST)
    valid = cv2.resize(valid.astype(np.uint8), (640, 480),
                       interpolation=cv2.INTER_NEAREST).astype(bool)
    blended = cv2.addWeighted(rgb, 1.0 - opacity, color, opacity, 0.0)
    rgb[valid] = blended[valid]
    cv2.putText(rgb, 'depth opacity %d%%' % int(round(opacity * 100.0)),
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, .52,
                (255, 255, 255), 1, cv2.LINE_AA)
    return draw_depth_legend(rgb, near, far, 'registered overlay')


class YoloVideoRecorder(VideoRecorder):
    """Record the dashboard's RGB+YOLO-only frame renderer."""

    def __init__(self, node, output_dir=None, fps=10.0):
        output_dir = output_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'recordings')
        super().__init__(lambda: draw_recording_camera(node),
                         lambda: getattr(node, 'camera', None) is not None,
                         output_dir, fps)


def to_screen(x, y, cx, cy, scale):
    return (int(MAP_SIZE/2 - (y-cy)*scale), int(MAP_SIZE/2 - (x-cx)*scale))


def map_view_center(node):
    """Center the moving web viewport on the latest robot position."""
    pose = getattr(node, 'pose', None)
    if pose is not None:
        return float(pose[0]), float(pose[1])
    return getattr(node, 'web_map_center', None) or (0.0, 0.0)


def draw_map(node, scale=MAP_SCALE, dx=0.0, dy=0.0, center=None):
    e, pose = node.explorer, node.pose
    # The occupancy data remains in world coordinates while this viewport
    # pans, keeping the robot fixed at the visual center.
    if center is None:
        cx, cy = map_view_center(node)
        cx, cy = cx + dx, cy + dy
    else:
        cx, cy = float(center[0]), float(center[1])
    # Every world-anchored overlay must use this render's exact transform.
    # Requiring scale in to_screen() prevents a future layer from silently
    # falling back to the historical fixed 50 px/m value.
    def overlay_point(x, y):
        return to_screen(x, y, cx, cy, scale)
    with node.map_lock:
        depth_mask_fn = getattr(e, 'depth_obstacle_mask', None)
        depth_mask = (depth_mask_fn() if callable(depth_mask_fn)
                      else np.zeros_like(e.state, dtype=bool))
        state = e.state.copy()
        slam_state = getattr(e, 'slam_state', state).copy()
        path = list(node.path)
    yy, xx = np.indices((MAP_SIZE, MAP_SIZE))
    wx = cx + (MAP_SIZE/2 - yy)/scale
    wy = cy + (MAP_SIZE/2 - xx)/scale
    raw_r = ((wy+e.half)/e.cell).astype(int)
    raw_c = ((wx+e.half)/e.cell).astype(int)
    inside = (raw_r >= 0) & (raw_r < e.n) & (raw_c >= 0) & (raw_c < e.n)
    r = np.clip(raw_r, 0, e.n-1)
    c = np.clip(raw_c, 0, e.n-1)
    img = np.full((MAP_SIZE, MAP_SIZE, 3), 90, np.uint8)
    img[inside & (state[r,c] == 1)] = (255,255,255)
    img[inside & (state[r,c] == 2)] = (0,0,0)
    depth_only = inside & depth_mask[r,c] & (slam_state[r,c] != 2)
    img[depth_only] = DEPTH_OBSTACLE_COLOR
    if path:
        pts = np.array([overlay_point(x, y) for x, y in path], np.int32)
        cv2.polylines(img, [pts], False, (230,140,40), 2)
    if pose:
        x,y,yaw = pose; center = overlay_point(x, y)
        touching = node.view.get('nearest', 99) < node.body_radius + .02
        outline = (40,40,235) if touching else (255,255,255)
        # A filled, asymmetric body marker makes rotation visible even when
        # the robot has barely translated.  The long nose points along SLAM's
        # world-frame yaw; the map itself remains fixed in the initial frame.
        nose = overlay_point(x+.42*math.cos(yaw), y+.42*math.sin(yaw))
        left = overlay_point(x+.22*math.cos(yaw+2.35), y+.22*math.sin(yaw+2.35))
        right = overlay_point(x+.22*math.cos(yaw-2.35), y+.22*math.sin(yaw-2.35))
        marker = np.array([nose, left, right], np.int32)
        cv2.fillConvexPoly(img, marker, (210,90,20))
        cv2.polylines(img, [marker], True, outline, 2)
        cv2.circle(img, center, max(1, int(round(node.body_radius * scale))),
                   (30, 210, 230), 1, cv2.LINE_AA)
        cv2.circle(img, center, 3, outline, -1)
        heading = 'Pose %.2f, %.2f m   heading %.1f deg' % (
            x, y, math.degrees(yaw))
        cv2.rectangle(img, (5, 5), (365, 28), (45,45,45), -1)
        cv2.putText(img, heading, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, (255,255,255), 1, cv2.LINE_AA)
    with node.object_lock:
        objects = node.object_tracker.snapshot(False)
    for obj in objects:
        point = overlay_point(obj['x'], obj['y'])
        if 0 <= point[0] < MAP_SIZE and 0 <= point[1] < MAP_SIZE:
            confirmed = obj.get('confirmed', True)
            color = CONFIRMED_OBJECT_COLOR if confirmed else PENDING_OBJECT_COLOR
            cv2.circle(img, point, 6, color, -1 if confirmed else 2)
            label = '%s%s #%d' % ('' if confirmed else '? ',
                                  obj.get('name', obj['label']), obj['id'])
            cv2.putText(img, label, (point[0]+8, point[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1)
    requested = getattr(node, 'user_goal', None)
    actual = getattr(node, 'nav_goal', None)
    if requested is not None:
        cv2.drawMarker(img, overlay_point(requested[0], requested[1]),
                       (210, 0, 210), cv2.MARKER_TILTED_CROSS, 16, 2)
    if actual is not None:
        cv2.drawMarker(img, overlay_point(actual[0], actual[1]),
                       (255, 200, 20), cv2.MARKER_CROSS, 18, 2)
    cv2.line(img, (10, MAP_SIZE-12), (60, MAP_SIZE-12), (0,0,0), 3)
    cv2.putText(img, '1 m', (12, MAP_SIZE-20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
    return img


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    # SO_REUSEADDR has different semantics on Windows and can let two servers
    # bind the same port. Use an exclusive bind there so fallback works.
    allow_reuse_address = not hasattr(socket, 'SO_EXCLUSIVEADDRUSE')

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def start(node, port=8000):
    recording_dir = getattr(node, 'video_recording_dir', None)
    recording_fps = getattr(node, 'video_recording_fps', 10.0)
    recorder = YoloVideoRecorder(node, recording_dir, recording_fps)
    node.video_recorder = recorder

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, body, content_type='text/plain; charset=utf-8', status=200,
                  headers=None):
            if isinstance(body, str): body = body.encode()
            self.send_response(status); self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body))); self.send_header('Cache-Control', 'no-store')
            for name, value in (headers or {}).items():
                self.send_header(name, str(value))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                # Browsers may cancel an old JPEG request when a refresh starts.
                pass
        def send_file(self, path):
            try:
                size = os.path.getsize(path)
                filename = os.path.basename(path)
                content_type = ('video/mp4' if filename.lower().endswith('.mp4')
                                else 'video/x-msvideo')
                recording = open(path, 'rb')
            except OSError:
                self.send_error(404)
                return
            try:
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(size))
                self.send_header('Content-Disposition',
                                 'attachment; filename="%s"' % filename)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                with recording:
                    while True:
                        chunk = recording.read(1024 * 1024)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass
        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/': self.reply(PAGE, 'text/html; charset=utf-8')
            elif path == '/cam.jpg': self.reply(jpeg(draw_camera(node)), 'image/jpeg')
            elif path == '/depth.jpg':
                self.reply(jpeg(draw_depth_camera(node)), 'image/jpeg')
            elif path == '/overlay.jpg':
                try:
                    opacity = float(parse_qs(urlparse(self.path).query).get(
                        'opacity', ['0.45'])[0])
                    image = draw_depth_overlay(node, opacity)
                except (TypeError, ValueError):
                    self.reply('Opacity must be between 0 and 1', status=400)
                    return
                self.reply(jpeg(image), 'image/jpeg')
            elif path == '/map.jpg':
                values = parse_qs(urlparse(self.path).query)
                try:
                    scale = float(values.get('scale', [MAP_SCALE])[0])
                    dx = float(values.get('dx', [0.0])[0])
                    dy = float(values.get('dy', [0.0])[0])
                except (TypeError, ValueError):
                    self.reply('Invalid map view', status=400); return
                if not (15.0 <= scale <= 180.0 and math.isfinite(dx) and
                        math.isfinite(dy) and abs(dx) <= 50.0 and abs(dy) <= 50.0):
                    self.reply('Invalid map view', status=400); return
                base_cx, base_cy = map_view_center(node)
                cx, cy = base_cx + dx, base_cy + dy
                headers = {'X-Map-Center-X': '%.9f' % cx,
                           'X-Map-Center-Y': '%.9f' % cy,
                           'X-Map-Scale': '%.9f' % scale}
                self.reply(jpeg(draw_map(node, scale, center=(cx, cy)), 85),
                           'image/jpeg', headers=headers)
            elif path == '/status': self.reply(node.web_status())
            elif path == '/battery.json':
                self.reply(json.dumps(node.web_battery()), 'application/json')
            elif path == '/speed.json':
                self.reply(json.dumps(node.web_speed()), 'application/json')
            elif path == '/recording.json':
                self.reply(json.dumps(recorder.status()), 'application/json')
            elif path == '/recording/download':
                recording_path = recorder.completed_path()
                if recording_path is None:
                    self.reply('No completed recording is available', status=404)
                else:
                    self.send_file(recording_path)
            elif path == '/objects.json':
                with node.object_lock:
                    objects = node.object_tracker.snapshot(False)
                    for item in objects:
                        item.setdefault(
                            'confirmed', item.get('count', 0) >=
                            node.object_tracker.min_observations)
                self.reply(json.dumps(objects), 'application/json')
            else: self.send_error(404)
        def do_POST(self):
            u = urlparse(self.path)
            if u.path in ('/pause','/resume','/clear-map','/cancel-route'):
                command = {'/clear-map': 'clear_map', '/cancel-route': 'cancel_route'}.get(
                    u.path, u.path[1:])
                node.web_request((command,)); self.reply('Got it: ' + command)
            elif u.path == '/recover-camera':
                if not node.paused:
                    self.reply('Pause autonomy before recovering the camera', status=409)
                    return
                _, message, status = recover_camera()
                self.reply(message, status=status)
            elif u.path in ('/recording/start', '/recording/stop'):
                try:
                    message = (recorder.start() if u.path.endswith('/start')
                               else recorder.stop())
                except RuntimeError as exc:
                    self.reply(str(exc), status=409)
                    return
                except (OSError, ValueError) as exc:
                    self.reply('Video recording failed: %s' % exc, status=500)
                    return
                self.reply(message)
            elif u.path == '/manual':
                command = parse_qs(u.query).get('command', [''])[0]
                if command not in ('forward', 'back', 'strafe_left', 'strafe_right',
                                    'left', 'right', 'stop'):
                    self.reply('Invalid manual command', status=400)
                    return
                node.web_request(('manual', command))
                self.reply('Manual: ' + command)
            elif u.path == '/speed':
                values = parse_qs(u.query)
                try:
                    translation = float(values['translation'][0])
                    rotation = float(values['rotation'][0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid speed', status=400); return
                speed = node.web_speed()
                if (not math.isfinite(translation) or not math.isfinite(rotation) or
                        not speed['minimum'] <= translation <= speed['maximum'] or
                        not speed['minimum'] <= rotation <= speed['maximum']):
                    self.reply('Speed must be between %.0f%% and %.0f%%' % (
                        speed['minimum'], speed['maximum']), status=400); return
                node.web_request(('speed', translation, rotation))
                self.reply('Speed: translation %.0f%%, rotation %.0f%%' % (
                    translation, rotation))
            elif u.path == '/navigate':
                values = parse_qs(u.query)
                try:
                    px, py = float(values['px'][0]), float(values['py'][0])
                    scale = float(values.get('scale', [MAP_SCALE])[0])
                    supplied_center = 'cx' in values or 'cy' in values
                    if supplied_center:
                        cx = float(values['cx'][0])
                        cy = float(values['cy'][0])
                    else:
                        dx = float(values.get('dx', [0.0])[0])
                        dy = float(values.get('dy', [0.0])[0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid map position', status=400); return
                if not (math.isfinite(px) and math.isfinite(py) and
                        0 <= px <= MAP_SIZE and 0 <= py <= MAP_SIZE and
                         15.0 <= scale <= 180.0 and
                         (not supplied_center or (math.isfinite(cx) and math.isfinite(cy) and
                                                  abs(cx) <= 1000.0 and abs(cy) <= 1000.0)) and
                         (supplied_center or (math.isfinite(dx) and math.isfinite(dy) and
                                              abs(dx) <= 50.0 and abs(dy) <= 50.0))):
                    self.reply('Map position is outside the viewport', status=400); return
                if not supplied_center:
                    cx, cy = map_view_center(node)
                    cx, cy = cx + dx, cy + dy
                x = cx + (MAP_SIZE/2.0 - py) / scale
                y = cy + (MAP_SIZE/2.0 - px) / scale
                node.web_request(('navigate', x, y))
                self.reply('Navigation requested: %.2f, %.2f m' % (x, y))
            elif u.path in ('/object/navigate', '/object/remove', '/object/rename'):
                values = parse_qs(u.query)
                try:
                    object_id = int(values['id'][0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid object id', status=400); return
                if object_id < 1:
                    self.reply('Invalid object id', status=400); return
                if u.path == '/object/navigate':
                    node.web_request(('navigate_object', object_id))
                    self.reply('Object navigation requested')
                elif u.path == '/object/remove':
                    node.web_request(('remove_object', object_id))
                    self.reply('Object removal requested')
                else:
                    name = values.get('name', [''])[0].strip()
                    if not name or len(name) > 40 or any(ord(ch) < 32 for ch in name):
                        self.reply('Name must be 1-40 printable characters', status=400); return
                    node.web_request(('rename_object', object_id, name))
                    self.reply('Object rename requested')
            else: self.send_error(404)
    if not isinstance(port, int) or not 0 <= port <= 65535:
        raise ValueError('port must be an integer between 0 and 65535')
    ports = (0,) if port == 0 else range(port, min(port + 10, 65536))
    for candidate in ports:
        try: server = Server(('0.0.0.0', candidate), Handler)
        except OSError: continue
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server.server_address[1]
    return None
