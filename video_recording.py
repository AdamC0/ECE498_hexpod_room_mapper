"""Background video recording independent of the HTTP dashboard server."""

import os
import shutil
import threading
import time

import cv2


class VideoRecorder(object):
    """Write timestamped rendered frames without blocking camera callbacks."""

    FRAME_SIZE = (640, 480)
    MINIMUM_FREE_BYTES = 256 * 1024 * 1024
    CODECS = (('mp4v', '.mp4'), ('MJPG', '.avi'))

    def __init__(self, render_frame, camera_ready, output_dir, fps=10.0):
        self.render_frame = render_frame
        self.camera_ready = camera_ready
        self.output_dir = output_dir
        self.fps = float(fps)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.writer = None
        self.current_path = None
        self.started_at = None
        self.frames = 0
        self.last_completed_path = None
        self.last_error = ''
        self._find_latest_completed()

    def _find_latest_completed(self):
        try:
            candidates = [os.path.join(self.output_dir, name)
                          for name in os.listdir(self.output_dir)
                          if name.lower().endswith(('.mp4', '.avi'))]
            candidates = [path for path in candidates
                          if os.path.isfile(path) and os.path.getsize(path) > 0]
            if candidates:
                self.last_completed_path = max(candidates, key=os.path.getmtime)
        except OSError:
            pass

    def _open_writer(self, stem):
        for codec, extension in self.CODECS:
            path = os.path.join(self.output_dir, stem + extension)
            writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*codec), self.fps, self.FRAME_SIZE)
            if writer.isOpened():
                return writer, path
            writer.release()
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except OSError:
                pass
        raise RuntimeError('No supported MP4 or AVI video encoder is available')

    def start(self):
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError('A video recording is already active')
            if not self.camera_ready():
                raise RuntimeError('RGB camera is not ready')
            os.makedirs(self.output_dir, exist_ok=True)
            if shutil.disk_usage(self.output_dir).free < self.MINIMUM_FREE_BYTES:
                raise RuntimeError('Less than 256 MB is free for video recording')
            stamp = time.time()
            base_stem = 'yolo_%s_%03d' % (
                time.strftime('%Y%m%d_%H%M%S', time.localtime(stamp)),
                int((stamp % 1.0) * 1000))
            stem = base_stem
            suffix = 1
            while any(os.path.exists(os.path.join(self.output_dir, stem + extension))
                      for _, extension in self.CODECS):
                stem = '%s_%d' % (base_stem, suffix)
                suffix += 1
            writer, path = self._open_writer(stem)
            self.stop_event.clear()
            self.writer = writer
            self.current_path = path
            self.started_at = time.time()
            self.frames = 0
            self.last_error = ''
            self.thread = threading.Thread(
                target=self._record_loop, args=(writer, path), daemon=True)
            self.thread.start()
            return 'Recording started: ' + os.path.basename(path)

    def _record_loop(self, writer, path):
        interval = 1.0 / max(1.0, self.fps)
        deadline = time.monotonic()
        error = ''
        try:
            while not self.stop_event.is_set():
                writer.write(self.render_frame())
                with self.lock:
                    self.frames += 1
                deadline += interval
                self.stop_event.wait(max(0.0, deadline - time.monotonic()))
        except Exception as exc:
            error = 'Video recording failed: %s' % exc
        finally:
            writer.release()
            completed = False
            try:
                completed = os.path.isfile(path) and os.path.getsize(path) > 0
            except OSError:
                pass
            with self.lock:
                if self.writer is writer:
                    self.writer = None
                    self.current_path = None
                    if completed and not error:
                        self.last_completed_path = path
                    self.last_error = error

    def stop(self):
        with self.lock:
            thread = self.thread
            active = thread is not None and thread.is_alive()
            if not active:
                raise RuntimeError('No video recording is active')
            self.stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=5.0)
        if thread.is_alive():
            raise RuntimeError('Video recorder did not stop within five seconds')
        with self.lock:
            path = self.last_completed_path
            error = self.last_error
        if error:
            raise RuntimeError(error)
        return 'Recording stopped: ' + (os.path.basename(path) if path else 'no video saved')

    def close(self):
        with self.lock:
            active = self.thread is not None and self.thread.is_alive()
            self.stop_event.set()
            thread = self.thread
        if active and thread is not threading.current_thread():
            thread.join(timeout=5.0)

    def status(self):
        with self.lock:
            recording = self.thread is not None and self.thread.is_alive()
            started_at = self.started_at
            frames = self.frames
            path = self.current_path if recording else self.last_completed_path
            error = self.last_error
        exists = bool(path and os.path.isfile(path))
        size = os.path.getsize(path) if exists else 0
        return {
            'recording': recording,
            'available': bool(not recording and exists and size > 0),
            'filename': os.path.basename(path) if path else None,
            'bytes': size,
            'frames': frames if recording else 0,
            'started_at': started_at if recording else None,
            'duration': max(0.0, time.time() - started_at) if recording and started_at else 0.0,
            'error': error,
        }

    def completed_path(self):
        with self.lock:
            path = self.last_completed_path
            recording = self.thread is not None and self.thread.is_alive()
        if recording or not path or not os.path.isfile(path):
            return None
        return path
