#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机标定工具 - 大恒相机"""
import cv2
import numpy as np
import glob
import argparse
import os
import time
import threading
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler

import gxipy as gx

CAM = None
DM = None

def init_camera(exp=30000, gain=10):
    global CAM, DM
    try:
        gx.gx_init_lib()
        DM = gx.DeviceManager()
        n, _ = DM.update_device_list()
        if n == 0:
            print("错误: 未找到相机")
            return False
        print(f"找到 {n} 个相机")
        CAM = DM.open_device_by_index(1)
        CAM.TriggerMode.set(gx.GxSwitchEntry.OFF)
        CAM.ExposureTime.set(exp)
        CAM.Gain.set(gain)
        print(f"曝光: {exp}us  增益: {gain}dB")
        try:
            CAM.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)
            print("自动白平衡: 开启")
        except:
            pass
        CAM.stream_on()
        print("数据流: 已启动")
        time.sleep(1.5)
        print("相机就绪")
        return True
    except Exception as e:
        print(f"初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def read_frame():
    global CAM
    if CAM is None:
        return None
    try:
        raw = CAM.data_stream[0].get_image(timeout=2000)
        if raw is None:
            return None
        rgb = raw.convert("RGB")
        if rgb is None:
            return None
        arr = rgb.get_numpy_array()
        if arr is None:
            return None
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    except:
        return None

def close_camera():
    global CAM
    if CAM:
        try:
            CAM.stream_off()
            CAM.close_device()
        except:
            pass
        CAM = None

def set_exposure(v):
    global CAM
    if CAM:
        try:
            CAM.ExposureTime.set(v)
        except:
            pass

def set_gain(v):
    global CAM
    if CAM:
        try:
            CAM.Gain.set(v)
        except:
            pass

class Calib:
    def __init__(self, sz=(9,6), sq=1.0):
        self.sz = sz
        self.objp = np.zeros((sz[0]*sz[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:sz[0], 0:sz[1]].T.reshape(-1, 2) * sq
        self.obj_pts = []
        self.img_pts = []
        self.img_sz = None
    
    def find(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.img_sz = gray.shape[::-1]
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, self.sz, flags)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            vis = img.copy()
            cv2.drawChessboardCorners(vis, self.sz, corners, ret)
            return corners, vis
        return None, img.copy()
    
    def add(self, corners):
        self.obj_pts.append(self.objp)
        self.img_pts.append(corners)
    
    def calibrate(self):
        if len(self.obj_pts) < 3:
            return None, None, None
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_pts, self.img_pts, self.img_sz, None, None
        )
        total_err = 0
        for i in range(len(self.obj_pts)):
            proj, _ = cv2.projectPoints(self.obj_pts[i], rvecs[i], tvecs[i], mtx, dist)
            err = cv2.norm(self.img_pts[i], proj, cv2.NORM_L2) / len(proj)
            total_err += err
        mean_err = total_err / len(self.obj_pts)
        print(f"\n标定完成! 重投影误差: {mean_err:.4f} 像素")
        print(f"相机内参:\n{mtx}")
        print(f"畸变系数: {dist.flatten()}")
        return mtx, dist, mean_err
    
    def save(self, mtx, dist, path):
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        fs.write("image_width", self.img_sz[0])
        fs.write("image_height", self.img_sz[1])
        fs.write("camera_matrix", mtx)
        fs.write("distortion_coefficients", dist)
        fs.release()
        print(f"已保存到: {path}")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--board-cols", type=int, default=7)
    p.add_argument("--board-rows", type=int, default=10)
    p.add_argument("--square-size", type=float, default=1.775)
    p.add_argument("--exposure", type=float, default=30000)
    p.add_argument("--gain", type=float, default=10.0)
    p.add_argument("--images", type=str)
    p.add_argument("--auto", action="store_true")
    p.add_argument("--interval", type=float, default=2.0)
    p.add_argument("--count", type=int, default=100)
    p.add_argument("--save-dir", type=str, default="./calib_images")
    p.add_argument("--web-port", type=int, default=8080)
    p.add_argument("--output", type=str, default="camera_calibration.yaml")
    a = p.parse_args()
    
    if a.images:
        cal = Calib((a.board_cols, a.board_rows), a.square_size)
        imgs = sorted(glob.glob(a.images))
        if not imgs:
            print(f"未找到: {a.images}")
            return
        print(f"\n找到 {len(imgs)} 张图像, 棋盘格: {a.board_cols}x{a.board_rows}\n")
        valid = 0
        for im in imgs:
            img = cv2.imread(im)
            if img is None:
                continue
            c, _ = cal.find(img)
            if c is not None:
                cal.add(c)
                valid += 1
                print(f"  OK {os.path.basename(im)}")
            else:
                print(f"  X  {os.path.basename(im)}")
        print(f"\n有效: {valid}/{len(imgs)}")
        if valid >= 3:
            mtx, dist, _ = cal.calibrate()
            if mtx is not None:
                cal.save(mtx, dist, a.output)
        else:
            print("有效图像不足 (至少需要3张)")
    
    elif a.auto:
        print("正在初始化相机...")
        if not init_camera(a.exposure, a.gain):
            print("相机打开失败!")
            return
        
        cal = Calib((a.board_cols, a.board_rows), a.square_size)
        os.makedirs(a.save_dir, exist_ok=True)
        
        state = {"frame": None, "detected": False, "captured": 0, "valid": 0, "reads": 0}
        lock = threading.Lock()
        last_capture = time.time()
        
        class Handler(SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-type", "text/html; charset=utf-8")
                    self.end_headers()
                    html = '''<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>相机标定</title>
<style>
body{font-family:Arial;background:#1a1a1a;color:white;text-align:center;margin:20px}
img{max-width:90%;border:2px solid #444}
.st{font-size:24px;margin:20px;padding:15px;background:#333;border-radius:10px}
.ok{color:#4CAF50} .no{color:#f44336}
.ctrl{margin:20px}
.ctrl input{width:100px;padding:5px}
.ctrl button{padding:10px 20px;margin:5px;cursor:pointer}
</style></head><body>
<h1>相机标定工具</h1>
<div class="st" id="st">加载中...</div>
<div class="ctrl">
<label>曝光:<input type="number" id="exp" value="''' + str(int(a.exposure)) + '''" step="5000"></label>
<label>增益:<input type="number" id="gn" value="''' + str(int(a.gain)) + '''" step="1"></label>
<button onclick="fetch('/set?e='+document.getElementById('exp').value+'&g='+document.getElementById('gn').value)">更新</button>
</div>
<img id="f" src="/frame.jpg"/>
<p>请将棋盘格移动到不同位置和角度</p>
<script>
setInterval(function(){
  document.getElementById('f').src='/frame.jpg?'+Date.now();
  fetch('/st').then(r=>r.json()).then(d=>{
    var e=document.getElementById('st');
    e.className='st '+(d.d?'ok':'no');
    e.innerHTML=(d.d?'检测到棋盘格':'未检测到棋盘格')+'<br>已拍:'+d.c+' 有效:'+d.v+'/''' + str(a.count) + ''' 帧:'+d.r;
  });
},500);
</script></body></html>'''
                    self.wfile.write(html.encode('utf-8'))
                elif self.path.startswith("/frame"):
                    self.send_response(200)
                    self.send_header("Content-type", "image/jpeg")
                    self.end_headers()
                    with lock:
                        if state["frame"] is not None:
                            _, buf = cv2.imencode(".jpg", state["frame"], [cv2.IMWRITE_JPEG_QUALITY, 80])
                            self.wfile.write(buf.tobytes())
                elif self.path == "/st":
                    self.send_response(200)
                    self.send_header("Content-type", "application/json")
                    self.end_headers()
                    with lock:
                        self.wfile.write(json.dumps({
                            "d": state["detected"], "c": state["captured"],
                            "v": state["valid"], "r": state["reads"]
                        }).encode())
                elif self.path.startswith("/set"):
                    from urllib.parse import urlparse, parse_qs
                    q = parse_qs(urlparse(self.path).query)
                    if "e" in q:
                        set_exposure(float(q["e"][0]))
                        print(f"曝光: {q['e'][0]}us")
                    if "g" in q:
                        set_gain(float(q["g"][0]))
                        print(f"增益: {q['g'][0]}dB")
                    self.send_response(200)
                    self.end_headers()
                else:
                    self.send_error(404)
            def log_message(self, *args):
                pass
        
        server = HTTPServer(("0.0.0.0", a.web_port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        
        print(f"\n{'='*50}")
        print(f"  棋盘格: {a.board_cols}x{a.board_rows} 内角点")
        print(f"  间隔: {a.interval}秒  目标: {a.count}张")
        print(f"  网页: http://localhost:{a.web_port}")
        print(f"{'='*50}")
        print(f"  Ctrl+C 停止\n")
        
        captured, valid, reads = 0, 0, 0
        
        try:
            while captured < a.count:
                frame = read_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                
                reads += 1
                corners, display = cal.find(frame)
                detected = corners is not None
                
                color = (0, 255, 0) if detected else (0, 0, 255)
                cv2.putText(display, "FOUND" if detected else "Not found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                cv2.putText(display, f"C:{captured}/{a.count} V:{valid}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                with lock:
                    state["frame"] = display
                    state["detected"] = detected
                    state["captured"] = captured
                    state["valid"] = valid
                    state["reads"] = reads
                
                if time.time() - last_capture >= a.interval:
                    captured += 1
                    last_capture = time.time()
                    filename = f"calib_{captured:03d}.jpg"
                    cv2.imwrite(os.path.join(a.save_dir, filename), frame)
                    if detected:
                        cal.add(corners)
                        valid += 1
                        print(f"[{captured}/{a.count}] OK {filename}")
                    else:
                        print(f"[{captured}/{a.count}] X  {filename}")
                
                time.sleep(0.03)
        
        except KeyboardInterrupt:
            print("\n用户中断")
        
        close_camera()
        server.shutdown()
        print(f"\n完成! 共{captured}张, 有效{valid}张")
        if valid >= 3:
            print(f"运行标定: python3 calibrate_camera.py --images '{a.save_dir}/*.jpg'")
    
    else:
        print("用法:")
        print("  采集: python3 calibrate_camera.py --auto")
        print("  标定: python3 calibrate_camera.py --images './calib_images/*.jpg'")

if __name__ == "__main__":
    main()
