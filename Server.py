import numpy as np
import cv2
from typing import Set, List
from fastapi import FastAPI, WebSocket, Request, WebSocketDisconnect
from fastapi.responses import StreamingResponse, HTMLResponse
import uvicorn
from fastapi.templating import Jinja2Templates
import io
import time
import asyncio

app = FastAPI()
templates = Jinja2Templates(directory="templates")

frame = None
slope_to_center = 0.0

class FrameProcess:
    def __init__(self):
        cv2.setUseOptimized(True)
        
    def set_roi(self, rec_frame:np.ndarray)->tuple:
        height, width = rec_frame.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        roi = np.array([[
            (int(width*-0.08), int(height*0.6)),
            (int(width*0.0), int(height*0.38)),
            (int(width*0.95), int(height*0.38)),
            (int(width*1.1), int(height*0.6))
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi, 255)
        masked_frame = cv2.bitwise_and(rec_frame, rec_frame, mask=mask)
        return masked_frame, roi        
        
    def perspective_transform(self, rec_frame, roi):
        height, width = rec_frame.shape[:2]
        src_points = np.float32(roi)
        dst_points = np.float32([
            [width * 0, height * 0.7],
            [width * 0, height * 0.3],
            [width * 1, height * 0.3],
            [width * 1, height * 0.7]
        ])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warp_perspective = cv2.warpPerspective(rec_frame, matrix, (width, height))
        return warp_perspective

    def image_transform(self, rec_frame):
        clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(8, 8))
        lab_frame = cv2.cvtColor(rec_frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab_frame)
        clahe_l = clahe.apply(l)
        clahe_lab = cv2.merge([clahe_l, a, b])
        lower_yellow_lab = np.array([145, 0, 135])
        upper_yellow_lab = np.array([245, 130, 255])
        mask_yellow_lab = cv2.inRange(clahe_lab, lower_yellow_lab, upper_yellow_lab)
        
        blur_frame = cv2.GaussianBlur(mask_yellow_lab, (5, 5), 0)
        edges_frame = cv2.Canny(blur_frame, 100, 150)
        
        return edges_frame

    def hough_transform(self, rec_frame):
        hough_detect_frame = cv2.HoughLinesP(rec_frame, 1, np.pi/180, 50, minLineLength=50, maxLineGap=30)

        left_lines = []
        right_lines = []
        
        if hough_detect_frame is not None:
            for line in hough_detect_frame:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / (x2 - x1) 
                        
                if slope < 0:
                    left_lines.append(line)
                elif slope > 0:
                    right_lines.append(line)
                    
        return left_lines, right_lines
    
    def average_slope_intercept(self, both_lines):
        if both_lines is None or len(both_lines) == 0:
            return None, None        
        x_vals, y_vals, slopes = [], [], []
        for line in both_lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                slope = float('inf')
            else:
                slope = (y2 - y1) / (x2 - x1)
            
            slopes.append(slope)
            x_vals.extend([x1, x2])
            y_vals.extend([y1, y2])

        avg_slope = np.mean(slopes)
        avg_intercept = np.mean(y_vals) - avg_slope * np.mean(x_vals)
        
        if np.isinf(avg_slope):
            avg_intercept = np.mean(x_vals)
        
        return avg_slope, avg_intercept 
    
    def make_line_points(self, y1, y2, avg_slope, avg_intercept):
        if avg_slope is None or avg_intercept is None:
            return None
        
        if np.isinf(avg_slope):  
            if avg_slope != 0:
                x1 = x2 = int(avg_intercept)  
                
        elif 0 <= avg_slope <= 0.1:         
            x1 = x2 = int(avg_intercept) 
            
        elif -0.1 <= avg_slope <= 0:
            x1 = x2 = int(avg_intercept) 

        else:
            x1 = int((y1 - avg_intercept) / avg_slope)
            x2 = int((y2 - avg_intercept) / avg_slope)
        
        return [x1, y1, x2, y2]
    

    def calculate_center_slope(self, left_line_points, right_line_points): 
        global slope_to_center
        if left_line_points is None and right_line_points is None:
            print("No lines detected")
            return None        
        
        if left_line_points is not None and right_line_points is not None:
            left_x1, left_y1, left_x2, left_y2 = left_line_points
            right_x1, right_y1, right_x2, right_y2 = right_line_points

            center_x1 = (left_x1 + right_x1) // 2
            center_x2 = (left_x2 + right_x2) // 2
            center_y1 = (left_y1 + right_y1) // 2
            center_y2 = (left_y2 + right_y2) // 2     
            
            if center_x2 - center_x1 != 0:
                slope_to_center = (center_y2 - center_y1) / (center_x2 - center_x1)
            else:
                slope_to_center = float('inf')

            return slope_to_center    
            
        elif left_line_points is not None:
            left_x1, left_y1, left_x2, left_y2 = left_line_points
            center_x1 = left_x1
            center_x2 = left_x2
            center_y1 = left_y1
            center_y2 = left_y2

            if center_x2 - center_x1 != 0:
                slope_to_center = (center_y2 - center_y1) / (center_x2 - center_x1)
            else:
                slope_to_center = float('inf')

            return slope_to_center

        elif right_line_points is not None:
            right_x1, right_y1, right_x2, right_y2 = right_line_points
            center_x1 = right_x1
            center_x2 = right_x2
            center_y1 = right_y1
            center_y2 = right_y2

            if center_x2 - center_x1 != 0:
                
                slope_to_center = (center_y2 - center_y1) / (center_x2 - center_x1)
            else:
                slope_to_center = float('inf')

            return slope_to_center
            

    def process_frame(self, rec_frame):
        set_roi_frame, roi = self.set_roi(rec_frame)  
        warp_perspective_frame = self.perspective_transform(set_roi_frame, roi)   
        image_transform_frame = self.image_transform(warp_perspective_frame)
        left_lines, right_lines = self.hough_transform(image_transform_frame)

        frame_copy = rec_frame.copy()

        left_avg_slope, left_avg_intercept = self.average_slope_intercept(left_lines)
        right_avg_slope, right_avg_intercept = self.average_slope_intercept(right_lines)

        roi_y_coords = []
        for point in roi[0]:
                roi_y_coords.append(point[1])
        p_y1 = max(roi_y_coords)
        p_y2 = min(roi_y_coords)

        left_line_points = self.make_line_points(p_y1, p_y2, left_avg_slope, left_avg_intercept)
        right_line_points = self.make_line_points(p_y1, p_y2, right_avg_slope, right_avg_intercept)
                     
        if left_line_points is not None:
            x1, y1, x2, y2 = left_line_points
            cv2.line(frame_copy, (x1, y1), (x2, y2), (255, 0, 0), 3)

        if right_line_points is not None:
            x1, y1, x2, y2 = right_line_points
            cv2.line(frame_copy, (x1, y1), (x2, y2), (0, 255, 0), 3)

        
        slope_to_center = self.calculate_center_slope(left_line_points, right_line_points)
        
        return set_roi_frame
    

fprocess = FrameProcess()

timer = 0.0
pre_command = 0b10100000
backward_count = 0

async def handle_command(message):
    global slope_to_center
    global timer
    global pre_command
    global backward_count
    try:
        if slope_to_center is not None and timer == 0.0:
            timer = time.monotonic()
            if -3.5 >= slope_to_center or slope_to_center >= 3.5:
                command = 0b00100000
                print("dir : Forward")
            elif slope_to_center < 3.5 and slope_to_center > 0:
                command = 0b11000000
                print("dir : Left")
            elif slope_to_center > -3.5 and slope_to_center < 0:
                command = 0b11100000
                print("dir : Right")
            else:
                command = 0b10100000
                print("dir : Stop")
                
            if command == 0b11000000 or command == 0b11100000:
                speed = int(31 * ((1 - (slope_to_center / 5) ** 2) ** 2))
                command = command | int(speed)        

            pre_command = command
            backward_count = 0
            
        else:
            if timer != 0.0:
                if time.monotonic() - timer >= 0.0 and time.monotonic() - timer < 0.2:
                    command = pre_command
                elif time.monotonic() - timer >= 0.2 and time.monotonic() - timer < 0.3:
                    command = 0b10100000
                    print("dir : Stop2")
                else:
                    command = 0b10100000
                    print("dir : Stop22")                
                    timer = 0.0
            else: 
                command = 0b01000000
                print("dir : backward2")
                backward_count += 1
                pre_command = command
                timer = time.monotonic()
            
        print(f"print command1 : {command}")
        print(f"slope_center : {slope_to_center}")
        slope_to_center = None
            
        command_bytes = command.to_bytes(1, byteorder='little')
        return command_bytes

    except Exception as e:
        print(f"Error in handle_command: {e}")
        return b'\x00'  

@app.websocket("/ws/esp32")
async def websocket_endpoint_esp32(websocket: WebSocket):
    await websocket.accept()
    global frame, slope_to_center
    try:
        while True:
            message = await websocket.receive_bytes()
            frame = cv2.imdecode(np.frombuffer(message, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            frame = fprocess.process_frame(frame) 
                
            command_bytes = await handle_command(slope_to_center)
            await websocket.send_bytes(command_bytes)
            await asyncio.sleep(0.08) 
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")


@app.get("/")
async def get_index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/video_feed")
async def video_feed():
    global frame
    if frame is not None:
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = buffer.tobytes()
        print("Streaming video frame")
        return StreamingResponse(io.BytesIO(jpg_as_text), media_type="image/jpeg")
    else:
        print("Failed to encode image")
        return HTMLResponse(content="Failed to encode image", status_code=500)
  
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=)