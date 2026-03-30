import cv2
import numpy as np
import heapq
import time
import math
import random
import datetime
from collections import deque

# ระบุพาทไฟล์รูปภาพของคุณ
image = cv2.imread(r"/home/isaac/fra532-lab3-planning-por-wa/images/filtered_kmutt.png")

scale_percent = 100
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
resized_image = cv2.resize(image, (width, height))

# =========================
# GLOBAL
# =========================
use_gvd = True
algorithm = "astar"  # "astar", "best_first", "bfs"
only_beige = False   
is_running = False   # ป้องกันการคลิกซ้อนขณะวาดเส้นแอนิเมชัน

# ✨ กำหนดพิกัดเริ่มต้นตามที่ระบุ
start_point = (1241, 657)
end_point = (486, 785)
all_paths = [] 

# ✨ กำหนดประตูเริ่มต้นตามที่ระบุ
doors = [(1004, 800), (1163, 765)]  
door_map = np.zeros((height, width), dtype=np.uint8)

# วาดประตูเริ่มต้นลงใน door_map
for d in doors:
    cv2.circle(door_map, d, 5, 255, -1)

# ค่า Penalty เริ่มต้น
crosspenalty = 1
buildingpenalty = 1
doorpenalty = 10

# ขยับพิกัด UI สำหรับหน้าต่าง Control Panel
checkbox_pos = (20, 30)
astar_checkbox_pos = (20, 70)
best_first_checkbox_pos = (20, 110) 
bfs_checkbox_pos = (20, 150)
beige_checkbox_pos = (20, 190)

# ตำแหน่งปุ่ม 4 ปุ่ม
run_btn_pos = (20, 240)         
clear_paths_btn_pos = (20, 290) 
clear_poses_btn_pos = (20, 340) 
clear_doors_btn_pos = (20, 390) 
checkbox_size = 20

# =====================================================================
def get_pixel_type(color):
    b, g, r = int(color[0]), int(color[1]), int(color[2])
    
    if abs(b - 213) < 15 and abs(g - 232) < 15 and abs(r - 242) < 15:
        return "BEIGE"
    if abs(b - 180) < 20 and abs(g - 105) < 20 and abs(r - 255) < 20:
        return "PINK"
        
    # สีตึกส้ม RGB(226, 95, 43) -> BGR(43, 95, 226)
    if abs(b - 43) < 20 and abs(g - 95) < 20 and abs(r - 226) < 20:
        return "BUILDING"
    # สีตึกเหลือง RGB(211, 193, 49) -> BGR(49, 193, 211)
    if abs(b - 49) < 20 and abs(g - 193) < 20 and abs(r - 211) < 20:
        return "BUILDING"
        
    return "WALL"

# =====================================================================
def compute_distance_map(img):
    rows, cols = img.shape[:2]
    binary = np.zeros((rows, cols), dtype=np.uint8)
    for y in range(rows):
        for x in range(cols):
            if get_pixel_type(img[y, x]) != "WALL":
                binary[y, x] = 255
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    dist = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
    return dist

def compute_path_distance(path):
    if path is None or len(path) < 2: return 0
    distance = 0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        distance += 1.414 if (x1 != x2 and y1 != y2) else 1
    return distance

def find_path_astar(img, start, end, distance_map, use_gvd, only_beige, d_map):
    rows, cols = img.shape[:2]
    open_set = []
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK", "BUILDING"] else "BEIGE"
    
    heapq.heappush(open_set, (0, 0, start[0], start[1], 0, start_safe_color))
    came_from = {}
    g_score = {(start[0], start[1], 0, start_safe_color): 0}
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        iterations += 1
        f, g, cx, cy, wall_count, last_safe_color = heapq.heappop(open_set)

        if (cx, cy) == end:
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            path = path[::-1]
            return path, g, iterations, (time.time() - start_time) * 1000, compute_path_distance(path)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows): continue
            
            pixel_type = get_pixel_type(img[ny, nx])

            curr_zone = "INSIDE" if last_safe_color == "BUILDING" else "OUTSIDE"
            if pixel_type == "BUILDING": next_zone = "INSIDE"
            elif pixel_type in ["BEIGE", "PINK"]: next_zone = "OUTSIDE"
            else: next_zone = curr_zone 
            
            if curr_zone != next_zone:
                if not (d_map[cy, cx] or d_map[ny, nx]):
                    continue 

            penalty = 0
            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE" and not d_map[ny, nx]: 
                    continue
                if last_safe_color == "PINK": penalty += crosspenalty
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                if only_beige: 
                    continue 
                if last_safe_color == "BEIGE": penalty += crosspenalty
                next_wall, next_safe = 0, "PINK"
            elif pixel_type == "BUILDING":
                next_wall, next_safe = 0, "BUILDING"
                penalty += buildingpenalty
            else:
                next_wall = wall_count + 1
                if next_wall > 5: continue
                next_safe = last_safe_color
                penalty += 15

            step_cost = 1.414 if dx != 0 and dy != 0 else 1
            gvd_bonus = 0
            
            # จัดการประตูดึงดูด (Door Magnet)
            door_discount_g = 0
            if d_map[ny, nx]: 
                door_discount_g = -doorpenalty 

            # GVD 
            if use_gvd and pixel_type in ["BEIGE", "PINK"]:
                dist_value = distance_map[ny, nx]
                goal_dist = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                if goal_dist > 30: 
                    if not d_map[ny, nx]:
                        gvd_bonus = -0.8 * dist_value

            tentative_g = g + step_cost + penalty + gvd_bonus + door_discount_g
            tentative_g = max(g + 0.1, tentative_g)

            neighbor_state = (nx, ny, next_wall, next_safe)

            if neighbor_state not in g_score or tentative_g < g_score[neighbor_state]:
                came_from[neighbor_state] = (cx, cy, wall_count, last_safe_color)
                g_score[neighbor_state] = tentative_g
                
                h = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                door_discount_priority = -50.0 if d_map[ny, nx] else 0
                priority = tentative_g + h + door_discount_priority
                priority = max(0, priority)
                
                heapq.heappush(open_set, (priority, tentative_g, nx, ny, next_wall, next_safe))
                
    return None, 0, iterations, 0, 0

# =====================================================================
def find_path_best_first(img, start, end, distance_map, use_gvd, only_beige, d_map):
    rows, cols = img.shape[:2]
    open_set = []
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK", "BUILDING"] else "BEIGE"

    h_start = ((start[0] - end[0])**2 + (start[1] - end[1])**2)**0.5
    heapq.heappush(open_set, (h_start, start[0], start[1], 0, start_safe_color, 0))
    came_from = {}
    visited = set([(start[0], start[1], 0, start_safe_color)])
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        iterations += 1
        h, cx, cy, wall_count, last_safe_color, g = heapq.heappop(open_set)

        if (cx, cy) == end:
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            return path[::-1], g, iterations, (time.time() - start_time) * 1000, compute_path_distance(path)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows): continue
            
            pixel_type = get_pixel_type(img[ny, nx])

            curr_zone = "INSIDE" if last_safe_color == "BUILDING" else "OUTSIDE"
            if pixel_type == "BUILDING": next_zone = "INSIDE"
            elif pixel_type in ["BEIGE", "PINK"]: next_zone = "OUTSIDE"
            else: next_zone = curr_zone
            
            if curr_zone != next_zone:
                if not (d_map[cy, cx] or d_map[ny, nx]): continue

            penalty = 0
            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE" and not d_map[ny, nx]: 
                    continue
                if last_safe_color == "PINK": penalty += crosspenalty
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                if only_beige: 
                    continue 
                if last_safe_color == "BEIGE": penalty += crosspenalty
                next_wall, next_safe = 0, "PINK"
            elif pixel_type == "BUILDING":
                next_wall, next_safe = 0, "BUILDING"
                penalty += buildingpenalty
            else:
                next_wall = wall_count + 1
                if next_wall > 5: continue
                next_safe = last_safe_color
                penalty += 15

            step_cost = 1.414 if dx != 0 and dy != 0 else 1
            
            door_discount_g = 0
            door_discount_priority = 0
            if d_map[ny, nx]: 
                door_discount_g = -doorpenalty
                door_discount_priority = -50.0
                
            gvd_bonus_cost, gvd_bonus_priority = 0, 0
            if use_gvd and pixel_type in ["BEIGE", "PINK"]:
                dist_value = distance_map[ny, nx]
                goal_dist = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                if goal_dist > 30:
                    if not d_map[ny, nx]:
                        gvd_bonus_cost = -0.5 * dist_value
                        gvd_bonus_priority = -3.0 * dist_value 

            tentative_g = g + step_cost + penalty + gvd_bonus_cost + door_discount_g
            tentative_g = max(g + 0.1, tentative_g) 
            
            neighbor_state = (nx, ny, next_wall, next_safe)

            if neighbor_state not in visited:
                visited.add(neighbor_state)
                came_from[neighbor_state] = (cx, cy, wall_count, last_safe_color)
                h_next = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                priority = h_next + penalty + gvd_bonus_priority + door_discount_priority
                priority = max(0, priority) 
                
                heapq.heappush(open_set, (priority, nx, ny, next_wall, next_safe, tentative_g))

    return None, 0, iterations, 0, 0

# =====================================================================
def find_path_bfs(img, start, end, only_beige, d_map):
    rows, cols = img.shape[:2]
    queue = deque()
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK", "BUILDING"] else "BEIGE"

    start_state = (start[0], start[1], 0, start_safe_color)
    queue.append(start_state)
    came_from = {}
    visited = set([start_state])
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while queue:
        iterations += 1
        cx, cy, wall_count, last_safe_color = queue.popleft()

        if (cx, cy) == end:
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            return path[::-1], len(path), iterations, (time.time() - start_time) * 1000, compute_path_distance(path)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows): continue
            
            pixel_type = get_pixel_type(img[ny, nx])

            curr_zone = "INSIDE" if last_safe_color == "BUILDING" else "OUTSIDE"
            if pixel_type == "BUILDING": next_zone = "INSIDE"
            elif pixel_type in ["BEIGE", "PINK"]: next_zone = "OUTSIDE"
            else: next_zone = curr_zone
            
            if curr_zone != next_zone:
                if not (d_map[cy, cx] or d_map[ny, nx]): continue

            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE": continue
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                if only_beige: 
                    continue # บล็อคสีชมพูทันทีถ้าเปิด only_beige
                next_wall, next_safe = 0, "PINK"
            elif pixel_type == "BUILDING":
                next_wall, next_safe = 0, "BUILDING"
            else:
                next_wall = wall_count + 1
                if next_wall > 5: continue
                next_safe = last_safe_color

            neighbor_state = (nx, ny, next_wall, next_safe)
            if neighbor_state not in visited:
                visited.add(neighbor_state)
                came_from[neighbor_state] = (cx, cy, wall_count, last_safe_color)
                queue.append(neighbor_state)

    return None, 0, iterations, 0, 0

# =====================================================================
def get_path_color_info(algo, gvd, beige):
    if algo == "astar": return ([255, 0, 0], "Blue") if gvd else ([130, 0, 0], "Dark Blue")
    elif algo == "best_first": return ([255, 255, 0], "Cyan") if gvd else ([130, 130, 0], "Teal")
    elif algo == "bfs": return ([0, 0, 0], "Magenta") if beige else ([0, 0, 255], "Red")
    return ([255, 255, 255], "White")

# =====================================================================
def draw_control_panel():
    panel = np.ones((450, 260, 3), dtype=np.uint8) * 240
    size = checkbox_size
    
    buttons = [
        (checkbox_pos, use_gvd, "Use GVD"),
        (astar_checkbox_pos, algorithm == "astar", "A*"),
        (best_first_checkbox_pos, algorithm == "best_first", "Best-First"),
        (bfs_checkbox_pos, algorithm == "bfs", "BFS"),
        (beige_checkbox_pos, only_beige, "Only Beige (All)"),
    ]
    
    for pos, is_checked, label in buttons:
        x, y = pos
        cv2.rectangle(panel, (x, y), (x+size, y+size), (0, 0, 0), 2)
        if is_checked:
            cv2.line(panel, (x, y), (x+size, y+size), (0, 0, 0), 2)
            cv2.line(panel, (x+size, y), (x, y+size), (0, 0, 0), 2)
        cv2.putText(panel, label, (x + size + 15, y + size - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)

    run_color = (0, 180, 0) if not is_running else (150, 150, 150)
    clear1_color = (0, 140, 255) if not is_running else (150, 150, 150)
    clear2_color = (0, 0, 200) if not is_running else (150, 150, 150)
    clear3_color = (0, 100, 220) if not is_running else (150, 150, 150)

    cx0, cy0 = run_btn_pos
    cv2.rectangle(panel, (cx0, cy0), (cx0 + 220, cy0 + 35), run_color, -1)
    cv2.putText(panel, "Run Algorithm", (cx0 + 45, cy0 + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cx1, cy1 = clear_paths_btn_pos
    cv2.rectangle(panel, (cx1, cy1), (cx1 + 220, cy1 + 35), clear1_color, -1)
    cv2.putText(panel, "Clear Paths Only", (cx1 + 40, cy1 + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cx2, cy2 = clear_poses_btn_pos
    cv2.rectangle(panel, (cx2, cy2), (cx2 + 220, cy2 + 35), clear2_color, -1)
    cv2.putText(panel, "Clear Start/Goal", (cx2 + 40, cy2 + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cx3, cy3 = clear_doors_btn_pos
    cv2.rectangle(panel, (cx3, cy3), (cx3 + 220, cy3 + 35), clear3_color, -1)
    cv2.putText(panel, "Clear Doors", (cx3 + 60, cy3 + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imshow("Control Panel", panel)

# =====================================================================
def redraw_map():
    display = resized_image.copy()
    line_thickness = 2 

    for p_data in all_paths:
        color = p_data["color"]
        path_points = np.array(p_data["path"], np.int32)
        
        if len(path_points) > 1:
            path_points = path_points.reshape((-1, 1, 2))
            cv2.polylines(display, [path_points], isClosed=False, color=color, thickness=line_thickness)
        elif len(path_points) == 1:
            cv2.circle(display, tuple(path_points[0]), line_thickness, color, -1)

    for d in doors:
        cv2.circle(display, d, 6, (0, 0, 0), -1) 
        cv2.circle(display, d, 4, (255, 255, 255), -1)

    if start_point: cv2.circle(display, start_point, 6, (0, 255, 0), -1)
    if end_point: cv2.circle(display, end_point, 6, (0, 255, 255), -1)

    cv2.imshow("Map Navigation", display)

# =====================================================================
def animate_path(path_to_draw, color):
    display = resized_image.copy()
    line_thickness = 2 
    
    for p_data in all_paths:
        old_color = p_data["color"]
        old_points = np.array(p_data["path"], np.int32)
        if len(old_points) > 1:
            old_points = old_points.reshape((-1, 1, 2))
            cv2.polylines(display, [old_points], False, old_color, line_thickness)
        elif len(old_points) == 1:
            cv2.circle(display, tuple(old_points[0]), line_thickness, old_color, -1)

    for d in doors:
        cv2.circle(display, d, 6, (0, 0, 0), -1) 
        cv2.circle(display, d, 4, (255, 255, 255), -1)
    if start_point: cv2.circle(display, start_point, 6, (0, 255, 0), -1)
    if end_point: cv2.circle(display, end_point, 6, (0, 255, 255), -1)

    path_points = np.array(path_to_draw, np.int32)
    step_size = 4  
    
    for i in range(1, len(path_points) + 1, step_size):
        temp_display = display.copy()
        current_segment = path_points[:i]
        
        if len(current_segment) > 1:
            current_segment = current_segment.reshape((-1, 1, 2))
            cv2.polylines(temp_display, [current_segment], False, color, line_thickness)
        elif len(current_segment) == 1:
            cv2.circle(temp_display, tuple(current_segment[0]), line_thickness, color, -1)
            
        cv2.imshow("Map Navigation", temp_display)
        cv2.waitKey(1)
        
    final_segment = path_points.reshape((-1, 1, 2))
    cv2.polylines(display, [final_segment], False, color, line_thickness)
    cv2.imshow("Map Navigation", display)
    cv2.waitKey(1)

# =====================================================================
def calculate_and_add_path():
    global is_running, crosspenalty, buildingpenalty, doorpenalty
    if not start_point or not end_point:
        print("กรุณากำหนดจุด Start และ Goal ให้ครบก่อนกดรัน")
        return

    is_running = True
    draw_control_panel()

    if algorithm == "astar":
        path, cost, iters, elapsed, dist = find_path_astar(resized_image, start_point, end_point, distance_map, use_gvd, only_beige, door_map)
    elif algorithm == "best_first":
        path, cost, iters, elapsed, dist = find_path_best_first(resized_image, start_point, end_point, distance_map, use_gvd, only_beige, door_map)
    elif algorithm == "bfs":
        path, cost, iters, elapsed, dist = find_path_bfs(resized_image, start_point, end_point, only_beige, door_map)

    if path:
        color_val, color_name = get_path_color_info(algorithm, use_gvd, only_beige)
        animate_path(path, color_val)
        all_paths.append({"path": path, "color": color_val})
        
        # ✨ เพิ่มการ Print แสดงค่า Pen ทั้ง 3 ค่า
        print(f"\n[{color_name}] Algo: {algorithm.upper()} | GVD: {use_gvd} | Only Beige: {only_beige}")
        print(f"Penalties -> Cross Pen: {crosspenalty} | Bldg Pen: {buildingpenalty} | Door Pen: {doorpenalty}")
        print(f"Cost/Length: {cost:.2f} | Iterations: {iters}")
        print(f"Time: {elapsed:.2f} ms | Distance: {dist:.2f}")
        
        filepath = "/home/isaac/fra532-lab3-planning-por-wa/data/datapart3/a*-BFS.txt"
        try:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(filepath, "a", encoding="utf-8") as f:
                f.write(f"[{timestamp}] Run Summary\n")
                f.write(f"Algorithm  : {algorithm.upper()}\n")
                f.write(f"Use GVD    : {use_gvd}\n")
                f.write(f"Only Beige : {only_beige}\n")
                
                # ✨ เพิ่มการบันทึกค่า Pen ทั้ง 3 ค่าลง Text file
                f.write(f"Cross Pen  : {crosspenalty}\n")
                f.write(f"Bldg Pen   : {buildingpenalty}\n")
                f.write(f"Door Pen   : {doorpenalty}\n")
                
                f.write(f"Cost       : {cost:.2f}\n")
                f.write(f"Iterations : {iters}\n")
                f.write(f"Time       : {elapsed:.2f} ms\n")
                f.write(f"Distance   : {dist:.2f}\n")
                
                path_str = " -> ".join([f"({p[0]}, {p[1]})" for p in path])
                f.write(f"Path Nodes : {path_str}\n")
                
                f.write("-" * 40 + "\n")
            print("💾 บันทึกผลลัพธ์พร้อมค่า Penalty ลงไฟล์ 'a*-BFS.txt' เรียบร้อยแล้ว")
        except Exception as e:
            print(f"❌ ไม่สามารถเขียนไฟล์ได้: {e}")

    else:
        print(f"\n❌ ไม่พบเส้นทาง (Algo: {algorithm.upper()})")
    is_running = False
    draw_control_panel()
    redraw_map()

# =====================================================================
distance_map = compute_distance_map(resized_image)

def control_click_event(event, x, y, flags, param):
    global use_gvd, algorithm, only_beige, start_point, end_point, all_paths, doors, door_map
    
    if event == cv2.EVENT_LBUTTONDOWN:
        if is_running: return 

        def is_clicked(pos):
            cx, cy = pos
            return cx <= x <= cx+checkbox_size and cy <= y <= cy+checkbox_size

        changed = False

        if is_clicked(checkbox_pos):
            use_gvd = not use_gvd
            changed = True
        elif is_clicked(astar_checkbox_pos):
            algorithm = "astar"
            changed = True
        elif is_clicked(best_first_checkbox_pos):
            algorithm = "best_first"
            changed = True
        elif is_clicked(bfs_checkbox_pos):
            algorithm = "bfs"
            changed = True
        elif is_clicked(beige_checkbox_pos):
            only_beige = not only_beige
            changed = True

        cx0, cy0 = run_btn_pos
        if cx0 <= x <= cx0 + 220 and cy0 <= y <= cy0 + 35:
            calculate_and_add_path()
            return
            
        cx1, cy1 = clear_paths_btn_pos
        if cx1 <= x <= cx1 + 220 and cy1 <= y <= cy1 + 35:
            all_paths.clear() 
            redraw_map()
            return

        cx2, cy2 = clear_poses_btn_pos
        if cx2 <= x <= cx2 + 220 and cy2 <= y <= cy2 + 35:
            start_point = None 
            end_point = None   
            all_paths.clear()  
            redraw_map()
            return

        cx3, cy3 = clear_doors_btn_pos
        if cx3 <= x <= cx3 + 220 and cy3 <= y <= cy3 + 35:
            doors.clear()
            door_map.fill(0)
            redraw_map()
            return

        if changed:
            draw_control_panel()

def map_click_event(event, x, y, flags, param):
    global start_point, end_point, all_paths, doors, door_map

    if event == cv2.EVENT_LBUTTONDOWN:
        if is_running: return 
        if start_point is not None:
            end_point = (x, y)
            print(f"📍 Set Goal Point: {(x, y)}")
            redraw_map()

    elif event == cv2.EVENT_RBUTTONDOWN:
        if is_running: return 
        start_point = (x, y)
        end_point = None
        all_paths.clear() 
        print(f"📍 Set Start Point: {(x, y)}")
        redraw_map()
        
    elif event == cv2.EVENT_MBUTTONDOWN:
        if is_running: return
        doors.append((x, y))
        cv2.circle(door_map, (x, y), 5, 255, -1) 
        print(f"🚪 Added Door Point: {(x, y)} | Total doors: {len(doors)}")
        redraw_map()

# =====================================================================
# Callback สำหรับ Trackbar
def on_cross_trackbar(val):
    global crosspenalty
    crosspenalty = val

def on_bldg_trackbar(val):
    global buildingpenalty
    buildingpenalty = val

def on_door_trackbar(val):
    global doorpenalty
    doorpenalty = val

# =====================================================================

cv2.namedWindow("Map Navigation")
cv2.setMouseCallback("Map Navigation", map_click_event)

cv2.namedWindow("Control Panel")
cv2.setMouseCallback("Control Panel", control_click_event)

cv2.createTrackbar("Cross Pen", "Control Panel", crosspenalty, 50, on_cross_trackbar)
cv2.createTrackbar("Bldg Pen", "Control Panel", buildingpenalty, 50, on_bldg_trackbar)
cv2.createTrackbar("Door Pen", "Control Panel", doorpenalty, 50, on_door_trackbar)

print("\n=== การใช้งานเมาส์และ UI ===")
print("จุดเริ่มต้น (Start), จุดหมาย (Goal), และประตู (Door) ถูกตั้งค่าเริ่มต้นให้แล้วครับ!")
print("คลิกขวา (Right Click)  : ตั้งค่าจุดเริ่มต้น (Start) ใหม่")
print("คลิกกลาง (Middle Click): เพิ่มจุดประตูเชื่อม (Door) ใหม่")
print("คลิกซ้าย (Left Click)   : ตั้งค่าจุดหมายปลายทาง (Goal) ใหม่")
print("Trackbar               : ปรับค่า Penalty ด้านล่างหน้าต่าง Control Panel")
print("======================\n")

redraw_map()
draw_control_panel()

cv2.waitKey(0)
cv2.destroyAllWindows()