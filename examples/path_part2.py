import cv2
import numpy as np
import heapq
import time
import math
import random
from collections import deque
import datetime # ✨ เพิ่มไลบรารีเวลา

image = cv2.imread(r"/home/isaac/fra532-lab3-planning-por-wa/images/filtered_kmutt.png")

scale_percent = 100
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
resized_image = cv2.resize(image, (width, height))

# =========================
# GLOBAL
# =========================
# ✨ สร้างตัวแปรสำหรับกำหนดพาทไฟล์ Log ตามที่คุณต้องการ
log_filepath = "/home/isaac/fra532-lab3-planning-por-wa/data/datapart2/a*-BFS.txt" 

use_gvd = False
algorithm = "astar"  # "astar", "best_first", "bfs"
only_beige = False   
is_running = False   # ป้องกันการคลิกซ้อนขณะกำลังวาดแอนิเมชัน

# ✨ กำหนดพิกัดเริ่มต้นและเป้าหมายให้แสดงทันทีที่เปิดโปรแกรม
start_point = (1245, 650)
end_points = [
   (473, 474), 
    (327, 662), 
    (850, 606), 
    (381, 555), 
    (701, 836), 
    (740, 444)
]      
all_paths = [] 

# ขยับพิกัด UI สำหรับหน้าต่าง Control Panel
checkbox_pos = (20, 30)
astar_checkbox_pos = (20, 70)
best_first_checkbox_pos = (20, 110) 
bfs_checkbox_pos = (20, 150)
beige_checkbox_pos = (20, 190)
run_btn_pos = (20, 235)     
clear_btn_pos = (20, 280)   
checkbox_size = 20

# =====================================================================
def get_pixel_type(color):
    b, g, r = int(color[0]), int(color[1]), int(color[2])
    if abs(b - 213) < 15 and abs(g - 232) < 15 and abs(r - 242) < 15:
        return "BEIGE"
    if abs(b - 180) < 20 and abs(g - 105) < 20 and abs(r - 255) < 20:
        return "PINK"
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

# =====================================================================
def compute_path_distance(path):
    if path is None or len(path) < 2:
        return 0
    distance = 0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        if x1 != x2 and y1 != y2:
            distance += 1.414
        else:
            distance += 1
    return distance

# =====================================================================
def find_path_astar(img, start, end, distance_map, use_gvd, only_beige):
    rows, cols = img.shape[:2]
    open_set = []
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK"] else "BEIGE"
    heapq.heappush(open_set, (0, 0, start[0], start[1], 0, start_safe_color))

    came_from = {}
    g_score = {(start[0], start[1], 0, start_safe_color): 0}
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    # ✨ รัศมีอนุโลมให้เดินบนสีชมพูใกล้ๆ จุด Start/Goal ของแต่ละช่วงได้ (พิกเซล)
    GRACE_RADIUS = 40.0 

    while open_set:
        iterations += 1
        f, g, cx, cy, wall_count, last_safe_color = heapq.heappop(open_set)
        
        if (cx, cy) == end:
            end_time = time.time()
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            path = path[::-1]
            elapsed = (end_time - start_time) * 1000
            dist = compute_path_distance(path)
            return path, g, iterations, elapsed, dist

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            pixel_type = get_pixel_type(img[ny, nx])
            penalty = 0

            # ✨ คำนวณระยะห่างจากจุด Start และ End (ของ segment ปัจจุบัน)
            dist_to_start = ((nx - start[0])**2 + (ny - start[1])**2)**0.5
            dist_to_end = ((nx - end[0])**2 + (ny - end[1])**2)**0.5

            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE":
                    continue
                if last_safe_color == "PINK": penalty += 0
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                # ✨ ใช้เงื่อนไข Grace Radius
                if only_beige: 
                    if dist_to_start > GRACE_RADIUS and dist_to_end > GRACE_RADIUS:
                        continue 
                if last_safe_color == "BEIGE": penalty += 0
                next_wall, next_safe = 0, "PINK"
            else:
                next_wall = wall_count + 1
                if next_wall > 5: continue
                next_safe = last_safe_color
                penalty += 0

            step_cost = 1.414 if dx != 0 and dy != 0 else 1
            gvd_bonus = 0
            if use_gvd:
                dist_value = distance_map[ny, nx]
                goal_dist = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                if pixel_type == "BEIGE" and goal_dist > 30:
                    gvd_bonus = -0.8 * dist_value

            tentative_g = g + step_cost + penalty + gvd_bonus
            neighbor_state = (nx, ny, next_wall, next_safe)

            if neighbor_state not in g_score or tentative_g < g_score[neighbor_state]:
                came_from[neighbor_state] = (cx, cy, wall_count, last_safe_color)
                g_score[neighbor_state] = tentative_g
                h = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                heapq.heappush(open_set, (tentative_g + h, tentative_g, nx, ny, next_wall, next_safe))
    return None, 0, iterations, 0, 0

# =====================================================================
def find_path_best_first(img, start, end, distance_map, use_gvd, only_beige):
    rows, cols = img.shape[:2]
    open_set = []
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK"] else "BEIGE"

    h_start = ((start[0] - end[0])**2 + (start[1] - end[1])**2)**0.5
    heapq.heappush(open_set, (h_start, start[0], start[1], 0, start_safe_color, 0))

    came_from = {}
    visited = set()
    visited.add((start[0], start[1], 0, start_safe_color))
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    # ✨ รัศมีอนุโลม
    GRACE_RADIUS = 40.0 

    while open_set:
        iterations += 1
        h, cx, cy, wall_count, last_safe_color, g = heapq.heappop(open_set)

        if (cx, cy) == end:
            end_time = time.time()
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            path = path[::-1]
            return path, g, iterations, (end_time - start_time) * 1000, compute_path_distance(path)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            pixel_type = get_pixel_type(img[ny, nx])
            penalty = 0

            # ✨ คำนวณระยะห่าง
            dist_to_start = ((nx - start[0])**2 + (ny - start[1])**2)**0.5
            dist_to_end = ((nx - end[0])**2 + (ny - end[1])**2)**0.5

            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE": continue
                if last_safe_color == "PINK": penalty += 0
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                # ✨ ใช้เงื่อนไข Grace Radius
                if only_beige: 
                    if dist_to_start > GRACE_RADIUS and dist_to_end > GRACE_RADIUS:
                        continue 
                if last_safe_color == "BEIGE": penalty += 0
                next_wall, next_safe = 0, "PINK"
            else:
                next_wall = wall_count + 1
                if next_wall > 5: continue
                next_safe = last_safe_color
                penalty += 0

            step_cost = 1.414 if dx != 0 and dy != 0 else 1
            gvd_bonus_cost = 0
            gvd_bonus_priority = 0
            if use_gvd:
                dist_value = distance_map[ny, nx]
                goal_dist = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                if pixel_type == "BEIGE" and goal_dist > 30:
                    gvd_bonus_cost = -0.8 * dist_value
                    gvd_bonus_priority = -60.0 * dist_value 

            tentative_g = g + step_cost + penalty + gvd_bonus_cost
            neighbor_state = (nx, ny, next_wall, next_safe)

            if neighbor_state not in visited:
                visited.add(neighbor_state)
                came_from[neighbor_state] = (cx, cy, wall_count, last_safe_color)
                h_next = ((nx - end[0])**2 + (ny - end[1])**2)**0.5
                priority = h_next + penalty + gvd_bonus_priority
                heapq.heappush(open_set, (priority, nx, ny, next_wall, next_safe, tentative_g))

    return None, 0, iterations, 0, 0

# =====================================================================
def find_path_bfs(img, start, end, only_beige):
    rows, cols = img.shape[:2]
    queue = deque()
    start_time = time.time()
    iterations = 0

    start_type = get_pixel_type(img[start[1], start[0]])
    start_safe_color = start_type if start_type in ["BEIGE", "PINK"] else "BEIGE"

    start_state = (start[0], start[1], 0, start_safe_color)
    queue.append(start_state)

    came_from = {}
    visited = set()
    visited.add(start_state)
    neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    # ✨ รัศมีอนุโลม
    GRACE_RADIUS = 40.0 

    while queue:
        iterations += 1
        cx, cy, wall_count, last_safe_color = queue.popleft()

        if (cx, cy) == end:
            end_time = time.time()
            path = []
            curr = (cx, cy, wall_count, last_safe_color)
            while curr in came_from:
                path.append((curr[0], curr[1]))
                curr = came_from[curr]
            path.append(start)
            path = path[::-1]
            return path, len(path), iterations, (end_time - start_time) * 1000, compute_path_distance(path)

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            pixel_type = get_pixel_type(img[ny, nx])

            # ✨ คำนวณระยะห่าง
            dist_to_start = ((nx - start[0])**2 + (ny - start[1])**2)**0.5
            dist_to_end = ((nx - end[0])**2 + (ny - end[1])**2)**0.5

            if pixel_type == "BEIGE":
                if wall_count > 0 and last_safe_color == "BEIGE": continue
                next_wall, next_safe = 0, "BEIGE"
            elif pixel_type == "PINK":
                # ✨ ใช้เงื่อนไข Grace Radius
                if only_beige: 
                    if dist_to_start > GRACE_RADIUS and dist_to_end > GRACE_RADIUS:
                        continue
                next_wall, next_safe = 0, "PINK"
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
    elif algo == "best_first": return ([180, 0, 180], "Purple") if gvd else ([90, 0, 90], "Dark Purple")
    elif algo == "bfs": return ([0, 0, 0], "Magenta") if beige else ([0, 0, 255], "Red")
    return ([255, 255, 255], "White")

# =====================================================================
def draw_control_panel():
    panel = np.ones((340, 260, 3), dtype=np.uint8) * 240
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

    # ปุ่ม Run
    rx, ry = run_btn_pos
    cv2.rectangle(panel, (rx, ry), (rx + 220, ry + 35), (0, 200, 0) if not is_running else (100, 100, 100), -1)
    cv2.putText(panel, "Run Pathfinding", (rx + 35, ry + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    # ปุ่ม Clear
    cx, cy = clear_btn_pos
    cv2.rectangle(panel, (cx, cy), (cx + 220, cy + 35), (0, 0, 200) if not is_running else (100, 100, 100), -1)
    cv2.putText(panel, "Clear Paths", (cx + 60, cy + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imshow("Control Panel", panel)

# =====================================================================
def redraw_map():
    display = resized_image.copy()
    
    line_thickness = 2 

    for p_data in all_paths:
        color = p_data["color"]
        for p in p_data["path"]:
            cv2.circle(display, (p[0], p[1]), line_thickness, color, -1)

    if start_point:
        cv2.circle(display, start_point, 6, (0, 255, 0), -1)
        cv2.putText(display, "S", (start_point[0] - 5, start_point[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1, cv2.LINE_AA)

    for ep in end_points:
        cv2.circle(display, ep, 6, (0, 0, 255), -1) 

    cv2.imshow("Map Navigation", display)

# =====================================================================
def calculate_and_add_path():
    global is_running
    if not start_point or not end_points:
        print("กรุณาเลือกจุด Start และ End (อย่างน้อย 1 จุด) ก่อนกด Run")
        return

    is_running = True 
    draw_control_panel()
    print(f"\nกำลังค้นหาเส้นทางไปยังเป้าหมายที่ใกล้ที่สุดทีละจุด... (รวม {len(end_points)} จุด)")
    
    all_paths.clear()

    unvisited = end_points.copy()
    current_start = start_point
    
    total_cost = 0
    total_dist = 0
    total_time = 0
    total_iters = 0  # ✨ เพิ่มตัวแปรสำหรับเก็บ Iterations รวม
    step = 1
    
    # ✨ ตัวแปรสำหรับเก็บพิกัดแยกตามช่วงการเดินทาง
    segment_paths = []

    # ================= ฟังก์ชันสำหรับเล่นแอนิเมชันตอนวาดเส้น ================= #
    def animate_path(path_to_draw, color):
        display = resized_image.copy()
        line_thickness = 2 
        
        for p_data in all_paths:
            for p in p_data["path"]:
                cv2.circle(display, (p[0], p[1]), line_thickness, p_data["color"], -1)
        
        if start_point:
            cv2.circle(display, start_point, 6, (0, 255, 0), -1)
            cv2.putText(display, "S", (start_point[0] - 5, start_point[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1, cv2.LINE_AA)
        
        for ep in unvisited:
            cv2.circle(display, ep, 6, (0, 0, 255), -1)
            
        for i, p in enumerate(path_to_draw):
            cv2.circle(display, (p[0], p[1]), line_thickness, color, -1)
            if i % 3 == 0:
                cv2.imshow("Map Navigation", display)
                cv2.waitKey(1) 
        
        cv2.imshow("Map Navigation", display)
        cv2.waitKey(1)
    # ================================================================ #

    try:
        while unvisited:
            nearest_target = min(unvisited, key=lambda p: math.hypot(current_start[0] - p[0], current_start[1] - p[1]))
            
            print(f"\n[ขาไป] ช่วงที่ {step}: ({current_start[0]}, {current_start[1]}) -> ({nearest_target[0]}, {nearest_target[1]})")
            
            if algorithm == "astar":
                path, cost, iters, elapsed, dist = find_path_astar(resized_image, current_start, nearest_target, distance_map, use_gvd, only_beige)
            elif algorithm == "best_first":
                path, cost, iters, elapsed, dist = find_path_best_first(resized_image, current_start, nearest_target, distance_map, use_gvd, only_beige)
            elif algorithm == "bfs":
                path, cost, iters, elapsed, dist = find_path_bfs(resized_image, current_start, nearest_target, only_beige)

            if path:
                color_val, color_name = get_path_color_info(algorithm, use_gvd, only_beige)
                
                animate_path(path, color_val)
                all_paths.append({"path": path, "color": color_val})
                
                print(f"[{color_name}] สำเร็จ | Cost: {cost} | Iterations: {iters} | Time: {elapsed:.2f} ms")
                
                total_cost += cost
                total_dist += dist
                total_time += elapsed
                total_iters += iters # ✨ บวก Iterations สะสม
                
                # ✨ เก็บพิกัดและ Iterations แยกตามช่วง
                segment_paths.append({
                    "name": f"ช่วงที่ {step}: ({current_start[0]}, {current_start[1]}) ไป ({nearest_target[0]}, {nearest_target[1]})",
                    "path": path,
                    "iters": iters # ✨ เก็บ iters ของช่วงนี้
                })
                
                current_start = nearest_target
                unvisited.remove(nearest_target)
                step += 1
            else:
                print(f"❌ ไม่พบเส้นทางไปจุดที่เลือก โปรแกรมจะข้ามจุดนี้ไปหาจุดถัดไปแทน")
                unvisited.remove(nearest_target) 
                
        # ขากลับ: วิ่งกลับไปหาจุดเริ่ม
        print(f"\n[ขากลับ] กลับจุดเริ่มต้น: ({current_start[0]}, {current_start[1]}) -> ({start_point[0]}, {start_point[1]})")
        if algorithm == "astar":
            path, cost, iters, elapsed, dist = find_path_astar(resized_image, current_start, start_point, distance_map, use_gvd, only_beige)
        elif algorithm == "best_first":
            path, cost, iters, elapsed, dist = find_path_best_first(resized_image, current_start, start_point, distance_map, use_gvd, only_beige)
        elif algorithm == "bfs":
            path, cost, iters, elapsed, dist = find_path_bfs(resized_image, current_start, start_point, only_beige)

        if path:
            color_val, color_name = get_path_color_info(algorithm, use_gvd, only_beige)
            
            animate_path(path, color_val)
            all_paths.append({"path": path, "color": color_val})
            
            print(f"[{color_name}] กลับจุดเริ่มสำเร็จ | Cost: {cost} | Iterations: {iters} | Time: {elapsed:.2f} ms")
            total_cost += cost
            total_dist += dist
            total_time += elapsed
            total_iters += iters # ✨ บวก Iterations สะสมขากลับ
            
            # ✨ เก็บพิกัดและ Iterations ขากลับ
            segment_paths.append({
                "name": f"ขากลับ: ({current_start[0]}, {current_start[1]}) กลับ ({start_point[0]}, {start_point[1]})",
                "path": path,
                "iters": iters # ✨ เก็บ iters ของขากลับ
            })
        else:
            print(f"❌ ไม่สามารถหาเส้นทางกลับไปยังจุดเริ่มต้นได้")

        print(f"\n--- สรุปการเดินทางรวม ---")
        print(f"Total Cost: {total_cost}")
        print(f"Total Distance: {total_dist:.2f}")
        print(f"Total Time: {total_time:.2f} ms")
        print(f"Total Iterations: {total_iters}") # ✨ แสดงยอดรวม Iterations ใน Terminal
        print("--------------------------")
        
        # ✨ ปลดคอมเมนต์แล้ว และเพิ่มการบันทึก Iterations ลงไฟล์
        try:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(log_filepath, "a", encoding="utf-8") as f:
                f.write(f"[{timestamp}] Multi-Point Run Summary\n")
                f.write(f"Algorithm  : {algorithm.upper()}\n")
                f.write(f"Use GVD    : {use_gvd}\n")
                f.write(f"Only Beige : {only_beige}\n")
                f.write(f"Points     : {len(end_points)} target(s) + return home\n")
                f.write(f"Total Cost : {total_cost:.2f}\n")
                f.write(f"Total Dist : {total_dist:.2f}\n")
                f.write(f"Total Time : {total_time:.2f} ms\n")
                f.write(f"Total Iters: {total_iters}\n") # ✨ บันทึก Iteration รวม
                
                # เขียนบันทึกพิกัดแยกตามช่วง
                f.write("Path Nodes by Segment:\n")
                for seg in segment_paths:
                    path_str = " -> ".join([f"({p[0]}, {p[1]})" for p in seg["path"]])
                    # ✨ ระบุ Iterations ของแต่ละช่วงด้วย
                    f.write(f"  - {seg['name']} | Iterations: {seg['iters']} | Nodes: {len(seg['path'])}\n")
                    f.write(f"    {path_str}\n\n")
                
                f.write("-" * 40 + "\n")
            print(f"💾 บันทึกผลสรุปรวมและพิกัดลงไฟล์ '{log_filepath}' เรียบร้อยแล้ว")
        except Exception as e:
            print(f"❌ ไม่สามารถเขียนไฟล์ได้: {e}")
            
        redraw_map()
        
    finally:
        is_running = False
        draw_control_panel()

distance_map = compute_distance_map(resized_image)

def control_click_event(event, x, y, flags, param):
    global use_gvd, algorithm, only_beige, start_point, end_points, all_paths

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

        rx, ry = run_btn_pos
        if rx <= x <= rx + 220 and ry <= y <= ry + 35:
            calculate_and_add_path()
            return
            
        cx, cy = clear_btn_pos
        if cx <= x <= cx + 220 and cy <= y <= cy + 35:
            start_point = None
            end_points.clear() 
            all_paths.clear()
            # ✨ เพิ่ม Print แจ้งเตือนเวลาเคลียร์ค่าทิ้งทั้งหมด
            print("🧹 Cleared Start and all Goal points.")
            redraw_map()
            draw_control_panel()
            return

        if changed:
            draw_control_panel()

def map_click_event(event, x, y, flags, param):
    global start_point, end_points, all_paths

    if event == cv2.EVENT_LBUTTONDOWN:
        if is_running: return 
        if start_point is not None:
            end_points.append((x, y))
            # ✨ Print แจ้งพิกัดเป้าหมาย (Waypoint) ที่เพิ่งถูกเพิ่มเข้าไป
            print(f"📍 Added Goal Point: {(x, y)} | Total goals: {len(end_points)}")
            redraw_map() 

    elif event == cv2.EVENT_RBUTTONDOWN:
        if is_running: return
        start_point = (x, y)
        end_points.clear() 
        all_paths.clear() 
        # ✨ Print แจ้งพิกัดเริ่มต้นใหม่ที่ถูกตั้งค่า
        print(f"📍 Set Start Point: {start_point}")
        redraw_map()

# =====================================================================
cv2.namedWindow("Map Navigation")
cv2.setMouseCallback("Map Navigation", map_click_event)

cv2.namedWindow("Control Panel")
cv2.setMouseCallback("Control Panel", control_click_event)

redraw_map()
draw_control_panel()

cv2.waitKey(0)
cv2.destroyAllWindows()