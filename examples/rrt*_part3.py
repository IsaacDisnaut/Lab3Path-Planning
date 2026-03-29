import random
import numpy as np
import cv2
import math
import time
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button

# =====================================================================
def get_pixel_type(color):
    b, g, r = int(color[0]), int(color[1]), int(color[2])
    # 0: BEIGE (ภายนอก/สีขาว)
    if abs(b - 213) < 15 and abs(g - 232) < 15 and abs(r - 242) < 15:
        return 0
    # 1: PINK (ลานสีชมพู)
    if abs(b - 180) < 20 and abs(g - 105) < 20 and abs(r - 255) < 20:
        return 1
    # 2: BUILDING (ตึกส้มและเหลือง)
    if abs(b - 43) < 20 and abs(g - 95) < 20 and abs(r - 226) < 20: # ส้ม
        return 2
    if abs(b - 49) < 20 and abs(g - 193) < 20 and abs(r - 211) < 20: # เหลือง
        return 2
    # 3: WALL (สิ่งกีดขวาง/กำแพง)
    return 3
# =====================================================================

class Node:
    def __init__(self, index, x, y, cost=0.0):
        self.index = index
        self.x = x
        self.y = y
        self.cost = cost 

    def Dist(self, others):
        return np.linalg.norm([self.x - others.x, self.y - others.y], 2)

class Tree:
    def __init__(self):
        self.nodes = [] 
        self.parents = {} 
        self.root = None

    def AddNodes(self, x, y, cost=0.0):
        new_index = len(self.nodes)
        new_node = Node(new_index, x, y, cost)
        self.nodes.append(new_node)
        return new_node

    def SetRoot(self, x, y):
        if self.root is None:
            root_node = self.AddNodes(x, y, cost=0.0)
            self.root = root_node.index
            self.parents[self.root] = None
            return True
        return False

    def AddParent(self, child, parent, edge_cost):
        self.parents[child] = (parent, edge_cost)

class MapDisplay:
    def __init__(self, bg_img, type_map):
        self.type_map = type_map
        self.bg_img = bg_img 
        self.door_mask = np.zeros_like(type_map, dtype=bool) 
        self.doors = [] 
        
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_axes([0.2, 0.05, 0.75, 0.85]) 
        self.ax.set_aspect('equal')
        self.ax.axis('off')    
        self.ax.imshow(bg_img)
        
        self.ax_check = self.fig.add_axes([0.02, 0.8, 0.15, 0.1])
        self.check = CheckButtons(self.ax_check, ['Use GVD Cost'], [False])
        self.use_gvd = False
        
        def on_check_clicked(label):
            self.use_gvd = not self.use_gvd
        self.check.on_clicked(on_check_clicked)

        # ✨ เพิ่มตัวแปรควบคุมการรัน
        self.run_flag = False 

        # ✨ 0. ปุ่ม Run Planning
        self.ax_btn_run = self.fig.add_axes([0.02, 0.72, 0.15, 0.06]) 
        self.btn_run = Button(self.ax_btn_run, 'Run Planning', color='lightgreen', hovercolor='lime')
        
        def on_run_clicked(event):
            if self.start_pt is not None and self.goal_pt is not None:
                self.run_flag = True
                self.ax.set_title("Planning Path with RRT*...")
                self.fig.canvas.draw()
            else:
                self.ax.set_title("⚠️ Please set Start and Goal first!")
                self.fig.canvas.draw()
        self.btn_run.on_clicked(on_run_clicked)

        # ✨ 1. ปุ่ม Clear Start/Goal
        self.ax_btn_sg = self.fig.add_axes([0.02, 0.62, 0.15, 0.06]) 
        self.btn_clear_sg = Button(self.ax_btn_sg, 'Clear Start/Goal', color='lightcoral', hovercolor='red')
        self.should_reset = False 
        
        def on_clear_sg_clicked(event):
            self.should_reset = True
            self.run_flag = False # ✨ หยุดรันด้วยถ้ากด Clear
            self.start_pt = None
            self.goal_pt = None
            
            self.ax.clear() 
            self.ax.set_aspect('equal')
            self.ax.axis('off')
            self.ax.imshow(self.bg_img) 
            self.ax.set_title("R-Click=Start | L-Click=Goal | Mid-Click=Door")
            
            for dx, dy in self.doors:
                self.ax.plot(dx, dy, 'co', markersize=8, zorder=4)
            
            self.fig.canvas.mpl_disconnect(self.cid) 
            self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
            self.fig.canvas.draw()
            
        self.btn_clear_sg.on_clicked(on_clear_sg_clicked)

        # ✨ 2. ปุ่ม Clear Path
        self.ax_btn_path = self.fig.add_axes([0.02, 0.52, 0.15, 0.06]) 
        self.btn_clear_path = Button(self.ax_btn_path, 'Clear Path', color='khaki', hovercolor='gold')

        def on_clear_path_clicked(event):
            self.should_reset = True
            self.run_flag = False # ✨ หยุดรันด้วย
            
            self.ax.clear() 
            self.ax.set_aspect('equal')
            self.ax.axis('off')
            self.ax.imshow(self.bg_img) 
            self.ax.set_title("Path Cleared! Click 'Run Planning' to recalculate.")
            
            if self.start_pt:
                self.ax.plot(self.start_pt[0], self.start_pt[1], 'go', markersize=8, zorder=5)
            if self.goal_pt:
                self.ax.plot(self.goal_pt[0], self.goal_pt[1], 'ro', markersize=8, zorder=5)
            for dx, dy in self.doors:
                self.ax.plot(dx, dy, 'co', markersize=8, zorder=4)
                
            self.fig.canvas.mpl_disconnect(self.cid) 
            self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
            self.fig.canvas.draw()
            
        self.btn_clear_path.on_clicked(on_clear_path_clicked)

        # ✨ 3. ปุ่ม Clear Doors
        self.ax_btn_door = self.fig.add_axes([0.02, 0.42, 0.15, 0.06]) 
        self.btn_clear_door = Button(self.ax_btn_door, 'Clear Doors', color='lightblue', hovercolor='cyan')

        def on_clear_doors_clicked(event):
            self.door_mask.fill(False) 
            self.doors.clear()         
            
            self.ax.clear() 
            self.ax.set_aspect('equal')
            self.ax.axis('off')
            self.ax.imshow(self.bg_img) 
            self.ax.set_title("R-Click=Start | L-Click=Goal | Mid-Click=Door")
            
            if self.start_pt:
                self.ax.plot(self.start_pt[0], self.start_pt[1], 'go', markersize=8, zorder=5)
            if self.goal_pt:
                self.ax.plot(self.goal_pt[0], self.goal_pt[1], 'ro', markersize=8, zorder=5)

            self.fig.canvas.draw()
            
        self.btn_clear_door.on_clicked(on_clear_doors_clicked)

        self.start_pt = (925, 486)
        self.goal_pt = (370, 588)
        
        self.ax.plot(self.start_pt[0], self.start_pt[1], 'go', markersize=8, zorder=5)
        print(f"set start at {self.start_pt}")
        
        self.ax.plot(self.goal_pt[0], self.goal_pt[1], 'ro', markersize=8, zorder=5)
        print(f"set goal at {self.goal_pt}")

        preset_doors = [(873, 577), (753, 600)]
        door_radius = 12
        for dx, dy in preset_doors:
            cv2.circle(self.door_mask.view(np.uint8), (dx, dy), door_radius, 1, -1)
            self.doors.append((dx, dy))
            self.ax.plot(dx, dy, 'co', markersize=8, zorder=4)
            print(f"set door at ({dx}, {dy})")

        self.ax.set_title("Points selected! Click 'Run Planning' to start.")
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

    def onclick(self, event):
        if event.inaxes != self.ax: return 
        if event.xdata is None or event.ydata is None: return
        x, y = int(event.xdata), int(event.ydata)
        
        if event.button in [1, 3]: 
            if self.type_map[y, x] == 3 and not self.door_mask[y, x]:
                self.ax.set_title("⚠️ Obstacle! Place a door here first, or click elsewhere.")
                self.fig.canvas.draw()
                return

        if event.button == 3: 
            self.start_pt = (x, y)
            self.ax.plot(x, y, 'go', markersize=8, zorder=5)
            print(f"set start at {self.start_pt}")
            
        elif event.button == 1: 
            self.goal_pt = (x, y)
            self.ax.plot(x, y, 'ro', markersize=8, zorder=5) 
            print(f"set goal at {self.goal_pt}")
            
        elif event.button == 2: 
            door_radius = 12
            cv2.circle(self.door_mask.view(np.uint8), (x, y), door_radius, 1, -1)
            self.doors.append((x, y)) 
            self.ax.plot(x, y, 'co', markersize=8, zorder=4) 
            print(f"set door at ({x}, {y})")

        if self.start_pt and self.goal_pt:
            self.ax.set_title("Points selected! Click 'Run Planning' to start.")
        self.fig.canvas.draw()

    def DrawNewEdge(self, parent_node, child_node):
        self.ax.plot([parent_node.x, child_node.x], [parent_node.y, child_node.y], 'y-', linewidth=0.5, alpha=0.5)
        self.ax.plot(child_node.x, child_node.y, 'mo', markersize=1)

    def CalculateAndDrawPath(self, tree, s, time_elapsed):
        path_cost = 0
        path_nodes = []
        while s in tree.parents.keys() and tree.parents[s] is not None:
            parent_idx = tree.parents[s][0]
            curr_node = tree.nodes[s]
            parent_node = tree.nodes[parent_idx]
            
            path_nodes.append((curr_node.x, curr_node.y))
            self.ax.plot([curr_node.x, parent_node.x], [curr_node.y, parent_node.y], 'b-', linewidth=1.0, zorder=4)
            path_cost += curr_node.Dist(parent_node)
            s = parent_idx
            
        path_nodes.append((tree.nodes[s].x, tree.nodes[s].y))    
        title_text = f"Path Found! | Time: {time_elapsed:.2f} s | Dist: {path_cost:.2f} px"
        self.ax.set_title(title_text, color='blue', fontweight='bold')
        print(f"✅ {title_text}")
        self.fig.canvas.draw()
        return path_cost, path_nodes[::-1]

# =====================================================================
def GetEdgeCost(n1, n2, type_map, door_mask, dist_map, use_gvd):
    base_dist = n1.Dist(n2)
    
    n1_y, n1_x = min(max(n1.y, 0), type_map.shape[0]-1), min(max(n1.x, 0), type_map.shape[1]-1)
    n2_y, n2_x = min(max(n2.y, 0), type_map.shape[0]-1), min(max(n2.x, 0), type_map.shape[1]-1)
    
    # 1. คิด Cost พื้นฐาน (บวกลบ GVD ตามที่ติ๊ก)
    if use_gvd:
        gvd_value = dist_map[n2_y, n2_x] 
        penalty = 1.0 - (gvd_value*0.8) 
        cost = base_dist * penalty
    else:
        cost = base_dist
        
    # 2. ✨ เช็คว่าจุดหมายปลายทางอยู่ในตึก (Type 2) หรือไม่
    if type_map[n2_y, n2_x] == 2:

        cost += (base_dist * 1.0) 
        
    return cost

def CollisionFree(type_map, door_mask, n1, n2):
    dist = n1.Dist(n2)
    if dist == 0: return True
    steps = max(1, int(dist))
    
    n1_y, n1_x = min(max(n1.y, 0), type_map.shape[0]-1), min(max(n1.x, 0), type_map.shape[1]-1)
    n2_y, n2_x = min(max(n2.y, 0), type_map.shape[0]-1), min(max(n2.x, 0), type_map.shape[1]-1)

    z1 = type_map[n1_y, n1_x]
    z2 = type_map[n2_y, n2_x]

    # ✨ บล็อคเป้าหมาย: ถ้าปลายทางคือ กำแพง(3) หรือ สีชมพู(1) ถือว่าชนทันที
    if (z2 == 3 or z2 == 1) and not door_mask[n2_y, n2_x]:
        return False

    last_zone = z1
    last_y, last_x = n1_y, n1_x

    for i in range(steps + 1):
        frac = i / steps
        x_chk = int(round((1 - frac) * n1.x + frac * n2.x))
        y_chk = int(round((1 - frac) * n1.y + frac * n2.y))
        
        if x_chk < 0 or x_chk >= type_map.shape[1] or y_chk < 0 or y_chk >= type_map.shape[0]: 
            return False
            
        curr_zone = type_map[y_chk, x_chk]
        is_door = door_mask[y_chk, x_chk]
        
        # ✨ บล็อคระหว่างทาง: ถ้าลากเส้นพาดผ่าน กำแพง(3) หรือ สีชมพู(1) ถือว่าชนทันที
        if (curr_zone == 3 or curr_zone == 1) and not is_door: 
            return False 
        
        # เช็คเงื่อนไขเดินข้ามโซน (เหลือแค่ 0 กับ 2)
        if curr_zone != 3 and curr_zone != 1 and curr_zone != last_zone:
            if curr_zone == 2 or last_zone == 2:
                if not is_door and not door_mask[last_y, last_x]:
                    return False 
                
        if curr_zone != 3 and curr_zone != 1:
            last_zone = curr_zone
            
        last_y, last_x = y_chk, x_chk
        
    return True

def CreateMapFromImage(image_path, scale_percent=75):
    img = cv2.imread(image_path)
    if img is None: return None, None, None
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    img = cv2.resize(img, (width, height))
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    type_map = np.zeros((height, width), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            type_map[y, x] = get_pixel_type(img[y, x])
                
    # ✨ นิยามพื้นที่วิ่งได้ใหม่: วิ่งได้เฉพาะ BEIGE(0) และ BUILDING(2) เท่านั้น
    free_space = np.uint8((type_map == 0) | (type_map == 2)) * 255
    dist_map = cv2.distanceTransform(free_space, cv2.DIST_L2, 5)
    cv2.normalize(dist_map, dist_map, 0, 1.0, cv2.NORM_MINMAX)
    
    return type_map, img_rgb, dist_map

def GenRandomPos(max_x, max_y, goal_pt):
    if random.random() < 0.10: return goal_pt
    return (random.randint(0, max_x - 1), random.randint(0, max_y - 1))

def FindNearest(tree, z_rand):
    min_dist = float('inf')
    z_nearest = None
    for n in tree.nodes:
        new_dist = z_rand.Dist(n)
        if new_dist < min_dist:
            z_nearest = n
            min_dist = new_dist
    return z_nearest

def Steer(z_nearest, z_rand):
    MAX_TRAVEL_DIST = 10 
    dist = z_nearest.Dist(z_rand)
    if dist < MAX_TRAVEL_DIST: return z_rand
    frac = MAX_TRAVEL_DIST / dist
    x_new = (1 - frac) * z_nearest.x + frac * z_rand.x
    y_new = (1 - frac) * z_nearest.y + frac * z_rand.y
    return Node(-1, int(round(x_new)), int(round(y_new)))

def RRT_Star(type_map, img_rgb, dist_map):
    h, w = type_map.shape
    dis = MapDisplay(img_rgb, type_map)
    
    plt.ion()
    plt.show()

    run_counter = 0 
    
    # ✨ เพิ่มการตั้งค่า Seed เป็น 1 ตรงนี้

    
    while plt.fignum_exists(dis.fig.number):
        dis.should_reset = False
        dis.run_flag = False # ✨ รีเซ็ตค่าการรัน
        seed = 1
        random.seed(seed)
        np.random.seed(seed)
        # ✨ วนลูปรอให้ผู้ใช้กดปุ่ม Run Planning
        while not dis.run_flag:
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return
            if dis.should_reset: break 
            
        if dis.should_reset: continue 

        run_counter += 1 
        dis.fig.canvas.mpl_disconnect(dis.cid) 
        xi, xg = dis.start_pt, dis.goal_pt

        T = Tree()
        T.SetRoot(xi[0], xi[1])
        goal_node = None
        
        print(f"\n🚀 RRT* is running (Run #{run_counter})...")
        start_time = time.time() 
        SEARCH_RADIUS = 20.0 
        
        for i in range(250000000):
            if dis.should_reset:
                print("\n🛑 Calculation stopped / Path Cleared.")
                break 

            pos_rand = GenRandomPos(w, h, xg)
            z_rand = Node(-1, pos_rand[0], pos_rand[1])
            z_nearest = FindNearest(T, z_rand)
            z_new = Steer(z_nearest, z_rand)
            
            if CollisionFree(type_map, dis.door_mask, z_nearest, z_new):
                near_nodes = []
                for n in T.nodes:
                    if n.Dist(z_new) <= SEARCH_RADIUS and CollisionFree(type_map, dis.door_mask, n, z_new):
                        near_nodes.append(n)
                
                min_cost = z_nearest.cost + GetEdgeCost(z_nearest, z_new, type_map, dis.door_mask, dist_map, dis.use_gvd)
                best_parent = z_nearest
                
                for near_node in near_nodes:
                    cost = near_node.cost + GetEdgeCost(near_node, z_new, type_map, dis.door_mask, dist_map, dis.use_gvd)
                    if cost < min_cost:
                        min_cost = cost
                        best_parent = near_node
                
                new_node = T.AddNodes(z_new.x, z_new.y, cost=min_cost)
                T.AddParent(new_node.index, best_parent.index, GetEdgeCost(best_parent, new_node, type_map, dis.door_mask, dist_map, dis.use_gvd))
                
                for near_node in near_nodes:
                    rewired_cost = new_node.cost + GetEdgeCost(new_node, near_node, type_map, dis.door_mask, dist_map, dis.use_gvd)
                    if rewired_cost < near_node.cost:
                        near_node.cost = rewired_cost
                        T.AddParent(near_node.index, new_node.index, GetEdgeCost(new_node, near_node, type_map, dis.door_mask, dist_map, dis.use_gvd))
                
                if i % 1 == 0: 
                    dis.DrawNewEdge(best_parent, new_node)
                    plt.pause(0.001) 
                print(f"\rNodes generated: {len(T.nodes)}", end="")
                if new_node.Dist(Node(-1, xg[0], xg[1])) <= 10:
                    if CollisionFree(type_map, dis.door_mask, new_node, Node(-1, xg[0], xg[1])):
                        final_cost = new_node.cost + GetEdgeCost(new_node, Node(-1, xg[0], xg[1]), type_map, dis.door_mask, dist_map, dis.use_gvd)
                        final_node = T.AddNodes(xg[0], xg[1], cost=final_cost)
                        T.AddParent(final_node.index, new_node.index, GetEdgeCost(new_node, final_node, type_map, dis.door_mask, dist_map, dis.use_gvd))
                        dis.DrawNewEdge(new_node, final_node)
                        goal_node = final_node
                        break 
        
        print() 
        
        if dis.should_reset: continue 
        
        end_time = time.time() 
        time_elapsed = end_time - start_time
        
        if goal_node is not None:
            # path_cost ตรงนี้คือระยะทางเพียวๆ (Distance)
            path_cost, path_coords = dis.CalculateAndDrawPath(T, goal_node.index, time_elapsed)
            
            # ✨ ดึงค่า Cost จริงๆ ที่สะสมมาจากตัวอัลกอริทึม
            actual_total_cost = goal_node.cost 
            
            try:
                log_file = "/home/isaac/fra532-lab3-planning-por-wa/data/datapart3/rrt*.txt" 
                with open(log_file, "a", encoding="utf-8") as f:
                    f.write(f"=== RRT* Run Result #{run_counter} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===\n")
                    f.write(f"- Time Taken: {time_elapsed:.4f} seconds\n")
                    f.write(f"- Iterations: {i+1}\n")
                    f.write(f"- Total Nodes Generated: {len(T.nodes)}\n") 
                    
                    # ✨ เพิ่มบรรทัดนี้ลงไปเพื่อโชว์ Cost
                    f.write(f"- Total Cost: {actual_total_cost:.2f}\n") 
                    
                    f.write(f"- Distance (Pixels): {path_cost:.2f} px\n")
                    f.write(f"- GVD Cost Used: {'Yes' if dis.use_gvd else 'No'}\n")
                    f.write(f"- Start: {dis.start_pt}, Goal: {dis.goal_pt}\n")
                    f.write(f"- Path Points ({len(path_coords)} points):\n  {path_coords}\n")
                    f.write(f"=========================================\n\n")
                print(f"📝 Successfully saved results to {log_file}")
            except Exception as e:
                print(f"❌ Failed to write to log: {e}")
                
        else:
            dis.ax.set_title("Path not found! Limit Reached.")
            
        print("\nรอผู้ใช้กด 'Clear Path' หรือ 'Clear Start/Goal' เพื่อเริ่มใหม่...")
        while not dis.should_reset:
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return

if __name__ == "__main__":
    img_path = r"/home/isaac/fra532-lab3-planning-por-wa/images/filtered_kmutt.png"
    type_map, img_rgb, dist_map = CreateMapFromImage(img_path, scale_percent=75)
    
    if type_map is not None:
        RRT_Star(type_map, img_rgb, dist_map)