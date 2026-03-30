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
    if abs(b - 213) < 15 and abs(g - 232) < 15 and abs(r - 242) < 15:
        return "BEIGE"
    return "WALL"
# =====================================================================
gvdcost = 0.8
'''Node class in the tree'''
class Node:
    def __init__(self, index, x, y, cost=0.0):
        self.index = index
        self.x = x
        self.y = y
        self.cost = cost 

    def Dist(self, others):
        return np.linalg.norm([self.x - others.x, self.y - others.y], 2)

'''Tree class'''
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

''' Class for Display and Interactive Clicking '''
class MapDisplay:
    def __init__(self, bg_img, grid):
        self.grid = grid
        self.bg_img = bg_img 
        
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_axes([0.2, 0.05, 0.75, 0.85]) 
        
        # กำหนดจุดตั้งต้น
        self.start_pt = (925, 486)
        self.goals = [(370, 588)] 
        
        # --- Checkbox GVD ---
        self.ax_check = self.fig.add_axes([0.02, 0.8, 0.15, 0.1])
        self.check = CheckButtons(self.ax_check, ['Use GVD Cost'], [False])
        self.use_gvd = False
        
        def on_check_clicked(label):
            self.use_gvd = not self.use_gvd
        self.check.on_clicked(on_check_clicked)

        # --- ปุ่ม Clear All (ลบทั้งหมด) ---
        self.ax_btn_clear_all = self.fig.add_axes([0.02, 0.65, 0.15, 0.08]) 
        self.btn_clear_all = Button(self.ax_btn_clear_all, 'Clear All', color='lightcoral', hovercolor='red')
        self.should_reset = False 
        
        def on_clear_all_clicked(event):
            self.should_reset = True
            self.start_pt = None
            self.goals = [] 
            self.run_flag = False
            self.redraw_base()
            self.ax.set_title("Right-Click = Start | Left-Click = Add Goals | Then click 'Run Planning'")
        self.btn_clear_all.on_clicked(on_clear_all_clicked)

        # --- ปุ่ม Clear Path (ลบแค่เส้น ไม่ลบจุด) ---
        self.ax_btn_clear_path = self.fig.add_axes([0.02, 0.55, 0.15, 0.08]) 
        self.btn_clear_path = Button(self.ax_btn_clear_path, 'Clear Path', color='lightskyblue', hovercolor='deepskyblue')
        self.should_clear_path = False 
        
        def on_clear_path_clicked(event):
            self.should_clear_path = True
            self.run_flag = False
            self.redraw_base()
            self.ax.set_title("Path cleared! Click 'Run Planning' to run again.")
        self.btn_clear_path.on_clicked(on_clear_path_clicked)

        # --- ปุ่ม Run Planning ---
        self.ax_btn_run = self.fig.add_axes([0.02, 0.45, 0.15, 0.08]) 
        self.btn_run = Button(self.ax_btn_run, 'Run Planning', color='lightgreen', hovercolor='lime')
        self.run_flag = False
        
        def on_run_clicked(event):
            if self.start_pt is not None and len(self.goals) > 0:
                self.run_flag = True
                self.ax.set_title("Planning Path to Multiple Goals...")
                self.fig.canvas.draw()
            else:
                self.ax.set_title("⚠️ Please set a Start point and at least 1 Goal first!")
                self.fig.canvas.draw()
        self.btn_run.on_clicked(on_run_clicked)

        # วาดภาพเริ่มต้น
        self.cid = None
        self.redraw_base()
        self.ax.set_title("Points pre-selected! Add more goals (Left-Click) or click 'Run Planning'")

    def redraw_base(self):
        """ล้างหน้าจอและวาดพื้นหลังพร้อมจุด Start/Goal ใหม่"""
        self.ax.clear() 
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.ax.imshow(self.bg_img)
        
        # วาด Start
        if self.start_pt is not None:
            self.ax.plot(self.start_pt[0], self.start_pt[1], 'go', markersize=10, zorder=6)
            self.ax.text(self.start_pt[0]+5, self.start_pt[1]+5, "Start", color='green', fontweight='bold', zorder=6)
            
        # วาด Goals
        for i, g in enumerate(self.goals):
            goal_num = i + 1
            self.ax.plot(g[0], g[1], 'ro', markersize=8, zorder=5) 
            self.ax.text(g[0]+5, g[1]+5, f"G{goal_num}", color='red', fontweight='bold', zorder=5)
            
        # เชื่อมต่อ Event การคลิกใหม่
        if self.cid is not None:
            self.fig.canvas.mpl_disconnect(self.cid)
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.draw()

    def onclick(self, event):
        if event.inaxes != self.ax: return 
        if event.xdata is None or event.ydata is None: return
        x, y = int(event.xdata), int(event.ydata)
        
        if self.grid[y, x] == 1:
            self.ax.set_title("⚠️ Obstacle! Please click exactly on the BEIGE path.")
            self.fig.canvas.draw()
            return

        if event.button == 3: # Right click -> Start
            if self.start_pt is None: 
                self.start_pt = (x, y)
                self.ax.plot(x, y, 'go', markersize=10, zorder=6)
                self.ax.text(x+5, y+5, "Start", color='green', fontweight='bold', zorder=6)
                print(f"Set Start at {self.start_pt}")
            
        elif event.button == 1: # Left click -> Add Multiple Goals
            self.goals.append((x, y))
            goal_num = len(self.goals)
            self.ax.plot(x, y, 'ro', markersize=8, zorder=5) 
            self.ax.text(x+5, y+5, f"G{goal_num}", color='red', fontweight='bold', zorder=5)
            print(f"Added Goal {goal_num} at {(x,y)}")

        self.fig.canvas.draw()

    def DrawNewEdge(self, parent_node, child_node):
        self.ax.plot([parent_node.x, child_node.x], [parent_node.y, child_node.y], 'y-', linewidth=0.5, alpha=0.5)
        self.ax.plot(child_node.x, child_node.y, 'mo', markersize=1)

    def CalculateAndDrawPath(self, tree, s):
        path_cost = 0
        path_nodes = [] 
        while s in tree.parents.keys() and tree.parents[s] is not None:
            parent_idx = tree.parents[s][0]
            curr_node = tree.nodes[s]
            parent_node = tree.nodes[parent_idx]
            
            path_nodes.append((curr_node.x, curr_node.y))
            
            self.ax.plot([curr_node.x, parent_node.x], [curr_node.y, parent_node.y], 'b-', linewidth=1.5, zorder=4)
            path_cost += curr_node.Dist(parent_node)
            s = parent_idx
            
        path_nodes.append((tree.nodes[s].x, tree.nodes[s].y)) 
        self.fig.canvas.draw()
        return path_cost, path_nodes[::-1] 

# =====================================================================
def GetEdgeCost(n1, n2, dist_map, use_gvd):
    base_dist = n1.Dist(n2)
    if not use_gvd: return base_dist 
    gvd_value = dist_map[n2.y, n2.x] 
    penalty =  - (gvdcost*gvd_value) 
    return base_dist + penalty
# =====================================================================

def CreateMapFromImage(image_path, scale_percent=75):
    img = cv2.imread(image_path)
    if img is None: return None, None, None
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    img = cv2.resize(img, (width, height))
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    grid = np.ones((height, width), dtype=np.uint8) 
    for y in range(height):
        for x in range(width):
            if get_pixel_type(img[y, x]) == "BEIGE": grid[y, x] = 0
                
    free_space = np.uint8(grid == 0) * 255
    dist_map = cv2.distanceTransform(free_space, cv2.DIST_L2, 5)
    cv2.normalize(dist_map, dist_map, 0, 1.0, cv2.NORM_MINMAX)
    return grid, img_rgb, dist_map

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

def CollisionFree(grid, n1, n2):
    dist = n1.Dist(n2)
    if dist == 0: return True
    steps = max(1, int(dist))
    for i in range(steps + 1):
        frac = i / steps
        x_chk = int(round((1 - frac) * n1.x + frac * n2.x))
        y_chk = int(round((1 - frac) * n1.y + frac * n2.y))
        if x_chk < 0 or x_chk >= grid.shape[1] or y_chk < 0 or y_chk >= grid.shape[0]: return False
        if grid[y_chk, x_chk] == 1: return False
    return True   

def RRT_Star(grid, img_rgb, dist_map):
    h, w = grid.shape
    dis = MapDisplay(img_rgb, grid)
    
    plt.ion()
    plt.show()

    run_counter = 0 

    while plt.fignum_exists(dis.fig.number):
        dis.should_reset = False
        dis.should_clear_path = False
        dis.run_flag = False
        
        while not dis.run_flag:
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return
            if dis.should_reset or dis.should_clear_path: 
                dis.should_reset = False
                dis.should_clear_path = False
                
        run_counter += 1 
            
        dis.fig.canvas.mpl_disconnect(dis.cid) 
        
        current_start = dis.start_pt
        unvisited_goals = dis.goals.copy() 
        
        total_time = 0.0
        total_distance = 0.0
        total_iterations = 0
        total_nodes = 0      # ✨ เพิ่มตัวแปรเก็บจำนวน Node รวม
        total_path_cost = 0.0 
        full_path_coords = []
        segment_num = 1
        
        print(f"\n🚀 Starting Multi-Goal RRT* (Run #{run_counter})...")
        random.seed(0)
        np.random.seed(0)
        while len(unvisited_goals) > 0 and not (dis.should_reset or dis.should_clear_path):
            
            unvisited_goals.sort(key=lambda g: math.hypot(g[0] - current_start[0], g[1] - current_start[1]))
            current_goal = unvisited_goals.pop(0) 
            
            print(f"--- Segment {segment_num}: Planning to {current_goal} ---")
            
            T = Tree()
            T.SetRoot(current_start[0], current_start[1])
            goal_node = None
            
            start_time = time.time() 
            SEARCH_RADIUS = 20.0 
            
            for i in range(25000000):
                if dis.should_reset or dis.should_clear_path: break 

                pos_rand = GenRandomPos(w, h, current_goal)
                z_rand = Node(-1, pos_rand[0], pos_rand[1])
                z_nearest = FindNearest(T, z_rand)
                z_new = Steer(z_nearest, z_rand)
                
                if CollisionFree(grid, z_nearest, z_new):
                    near_nodes = []
                    for n in T.nodes:
                        if n.Dist(z_new) <= SEARCH_RADIUS and CollisionFree(grid, n, z_new):
                            near_nodes.append(n)
                    
                    min_cost = z_nearest.cost + GetEdgeCost(z_nearest, z_new, dist_map, dis.use_gvd)
                    best_parent = z_nearest
                    
                    for near_node in near_nodes:
                        cost = near_node.cost + GetEdgeCost(near_node, z_new, dist_map, dis.use_gvd)
                        if cost < min_cost:
                            min_cost = cost
                            best_parent = near_node
                    
                    new_node = T.AddNodes(z_new.x, z_new.y, cost=min_cost)
                    T.AddParent(new_node.index, best_parent.index, GetEdgeCost(best_parent, new_node, dist_map, dis.use_gvd))
                    
                    if len(T.nodes) % 1 == 0: 
                        print(f"\rNodes in segment tree: {len(T.nodes)}", end="") # ปรับให้ปริ้นท์ทับบรรทัดเดิมจะได้ไม่รก Terminal
                    
                    for near_node in near_nodes:
                        rewired_cost = new_node.cost + GetEdgeCost(new_node, near_node, dist_map, dis.use_gvd)
                        if rewired_cost < near_node.cost:
                            near_node.cost = rewired_cost
                            T.AddParent(near_node.index, new_node.index, GetEdgeCost(new_node, near_node, dist_map, dis.use_gvd))
                    
                    
                    dis.DrawNewEdge(best_parent, new_node)
                    if i % 2 == 0: 
                        plt.pause(0.001)
                    
                    if new_node.Dist(Node(-1, current_goal[0], current_goal[1])) <= 10:
                        if CollisionFree(grid, new_node, Node(-1, current_goal[0], current_goal[1])):
                            final_cost = new_node.cost + GetEdgeCost(new_node, Node(-1, current_goal[0], current_goal[1]), dist_map, dis.use_gvd)
                            final_node = T.AddNodes(current_goal[0], current_goal[1], cost=final_cost)
                            T.AddParent(final_node.index, new_node.index, GetEdgeCost(new_node, final_node, dist_map, dis.use_gvd))
                            dis.DrawNewEdge(new_node, final_node)
                            goal_node = final_node
                            total_iterations += (i + 1)
                            break 
            
            print() # ขึ้นบรรทัดใหม่หลังออกจาก Loop
            if dis.should_reset or dis.should_clear_path: break
            
            if goal_node is not None:
                end_time = time.time()
                time_elapsed = end_time - start_time
                total_time += time_elapsed
                total_nodes += len(T.nodes) # ✨ นำจำนวน Node ในแต่ละ Segment มาบวกสะสม
                
                segment_cost, segment_path = dis.CalculateAndDrawPath(T, goal_node.index)
                total_distance += segment_cost
                
                for idx in range(len(segment_path) - 1):
                    n1 = Node(-1, segment_path[idx][0], segment_path[idx][1])
                    n2 = Node(-1, segment_path[idx+1][0], segment_path[idx+1][1])
                    total_path_cost += GetEdgeCost(n1, n2, dist_map, use_gvd=dis.use_gvd)
                
                if len(full_path_coords) > 0:
                    full_path_coords.extend(segment_path[1:]) 
                else:
                    full_path_coords.extend(segment_path)
                
                print(f"✅ Segment {segment_num} found! Dist: {segment_cost:.2f} px")
                
                current_start = current_goal
                segment_num += 1
            else:
                print(f"❌ Failed to reach {current_goal}")
                break
                
        if dis.should_reset or dis.should_clear_path: continue 
        
        if len(unvisited_goals) == 0:
            title_text = f"All Goals Reached! | Run #{run_counter} | Time: {total_time:.2f} s | Dist: {total_distance:.2f} px"
            dis.ax.set_title(title_text, color='blue', fontweight='bold')
            print(f"\n🎉 {title_text}")
            dis.fig.canvas.draw()
            
            try:
                with open("/home/isaac/fra532-lab3-planning-por-wa/data/datapart1/rrt*.txt", "a", encoding="utf-8") as f:
                    f.write(f"=== RRT* Run Result #{run_counter} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===\n")
                    f.write(f"- Time Taken: {total_time:.4f} seconds\n")
                    f.write(f"- Iterations: {total_iterations}\n")
                    f.write(f"- Total Nodes Generated: {total_nodes}\n") # ✨ เพิ่มบันทึกจำนวน Node ลง Log
                    f.write(f"- Distance (Pixels): {total_distance:.2f} px\n")
                    f.write(f"- Total Cost: {total_path_cost:.2f}\n")
                    f.write(f"- GVD Checked during Planning: {'Yes' if dis.use_gvd else 'No'}\n")
                    f.write(f"- Path Points ({len(full_path_coords)} points):\n  {full_path_coords}\n")
                    f.write(f"=========================================\n\n")
                print("📝 Successfully saved results to log.txt")
            except Exception as e:
                print(f"❌ Failed to write to log.txt: {e}")
            
        print("\nWaiting for user to click 'Clear Path' or 'Clear All'...")
        while not (dis.should_reset or dis.should_clear_path):
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return

if __name__ == "__main__":
    img_path = r"/home/isaac/fra532-lab3-planning-por-wa/images/filtered_kmutt.png"
    grid, img_rgb, dist_map = CreateMapFromImage(img_path, scale_percent=75)
    
    if grid is not None:
        RRT_Star(grid, img_rgb, dist_map)