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
        
        self.start_pt = (925, 486)
        self.goals = [
            (556, 325),
            (526, 628), 
            (248, 495), 
            (638, 455), 
            (352, 356), 
            (287, 418)  
        ] 
        self.cid = None
        self.run_flag = False
        self.should_reset = False 
        
        # --- Checkbox GVD ---
        self.ax_check = self.fig.add_axes([0.02, 0.80, 0.15, 0.1])
        self.check = CheckButtons(self.ax_check, ['Use GVD Cost'], [False])
        self.use_gvd = False
        
        def on_check_clicked(label):
            self.use_gvd = not self.use_gvd
        self.check.on_clicked(on_check_clicked)

        # --- ปุ่ม Clear All ---
        self.ax_btn_clear_all = self.fig.add_axes([0.02, 0.65, 0.15, 0.08]) 
        self.btn_clear_all = Button(self.ax_btn_clear_all, 'Clear All', color='lightcoral', hovercolor='red')
        
        def on_clear_all_clicked(event):
            self.should_reset = True
            self.start_pt = None
            self.goals = [] 
            self.run_flag = False
            self.redraw_base()
            self.ax.set_title("Cleared All! Right-Click = Start | Left-Click = Add Goals")
        self.btn_clear_all.on_clicked(on_clear_all_clicked)

        # --- ปุ่ม Clear Path ---
        self.ax_btn_clear_path = self.fig.add_axes([0.02, 0.55, 0.15, 0.08]) 
        self.btn_clear_path = Button(self.ax_btn_clear_path, 'Clear Path', color='lightskyblue', hovercolor='deepskyblue')
        
        def on_clear_path_clicked(event):
            self.should_reset = True
            self.run_flag = False
            self.redraw_base()
            self.ax.set_title("Path cleared! Points kept. Click 'Run Planning' to run again.")
        self.btn_clear_path.on_clicked(on_clear_path_clicked)

        # --- ปุ่ม Run Planning ---
        self.ax_btn_run = self.fig.add_axes([0.02, 0.45, 0.15, 0.08]) 
        self.btn_run = Button(self.ax_btn_run, 'Run Planning', color='lightgreen', hovercolor='lime')
        
        def on_run_clicked(event):
            if self.start_pt is not None and len(self.goals) > 0:
                self.run_flag = True
                self.ax.set_title("Planning Path with Bi-Directional Search...")
                self.fig.canvas.draw()
            else:
                self.ax.set_title("⚠️ Please set a Start point and at least 1 Goal first!")
                self.fig.canvas.draw()
        self.btn_run.on_clicked(on_run_clicked)

        self.redraw_base()
        self.ax.set_title("Points pre-selected! Add more goals (Left-Click) or click 'Run Planning'")

    def redraw_base(self):
        self.ax.clear() 
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.ax.imshow(self.bg_img)
        
        if self.start_pt is not None:
            self.ax.plot(self.start_pt[0], self.start_pt[1], 'go', markersize=10, zorder=6)
            self.ax.text(self.start_pt[0]+5, self.start_pt[1]+5, "Start", color='green', fontweight='bold', zorder=6)
            
        for i, g in enumerate(self.goals):
            goal_num = i + 1
            self.ax.plot(g[0], g[1], 'ro', markersize=8, zorder=5) 
            self.ax.text(g[0]+5, g[1]+5, f"G{goal_num}", color='red', fontweight='bold', zorder=5)
            
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

        if event.button == 3:
            if self.start_pt is None: 
                self.start_pt = (x, y)
                self.ax.plot(x, y, 'go', markersize=10, zorder=6)
                self.ax.text(x+5, y+5, "Start", color='green', fontweight='bold', zorder=6)
                print(f"Set Start at {self.start_pt}")
            
        elif event.button == 1: 
            self.goals.append((x, y))
            goal_num = len(self.goals)
            self.ax.plot(x, y, 'ro', markersize=8, zorder=5) 
            self.ax.text(x+5, y+5, f"G{goal_num}", color='red', fontweight='bold', zorder=5)
            print(f"Added Goal {goal_num} at {(x,y)}")

        self.fig.canvas.draw()

    def DrawNewEdge(self, parent_node, child_node, color='y-', marker='mo'):
        self.ax.plot([parent_node.x, child_node.x], [parent_node.y, child_node.y], color, linewidth=0.5, alpha=0.5)
        self.ax.plot(child_node.x, child_node.y, marker, markersize=1)

    def CalculateAndDrawBiPath(self, tree_start, node_start_idx, tree_goal, node_goal_idx):
        path_start = []
        s = node_start_idx
        while s in tree_start.parents and tree_start.parents[s] is not None:
            parent_idx = tree_start.parents[s][0]
            curr_node = tree_start.nodes[s]
            parent_node = tree_start.nodes[parent_idx]
            path_start.append((curr_node.x, curr_node.y))
            self.ax.plot([curr_node.x, parent_node.x], [curr_node.y, parent_node.y], 'b-', linewidth=1.0, zorder=4)
            s = parent_idx
        path_start.append((tree_start.nodes[s].x, tree_start.nodes[s].y))
        path_start = path_start[::-1] 
        
        path_goal = []
        s = node_goal_idx
        while s in tree_goal.parents and tree_goal.parents[s] is not None:
            parent_idx = tree_goal.parents[s][0]
            curr_node = tree_goal.nodes[s]
            parent_node = tree_goal.nodes[parent_idx]
            path_goal.append((curr_node.x, curr_node.y))
            self.ax.plot([curr_node.x, parent_node.x], [curr_node.y, parent_node.y], 'b-', linewidth=1.0, zorder=4)
            s = parent_idx
        path_goal.append((tree_goal.nodes[s].x, tree_goal.nodes[s].y))
        
        curr_A = tree_start.nodes[node_start_idx]
        curr_B = tree_goal.nodes[node_goal_idx]
        self.ax.plot([curr_A.x, curr_B.x], [curr_A.y, curr_B.y], 'b-', linewidth=1.0, zorder=4)
        
        self.fig.canvas.draw()
        return path_start + path_goal

# =====================================================================
def GetEdgeCost(n1, n2, dist_map, use_gvd):
    base_dist = n1.Dist(n2)
    if not use_gvd: return base_dist
    gvd_value = dist_map[n2.y, n2.x] 
    penalty = 1.0 - (0.8*gvd_value) 
    return base_dist * penalty
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
        dis.run_flag = False
        
        while not dis.run_flag:
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return
            if dis.should_reset: break 
            
        if dis.should_reset: continue 

        run_counter += 1
        dis.fig.canvas.mpl_disconnect(dis.cid) 
        
        current_start = dis.start_pt
        original_start = dis.start_pt 
        unvisited_goals = dis.goals.copy() 
        is_returning_home = False 
        
        total_time = 0.0
        total_distance = 0.0
        total_iterations = 0
        total_nodes = 0
        total_path_cost = 0.0 
        full_path_coords = []
        segment_num = 1
        
        print(f"\n🚀 Starting Bi-Directional Multi-Goal RRT* (Run #{run_counter})...")
        random.seed(0)
        np.random.seed(0)
        while (len(unvisited_goals) > 0 or not is_returning_home) and not dis.should_reset:
            
            if len(unvisited_goals) > 0:
                unvisited_goals.sort(key=lambda g: math.hypot(g[0] - current_start[0], g[1] - current_start[1]))
                current_goal = unvisited_goals.pop(0) 
                print(f"--- Segment {segment_num}: Planning {current_start} <--> {current_goal} ---")
            else:
                current_goal = original_start
                is_returning_home = True
                print(f"--- Segment {segment_num}: Returning HOME {current_start} <--> {current_goal} ---")
            
            T_start = Tree()
            T_start.SetRoot(current_start[0], current_start[1])
            
            T_goal = Tree()
            T_goal.SetRoot(current_goal[0], current_goal[1])
            
            T_A, T_B = T_start, T_goal
            is_A_start_tree = True
            
            node_A_connect = None
            node_B_connect = None
            
            start_time = time.time() 
            SEARCH_RADIUS = 20.0 
            
            for i in range(25000000):
                if dis.should_reset: break 

                target_pt = current_goal if is_A_start_tree else current_start
                pos_rand = GenRandomPos(w, h, target_pt)
                z_rand = Node(-1, pos_rand[0], pos_rand[1])
                z_nearest_A = FindNearest(T_A, z_rand)
                z_new_A = Steer(z_nearest_A, z_rand)
                
                if CollisionFree(grid, z_nearest_A, z_new_A):
                    near_nodes_A = []
                    for n in T_A.nodes:
                        if n.Dist(z_new_A) <= SEARCH_RADIUS and CollisionFree(grid, n, z_new_A):
                            near_nodes_A.append(n)
                    
                    min_cost_A = z_nearest_A.cost + GetEdgeCost(z_nearest_A, z_new_A, dist_map, dis.use_gvd)
                    best_parent_A = z_nearest_A
                    
                    for near_node in near_nodes_A:
                        cost = near_node.cost + GetEdgeCost(near_node, z_new_A, dist_map, dis.use_gvd)
                        if cost < min_cost_A:
                            min_cost_A = cost
                            best_parent_A = near_node
                    
                    new_node_A = T_A.AddNodes(z_new_A.x, z_new_A.y, cost=min_cost_A)
                    T_A.AddParent(new_node_A.index, best_parent_A.index, GetEdgeCost(best_parent_A, new_node_A, dist_map, dis.use_gvd))
                    
                    for near_node in near_nodes_A:
                        rewired_cost = new_node_A.cost + GetEdgeCost(new_node_A, near_node, dist_map, dis.use_gvd)
                        if rewired_cost < near_node.cost:
                            near_node.cost = rewired_cost
                            T_A.AddParent(near_node.index, new_node_A.index, GetEdgeCost(new_node_A, near_node, dist_map, dis.use_gvd))
                    
                    if i % 500 == 0:
                        color = 'y-' if is_A_start_tree else 'c-' 
                        dis.DrawNewEdge(best_parent_A, new_node_A, color=color)

                    z_nearest_B = FindNearest(T_B, new_node_A)
                    z_new_B = Steer(z_nearest_B, new_node_A)

                    if CollisionFree(grid, z_nearest_B, z_new_B):
                        near_nodes_B = []
                        for n in T_B.nodes:
                            if n.Dist(z_new_B) <= SEARCH_RADIUS and CollisionFree(grid, n, z_new_B):
                                near_nodes_B.append(n)
                        
                        min_cost_B = z_nearest_B.cost + GetEdgeCost(z_nearest_B, z_new_B, dist_map, dis.use_gvd)
                        best_parent_B = z_nearest_B
                        
                        for near_node in near_nodes_B:
                            cost = near_node.cost + GetEdgeCost(near_node, z_new_B, dist_map, dis.use_gvd)
                            if cost < min_cost_B:
                                min_cost_B = cost
                                best_parent_B = near_node
                        
                        new_node_B = T_B.AddNodes(z_new_B.x, z_new_B.y, cost=min_cost_B)
                        T_B.AddParent(new_node_B.index, best_parent_B.index, GetEdgeCost(best_parent_B, new_node_B, dist_map, dis.use_gvd))
                        
                        for near_node in near_nodes_B:
                            rewired_cost = new_node_B.cost + GetEdgeCost(new_node_B, near_node, dist_map, dis.use_gvd)
                            if rewired_cost < near_node.cost:
                                near_node.cost = rewired_cost
                                T_B.AddParent(near_node.index, new_node_B.index, GetEdgeCost(new_node_B, near_node, dist_map, dis.use_gvd))
                        
                        if i % 500 == 0:
                            color = 'c-' if is_A_start_tree else 'y-' 
                            dis.DrawNewEdge(best_parent_B, new_node_B, color=color)

                        if new_node_B.Dist(new_node_A) <= 10 and CollisionFree(grid, new_node_B, new_node_A):
                            if is_A_start_tree:
                                node_A_connect = new_node_A
                                node_B_connect = new_node_B
                            else:
                                node_A_connect = new_node_B
                                node_B_connect = new_node_A
                                
                            dis.DrawNewEdge(new_node_A, new_node_B, color='r-') 
                            total_iterations += (i + 1)
                            break 
                            
                T_A, T_B = T_B, T_A
                is_A_start_tree = not is_A_start_tree
                
                # ✨ จุดเปลี่ยนหลักอยู่ตรงนี้ครับ! อัปเดต Title แบบเรียลไทม์ทุกๆ 10 ลูป
                if i % 1 == 0: 
                    current_seg_nodes = len(T_start.nodes) + len(T_goal.nodes)
                    running_total = total_nodes + current_seg_nodes
                    print(f"Planning Segment {segment_num}... | Real-time Nodes: {running_total}")

            print()
            if dis.should_reset: break
            
            if node_A_connect is not None and node_B_connect is not None:
                end_time = time.time()
                time_elapsed = end_time - start_time
                total_time += time_elapsed
                
                segment_nodes = len(T_start.nodes) + len(T_goal.nodes)
                total_nodes += segment_nodes
                
                segment_path = dis.CalculateAndDrawBiPath(T_start, node_A_connect.index, T_goal, node_B_connect.index)
                
                segment_cost = 0.0
                for idx in range(len(segment_path) - 1):
                    n1 = Node(-1, segment_path[idx][0], segment_path[idx][1])
                    n2 = Node(-1, segment_path[idx+1][0], segment_path[idx+1][1])
                    segment_cost += n1.Dist(n2)
                    total_path_cost += GetEdgeCost(n1, n2, dist_map, use_gvd=dis.use_gvd)
                
                total_distance += segment_cost
                
                if len(full_path_coords) > 0:
                    full_path_coords.extend(segment_path[1:]) 
                else:
                    full_path_coords.extend(segment_path)
                
                print(f"✅ Segment {segment_num} found! Dist: {segment_cost:.2f} px | Nodes Generated: {segment_nodes}")
                
                current_start = current_goal
                segment_num += 1
            else:
                print(f"❌ Failed to reach {current_goal}")
                break
                
        if dis.should_reset: continue 
        
        if is_returning_home and node_A_connect is not None:
            title_text = f"Bi-RRT* Complete! | Run #{run_counter} | Time: {total_time:.2f}s | Dist: {total_distance:.2f}px | Total Nodes: {total_nodes}"
            dis.ax.set_title(title_text, color='blue', fontweight='bold')
            print(f"\n🎉 {title_text}")
            dis.fig.canvas.draw()
            
            try:
                with open("/home/isaac/fra532-lab3-planning-por-wa/data/datapart2/rrt*_bi.txt", "a", encoding="utf-8") as f:
                    f.write(f"=== Bi-RRT* Run Result #{run_counter} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===\n")
                    f.write(f"- Time Taken: {total_time:.4f} seconds\n")
                    f.write(f"- Iterations: {total_iterations}\n")
                    f.write(f"- Total Nodes Generated: {total_nodes}\n")
                    f.write(f"- Distance (Pixels): {total_distance:.2f} px\n")
                    f.write(f"- Total Cost: {total_path_cost:.2f}\n")
                    f.write(f"- GVD Checked during Planning: {'Yes' if dis.use_gvd else 'No'}\n")
                    f.write(f"- Path Points ({len(full_path_coords)} points):\n  {full_path_coords}\n")
                    f.write(f"=========================================\n\n")
                print("📝 Successfully saved results!")
            except Exception as e:
                print(f"❌ Failed to write to log: {e}")
            
        print("Waiting for user to click 'Clear Path' or 'Clear All' to restart...")
        while not dis.should_reset:
            plt.pause(0.1)
            if not plt.fignum_exists(dis.fig.number): return

if __name__ == "__main__":
    img_path = r"/home/isaac/fra532-lab3-planning-por-wa/images/filtered_kmutt.png"
    grid, img_rgb, dist_map = CreateMapFromImage(img_path, scale_percent=75)
    
    if grid is not None:
        RRT_Star(grid, img_rgb, dist_map)