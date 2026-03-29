[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/yCAbp7gK)
# LAB3: Motion Planning

# FRA532 Lab 3: Robot Path Planning 🤖

โปรเจกต์นี้เป็นการทดสอบและเปรียบเทียบประสิทธิภาพของอัลกอริทึมค้นหาเส้นทาง (Path Planning Algorithms) ต่างๆ สำหรับหุ่นยนต์ โดยแบ่งการทดสอบออกเป็น 3 ภารกิจหลัก ตั้งแต่การวิ่งไปยังจุดหมาย ไปจนถึงการเคลื่อนที่ผ่านสิ่งกีดขวางและช่องประตูแคบๆ

## Algorithms Overview 

ในโปรเจกต์นี้ เราได้ทดสอบและเปรียบเทียบอัลกอริทึมดังต่อไปนี้:

* **BFS (Breadth-First Search):** อัลกอริทึมที่ค้นหาเส้นทางโดยกระจายตัวออกไปทุกทิศทางรอบๆ ตัวอย่างเท่าเทียมกัน การันตีว่าจะพบเส้นทางที่สั้นที่สุดเสมอ (ในกรณีที่ทุกก้าวมีต้นทุนเท่ากัน) แต่มักจะใช้เวลาและหน่วยความจำในการคำนวณสูง  
* **Best-First Search (Greedy):** อัลกอริทึมที่เน้นพุ่งเป้าไปหาเป้าหมายให้เร็วที่สุด โดยพิจารณาจากค่า Heuristic (ระยะประเมินถึงเป้าหมาย) ทำงานได้รวดเร็วมาก แต่อาจจะได้เส้นทางที่ไม่อุดมคติหรือพาไปติดซอกตึกได้  
* **A-star:** อัลกอริทึมยอดนิยมที่ผสมผสานระหว่างระยะทางที่เดินมาแล้ว (Cost) และระยะทางที่คาดว่าจะไปถึง (Heuristic) ทำให้สามารถหาเส้นทางที่สั้นที่สุดได้อย่างแม่นยำและมีประสิทธิภาพกว่า BFS
* **RRT-star (Rapidly-exploring Random Tree Star):** อัลกอริทึมสายสุ่มที่สร้างกิ่งก้านสาขาไปยังพื้นที่ว่าง เหมาะสำหรับพื้นที่ที่มีขนาดกว้างและซับซ้อน (Continuous space) โดยเวอร์ชัน Star (*) จะมีการปรับปรุงเส้นทาง (Rewire) ให้สั้นและเรียบเนียนขึ้นเสมอเมื่อเวลาผ่านไป
* **Centerline Follow:** เทคนิคการตีเส้นทางให้อยู่กึ่งกลางระหว่างสิ่งกีดขวาง (Clearance) เพื่อนำมาประยุกต์ใช้ร่วมกับอัลกอริทึมหลัก ช่วยให้หุ่นยนต์รักษาระยะห่างที่ปลอดภัยจากกำแพง ลดความเสี่ยงในการชนขอบประตูหรือสิ่งกีดขวาง

---

## Part 1
### Results (ผลการทดสอบ)

| BFS | Best-First | A* | RRT* |
|:---:|:---:|:---:|:---:|
| ![BFS Part1](images/part1_bfs.png) | ![BestFirst Part1](images/part1_bestfirst.png) | ![Astar Part1](images/part1_astar.png) | ![RRT Part1](images/part1_rrt.png) |

---

## Part 2: 


| ![BFS Part2](images/part2_bfs.png) | ![BestFirst Part2](images/part2_bestfirst.png) | ![Astar Part2](images/part2_astar.png) | ![RRT Part2](images/part2_rrt.png) |

---

## Part 3


---

## 📊 Discussion (สรุปและอภิปรายผล)
