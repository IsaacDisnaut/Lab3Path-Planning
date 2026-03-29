[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/yCAbp7gK)
# FRA532 Lab 3: Robot Path Planning 

## Algorithms Overview 
* **BFS (Breadth-First Search):** อัลกอริทึมที่ค้นหาเส้นทางโดยกระจายตัวออกไปทุกทิศทางรอบๆ ตัวอย่างเท่าเทียมกัน การันตีว่าจะพบเส้นทางที่สั้นที่สุดเสมอ (ในกรณีที่ทุกก้าวมีต้นทุนเท่ากัน) แต่มักจะใช้เวลาและหน่วยความจำในการคำนวณสูง  
* **Best-First Search (Greedy):** อัลกอริทึมที่เน้นพุ่งเป้าไปหาเป้าหมายให้เร็วที่สุด โดยพิจารณาจากค่า Heuristic (ระยะประเมินถึงเป้าหมาย) ทำงานได้รวดเร็วมาก แต่อาจจะได้เส้นทางที่ไม่อุดมคติหรือพาไปติดซอกตึกได้  
* **A-star:** อัลกอริทึมยอดนิยมที่ผสมผสานระหว่างระยะทางที่เดินมาแล้ว (Cost) และระยะทางที่คาดว่าจะไปถึง (Heuristic) ทำให้สามารถหาเส้นทางที่optimalี่สุดได้
* **RRT-star (Rapidly-exploring Random Tree Star):** อัลกอริทึมสายสุ่มที่สร้างกิ่งก้านสาขาไปยังพื้นที่ว่าง เหมาะสำหรับพื้นที่ที่มีขนาดกว้างและซับซ้อน (Continuous space) โดยเวอร์ชัน Star (*) จะมีการปรับปรุงเส้นทาง (Rewire) ให้สั้นและเรียบเนียนขึ้นเสมอเมื่อเวลาผ่านไป
* **Centerline Follow:** เทคนิคการตีเส้นทางให้อยู่กึ่งกลางระหว่างสิ่งกีดขวาง (Clearance) เพื่อนำมาประยุกต์ใช้ร่วมกับอัลกอริทึมหลัก ช่วยให้หุ่นยนต์รักษาระยะห่างที่ปลอดภัยจากกำแพง ลดความเสี่ยงในการชนขอบประตูหรือสิ่งกีดขวาง

---

## Part 1
### A*  
<img width="1536" height="1165" alt="A*nogvd" src="https://github.com/user-attachments/assets/18f5a3b9-8843-4280-9a3e-a2390be886e9" /><br>
with centerline follow<br>
<img width="1536" height="1165" alt="A*gvd" src="https://github.com/user-attachments/assets/23e8ab65-abf6-4aae-9912-58fd7ea02894" /><br><br>
### Best First  
<img width="1536" height="1165" alt="Bestfirstnogvd" src="https://github.com/user-attachments/assets/cad41c63-2336-4ad2-9425-13a28a6a7ddb" /><br>
<img width="1536" height="1165" alt="Bestfirstgvd" src="https://github.com/user-attachments/assets/bce54d13-87b5-431d-8a5a-b19b4dea8f60" /><br><br>
### BFS
<img width="1536" height="1165" alt="BFSnogvd" src="https://github.com/user-attachments/assets/395439e3-dfaf-434f-ac87-77547452f8da" /><br>
### RRT*
<img width="844" height="670" alt="rrt*nogvd5" src="https://github.com/user-attachments/assets/d0a75586-a243-4bc8-a701-be96d8f705e5" /><br>
<img width="844" height="670" alt="rrt*gvd6" src="https://github.com/user-attachments/assets/0d7dfb05-ba08-47a7-b2bc-a9a70fab0251" /><br>


---
## Part 2: 


---

## Part 3


---

## 📊 Discussion (สรุปและอภิปรายผล)
