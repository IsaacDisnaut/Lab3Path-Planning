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
with centerline follow<br>
<img width="1536" height="1165" alt="Bestfirstgvd" src="https://github.com/user-attachments/assets/bce54d13-87b5-431d-8a5a-b19b4dea8f60" /><br><br>
### BFS
<img width="1536" height="1165" alt="BFSnogvd" src="https://github.com/user-attachments/assets/395439e3-dfaf-434f-ac87-77547452f8da" /><br>
### RRT*
<img width="844" height="670" alt="rrt*nogvd5" src="https://github.com/user-attachments/assets/d0a75586-a243-4bc8-a701-be96d8f705e5" /><br>
with centerline follow<br>
<img width="844" height="670" alt="rrt*gvd6" src="https://github.com/user-attachments/assets/0d7dfb05-ba08-47a7-b2bc-a9a70fab0251" /><br>
<p align="center">
  <img width="490" height="561" alt="image" src="https://github.com/user-attachments/assets/09fd13e0-449c-4cb4-820f-858336ef5cc0" />
</p>

---
## Part 2: 
### A*  
![A*nogvd](https://github.com/user-attachments/assets/dd3b8e42-ecf6-4c5f-96d8-dc6e25c660f2)<br>
with center line follow<br>
![A*gvd](https://github.com/user-attachments/assets/bd61de8c-672b-407d-831b-5e62e89ad748)<br>
### Best First  
![Besfirstnogvd](https://github.com/user-attachments/assets/f7e3ffdc-ab05-440e-8f01-d9eaa8705c4d)<br>
with center line follow<br>
![Bestfirstgvd](https://github.com/user-attachments/assets/51e22dc2-f08b-4911-9af9-c9468bd7b05d)<br>
### BFS
![BFSgvd](https://github.com/user-attachments/assets/1c3d9f55-db11-4980-ba64-081d1744632a)<br>
### RRT*
<img width="814" height="679" alt="rrt*nogvd3" src="https://github.com/user-attachments/assets/bd31ea05-1850-440e-bb58-28619087273b" /><br>
with centerline follow<br>
<img width="844" height="670" alt="rrt*gvd3" src="https://github.com/user-attachments/assets/52756160-0065-426f-842e-9716beb5e318" /><br>

<p align="center">
  <img width="483" height="551" alt="image" src="https://github.com/user-attachments/assets/4dec8303-94b2-4be5-9e9f-4a6032a552b7" />
</p>

---

## Part 3
### A*  
<img width="1536" height="1165" alt="A*nogvd" src="https://github.com/user-attachments/assets/ce793824-ee6d-40d4-9ba4-c0401549e3ef" /><br>

with centerline follow<br>
<img width="1536" height="1165" alt="A*gvd" src="https://github.com/user-attachments/assets/69fb273a-e56d-4154-bf39-b66c432f3d0f" /><br><br>

### Best First  
<img width="1536" height="1165" alt="Bestfirstnogvd" src="https://github.com/user-attachments/assets/98a2d2a7-7e65-42f3-91cb-440efd5eae70" /><br>

with centerline follow<br>
<img width="1536" height="1165" alt="Bestfirstgvd" src="https://github.com/user-attachments/assets/9ba462ba-2263-48dd-98a7-c838246c8b35" /><br><br>

### BFS
<img width="1536" height="1165" alt="BFSnogvd" src="https://github.com/user-attachments/assets/a921f080-e1a4-47cc-b144-73b2c7c46fa9" /><br>

### RRT*
<img width="844" height="670" alt="rrt*nogvd" src="https://github.com/user-attachments/assets/302beebc-49b0-467b-9881-08e955793c74" /><br>

with centerline follow<br>
<img width="844" height="670" alt="rrt*gvd2" src="https://github.com/user-attachments/assets/a32c5cf1-52cd-41bb-a4f4-bc0fb192c4a6" /><br>

<p align="center">
  <img width="483" height="551" alt="image" src="https://github.com/user-attachments/assets/8a321b07-61ca-4ebb-8200-e8756b02c1fa" />
</p>

**Door cost =-5**<br>
### A*
<img width="1536" height="1165" alt="A*nogvdBP0DP5" src="https://github.com/user-attachments/assets/deb48318-303b-4ad9-9e5d-a905ef75fe8f" /><br>
with centerline follow<br>
<img width="1536" height="1165" alt="A*gvdBP0DP5" src="https://github.com/user-attachments/assets/9e3544c6-b9e4-4fd5-87ab-33be2757e488" /><br>
<p align="center">
  <img width="474" height="186" alt="image" src="https://github.com/user-attachments/assets/f8672db7-1352-4c96-9e90-e6940317b5d3" />

</p>

---

## Discussion (สรุปและอภิปรายผล)<br>
### Part1<br>
วิธีที่ใช้ระยะทางที่สั้นที่สุดคือ RRT* เนื่องจากไม่ได้เคลื่อนที่แบบที่ละพิกเซลจึงใช้ระยะทางเพียง 958.99 หากคิดเฉพาะวิฑีที่เดินทีละ พิกเซล A*และ Breadth firstจะเร็วที่สุดที่ 990.79 
ในด้านความเร็ว Best first จะมีความเร็วที่สุดที่สุดที่เวลา 0.06s เมื่อเปิดใช้งาน centerline follow จะพบว่า costลดลงเพราะระบบจะทำการลดcost เมื่อเส้นเข้าใกล้จุดกลาง
ระยะทางจะเพิ่มขึ้นเล็กน้อย และจะใช้เวลาคำนวณที่นานขึ้น
### Part2<br>
สำหรับระยะทางที่สั้นที่สุดยังคงเป็น RRT* ที่ระยะ 2382.81 ซึ่งสั้นกว่า A* และ Breadth first เกือบ500 พิกเซล และBest firstยังคงเป็นวิธีที่ใช้เวลาน้อยที่สุดที่ 0.62s 
เมื่อเปิดใช้งาน centerline follow A* ใช้เวลามากถึง 46.83วินาที และมีiterationถึง 1.8M iteration เนื่องจากระยะทางที่ไกลและต้องตัดสินใจเลือกระหว่างทางตรงชิดกำแพงหรือทางตรงกลางถนน ในขณะที่ Best first ใช้iterationเพียง 25227 ซึ่งน้อบลงไปเกือบ2เท่าเนื่องจากการที่เส้นวิ่งอยู่ตรงกลางสามารถช่วยลดการวิ่งอ้อมขอบถนนไม่ตอดขอบกำแพง
### Part3<br>
เมื่อกำหนดให้ทางเดินในอาคารมีค่าcostสูงกว่าทางถนนอยู่ 1 A* จะเป็นอัลกอริธึมเดียวที่เลือกที่จะเดินอ้อมเนื่องจากA*จะคำนวณเส้นทางที่optimal หรือค่าcostน้อยที่สุด ในขณะที่BFS และ Best first จะคำนวณจากจุดที่ระยะอยู่ใกล้ปลายทางที่สุดก่อน และRRT* ขึ้นอยู่กันการสุ่ม ทำให้บางครั้งเดินทะลุและบางครั้งเดินอ้อม 
