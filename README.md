[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/yCAbp7gK)
# FRA532 Lab 3: Robot Path Planning 

## Algorithms Overview 
### A-star 
&emsp;อัลกอริทึมที่ผสมระหว่างระยะทางที่เดินมาแล้ว (Cost) และระยะทางที่คาดว่าจะไปถึง (Heuristic) ทำให้สามารถหาเส้นทางที่optimalที่สุดได้<br><br>
$$f(n)= g(n)+h(n)$$

* **$f(n)$**: ต้นทุนรวมที่คาดการณ์ (Estimated Total Cost) อัลกอริทึมจะเลือกขยายโหนดที่มีค่า $f(n)$ ต่ำที่สุดเสมอ
* **$g(n)$**: ต้นทุนสะสมจริง (Actual Cost) ที่ใช้ในการเดินทางจากจุดเริ่มต้น (Start) มาจนถึงโหนดปัจจุบัน $n$ (รวมถึงค่าปรับหรือ Penalty Cost ต่างๆ บนเส้นทาง)
* **$h(n)$**: ต้นทุนคาดการณ์ (Heuristic Cost) จากโหนดปัจจุบัน $n$ ไปยังจุดหมาย (Goal) โดยทั่วไปใช้ระยะทางยุคลิด (Euclidean Distance)<br>
### Best-First Search
&emsp;อัลกอริทึมที่เน้นพุ่งเป้าไปหาเป้าหมายให้เร็วที่สุด โดยพิจารณาจากค่า Heuristic (ระยะประเมินถึงเป้าหมาย) ทำงานได้รวดเร็วมาก แต่อาจจะได้เส้นทางที่ไม่อุดมคติหรือพาไปติดซอกตึกได้ <br><br>
$$f(n)=h(n)$$
* **$h(n)$**: ต้นทุนคาดการณ์ (Heuristic Cost) จากโหนดปัจจุบันไปยังจุดเป้าหมาย
* **ข้อสังเกต:** การละทิ้งตัวแปร $g(n)$ ทำให้อัลกอริทึมนี้ทำงานได้รวดเร็วมาก แต่อาจนำไปสู่เส้นทางที่ไม่ใช่ระยะทางที่สั้นที่สุด (Sub-optimal Path) และมีแนวโน้มที่จะตัดผ่านอุปสรรคหากการประเมินชี้ว่าทำให้เข้าใกล้เป้าหมายมากขึ้น<br>
### BFS (Breadth-First Search)
&emsp;อัลกอริทึมที่ค้นหาเส้นทางโดยกระจายตัวออกไปทุกทิศทางรอบๆ ตัวอย่างเท่าเทียมกัน การันตีว่าจะพบเส้นทางที่สั้นที่สุดเสมอ (ในกรณีที่ทุกก้าวมีต้นทุนเท่ากัน) แต่มักจะใช้เวลาและหน่วยความจำในการคำนวณสูง <br><br>
$$f(n)=g(n)$$
* **$g(n)$**: จำนวนก้าวหรือต้นทุนสะสมจากจุดเริ่มต้น โดยละทิ้งตัวแปร $h(n)$ เนื่องจากไม่มีการประเมินระยะทางไปยังเป้าหมาย
* **ข้อสังเกต:** BFS รับประกันการค้นพบเส้นทางที่สั้นที่สุดในเชิงจำนวนก้าว (บนกราฟที่ไม่มีการถ่วงน้ำหนัก) แต่ต้องแลกมากับการใช้ทรัพยากรในการประมวลผลและหน่วยความจำที่สูงมากเมื่อขนาดของแผนที่ใหญ่ขึ้น<br>

### RRT-star (Rapidly-exploring Random Tree Star)<br>
&emsp;อัลกอริทึมสายสุ่มที่สร้างกิ่งก้านสาขาไปยังพื้นที่ว่าง เหมาะสำหรับพื้นที่ที่มีขนาดกว้างและซับซ้อน (Continuous space) โดยเวอร์ชัน Star (*) จะมีการปรับปรุงเส้นทาง (Rewire) ให้สั้นและเรียบเนียนขึ้นเสมอเมื่อเวลาผ่านไป<br><br>
$$C(x_{new})=\min_{x_{near}\in X_{near}}\{C(x_{near})+c(x_{near},x_{new})\}$$
<br>สุ่มหยิบโหนดเพื่อนบ้านแต่ละตัว ($x_{near​}$) ที่อยู่ในกลุ่มเพื่อนบ้านทั้งหมด ($X_{near}$​) มาคำนวณหาค่า Cost ทีละตัว แล้วเลือกตัวที่ให้ผลลัพธ์ที่ 'น้อยที่สุด' ออกมาใช้งาน
* **$x_{new}$**: โหนดพิกัดใหม่ที่เพิ่งสุ่มได้และผ่านการตรวจสอบการชนกับอุปสรรคแล้ว
* **$x_{near}$**: กลุ่มของโหนดเดิมที่อยู่ในรัศมีที่กำหนดรอบๆ โหนด $x_{new}$
* **$C(x)$**: ต้นทุนรวม (ระยะทาง) จากจุดเริ่มต้นมาจนถึงโหนด $x$ 
* **$c(x_{near},x_{new})$**: ต้นทุนในการเคลื่อนที่ (ระยะทางเส้นตรง) จากโหนด $x_{near}$ ไปยังโหนด $x_{new}$
* **ข้อสังเกต:** สมการนี้ทำให้ RRT* สามารถปรับแก้โครงสร้างของเส้นทางให้สั้นลงอย่างต่อเนื่อง (Asymptotic Optimality) ส่งผลให้ได้เส้นทางที่สั้นและตรงที่สุดในพื้นที่ว่าง<br>

### Centerline Follow<br> 
&emsp;เป็นการตีเส้นทางให้อยู่กึ่งกลางระหว่างสิ่งกีดขวาง (Clearance) เพื่อนำมาประยุกต์ใช้ร่วมกับอัลกอริทึมหลัก ช่วยให้หุ่นยนต์รักษาระยะห่างที่ปลอดภัยจากกำแพง ลดความเสี่ยงในการชนขอบประตูหรือสิ่งกีดขวาง<br><br>
$$Cost = cost_{step}+(-Bonus*R_{norm})$$
<br><br>$$R_{norm}=\frac{R-R_{min}}{R_{max}-R_{min}}$$
* **$Cost$**: ค่าcostรวมของแต่ละพิกเซล
* **$cost_{step}$**:ค่าcost ในแต่ละการเดิน เช่น 1หรือ 1.414
* **$Bonus$**: ค่าตัวคูณสำหรับลดค่าcost
* **$R_{norm}$**: สัดส่วนระยะที่สั้นที่สุดจากพิกเซลทางเดินจนถึงพิกเซล/nodeกำแพงที่ใกล้ที่สุดมีค่า 0-1
* **$R$**: ระยะจากพิกเซล/node ปัจจุบันถึงกำแพงที่ใกล้ที่สุด
* **$R_{min}$**: ระยะจากพิกเซล/node จนถึงกำแพงที่มีค่าน้อยที่สุดในmap ในที่นี้คือ 0
* **$R_{max}$**: ระยะจากพิกเซล/node จนถึงกำแพงที่มีค่ามากที่สุดในmap 
---

## Part 1
### A*  
<img width="1536" height="1165" alt="A*nogvd" src="https://github.com/user-attachments/assets/18f5a3b9-8843-4280-9a3e-a2390be886e9" /><br>
with centerline follow<br>
<img width="1536" height="1165" alt="Map Navigation_screenshot_30 03 2026" src="https://github.com/user-attachments/assets/1a46b9dd-9159-49f4-819b-45ec94c06a9f" /><br><br>

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

[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/yCAbp7gK)
# FRA532 Lab 3: Robot Path Planning 

## Algorithms Overview 
### A-star 
&emsp;อัลกอริทึมที่ผสมระหว่างระยะทางที่เดินมาแล้ว (Cost) และระยะทางที่คาดว่าจะไปถึง (Heuristic) ทำให้สามารถหาเส้นทางที่optimalที่สุดได้<br><br>
$$f(n)= g(n)+h(n)$$

* **$f(n)$**: ต้นทุนรวมที่คาดการณ์ (Estimated Total Cost) อัลกอริทึมจะเลือกขยายโหนดที่มีค่า $f(n)$ ต่ำที่สุดเสมอ
* **$g(n)$**: ต้นทุนสะสมจริง (Actual Cost) ที่ใช้ในการเดินทางจากจุดเริ่มต้น (Start) มาจนถึงโหนดปัจจุบัน $n$ (รวมถึงค่าปรับหรือ Penalty Cost ต่างๆ บนเส้นทาง)
* **$h(n)$**: ต้นทุนคาดการณ์ (Heuristic Cost) จากโหนดปัจจุบัน $n$ ไปยังจุดหมาย (Goal) โดยทั่วไปใช้ระยะทางยุคลิด (Euclidean Distance)<br>
### Best-First Search
&emsp;อัลกอริทึมที่เน้นพุ่งเป้าไปหาเป้าหมายให้เร็วที่สุด โดยพิจารณาจากค่า Heuristic (ระยะประเมินถึงเป้าหมาย) ทำงานได้รวดเร็วมาก แต่อาจจะได้เส้นทางที่ไม่อุดมคติหรือพาไปติดซอกตึกได้ <br><br>
$$f(n)=h(n)$$
* **$h(n)$**: ต้นทุนคาดการณ์ (Heuristic Cost) จากโหนดปัจจุบันไปยังจุดเป้าหมาย
* **ข้อสังเกต:** การละทิ้งตัวแปร $g(n)$ ทำให้อัลกอริทึมนี้ทำงานได้รวดเร็วมาก แต่อาจนำไปสู่เส้นทางที่ไม่ใช่ระยะทางที่สั้นที่สุด (Sub-optimal Path) และมีแนวโน้มที่จะตัดผ่านอุปสรรคหากการประเมินชี้ว่าทำให้เข้าใกล้เป้าหมายมากขึ้น<br>
### BFS (Breadth-First Search)
&emsp;อัลกอริทึมที่ค้นหาเส้นทางโดยกระจายตัวออกไปทุกทิศทางรอบๆ ตัวอย่างเท่าเทียมกัน การันตีว่าจะพบเส้นทางที่สั้นที่สุดเสมอ (ในกรณีที่ทุกก้าวมีต้นทุนเท่ากัน) แต่มักจะใช้เวลาและหน่วยความจำในการคำนวณสูง <br><br>
$$f(n)=g(n)$$
* **$g(n)$**: จำนวนก้าวหรือต้นทุนสะสมจากจุดเริ่มต้น โดยละทิ้งตัวแปร $h(n)$ เนื่องจากไม่มีการประเมินระยะทางไปยังเป้าหมาย
* **ข้อสังเกต:** BFS รับประกันการค้นพบเส้นทางที่สั้นที่สุดในเชิงจำนวนก้าว (บนกราฟที่ไม่มีการถ่วงน้ำหนัก) แต่ต้องแลกมากับการใช้ทรัพยากรในการประมวลผลและหน่วยความจำที่สูงมากเมื่อขนาดของแผนที่ใหญ่ขึ้น<br>

### RRT-star (Rapidly-exploring Random Tree Star)<br>
&emsp;อัลกอริทึมสายสุ่มที่สร้างกิ่งก้านสาขาไปยังพื้นที่ว่าง เหมาะสำหรับพื้นที่ที่มีขนาดกว้างและซับซ้อน (Continuous space) โดยเวอร์ชัน Star (*) จะมีการปรับปรุงเส้นทาง (Rewire) ให้สั้นและเรียบเนียนขึ้นเสมอเมื่อเวลาผ่านไป<br><br>
$$C(x_{new})=\min_{x_{near}\in X_{near}}\{C(x_{near})+c(x_{near},x_{new})\}$$
<br>สุ่มหยิบโหนดเพื่อนบ้านแต่ละตัว ($x_{near​}$) ที่อยู่ในกลุ่มเพื่อนบ้านทั้งหมด ($X_{near}$​) มาคำนวณหาค่า Cost ทีละตัว แล้วเลือกตัวที่ให้ผลลัพธ์ที่ 'น้อยที่สุด' ออกมาใช้งาน
* **$x_{new}$**: โหนดพิกัดใหม่ที่เพิ่งสุ่มได้และผ่านการตรวจสอบการชนกับอุปสรรคแล้ว
* **$x_{near}$**: กลุ่มของโหนดเดิมที่อยู่ในรัศมีที่กำหนดรอบๆ โหนด $x_{new}$
* **$C(x)$**: ต้นทุนรวม (ระยะทาง) จากจุดเริ่มต้นมาจนถึงโหนด $x$ 
* **$c(x_{near},x_{new})$**: ต้นทุนในการเคลื่อนที่ (ระยะทางเส้นตรง) จากโหนด $x_{near}$ ไปยังโหนด $x_{new}$
* **ข้อสังเกต:** สมการนี้ทำให้ RRT* สามารถปรับแก้โครงสร้างของเส้นทางให้สั้นลงอย่างต่อเนื่อง (Asymptotic Optimality) ส่งผลให้ได้เส้นทางที่สั้นและตรงที่สุดในพื้นที่ว่าง<br>

### Centerline Follow<br> 
&emsp;เป็นการตีเส้นทางให้อยู่กึ่งกลางระหว่างสิ่งกีดขวาง (Clearance) เพื่อนำมาประยุกต์ใช้ร่วมกับอัลกอริทึมหลัก ช่วยให้หุ่นยนต์รักษาระยะห่างที่ปลอดภัยจากกำแพง ลดความเสี่ยงในการชนขอบประตูหรือสิ่งกีดขวาง<br><br>
$$Cost = cost_{step}+(-Bonus*R_{norm})$$
<br><br>$$R_{norm}=\frac{R-R_{min}}{R_{max}-R_{min}}$$
* **$Cost$**: ค่าcostรวมของแต่ละพิกเซล
* **$cost_{step}$**:ค่าcost ในแต่ละการเดิน เช่น 1หรือ 1.414
* **$Bonus$**: ค่าตัวคูณสำหรับลดค่าcost
* **$R_{norm}$**: สัดส่วนระยะที่สั้นที่สุดจากพิกเซลทางเดินจนถึงพิกเซล/nodeกำแพงที่ใกล้ที่สุดมีค่า 0-1
* **$R$**: ระยะจากพิกเซล/node ปัจจุบันถึงกำแพงที่ใกล้ที่สุด
* **$R_{min}$**: ระยะจากพิกเซล/node จนถึงกำแพงที่มีค่าน้อยที่สุดในmap ในที่นี้คือ 0
* **$R_{max}$**: ระยะจากพิกเซล/node จนถึงกำแพงที่มีค่ามากที่สุดในmap 
---

## Part 1
### A*  
<img width="1536" height="1165" alt="A*nogvd" src="https://github.com/user-attachments/assets/18f5a3b9-8843-4280-9a3e-a2390be886e9" /><br>
with centerline follow<br>
![alt text](<data/datapart1/Map Navigation_screenshot_30.03.2026.png>)<br><br>
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

## Discussion

การทดลองนี้ได้ประเมินประสิทธิภาพของอัลกอริทึมค้นหาเส้นทาง 4 รูปแบบ ได้แก่ A*, Best-First Search, Breadth-First Search (BFS) และ RRT* ในหลากหลายสถานการณ์ การศึกษานี้เน้นวิเคราะห์ผลกระทบของการนำเทคนิค **"Centerline Follow"** มาใช้ ซึ่งเป็น Cost modifiersเพื่อจำลองเส้นทางอุดมคติแบบ Centerline 

เส้นทางแบบ Centerline อุดมคตินี้จะเพิ่มความปลอดภัยสูงสุดโดยการรักษาระยะห่างจากสิ่งกีดขวางทั้งหมดให้เท่าๆกัน ผลลัพธ์ที่ได้ชี้ให้เห็นถึงความได้เปรียบเสียเปรียบ (Trade-offs) อย่างชัดเจนระหว่างประสิทธิภาพในการคำนวณ ความสมบูรณ์แบบของเส้นทาง (Optimality) และความสามารถในการปรับตัวเข้ากับสภาพแวดล้อม

### 1. พฤติกรรมพื้นฐานของอัลกอริทึมและลักษณะเส้นทาง
เมื่อไม่มีการปรับแต่งด้วยเทคนิค Centerline Follow อัลกอริทึมต่างๆ ทำงานได้ตรงตามทฤษฎีการออกแบบ:
* **A*:** สามารถคำนวณหาเส้นทางที่ดีที่สุด (Optimal path) โดยอิงจากต้นทุนรวมได้อย่างสม่ำเสมอ ผ่านการรักษาสมดุลระหว่างระยะทางที่เดินจริง ($g(n)$) และค่าความน่าจะเป็นหรือฮิวริสติก ($h(n)$)
* **Best-First Search:** พิสูจน์ให้เห็นว่าเป็นอัลกอริทึมที่คำนวณได้เร็วที่สุด (เช่น 0.06 วินาที ในสถานการณ์ที่ 1) โดยการมุ่งเน้นไปที่ค่าฮิวริสติกอย่างหนักหน่วง แม้ว่าวิธีการแบบละโมบ (Greedy approach) นี้จะทำให้ได้เส้นทางที่ไม่ดีที่สุดและมักจะเดินชิดกำแพงก็ตาม
* **BFS:** สามารถหาระยะทางกายภาพที่สั้นที่สุดบนแผนที่แบบไม่มีการถ่วงน้ำหนักได้ (เช่น ทำระยะได้ 990.79px เท่ากับ A* ในสถานการณ์ที่ 1) แต่ต้องแลกมากับภาระการคำนวณที่มหาศาล (174,174 รอบ) เนื่องจากการค้นหาแบบปูพรมทุกทิศทางโดยไม่มีค่าฮิวริสติกมาช่วยนำทาง
* **RRT*:** ซึ่งทำงานในพื้นที่ต่อเนื่อง (Continuous space) สามารถก้าวข้ามข้อจำกัดของโครงสร้างแบบตาราง (Grid) และหาระยะทางระหว่างจุดต่อจุดที่สั้นและเป็นธรรมชาติกว่าได้สำเร็จ (958.99px ในสถานการณ์ที่ 1)

### 2. ผลกระทบของเส้นทางอุดมคติแบบ Centerline
การผสานรวมเทคนิค Centerline Follow ทำให้เกิดความแตกต่างอย่างสุดขั้วในประสิทธิภาพของอัลกอริทึมที่ใช้ค่าฮิวริสติก การให้ "โบนัสส่วนลด" (Cost bonus) แก่โหนดที่อยู่ห่างจากสิ่งกีดขวาง ทำให้โครงสร้างต้นทุนของแผนที่เปลี่ยนไปเพื่อดึงเส้นทางให้เข้าใกล้จุดอุดมคติแบบ Centerline

* **ประสิทธิภาพที่ลดลงของ A*:** โบนัส Centerline ได้สร้างความขัดแย้งในการคำนวณอย่างรุนแรง อัลกอริทึมต้องคอยประเมินความคุ้มค่าระหว่างการเลือก *"เส้นทางที่สั้นกว่าแต่ชิดกำแพง"* กับ *"เส้นทางที่อ้อมกว่าแต่ได้โบนัสตรงกลางถนน"* ความลังเลนี้ปรากฏให้เห็นในทุกตาราง แต่รุนแรงที่สุดในสถานการณ์ที่ 2 เวลาที่ใช้ในการประมวลผลพุ่งสูงจาก 3.72 วินาที เป็น 46.83 วินาที และจำนวนรอบ (Iterations) ทะลุจาก 274,741 ไปเกือบ 1.88 ล้านรอบ แม้ว่าเส้นทางสุดท้ายจะใกล้เคียงกับอุดมคติแบบ Centerline ที่ปลอดภัย (ต้นทุนรวมลดลงเหลือ 2,714.45) แต่ภาระการประมวลผลที่ตามมาทำให้การจับคู่นี้ไม่มีประสิทธิภาพอย่างยิ่งสำหรับแผนที่ขนาดใหญ่และซับซ้อน
* **การทำงานร่วมกันอย่างลงตัวกับ Best-First Search:** ในทางกลับกัน Best-First Search แสดงให้เห็นถึงการทำงานร่วมกับเทคนิค Centerline ได้อย่างยอดเยี่ยม เนื่องจากปกติ Best-First มักจะไปติดในมุมอับ (Local minima) หรือเดินเฉียดกำแพง โบนัส Centerline จึงทำหน้าที่เป็น "เส้นนำทาง" ที่ทรงพลัง ซึ่งเห็นได้ชัดในสถานการณ์ที่ 2 การเปิดใช้ Centerline Follow ช่วยลดจำนวนรอบการทำงานลงจาก 46,265 เหลือเพียง 25,227 โบนัสนี้ประสบความสำเร็จในการดึงการค้นหาให้ออกมายังพื้นที่โล่ง ป้องกันการติดทางตัน และสร้างเส้นทางที่ปลอดภัยในเวลาเพียง 0.64 วินาที

### 3. ความสามารถในการปรับตัวต่อความแปรปรวนของต้นทุนสภาพแวดล้อม
เลือกใช้เป็นเงื่อนไข สามารถผ่านตัวอาคารได้แต่จะมีค่า costที่เพิ่มขึ้น

* **เพื่อม cost ของตัวอาคาร1แต้ม (BP+1):** เมื่อมีการเพิ่มต้นทุน (Cost) ที่สูงขึ้นในพื้นที่ตัวอาคาร A* เลือกที่จะเดินอ้อมโซนลงโทษ ส่งผลให้ระยะทางกายภาพไกลขึ้น (993.96px) แต่ได้ต้นทุนทางคณิตศาสตร์ที่ต่ำที่สุดและคุ้มค่าที่สุด ในทางตรงข้าม อัลกอริทึมที่ไม่สนใจค่าน้ำหนักของพื้นที่อย่าง BFS และ Best-First กลับเลือกที่จะดันทุรังเดินทะลุตัวอาคารไปตรงๆ BFS ทำระยะทางได้สั้นที่สุด (880.84px) เพียงเพราะมันนับแค่จำนวนก้าว โดยล้มเหลวในจุดประสงค์เรื่องการหลีกเลี่ยงพื้นที่อันตรายที่มีต้นทุนสูงอย่างสิ้นเชิง
* **โบนัสจุดประตูทางผ่าน (DP-5):** เมื่อมีการลดต้นทุนอย่างมีนัยสำคัญ (-5) ที่บริเวณ "จุดประตูทางผ่าน" A* ได้ล็อคเส้นทางของตนเองอย่างเหนียวแน่นเพื่อเดินผ่านประตูเหล่านี้ สิ่งที่น่าสนใจคือ ในสถานการณ์ 3 DP-5 ระยะทางกายภาพของ A* คงที่อยู่ที่ 917.99px เป๊ะ ไม่ว่าจะเปิดใช้ Centerline Follow หรือไม่ก็ตาม สิ่งนี้บ่งชี้ว่าส่วนลด DP-5 ที่ประตูนั้นมีผลทางคณิตศาสตร์อย่างมหาศาลจนทำหน้าที่เป็น "สมอ" ยึดเส้นทาง ซึ่งไปลบล้างการเบี่ยงเบนเส้นทางเล็กๆ น้อยๆ ที่มักจะเกิดจากโบนัส Centerline อย่างไรก็ตาม A* ยังคงต้องเผชิญกับภาระการประมวลผลอย่างหนัก จำนวนรอบการทำงานเพิ่มขึ้นกว่าเท่าตัว (จาก 159k เป็น 338k) เพราะอัลกอริทึมต้องคำนวณเปรียบเทียบอย่างหนักระหว่างการพุ่งเป้าไปที่โบนัสประตู กับการคำนวณรักษาระยะห่างตรงกลางถนน (Clearance)
