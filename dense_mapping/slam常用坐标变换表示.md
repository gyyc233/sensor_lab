
## Tcw与Twc

R<sub>12</sub> 表示“坐标系2的向量变换到坐标系1中”
t<sub>12</sub> 表示“坐标系1原点指向坐标系2原点的向量，在坐标系1下取的坐标”，记为“从1到2的向量”

向量a在12两个坐标系下的坐标为a1 ,a2,有


a<sub>1</sub> = R<sub>12</sub> * a<sub>2</sub> + t<sub>12</sub>

a<sub>1</sub> = T<sub>12</sub> * a<sub>2</sub>

T<sub>12</sub> 表示把2坐标系下的点变换到1坐标系中

如果1为世界坐标系，2为相机坐标系，有

a<sub>w</sub> = T<sub>wc</sub> * a<sub>c</sub>

> T<sub>wc</sub> 中的t<sub>wc</sub> 是相机在世界坐标系W下的位置, R<sub>wc</sub> 表示相机在世界坐标系W下的姿态

例如 相机原点坐标Ｏ<sub>c</sub>为零向量，则

Ｏ<sub>w</sub> = R<sub>wc</sub> * O<sub>c</sub> + t<sub>wc</sub> = t<sub>wc</sub>

可以看出Ｔ<sub>wc</sub>的平移变量t<sub>wc</sub>就是相机在世界坐标系下的位置
