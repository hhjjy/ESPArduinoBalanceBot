import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from matplotlib.patches import Rectangle, Arrow
from matplotlib.path import Path
import matplotlib.patches as patches

# 創建一個圖形
plt.figure(figsize=(12, 6))
ax = plt.gca()

# 設置坐標軸範圍
ax.set_xlim(0, 12)
ax.set_ylim(0, 6)
ax.axis('off')  # 隱藏坐標軸

# 繪製方塊和連接線
# 角度控制器
angle_controller = Rectangle((1, 3.5), 1.5, 1, fill=False, edgecolor='blue', linewidth=2)
ax.add_patch(angle_controller)
ax.text(1.75, 4, '角度控制器', ha='center', va='center')

# 速度控制器
speed_controller = Rectangle((4, 3.5), 1.5, 1, fill=False, edgecolor='green', linewidth=2)
ax.add_patch(speed_controller)
ax.text(4.75, 4, '速度控制器', ha='center', va='center')

# 電機驅動器
motor_driver = Rectangle((7, 3.5), 1.5, 1, fill=False, edgecolor='red', linewidth=2)
ax.add_patch(motor_driver)
ax.text(7.75, 4, '電機驅動器', ha='center', va='center')

# 小車系統
cart_system = Rectangle((10, 3.5), 1.5, 1, fill=False, edgecolor='purple', linewidth=2)
ax.add_patch(cart_system)
ax.text(10.75, 4, '小車系統', ha='center', va='center')

# 角度傳感器
angle_sensor = Rectangle((10, 1.5), 1.5, 1, fill=False, edgecolor='orange', linewidth=2)
ax.add_patch(angle_sensor)
ax.text(10.75, 2, '角度傳感器', ha='center', va='center')

# 速度傳感器
speed_sensor = Rectangle((7, 1.5), 1.5, 1, fill=False, edgecolor='brown', linewidth=2)
ax.add_patch(speed_sensor)
ax.text(7.75, 2, '速度傳感器', ha='center', va='center')

# 繪製連接線和箭頭
# 角度參考輸入到角度控制器
plt.arrow(0.5, 4, 0.5, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
ax.text(0.3, 4.2, '角度參考', ha='center', va='center')

# 角度控制器到速度控制器
plt.arrow(2.5, 4, 1.5, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 速度控制器到電機驅動器
plt.arrow(5.5, 4, 1.5, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 電機驅動器到小車系統
plt.arrow(8.5, 4, 1.5, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 小車系統到角度傳感器
plt.arrow(10.75, 3.5, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 小車系統到速度傳感器
plt.arrow(10.75, 3.5, 0, -1, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
plt.arrow(10.75, 2.5, -2, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 速度傳感器到速度控制器（內環反饋）
plt.arrow(7.75, 1.5, 0, -0.5, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
plt.arrow(7.75, 1, -3, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
plt.arrow(4.75, 1, 0, 2.5, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 角度傳感器到角度控制器（外環反饋）
plt.arrow(10.75, 1.5, 0, -0.5, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
plt.arrow(10.75, 1, -9, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)
plt.arrow(1.75, 1, 0, 2.5, head_width=0.1, head_length=0.1, fc='black', ec='black', linewidth=1.5)

# 加入求和點
sum_circle_angle = plt.Circle((1.75, 3), 0.2, fill=False, edgecolor='black', linewidth=1.5)
ax.add_patch(sum_circle_angle)
plt.text(1.75, 3, '+', ha='center', va='center')
plt.text(1.55, 3, '-', ha='center', va='center')

sum_circle_speed = plt.Circle((4.75, 3), 0.2, fill=False, edgecolor='black', linewidth=1.5)
ax.add_patch(sum_circle_speed)
plt.text(4.75, 3, '+', ha='center', va='center')
plt.text(4.55, 3, '-', ha='center', va='center')

# 添加標題
plt.title('平衡小車雙環控制系統方塊圖（外環角度控制，內環速度控制）', fontsize=14)

# 保存和顯示圖形
plt.tight_layout()
plt.savefig('balance_cart_control_diagram.png', dpi=300, bbox_inches='tight')
plt.show()
