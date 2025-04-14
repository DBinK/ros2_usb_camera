# 更灵活的 ROS2 USB 摄像头节点

提供更多的参数控制, 包含: 

```python
# 摄像头参数
camera_params = {
    'camera_id': 0,
    'image_width': 1280,
    'image_height': 720,
    'auto_exposure': 1,
    'exposure_time': 100,
    'fps': 60,
    'gain': 100,
}
```

具体还要看摄像头本身支不支持该参数, 可以使用 `qv4l2` 检查支持调整的参数
