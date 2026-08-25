# Models

Model binaries are intentionally excluded from Git.

The recovered ground-station detector expects an OpenVINO YOLO person-detection model path through the ROS parameter `~model_xml`. The public default points to:

```text
/opt/models/yolov8n_openvino_model/yolov8n.xml
```

Override this parameter for your deployment. Do not commit large `.pt`, `.onnx`, `.engine`, `.bin`, or generated model caches unless you have a clear distribution/license reason.
