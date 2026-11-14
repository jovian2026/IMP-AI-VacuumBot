# pip install ai-edge-litert --break-system-packages   (or tflite-runtime)
import numpy as np, cv2
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter

labels = [l.strip().split(" ", 1)[-1] for l in open("labels.txt")]
interp = Interpreter("model_unquant.tflite"); interp.allocate_tensors()
inp, out = interp.get_input_details()[0], interp.get_output_details()[0]

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

while True:
    frame = cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2RGB)
    h, w = frame.shape[:2]; s = min(h, w)                      # centre square crop, like TM
    crop = frame[(h-s)//2:(h+s)//2, (w-s)//2:(w+s)//2]
    x = cv2.resize(crop, (224, 224)).astype(np.float32) / 127.5 - 1   # TM normalisation
    interp.set_tensor(inp["index"], x[None])
    interp.invoke()
    p = interp.get_tensor(out["index"])[0]
    print(labels[p.argmax()], round(float(p.max()), 2))