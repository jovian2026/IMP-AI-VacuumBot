import numpy as np, cv2
from collections import deque, Counter
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter

MODEL = "ei-imp_vacuum_floorclassifier-transfer-learning-tensorflow-lite-float32-model.3.lite"     #downloaded file name
labels = ["carpet", "concrete", "marble", "wood"] # EI = alphabetical order

interp = Interpreter(MODEL)
interp.allocate_tensors()
inp, out = interp.get_input_details()[0], interp.get_output_details()[0]
H, W = inp["shape"][1], inp["shape"][2]
print("Model input:", inp["shape"], inp["dtype"])   # expect [1 160 160 3] float32

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

history = deque(maxlen=10)
mode = "hard"

while True:
    frame = cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2RGB)
    h, w = frame.shape[:2]; s = min(h, w)
    crop = frame[(h-s)//2:(h+s)//2, (w-s)//2:(w+s)//2]          # centre square, like "fit shortest axis"
    x = cv2.resize(crop, (W, H)).astype(np.float32) / 255.0      # EI scaling: 0..1

    interp.set_tensor(inp["index"], x[None])
    interp.invoke()
    p = interp.get_tensor(out["index"])[0]

    label, conf = labels[p.argmax()], float(p.max())
    if conf >= 0.80:
        history.append(label)
    if len(history) == 10:
        top, count = Counter(history).most_common(1)[0]
        if count >= 7:
            new_mode = "carpet" if top == "carpet" else "hard"
            if new_mode != mode:
                mode = new_mode
                print(">>> MODE:", mode)

    print(f"{label:<9} {conf:.2f} | mode: {mode}")