"""
Floor classifier with live camera preview (Edge Impulse float32 TFLite model)

Window shows:
  - yellow square  = the exact area the model looks at (centre crop)
  - score bars     = confidence for all 4 classes
  - bottom line    = smoothed label and cleaning mode

Keys (click the preview window first):
  1 = save frame as carpet     2 = concrete
  3 = marble                   4 = wood
  q = quit
Hold a number key down to save many frames quickly.
Saved frames go to ~/floor_capture/<class>/ for retraining.
"""
import os, time
import numpy as np, cv2
from collections import deque, Counter
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "ei-imp_vacuum_floorclassifier-transfer-learning-tensorflow-lite-float32-model.3.lite")
labels = ["carpet", "concrete", "marble", "wood"]  # EI = alphabetical order
SAVE_DIR = os.path.expanduser("~/floor_capture")
CONF_MIN, WINDOW, VOTES = 0.80, 10, 7              # smoothing: 7 of last 10 frames above 0.80

# ---------- model ----------
interp = Interpreter(MODEL)
interp.allocate_tensors()
inp, out = interp.get_input_details()[0], interp.get_output_details()[0]
H, W = inp["shape"][1], inp["shape"][2]
print("Model input:", inp["shape"], inp["dtype"])

# ---------- camera ----------
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

history = deque(maxlen=WINDOW)
mode = "hard"
saved = {l: 0 for l in labels}
flash_text, flash_until = "", 0
font = cv2.FONT_HERSHEY_SIMPLEX
bar_colour = {"carpet": (60, 60, 230), "concrete": (180, 180, 180),
              "marble": (230, 200, 60), "wood": (40, 120, 200)}   # BGR


def text(img, s, org, scale=0.55, colour=(255, 255, 255), thick=1):
    (tw, th), _ = cv2.getTextSize(s, font, scale, thick)
    x, y = org
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + 4), (0, 0, 0), -1)
    cv2.putText(img, s, (x, y), font, scale, colour, thick, cv2.LINE_AA)


while True:
    bgr = cam.capture_array()                       # picamera2 "RGB888" is BGR order (OpenCV style)
    h, w = bgr.shape[:2]
    s = min(h, w); y0 = (h - s) // 2; x0 = (w - s) // 2
    crop_bgr = bgr[y0:y0 + s, x0:x0 + s]

    # ---- classify ----
    rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    x = cv2.resize(rgb, (W, H)).astype(np.float32) / 255.0
    interp.set_tensor(inp["index"], x[None])
    interp.invoke()
    p = interp.get_tensor(out["index"])[0]
    label, conf = labels[p.argmax()], float(p.max())

    # ---- smoothing / mode ----
    if conf >= CONF_MIN:
        history.append(label)
    stable = "-"
    if len(history) == WINDOW:
        top, count = Counter(history).most_common(1)[0]
        if count >= VOTES:
            stable = top
            new_mode = "carpet" if top == "carpet" else "hard"
            if new_mode != mode:
                mode = new_mode
                print(">>> MODE:", mode)
    print(f"{label:<9} {conf:.2f} | mode: {mode}")

    # ---- draw preview ----
    view = bgr.copy()
    cv2.rectangle(view, (x0, y0), (x0 + s - 1, y0 + s - 1), (0, 255, 255), 2)
    for i, (lab, pr) in enumerate(zip(labels, p)):
        y = 30 + i * 28
        cv2.rectangle(view, (10, y - 16), (10 + 160, y + 4), (0, 0, 0), -1)
        cv2.rectangle(view, (10, y - 16), (10 + int(pr * 160), y + 4), bar_colour[lab], -1)
        cv2.putText(view, f"{lab} {pr:.2f}", (14, y), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    text(view, f"NOW: {label} {conf:.2f}   STABLE: {stable}   MODE: {mode.upper()}",
         (10, h - 15), 0.6, (0, 255, 0) if mode == "hard" else (0, 140, 255), 2)
    text(view, "1 carpet  2 concrete  3 marble  4 wood = save | q = quit", (10, h - 45), 0.45)
    if time.time() < flash_until:
        text(view, flash_text, (w - 230, 30), 0.55, (0, 255, 255))

    cv2.imshow("Floor classifier", view)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    if key in (ord("1"), ord("2"), ord("3"), ord("4")):
        cls = labels[key - ord("1")]
        folder = os.path.join(SAVE_DIR, cls)
        os.makedirs(folder, exist_ok=True)
        cv2.imwrite(os.path.join(folder, f"{cls}.picam{int(time.time() * 1000)}.jpg"), crop_bgr)
        saved[cls] += 1
        flash_text, flash_until = f"saved {cls}: {saved[cls]}", time.time() + 0.8

cam.stop()
cv2.destroyAllWindows()
print("Saved frames:", saved, "->", SAVE_DIR)