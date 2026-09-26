# 4. Camera and directions

## Which way is which

```text
                 north (heading 0°)
                        ▲
                        │
  west (270°) ◄─────── drone ───────► east (90°)   ← the drone starts here, facing east
                        │
                        ▼
                 south (180°)
```

- `heading_deg` grows when the drone turns **clockwise** (to the right).
  A positive `yaw_rate` turns clockwise, so it increases the heading.
- `roll` +: the drone tilts and slides to **its own right**. `pitch` +: it
  flies **forward**, in the direction the camera looks.

## What the camera sees

- It looks **straight ahead** (not tilted down), from under the nose of the
  drone: 3.5 cm in front of the centre, 1.8 cm below it.
- Picture: **1280 × 720** pixels, about **18 per second**. Field of view:
  about **80° wide and 50° high**.
- The camera is fixed to the drone, so **it tilts with the drone.** When you
  pitch forward (nose down), everything in the picture moves **up**; when
  you roll right, the picture rotates. Keep this in mind before reading
  "the balloon is above the centre" as "climb".
- On the ground the camera is only a few centimetres high: it mostly sees
  the floor and the sky until the drone takes off.

## From a pixel to a direction

Pixel `(u, v)`: `u` counts from the left (0 … 1279), `v` from the top
(0 … 719). The centre is (640, 360). With a focal length of about **763
pixels**:

```python
import math
F = 763.0                                               # pixels (80° field of view)
bearing_deg   = math.degrees(math.atan((u - 640) / F))  # + = to the right
elevation_deg = math.degrees(math.atan((360 - v) / F))  # + = above the centre
```

A balloon at `u = 900` is about 19° to the right: turn right
(`yaw_rate` > 0) to face it.

## From a balloon's size to its distance

A balloon is 0.30 m wide and 0.45 m tall. Its width in the picture tells
you roughly how far away it is:

```python
distance_m ≈ 0.30 * 763 / width_px      # ≈ 229 / width_px
```

| distance | balloon width in the picture |
|---|---|
| 0.5 m | ~460 px |
| 1 m | ~230 px |
| 2 m | ~115 px |
| 3 m | ~75 px |
| 4 m | ~57 px |

It is an estimate: the balloon's edge is blurred by light and shade, and a
balloon partly outside the picture looks smaller than it is.

## Balloon colours

The balloons are plain, bright colours. Their base colours (RGB, 0–255) are
roughly:

| balloon | R | G | B |
|---|---|---|---|
| red | 217 | 13 | 13 |
| yellow | 242 | 204 | 13 |
| blue | 13 | 64 | 230 |
| green | 13 | 166 | 26 |

Light and shadow change these values, and the image has a little noise.
Test ranges rather than exact values. The example `hello_drone.py` uses
these rough masks:

```python
r, g, b = (frame.image[..., i].astype(int) for i in range(3))
green  = (g > 90) & (r < 50) & (b < 60)
blue   = (b > 120) & (r < 60) & (g < 110)
yellow = (r > 150) & (g > 120) & (b < 70)
red    = (r > 120) & (g < 60) & (b < 60)
```

For speed, work on a smaller picture: `small = frame.image[::4, ::4]` is
320 × 180 and 16 times faster. Divide pixel positions by 4 accordingly (or
multiply back).

A simple start: find the pixels of a colour, take their average position
(`numpy.nonzero(mask)`, then the mean of the columns and rows) and their
count (a bigger count means a closer balloon). OpenCV's
`cv2.connectedComponentsWithStats` separates two balloons of the same
colour.

## Look at the pictures yourself

- In the analysis notebook, the last section shows the camera at key
  moments. `runlog.show_frames(run, [('my label', run.arm_time_s + 3)])` shows the
  picture 3 s after arming (times are simulation time; arming is at about
  4 s).
- Every run folder has `camera.mp4`: open it with any video player. The
  recording is scaled down to 640 × 360; your controller gets the full
  1280 × 720.
- Inside your controller, save a picture to look at later (only a few,
  it is slow): `import matplotlib.pyplot as plt; plt.imsave('frame.png', frame.image)`.

**Next:** [5. Testing and improving](5_testing_and_improving.md)
