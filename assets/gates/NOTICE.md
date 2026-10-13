# Notice for the gate models

## Where `square/gate.xml` comes from

`square/gate.xml` is a modified copy of `lsy_drone_racing/envs/assets/gate.xml` from the
repository `learnsyslab/lsy_drone_racing`, commit `ac7377baa1cfd8fef1bcdc31eeaebff382f453b7`.

That repository is under the MIT License:

```
MIT License

Copyright (c) 2024 Learning Systems and Robotics Lab (LSY)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

The license text above was read from `LICENSE` at the pinned commit.

## What was changed

| Part | Original | Changed to |
|---|---|---|
| Banner pictures `gate_top.png`, `gate_bottom.png`, `gate_left.png`, `gate_right.png` | pictures of LSY with the TUM mark and the LSY logo | new pictures made from LSY's four strips of the same commit: the green ground and the checker blocks stay, the TUM mark and the LSY logo are replaced by the word GATE (set in Arial Bold); same file names and pixel sizes |
| Material `frame_mat` (the thin outer frame) | gray | neon yellow (240, 255, 0), with `emission="1"` so that it glows |
| The four ropes (`rope_*`) | thin dark capsules | red (214, 40, 40) coil springs, one shared mesh `spring.stl`, no collision |
| Post `gate_stand` | plain beige material | bark material; four thin visual panels `post_bark_*` carry the bark picture `gate_post_bark.png` on the four long sides |

Not changed: the sizes, all collision shapes (`col_*` and `gate_stand`), the body name `gate`, and
every geom name of the original. The four panels `post_bark_*` are new geoms without collision.

## Where the other new files come from

- `square/spring.stl` is made by `tools/make_spring_mesh.py` with numpy alone (own work).
- The banner pictures (derived from LSY's strips, see above) and the bark picture were made with
  `coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-12_gate_look/make_gate_look.py`.
  The bark picture is drawn by that script from a fixed random seed; it is not a photograph.
