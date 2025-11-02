# Robot Controller FP - Webots Controller
Choose input mode when run the `controller-fp` file :
   - **Option 1**: Serial input from COM3
   - **Option 2**: Keyboard input (WASD + EQRF)

## Input Modes

### Serial Input Mode (Option 1)
- Connects to COM3 port at 115200 baud
- Send single characters via serial terminal
- Supports both uppercase and lowercase letters
- Characters are mapped to the same poses as keyboard input

### Keyboard Input Mode (Option 2)
- Uses Webots built-in keyboard API
- Controls are active when the simulation window is focused

## Available Commands

### Keyboard Controls
| Key | Action | Pose File |
|-----|--------|-----------|
| **W** | jalan maju | `pose-jalan-maju.json` |
| **A** | geser kiri | `pose-geser-kiri.json` |
| **S** | jalan mundur | `pose-jalan-mundur.json` |
| **D** | geser kanan | `pose-geser-kanan.json` |
| **E** | belok kanan | `pose-belok-kanan.json` |
| **Q** | belok kiri | `pose-belok-kiri.json` |
| **R** | jongkok | `pose-jongkok.json` |
| **F** | berdiri | `pose-berdiri.json` |