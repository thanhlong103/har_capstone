const express = require("express");
const { exec } = require("child_process");
const cors = require("cors");

const app = express();
app.use(express.json());
app.use(cors());

app.post("/api/send_goal", (req, res) => {
    const { x, y } = req.body;

    if (x === undefined || y === undefined) {
        return res.status(400).json({ error: "Missing x or y value" });
    }

    const cmd = `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" -f`;

    exec(cmd, (error, stdout, stderr) => {
        if (error) {
            console.error(`Error: ${stderr}`);
            return res.status(500).json({ error: stderr });
        }
        res.json({ message: stdout });
    });
});

const PORT = 5000;
app.listen(PORT, () => console.log(`Server running on port ${PORT}`));
