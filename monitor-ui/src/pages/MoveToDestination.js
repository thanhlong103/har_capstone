import React, { useState } from "react";
import { Button , Box} from "@mui/material";
import MapImage from "./sim_map.png"
import { useNavigate } from "react-router-dom";

const MAP_CONFIG = {
  resolution: 0.05, // meters per pixel
  origin: [-2.92, -2.72], // map origin (x, y) in meters
  mapWidth: 575, //in pixel
  mapHeight: 350,
};

const SendNavGoal = () => {
  const [goal, setGoal] = useState(null);
  const navigate = useNavigate();
  
  const handleMapClick = (event) => {
    const img = event.target.getBoundingClientRect();
    const clickX = event.clientX - img.left;
    const clickY = event.clientY - img.top;

    // Convert pixel coordinates to world coordinates
    const worldX = MAP_CONFIG.origin[0] + clickX * MAP_CONFIG.resolution;
    const worldY = MAP_CONFIG.origin[1] + (MAP_CONFIG.mapHeight - clickY) * MAP_CONFIG.resolution;

    setGoal({ x: worldX, y: worldY });
  };

  const sendGoal = async () => {
    if (!goal) return alert("Please select a goal on the map!");

    // setTimeout(async () => {
      try {
        const response = await fetch("http://localhost:5000/api/send_goal", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(goal),
        });

        const data = await response.json();
        if (response.ok) {
          alert(`Goal sent successfully: X=${goal.x}, Y=${goal.y}`);
        } else {
          alert("Failed to send goal: " + data.error);
        }
      } catch (error) {
        console.error("Error sending goal:", error);
        alert("Failed to send goal");
      }
    // }, 5000); // Wait 5s before sending
  };

  return (
    <Box display="flex" flexDirection="column" justifyContent="center" alignItems="center" height="100vh">
      <div className="p-6 max-w-lg mx-auto bg-white rounded-xl shadow-md space-y-4">
        <h1 className="text-xl font-semibold">Select Goal on Map</h1>
        <div style={{ position: "relative" }}>
          <img
            src={MapImage}
            alt="Map"
            width={MAP_CONFIG.mapWidth}
            height={MAP_CONFIG.mapHeight}
            onClick={handleMapClick}
            style={{ cursor: "crosshair", border: "2px solid black" }}
          />
          {goal && (
            <div
              style={{
                position: "absolute",
                left: `${(goal.x - MAP_CONFIG.origin[0]) / MAP_CONFIG.resolution}px`,
                top: `${MAP_CONFIG.mapHeight - (goal.y - MAP_CONFIG.origin[1]) / MAP_CONFIG.resolution}px`,
                transform: "translate(-50%, -50%)",
                width: "10px",
                height: "10px",
                backgroundColor: "red",
                borderRadius: "50%",
              }}
            ></div>
          )}
        </div>
        <Button variant="contained" color="primary" onClick={sendGoal} fullWidth>
          Send Goal (Wait 5s)
        </Button>
        <Button variant="outlined" color="secondary" onClick={() => navigate("/")} fullWidth>
          Back to Home
        </Button>
      </div>
    </Box>
  );
};

export default SendNavGoal;
