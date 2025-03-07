import React, { useEffect, useState } from 'react';
import { Box, Typography , Button} from '@mui/material';
import ROSLIB from 'roslib';
import * as d3 from 'd3';
import { useNavigate } from "react-router-dom";

function MapPage() {
  const [scanData, setScanData] = useState(null);
  const navigate = useNavigate();

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090', // Replace with your WebSocket server URL
    });
  
    // Handle ROS connection errors
    ros.on('error', (error) => {
      console.error('ROS connection error:', error);
    });
  
    ros.on('connection', () => {
      console.log('Connected to ROS.');
    });
  
    ros.on('close', () => {
      console.warn('Connection to ROS closed.');
    });
  
    // Subscribe to the \scan topic
    const scanListener = new ROSLIB.Topic({
      ros: ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan',
    });
  
    scanListener.subscribe((message) => {
      setScanData(message);
    });

    return () => {
      scanListener.unsubscribe();
      ros.close();
    };
  }, []);
  

  useEffect(() => {
    if (scanData) {
      const { ranges, angle_min, angle_increment } = scanData;

      // Clear previous visualization
      d3.select('#scan-visualization').selectAll('*').remove();

      // Set up dimensions
      const width = 800;
      const height = 580;
      const svg = d3
        .select('#scan-visualization')
        .append('svg')
        .attr('width', width)
        .attr('height', height);

      const g = svg.append('g').attr('transform', `translate(${width / 2}, ${height / 2})`);

      // Scale for points
      const scale = d3.scaleLinear().domain([0, Math.max(...ranges)]).range([0, width / 2]);

      // Convert ranges to polar coordinates
      const points = ranges.map((range, i) => {
        const angle = angle_min + i * angle_increment;
        return {
          y: scale(range) * Math.cos(angle),
          x: scale(range) * Math.sin(angle),
        };
      });

      // Draw points
      g.selectAll('circle')
        .data(points)
        .enter()
        .append('circle')
        .attr('cx', (d) => d.x)
        .attr('cy', (d) => d.y)
        .attr('r', 1)
        .attr('fill', 'blue');
    }
  }, [scanData]);

  return (
    <Box display="flex" flexDirection="column" justifyContent="center" alignItems="center" height="100vh">
      <Typography variant="h2">LiDAR Scan Streaming</Typography>
      <Box id="scan-visualization" style={{}} justifyContent="center"></Box>
      <div className="p-6 max-w-md mx-auto bg-white rounded-xl shadow-md space-y-4">
        <Button variant="outlined" color="secondary" onClick={() => navigate("/")} fullWidth>
          Back to Home
        </Button>
      </div>
    </Box>
  );
}

export default MapPage;
