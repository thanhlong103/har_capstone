import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const MoveToDestination = () => {
  const mapContainerRef = useRef(null);

  useEffect(() => {
    // Create a connection to the ROS 2 system
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Replace with your TurtleBot3 IP address
    });

    ros.on('connection', () => {
      console.log('Connected to ROS 2 WebSocket');
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS 2:', error);
    });

    // Subscribe to the /map topic published by Cartographer
    const mapTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    });

    mapTopic.subscribe((message) => {
      console.log('Received map:', message);
      // Render the map on the canvas
      renderMap(message);
    });

    const renderMap = (map) => {
      const canvas = mapContainerRef.current;
      const ctx = canvas.getContext('2d');
      const mapWidth = map.info.width;
      const mapHeight = map.info.height;

      // Scale map to fit canvas size
      const canvasWidth = canvas.width;
      const canvasHeight = canvas.height;

      const scaleX = canvasWidth / mapWidth;
      const scaleY = canvasHeight / mapHeight;

      // Clear the canvas before drawing
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Create image data for the original map
      const imageData = ctx.createImageData(mapWidth, mapHeight);
      for (let i = 0; i < map.data.length; i++) {
        const value = map.data[i];

        let color;
        if (value === 100) {
          color = [0, 0, 0, 255]; // Black for occupied space
        } else if (value === 0) {
          color = [255, 255, 255, 255]; // White for free space
        } else if (value === -1) {
          color = [192, 192, 192, 255]; // Gray for unknown space
        } else {
          color = [255, 255, 255, 255]; // Default to white for unexpected values
        }

        const index = i * 4;
        imageData.data[index] = color[0];
        imageData.data[index + 1] = color[1];
        imageData.data[index + 2] = color[2];
        imageData.data[index + 3] = color[3];
      }

      // Create a new image data object for the scaled map
      const scaledImageData = ctx.createImageData(canvasWidth, canvasHeight);

      // Loop through the canvas pixels and copy from the original image data
      for (let y = 0; y < canvasHeight; y++) {
        for (let x = 0; x < canvasWidth; x++) {
          const mapX = -Math.floor(x / scaleX);
          const mapY = Math.floor(y / scaleY);
          const index = (y * canvasWidth + x) * 4;
          const mapIndex = (mapY * mapWidth + mapX) * 4;

          // Copy the pixel data from the map to the scaled image data
          scaledImageData.data[index] = imageData.data[mapIndex];
          scaledImageData.data[index + 1] = imageData.data[mapIndex + 1];
          scaledImageData.data[index + 2] = imageData.data[mapIndex + 2];
          scaledImageData.data[index + 3] = imageData.data[mapIndex + 3];
        }
      }

      // Draw the scaled image data to the canvas
      ctx.putImageData(scaledImageData, 0, 0);
    };

    // Clean up when the component is unmounted
    return () => {
      mapTopic.unsubscribe();
    };
  }, []);

  return (
    <div>
      <h1>Move To Destination</h1>
      <canvas ref={mapContainerRef} width="800" height="800" style={{ border: '1px solid black' }} />
    </div>
  );
};

export default MoveToDestination;
