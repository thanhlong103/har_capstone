import React, { useEffect } from 'react';
import ROSLIB from 'roslib';

const RosConnection = () => {
  useEffect(() => {
    // Connect to ROS
    const ros = new ROSLIB.Ros({
      url: 'ws://<ROS_IP_ADDRESS>:9090' // Replace with ROS server IP
    });

    ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
    });


    return () => {
      listener.unsubscribe();
      ros.close();
    };
  }, []);

  return <div>Streaming ROS Data...</div>;
};

export default RosConnection;
