import React from 'react';

const MoveToDestination = () => {
  return (
    <div>
      <h1>Move to Destination</h1>
      {/* Replace the src URL with your ROS web_video_server stream */}
      <img
        src="http://<ROS_IP_ADDRESS>:8080/stream?topic=/camera/image_raw"
        alt="RViz Stream"
        style={{ width: '100%', height: 'auto' }}
      />
    </div>
  );
};

export default MoveToDestination;
