import React, { useEffect, useState } from 'react';
import { useROS } from '../utils/ROSConnection';
import ROSLIB from 'roslib';

const PointPublisher = ({ editPoints }) => {
  const { ros, connected } = useROS();
  const [pathPublisher, setPathPublisher] = useState(null);

  useEffect(() => {
    if (connected && ros) {
      const publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/setpoints',
        messageType: 'nav_msgs/Path',
      });
      setPathPublisher(publisher);
      console.log('Topic initialized: /setpoints');
    }
  }, [connected, ros]);

  const handlePublish = () => {
    if (!connected) {
      console.error('Not connected to ROS');
      return;
    }

    if (!pathPublisher) {
      console.error('Topic setpoints not initialized');
      return;
    }

    // Format the editPoints into the correct structure for Path message
    const poses = editPoints.map(point => ({
      pose: {
        position: {
          x: point.lng,
          y: point.lat,
          z: point.elevation,
        },
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 1,
        },
      },
      header: {
        stamp: { secs: 0, nsecs: 0 },
        frame_id: 'world',
      },
    }));

    const pathMessage = new ROSLIB.Message({
      header: {
        stamp: { secs: 0, nsecs: 0 },
        frame_id: 'world',
      },
      poses: poses,
    });

    pathPublisher.publish(pathMessage);
    console.log('Published to /setpoints:', pathMessage);
  };

  useEffect(() => {
    if (editPoints) {
      handlePublish();
    }
  }, [editPoints]);

  return null; // or some JSX if you need to render anything
};

export default PointPublisher;
