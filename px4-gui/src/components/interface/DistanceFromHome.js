import React from 'react';
import { Typography } from '@mui/material';

const DistanceFromHome = ({ homeX, homeY, Xpos, Ypos }) => {
  const calculateDistance = (x1, y1, x2, y2) => {
    const deltaX = x2 - x1;
    const deltaY = y2 - y1;
    return Math.sqrt(deltaX * deltaX + deltaY * deltaY).toFixed(2);
  };

  const distance = calculateDistance(homeX, homeY, Xpos, Ypos);

  return (
    <div>
      <Typography>Distance: {distance} m</Typography>
    </div>
  );
};

export default DistanceFromHome;
