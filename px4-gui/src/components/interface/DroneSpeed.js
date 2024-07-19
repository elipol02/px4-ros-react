import React from 'react';
import { Typography } from '@mui/material';

function calculateVelocityScalar(x, y, z) {
    return Math.sqrt(x * x + y * y + z * z);
}

const VelocityScalar = ({ x, y, z }) => {
    const speed = calculateVelocityScalar(x, y, z);
    
    return (
        <div>
            <Typography>Speed: {speed.toFixed(2)} m/s</Typography>
        </div>
    );
};

export default VelocityScalar;
