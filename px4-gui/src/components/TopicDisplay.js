import React from 'react';
import { Typography, Box } from '@mui/material';

const TopicDisplay = ({ fields }) => {
  return (
    <Box>
      {Object.keys(fields).flatMap((topic) =>
        Object.keys(fields[topic]).map((key) => (
          <Box key={`${topic}-${key}`} display="flex" flexDirection="row" alignItems="center" mb={1}>
            <Typography variant="body1" style={{ fontWeight: 'bold', marginRight: '8px' }}>
              {key}:
            </Typography>
            <Typography variant="body1">
              {fields[topic][key] !== undefined ? String(fields[topic][key]) : 'N/A'}
            </Typography>
          </Box>
        ))
      )}
    </Box>
  );
};

export default TopicDisplay;
