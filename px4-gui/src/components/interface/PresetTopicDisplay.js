// components/PresetTopicDisplay.js
import React from 'react';
import { Typography, Box, Card, CardContent } from '@mui/material';

const PresetTopicDisplay = ({ fields, getLastItem }) => {
  return (
    <Box sx={{ p: 2, my: 2, display: 'flex', justifyContent: 'space-between', width: 300 }}>
      <Card style={{ overflow: 'auto', height: '100%', width: '100%' }}>
        <CardContent style={{ padding: '8px' }}>
          {Object.entries(fields).map(([key, value]) => (
            <Typography key={key} variant="body2" component="p">
              {`${key}: ${getLastItem(key)}`}
            </Typography>
          ))}
        </CardContent>
      </Card>
    </Box>
  );
};

export default PresetTopicDisplay;
