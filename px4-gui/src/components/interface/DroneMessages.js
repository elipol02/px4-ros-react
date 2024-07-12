import React, { useState, useEffect, useRef } from 'react';
import { Typography, Box, Card, CardContent } from '@mui/material';

const severityLevels = [
  'Emergency',
  'Alert',
  'Critical',
  'Error',
  'Warning',
  'Notice',
  'Info',
  'Debug',
];

const DroneMessages = ({ severities, messages }) => {
  const [messageHistory, setMessageHistory] = useState([]);
  const cardContentRef = useRef(null);

  useEffect(() => {
    if (severities && messages && severities.length === messages.length) {
      const newMessageHistory = severities.map((severity, index) => ({
        severity: severityLevels[severity],
        message: messages[index],
      }));
      setMessageHistory(newMessageHistory);
    }
  }, [severities, messages]);

  useEffect(() => {
    if (cardContentRef.current) {
      cardContentRef.current.scrollTop = cardContentRef.current.scrollHeight;
    }
  }, [messageHistory]);

  return (
    <Box sx={{ p: 2, my: 2, display: 'flex', justifyContent: 'space-between', width: 300, height: 300 }}>
      <Card style={{ overflow: 'hidden', height: '100%', width: '100%' }}>
        <CardContent ref={cardContentRef} style={{ overflowY: 'auto', height: '95%', padding: '8px' }}>
          {messageHistory.map((msg, index) => (
            <Typography key={index} variant="body2" component="p">
              {`${msg.severity}: ${msg.message}`}
            </Typography>
          ))}
        </CardContent>
      </Card>
    </Box>
  );
};

export default DroneMessages;
