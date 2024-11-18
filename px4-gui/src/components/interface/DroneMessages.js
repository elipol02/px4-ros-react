import React, { useState, useEffect, useRef } from 'react';
import { Typography, Box } from '@mui/material';

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

const DroneMessages = ({ severities, messages, seconds, nanoseconds, setLastMessageSeverity, setLastMessage, showMessages, isSmallScreen }) => {
  const [messageHistory, setMessageHistory] = useState([]);
  const contentRef = useRef(null);
  const seenTimestamps = useRef(new Set());

  useEffect(() => {
    if (severities && messages && severities.length === messages.length && seconds && nanoseconds) {
      const newMessageHistory = [];
      for (let i = 0; i < severities.length; i++) {
        const timestamp = `${seconds[i]}-${nanoseconds[i]}`;
        if (!seenTimestamps.current.has(timestamp)) {
          seenTimestamps.current.add(timestamp);
          newMessageHistory.push({
            severity: severityLevels[severities[i]],
            message: messages[i],
            timestamp,
          });
        }
      }
      if (newMessageHistory.length > 0) {
        setMessageHistory(prev => [...prev, ...newMessageHistory]);
      }
    }
  }, [severities, messages, seconds, nanoseconds]);

  useEffect(() => {
    if (messageHistory.length > 0) {
      setLastMessageSeverity(messageHistory[messageHistory.length - 1].severity);
      setLastMessage(messageHistory[messageHistory.length - 1].message);
    }
  }, [messageHistory, setLastMessageSeverity, setLastMessage]);

  useEffect(() => {
    if (contentRef.current) {
      contentRef.current.scrollTop = contentRef.current.scrollHeight;
    }
  }, [messageHistory, showMessages]);

  return (
    <>
      {showMessages && (
        <Box
          ref={contentRef}
          sx={{
            display: 'flex',
            flexDirection: 'column',
            width: isSmallScreen ? 175 :300,
            height: isSmallScreen ? 100 : 150,
            overflow: 'auto',
            backgroundColor: 'rgba(255, 255, 255, 0.8)',
            padding: 1,
            borderRadius: isSmallScreen && 1,
          }}
        >
          {messageHistory.map((msg, index) => (
            <Typography key={index} variant="body2" component="p" sx={{ paddingLeft: 1 }}>
              {`${msg.severity}: ${msg.message}`}
            </Typography>
          ))}
        </Box>
      )}
    </>
  );
};

export default DroneMessages;
