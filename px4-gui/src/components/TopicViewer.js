import React, { useState, useEffect } from 'react';
import { Card, CardContent, Typography, IconButton, Table, TableBody, TableCell, TableContainer, TableRow, Paper } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import { ResizableBox } from 'react-resizable';
import 'react-resizable/css/styles.css';
import { useROS } from './ROSContext';

const TopicViewer = ({ ros, topicName, messageType, onRemove }) => {
  const [message, setMessage] = useState(null);
  const [numRows, setNumRows] = useState(0);
  const { getOrCreateTopic, unsubscribeCallback } = useROS();

  useEffect(() => {
    const messageCallback = (msg) => {
      setMessage(msg);
      setNumRows(Object.keys(flattenObject(msg)).length);
    };

    getOrCreateTopic(topicName, messageType, messageCallback);

    return () => {
      unsubscribeCallback(topicName, messageCallback);
    };
  }, [ros, topicName, messageType, getOrCreateTopic, unsubscribeCallback]);

  const flattenObject = (obj, parent = '', res = {}) => {
    for (let key in obj) {
      const propName = parent ? `${parent}.${key}` : key;
      if (typeof obj[key] === 'object' && obj[key] !== null) {
        flattenObject(obj[key], propName, res);
      } else {
        res[propName] = obj[key];
      }
    }
    return res;
  };

  const renderMessageTable = (message) => {
    if (!message) return null;

    const flatMessage = flattenObject(message);

    return (
      <TableContainer component={Paper} style={{ boxShadow: 'none' }}>
        <Table size="small" style={{ border: 'none' }}>
          <TableBody>
            {Object.entries(flatMessage).map(([key, value]) => (
              <TableRow key={key}>
                <TableCell component="th" scope="row" style={{ padding: '2px' }}>
                  {key}
                </TableCell>
                <TableCell style={{ padding: '2px' }}>{String(value)}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    );
  };

  const parseMessage = (msg) => {
    try {
      return typeof msg === 'string' ? JSON.parse(msg) : msg;
    } catch (e) {
      return msg;
    }
  };

  const parsedMessage = parseMessage(message);

  const rowHeight = 25; // Approximate row height in pixels
  const headerHeight = 55; // Approximate header height in pixels
  const maxHeight = headerHeight + numRows * rowHeight; // Calculate max height based on number of rows

  return (
    <ResizableBox
      width={300}
      height={maxHeight}
      minConstraints={[150, 150]}
      maxConstraints={[1000, maxHeight]}
      resizeHandles={['se']}
    >
      <Card variant="outlined" style={{ overflow: 'auto', height: '100%' }}>
        <CardContent style={{ padding: '8px' }}>
          <Typography variant="h6" component="h2" gutterBottom>
            {topicName}
            <IconButton color="primary" onClick={onRemove} style={{ float: 'right', padding: '0', marginTop: '-4px' }}>
              <CloseIcon />
            </IconButton>
          </Typography>
          {renderMessageTable(parsedMessage)}
        </CardContent>
      </Card>
    </ResizableBox>
  );
};

export default TopicViewer;
