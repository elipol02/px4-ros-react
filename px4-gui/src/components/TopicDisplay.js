// components/TopicDisplay.js
import React from 'react';
import { Typography, Box, IconButton, Table, TableBody, TableCell, TableContainer, TableRow, Card, CardContent } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';

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
    <TableContainer style={{ boxShadow: 'none' }}>
      <Table size="small" style={{ border: 'none' }}>
        <TableBody>
          {Object.entries(flatMessage).map(([key, value]) => (
            <TableRow key={key}>
              <TableCell component="th" scope="row" style={{ padding: '2px', wordWrap: 'break-word', maxWidth: '150px' }}>
                {key}
              </TableCell>
              <TableCell style={{ padding: '2px', wordWrap: 'break-word', maxWidth: '150px' }}>{String(value)}</TableCell>
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

const TopicDisplay = ({ topic, messages, onRemove }) => {
  const message = messages.length > 0 ? messages[0] : null;
  const parsedMessage = parseMessage(message);

  return (
    <Box sx={{ p: 2, my: 2, display: 'flex', justifyContent: 'space-between', width: 300 }}>
      <Card style={{ overflow: 'auto', height: '100%', width: '100%' }}>
        <CardContent style={{ padding: '8px' }}>
          <Typography variant="h6" component="h2" gutterBottom>
            {topic}
            <IconButton color="primary" onClick={() => onRemove(topic)} style={{ float: 'right', padding: '0', marginTop: '-4px' }}>
              <CloseIcon />
            </IconButton>
          </Typography>
          {renderMessageTable(parsedMessage)}
        </CardContent>
      </Card>
    </Box>
  );
};

export default TopicDisplay;
