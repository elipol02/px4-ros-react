// components/Dropdown.js
import React, { useState, useEffect } from 'react';
import { Box, Button, FormControl, InputLabel, MenuItem, Select } from '@mui/material';
import { useROS } from '../utils/ROSConnection';

const Dropdown = ({ addTopic }) => {
  const { ros } = useROS();
  const [topic, setTopic] = useState('');
  const [topics, setTopics] = useState([]);

  useEffect(() => {
    if (!ros) return;

    const getTopics = () => {
      ros.getTopics((result) => {
        setTopics(result.topics);
      });
    };

    getTopics();
    const interval = setInterval(getTopics, 2000); // Update topics list every 2 seconds

    return () => clearInterval(interval);
  }, [ros]);

  const handleAdd = () => {
    if (topic) {
      addTopic(topic);
      setTopic('');
    }
  };

  return (
    <Box p={2} sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
      <FormControl fullWidth>
        <InputLabel id="topic-select-label">Echo a ROS Topic</InputLabel>
        <Select
          labelId="topic-select-label"
          value={topic}
          label="Select ROS Topic"
          onChange={(e) => setTopic(e.target.value)}
          MenuProps={{
            disableScrollLock: true,
            PaperProps: {
              style: {
                transition: 'none',
                animation: 'none',
              },
            },
            transitionDuration: 0,
          }}
        >
          {topics.map((t) => (
            <MenuItem key={t} value={t}>
              {t}
            </MenuItem>
          ))}
        </Select>
      </FormControl>
      <Button variant="contained" color="tertiary" onClick={handleAdd}>
        Add
      </Button>
    </Box>
  );
};

export default Dropdown;
