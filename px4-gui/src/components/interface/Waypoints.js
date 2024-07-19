import React, { useEffect, useMemo } from 'react';
import { Tooltip as MuiToolTip, Button, Slider, Box } from '@mui/material';
import { Line } from 'react-chartjs-2';
import { Chart, LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Legend } from 'chart.js';
import dragData from 'chartjs-plugin-dragdata';
import NotInterestedIcon from '@mui/icons-material/NotInterested';
import VerticalAlignCenterIcon from '@mui/icons-material/VerticalAlignCenter';

Chart.register(LineElement, PointElement, LinearScale, CategoryScale, Title, Tooltip, Legend);
Chart.register(dragData);

const WaypointsComponent = ({ setPoints, points, setEditPoints, pointsMessage, defaultElevation, setDefaultElevation, showWaypointEditor }) => {
  useEffect(() => {
    if (pointsMessage && pointsMessage.poses) {
      const newPoints = pointsMessage.poses.map(pose => ({
        lng: pose.pose.position.x,
        lat: pose.pose.position.y,
        id: Date.now() + Math.random(), // Ensure unique ID for each point
        elevation: pose.pose.position.z,
      }));
      setPoints(newPoints);
    }
  }, [pointsMessage]);

  const handleClearMarkers = () => {
    setEditPoints([]);
  };

  const handleElevationChange = (index, newElevation) => {
    setEditPoints(locations =>
      locations.map((location, i) =>
        i === index ? { ...location, elevation: newElevation } : location
      )
    );
  };

  const handleSetAllToDefaultElevation = () => {
    setEditPoints(locations =>
      locations.map(location => ({ ...location, elevation: defaultElevation }))
    );
  };

  const chartData = useMemo(() => ({
    labels: points.map((_, index) => `${index + 1}`),
    datasets: [
      {
        label: 'Elevation',
        data: points.map(location => location.elevation),
        borderColor: '#3e95cd',
        backgroundColor: '#7bb6dd',
        fill: true,
        tension: 0.1,
      },
    ],
  }), [points]);

  const chartOptions = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    plugins: {
      legend: {
        display: false,
      },
      tooltip: {
        enabled: false,
      },
      dragData: {
        round: 1,
        showTooltip: true,
        onDragEnd: (e, datasetIndex, index, value) => {
          handleElevationChange(index, value);
        },
      },
    },
    scales: {
      y: {
        min: 0,
        max: 100,
        title: {
          display: true,
          text: 'Elevation',
        },
        ticks: {
          stepSize: 10,
        },
      },
    },
  }), [handleElevationChange]);

  return (
    <div style={{ width: "100%"}}>
      {showWaypointEditor && (
        <Box sx={{ flexShrink: 0, width: '100%' }}>
          <div style={{ padding: '10px', marginTop: '-48px'}}>
            <Box
              sx= {{
                display: 'flex',
                flexDirection: 'row',
                gap: 0.5,
                zIndex: 1000,
              }}
            >
              <MuiToolTip title="Clear Waypoints" placement="top">
                <Button variant="contained" size="small" color="tertiary" onClick={handleClearMarkers} style={{ marginBottom: '10px' }}>
                  <NotInterestedIcon/>
                </Button>
              </MuiToolTip>
              <MuiToolTip title="Set all to default elevation" placement="top">
                <Button variant="contained" size="small" color="tertiary" onClick={handleSetAllToDefaultElevation} style={{ marginBottom: '10px' }}>
                  <VerticalAlignCenterIcon/>
                </Button>
              </MuiToolTip>
              <Slider
                value={defaultElevation}
                min={0}
                max={100}
                step={1}
                color="dark"
                onChange={(e, newValue) => setDefaultElevation(newValue)}
                valueLabelDisplay="auto"
                aria-labelledby="default-elevation-slider"
                style={{ marginBottom: '10px', width: '200px' }}
              />
            </Box>
            <div style={{ height: '100px' }}>
              <Line data={chartData} options={chartOptions} />
            </div>
          </div>
        </Box>
      )}
    </div>
  );
};

export default WaypointsComponent;
