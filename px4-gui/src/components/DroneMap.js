import React, { useState, useEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import mapboxgl from '!mapbox-gl'; // eslint-disable-line import/no-webpack-loader-syntax
import NavigationIcon from '@mui/icons-material/Navigation';
import Button from '@mui/material/Button';

mapboxgl.accessToken = 'pk.eyJ1IjoiZWxpcG9sMDIiLCJhIjoiY2x4cDE0eGNqMDVobjJrcGtzODUxMXZtMyJ9.e0Iye6AtNZEH_B806foH5w'; // Replace with your Mapbox access token

const DroneMap = ({ latitude, longitude, angle }) => {
  const mapContainer = useRef(null);
  const map = useRef(null);
  const marker = useRef(null);
  const [zoom, setZoom] = useState(15);
  const [bearing, setBearing] = useState(0);
  const path = useRef({
    type: 'FeatureCollection',
    features: [{
      type: 'Feature',
      geometry: {
        type: 'LineString',
        coordinates: []
      }
    }]
  });

  useEffect(() => {
    if (!latitude || !longitude) return; // wait for latitude and longitude to be set

    if (map.current) return; // initialize map only once
    map.current = new mapboxgl.Map({
      container: mapContainer.current,
      style: 'mapbox://styles/mapbox/satellite-streets-v11',
      center: [longitude, latitude],
      zoom: zoom,
      bearing: bearing
    });

    map.current.on('rotate', () => {
      setBearing(map.current.getBearing());
    });

    // Add navigation controls
    map.current.addControl(new mapboxgl.NavigationControl());

    // Create a custom marker using NavigationIcon
    const markerElement = document.createElement('div');
    markerElement.className = 'custom-marker';
    markerElement.style.display = 'flex';
    markerElement.style.alignItems = 'center';
    markerElement.style.justifyContent = 'center';

    // Render NavigationIcon inside the custom marker using ReactDOM
    ReactDOM.render(
      <NavigationIcon style={{ transform: `rotate(${angle}deg)`, fontSize: '36px', color: '#1111DD' }} />,
      markerElement
    );

    // Add the marker to the map
    marker.current = new mapboxgl.Marker({
        element: markerElement
      })
      .setLngLat([longitude, latitude])
      .addTo(map.current);

    // Add the path source and layer to the map
    map.current.on('load', () => {
      map.current.addSource('path', {
        type: 'geojson',
        data: path.current
      });

      map.current.addLayer({
        id: 'path',
        type: 'line',
        source: 'path',
        layout: {
          'line-join': 'round',
          'line-cap': 'round'
        },
        paint: {
          'line-color': '#ff0000',
          'line-width': 4
        }
      });
    });

  }, [latitude, longitude]); // run effect only when latitude or longitude change

  useEffect(() => {
    if (!latitude || !longitude) return; // wait for latitude and longitude to be set

    if (marker.current) {
      marker.current.setLngLat([longitude, latitude]);
      
      // Update the rotation of the icon
      const iconElement = marker.current.getElement().querySelector('svg');
      if (iconElement) {
        iconElement.style.transform = `rotate(${angle - bearing}deg)`;
        iconElement.style.color = '#1111DD';
      }

      // Update the path with the new marker position
      path.current.features[0].geometry.coordinates.push([longitude, latitude]);
      if (map.current.getSource('path')) {
        map.current.getSource('path').setData(path.current);
      }
    }
  }, [latitude, longitude, angle, bearing]);

  const handleCenter = () => {
    if (map.current) {
      map.current.setCenter([longitude, latitude]);
    }
  };

  return (
    <div>
      <div ref={mapContainer} className="map-container" style={{ height: '500px' }} />
      <Button onClick={handleCenter}>Center</Button>
    </div>
  );
};

export default DroneMap;
