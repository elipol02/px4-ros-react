import React, { useState, useEffect, useRef } from 'react';
import { createRoot } from 'react-dom/client';
import mapboxgl from '!mapbox-gl'; // eslint-disable-line import/no-webpack-loader-syntax
import NavigationIcon from '@mui/icons-material/Navigation';
import LocationOnIcon from '@mui/icons-material/LocationOn';
import Button from '@mui/material/Button';

mapboxgl.accessToken = 'pk.eyJ1IjoiZWxpcG9sMDIiLCJhIjoiY2x4cDE0eGNqMDVobjJrcGtzODUxMXZtMyJ9.e0Iye6AtNZEH_B806foH5w'; // Replace with your Mapbox access token

const DroneMap = ({ latitude, longitude, angle, setClickedCoords, clickedLocations, setClickedLocations, handlePublishWaypoints }) => {
  const mapContainer = useRef(null);
  const map = useRef(null);
  const marker = useRef(null);
  const markers = useRef(new Map());
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

    // Render NavigationIcon inside the custom marker using createRoot
    const root = createRoot(markerElement);
    root.render(
      <NavigationIcon style={{ transform: `rotate(${angle}deg)`, fontSize: '42px', color: '#4444FF' }} />
    );

    // Add the marker to the map
    marker.current = new mapboxgl.Marker({
      element: markerElement,
      anchor: 'center',
      draggable: false
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

    // Add click event listener to the map
    map.current.on('click', (e) => {
      const { lng, lat } = e.lngLat;
      setClickedCoords({ lng, lat });
      setClickedLocations(locations => [...locations, { lng, lat, id: Date.now() }]);
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
      }

      // Update the path with the new marker position
      path.current.features[0].geometry.coordinates.push([longitude, latitude]);
      if (map.current.getSource('path')) {
        map.current.getSource('path').setData(path.current);
      }
    }
  }, [latitude, longitude, angle, bearing]);

  useEffect(() => {
    // Remove existing markers that are not in the current state
    markers.current.forEach((marker, id) => {
      if (!clickedLocations.some(location => location.id === id)) {
        marker.remove();
        markers.current.delete(id);
      }
    });

    // Add or update markers
    clickedLocations.forEach((location, index) => {
      const { lng, lat, id } = location;

      if (!markers.current.has(id)) {
        const clickMarkerElement = document.createElement('div');
        clickMarkerElement.className = 'click-marker';

        // Render LocationOnIcon inside the click marker using createRoot
        const root = createRoot(clickMarkerElement);
        root.render(
          <>
            <LocationOnIcon style={{ fontSize: '36px', color: '#FF0000' }} />
            <div style={{ 
              position: 'absolute', 
              top: '0', 
              left: '50%', 
              transform: 'translate(-50%, -100%)',
              fontSize: '12px',
              color: '#FFFFFF',
              background: '#000000',
              borderRadius: '50%',
              padding: '2px 5px'
            }}>
              {index + 1}
            </div>
          </>
        );

        const newMarker = new mapboxgl.Marker({
          element: clickMarkerElement,
          anchor: 'bottom',
          draggable: true
        })
        .setLngLat([lng, lat])
        .addTo(map.current);

        const handleContextMenu = (e) => {
          e.preventDefault();
          newMarker.remove();
          markers.current.delete(id);
          setClickedLocations(locations => locations.filter(location => location.id !== id));
        };

        const handleDragEnd = () => {
          const newLngLat = newMarker.getLngLat();
          setClickedLocations(locations => locations.map(location => location.id === id ? { ...location, lng: newLngLat.lng, lat: newLngLat.lat } : location));
        };

        newMarker.getElement().addEventListener('contextmenu', handleContextMenu);
        newMarker.on('dragend', handleDragEnd);

        markers.current.set(id, newMarker);
      } else {
        const marker = markers.current.get(id);
        marker.setLngLat([lng, lat]);

        const numberElement = marker.getElement().querySelector('div');
        numberElement.textContent = index + 1;
      }
    });
  }, [clickedLocations]);

  const handleCenter = () => {
    if (map.current) {
      map.current.setCenter([longitude, latitude]);
    }
  };

  const handleClearMarkers = () => {
    markers.current.forEach(marker => marker.remove());
    markers.current.clear();
    setClickedLocations([]);
  };

  return (
    <div style={{ position: 'relative', height: '500px' }}>
      <div ref={mapContainer} className="map-container" style={{ height: '100%', width: '100%' }} />
      <div style={{ position: 'absolute', top: '10px', left: '10px', zIndex: 1, display: 'flex', flexDirection: 'column' }}>
        <Button variant="contained" size="small" color="primary" onClick={handleCenter} style={{ marginBottom: '10px' }}>Center</Button>
        <Button variant="contained" size="small" color="primary" onClick={handleClearMarkers}>Clear Markers</Button>
      </div>
    </div>
  );
};

export default DroneMap;
