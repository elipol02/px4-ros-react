import React, { useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import mapboxgl from '!mapbox-gl'; // eslint-disable-line import/no-webpack-loader-syntax
import NavigationIcon from '@mui/icons-material/Navigation';
import LocationOnIcon from '@mui/icons-material/LocationOn';
import NearMeIcon from '@mui/icons-material/NearMe';
import AirplanemodeInactiveIcon from '@mui/icons-material/AirplanemodeInactive';
import ApartmentIcon from '@mui/icons-material/Apartment';
import LandscapeIcon from '@mui/icons-material/Landscape';
import MenuIcon from '@mui/icons-material/Menu';
import { Button, Tooltip, Box } from '@mui/material';


mapboxgl.accessToken = ''; // Replace with your Mapbox access token

const MapComponent = ({ latitude, longitude, angle, points, setEditPoints, defaultElevation }) => {
  const [noFlyZones, setNoFlyZones] = useState(null);
  const [mapStyle, setMapStyle] = useState('mapbox://styles/mapbox/satellite-streets-v11');
  const [showMenu, setShowMenu] = useState(false);
  const [layersVisible, setLayersVisible] = useState({
    noFlyZones: true,
    terrain: false,
    buildings: false,
  });
  const mapContainer = useRef(null);
  const map = useRef(null);
  const marker = useRef(null);
  const markers = useRef(new Map());
  const pointMemory = useRef([]);
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
  const defaultElevationRef = useRef(defaultElevation);

  // Set default elevation for markers
  useEffect(() => {
    defaultElevationRef.current = defaultElevation;
  }, [defaultElevation]);

  // Initialize map
  useEffect(() => {
    if (!latitude || !longitude) return; // wait for latitude and longitude to be set

    if (map.current) return; // initialize map only once
    map.current = new mapboxgl.Map({
      container: mapContainer.current,
      style: 'mapbox://styles/mapbox/satellite-v9',
      center: [longitude, latitude],
      zoom: 15,
      bearing: bearing,
    });

    map.current.on('rotate', () => {
      setBearing(map.current.getBearing());
    });

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

      // Add no-fly zones layer
      if (noFlyZones && layersVisible.noFlyZones) {
        map.current.addSource('no-fly-zones', {
          type: 'geojson',
          data: noFlyZones
        });

        map.current.addLayer({
          id: 'no-fly-zones-layer',
          type: 'fill',
          source: 'no-fly-zones',
          paint: {
            'fill-color': '#FF0000',
            'fill-opacity': 0.2
          }
        });
      }
      
      // Add the 3D buildings layer
      if (layersVisible.buildings) {
        map.current.addLayer({
          'id': '3d-buildings',
          'source': 'composite',
          'source-layer': 'building',
          'filter': ['==', 'extrude', 'true'],
          'type': 'fill-extrusion',
          'minzoom': 15,
          'paint': {
            'fill-extrusion-color': '#aaa',
            'fill-extrusion-height': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'height']
            ],
            'fill-extrusion-base': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'min_height']
            ],
            'fill-extrusion-opacity': 0.9
          }
        });
      }
      
      // Add the terrain layer
      if (layersVisible.terrain) {
        map.current.addSource('mapbox-dem', {
          'type': 'raster-dem',
          'url': 'mapbox://mapbox.mapbox-terrain-dem-v1',
          'tileSize': 512,
          'maxzoom': 14
        });
        map.current.setTerrain({ 'source': 'mapbox-dem' });
        

        // Add sky layer for better visualization
        map.current.addLayer({
          'id': 'sky',
          'type': 'sky',
          'paint': {
            'sky-type': 'atmosphere',
            'sky-atmosphere-sun': [0.0, 0.0],
            'sky-atmosphere-sun-intensity': 15
          }
        });
      }
      
    });
    // Add click event listener to the map
    map.current.on('click', (e) => {
      const { lng, lat } = e.lngLat;
      const newId = Date.now();
      console.log(`Adding new point: ${newId} at (${lng}, ${lat})`);
      pointMemory.current.push({ lng, lat, id: newId, elevation: defaultElevationRef.current });
      setEditPoints([...pointMemory.current]);
    });

  }, [latitude, longitude]);

  // Toggle custom layers
  useEffect(() => {
    if (map.current) {
      // Toggle no-fly zones layer visibility
      if (map.current.getLayer('no-fly-zones-layer')) {
        map.current.setLayoutProperty('no-fly-zones-layer', 'visibility', layersVisible.noFlyZones ? 'visible' : 'none');
      }
  
      // Toggle 3D buildings layer visibility
      if (map.current.getLayer('3d-buildings')) {
        map.current.setLayoutProperty('3d-buildings', 'visibility', layersVisible.buildings ? 'visible' : 'none');
      } else if (layersVisible.buildings) {
        map.current.addLayer({
          id: '3d-buildings',
          source: 'composite',
          'source-layer': 'building',
          filter: ['==', 'extrude', 'true'],
          type: 'fill-extrusion',
          minzoom: 15,
          paint: {
            'fill-extrusion-color': '#aaa',
            'fill-extrusion-height': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'height']
            ],
            'fill-extrusion-base': [
              'interpolate',
              ['linear'],
              ['zoom'],
              15,
              0,
              15.05,
              ['get', 'min_height']
            ],
            'fill-extrusion-opacity': 0.9
          }
        });
      }
  
      // Toggle terrain layer visibility
      if (map.current.getSource('mapbox-dem')) {
        if (layersVisible.terrain) {
          map.current.setTerrain({ source: 'mapbox-dem' });
        } else {
          map.current.setTerrain(null);
        }
      } else if (layersVisible.terrain) {
        map.current.addSource('mapbox-dem', {
          type: 'raster-dem',
          url: 'mapbox://mapbox.mapbox-terrain-dem-v1',
          tileSize: 512,
          maxzoom: 14
        });
        map.current.setTerrain({ source: 'mapbox-dem' });
  
        // Add sky layer for better visualization
        map.current.addLayer({
          id: 'sky',
          type: 'sky',
          paint: {
            'sky-type': 'atmosphere',
            'sky-atmosphere-sun': [0.0, 0.0],
            'sky-atmosphere-sun-intensity': 15
          }
        });
      }
    }
  }, [layersVisible]);
  

  // Update rotation of the marker and update the path
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

  // Editing Markers
  useEffect(() => {
    if (map.current) {
      // Remove existing markers that are not in the current state
      markers.current.forEach((marker, id) => {
        if (!points.some(location => location.id === id)) {
          marker.remove();
          markers.current.delete(id);
        }
      });

      // Update pointMemory with current points
      pointMemory.current = [...points];

      // Add or update markers
      points.forEach((location, index) => {
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
            console.log(`Removing point: ${id}`);
            newMarker.remove();
            markers.current.delete(id);
            pointMemory.current = pointMemory.current.filter(location => location.id !== id);
            setEditPoints([...pointMemory.current]);
          };

          const handleDragEnd = () => {
            const newLngLat = newMarker.getLngLat();
            console.log(`Dragging point: ${id} to (${newLngLat.lng}, ${newLngLat.lat})`);
            pointMemory.current = pointMemory.current.map(location => location.id === id ? { ...location, lng: newLngLat.lng, lat: newLngLat.lat } : location);
            setEditPoints([...pointMemory.current]);
          };

          newMarker.getElement().addEventListener('contextmenu', handleContextMenu);
          newMarker.on('dragend', handleDragEnd);

          markers.current.set(id, newMarker);
        } else {
          const marker = markers.current.get(id);
          marker.setLngLat([lng, lat]);

          // Check and update the number element
          const numberElement = marker.getElement().querySelector('div');
          if (numberElement) {
            numberElement.textContent = index + 1;
          } else {
            console.error(`Number element not found for marker with id: ${id}`);
          }
        }
      });
    }
  }, [points]);

  // Center map handler
  const handleCenter = () => {
    if (map.current) {
      map.current.setCenter([longitude, latitude]);

    }
  };
  
  // Toggle layers handler
  const toggleLayerVisibility = (layer) => {
    setLayersVisible((prevState) => ({
      ...prevState,
      [layer]: !prevState[layer],
    }));
  };

  // Toggle menu
  const handleShowMenu = () => {
    setShowMenu(!showMenu);
  };

  // Set map style
  const changeMapStyle = (style) => {
    setMapStyle(style);
    if (map.current) {
      map.current.setStyle(style);
    }
  };

  // Fetch no-fly zone data from GitHub
  useEffect(() => {
    
    fetch('https://raw.githubusercontent.com/mapbox/drone-feedback/master/sources/geojson/5_mile_airport.geojson')
      .then(response => response.json())
      .then(data => setNoFlyZones(data));
  }, []);

  return (
    <div style={{ height: '100%' }}>
      <Box
          sx={{
          position: 'absolute',
          top: 136,
          left: 16,
          display: 'flex',
          flexDirection: 'column',
          gap: 1.5,
          zIndex: 1000,
          }}
      >
        <Tooltip title="Map Menu" placement="right">
          <Button variant="contained" size="small" color="secondary" onClick={handleShowMenu}>
            <MenuIcon fontSize='small'/>
          </Button>
        </Tooltip>

        {showMenu && (
          <Box
            sx={{
            position: 'relative',
            left: 16,
            display: 'flex',
            flexDirection: 'column',
            gap: 1.5,
            zIndex: 1000,
            }}
          >
            <Tooltip title="Center map" placement="right">
              <Button 
                variant="contained"
                size="small"
                color="secondary"
                onClick={handleCenter}
                sx={{
                  width: 32,
                  height: 32,
                  borderRadius: '50%',
                  minWidth: 0,
                  padding: 0,
                }}
              >
                <NearMeIcon fontSize="small"/>
              </Button>
            </Tooltip>
            <Tooltip title="Toggle no-fly zone" placement="right">
              <Button 
                variant="contained"
                size="small"
                color={layersVisible.noFlyZones ? "tertiary" : "secondary"}
                onClick={() => toggleLayerVisibility('noFlyZones')}
                sx={{
                  width: 32,
                  height: 32,
                  borderRadius: '50%',
                  minWidth: 0,
                  padding: 0,
                }}
              >  
                <AirplanemodeInactiveIcon fontSize="small"/>
              </Button>
            </Tooltip>
            <Tooltip title="Toggle landscape" placement="right">
              <Button 
                variant="contained"
                size="small"
                color={layersVisible.terrain ? "tertiary" : "secondary"}
                onClick={() => toggleLayerVisibility('terrain')}
                sx={{
                  width: 32,
                  height: 32,
                  borderRadius: '50%',
                  minWidth: 0,
                  padding: 0,
                }}
              > 
                <LandscapeIcon fontSize="small"/>
              </Button>
            </Tooltip>
            <Tooltip title="Toggle buildings" placement="right">
              <Button 
                variant="contained"
                size="small"
                color={layersVisible.buildings ? "tertiary" : "secondary"}
                onClick={() => toggleLayerVisibility('buildings')}
                sx={{
                  width: 32,
                  height: 32,
                  borderRadius: '50%',
                  minWidth: 0,
                  padding: 0,
                }}
              >
                <ApartmentIcon fontSize="small"/>
              </Button>
            </Tooltip>
          </Box>    
        )}    
      </Box>
      <div ref={mapContainer} className="map-container" style={{ height: '100%', width: '100%' }} />
    </div>
  );
};

export default MapComponent;
