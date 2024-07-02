import React, { useState, useRef, useEffect } from 'react';
import mapboxgl from 'mapbox-gl';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import 'mapbox-gl/dist/mapbox-gl.css';
import Button from '@mui/material/Button';

mapboxgl.accessToken = 'pk.eyJ1IjoiZWxpcG9sMDIiLCJhIjoiY2x4cDE0eGNqMDVobjJrcGtzODUxMXZtMyJ9.e0Iye6AtNZEH_B806foH5w';

const DroneMap = ({ latitude, longitude, angle, altitude }) => {
  const mapContainer = useRef(null);
  const map = useRef(null);
  const [bearing, setBearing] = useState(0);
  const modelRef = useRef(null);

  useEffect(() => {
    if (!latitude || !longitude) return; // wait until latitude and longitude are loaded

    const modelOrigin = [longitude, latitude];
    const modelAltitude = altitude;
    const modelRotate = [Math.PI / 2, 0, angle];

    const modelTransform = mapboxgl.MercatorCoordinate.fromLngLat(
      modelOrigin,
      modelAltitude
    );

    const translate = {
      x: modelTransform.x,
      y: modelTransform.y,
      z: modelTransform.z,
    };

    const scale = modelTransform.meterInMercatorCoordinateUnits();

    if (map.current) return; // initialize map only once

    map.current = new mapboxgl.Map({
      container: mapContainer.current,
      style: 'mapbox://styles/mapbox/satellite-streets-v11',
      center: modelOrigin,
      zoom: 18,
      bearing: bearing
    });
    map.current.on('rotate', () => {
      setBearing(map.current.getBearing());
    });

    // Add navigation controls
    map.current.addControl(new mapboxgl.NavigationControl());

    map.current.on('style.load', () => {
      map.current.addLayer({
        id: '3d-model',
        type: 'custom',
        renderingMode: '3d',
        onAdd: function (map, gl) {
          const renderer = new THREE.WebGLRenderer({
            canvas: map.getCanvas(),
            context: gl,
            antialias: true,
          });

          renderer.autoClear = false;

          const scene = new THREE.Scene();
          const camera = new THREE.Camera();

          const directionalLight = new THREE.DirectionalLight(0xffffff);
          directionalLight.position.set(0, -70, 100).normalize();
          scene.add(directionalLight);

          const directionalLight2 = new THREE.DirectionalLight(0xffffff);
          directionalLight2.position.set(0, 70, 100).normalize();
          scene.add(directionalLight2);

          const loader = new GLTFLoader();
          loader.load('/models/scene.gltf', (gltf) => {
            console.log('Model loaded successfully:', gltf);
            modelRef.current = gltf.scene;
            scene.add(gltf.scene);
          }, undefined, (error) => {
            console.error('Error loading model:', error);
          });

          this.renderer = renderer;
          this.scene = scene;
          this.camera = camera;
        },
        render: function (gl, matrix) {
          if (!modelRef.current) return;

          const rotationX = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 0, 0), modelRotate[0]);
          const rotationY = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0, 0, 1), modelRotate[1]);
          const rotationZ = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0, 1, 0), modelRotate[2]);

          const m = new THREE.Matrix4().fromArray(matrix);
          const l = new THREE.Matrix4()
            .makeTranslation(translate.x, translate.y, translate.z)
            .scale(new THREE.Vector3(scale, -scale, scale))
            .multiply(rotationX)
            .multiply(rotationY)
            .multiply(rotationZ);

          this.camera.projectionMatrix = m.multiply(l);
          this.renderer.resetState();
          this.renderer.render(this.scene, this.camera);
        },
      });

      // Add the 3D buildings layer
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

      /*
      // Add the terrain layer
      map.current.addSource('mapbox-dem', {
        'type': 'raster-dem',
        'url': 'mapbox://mapbox.mapbox-terrain-dem-v1',
        'tileSize': 512,
        'maxzoom': 14
      });
      map.current.setTerrain({ 'source': 'mapbox-dem' });
      */

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
    });
  }, [latitude, longitude, angle, altitude]);

  useEffect(() => {
    if (modelRef.current) {
      const modelTransform = mapboxgl.MercatorCoordinate.fromLngLat(
        [longitude, latitude],
        altitude
      );

      const translate = {
        x: modelTransform.x,
        y: modelTransform.y,
        z: modelTransform.z,
      };

      modelRef.current.position.set(translate.x, translate.y, translate.z);
    }
  }, [latitude, longitude, altitude]);

  if (!latitude || !longitude) {
    return <div>Loading...</div>; // or any other loading indicator
  }

  const handleCenter = () => {
    if (map.current) {
      map.current.setCenter([longitude, latitude]);
    }
  };

  return (
    <div>
      <div ref={mapContainer} className="map-container" style={{ height: '750px' }} />
      <Button onClick={handleCenter}>Center</Button>
    </div>
  );
};

export default DroneMap;