import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { Slider } from '@mui/material';

const ThreeJSViewer = () => {
  const threeContainer = useRef(null);
  const modelRef = useRef(null); // Reference to the loaded model
  const [modelScale, setModelScale] = useState(1);

  useEffect(() => {
    // Initialize Three.js scene
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(500, 500);
    threeContainer.current.appendChild(renderer.domElement);

    // Add lighting
    const light = new THREE.AmbientLight(0x404040); // soft white light
    scene.add(light);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(0, 1, 0).normalize();
    scene.add(directionalLight);

    // Load the OBJ model
    const loader = new OBJLoader();
    loader.load('/Plane.obj', (obj) => {
      modelRef.current = obj;
      obj.scale.set(modelScale, modelScale, modelScale); // Set initial scale
      scene.add(obj);
      console.log('Model loaded in separate scene:', obj); // Debug: Log model load
    }, undefined, (error) => {
      console.error('Error loading model in separate scene:', error); // Debug: Log errors
    });

    camera.position.z = 5;

    const animate = () => {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      // Cleanup
      threeContainer.current.removeChild(renderer.domElement);
    };
  }, []); // Empty dependency array to run only once

  useEffect(() => {
    if (modelRef.current) {
      modelRef.current.scale.set(modelScale, modelScale, modelScale);
      console.log('Model scale updated:', modelScale);
    }
  }, [modelScale]); // Update model scale when modelScale state changes

  return (
    <div>
      <div ref={threeContainer} style={{ height: '500px', width: '500px' }} />
      <div style={{ marginTop: '10px' }}>
        <label>Model Scale</label>
        <Slider
          value={modelScale}
          min={0.1}
          max={10}
          step={0.1}
          onChange={(e, newValue) => setModelScale(newValue)}
          valueLabelDisplay="auto"
        />
      </div>
    </div>
  );
};

export default ThreeJSViewer;
