// SettingsContext.js
import React, { createContext, useState, useEffect } from 'react';

const SettingsContext = createContext();

const SettingsProvider = ({ children }) => {
  const [settings, setSettings] = useState(() => {
    // Load settings from local storage or use default settings
    const savedSettings = localStorage.getItem('settings');
    return savedSettings ? JSON.parse(savedSettings) : { theme: 'light', language: 'en' };
  });

  useEffect(() => {
    // Save settings to local storage whenever they change
    localStorage.setItem('settings', JSON.stringify(settings));
  }, [settings]);

  return (
    <SettingsContext.Provider value={{ settings, setSettings }}>
      {children}
    </SettingsContext.Provider>
  );
};

export { SettingsContext, SettingsProvider };
