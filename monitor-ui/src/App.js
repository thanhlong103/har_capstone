import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Home from './pages/Home';
import MapPage from './pages/MapPage';
import MoveToDestination from './pages/MoveToDestination';
import MakerspaceGuide from './pages/MakerspaceGuide';
import BackToHome from './pages/BackToHome';

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/map" element={<MapPage />} />
        <Route path="/move-to-destination" element={<MoveToDestination />} />
        <Route path="/makerspace-guide" element={<MakerspaceGuide />} />
        <Route path="/back-to-home" element={<BackToHome />} />
      </Routes>
    </Router>
  );
}

export default App;
