import React from 'react';
import { Box, Button } from '@mui/material';
import { useNavigate } from 'react-router-dom';

function Home() {
  const navigate = useNavigate();

  const handleNavigate = () => {
    navigate('/map');
  };

  return (
    <Box
      display="flex"
      justifyContent="center"
      alignItems="center"
      height="100vh"
    >
      <Button variant="contained" color="primary" onClick={handleNavigate}>
        Mapping Stream
      </Button>
    </Box>
  );
}

export default Home;
