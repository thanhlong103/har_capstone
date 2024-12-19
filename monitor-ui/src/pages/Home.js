import React from 'react';
import { useNavigate } from 'react-router-dom';
import { Button, Grid } from '@mui/material';

function Home() {
  const navigate = useNavigate();

  return (
    <div style={{ 
      width: '100vw', 
      height: '100vh', 
      margin: 'auto', 
      display: 'flex', 
      justifyContent: 'center', 
      alignItems: 'center', 
      position: 'relative', 
      backgroundColor: '#00196E'  // Set background color
    }}>
      <div style={{ 
        width: '90vw', 
        height: '90vh', 
        margin: 'auto', 
        display: 'flex', 
        justifyContent: 'center', 
        alignItems: 'center', 
        position: 'relative', 
      }}>
        <Grid container spacing={2} style={{ height: '100%' }}>
          {/* Laser Scan Stream Button */}
          <Grid item xs={6} style={{ height: '50%' }}>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: '#FFAD1D', 
                width: '100%', 
                height: '100%', 
                fontSize: '1.5rem',
                color: '#333' // Title color
              }}
              onClick={() => navigate('/map')}
            >
              Laser Scan Stream
            </Button>
          </Grid>

          {/* Move to Destination Button */}
          <Grid item xs={6} style={{ height: '50%' }}>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: '#FFAD1D', 
                width: '100%', 
                height: '100%', 
                fontSize: '1.5rem',
                color: '#333' // Title color
              }}
              onClick={() => navigate('/move-to-destination')}
            >
              Move to Destination
            </Button>
          </Grid>

          {/* Makerspace Guide Button */}
          <Grid item xs={6} style={{ height: '50%' }}>
            <Button
              variant="contained"
              style={{ 
                backgroundColor: '#FFAD1D', 
                width: '100%', 
                height: '100%', 
                fontSize: '1.5rem',
                color: '#333' // Title color
              }}
              onClick={() => navigate('/makerspace-guide')}
            >
              Makerspace Guide
            </Button>
          </Grid>

          {/* Back to Home Button */}
          <Grid item xs={6} style={{ height: '50%' }}>
            <Button
              variant="outlined"
              style={{ 
                borderColor: '#FFAD1D', 
                color: '#FFAD1D', 
                width: '100%', 
                height: '100%', 
                fontSize: '1.5rem'
              }}
              onClick={() => navigate('/back-to-home')}
            >
              Back to Home
            </Button>
          </Grid>
        </Grid>
      </div>
    </div>
  );
}

export default Home;
