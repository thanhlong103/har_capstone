import React from 'react';
import { useNavigate } from 'react-router-dom';
import { Button, Grid, Typography } from '@mui/material';

function Home() {
  const navigate = useNavigate();

  return (
    <div style={{ 
      width: '100vw', 
      height: '100vh', 
      margin: 'auto', 
      display: 'flex', 
      flexDirection: 'column', // Stack title and content vertically
      justifyContent: 'center', 
      alignItems: 'center', 
      position: 'relative', 
      backgroundColor: '#00196E'  // Set background color
    }}>
      {/* Title */}
      <Typography 
        variant="h2" 
        style={{ 
          color: '#FFAD1D',
          fontWeight: 'bold',
          marginTop: '10px', 
          marginBottom: '0px', // Add space between title and buttons
          textAlign: 'center',
        }}
      >
        SOCIALLY AWARE SERVICE ROBOT
      </Typography>

      <div style={{ 
        width: '95vw', 
        height: '90vh', 
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
                fontSize: '3rem',
                borderRadius: '20px',
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
                fontSize: '3rem',
                borderRadius: '20px',
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
                fontSize: '3rem',
                borderRadius: '20px',
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
                fontSize: '3rem',
                borderRadius: '20px'
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
