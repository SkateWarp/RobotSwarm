import { styled } from '@mui/material/styles';

const Root = styled('div')(({ theme }) => ({
  '& > .logo-icon': {
    height: 40,
    objectFit: 'contain',
    objectPosition: 'left center',
    transition: theme.transitions.create(['width', 'height'], {
      duration: theme.transitions.duration.shortest,
      easing: theme.transitions.easing.easeInOut,
    }),
    width: 148,
  },
}));

function Logo() {
  return (
    <Root className="flex items-center">
      <img className="logo-icon" src="assets/images/logos/logo.png" alt="RobotSwarm" />
    </Root>
  );
}

export default Logo;
