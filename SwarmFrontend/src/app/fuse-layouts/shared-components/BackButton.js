import React from 'react';
import Button from '@mui/material/Button';
import KeyboardBackspace from '@mui/icons-material/KeyboardBackspace';
import { useNavigate } from "react-router-dom";

const buttonStyle = {
	// marginLeft: '2%',
	backgroundColor: 'white',
    /*position: absoluteSize,*/
    left: 0.93,
    borderRadius: 5,
    padding: 5,
    color: 'black',
    top: 26,
    transition: 0.5,
    boxShadow: '0 0 5px -1px rgba(0,0,0,0.75)',
};

export default function BackButton ({ className = '' }) {

	let history = useNavigate();

    return (
        <Button className={className} style={buttonStyle} variant="contained" size="medium" onClick={() => history(-1)}>
            <KeyboardBackspace />
        </Button>
    );
}
